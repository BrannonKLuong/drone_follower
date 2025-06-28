import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped # For 3D hand position
from visualization_msgs.msg import Marker # To visualize 3D hand position in RViz
from std_msgs.msg import ColorRGBA # For marker color
from sensor_msgs.msg import PointCloud2, PointField # For publishing PointCloud2
from sensor_msgs.msg import Imu # For publishing IMU data

import cv2
import mediapipe as mp

# Realsense specific imports
import pyrealsense2 as rs
import numpy as np
import datetime as dt # For FPS calculation
import struct # For packing PointCloud2 data

class HandGestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_recognition_node')

        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1, # Limiting to 1 hand for simplicity, can change later
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # Setup for OpenCV window display
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale = .5
        self.color = (0,150,255) # BGR format
        self.thickness = 1

        # ====== RealSense Camera Setup ======
        self.realsense_ctx = rs.context()
        self.connected_devices = [] # List of serial numbers for present cameras
        for i in range(len(self.realsense_ctx.devices)):
            detected_camera = self.realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
            self.get_logger().info(f"Found RealSense camera: {detected_camera}")
            self.connected_devices.append(detected_camera)
        
        if not self.connected_devices:
            self.get_logger().error("No RealSense cameras found! Shutting down.")
            rclpy.shutdown()
            return

        self.device_serial = self.connected_devices[0] # Use the first detected camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.background_removed_color = 153 # Grey for background in displayed image

        # Set stream resolution and FPS
        self.stream_res_x = 640
        self.stream_res_y = 480
        self.stream_fps = 30

        self.config.enable_device(self.device_serial)
        self.config.enable_stream(rs.stream.depth, self.stream_res_x, self.stream_res_y, rs.format.z16, self.stream_fps)
        self.config.enable_stream(rs.stream.color, self.stream_res_x, self.stream_res_y, rs.format.bgr8, self.stream_fps)
        self.config.enable_stream(rs.stream.accel) # Enable accelerometer
        self.config.enable_stream(rs.stream.gyro)  # Enable gyroscope
        
        # Start the pipeline and get the profile for depth scale and alignment
        try:
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info("RealSense pipeline started successfully.")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            self.get_logger().error("This often means the camera is already in use or there's a USB issue.")
            rclpy.shutdown()
            return

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Get depth Scale
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.get_logger().info(f"\tDepth Scale for Camera SN {self.device_serial} is: {self.depth_scale}")

        # Set clipping distance for visualization (optional)
        self.clipping_distance_in_meters = 2 # Example: ignore points beyond 2 meters for display
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale
        self.get_logger().info(f"\tClipping distance set to {self.clipping_distance_in_meters}m for visualization.")

        # Get stream intrinsics (needed for point cloud deprojection)
        self.color_intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().intrinsics
        
        # ROS2 Publishers
        self.command_publisher = self.create_publisher(String, '/hand_commands', 10)
        self.hand_3d_position_publisher = self.create_publisher(PointStamped, '/hand_3d_position', 10)
        self.hand_marker_publisher = self.create_publisher(Marker, '/hand_3d_marker', 10)
        
        # --- Publishers for PointCloud2 and IMU data ---
        # These topics will be consumed by other nodes, like obstacle_perception_node.py
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/camera/camera/depth/color/points', 10)
        self.imu_publisher = self.create_publisher(Imu, '/camera/camera/imu', 10)

        # To store last IMU data for uniting (if needed by your drone brain)
        self.last_accel_data = None
        self.last_gyro_data = None

        # Initialize decimation filter
        self.decimation_filter = rs.decimation_filter()
        self.decimation_filter.set_option(rs.option.filter_magnitude, 8) # Set decimation factor to 8

        # Timer to get frames and process
        self.timer = self.create_timer(1.0 / self.stream_fps, self.timer_callback)

        self.get_logger().info('Hand Gesture Recognition Node has started and is controlling RealSense.')

    def timer_callback(self):
        # Get and align frames
        frames = self.pipeline.wait_for_frames()
        
        # Process IMU frames first if available in the frameset
        # The frameset might contain multiple types of frames (depth, color, accel, gyro)
        for frame in frames:
            # Check if the frame is a motion frame (for IMU data)
            # and if it's either an accelerometer or gyroscope stream
            if frame.is_motion_frame(): # Correctly identifies IMU frames
                if frame.get_profile().stream_type() == rs.stream.accel or \
                   frame.get_profile().stream_type() == rs.stream.gyro:
                    self.process_imu_frame(frame)

        aligned_frames = self.align.process(frames)
        
        # Apply decimation filter to the aligned depth frame
        decimated_depth_frame = self.decimation_filter.process(aligned_frames.get_depth_frame())
        
        color_frame = aligned_frames.get_color_frame()
        
        if not decimated_depth_frame or not color_frame: # Use decimated_depth_frame here
            return

        # Convert images to NumPy arrays
        depth_image = np.asanyarray(decimated_depth_frame.get_data()) # Use decimated depth image
        color_image = np.asanyarray(color_frame.get_data())

        # Flip horizontally for MediaPipe and display
        color_image_flipped = cv2.flip(color_image, 1)
        depth_image_flipped = cv2.flip(depth_image, 1) # Use flipped decimated depth image

        # Convert BGR (from RealSense) to RGB (for MediaPipe)
        color_images_rgb = cv2.cvtColor(color_image_flipped, cv2.COLOR_BGR2RGB)

        # Process hands with MediaPipe
        results = self.hands.process(color_images_rgb)
        
        command = "NO_HAND" # Default command
        display_image = color_image_flipped.copy() # Make a writable copy for drawing

        # --- Process Hand Gestures and 3D Position ---
        if results.multi_hand_landmarks:
            # Get the width and height from the decimated_depth_frame's profile
            # Cast to rs.video_stream_profile to access width/height
            decimated_profile = decimated_depth_frame.profile.as_video_stream_profile()
            decimated_width = decimated_profile.width() # CALL THE METHOD
            decimated_height = decimated_profile.height() # CALL THE METHOD

            for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                self.mp_drawing.draw_landmarks(display_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Get the 3D position of the middle finger knuckle (or another point)
                middle_finger_knuckle_2d = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP] 
                
                # Convert normalized 2D coords to pixel coords using decimated dimensions
                x_pixel = int(middle_finger_knuckle_2d.x * decimated_width)
                y_pixel = int(middle_finger_knuckle_2d.y * decimated_height)

                # Ensure pixel coords are within image bounds
                x_pixel = np.clip(x_pixel, 0, decimated_width - 1)
                y_pixel = np.clip(y_pixel, 0, decimated_height - 1)

                mfk_distance_raw = depth_image_flipped[y_pixel, x_pixel] # Sample from decimated depth image
                mfk_distance_meters = mfk_distance_raw * self.depth_scale
                
                if mfk_distance_raw > 0 and mfk_distance_meters < self.clipping_distance_in_meters:
                    # Get intrinsics from the decimated frame's profile for accurate deprojection
                    decimated_intrinsics = decimated_profile.intrinsics
                    point_3d = rs.rs2_deproject_pixel_to_point(decimated_intrinsics, [x_pixel, y_pixel], mfk_distance_meters)
                    
                    hand_pos_msg = PointStamped()
                    hand_pos_msg.header.stamp = self.get_clock().now().to_msg()
                    hand_pos_msg.header.frame_id = "camera_color_optical_frame" # Frame of the color stream
                    hand_pos_msg.point.x = point_3d[0]
                    hand_pos_msg.point.y = point_3d[1]
                    hand_pos_msg.point.z = point_3d[2]
                    self.hand_3d_position_publisher.publish(hand_pos_msg)
                    self.publish_hand_marker(hand_pos_msg.point.x, hand_pos_msg.point.y, hand_pos_msg.point.z, hand_idx)
                    
                    # Existing 2D Gesture Logic
                    fingers_up = []
                    thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].x
                    thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP].x
                    # This check might need refinement based on hand orientation/handedness from MediaPipe results
                    if thumb_tip < thumb_ip: fingers_up.append(1)
                    else: fingers_up.append(0)
                    
                    finger_tip_ids = [self.mp_hands.HandLandmark.INDEX_FINGER_TIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP, 
                                      self.mp_hands.HandLandmark.RING_FINGER_TIP, self.mp_hands.HandLandmark.PINKY_TIP]
                    finger_pip_ids = [self.mp_hands.HandLandmark.INDEX_FINGER_PIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
                                      self.mp_hands.HandLandmark.RING_FINGER_PIP, self.mp_hands.HandLandmark.PINKY_PIP]

                    for i in range(4):
                        if hand_landmarks.landmark[finger_tip_ids[i]].y < hand_landmarks.landmark[finger_pip_ids[i]].y:
                            fingers_up.append(1)
                        else: fingers_up.append(0)
                    
                    num_fingers = sum(fingers_up)
                    
                    if num_fingers == 0: command = "HOVER"
                    elif num_fingers == 1: command = "GO_THROUGH_DOOR" # This is where 3D pointing would integrate
                    elif num_fingers == 5: command = "RESUME_STROBE_FOLLOW"
                    else: command = "UNKNOWN_GESTURE"
                    
                    cv2.putText(display_image, f"Distance: {mfk_distance_meters:.2f} m", (10, 30), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                else: command = "NO_VALID_HAND_DEPTH" 

        else: # No hands detected at all by MediaPipe
            command = "NO_HAND"
            cv2.putText(display_image, "No Hands Detected", (10, 30), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

        # Publish the detected command
        command_msg = String()
        command_msg.data = command
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f'Publishing command: {command}')
        
        # Display the image with annotations using cv2.imshow()
        cv2.imshow('Hand Gesture Recognition', display_image)
        cv2.waitKey(1)

        # --- Publish PointCloud2 ---
        # Pass the decimated_depth_frame to the point cloud publisher
        self.publish_pointcloud(decimated_depth_frame, color_frame)


    def process_imu_frame(self, frame):
        """Processes IMU (accelerometer and gyroscope) frames and publishes them."""
        # Using self.get_clock().now().to_msg() for ROS timestamp for consistency
        
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        # IMU data is published in camera_imu_optical_frame for consistency with RealSense ROS wrapper
        imu_msg.header.frame_id = "camera_imu_optical_frame"

        if frame.get_profile().stream_type() == rs.stream.accel:
            accel_data = frame.as_motion_frame().get_motion_data()
            self.last_accel_data = accel_data # Store for combining with gyro
            
            imu_msg.linear_acceleration.x = accel_data.x
            imu_msg.linear_acceleration.y = accel_data.y
            imu_msg.linear_acceleration.z = accel_data.z
            # Covariance values (placeholder, fine-tune for real application)
            imu_msg.linear_acceleration_covariance[0] = 0.01 
            imu_msg.linear_acceleration_covariance[4] = 0.01 
            imu_msg.linear_acceleration_covariance[8] = 0.01 
            
            # If we also have recent gyro data, combine and publish
            if self.last_gyro_data is not None:
                imu_msg.angular_velocity.x = self.last_gyro_data.x
                imu_msg.angular_velocity.y = self.last_gyro_data.y
                imu_msg.angular_velocity.z = self.last_gyro_data.z
                imu_msg.angular_velocity_covariance[0] = 0.001
                imu_msg.angular_velocity_covariance[4] = 0.001
                imu_msg.angular_velocity_covariance[8] = 0.001
                
                # Orientation is often derived from IMU fusion algorithms, set to unknown/identity for raw data
                imu_msg.orientation.w = 0.0 # Placeholder if not fused
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation_covariance[0] = -1.0 # Means no orientation provided
                
                self.imu_publisher.publish(imu_msg)
                self.last_gyro_data = None # Reset after publishing combined
                self.last_accel_data = None # Reset after publishing combined

        elif frame.get_profile().stream_type() == rs.stream.gyro:
            gyro_data = frame.as_motion_frame().get_motion_data()
            self.last_gyro_data = gyro_data # Store for combining with accel
            
            imu_msg.angular_velocity.x = gyro_data.x
            imu_msg.angular_velocity.y = gyro_data.y
            imu_msg.angular_velocity.z = gyro_data.z
            imu_msg.angular_velocity_covariance[0] = 0.001
            imu_msg.angular_velocity_covariance[4] = 0.001
            imu_msg.angular_velocity_covariance[8] = 0.001

            # If we also have recent accel data, combine and publish
            if self.last_accel_data is not None:
                imu_msg.linear_acceleration.x = self.last_accel_data.x
                imu_msg.linear_acceleration.y = self.last_accel_data.y
                imu_msg.linear_acceleration.z = self.last_accel_data.z
                imu_msg.linear_acceleration_covariance[0] = 0.01
                imu_msg.linear_acceleration_covariance[4] = 0.01
                imu_msg.linear_acceleration_covariance[8] = 0.01
                
                imu_msg.orientation.w = 0.0 
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation_covariance[0] = -1.0 
                
                self.imu_publisher.publish(imu_msg)
                self.last_accel_data = None # Reset after publishing combined
                self.last_gyro_data = None # Reset after publishing combined


    def publish_pointcloud(self, aligned_depth_frame, color_frame):
        """
        Generates and publishes a PointCloud2 message from aligned depth and color frames.
        """
        # Create a RealSense point cloud object
        pc = rs.pointcloud()
        
        # Map depth data to color data for textured point cloud
        points = pc.calculate(aligned_depth_frame)
        pc.map_to(color_frame)

        # Convert points to numpy array
        vtx = np.asanyarray(points.get_vertices()) # XYZ coordinates
        tex = np.asanyarray(points.get_texture_coordinates()) # UV texture coordinates

        # Get color image data for RGB values
        color_image_data = np.asanyarray(color_frame.get_data())

        # Prepare data for PointCloud2 message
        points_3d = []
        for i in range(len(vtx)):
            # Only include valid depth points (where Z is not 0)
            if vtx[i][2] != 0: 
                # Get RGB from color image based on texture coordinates
                # Clip coordinates to ensure they are within the image bounds
                x_tex = np.clip(int(tex[i][0] * color_frame.width), 0, color_frame.width - 1)
                y_tex = np.clip(int(tex[i][1] * color_frame.height), 0, color_frame.height - 1)
                
                # Get BGR color from the color image data
                color_pixel = color_image_data[y_tex, x_tex]
                b, g, r = color_pixel[0], color_pixel[1], color_pixel[2]
                
                # Pack BGR into a single 32-bit integer for the 'rgb' field
                # Corrected: Use bit shifting to combine B, G, R into a single uint32 (0x00RRGGBB)
                rgb = (r << 16) | (g << 8) | b 

                # Append XYZ and RGB to the list of points
                points_3d.append([vtx[i][0], vtx[i][1], vtx[i][2], rgb])

        if not points_3d: # If no valid points, don't publish
            return

        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        # Point cloud is in camera_color_optical_frame since it's aligned to color
        pc_msg.header.frame_id = "camera_color_optical_frame" 
        
        # Define fields for XYZ (Float32) and RGB (UInt32)
        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1), # Corrected offset for RGB field
        ]
        pc_msg.point_step = 16 # Corrected: Total size of a point in bytes (3*float32 + 1*uint32 = 16 bytes)
        pc_msg.row_step = pc_msg.point_step * len(points_3d) # (number of points) * (size of each point)
        pc_msg.height = 1 # Unordered point cloud
        pc_msg.width = len(points_3d) # Number of points
        pc_msg.is_dense = True # True if there are no invalid (NaN/Inf) points

        # Pack data into a bytearray
        data_bytes = bytearray()
        for p in points_3d:
            data_bytes.extend(struct.pack('<fffI', p[0], p[1], p[2], p[3])) # <fffI = little-endian float float float uint32
        pc_msg.data = bytes(data_bytes)

        self.point_cloud_publisher.publish(pc_msg)


    def publish_hand_marker(self, x, y, z, marker_id):
        """Publishes an RViz marker for the detected hand's 3D position."""
        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame" # Must match frame of the 3D point
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hand_positions"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0 # Identity quaternion (no rotation for a sphere)
        marker.scale.x = 0.1 # Small sphere radius
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8) # Green sphere with transparency
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg() # Disappear quickly
        self.hand_marker_publisher.publish(marker)

    def destroy_node(self):
        """Ensures the RealSense pipeline is stopped when the node is shut down."""
        if hasattr(self, 'pipeline'): # Check if pipeline was successfully started
            self.pipeline.stop()
            self.get_logger().info(f"RealSense pipeline stopped for SN: {self.device_serial}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    hand_gesture_node = HandGestureRecognitionNode()
    rclpy.spin(hand_gesture_node)
    # Ensure node is properly destroyed even if rclpy.spin exits early (e.g., Ctrl+C)
    if rclpy.ok(): 
        hand_gesture_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()