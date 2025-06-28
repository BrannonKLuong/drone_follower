import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Imu

import cv2
import mediapipe as mp

# Realsense specific imports
import pyrealsense2 as rs
import numpy as np
import datetime as dt
import struct

# TF2_ROS imports for static transforms
from tf2_ros import StaticTransformBroadcaster

class HandGestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_recognition_node')

        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # Setup for OpenCV window display
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale = .7 # Slightly increased font scale for better visibility
        self.color = (0,150,255) # BGR format
        self.thickness = 2 # Slightly increased thickness

        # ====== RealSense Camera Setup ======
        self.realsense_ctx = rs.context()
        self.connected_devices = []
        for i in range(len(self.realsense_ctx.devices)):
            detected_camera = self.realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
            self.get_logger().info(f"Found RealSense camera: {detected_camera}")
            self.connected_devices.append(detected_camera)
        
        if not self.connected_devices:
            self.get_logger().error("No RealSense cameras found! Shutting down.")
            rclpy.shutdown()
            return

        self.device_serial = self.connected_devices[0]
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.background_removed_color = 153

        # --- INCREASED RESOLUTION FOR CV2.IMSHOW ---
        self.stream_res_x = 1280 # Changed from 640
        self.stream_res_y = 720  # Changed from 480
        self.stream_fps = 30

        self.config.enable_device(self.device_serial)
        self.config.enable_stream(rs.stream.depth, self.stream_res_x, self.stream_res_y, rs.format.z16, self.stream_fps)
        self.config.enable_stream(rs.stream.color, self.stream_res_x, self.stream_res_y, rs.format.bgr8, self.stream_fps)
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)
        
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

        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.get_logger().info(f"\tDepth Scale for Camera SN {self.device_serial} is: {self.depth_scale}")

        self.clipping_distance_in_meters = 2
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale
        self.get_logger().info(f"\tClipping distance set to {self.clipping_distance_in_meters}m for visualization.")

        self.color_intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().intrinsics
        
        # ROS2 Publishers
        self.command_publisher = self.create_publisher(String, '/hand_commands', 10)
        self.hand_3d_position_publisher = self.create_publisher(PointStamped, '/hand_3d_position', 10)
        self.hand_marker_publisher = self.create_publisher(Marker, '/hand_3d_marker', 10)
        
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/camera/camera/depth/color/points', 10)
        self.imu_publisher = self.create_publisher(Imu, '/camera/camera/imu', 10)

        self.last_accel_data = None
        self.last_gyro_data = None

        self.decimation_filter = rs.decimation_filter()
        self.decimation_filter.set_option(rs.option.filter_magnitude, 8)

        # Initialize StaticTransformBroadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static camera internal TFs once
        self.publish_static_camera_tfs()

        # Timer to get frames and process
        self.timer = self.create_timer(1.0 / self.stream_fps, self.timer_callback)

        self.get_logger().info('Hand Gesture Recognition Node has started and is controlling RealSense.')

    def publish_static_camera_tfs(self):
        """
        Publishes the static transforms for the RealSense camera's internal frames.
        These are typically fixed relative to each other.
        """
        now = self.get_clock().now().to_msg()

        # camera_link -> camera_depth_optical_frame
        # This transform aligns the camera_link (base frame) to the depth optical frame (Z-forward, X-right, Y-down).
        # These are standard rotations for RealSense cameras to get to the optical frame.
        t_depth_opt = TransformStamped()
        t_depth_opt.header.stamp = now
        t_depth_opt.header.frame_id = 'camera_link'
        t_depth_opt.child_frame_id = 'camera_depth_optical_frame'
        t_depth_opt.transform.translation.x = 0.0
        t_depth_opt.transform.translation.y = 0.0
        t_depth_opt.transform.translation.z = 0.0
        # Quaternion for a rotation of -pi/2 around X and -pi/2 around Y
        # This transforms a standard camera_link (X-forward, Y-left, Z-up) to an optical frame (X-right, Y-down, Z-forward)
        t_depth_opt.transform.rotation.x = -0.5
        t_depth_opt.transform.rotation.y = 0.5
        t_depth_opt.transform.rotation.z = -0.5
        t_depth_opt.transform.rotation.w = 0.5
        self.static_tf_broadcaster.sendTransform(t_depth_opt)
        self.get_logger().info("Published static TF: camera_link -> camera_depth_optical_frame")


        # camera_depth_optical_frame -> camera_color_optical_frame
        # Small offset for the color sensor relative to the depth sensor.
        # These values are approximate and might need fine-tuning based on RealSense D435i specs.
        t_color_opt = TransformStamped()
        t_color_opt.header.stamp = now
        t_color_opt.header.frame_id = 'camera_depth_optical_frame'
        t_color_opt.child_frame_id = 'camera_color_optical_frame'
        t_color_opt.transform.translation.x = -0.015 # Example: 1.5 cm left
        t_color_opt.transform.translation.y = 0.0
        t_color_opt.transform.translation.z = 0.0
        t_color_opt.transform.rotation.w = 1.0 # Identity rotation
        self.static_tf_broadcaster.sendTransform(t_color_opt)
        self.get_logger().info("Published static TF: camera_depth_optical_frame -> camera_color_optical_frame")


        # camera_depth_optical_frame -> camera_imu_optical_frame
        # IMU frame relative to depth frame. Often very close to identity.
        t_imu_opt = TransformStamped()
        t_imu_opt.header.stamp = now
        t_imu_opt.header.frame_id = 'camera_depth_optical_frame'
        t_imu_opt.child_frame_id = 'camera_imu_optical_frame'
        t_imu_opt.transform.translation.x = 0.0
        t_imu_opt.transform.translation.y = 0.0
        t_imu_opt.transform.translation.z = 0.0
        t_imu_opt.transform.rotation.w = 1.0 # Identity rotation
        self.static_tf_broadcaster.sendTransform(t_imu_opt)
        self.get_logger().info("Published static TF: camera_depth_optical_frame -> camera_imu_optical_frame")


    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        
        for frame in frames:
            if frame.is_motion_frame():
                if frame.get_profile().stream_type() == rs.stream.accel or \
                   frame.get_profile().stream_type() == rs.stream.gyro:
                    self.process_imu_frame(frame)

        aligned_frames = self.align.process(frames)
        
        decimated_depth_frame = self.decimation_filter.process(aligned_frames.get_depth_frame())
        
        color_frame = aligned_frames.get_color_frame()
        
        if not decimated_depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(decimated_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        color_image_flipped = cv2.flip(color_image, 1)
        depth_image_flipped = cv2.flip(depth_image, 1)

        color_images_rgb = cv2.cvtColor(color_image_flipped, cv2.COLOR_BGR2RGB)

        results = self.hands.process(color_images_rgb)
        
        command = "NO_HAND"
        display_image = color_image_flipped.copy()

        if results.multi_hand_landmarks:
            decimated_profile = decimated_depth_frame.profile.as_video_stream_profile()
            decimated_width = decimated_profile.width()
            decimated_height = decimated_profile.height()

            for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                self.mp_drawing.draw_landmarks(display_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                middle_finger_knuckle_2d = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP] 
                
                x_pixel = int(middle_finger_knuckle_2d.x * decimated_width)
                y_pixel = int(middle_finger_knuckle_2d.y * decimated_height)

                x_pixel = np.clip(x_pixel, 0, decimated_width - 1)
                y_pixel = np.clip(y_pixel, 0, decimated_height - 1)

                mfk_distance_raw = depth_image_flipped[y_pixel, x_pixel]
                mfk_distance_meters = mfk_distance_raw * self.depth_scale
                
                if mfk_distance_raw > 0 and mfk_distance_meters < self.clipping_distance_in_meters:
                    decimated_intrinsics = decimated_profile.intrinsics
                    point_3d = rs.rs2_deproject_pixel_to_point(decimated_intrinsics, [x_pixel, y_pixel], mfk_distance_meters)
                    
                    hand_pos_msg = PointStamped()
                    hand_pos_msg.header.stamp = self.get_clock().now().to_msg()
                    hand_pos_msg.header.frame_id = "camera_color_optical_frame"
                    hand_pos_msg.point.x = point_3d[0]
                    hand_pos_msg.point.y = point_3d[1]
                    hand_pos_msg.point.z = point_3d[2]
                    self.hand_3d_position_publisher.publish(hand_pos_msg)
                    self.publish_hand_marker(hand_pos_msg.point.x, hand_pos_msg.point.y, hand_pos_msg.point.z, hand_idx)
                    
                    fingers_up = []
                    # Check thumb
                    # For right hand, thumb_tip.x < thumb_ip.x means thumb is open
                    # For left hand, thumb_tip.x > thumb_ip.x means thumb is open
                    # MediaPipe's multi_handedness[i].classification[0].label can be 'Right' or 'Left'
                    hand_label = results.multi_handedness[hand_idx].classification[0].label
                    if hand_label == 'Right':
                        if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].x < hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP].x:
                            fingers_up.append(1)
                        else:
                            fingers_up.append(0)
                    else: # Left hand
                        if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].x > hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP].x:
                            fingers_up.append(1)
                        else:
                            fingers_up.append(0)

                    finger_tip_ids = [self.mp_hands.HandLandmark.INDEX_FINGER_TIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP, 
                                      self.mp_hands.HandLandmark.RING_FINGER_TIP, self.mp_hands.HandLandmark.PINKY_TIP]
                    finger_pip_ids = [self.mp_hands.HandLandmark.INDEX_FINGER_PIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
                                      self.mp_hands.HandLandmark.RING_FINGER_PIP, self.mp_hands.HandLandmark.PINKY_PIP]

                    for i in range(4): # Check other 4 fingers
                        if hand_landmarks.landmark[finger_tip_ids[i]].y < hand_landmarks.landmark[finger_pip_ids[i]].y:
                            fingers_up.append(1)
                        else: fingers_up.append(0)
                    
                    num_fingers = sum(fingers_up)
                    
                    # --- UPDATED GESTURE MAPPING ---
                    if num_fingers == 0: # Fist
                        command = "LAND"
                    elif num_fingers == 1: # Index finger pointing
                        command = "MOVE_FORWARD"
                    elif num_fingers == 5: # Open palm (high five)
                        command = "HOVER"
                    else:
                        command = "UNKNOWN_GESTURE"
                    
                    cv2.putText(display_image, f"Distance: {mfk_distance_meters:.2f} m", (10, 30), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                    cv2.putText(display_image, f"Fingers Up: {num_fingers}", (10, 60), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                else: command = "NO_VALID_HAND_DEPTH" 

        else:
            command = "NO_HAND"
            cv2.putText(display_image, "No Hands Detected", (10, 30), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

        command_msg = String()
        command_msg.data = command
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f'Publishing command: {command}')
        
        cv2.imshow('Hand Gesture Recognition', display_image)
        cv2.waitKey(1)

        self.publish_pointcloud(decimated_depth_frame, color_frame)


    def process_imu_frame(self, frame):
        """Processes IMU (accelerometer and gyroscope) frames and publishes them."""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "camera_imu_optical_frame"

        if frame.get_profile().stream_type() == rs.stream.accel:
            accel_data = frame.as_motion_frame().get_motion_data()
            self.last_accel_data = accel_data
            
            imu_msg.linear_acceleration.x = accel_data.x
            imu_msg.linear_acceleration.y = accel_data.y
            imu_msg.linear_acceleration.z = accel_data.z
            imu_msg.linear_acceleration_covariance[0] = 0.01 
            imu_msg.linear_acceleration_covariance[4] = 0.01 
            imu_msg.linear_acceleration_covariance[8] = 0.01 
            
            if self.last_gyro_data is not None:
                imu_msg.angular_velocity.x = self.last_gyro_data.x
                imu_msg.angular_velocity.y = self.last_gyro_data.y
                imu_msg.angular_velocity.z = self.last_gyro_data.z
                imu_msg.angular_velocity_covariance[0] = 0.001
                imu_msg.angular_velocity_covariance[4] = 0.001
                imu_msg.angular_velocity_covariance[8] = 0.001
                
                imu_msg.orientation.w = 0.0
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation_covariance[0] = -1.0
                
                self.imu_publisher.publish(imu_msg)
                self.last_gyro_data = None
                self.last_accel_data = None

        elif frame.get_profile().stream_type() == rs.stream.gyro:
            gyro_data = frame.as_motion_frame().get_motion_data()
            self.last_gyro_data = gyro_data
            
            imu_msg.angular_velocity.x = gyro_data.x
            imu_msg.angular_velocity.y = gyro_data.y
            imu_msg.angular_velocity.z = gyro_data.z
            imu_msg.angular_velocity_covariance[0] = 0.001
            imu_msg.angular_velocity_covariance[4] = 0.001
            imu_msg.angular_velocity_covariance[8] = 0.001

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
                self.last_accel_data = None
                self.last_gyro_data = None


    def publish_pointcloud(self, aligned_depth_frame, color_frame):
        """
        Generates and publishes a PointCloud2 message from aligned depth and color frames.
        """
        pc = rs.pointcloud()
        
        points = pc.calculate(aligned_depth_frame)
        pc.map_to(color_frame)

        vtx = np.asanyarray(points.get_vertices())
        tex = np.asanyarray(points.get_texture_coordinates())

        color_image_data = np.asanyarray(color_frame.get_data())

        points_3d = []
        for i in range(len(vtx)):
            if vtx[i][2] != 0: 
                x_tex = np.clip(int(tex[i][0] * color_frame.width), 0, color_frame.width - 1)
                y_tex = np.clip(int(tex[i][1] * color_frame.height), 0, color_frame.height - 1)
                
                color_pixel = color_image_data[y_tex, x_tex]
                b, g, r = color_pixel[0], color_pixel[1], color_pixel[2]
                
                rgb = (r << 16) | (g << 8) | b 

                points_3d.append([vtx[i][0], vtx[i][1], vtx[i][2], rgb])

        if not points_3d:
            return

        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = "camera_color_optical_frame" 
        
        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        pc_msg.point_step = 16
        pc_msg.row_step = pc_msg.point_step * len(points_3d)
        pc_msg.height = 1
        pc_msg.width = len(points_3d)
        pc_msg.is_dense = True

        data_bytes = bytearray()
        for p in points_3d:
            data_bytes.extend(struct.pack('<fffI', p[0], p[1], p[2], p[3]))
        pc_msg.data = bytes(data_bytes)

        self.point_cloud_publisher.publish(pc_msg)


    def publish_hand_marker(self, x, y, z, marker_id):
        """Publishes an RViz marker for the detected hand's 3D position."""
        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hand_positions"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        self.hand_marker_publisher.publish(marker)

    def destroy_node(self):
        """Ensures the RealSense pipeline is stopped when the node is shut down."""
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()
            self.get_logger().info(f"RealSense pipeline stopped for SN: {self.device_serial}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    hand_gesture_node = HandGestureRecognitionNode()
    rclpy.spin(hand_gesture_node)
    if rclpy.ok(): 
        hand_gesture_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
