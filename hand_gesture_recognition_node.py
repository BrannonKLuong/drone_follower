import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3Stamped, Point
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
import math

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
        self.fontScale = .7
        self.color = (0,150,255) # BGR format
        self.thickness = 2

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

        self.stream_res_x = 1280
        self.stream_res_y = 720
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

        # --- NEW PUBLISHER for 3D Pointing Vector ---
        self.hand_pointing_publisher = self.create_publisher(Vector3Stamped, '/hand_pointing_vector', 10)
        self.pointing_arrow_marker_publisher = self.create_publisher(Marker, '/hand_pointing_arrow', 10)


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
        t_depth_opt = TransformStamped()
        t_depth_opt.header.stamp = now
        t_depth_opt.header.frame_id = 'camera_link'
        t_depth_opt.child_frame_id = 'camera_depth_optical_frame'
        t_depth_opt.transform.translation.x = 0.0
        t_depth_opt.transform.translation.y = 0.0
        t_depth_opt.transform.translation.z = 0.0
        t_depth_opt.transform.rotation.x = -0.5
        t_depth_opt.transform.rotation.y = 0.5
        t_depth_opt.transform.rotation.z = -0.5
        t_depth_opt.transform.rotation.w = 0.5
        self.static_tf_broadcaster.sendTransform(t_depth_opt)
        self.get_logger().info("Published static TF: camera_link -> camera_depth_optical_frame")


        # camera_depth_optical_frame -> camera_color_optical_frame
        t_color_opt = TransformStamped()
        t_color_opt.header.stamp = now
        t_color_opt.header.frame_id = 'camera_depth_optical_frame'
        t_color_opt.child_frame_id = 'camera_color_optical_frame'
        t_color_opt.transform.translation.x = -0.015
        t_color_opt.transform.translation.y = 0.0
        t_color_opt.transform.translation.z = 0.0
        t_color_opt.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t_color_opt)
        self.get_logger().info("Published static TF: camera_depth_optical_frame -> camera_color_optical_frame")


        # camera_depth_optical_frame -> camera_imu_optical_frame
        t_imu_opt = TransformStamped()
        t_imu_opt.header.stamp = now
        t_imu_opt.header.frame_id = 'camera_depth_optical_frame'
        t_imu_opt.child_frame_id = 'camera_imu_optical_frame'
        t_imu_opt.transform.translation.x = 0.0
        t_imu_opt.transform.translation.y = 0.0
        t_imu_opt.transform.translation.z = 0.0
        t_imu_opt.transform.rotation.w = 1.0
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

        # Initialize pointing vector to zero if no hand or not pointing
        pointing_vector_msg = Vector3Stamped()
        pointing_vector_msg.header.stamp = self.get_clock().now().to_msg()
        pointing_vector_msg.header.frame_id = "camera_color_optical_frame"
        pointing_vector_msg.vector.x = 0.0
        pointing_vector_msg.vector.y = 0.0
        pointing_vector_msg.vector.z = 0.0

        pointing_direction_text = "N/A" # Initialize text for display
        arrow_start_2d = None
        arrow_end_2d = None


        if results.multi_hand_landmarks:
            decimated_profile = decimated_depth_frame.profile.as_video_stream_profile()
            decimated_width = decimated_profile.width()
            decimated_height = decimated_profile.height()

            for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                self.mp_drawing.draw_landmarks(display_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Use WRIST as the primary hand position for display distance
                wrist_2d_norm = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST] 
                
                x_pixel_wrist = int(wrist_2d_norm.x * decimated_width)
                y_pixel_wrist = int(wrist_2d_norm.y * decimated_height)

                x_pixel_wrist = np.clip(x_pixel_wrist, 0, decimated_width - 1)
                y_pixel_wrist = np.clip(y_pixel_wrist, 0, decimated_height - 1)

                wrist_distance_raw = depth_image_flipped[y_pixel_wrist, x_pixel_wrist]
                wrist_distance_meters = wrist_distance_raw * self.depth_scale
                
                if wrist_distance_raw > 0 and wrist_distance_meters < self.clipping_distance_in_meters:
                    decimated_intrinsics = decimated_profile.intrinsics
                    wrist_3d = rs.rs2_deproject_pixel_to_point(decimated_intrinsics, [x_pixel_wrist, y_pixel_wrist], wrist_distance_meters)
                    
                    hand_pos_msg = PointStamped()
                    hand_pos_msg.header.stamp = self.get_clock().now().to_msg()
                    hand_pos_msg.header.frame_id = "camera_color_optical_frame"
                    hand_pos_msg.point.x = wrist_3d[0]
                    hand_pos_msg.point.y = wrist_3d[1]
                    hand_pos_msg.point.z = wrist_3d[2]
                    self.hand_3d_position_publisher.publish(hand_pos_msg)
                    self.publish_hand_marker(hand_pos_msg.point.x, hand_pos_msg.point.y, hand_pos_msg.point.z, hand_idx)
                    
                    fingers_up = []
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
                                      self.mp_hands.HandLandmark.RING_FINGER_TIP, self.mp_hands.HandLandmark.PINKY_PIP] # Corrected RING_FINGER_PIP

                    for i in range(4):
                        if hand_landmarks.landmark[finger_tip_ids[i]].y < hand_landmarks.landmark[finger_pip_ids[i]].y:
                            fingers_up.append(1)
                        else: fingers_up.append(0)
                    
                    num_fingers = sum(fingers_up)
                    
                    if num_fingers == 0: # Fist
                        command = "LAND"
                    elif num_fingers == 2: # --- CHANGED: MOVE_FORWARD now requires 2 fingers ---
                        command = "MOVE_FORWARD"
                        # --- Calculate and Publish 3D Pointing Vector using WRIST and INDEX_FINGER_MCP ---
                        wrist_2d_norm = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                        index_mcp_2d_norm = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]

                        # Convert normalized 2D coords to pixel coords
                        wrist_x_pixel_norm = wrist_2d_norm.x
                        wrist_y_pixel_norm = wrist_2d_norm.y
                        index_mcp_x_pixel_norm = index_mcp_2d_norm.x
                        index_mcp_y_pixel_norm = index_mcp_2d_norm.y

                        # Get depths for wrist and index_mcp
                        # Need to get pixel coordinates for depth lookup
                        wrist_depth_x_pixel = int(wrist_x_pixel_norm * decimated_width)
                        wrist_depth_y_pixel = int(wrist_y_pixel_norm * decimated_height)
                        index_mcp_depth_x_pixel = int(index_mcp_x_pixel_norm * decimated_width)
                        index_mcp_depth_y_pixel = int(index_mcp_y_pixel_norm * decimated_height)

                        # Clip to bounds for depth lookup
                        wrist_depth_x_pixel = np.clip(wrist_depth_x_pixel, 0, decimated_width - 1)
                        wrist_depth_y_pixel = np.clip(wrist_depth_y_pixel, 0, decimated_height - 1)
                        index_mcp_depth_x_pixel = np.clip(index_mcp_depth_x_pixel, 0, decimated_width - 1)
                        index_mcp_depth_y_pixel = np.clip(index_mcp_depth_y_pixel, 0, decimated_height - 1)


                        wrist_distance_raw_for_vec = depth_image_flipped[wrist_depth_y_pixel, wrist_depth_x_pixel]
                        index_mcp_distance_raw_for_vec = depth_image_flipped[index_mcp_depth_y_pixel, index_mcp_depth_x_pixel]

                        if wrist_distance_raw_for_vec > 0 and index_mcp_distance_raw_for_vec > 0:
                            wrist_distance_meters_for_vec = wrist_distance_raw_for_vec * self.depth_scale
                            index_mcp_distance_meters_for_vec = index_mcp_distance_raw_for_vec * self.depth_scale

                            # Deproject to 3D points
                            wrist_3d_vec = rs.rs2_deproject_pixel_to_point(decimated_intrinsics, [wrist_depth_x_pixel, wrist_depth_y_pixel], wrist_distance_meters_for_vec)
                            index_mcp_3d_vec = rs.rs2_deproject_pixel_to_point(decimated_intrinsics, [index_mcp_depth_x_pixel, index_mcp_depth_y_pixel], index_mcp_distance_meters_for_vec)

                            # Calculate vector from WRIST to INDEX_FINGER_MCP (representing forearm direction)
                            vec_x = index_mcp_3d_vec[0] - wrist_3d_vec[0]
                            vec_y = index_mcp_3d_vec[1] - wrist_3d_vec[1]
                            vec_z = index_mcp_3d_vec[2] - wrist_3d_vec[2]

                            # Normalize the vector
                            magnitude = math.sqrt(vec_x**2 + vec_y**2 + vec_z**2)
                            if magnitude > 1e-6:
                                pointing_vector_msg.vector.x = vec_x / magnitude
                                pointing_vector_msg.vector.y = vec_y / magnitude
                                pointing_vector_msg.vector.z = vec_z / magnitude
                            
                            self.hand_pointing_publisher.publish(pointing_vector_msg)
                            # Use wrist_3d as the origin for the pointing arrow marker
                            self.publish_pointing_arrow_marker(Point(x=wrist_3d_vec[0], y=wrist_3d_vec[1], z=wrist_3d_vec[2]), pointing_vector_msg.vector, hand_idx)

                            # --- NEW: Visualize 2D projection of forearm vector on cv2.imshow ---
                            # Project the 3D forearm vector's endpoint back to 2D for visualization
                            # Start point for 2D arrow is wrist_2d_norm (normalized coordinates)
                            arrow_start_2d = (int(wrist_2d_norm.x * color_image_flipped.shape[1]), int(wrist_2d_norm.y * color_image_flipped.shape[0]))
                            
                            # Calculate a point along the normalized vector for the arrow end
                            # Project a point 0.1m along the vector from the wrist_3d_vec
                            arrow_end_3d_for_proj = [
                                wrist_3d_vec[0] + pointing_vector_msg.vector.x * 0.1,
                                wrist_3d_vec[1] + pointing_vector_msg.vector.y * 0.1,
                                wrist_3d_vec[2] + pointing_vector_msg.vector.z * 0.1
                            ]
                            
                            # Project this 3D point back to 2D pixel coordinates for the color image
                            arrow_end_2d_pixel = rs.rs2_project_point_to_pixel(self.color_intrinsics, arrow_end_3d_for_proj)
                            arrow_end_2d = (int(arrow_end_2d_pixel[0]), int(arrow_end_2d_pixel[1]))

                            # --- Calculate and Display Direction (Left/Right/Up/Down) ---
                            # In camera_color_optical_frame: X is right, Y is down, Z is forward
                            vx, vy, vz = pointing_vector_msg.vector.x, pointing_vector_msg.vector.y, pointing_vector_msg.vector.z

                            # Horizontal angle (yaw) in XZ plane relative to Z-axis (forward)
                            # Use a small epsilon to avoid division by zero if vz is near zero
                            horizontal_angle_rad = math.atan2(vx, vz if abs(vz) > 1e-6 else (1e-6 if vx >= 0 else -1e-6))
                            horizontal_angle_deg = math.degrees(horizontal_angle_rad)

                            # Vertical angle (pitch)
                            horizontal_projection_magnitude = math.sqrt(vx**2 + vz**2)
                            vertical_angle_rad = 0.0
                            if horizontal_projection_magnitude > 1e-6:
                                vertical_angle_rad = math.atan2(-vy, horizontal_projection_magnitude) # -vy for intuitive "up"
                            vertical_angle_deg = math.degrees(vertical_angle_rad)

                            # Determine descriptive text
                            horiz_text = ""
                            if abs(horizontal_angle_deg) < 25: # Within +/- 25 deg of straight forward/backward
                                if vz > 0: horiz_text = "Forward"
                                else: horiz_text = "Backward"
                            elif horizontal_angle_deg > 0: horiz_text = "Right"
                            else: horiz_text = "Left"
                            
                            vert_text = ""
                            if abs(vertical_angle_deg) < 10: vert_text = "Level"
                            elif vertical_angle_deg > 0: vert_text = "Up"
                            else: vert_text = "Down"

                            pointing_direction_text = f"H: {horiz_text} ({abs(horizontal_angle_deg):.1f} deg), V: {vert_text} ({abs(vertical_angle_deg):.1f} deg)"
                            
                        else:
                            self.get_logger().warn("Could not get valid depth for wrist or index_mcp for pointing vector.")
                            pointing_direction_text = "Depth Error"

                    elif num_fingers == 5: # Open palm (high five)
                        command = "HOVER"
                    else:
                        command = "UNKNOWN_GESTURE"
                    
                    cv2.putText(display_image, f"Distance: {wrist_distance_meters:.2f} m", (10, 30), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                    cv2.putText(display_image, f"Fingers Up: {num_fingers}", (10, 60), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                    cv2.putText(display_image, f"Pointing: {pointing_direction_text}", (10, 90), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                else:
                    command = "NO_VALID_HAND_DEPTH" 
                    cv2.putText(display_image, "No Valid Hand Depth", (10, 30), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

        else:
            command = "NO_HAND"
            cv2.putText(display_image, "No Hands Detected", (10, 30), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
            cv2.putText(display_image, f"Pointing: {pointing_direction_text}", (10, 90), self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

        # Always publish the pointing vector message, even if it's zero (no hand/not pointing)
        self.hand_pointing_publisher.publish(pointing_vector_msg)

        # --- NEW: Draw 2D pointing arrow on display_image ---
        if arrow_start_2d and arrow_end_2d:
            cv2.arrowedLine(display_image, arrow_start_2d, arrow_end_2d, (0, 255, 0), 3) # Green arrow

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

    def publish_pointing_arrow_marker(self, origin_point, direction_vector, marker_id):
        """Publishes an RViz arrow marker for the 3D pointing vector."""
        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame" # Same frame as the pointing vector
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hand_pointing_arrows"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Start of the arrow (e.g., middle finger knuckle or base of index finger)
        marker.points.append(origin_point)
        
        # End point (origin + scaled direction vector)
        arrow_length = 0.3 # meters
        end_point = Point()
        end_point.x = origin_point.x + direction_vector.x * arrow_length
        end_point.y = origin_point.y + direction_vector.y * arrow_length
        end_point.z = origin_point.z + direction_vector.z * arrow_length
        marker.points.append(end_point)

        marker.scale.x = 0.02 # Shaft diameter
        marker.scale.y = 0.04 # Head diameter
        marker.scale.z = 0.0 # Not used for ARROW type with points

        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8) # Orange arrow
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        self.pointing_arrow_marker_publisher.publish(marker)


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
