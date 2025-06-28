#!/bin/bash

# This script launches the entire drone simulation and control system,
# with the RealSense D435i camera managed directly by the Hand Gesture Recognition node.

# Define the project directory for easier path management.
PROJECT_DIR=~/drone_project

# --- Start ROS2 Nodes in Separate Terminals ---

# Terminal 1: Mock PX4 Interface (The Simulated Drone Flight Controller)
# This node mimics a real PX4 flight controller, publishing drone position and status.
gnome-terminal --tab --title="Mock PX4" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./mock_px4.py; exec bash"

# TERMINAL 2: REAL SENSE D435I CAMERA NODE (ROS2 Wrapper) - REMOVED!
# The hand_gesture_recognition_node.py now directly controls the RealSense camera,
# and publishes PointCloud2, IMU, and image data. This avoids camera access conflicts.
# The following line is commented out as it is no longer needed:
# gnome-terminal --tab --title="RealSense Camera" -- bash -c "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true unite_imu_method:=2 pointcloud.enable:=true align_depth.enable:=true depth_module.profile:=640x480x30 decimation_filter.enable:=true decimation_filter.decimation_factor:=8; exec bash"

# Terminal 3: RViz2 (3D Visualization Tool)
# Visualizes the drone's environment, sensor data, and simulated movements.
gnome-terminal --tab --title="RViz2" -- bash -c "source /opt/ros/humble/setup.bash && rviz2; exec bash"

# Terminal 4: Obstacle Perception Node (The Drone's Eyes - processes PointCloud2)
# IMPORTANT: This node must now subscribe to the PointCloud2 topic published by
# hand_gesture_recognition_node.py (which is /camera/camera/depth/color/points).
# Ensure obstacle_perception_node.py is updated to subscribe to this topic if it isn't already.
gnome-terminal --tab --title="Obstacle Perception" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./obstacle_perception_node.py; exec bash"

# Terminal 5: Drone Brain (The Drone's High-Level Logic)
# Implements the potential field algorithm and drone state machine.
gnome-terminal --tab --title="Drone Brain" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./current_fly_script.py; exec bash"

# Terminal 6: Odometry to TF Publisher
# Publishes the 'odom' to 'base_link' transform for RViz visualization of drone movement.
gnome-terminal --tab --title="Odometry TF" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./px4_odometry_to_tf_publisher.py; exec bash"

# Terminal 7: Static TF from base_link to camera_color_optical_frame
# This connects the drone's 'base_link' frame to the camera's 'camera_color_optical_frame' for RViz.
# This transform is crucial for visualizing the hand position and point cloud correctly relative to the drone.
# Translation (x y z): 0.1m forward, 0.0m side, 0.05m up relative to base_link.
# Rotation (roll_rad pitch_rad yaw_rad): -1.5707 (-90 deg roll), -0.17 (-10 deg pitch), 0.0 (0 deg yaw).
# This configuration aligns the camera's Z-axis (forward) with the drone's X-axis (forward),
# while adding a 10-degree downward pitch, and accounts for the 90-degree yaw correction.
gnome-terminal --tab --title="Camera Static TF" -- bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0.1 0.0 0.05 -1.5707 -0.17 0.0 base_link camera_color_optical_frame; exec bash"

# Terminal 8: Strobe Light Publisher (The Moving Target)
# Simulates the target the drone will follow.
gnome-terminal --tab --title="Strobe Light Publisher" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./strobe_light.py; exec bash"

# Terminal 9: Interactive Hand Command Publisher (Keyboard Interface for Manual Control)
# Allows a human operator to issue high-level commands via keyboard.
gnome-terminal --tab --title="Interactive Hand Command" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./hand_command_publisher.py; exec bash"

# Terminal 10: Hand Gesture Recognition Node (The Operator Interface - now controls RealSense directly)
# This node directly manages the RealSense camera and publishes all camera-related ROS topics.
gnome-terminal --tab --title="Hand Gesture Recognition" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./hand_gesture_recognition_node.py; exec bash"

echo "All drone project components are launching in separate terminals. Please check each terminal for output."
echo "Ensure your RealSense D435i is connected to the VM via a USB 3.x port."
echo "Remember to update obstacle_perception_node.py to subscribe to /camera/camera/depth/color/points if it's not already."