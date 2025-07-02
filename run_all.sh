#!/bin/bash

# This script launches the entire drone simulation and control system,
# specifically configured to test obstacle avoidance with a corrected TF tree.

PROJECT_DIR=~/drone_project

# --- Start ROS2 Nodes in Separate Terminals ---

# Terminal 1: Mock PX4 Interface
gnome-terminal --tab --title="Mock PX4" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./mock_px4.py; exec bash"

# Terminal 2: RViz2 (3D Visualization Tool)
gnome-terminal --tab --title="RViz2" -- bash -c "source /opt/ros/humble/setup.bash && rviz2; exec bash"

# Terminal 3: Obstacle Perception Node
gnome-terminal --tab --title="Obstacle Perception" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./obstacle_perception_node.py; exec bash"

# Terminal 4: Drone Brain (The Drone's High-Level Logic)
gnome-terminal --tab --title="Drone Brain" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./current_fly_script.py; exec bash"

# Terminal 5: Odometry to TF Publisher
gnome-terminal --tab --title="Odometry TF" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./px4_odometry_to_tf_publisher.py; exec bash"

# --- TF CHAIN PUBLISHERS ---
# These are now all handled here to ensure a complete and stable TF tree from the start.

# Terminal 6: Static TF from base_link to camera_link (Drone body to camera mount)
gnome-terminal --tab --title="Camera Mount TF" -- bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0.1 0.0 0.0 0 0 0 base_link camera_link; exec bash"

# --- CORRECTED: Using correct quaternion for camera optical frame transform ---
# Terminal 7: Static TF from camera_link to camera_depth_optical_frame
gnome-terminal --tab --title="Camera Depth TF" -- bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 camera_link camera_depth_optical_frame; exec bash"

# Terminal 8: Static TF from camera_depth_optical_frame to camera_color_optical_frame
gnome-terminal --tab --title="Camera Color TF" -- bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0.015 0 0 0 0 0 camera_depth_optical_frame camera_color_optical_frame; exec bash"


# --- SENSOR AND VISUALIZATION NODES ---

# Terminal 9: Simulated Depth Sensor (Creates the obstacle course)
gnome-terminal --tab --title="Simulated Obstacles" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./simulated_depth_sensor.py; exec bash"

# Terminal 10: Vertical Grid Publisher for RViz (Optional but helpful)
# gnome-terminal --tab --title="Vertical Grid" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./vertical_grid_publisher.py; exec bash"

# --- DISABLED NODES FOR THIS TEST ---
# The hand gesture node is disabled to ensure the point cloud data comes from the simulator only.
# gnome-terminal --tab --title="Hand Gesture Recognition" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./hand_gesture_recognition_node.py; exec bash"

echo "Collision Avoidance Test Environment launching..."
echo "Simulated Obstacle node is ENABLED."
echo "Hand Gesture node is DISABLED."
