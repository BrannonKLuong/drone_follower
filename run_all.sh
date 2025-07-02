#!/bin/bash

# This script launches a hybrid test environment to validate
# hand-controlled flight with simultaneous obstacle avoidance.

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

# Terminal 6: Static TF from base_link to camera_link
gnome-terminal --tab --title="Camera Mount TF" -- bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0.1 0.0 0.0 0 0 0 base_link camera_link; exec bash"

# --- HYBRID TEST SETUP ---
# Both the hand gesture node AND the simulated obstacle node are active.

# Terminal 7: Hand Gesture Recognition Node (Provides control commands)
gnome-terminal --tab --title="Hand Gesture Recognition" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./hand_gesture_recognition_node.py; exec bash"

# Terminal 8: Simulated Depth Sensor (Provides the obstacle environment)
gnome-terminal --tab --title="Simulated Obstacles" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./simulated_depth_sensor.py; exec bash"


echo "Hybrid Test Environment (Hand Control + Avoidance) launching..."
echo "Both Hand Gesture node and Simulated Obstacle node are ENABLED."
