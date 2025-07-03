#!/bin/bash

# This script launches the full test environment to validate
# hand-controlled flight with simultaneous obstacle avoidance and strobe following.

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

# --- REAL-TIME SENSOR AND GOAL NODES ---

# Terminal 7: Hand Gesture Recognition Node (Provides control commands and real point cloud)
gnome-terminal --tab --title="Hand Gesture Recognition" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./hand_gesture_recognition_node.py; exec bash"

# NEW: Terminal 8: Strobe Light Publisher (Provides the autonomous goal)
gnome-terminal --tab --title="Strobe Light" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./strobe_light.py; exec bash"

# DISABLED: The simulated depth sensor is now disabled to allow the RealSense camera from the
# hand gesture node to be the sole provider of point cloud data.
# gnome-terminal --tab --title="Simulated Obstacles" -- bash -c "cd $PROJECT_DIR && source /opt/ros/humble/setup.bash && python3 ./simulated_depth_sensor.py; exec bash"


echo "Full Test Environment (Strobe Following + Avoidance) launching..."
echo "Simulated obstacle node is DISABLED. RealSense is the primary sensor."
echo "Strobe light publisher is ENABLED."