#!/bin/bash

# Define the workspace setup file
WORKSPACE_SETUP_FILE="$HOME/ros2_ws/install/setup.bash"

# Source the ROS 2 workspace setup file in the current terminal (for this script's execution)
if [ -f "$WORKSPACE_SETUP_FILE" ]; then
    source "$WORKSPACE_SETUP_FILE"
    echo "ROS 2 workspace sourced: $WORKSPACE_SETUP_FILE"
else
    echo "Error: ROS 2 workspace setup file not found at $WORKSPACE_SETUP_FILE"
    exit 1
fi

# Define the project package name (for ros2 run commands)
PROJECT_PACKAGE_NAME="drone_project"

# Define the base directory for your Python scripts within the package
# This assumes your Python nodes are in ~/ros2_ws/src/drone_project/drone_project/
PYTHON_NODES_DIR="$HOME/ros2_ws/src/$PROJECT_PACKAGE_NAME/$PROJECT_PACKAGE_NAME"

# Function to open a new gnome-terminal tab and run a command
run_in_new_tab() {
    local tab_title=$1
    local command_to_run=$2
    gnome-terminal --tab --title="$tab_title" -- bash -c "\
        # --- CRITICAL: Source ROS 2 environments in correct order ---
        # 1. Source the main ROS 2 humble setup.bash first
        source \"/opt/ros/humble/setup.bash\"; \
        # 2. THEN, source your workspace's setup.bash to overlay it
        source \"$WORKSPACE_SETUP_FILE\"; \
        \
        # Execute the command, and then pause the terminal
        ( $command_to_run ); \
        read -p \"Press Enter to close this terminal...\" -n 1; \
        exec bash"
}

echo "Launching ROS 2 nodes for the Autonomous Tactical Scouting Drone in separate tabs..."

# --- Launching Individual Nodes ---

# Mock PX4 Node
run_in_new_tab "Mock PX4" "python3 $PYTHON_NODES_DIR/mock_px4.py"

# Obstacle Perception Node
run_in_new_tab "Obstacle Perception" "python3 $PYTHON_NODES_DIR/obstacle_perception_node.py"

# Drone Commander Node
run_in_new_tab "Drone Commander" "python3 $PYTHON_NODES_DIR/current_fly_script.py"

# PX4 Odometry to TF Publisher Node
run_in_new_tab "PX4 TF Publisher" "python3 $PYTHON_NODES_DIR/px4_odometry_to_tf_publisher.py"

# RViz2 Node
run_in_new_tab "RViz2" "ros2 run rviz2 rviz2"

# Static TF for Camera Mount (using ros2 run tf2_ros static_transform_publisher)
# Arguments: x y z qx qy qz qw parent_frame_id child_frame_id
run_in_new_tab "Camera TF" "ros2 run tf2_ros static_transform_publisher 0.1 0 0 0 0 0 1 base_link camera_link"

# Hand Gesture Recognition Node
run_in_new_tab "Hand Gesture" "python3 $PYTHON_NODES_DIR/hand_gesture_recognition_node.py"

# Simulated Depth Sensor Node (Optional, uncomment if you want to use it)
# run_in_new_tab "Simulated Depth Sensor" "python3 $PYTHON_NODES_DIR/simulated_depth_sensor.py"

# Strobe Light Node (Optional, uncomment if you want to use it)
# run_in_new_tab "Strobe Light" "python3 $PYTHON_NODES_DIR/strobe_light.py"

echo "All nodes launched. Check the new terminal tabs."
