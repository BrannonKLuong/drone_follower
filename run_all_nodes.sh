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
PYTHON_NODES_DIR="$HOME/ros2_ws/src/$PROJECT_PACKAGE_NAME/$PROJECT_PACKAGE_NAME"

# Define the installation directory for orb_slam3_ros2 (where its executables are installed)
ORB_SLAM3_INSTALL_DIR="$HOME/ros2_ws/install/orb_slam3_ros2/share/orb_slam3_ros2"

# Define the source directory for ORB_SLAM3 library (where libORB_SLAM3.so is located after build.sh)
ORB_SLAM3_SOURCE_DIR="$HOME/ros2_ws/src/ros2_orb_slam3/ORB_SLAM3"

# Define the CORRECT path to the ORB_SLAM3 configuration YAML file
ORB_SLAM3_CONFIG_FILE="$HOME/ros2_ws/src/ros2_orb_slam3/ORB_SLAM3/Examples/Monocular-Inertial/RealSense_D435i.yaml"

# Define a directory for node logs
LOG_DIR="$HOME/ros2_ws/log/node_outputs"
mkdir -p "$LOG_DIR" # Ensure log directory exists

# Function to open a new gnome-terminal tab and run a command
run_in_new_tab() {
    local tab_title=$1
    local command_to_run=$2
    local extra_env_cmd=$3 # Renamed to extra_env_cmd to be more explicit about its use
    local log_file="${LOG_DIR}/${tab_title// /_}_$(date +%Y%m%d_%H%M%S).log" # Create unique log file name

    gnome-terminal --tab --title="$tab_title" -- bash -c "\
        # Redirect all output from this setup phase to the log file immediately
        exec &> >(tee -a \"${log_file}\"); \
        \
        echo \"--------------------------------------------------\"; \
        echo \"Node: $tab_title - Started at $(date)\"; \
        echo \"Command: ${command_to_run}\"; \
        echo \"Log file: ${log_file}\"; \
        echo \"--------------------------------------------------\"; \
        \
        # Exit immediately if a command exits with a non-zero status.
        set -e; \
        \
        # Trap signals to ensure cleanup and prompt before closing.
        # This trap is set after initial logging to ensure it's captured.
        trap 'echo \"Node \\\"$tab_title\\\" finished/crashed. Check log file: ${log_file}. Press Enter to close this terminal...\"; read -t 10;' EXIT; \
        \
        # --- CRITICAL: Source ROS 2 environments in correct order ---
        # 1. Source the main ROS 2 humble setup.bash first
        source \"/opt/ros/humble/setup.bash\"; \
        # 2. THEN, source your workspace's setup.bash to overlay it
        source \"$WORKSPACE_SETUP_FILE\"; \
        \
        # Execute any extra environment setup commands.
        ${extra_env_cmd} \
        \
        # Execute the main command. Its output is already being teed to the log file.
        $command_to_run; \
        \
        echo \"Node: $tab_title - Exited successfully at $(date)\"; \
        "
}

echo "Launching ROS 2 nodes for the Autonomous Tactical Scouting Drone in separate tabs..."
echo "Node outputs will also be logged to: $LOG_DIR"

# --- Launching Individual Nodes ---

# Mock PX4 Node
run_in_new_tab "Mock PX4" "python3 $PYTHON_NODES_DIR/mock_px4.py" ""

# Obstacle Perception Node
run_in_new_tab "Obstacle Perception" "python3 $PYTHON_NODES_DIR/obstacle_perception_node.py" ""

# Drone Commander Node
run_in_new_tab "Drone Commander" "python3 $PYTHON_NODES_DIR/current_fly_script.py" ""

# PX4 Odometry to TF Publisher Node (for RViz visualization)
run_in_new_tab "PX4 TF Publisher" "python3 $PYTHON_NODES_DIR/px4_odometry_to_tf_publisher.py" ""

# RViz2 Node
run_in_new_tab "RViz2" "ros2 run rviz2 rviz2" ""

# Static TF for Camera Mount (using ros2 run tf2_ros static_transform_publisher)
# Arguments: x y z qx qy qz qw parent_frame_id child_frame_id
run_in_new_tab "Camera TF" "ros2 run tf2_ros static_transform_publisher 0.1 0 0 0 0 0 1 base_link camera_link" ""

# ORB_SLAM3 Node
# IMPORTANT: Ensure the vocabulary and config files exist at these paths.
# Replace 'ORBvoc.txt' and 'RealSense_D435i_mono_imu.yaml' with your actual file names.
run_in_new_tab "ORB_SLAM3" "ros2 run orb_slam3_ros2 orb_alt $ORB_SLAM3_INSTALL_DIR/Vocabulary/ORBvoc.txt $ORB_SLAM3_CONFIG_FILE" "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ORB_SLAM3_SOURCE_DIR}/lib"

# Hand Gesture Recognition Node (Temporarily commented out to avoid camera conflicts)
# run_in_new_tab "Hand Gesture" "python3 $PYTHON_NODES_DIR/hand_gesture_recognition_node.py" ""

# Simulated Depth Sensor Node (Optional, uncomment if you want to use it)
# run_in_new_tab "Simulated Depth Sensor" "python3 $PYTHON_NODES_DIR/simulated_depth_sensor.py" ""

# Strobe Light Node (Optional, uncomment if you want to use it)
run_in_new_tab "Strobe Light" "python3 $PYTHON_NODES_DIR/strobe_light.py" ""

echo "All nodes launched. Check the new terminal tabs."


