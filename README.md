# Autonomous Tactical Scouting Drone

**Status:** `Simulation Phase Complete - Validated on NVIDIA Orin Nano & Ready for Hardware Integration`

This repository contains the **ROS 2** software stack for a compact, autonomous drone designed for tactical indoor and close-quarters operations. The system uses an **offboard computing architecture** to enable a small, agile aerial platform to perform complex perception and navigation tasks, with real-time control via hand gestures from an operator.

---

## Current System State & Key Achievements (Simulation on NVIDIA Orin Nano)

The core control loop and perception pipeline have been successfully migrated and validated on the NVIDIA Orin Nano Developer Kit. The system now demonstrates robust performance and is ready for integration with the physical flight controller.

### Highlights:
* **Full System Operational:** All custom ROS 2 nodes (Hand Gesture Recognition, Obstacle Perception, Drone Commander, Mock PX4, TF Publishers) are running concurrently on the NVIDIA Orin Nano.
* **RealSense Camera Integrated:** The Intel RealSense D435i camera is fully functional, providing real-time color, depth, and IMU data.
* **GPU Accelerated Perception:**
    * **Euclidean Clustering:** The `obstacle_perception_node.py` now leverages **CuPy** for GPU acceleration of its clustering algorithm, significantly reducing CPU load for this intensive task. `tegrastats` confirms `GR3D_FREQ` (GPU utilization) hitting 30-40% during operation.
    * **MediaPipe:** Hand detection in `hand_gesture_recognition_node.py` implicitly utilizes the GPU for its AI inference.
* **Intuitive Control:** 3D hand gestures (pointing for movement, fist for land) are correctly interpreted and control the mock drone's movement and yaw in 3D space.
* **Visual Feedback:** Real-time annotated video feed (OpenCV window) and comprehensive 3D visualization in RViz2 (via NoMachine) are fully operational.
* **Robust Launch System:** The project uses a custom bash script (`run_all_nodes.sh`) to reliably launch all nodes in separate terminal tabs, ensuring proper environment sourcing.

### Live Demonstration
*(Update these GIFs once you have captures from the Orin Nano)*
For a detailed view of the 3D hand tracking and gesture recognition with depth data, see the GIF below.
![Hand Gesture Recognition with Depth](./assets/Depth%20Directional%20Hand%20Commands.gif)

---

## Core Features & Capabilities (Implemented)

* **Real-Time 3D Hand Gesture Control:** The drone is controlled by high-level commands generated from an **AI-powered perception node**. The system uses a **debouncing algorithm**, requiring a gesture to be held for several frames before a command is confirmed. Implemented gestures include:
    * **Open Palm (`MOVE_FORWARD`):** Commands the drone to move in the 3D direction pointed by the hand, with precise yaw control.
    * **Fist (`LAND`):** Commands the drone to initiate a safe landing sequence.
    * **No Hand (`HOVER`):** The drone holds its position if no command is detected.

* **Advanced Obstacle Avoidance via Potential Fields:** The drone uses a custom **potential field algorithm** for navigation. This method calculates a real-time trajectory based on two competing forces:
    * **Attractive Force:** A vector pulling the drone towards its current goal (e.g., a waypoint or the direction of a hand gesture).
    * **Repulsive Force:** A vector pushing the drone away from any nearby obstacles. This system uses a **Euclidean Clustering algorithm** (GPU-accelerated via **CuPy**) to perceive and react to multiple distinct obstacles simultaneously (e.g., navigating through a doorway or slalom course).

* **Dual-Mode Control System:** A state machine in the drone's "brain" (`current_fly_script.py`) manages its behavior, allowing it to operate in two primary modes:
    * **`HAND_CONTROL`:** The default mode, where the drone is controlled directly by the operator's hand gestures.
    * **`MISSION`:** An autonomous mode where the drone follows a pre-defined list of waypoints, using its advanced obstacle avoidance to navigate the environment.

---

## System Architecture

This project uses a **Remote Computing** (or "**Offboard Computing**") architecture to maximize the drone's agility and flight time while enabling powerful AI processing.

1.  **On-Drone Hardware:** A lightweight **NVIDIA Orin Nano Developer Kit** acts as the on-board compute platform. Its purpose is to perform all real-time sensor processing, perception, navigation, and control directly on the drone, enabling **GPS-denied autonomous flight**.
2.  **Ground Station/Development:** A remote PC (accessed via NoMachine) serves as the development and visualization platform.

---

## Software Node Breakdown (Operational)

This system is composed of several custom ROS 2 nodes that work in concert.

* **`current_fly_script.py`:** The central command node or "**brain**." It subscribes to all sensor and command inputs, manages the control state (Hand vs. Mission), and implements the **Potential Field algorithm** using multi-obstacle data from the perception node to calculate a combined repulsive force.
* **`hand_gesture_recognition_node.py`:** This node directly interfaces with the **Intel RealSense D435i camera** using `pyrealsense2`. It performs the following critical functions:
    * Acquires synchronized color, depth, and IMU frames.
    * Applies a decimation filter to the depth data for performance.
    * Uses OpenCV and **MediaPipe (GPU-accelerated)** to detect hand landmarks.
    * Projects 2D hand landmarks to 3D coordinates using depth data.
    * Employs a robust method using dot products for accurate finger extension detection.
    * Calculates a 3D pointing vector from the index finger.
    * Publishes detected hand commands (`String`), 3D hand positions (`geometry_msgs/PointStamped`), and 3D pointing vectors (`geometry_msgs/Vector3Stamped`).
    * Publishes `sensor_msgs/PointCloud2` (`/camera/camera/depth/color/points`) and `sensor_msgs/Imu` data (`/camera/camera/imu`).
    * Publishes static TF transforms for the camera's internal frames.
    * Displays a live, annotated video feed using `cv2.imshow()`.
* **`obstacle_perception_node.py`:** This node subscribes to the real-time `PointCloud2` data. It processes this data using a **Euclidean Clustering algorithm** (GPU-accelerated via **CuPy**) to group points into distinct objects. It then publishes the 3D centroids of all detected clusters to the `/detected_obstacles_centroids` topic.
* **`mock_px4.py`:** In the simulation, this node mimics a real PX4 flight controller. It receives `TrajectorySetpoint` messages and publishes the drone's changing position and status. This will be replaced by the real **Cube Orange+** in the hardware phase.
* **`px4_odometry_to_tf_publisher.py`:** This node subscribes to the drone's mock odometry data (`/fmu/out/vehicle_odometry`) and publishes the dynamic `odom` to `base_link` TF transform, allowing RViz to visualize the drone's movement.
* **`strobe_light.py`:** A simulation node that publishes a moving target for the drone to follow (optional).
* **`simulated_depth_sensor.py`:** This node can be used to create virtual obstacle courses by publishing simulated `PointCloud2` data (optional, for hybrid testing).

---

## Installation Guide: Setting up NVIDIA Orin Nano for Drone Development

This guide details the steps to set up a fresh NVIDIA Orin Nano Developer Kit with Ubuntu 22.04 (JetPack), ROS 2 Humble, Intel RealSense drivers, and all necessary project dependencies.

### 1. Initial Orin Nano Setup (JetPack Flashing)

* **Host PC:** Use an Ubuntu 22.04 VM (or native Linux PC) to run NVIDIA SDK Manager.
* **Flash JetPack:**
    1.  Download and install [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager) on your host PC.
    2.  Run SDK Manager, log in, select "Jetson Orin Nano Developer Kit" and the latest JetPack version for Ubuntu 22.04.
    3.  Connect your Orin Nano to your host PC's USB recovery port and put it into recovery mode (power off, hold recovery button, plug in power, release button).
    4.  Follow SDK Manager prompts to flash JetPack onto your Orin Nano's eMMC/SD card.
* **First Boot (Direct Connection Required):** After flashing, disconnect from the host PC. Connect a **monitor, keyboard, and mouse directly** to your Orin Nano. Power it on. Complete the initial Ubuntu setup wizard (accept EULA, create user, set locale, connect to network). This step cannot be done headless.

### 2. System Updates and Essential Tools

* **Open a terminal on your Orin Nano (via direct connection or NoMachine if already set up).**
* **Update System:**
    ```bash
    sudo apt update
    sudo apt upgrade -y
    ```
* **Install Essential Tools:**
    ```bash
    sudo apt install nano git curl wget htop tmux -y
    ```

### 3. Install NoMachine Server (for Remote Desktop Access)

* **Open a web browser on your Orin Nano.**
* Go to the [NoMachine Download Page](https://www.nomachine.com/download/linux).
* Download the **ARM64 / AARCH64** `.deb` package for Ubuntu 22.04.
* **Install:**
    ```bash
    cd ~/Downloads
    sudo dpkg -i nomachine_*.deb # Replace with actual filename
    ```
* You can now connect to your Orin Nano remotely using the NoMachine client on your PC.

### 4. Install ROS 2 Humble (Debian Packages)

* **Open a terminal on your Orin Nano (via NoMachine).**
* **Set Locale:**
    ```bash
    sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
* **Add ROS 2 Repository:**
    ```bash
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
* **Install ROS 2 Humble Desktop:**
    ```bash
    sudo apt update
    sudo apt upgrade -y
    sudo apt install ros-humble-desktop -y
    ```
* **Install ROS 2 Development Tools:**
    ```bash
    sudo apt install python3-pip -y
    pip3 install -U colcon-common-extensions rosdepc
    sudo rosdepc init # This might fail with sudo, but rosdepc update should work
    rosdepc update
    ```
* **Fix PATH for ~/.local/bin (crucial for pip-installed tools):**
    ```bash
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    ```

### 5. Install Intel RealSense Drivers and SDK

* **Open a terminal on your Orin Nano (via NoMachine).**
* **Register Public Key:**
    ```bash
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    ```
* **Add Repository:**
    ```bash
    sudo add-apt-repository "deb [https://librealsense.intel.com/Debian/apt-repo](https://librealsense.intel.com/Debian/apt-repo) $(lsb_release -cs) main" -u
    ```
* **Install Core SDK Packages:**
    ```bash
    sudo apt-get install librealsense2-utils -y
    sudo apt-get install librealsense2-dev -y
    ```
* **Install Python Bindings (via pip for reliability):**
    ```bash
    pip3 install pyrealsense2
    ```

### 6. Install RealSense Kernel Modules (Jetson-Specific)

* **Open a terminal on your Orin Nano (via NoMachine).**
* **Download Modules:**
    ```bash
    cd ~
    wget [https://github.com/jetsonhacks/jetson-orin-librealsense/raw/main/install-modules.tar.gz](https://github.com/jetsonhacks/jetson-orin-librealsense/raw/main/install-modules.tar.gz)
    wget [https://github.com/jetsonhacks/jetson-orin-librealsense/raw/main/install-modules.tar.gz.sha256](https://github.com/jetsonhacks/jetson-orin-librealsense/raw/main/install-modules.tar.gz.sha256)
    ```
* **Verify Checksum:**
    ```bash
    sha256sum -c install-modules.tar.gz.sha256
    ```
* **Extract and Install:**
    ```bash
    tar -xzf install-modules.tar.gz
    cd install-modules
    sudo ./install-realsense-modules.sh
    ```
* **Reboot Orin Nano:**
    ```bash
    sudo reboot
    ```
* **Licensing Note:** The kernel modules provided by JetsonHacks are derived from Intel RealSense code and may incorporate components under the GNU General Public License (GPL) as part of the Linux kernel. For full licensing details, please refer to the `THIRD-PARTY-LICENSES.md` file in this repository.

### 7. Project Setup and Build

* **After reboot, reconnect via NoMachine.**
* **Open a terminal.**
* **Create ROS 2 Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
* **Clone Your Project Repository:**
    ```bash
    git clone [https://github.com/BrannonKLuong/drone_follower.git](https://github.com/BrannonKLuong/drone_follower.git) drone_project
    ```
* **Fix `drone_project` Package Structure (Crucial for colcon build):**
    ```bash
    cd ~/ros2_ws/src/drone_project/
    mkdir -p drone_project
    touch __init__.py # In ~/ros2_ws/src/drone_project/
    touch drone_project/__init__.py # In ~/ros2_ws/src/drone_project/drone_project/
    mv current_fly_script.py hand_gesture_recognition_node.py mock_px4.py obstacle_perception_node.py px4_odometry_to_tf_publisher.py simulated_depth_sensor.py strobe_light.py drone_project/
    mkdir -p resource
    touch resource/drone_project
    ```
* **Create `package.xml` (in `~/ros2_ws/src/drone_project/`):**
    ```bash
    nano package.xml
    ```
    (Copy content from previous conversations)
* **Create `setup.py` (in `~/ros2_ws/src/drone_project/`):**
    ```bash
    nano setup.py
    ```
    (Copy content from previous conversations)
* **Install Python Dependencies (CuPy, MediaPipe):**
    ```bash
    pip3 install cupy-cuda12x # Or specific CUDA 12.x version if known
    pip3 install mediapipe opencv-python numpy
    ```
* **Build Your ROS 2 Workspace:**
    ```bash
    cd ~/ros2_ws
    rm -rf build install log
    colcon build
    ```
* **Install `px4_msgs` (if `colcon build` failed for it):**
    * If `colcon build` failed on `px4_msgs` with an `ament_cmake` error, or `px4_msgs` wasn't built:
        ```bash
        cd ~/ros2_ws/src
        git clone [https://github.com/PX4/px4_msgs.git](https://github.com/PX4/px4_msgs.git)
        cd ~/ros2_ws
        colcon build # Rebuild workspace to include px4_msgs
        ```

### 8. Final Sourcing and Launch Script Setup

* **Edit `~/.bashrc` (Crucial for environment):**
    ```bash
    nano ~/.bashrc
    ```
    * Find any lines like `source /opt/ros/humble/setup.bash` and **comment them out** (add `#` at the beginning).
    * Ensure the very last line of your `.bashrc` is:
        ```bash
        source ~/ros2_ws/install/setup.bash
        ```
    * Save and Exit.
* **Open a BRAND NEW terminal (via NoMachine).**
* **Verify Sourcing:**
    ```bash
    echo $AMENT_PREFIX_PATH
    ros2 pkg list | grep drone_project
    ros2 -v # Should now work
    ```
* **Create/Verify `run_all_nodes.sh` (in `~/ros2_ws/src/drone_project/`):**
    ```bash
    nano run_all_nodes.sh
    ```
    (Copy content from previous conversations, ensuring it uses `gnome-terminal` tabs and explicit sourcing within each command: `source $WORKSPACE_SETUP_FILE && $PROJECT_BIN_DIR/mock_px4; exec bash`).
* **Make Executable:**
    ```bash
    chmod +x run_all_nodes.sh
    ```
* **Launch the System:**
    ```bash
    cd ~/ros2_ws/src/drone_project/
    ./run_all_nodes.sh
    ```

---

## RViz2 Configuration (on NoMachine Desktop)

Once RViz2 launches, configure its displays:

* **Global Options:** Fixed Frame: `odom`.
* **Add Displays:**
    * **TF:** All coordinate frames.
    * **PointCloud2:** Topic: `/camera/camera/depth/color/points`, Color Transformer: `RGB8`.
    * **MarkerArray:** Topic: `/perceived_obstacle_markers` (red spheres).
    * **Marker:** Topic: `/obstacle_markers` (blue cylinders, if simulated obstacles are used).
    * **Marker:** Topic: `/hand_3d_marker` (green sphere).
    * **Marker:** Topic: `/drone_target_marker` (purple sphere).
    * **Marker:** Topic: `/hand_pointing_arrow` (orange arrow).
    * **(Optional) PoseStamped:** Topic: `/strobe_light_position` (magenta sphere, if strobe light is used).

---

## Next Steps (Hardware Integration)

The project has successfully completed its **simulation validation phase on the NVIDIA Orin Nano**. The next major focus is on integrating the validated software stack with the physical drone hardware.

