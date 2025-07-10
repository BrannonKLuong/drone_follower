# Autonomous Tactical Scouting Drone

**Status:** `ArduPilot Integration Complete - Validated on NVIDIA Orin Nano & Ready for Flight Controller Testing`

This repository contains the **ROS 2** software stack for a compact, autonomous drone designed for tactical indoor and close-quarters operations. The system uses an **offboard computing architecture** to enable a small, agile aerial platform to perform complex perception and navigation tasks, with real-time control via hand gestures from an operator.

---

## Current System State & Key Achievements (Simulation on NVIDIA Orin Nano with ArduPilot Integration)

The core control loop and perception pipeline have been successfully migrated and validated on the NVIDIA Orin Nano Developer Kit, now communicating with a **mock ArduPilot flight controller**. This transition was necessary due to the **Cube Orange+ flight controller's compatibility with ArduPilot**, rather than PX4. The system now demonstrates robust performance and is ready for integration with the physical ArduPilot-flashed flight controller.

### Highlights:

* **ArduPilot Compatibility:** The entire ROS 2 communication layer has been refactored to interface with ArduPilot-like messages and topics, replacing the previous PX4-specific interfaces.

* **Full System Operational:** All custom ROS 2 nodes (Hand Gesture Recognition, Obstacle Perception, Drone Commander, Mock ArduPilot, TF Publishers) are running concurrently on the NVIDIA Orin Nano.

* **RealSense Camera Integrated:** The Intel RealSense D435i camera is fully functional, providing real-time color, depth, and IMU data.

* **GPU Accelerated Perception:**

    * **Euclidean Clustering:** The `obstacle_perception_node.py` leverages **CuPy** for GPU acceleration of its clustering algorithm, significantly reducing CPU load for this intensive task.

    * **MediaPipe:** Hand detection in `hand_gesture_recognition_node.py` implicitly utilizes the GPU for its AI inference.

* **Intuitive Control:** 3D hand gestures (pointing for movement, fist for land) are correctly interpreted and control the mock drone's movement and yaw in 3D space.

* **Visual Feedback:** Real-time annotated video feed (OpenCV window) and comprehensive 3D visualization in RViz2 (via NoMachine) are fully operational.

* **Robust Launch System:** The project uses a custom bash script (`run_all_nodes.sh`) to reliably launch all nodes in separate terminal tabs, ensuring proper environment sourcing.

### Live Demonstration

For a detailed view of the 3D hand tracking and gesture recognition with depth data, see the GIF below.

![Hand Gesture Recognition with Depth](./assets/Depth%20Directional%20Hand%20Commands.gif)

### Other Demonstrations

| Hand Gesture Control (Simulated 2D Webcam) | Drone Following Strobe Light Node |
| :----------------------------------------- | :-------------------------------- |
| ![Hand Gesture Control](./assets/Drone%20Hand%20Gesture%20Ros.gif) | ![Strobe Following](./assets/Drone%20Following%20Strobe%20Node%20Ros.gif) |

* **On the left (Simulated 2D):** The "**Hand Gesture Recognition**" node processes a live video feed, using an **AI model (MediaPipe)** to classify hand poses into high-level flight commands.

* **On the right:** The drone autonomously follows strobe light node.

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

* **`mock_ardupilot.py`:** In the simulation, this node mimics an **ArduPilot flight controller**. It receives standard ROS 2 `PoseStamped` messages for setpoints and `String` commands (e.g., "ARM", "GUIDED", "LAND"), and publishes the drone's changing position and flight mode via `nav_msgs/Odometry` and `std_msgs/String`. This node facilitates development and testing with ArduPilot's expected communication patterns.

* **`ardupilot_odometry_to_tf_publisher.py`:** This node subscribes to the drone's mock odometry data (`/ardupilot/out/odometry`) and publishes the dynamic `odom` to `base_link` TF transform, allowing RViz to visualize the drone's movement.

* **`strobe_light.py`:** A simulation node that publishes a moving target for the drone to follow (optional, currently disabled in `run_all_nodes.sh`).

* **`simulated_depth_sensor.py`:** This node can be used to create virtual obstacle courses by publishing simulated `PointCloud2` data (optional, currently disabled in `run_all_nodes.sh`).

---

## PX4 Compatibility Note (Historical Context)

Prior to the current ArduPilot integration, this project was configured to interface with PX4 flight controllers. The original PX4-compatible nodes (`mock_px4.py`, `px4_odometry_to_tf_publisher.py`, and the PX4-specific message imports in `current_fly_script.py`) have been replaced.

**While the current active codebase is for ArduPilot, the core logic and structure of the `DroneCommander` and perception nodes are adaptable.** If there's a future need to revert to PX4 or support both, the previous versions of the PX4 interface nodes could be re-integrated, and the `current_fly_script.py` could be modified to conditionally use PX4 messages/topics.

---
## Future Work / Next Steps

Once the Cube Orange+ flight controller is fully integrated and the drone achieves basic flight functionality (e.g., motors fire, stable hover), the next significant step will be to implement a **Simultaneous Localization and Mapping (SLAM) algorithm**. This will enable the drone to:

* **Build a map of its unknown environment** in real-time.
* **Localize itself within that map** without relying solely on external positioning systems like GPS.

This is crucial for robust autonomous navigation in complex indoor and GPS-denied environments, allowing the drone to understand its surroundings and track its own position accurately.

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
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
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
    # Ensure the internal drone_project directory exists and contains the Python nodes
    mkdir -p drone_project
    # Create __init__.py files for Python package recognition
    touch __init__.py # In ~/ros2_ws/src/drone_project/
    touch drone_project/__init__.py # In ~/ros2_ws/src/drone_project/drone_project/
    # Move Python scripts into the inner package directory
    mv current_fly_script.py hand_gesture_recognition_node.py mock_ardupilot.py obstacle_perception_node.py ardupilot_odometry_to_tf_publisher.py simulated_depth_sensor.py strobe_light.py drone_project/
    # Create the launch directory and move the launch file into it
    mkdir -p launch
    mv run_all_advanced.launch.py launch/
    # Create the resource directory
    mkdir -p resource
    touch resource/drone_project
    ```

* **Create `package.xml` (in `~/ros2_ws/src/drone_project/`):**

    ```bash
    nano package.xml
    ```

    (Copy content from our previous conversations for `package.xml`)

* **Create `setup.py` (in `~/ros2_ws/src/drone_project/`):**

    ```bash
    nano setup.py
    ```

    (Copy content from our previous conversations for `setup.py`)

* **Install Python Dependencies (CuPy, MediaPipe):**

    ```bash
    pip3 install cupy-cuda12x # Or specific CUDA 12.x version if known
    pip3 install mediapipe opencv-python numpy
    ```

* **Build Your ROS 2 Workspace:**

    ```bash
    cd ~/ros2_ws
    rm -rf build install log
    colcon build --packages-select drone_project
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

    (Copy content from our previous conversations for `run_all_nodes.sh`, ensuring it launches individual nodes in separate tabs and the simulated components are commented out).

* **Make Executable:**

    ```bash
    chmod +x run_all_nodes.sh
    ```

* **Launch the System:**

    ```bash
    cd ~/ros2_ws/src/drone_project/
    ./run_all_nodes.sh
    ```

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

## Acknowledgements and Licensing

* **JetsonHacks/jetson-orin-librealsense**: The kernel modules for Intel RealSense cameras on NVIDIA Jetson Orin devices are adapted from the work by JetsonHacks. This repository is licensed under the MIT License.

    * Repository: <https://github.com/jetsonhacks/jetson-orin-librealsense>

    * License: [MIT License](https://github.com/jetsonhacks/jetson-orin-librealsense/blob/main/LICENSE)

* **Intel RealSense SDK (librealsense)**: The Intel RealSense SDK itself, which the kernel modules support, is covered by Intel's own license.

* **GNU General Public License (GPL) Version 2**: Components of the Linux kernel, which the RealSense kernel modules interact with, are typically licensed under the GNU GPL.

* **ArduPilot**: This project leverages the concepts and communication protocols compatible with ArduPilot. ArduPilot is open-source and licensed under the **GNU General Public License (GPL) version 3**. While this project is a personal endeavor and does not directly re-distribute ArduPilot code, users integrating with a physical ArduPilot flight controller should be aware of its licensing.

    * ArduPilot Website: <https://ardupilot.org/>

    * ArduPilot License Information: [https://ardupilot.org/dev/docs/licensing.html](https://ardupilot.org/dev/docs/license-gplv3.html)
