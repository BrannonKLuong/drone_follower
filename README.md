# Autonomous Tactical Scouting Drone

**Status:** `ORB_SLAM3 Integrated - Validated on NVIDIA Orin Nano & PX4 Ready` 🚀

This repository contains the **ROS 2** software stack for a compact, autonomous drone designed for tactical indoor and close-quarters operations. The system uses an **offboard computing architecture** to enable an aerial platform to perform complex perception and navigation tasks, with real-time localization provided by ORB_SLAM3.

---

## Current System State & Key Achievements (Simulation on NVIDIA Orin Nano with PX4 Integration)

The core control loop and perception pipeline have been successfully migrated and validated on the NVIDIA Orin Nano Developer Kit. The system now communicates with a **mock PX4 flight controller** and integrates ORB_SLAM3 for robust visual-inertial localization. This setup demonstrates enhanced performance and readiness for integration with a physical PX4-flashed flight controller.

### Highlights:

* **PX4 Compatibility:** The entire ROS 2 communication layer is refactored to interface with PX4-like messages and topics.
* **ORB_SLAM3 Integrated Localization:** ORB_SLAM3 is now the primary localization source. It directly processes RealSense camera data (color and IMU) to provide high-accuracy pose estimates to the mock PX4 flight controller.
* **Full System Operational:** All custom ROS 2 nodes (ORB_SLAM3, Obstacle Perception, Drone Commander, Mock PX4, TF Publishers) are running concurrently on the NVIDIA Orin Nano.
* **RealSense Camera Integrated:** The Intel RealSense D435i camera is fully functional, with its data primarily consumed by ORB_SLAM3.
* **GPU Accelerated Perception:**
    * **Euclidean Clustering:** The `obstacle_perception_node.py` leverages **CuPy** for GPU acceleration of its clustering algorithm, significantly reducing CPU load.
    * **MediaPipe:** (Note: `hand_gesture_recognition_node.py` is currently disabled to avoid camera conflicts with ORB_SLAM3. When active, it implicitly utilizes the GPU for AI inference.)
* **Intuitive Control:** (Note: Hand gesture control is currently disabled. When active, 3D hand gestures (pointing for movement, fist for land) are correctly interpreted and control the mock drone's movement and yaw in 3D space.)
* **Visual Feedback:** Real-time annotated video feed (from ORB_SLAM3's viewer) and comprehensive 3D visualization in RViz2 (via NoMachine) are fully operational.
* **Robust Launch System:** The project uses a custom bash script (`run_all_nodes.sh`) to reliably launch all nodes in separate terminal tabs, ensuring proper environment sourcing and library paths for ORB_SLAM3.

### Live Demonstration

For a detailed view of the 3D hand tracking and gesture recognition with depth data, see the GIF below.
*(Note: This GIF shows hand gesture control, which is currently disabled in the ORB_SLAM3 integrated setup due to camera conflicts. A new GIF showing ORB_SLAM3 in action would be more representative.)*

![Hand Gesture Recognition with Depth](./assets/Depth%20Directional%20Hand%20Commands.gif)

### Orb Slam3 Demonstration
![ORB_SLAM3 perform real-time visual-inertial odometry and mapping](./assets/orb3_slam_with_tf.gif)

### Other Demonstrations

| Hand Gesture Control (Simulated 2D Webcam) | Drone Following Strobe Light Node |
| :----------------------------------------- | :-------------------------------- |
| ![Hand Gesture Control](./assets/Drone%20Hand%20Gesture%20Ros.gif) | ![Strobe Following](./assets/Drone%20Following%20Strobe%20Node%20Ros.gif) |

* **On the left (Simulated 2D):** The "**Hand Gesture Recognition**" node processes a live video feed, using an **AI model (MediaPipe)** to classify hand poses into high-level flight commands.
* **On the right:** The drone autonomously follows strobe light node.

---

## Core Features & Capabilities (Implemented)

* **Real-Time 3D Localization (via ORB_SLAM3):** The drone now uses ORB_SLAM3 for robust visual-inertial odometry, providing accurate pose estimates in GPS-denied environments.
* **Advanced Obstacle Avoidance via Potential Fields:** The drone uses a custom **potential field algorithm** for navigation. This method calculates a real-time trajectory based on two competing forces:
    * **Attractive Force:** A vector pulling the drone towards its current goal (e.g., a waypoint or a command target).
    * **Repulsive Force:** A vector pushing the drone away from any nearby obstacles. This system uses a **Euclidean Clustering algorithm** (GPU-accelerated via **CuPy**) to group points into distinct objects. It then publishes the 3D centroids of all detected clusters to the `/detected_obstacles_centroids` topic.
* **Dual-Mode Control System:** A state machine in the drone's "brain" (`current_fly_script.py`) manages its behavior, allowing it to operate in two primary modes:
    * **`HAND_CONTROL`:** (Currently disabled due to camera conflict with SLAM). When active, the drone is controlled directly by the operator's hand gestures.
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
* **`orb_slam3_ros2` (Executable: `orb_alt` or `imu_mono_node_cpp`):** This is the core SLAM node. It directly interfaces with the **Intel RealSense D435i camera** to acquire synchronized color and IMU frames. It performs visual-inertial odometry and loop closure, publishing the estimated camera pose as a `geometry_msgs/PoseStamped` message on `/orb_slam3/camera_pose`.
* **`hand_gesture_recognition_node.py`:** (Currently disabled in the launch script due to direct camera access conflict with ORB_SLAM3). When active, this node directly interfaces with the RealSense camera, uses MediaPipe for hand detection, projects 2D landmarks to 3D, calculates pointing vectors, and publishes hand commands. It also publishes `sensor_msgs/PointCloud2` and `sensor_msgs/Imu` data.
* **`obstacle_perception_node.py`:** This node subscribes to the real-time `PointCloud2` data (from RealSense, either directly or via `simulated_depth_sensor.py`). It processes this data using a **Euclidean Clustering algorithm** (GPU-accelerated via **CuPy**) to group points into distinct objects. It then publishes the 3D centroids of all detected clusters to the `/detected_obstacles_centroids` topic.
* **`mock_px4.py`:** In the simulation, this node mimics a **PX4 flight controller**. It now subscribes to the `/orb_slam3/camera_pose` topic to receive external vision input from ORB_SLAM3. It receives standard ROS 2 `TrajectorySetpoint` messages for position setpoints and `VehicleCommand` messages (e.g., ARM, GUIDED, LAND), and publishes the drone's changing position and flight mode via `px4_msgs/VehicleLocalPosition` and `px4_msgs/VehicleStatus`.
* **`px4_odometry_to_tf_publisher.py`:** This node subscribes to the drone's mock odometry data (`/fmu/out/vehicle_odometry` from `mock_px4.py`) and publishes the dynamic `odom` to `base_link` TF transform, allowing RViz to visualize the drone's movement.
* **`strobe_light.py`:** A simulation node that publishes a moving target for the drone to follow.
* **`simulated_depth_sensor.py`:** This node can be used to create virtual obstacle courses by publishing simulated `PointCloud2` data (optional, currently disabled in `run_all_nodes.sh`).

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
    mv current_fly_script.py hand_gesture_recognition_node.py mock_px4.py obstacle_perception_node.py px4_odometry_to_tf_publisher.py simulated_depth_sensor.py strobe_light.py drone_project/
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
* **Build Your ROS 2 Workspace (Initial Build):**
    ```bash
    cd ~/ros2_ws
    rm -rf build install log
    rosdepc install -i --from-path src --rosdistro humble -y
    colcon build --packages-select drone_project
    ```

---

## ORB_SLAM3 Installation and Integration Journey 🗺️

This section details the comprehensive steps undertaken to successfully integrate **ORB_SLAM3** for robust visual-inertial localization within this ROS 2 project. This involved resolving various build errors, managing camera resource conflicts, and correctly interfacing with the PX4 flight controller simulation.

### 1. Clone ORB_SLAM3 ROS 2 Wrapper and Dependencies:

The first step was to bring the ORB_SLAM3 ROS 2 wrapper into the workspace, ensuring its core library and external dependencies were properly fetched.

* **Navigate to ROS 2 Workspace Source:**
    ```bash
    cd ~/ros2_ws/src
    ```
* **Clone `ros2_orb_slam3` Repository:**
    ```bash
    git clone [https://github.com/your-username/ros2_orb_slam3.git](https://github.com/your-username/ros2_orb_slam3.git) # Replace with the actual repository URL
    ```
* **Initialize and Update Submodules:**
    Navigate into the core ORB_SLAM3 directory (usually located within the cloned wrapper) and initialize its Git submodules. This is crucial for fetching necessary third-party libraries like DBoW2, g2o, and Sophus.
    ```bash
    cd ros2_orb_slam3/ORB_SLAM3
    git submodule update --init --recursive
    ```
* **Build ORB_SLAM3 Core Library:**
    After fetching submodules, compile the core ORB_SLAM3 library using its provided build script. This generates the essential shared libraries (`libORB_SLAM3.so`, `libDBoW2.so`, `libg2o.so`, etc.) that the ROS 2 wrapper will link against.
    ```bash
    chmod +x build.sh # Ensure the build script is executable
    ./build.sh
    ```
    *This step was critical in resolving initial "fatal error: Thirdparty/DBoW2/DBoW2/BowVector.h: No such file or directory" errors, confirming that ORB_SLAM3's internal dependencies were correctly built and available.*

### 2. Configure `ros2_orb_slam3` for ROS 2 Build:

With the core ORB_SLAM3 library successfully built, precise configuration of the `ros2_orb_slam3` package's CMake build system was required to correctly link against ORB_SLAM3 and other system libraries in the ROS 2 environment.

* **Edit `CMakeLists.txt` for `ros2_orb_slam3`:**
    ```bash
    gedit ~/ros2_ws/src/ros2_orb_slam3/CMakeLists.txt
    ```
* **Key CMake Modifications Applied:**
    * **CMake Policies:** Added `cmake_policy(SET CMP0074 NEW)` near the top of the file. This policy helps manage `find_package` behavior and suppresses specific warnings related to PCL.
    * **Define ORB_SLAM3 Source Directory:** Ensured the `ORB_SLAM3_SOURCE_DIR` variable was correctly set to point to the root of the ORB_SLAM3 core library: `set(ORB_SLAM3_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3)`.
    * **Adjust Include Directories:** Crucially, the root ORB_SLAM3 source directory (`${ORB_SLAM3_SOURCE_DIR}`) was explicitly added to `include_directories`. This was vital for the compiler to correctly resolve internal relative paths within ORB_SLAM3's own headers (e.g., `#include "Thirdparty/DBoW2/DBoW2/BowVector.h"`). Additional specific include paths for `g2o` and `Sophus` were also ensured.
    * **Specify Library Link Directories:** `link_directories` commands were added to point to the `lib/` folders where the compiled `libORB_SLAM3.so` and its third-party components (`g2o`, `DBoW2`) resided after the `build.sh` step.
    * **Direct Library Linking:** In the `target_link_libraries` sections for the `orb_slam3_ros2` executables (e.g., `imu_mono_node_cpp`, `orb_alt`), explicit links were made to `ORB_SLAM3`, `g2o`, `DBoW2`, and critically, `realsense2`.
        *This resolved persistent `undefined reference to rs2_...` linker errors, enabling the ROS 2 node to correctly interact with the RealSense SDK at runtime.*
    * **C++ Standard:** Explicitly set `CMAKE_CXX_STANDARD` to `17` to ensure modern C++ features are used.

### 3. Adapting for PX4 and SLAM Pose Input:

The drone's mock flight controller (`mock_px4.py`) and the overall system launch mechanism were updated to seamlessly consume ORB_SLAM3's pose output, establishing a robust PX4-exclusive simulation environment.

* **`mock_px4.py` Modifications (`~/ros2_ws/src/drone_project/drone_project/mock_px4.py`):**
    * **SLAM Pose Subscription:** The `mock_px4.py` node was updated to subscribe to the `/orb_slam3/camera_pose` topic (a `geometry_msgs/PoseStamped` message).
    * **External Vision Integration Callback:** A new callback function, `external_vision_pose_callback`, was implemented. This function takes the incoming pose (expected in ROS ENU frame) from SLAM, converts it to PX4's native NED frame, and then directly updates the mock drone's internal simulated `current_x`, `current_y`, `current_z`, and `current_yaw`. This effectively mimics how a real PX4's Extended Kalman Filter (EKF) would fuse external vision data for localization.
    * **Simulated Movement Logic Adjustment:** The `publish_sim_status` logic was adjusted. If external vision data is being received from SLAM, the mock drone's position is now primarily driven by the SLAM input, rather than its internal simple movement simulation.

* **Launch Script Refinement (`~/ros2_ws/src/drone_project/run_all_nodes.sh`):**
    * **ORB_SLAM3 Node Launch:** A specific `run_in_new_tab` command was added to launch the `orb_slam3_ros2` executable (e.g., `orb_alt`). This command passes the required ORB_SLAM3 vocabulary and camera configuration file paths as arguments.
    * **`LD_LIBRARY_PATH` Export:** Crucially, an `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ORB_SLAM3_SOURCE_DIR}/lib` command was embedded directly within the `run_in_new_tab` call for the ORB_SLAM3 terminal. This ensures that the system's dynamic linker can find `libORB_SLAM3.so` at runtime, resolving "cannot open shared object file" errors that occurred during execution.
    * **Camera Resource Management:** The `hand_gesture_recognition_node.py` launch command was explicitly commented out in the `run_all_nodes.sh` script. This was a necessary step to prevent simultaneous, conflicting access to the RealSense camera by both ORB_SLAM3 (which directly controls the camera pipeline) and the hand gesture node.
    * **PX4 TF Publisher:** Confirmed that `px4_odometry_to_tf_publisher.py` is correctly used for TF broadcasting, maintaining alignment with the PX4-exclusive focus.
    * **Debugging Aid:** The `run_in_new_tab` function itself was enhanced to include a `read -p` prompt. This keeps individual terminal tabs open after a node exits (even if it crashes), allowing for easier diagnosis by displaying any error messages.

### 4. Final Build and Verification:

After all code modifications and configuration changes, a rigorous clean and complete rebuild of the entire ROS 2 workspace was performed to ensure all changes were applied and dependencies correctly linked.

```bash
cd ~/ros2_ws
rm -rf build install log # Full clean is highly recommended
rosdepc install -i --from-path src --rosdistro humble -y
export COLCON_MAKE_ARGS="-j4" # Using 4 parallel jobs for faster build
colcon build --parallel-workers 1 --executor sequential
unset COLCON_MAKE_ARGS
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
