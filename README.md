# Autonomous Tactical Scouting Drone

**Status:** `In Progress - Real-time Perception Integration & HITL Validation`

This repository contains the ROS 2 software stack for a compact, autonomous drone designed for tactical indoor and close-quarters operations. The system uses an offboard computing architecture to enable a small, agile aerial platform to perform complex perception and navigation tasks, with real-time control via hand gestures from an operator.

## Live Demonstration

The core control loop and simulated perception have been successfully validated in a software-in-the-loop (SITL) environment. The project is now actively integrating real-time sensor data from an Intel RealSense D435i depth camera.

### Hand Gesture Control (RealSense 3D Depth)

For a detailed view of the 3D hand tracking and gesture recognition with depth data, see the GIF below.
![Hand Gesture Recognition with Depth](./assets/Depth%Directional%Hand%Commands.gif)

### Other Demonstrations

| Hand Gesture Control (Simulated 2D Webcam) | Strobe Following & Obstacle Avoidance |
| :--------------------------------------------------------- | :------------------------------------------------------------------- |
| ![Hand Gesture Control](./assets/Drone%20Hand%20Gesture%20Ros.gif) | ![Strobe Following](./assets/Drone%20Following%20Strobe%20Node%20Ros.gif) |

* **On the left (Simulated 2D):** The "Hand Gesture Recognition" node processes a live video feed, using an AI model (MediaPipe) to classify hand poses into high-level flight commands. **(Note: This demonstration uses a standard 2D webcam as a stand-in for the final system's infrared sensor).**

* **On the right:** The drone autonomously follows a moving target (the magenta sphere).

---

## Core Features & Capabilities

* **Real-Time Hand Gesture Control (Upgraded to 3D Perception & Robustness):** The drone is controlled by high-level commands generated from an AI-powered perception node. The system uses a debouncing algorithm, requiring a gesture to be held for several frames before a command is confirmed, preventing accidental actions. Implemented gestures include:
    * **Open Palm (`MOVE_FORWARD`):** Commands the drone to move forward in its current direction. The detection of an open palm (5 fingers extended) is now more lenient and accurate, reliably recognizing all five extended fingers from multiple angles. This functionality incorporates true 3D directional pointing using the Intel RealSense D435i's depth data. The system tracks the 3D position of the hand and derives a smoothed 3D pointing vector. Drone movement is currently constrained to the world X-Z plane (forward/backward and up/down relative to the camera's orientation), preventing lateral movement along the world Y-axis.
    * **Fist (`LAND`):** Commands the drone to initiate landing. The detection logic for a fist has been significantly refined to be more robust across various hand orientations (e.g., upwards from front or back). It now uses advanced landmark analysis to accurately identify a fully curled fist (0 fingers extended), even with slight variations in finger positioning.
    * **No Hand / No Command (`HOVER`):** When no recognized hand gesture is detected, or for ambiguous states, the drone is commanded to stop its current task and hold its position.

* **Dynamic Obstacle Avoidance via Potential Fields:** The drone uses a custom potential field algorithm for navigation. This method calculates a real-time trajectory based on two competing forces:
    * **Attractive Force:** A vector pulling the drone towards its current goal (e.g., a waypoint or a moving target).
    * **Repulsive Force:** A vector pushing the drone away from any nearby obstacles. This force is generated using real-time `PointCloud2` data from the depth sensor, allowing the drone to smoothly maneuver around objects without a pre-planned path.

* **Multi-State Control System:** A state machine manages the drone's behavior, allowing it to seamlessly arbitrate between different modes, such as following a target, executing a manual command, or initiating a "lost target" search pattern.

## System Architecture

This project uses a **Remote Computing (or "Offboard Computing")** architecture to maximize the drone's agility and flight time while enabling powerful AI processing.

1.  **On-Drone Hardware:** A lightweight **NVIDIA Orin Nano Developer Kit** acts as the on-board compute platform. Its purpose is to perform all real-time sensor processing, perception, navigation, and control directly on the drone, enabling **GPS-denied autonomous flight**.
2.  **Ground Station:** A powerful ground computer (Ubuntu VM) currently serves as a development and visualization platform. While the ultimate goal is on-board autonomy, the ground station is used for initial testing, debugging, and visualization (e.g., RViz).

## Software Node Breakdown

This system is composed of several custom ROS 2 nodes that work in concert.

* **`current_fly_script.py`:** The central command node. It subscribes to all sensor and command inputs and is responsible for making high-level decisions. It implements the core Potential Field algorithm, using the simplified threat data from the perception node to calculate repulsive forces. It now also subscribes to `/hand_commands` to directly control the drone based on gestures, incorporating the world Y-axis movement restriction and minimum altitude safety.
* **`hand_gesture_recognition_node.py`:** This node has been significantly upgraded. It directly interfaces with the Intel RealSense D435i camera using `pyrealsense2`. It performs the following critical functions:
    * Acquires synchronized color, depth, and IMU frames from the RealSense.
    * Applies a decimation filter to the depth data for performance optimization.
    * Uses OpenCV and MediaPipe to detect hand landmarks on the color image.
    * Projects 2D hand landmarks to 3D coordinates using depth data and camera intrinsics.
    * **Improved Finger Extension Detection:** Employs a robust method using dot products between finger segments to accurately determine if each finger (including the thumb) is extended or curled, making gesture recognition more reliable across various hand angles.
    * Calculates a 3D pointing vector from the index finger (when applicable).
    * Publishes detected hand commands (`String`), 3D hand positions (`geometry_msgs/PointStamped`), and 3D pointing vectors (`geometry_msgs/Vector3Stamped`).
    * Publishes `sensor_msgs/PointCloud2` data (`/camera/camera/depth/color/points`) and `sensor_msgs/Imu` data (`/camera/camera/imu`) directly from the RealSense camera.
    * Publishes static TF transforms for the camera's internal frames (`camera_link` to `camera_depth_optical_frame`, `camera_depth_optical_frame` to `camera_color_optical_frame`, `camera_depth_optical_frame` to `camera_imu_optical_frame`).
    * Displays a live, annotated video feed using `cv2.imshow()` for real-time visual feedback, including a visible green pointing arrow and readable angle/direction text.
* **`obstacle_perception_node.py`:** This node subscribes to the real-time `PointCloud2` data published by `hand_gesture_recognition_node.py` (`/camera/camera/depth/color/points`). It processes this complex 3D data to find the most immediate threat to the drone and publishes the stable 3D coordinates of that threat to the `/detected_obstacle` topic.
* **`mock_px4.py`:** In the simulation, this node mimics a real PX4 flight controller. It receives `TrajectorySetpoint` messages and publishes the drone's changing position and status. This is replaced by the real Cube Orange+ in the hardware phase.
* **`px4_odometry_to_tf_publisher.py`:** This node subscribes to the drone's mock odometry data (`/fmu/out/vehicle_odometry`) and publishes the dynamic `odom` to `base_link` TF transform, allowing RViz to visualize the drone's movement.
* **`static_transform_publisher`:** A ROS 2 utility used in the `run_all.sh` script to publish the static transform from `base_link` (drone body) to `camera_link` (camera base frame).
* **`strobe_light_publisher.py`:** This node simulates the moving target that the drone is tasked with following. It publishes the 3D position of the strobe light.
* **`simulated_depth_sensor_publisher.py`:** This node can be used to create virtual maze environments by publishing simulated `PointCloud2` data. (Note: Currently configured for testing purposes, but the primary real-time depth data comes from the Intel RealSense D435i via `hand_gesture_recognition_node.py`).
## How to Run (RealSense Integrated Simulation)

The simulation is launched using the `run_all.sh` script, which automates the setup of the entire environment.

1.  **Ensure RealSense D435i is Connected:** Plug your Intel RealSense D435i camera into a USB 3.x port on your host machine. In VMware Workstation Pro, ensure it is connected to the Ubuntu VM (VM > Removable Devices > Intel(R) RealSense(TM) D435i > Connect (Disconnect from Host)).

2.  **Navigate to Project Directory:** Open a terminal in your Ubuntu VM and go to your project folder:

    ```bash
    cd ~/drone_project
    ```

3.  **Make `run_all.sh` Executable:** (Only needs to be done once)

    ```bash
    chmod +x run_all.sh
    ```

4.  **Run the Simulation:**

    ```bash
    ./run_all.sh
    ```

    This script will open multiple terminal windows, each running a specific ROS 2 node. A separate OpenCV window will also appear, displaying the live annotated camera feed.

## RViz2 Configuration (Terminal 3: RViz2)

Once RViz2 launches, configure its displays to visualize the system:

* **Global Options:**
    * **Fixed Frame:** Set to `odom`. This is the world coordinate frame.

* **Add Displays:**
    * **TF:** To visualize the coordinate frames (`odom`, `base_link`, `camera_link`, `camera_depth_optical_frame`, `camera_color_optical_frame`, `camera_imu_optical_frame`). All links in the chain should show "Transform OK".
    * **PointCloud2:**
        * **Topic:** `/camera/camera/depth/color/points`
        * **Color Transformer:** `RGB8` (for colorized point cloud).
        * **Style:** `Points`, **Size (Pixels):** `2` or `3` (adjust for visibility).
    * **Imu:**
        * **Topic:** `/camera/camera/imu`
    * **Marker:**
        * **Topic:** `/hand_3d_marker` (Green sphere marking detected hand position).
        * **Topic:** `/hand_pointing_arrow` (Orange arrow showing 3D pointing direction - *if implemented*).
        * **Topic:** `/perceived_obstacle_marker` (Red sphere marking detected obstacles).
        * **Topic:** `/obstacle_markers` (Orange/Cyan cylinders for simulated maze walls/obstacles - only if `simulated_depth_sensor.py` is re-enabled for testing purposes).
    * **PoseStamped:**
        * **Topic:** `/strobe_light_position` (Magenta sphere for the target).

## Current Status & Next Steps

The project is currently in the **Hardware-in-the-Loop (HITL) validation phase**, with a focus on integrating real-time sensor data and refining perception.

**Current Working State:**

* **Stable ROS2 Environment:** All core ROS2 components are installed and running reliably on the Ubuntu VM.

* **RealSense D435i Integration:** The camera is successfully detected and controlled directly by `hand_gesture_recognition_node.py`.

* **Real-time Sensor Data Publishing:** `hand_gesture_recognition_node.py` now publishes:
    * Decimated `PointCloud2` data (`/camera/camera/depth/color/points`).
    * Combined IMU data (`/camera/camera/imu`).
    * Live annotated video feed via `cv2.imshow()`.

* **3D Hand Tracking:** The system successfully detects hands in 2D, projects key landmarks to 3D space, and calculates a 3D pointing vector.

* **TF Tree Management:** The TF chain from `odom` to `base_link` and then to the camera's optical frames (`camera_link` -> `camera_depth_optical_frame` -> `camera_color_optical_frame` / `camera_imu_optical_frame`) is being published correctly.

* **RViz Visualization:** Point clouds, IMU data, 3D hand positions, and pointing arrows are now visualizable in RViz.

* **Core Drone Logic:** The mock PX4 and drone brain (`current_fly_script.py`) are operational, now responding to hand gesture commands (`HOVER`, `MOVE_FORWARD`, `LAND`).

**Immediate Next Steps (Current Focus - Refinement):**

* **Refine 3D Pointing Control:**
    * Further integrate the 3D pointing vector into `current_fly_script.py` to command drone movement with more precision.
    * Implement debouncing and smoothing for 3D pointing commands to ensure stable drone control.

* **Advanced Obstacle Avoidance:**
    * Further refine the `obstacle_perception_node.py` to classify and track multiple obstacles.
    * Enhance the Potential Field algorithm in `current_fly_script.py` to handle more complex obstacle geometries or dynamic obstacles.

**Future Development Phases:**

* **Hardware Integration (Phase 2b):**
    * **Control Pipeline Validation:** Integrate the CubePilot Cube Orange+ flight controller with the **NVIDIA Orin Nano Developer Kit**.
    * **Perception Pipeline Validation:** Stream real-time `PointCloud2` and infrared data from the Intel RealSense D435i to the **NVIDIA Orin Nano Developer Kit** for on-board processing.
    * **Physical Flight Testing:** Integrate all validated electronics onto a 5-inch drone frame for physical flight testing in a controlled environment, focusing on **GPS-denied autonomous flight capabilities**.
