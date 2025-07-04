# Autonomous Tactical Scouting Drone

**Status:** `Simulation Phase Complete - Validated and Ready for Hardware Integration`

This repository contains the **ROS 2** software stack for a compact, autonomous drone designed for tactical indoor and close-quarters operations. The system uses an **offboard computing architecture** to enable a small, agile aerial platform to perform complex perception and navigation tasks, with real-time control via hand gestures from an operator.

---

## Live Demonstration

The core control loop and simulated perception have been successfully validated in a **software-in-the-loop (SITL)** environment.

### Hand Gesture Control (RealSense 3D Depth)

For a detailed view of the 3D hand tracking and gesture recognition with depth data, see the GIF below.
![Hand Gesture Recognition with Depth](./assets/Depth%20Directional%20Hand%20Commands.gif)

### Other Demonstrations

| Hand Gesture Control (Simulated 2D Webcam)                               | Drone Following Strobe Light Node                                  |
| :----------------------------------------------------------------------- | :----------------------------------------------------------------- |
| ![Hand Gesture Control](./assets/Drone%20Hand%20Gesture%20Ros.gif)       | ![Strobe Following](./assets/Drone%20Following%20Strobe%20Node%20Ros.gif) |

* **On the left (Simulated 2D):** The "**Hand Gesture Recognition**" node processes a live video feed, using an **AI model (MediaPipe)** to classify hand poses into high-level flight commands.
* **On the right:** The drone autonomously follows strobe light node.

---

## Core Features & Capabilities

* **Real-Time 3D Hand Gesture Control:** The drone is controlled by high-level commands generated from an **AI-powered perception node**. The system uses a **debouncing algorithm**, requiring a gesture to be held for several frames before a command is confirmed. Implemented gestures include:
    * **Open Palm (`MOVE_FORWARD`):** Commands the drone to move in the 3D direction pointed by the hand.
    * **Fist (`LAND`):** Commands the drone to initiate a safe landing sequence.
    * **No Hand (`HOVER`):** The drone holds its position if no command is detected.

* **Advanced Obstacle Avoidance via Potential Fields:** The drone uses a custom **potential field algorithm** for navigation. This method calculates a real-time trajectory based on two competing forces:
    * **Attractive Force:** A vector pulling the drone towards its current goal (e.g., a waypoint or the direction of a hand gesture).
    * **Repulsive Force:** A vector pushing the drone away from any nearby obstacles. This system has been upgraded to use a **Euclidean Clustering algorithm**, allowing it to perceive and react to multiple distinct obstacles simultaneously (e.g., navigating through a doorway or slalom course).

* **Dual-Mode Control System:** A state machine in the drone's "brain" (`current_fly_script.py`) manages its behavior, allowing it to operate in two primary modes:
    * **`HAND_CONTROL`:** The default mode, where the drone is controlled directly by the operator's hand gestures.
    * **`MISSION`:** An autonomous mode where the drone follows a pre-defined list of waypoints, using its advanced obstacle avoidance to navigate the environment.

---

## System Architecture

This project uses a **Remote Computing** (or "**Offboard Computing**") architecture to maximize the drone's agility and flight time while enabling powerful AI processing.

1.  **On-Drone Hardware:** A lightweight **NVIDIA Orin Nano Developer Kit** acts as the on-board compute platform. Its purpose is to perform all real-time sensor processing, perception, navigation, and control directly on the drone, enabling **GPS-denied autonomous flight**.
2.  **Ground Station:** A powerful ground computer (Ubuntu VM) currently serves as the development, simulation, and visualization platform.

---

## Software Node Breakdown

This system is composed of several custom ROS 2 nodes that work in concert.

* **`current_fly_script.py`:** The central command node or "**brain**." It subscribes to all sensor and command inputs, manages the control state (Hand vs. Mission), and implements the **Potential Field algorithm** using multi-obstacle data from the perception node to calculate a combined repulsive force.
* **`hand_gesture_recognition_node.py`:** This node has been significantly upgraded. It directly interfaces with the **Intel RealSense D435i camera** using `pyrealsense2`. It performs the following critical functions:
    * Acquires synchronized color, depth, and IMU frames from the RealSense.
    * Applies a decimation filter to the depth data for performance optimization.
    * Uses OpenCV and MediaPipe to detect hand landmarks on the color image.
    * Projects 2D hand landmarks to 3D coordinates using depth data and camera intrinsics.
    * **Improved Finger Extension Detection:** Employs a robust method using dot products between finger segments to accurately determine if each finger (including the thumb) is extended or curled, making gesture recognition more reliable across various hand angles.
    * Calculates a 3D pointing vector from the index finger (when applicable).
    * Publishes detected hand commands (`String`), 3D hand positions (`geometry_msgs/PointStamped`), and 3D pointing vectors (`geometry_msgs/Vector3Stamped`).
    * Publishes `sensor_msgs/PointCloud2` data (`/camera/camera/depth/color/points`) and `sensor_msgs/Imu` data (`/camera/camera/imu`) directly from the RealSense camera.
    * Publishes static TF transforms for the camera's internal frames (`camera_link` to `camera_depth_optical_frame`, `camera_depth_optical_frame` to `camera_color_optical_frame`, `camera_depth_optical_frame` to `camera_imu_optical_frame`).
    * Displays a live, annotated video feed using `cv2.imshow()` for real-time visual feedback.
* **`obstacle_perception_node.py`:** This node subscribes to the real-time `PointCloud2` data. It processes this complex 3D data using a **Euclidean Clustering algorithm** to group points into distinct objects. It then publishes the 3D centroids of all detected clusters to the `/detected_obstacles_centroids` topic for the drone's brain to use.
* **`mock_px4.py`:** In the simulation, this node mimics a real PX4 flight controller. It receives `TrajectorySetpoint` messages and publishes the drone's changing position and status. This will be replaced by the real **Cube Orange+** in the hardware phase.
* **`px4_odometry_to_tf_publisher.py`:** This node subscribes to the drone's mock odometry data (`/fmu/out/vehicle_odometry`) and publishes the dynamic `odom` to `base_link` TF transform, allowing RViz to visualize the drone's movement.
* **`strobe_light.py`:** A simulation node that publishes a moving target for the drone to follow. This is not used in the primary hand-control or waypoint mission modes but can be re-enabled for "follow-the-leader" style missions.
* **`simulated_depth_sensor.py`:** This node can be used to create virtual obstacle courses by publishing simulated `PointCloud2` data. It is essential for testing and validation but is disabled for real-world flight.

---

## How to Run (Final Software Version)

The system is now launched using a robust **Python launch file**, which is the standard for ROS 2. This replaces the less reliable `run_all.sh` script, which was prone to race conditions.

### 1. Configure the Flight Mode

Before launching, decide which mode you want to test. Open `current_fly_script.py` and set the `test_scenario_active` parameter:

* **For Hand Control:** `self.declare_parameter('test_scenario_active', False)`
* **For Autonomous Mission:** `self.declare_parameter('test_scenario_active', True)`

### 2. Launch the System

Open a single terminal, navigate to your project directory (`cd ~/drone_project`), and use the appropriate launch command.

#### For Real-World Flight (Hardware Ready)

This is the primary launch configuration. It uses the RealSense camera for both hand control and real-world obstacle perception.

* Ensure a file named `run_all_hardware_ready.launch.py` exists with the correct configuration.
* Connect your RealSense camera.
* Run the command:
    ```bash
    ros2 launch ./run_all_hardware_ready.launch.py
    ```

#### For Hybrid Testing (Real Hand Control + Simulated Obstacles)

This is the most comprehensive validation test. It uses your real hand for control while the drone navigates a simulated slalom course.

* Ensure a file named `hybrid_test.launch.py` exists with the correct configuration.
* Connect your RealSense camera.
* Run the command:
    ```bash
    ros2 launch ./hybrid_test.launch.py
    ```

---

## RViz2 Configuration

Once RViz2 launches, configure its displays to visualize the system:

* **Global Options:**
    * **Fixed Frame:** Set to `odom`.

* **Add Displays:**
    * **TF:** To visualize all coordinate frames.
    * **PointCloud2:**
        * **Topic:** `/camera/camera/depth/color/points`
    * **MarkerArray:**
        * **Topic:** `/perceived_obstacle_markers` (This will show the red spheres for each detected obstacle cluster).
    * **Marker:**
        * **Topic:** `/obstacle_markers` (This will show the blue pillars from the simulated environment).
        * **Topic:** `/hand_3d_marker` (Green sphere for hand position).

---

## Current Status & Next Steps

The project has successfully completed its **software-in-the-loop (SITL) validation phase**. The core logic for control, perception, and navigation has been implemented and tested.

**Immediate Next Steps:**

* **Hardware Integration:** The primary focus is now on transitioning the validated software stack to the physical hardware platform.
    * **On-Board Computer Setup:** Deploy the ROS 2 environment and your project files to the **NVIDIA Orin Nano**.
    * **Flight Controller Integration:** Replace the `mock_px4.py` node with a connection to the real **Cube Orange+** flight controller (e.g., using a Micro-ROS agent).
    * **Physical Assembly:** Mount all components (Orin, RealSense, Cube) onto the 5-inch drone frame.
    * **Physical Flight Testing:** Begin flight tests in a safe, controlled environment, starting with basic maneuvers and progressing to the full hand-controlled, obstacle-avoiding flight.
