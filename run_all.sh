#!/bin/bash

# This script launches the entire simulation and control system.
# It now includes a terminal that automatically connects to the Raspberry Pi
# to start the video stream.

# Terminal 1: Mock PX4 Interface (The Simulated Drone)
gnome-terminal --tab --title="Mock PX4" -- bash -c "source ~/.bashrc && python3 ~/mock_px4_interface.py; exec bash"

# Terminal 2: VIDEO STREAM FROM RASPBERRY PI
# This command automatically SSH's into your Pi and starts the camera node.
# It assumes your Pi's hostname is 'drone-pi.local' and your username is 'brannon'.
gnome-terminal --tab --title="PI_CAM_STREAM" -- bash -c "ssh brannon@drone-pi.local 'source /opt/ros/humble/setup.bash && ros2 run v4l2_camera v4l2_camera_node'; exec bash"

# Terminal 3: RViz2 (Visualization)
gnome-terminal --tab --title="RViz2" -- bash -c "source ~/.bashrc && rviz2; exec bash"

# Terminal 4: Obstacle Perception Node (The Drone's Eyes)
# This node needs to be created in your drone_autonomy package
# gnome-terminal --tab --title="Obstacle Perception" -- bash -c "cd ~/px4_ros_ws && source install/setup.bash && ros2 run drone_autonomy obstacle_perception_node; exec bash"

# Terminal 5: Drone Brain (The Drone's High-Level Logic)
gnome-terminal --tab --title="Drone Brain" -- bash -c "source ~/.bashrc && python3 ~/fly_up_land.py; exec bash"

# Terminal 6: Odometry to TF Publisher
gnome-terminal --tab --title="Odometry TF" -- bash -c "source ~/.bashrc && python3 ~/px4_odometry_to_tf_publisher.py; exec bash"

# The Strobe Light and old Keyboard publisher are commented out for this test
# Terminal 7: strobe_light_publisher.py
# gnome-terminal --tab --title="Strobe Light Publisher" -- bash -c "source ~/.bashrc && python3 ~/strobe_light_publisher.py; exec bash"
# Terminal 8: interactive_hand_command_publisher.py
# gnome-terminal --tab --title="Interactive Hand Command" -- bash -c "cd ~/px4_ros_ws && source install/setup.bash && ros2 run drone_autonomy hand_gestures; exec bash"

