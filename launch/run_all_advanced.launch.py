import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts the complete project, combining the original
    hand gesture control with the new advanced obstacle avoidance system,
    now adapted for ArduPilot.
    """
    # Get the directory of the current launch file
    # This assumes the Python scripts are in the same package as the launch file
    # and that the package is correctly installed.
    # In a colcon workspace, 'drone_project' will be found by ros2 pkg prefix.
    project_dir = os.path.expanduser('~/ros2_ws/src/drone_project/drone_project')

    # --- Core Nodes (now ArduPilot compatible) ---

    # Replaced mock_px4_node with mock_ardupilot_node
    mock_ardupilot_node = ExecuteProcess(
        cmd=['python3', os.path.join(project_dir, 'mock_ardupilot.py')],
        name='mock_ardupilot_node',
        output='screen'
    )

    obstacle_perception_node = ExecuteProcess(
        cmd=['python3', os.path.join(project_dir, 'obstacle_perception_node.py')],
        name='obstacle_perception_node',
        output='screen'
    )

    drone_commander_node = ExecuteProcess(
        cmd=['python3', os.path.join(project_dir, 'current_fly_script.py')],
        name='drone_commander_node',
        output='screen'
    )

    # Replaced px4_odometry_to_tf_publisher_node with ardupilot_odometry_to_tf_publisher_node
    ardupilot_odometry_tf_publisher_node = ExecuteProcess(
        cmd=['python3', os.path.join(project_dir, 'ardupilot_odometry_to_tf_publisher.py')],
        name='ardupilot_odometry_to_tf_publisher_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    # --- Sensor and Control Nodes ---

    # This node provides hand commands, the camera's TF frames, and the real point cloud.
    hand_gesture_node = ExecuteProcess(
        cmd=['python3', os.path.join(project_dir, 'hand_gesture_recognition_node.py')],
        name='hand_gesture_node',
        output='screen'
    )

    # This node provides the simulated pillars for the obstacle course.
    # For the hybrid test, both this and the hand gesture node are active.
    simulated_depth_sensor_node = ExecuteProcess(
       cmd=['python3', os.path.join(project_dir, 'simulated_depth_sensor.py')],
       name='simulated_depth_sensor_node',
       output='screen'
    )

    # Optional: Strobe light node for following scenarios
    strobe_light_node = ExecuteProcess(
        cmd=['python3', os.path.join(project_dir, 'strobe_light.py')],
        name='strobe_light_node',
        output='screen'
    )

    # --- Static TF for the drone body to camera mount ---
    # This is still needed because hand_gesture_recognition_node only defines the
    # camera's *internal* transforms.
    tf_cam_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_cam_mount',
        arguments=['0.1', '0', '0', '0', '0', '0', '1', 'base_link', 'camera_link']
    )


    return LaunchDescription([
        mock_ardupilot_node, # Updated node name
        obstacle_perception_node,
        drone_commander_node,
        ardupilot_odometry_tf_publisher_node, # Updated node name
        rviz_node,
        tf_cam_mount,
        hand_gesture_node,
        simulated_depth_sensor_node,
        strobe_light_node, # Include strobe light node
    ])

