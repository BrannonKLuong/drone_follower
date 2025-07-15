import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts the complete project, combining the original
    hand gesture control with the new advanced obstacle avoidance system.
    """
    project_dir = os.path.expanduser('~/drone_project')

    # --- Core Nodes ---
    
    mock_px4_node = ExecuteProcess(
        cmd=['python3', os.path.join(project_dir, 'mock_px4.py')],
        name='mock_px4_node',
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

    odometry_tf_publisher_node = ExecuteProcess(
        cmd=['python3', os.path.join(project_dir, 'px4_odometry_to_tf_publisher.py')],
        name='odometry_tf_publisher_node',
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
    # simulated_depth_sensor_node = ExecuteProcess(
    #    cmd=['python3', os.path.join(project_dir, 'simulated_depth_sensor.py')],
    #    name='simulated_depth_sensor_node',
    #    output='screen'
    # )
    
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
        mock_px4_node,
        obstacle_perception_node,
        drone_commander_node,
        odometry_tf_publisher_node,
        rviz_node,
        tf_cam_mount,
        hand_gesture_node,
        #simulated_depth_sensor_node,
    ])

