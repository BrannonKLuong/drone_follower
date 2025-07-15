from setuptools import setup
import os
from glob import glob

package_name = 'drone_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Explicitly copy the specific launch file to the 'launch' subdirectory in the install space
        (os.path.join('share', package_name, 'launch'), [os.path.join('launch', 'run_all_advanced.launch.py')]),
        # Ensure the resource file is correctly installed into the package's share directory
        (os.path.join('share', package_name, 'resource'), [os.path.join('resource', package_name)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brannon',
    maintainer_email='brannon.k.luong@gmail.com',
    description='ROS 2 package for autonomous drone control and perception with ArduPilot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    include_package_data=True,
    entry_points={
        'console_scripts': [
            #'mock_ardupilot = drone_project.mock_ardupilot:main',
            'ardupilot_interface_node = drone_project.ardupilot_interface_node:main',
            'ardupilot_odometry_to_tf_publisher = drone_project.ardupilot_odometry_to_tf_publisher:main',
            'current_fly_script = drone_project.current_fly_script:main',
            'hand_gesture_recognition_node = drone_project.hand_gesture_recognition_node:main',
            'obstacle_perception_node = drone_project.obstacle_perception_node:main',
            'simulated_depth_sensor = drone_project.simulated_depth_sensor:main',
            'strobe_light = drone_project.strobe_light:main',
        ],
    },
)

