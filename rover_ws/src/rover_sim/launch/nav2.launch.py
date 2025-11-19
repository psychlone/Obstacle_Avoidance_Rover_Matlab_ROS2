from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rover = get_package_share_directory('rover_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_odom',
    output='screen',
    parameters=[os.path.join(pkg_rover, 'config', 'ekf.yaml')]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'map': os.path.join(pkg_rover, 'config', 'map.yaml'),
                'params_file': os.path.join(pkg_rover, 'config', 'nav2_params.yaml'),
            }.items()

        ),ekf_node
    ])
