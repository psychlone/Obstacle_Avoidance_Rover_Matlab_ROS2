from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rover = get_package_share_directory('rover_sim')

    xacro_file = os.path.join(pkg_rover, 'urdf', 'rover.urdf.xacro')

    # Run xacro to generate robot_description
    robot_description = Command(['xacro ', xacro_file])

    # Launch robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Launch joint_state_publisher_gui to move joints interactively
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Launch RViz2 with robot model display
    rviz_config_path = os.path.join(pkg_rover, 'config', 'display.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        rviz_node
    ])
