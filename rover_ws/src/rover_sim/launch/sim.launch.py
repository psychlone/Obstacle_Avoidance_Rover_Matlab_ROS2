from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rover = get_package_share_directory('rover_sim')

    # Paths
    xacro_file = os.path.join(pkg_rover, 'urdf', 'rover.urdf.xacro')

    # Convert Xacro to URDF at runtime
    robot_description = Command(['xacro ', xacro_file])

    # Gazebo world file (optional)
    world_path = os.path.join(pkg_rover, 'worlds', 'obstacle_world.world')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )

    # Publish robot_state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rover', '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )

    static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    static_tf_odom_base = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )


    

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        static_tf,
        static_tf_odom_base
    ])
