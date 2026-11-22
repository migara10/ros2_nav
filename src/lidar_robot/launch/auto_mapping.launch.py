import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lidar_robot')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    foxglove_pkg = get_package_share_directory('foxglove_bridge')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # hardware, IMU, EKF (no RViz on the robot)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'bringup.launch.py')),
        launch_arguments={'use_rviz': 'false'}.items(),
    )

    # SLAM Toolbox builds the map and publishes map -> odom
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')),
    )

    # Nav2 navigation servers only (no AMCL and no map_server, SLAM provides /map)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    # the node that chooses frontiers and sends goals to Nav2
    explorer = Node(
        package='lidar_robot',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(foxglove_pkg, 'launch', 'foxglove_bridge_launch.xml')
        ),
        launch_arguments={'port': '8765'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='false = start SLAM and Nav2 only, so goals can be sent by hand'),
        bringup,
        # wait for the IMU calibration and the Arduino connection first
        TimerAction(period=10.0, actions=[slam]),
        TimerAction(period=16.0, actions=[navigation]),
        TimerAction(period=22.0, actions=[foxglove]),
        # start exploring only after Nav2 is active
        TimerAction(period=50.0, actions=[explorer]),
    ])