import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('lidar_robot')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(pkg, 'maps', 'my_map_v1.yaml')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
    ])