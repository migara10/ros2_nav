import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('lidar_robot')
    slam_pkg = get_package_share_directory('slam_toolbox')
    params = os.path.join(pkg, 'config', 'mapper_params_online_async.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': params,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([slam])