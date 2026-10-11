import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lidar_robot')
    xacro_file = os.path.join(pkg, 'urdf', 'lidar_robot.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'display.rviz')

    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args,
        ),
    ])