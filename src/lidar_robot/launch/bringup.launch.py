import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARDUINO_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
LIDAR_PORT = ('/dev/serial/by-id/'
              'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')


def generate_launch_description():
    pkg = get_package_share_directory('lidar_robot')
    xacro_file = os.path.join(pkg, 'urdf', 'lidar_robot.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'display.rviz')

    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value=ARDUINO_PORT),
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        Node(
            package='lidar_robot',
            executable='arduino_bridge',
            parameters=[{
                'port': LaunchConfiguration('arduino_port'),
                'min_pwm': 130,
                'wheel_sep': 0.115,
                'wheel_radius': 0.03,
                'ticks_per_rev': 1403.0,
                'left_gain': 1.10,
            }],
            output='screen',
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'angle_compensate': True,
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args,
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])