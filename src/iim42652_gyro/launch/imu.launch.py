from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('iim42652_gyro')) / 'config' / 'imu.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        Node(package='iim42652_gyro', executable='imu_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])
