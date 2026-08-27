from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('humanoid_driver_runtime'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(share / 'config' / 'mock_driver.yaml')),
        Node(
            package='humanoid_driver_runtime',
            executable='humanoid_driver_runtime_node',
            name='humanoid_driver_runtime',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
