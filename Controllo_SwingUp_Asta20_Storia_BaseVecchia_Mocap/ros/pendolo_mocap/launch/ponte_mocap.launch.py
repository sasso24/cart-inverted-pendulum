"""ros2 launch pendolo_mocap ponte_mocap.launch.py [parametri:=/percorso/altro.yaml]"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    predefinito = os.path.join(get_package_share_directory('pendolo_mocap'), 'config', 'ponte_mocap.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('parametri', default_value=predefinito),
        Node(package='pendolo_mocap', executable='ponte_mocap', name='ponte_mocap',
             parameters=[LaunchConfiguration('parametri')], output='screen'),
    ])
