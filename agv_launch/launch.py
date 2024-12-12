import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    forklift_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('forklift'), 'launch'),
            '/forklift_launch.py']),
    )
    js2fork_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('js2fork'), 'launch'),
            '/js2fork_launch.py']),
    )
    joystick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joystick_ros2'), 'launch'),
            '/joystick_ros2_launch.py']),
    )
    tf2_broadcaster_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tf_publisher'), 'launch'),
            '/tf2_broadcaster_launch.py']),
    )
    
    
    return LaunchDescription([
        forklift_node,
        js2fork_node,
        joystick_node,
        tf2_broadcaster_node,
    ])