from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame',
            default_value='base_link',  # Provide a default value
            description='Target frame for the TF publisher'
        ),
        Node(
            package='tf_publisher',
            executable='tf_publisher_node',
            name='tf_publisher',
            parameters=[{'target_frame': LaunchConfiguration('target_frame')}],
        )
    ])
