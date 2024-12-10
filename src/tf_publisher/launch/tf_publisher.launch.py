import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='tf_publisher',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'),
                'source_frame': LaunchConfiguration('source_frame')
            }]
        )
    ])
