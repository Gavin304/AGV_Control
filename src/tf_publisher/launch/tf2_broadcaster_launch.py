from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_publisher',
            executable='tf2_broadcaster', 
            name='tf2_broadcaster_node',
            output='screen'
        ),
    ])
