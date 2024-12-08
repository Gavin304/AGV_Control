from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forklift',  # This is the package where the forklift node is located
            executable='forklift',  # The executable inside the forklift package
            name='forklift_node',
            output='screen'
        ),
    ])
