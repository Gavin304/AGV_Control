from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joystick_ros2',  # This is the package where the forklift node is located
            executable='joystick_ros2',  # The executable inside the forklift package
            name='joystick_node',
            output='screen'
        ),
    ])
