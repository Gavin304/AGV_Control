from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='js2fork', 
            executable='js2fork',  
            name='js2fork_node',
            output='screen'
        ),
    ])
