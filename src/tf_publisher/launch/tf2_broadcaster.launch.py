from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static TF2 broadcaster
        Node(
            package='tf_publisher',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_frame_tf2_broadcaster',
            output='screen'
        ),
        # Dynamic frame TF2 broadcaster
        Node(
            package='tf_publisher',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_frame_tf2_broadcaster',
            output='screen'
        ),
    ])
