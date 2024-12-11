from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},  # Use true if working with simulation
                {'map_file_name': ''},  # Leave empty for online SLAM
                {'odom_frame': 'odom'},  # Your robot's odom frame
                {'map_frame': 'map'},  # Global frame for localization
                {'base_frame': 'base_link'},  # Robot's base frame
                {'scan_topic': '/scan'},  # Laser scan topic
                {'use_scan_matching': True},  # Enable scan matching
                {'use_loop_closing': False},  # Disable loop closing for faster processing
            ],
        ),
        Node(
            package='tf_publisher',
            executable='tf2_broadcaster',
            name='tf_publisher',
            output='screen',
            parameters=[
                {'target_frame': 'base_link'},  # Target frame for the TF publisher
            ],
        )
    ])
