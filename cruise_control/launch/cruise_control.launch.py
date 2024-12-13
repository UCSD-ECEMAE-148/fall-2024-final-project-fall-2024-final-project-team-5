from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cruise_control',
            executable='dist_publisher',
            output='screen'),
    ])
