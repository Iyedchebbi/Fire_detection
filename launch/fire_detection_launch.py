from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fire_detection',
            executable='fire_detection_node',
            name='fire_detection_node',
            output='screen'
        )
    ])

