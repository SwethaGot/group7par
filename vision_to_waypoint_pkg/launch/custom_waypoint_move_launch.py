from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_to_waypoint_pkg',
            executable='custom_waypoint_move_server',
            name='custom_waypoint_move_server',
            output='screen'
        )
    ])
