from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='virtual_environment',
            executable='virtual_wall',
            name='virtual_wall',
            output='screen'
        ),
        Node(
            package='virtual_environment',
            executable='collision_checker',
            name='collision_checker',
            output='screen'
        )
    ])
