from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ----------------------------
        # Touch 控制节点（clutch + 位姿输出）
        # ----------------------------
        Node(
            package='omni_control',
            executable='touch_hybrid_pose_control',
            name='touch_hybrid_pose_control',
            output='screen'
        ),

        # ----------------------------
        # Touch 末端位姿可视化（红色球）
        # ----------------------------
        Node(
            package='omni_control',
            executable='touch_control_visualizer',
            name='touch_control_visualizer',
            output='screen'
        ),
    ])
