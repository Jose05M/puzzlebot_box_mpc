from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # Cámara
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                '/home/puzzlebot/ros2_packages_ws/src/ros_deep_learning/launch/video_source.ros2.launch'
            )
        ),
        # Odometría
        Node(
            package='puzzlebot_box_mpc',
            executable='puzzlebot_odometry',
            name='puzzlebot_odometry',
            output='screen'
        ),
        # MPC
        Node(
            package='puzzlebot_box_mpc',
            executable='mpc_hw',
            name='puzzlebot_box_mpc_hw_node',
            output='screen'
        ),
    ])
	
