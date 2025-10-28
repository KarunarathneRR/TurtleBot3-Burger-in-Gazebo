#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='~/.tb3_dqn.pt',
            description='Path to the trained DQN model'
        ),
        Node(
            package='tb3_dqn_nav',
            executable='dqn_run',
            name='dqn_run',
            parameters=[{'model_path': LaunchConfiguration('model_path')}]
        )
    ])