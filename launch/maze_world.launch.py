#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, TextSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    
    world = PathJoinSubstitution([
        FindPackageShare('my_simulations'), 'worlds', 'maze_world.world'
    ])

    my_models  = PathJoinSubstitution([FindPackageShare('my_simulations'), 'models'])
    tb3_models = PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'models'])
    
    # Set TURTLEBOT3_MODEL environment variable (CRITICAL!)
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )
    
    set_paths = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=TextSubstitution(text='')),
            TextSubstitution(text=':'), my_models,
            TextSubstitution(text=':'), tb3_models
        ]
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': world}.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'launch', 'robot_state_publisher.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py'])
        ]),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )

    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        set_tb3_model,  # THIS IS THE KEY FIX
        set_paths,
        gz,
        rsp,
        spawn_tb3
    ])