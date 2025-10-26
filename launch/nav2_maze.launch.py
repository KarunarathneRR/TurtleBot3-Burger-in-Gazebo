from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    my_share = get_package_share_directory('my_simulations')
    nav2_share = get_package_share_directory('turtlebot3_navigation2')
    map_yaml  = os.path.join(my_share, 'maps', 'map.yaml')
    params    = os.path.join(my_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation2.launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'map': map_yaml,
                'params_file': params
            }.items()
        )
    ])
