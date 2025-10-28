from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set TurtleBot3 model (required)
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )
    
    my_share = get_package_share_directory('my_simulations')
    nav2_share = get_package_share_directory('turtlebot3_navigation2')
    import os as _os
    this_dir = _os.path.dirname(__file__)
    map_yaml  = _os.path.join(_os.path.dirname(this_dir), 'maps', 'map.yaml')
    params    = _os.path.join(my_share, 'config', 'nav2_params.yaml')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    return LaunchDescription([
        set_tb3_model,
        declare_use_sim_time,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_os.path.join(this_dir, 'nav2_bringup_min.launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'map': map_yaml,
                'params_file': params
            }.items()
        )
    ])
