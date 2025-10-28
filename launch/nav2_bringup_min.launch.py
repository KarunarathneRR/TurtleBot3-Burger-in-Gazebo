#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_map = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        description='Full path to Nav2 params yaml'
    )

    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        emulate_tty=True,
        composable_node_descriptions=[
            # Map server
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml}, params_file]
            ),
            # AMCL localization
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
            # Planner server
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
            # Controller server
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
            # Smoother server
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
            # Velocity smoother
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
            # Collision monitor
            ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
            # Behavior server
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
            # BT Navigator
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
            # Waypoint follower
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[{'use_sim_time': use_sim_time}, params_file]
            ),
        ]
    )

    # Lifecycle managers (do NOT include route_server)
    lifecycle_mgr_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    lifecycle_mgr_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'controller_server', 'planner_server', 'bt_navigator', 'waypoint_follower',
                'smoother_server', 'velocity_smoother', 'collision_monitor', 'behavior_server'
            ]}
        ]
    )

    return LaunchDescription([
        declare_use_sim,
        declare_map,
        declare_params,
        container,
        lifecycle_mgr_localization,
        lifecycle_mgr_navigation,
    ])
