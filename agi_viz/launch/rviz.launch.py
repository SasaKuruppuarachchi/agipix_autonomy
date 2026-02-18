import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz_node(toggle_name: str, node_name: str, package_name: str, config_name: str):
    config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        config_name,
    )

    return Node(
        package='rviz2',
        executable='rviz2',
        name=node_name,
        output='screen',
        arguments=['-d', config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration(toggle_name)),
    )


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock for all RViz instances.',
        ),
        DeclareLaunchArgument(
            'launch_all',
            default_value='true',
            description='Launch all configured RViz layouts.',
        ),
    ]

    rviz_targets = [
        ('autonomous_flight_dynamic', 'rviz_autonomous_flight_dynamic', 'autonomous_flight', 'dynamic.rviz'),
        ('global_planner_map', 'rviz_global_planner_map', 'global_planner', 'map.rviz'),
        ('global_planner_navigation', 'rviz_global_planner_navigation', 'global_planner', 'navigation.rviz'),
        ('global_planner_rrt_interactive', 'rviz_global_planner_rrt_interactive', 'global_planner', 'rrt_interactive.rviz'),
        ('map_manager_map', 'rviz_map_manager_map', 'map_manager', 'map.rviz'),
        ('onboard_detector_detector', 'rviz_onboard_detector', 'onboard_detector', 'detector.rviz'),
        ('trajectory_planner_bspline_interactive', 'rviz_trajectory_planner_bspline_interactive', 'trajectory_planner', 'bspline_interactive.rviz'),
        ('trajectory_planner_poly_interactive', 'rviz_trajectory_planner_poly_interactive', 'trajectory_planner', 'poly_interactive.rviz'),
        ('trajectory_planner_test_solver', 'rviz_trajectory_planner_test_solver', 'trajectory_planner', 'testSolver.rviz'),
    ]

    nodes = []
    for toggle_suffix, node_name, package_name, config_name in rviz_targets:
        toggle_name = f'launch_{toggle_suffix}'
        launch_args.append(
            DeclareLaunchArgument(
                toggle_name,
                default_value=LaunchConfiguration('launch_all'),
                description=f'Launch RViz config {package_name}/rviz/{config_name}.',
            )
        )
        nodes.append(_rviz_node(toggle_name, node_name, package_name, config_name))

    return LaunchDescription(launch_args + nodes)
