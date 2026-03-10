import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz_node(toggle_name: str, node_name: str, config_name: str):
    config_path = os.path.join(
        get_package_share_directory('agi_viz'),
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
    urdf_path = os.path.join(
        get_package_share_directory('agi_viz'),
        'assets',
        'agipix.urdf',
    )
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock for all RViz instances.',
        ),
        DeclareLaunchArgument(
            'publish_agipix_urdf',
            default_value='true',
            description='Publish agipix URDF for RobotModel (root link: drone0/base_link).',
        ),
        DeclareLaunchArgument(
            'launch_all',
            default_value='false',
            description='Launch all configured RViz layouts.',
        ),
    ]

    rviz_targets = [
        ('static', 'rviz_static', 'static.rviz'),
        ('dynamic', 'rviz_dynamic', 'dynamic.rviz'),
        ('explore', 'rviz_explore', 'explore.rviz'),
        ('climb', 'rviz_climb', 'climb.rviz'),
        ('autonomous_flight_dynamic', 'rviz_autonomous_flight_dynamic', 'autonomous_flight_dynamic.rviz'),
        ('global_planner_map', 'rviz_global_planner_map', 'global_planner_map.rviz'),
        ('global_planner_navigation', 'rviz_global_planner_navigation', 'global_planner_navigation.rviz'),
        ('global_planner_rrt_interactive', 'rviz_global_planner_rrt_interactive', 'global_planner_rrt_interactive.rviz'),
        ('map_manager_map', 'rviz_map_manager_map', 'map_manager_map.rviz'),
        ('onboard_detector_detector', 'rviz_onboard_detector', 'onboard_detector_detector.rviz'),
        ('trajectory_planner_bspline_interactive', 'rviz_trajectory_planner_bspline_interactive', 'trajectory_planner_bspline_interactive.rviz'),
        ('trajectory_planner_poly_interactive', 'rviz_trajectory_planner_poly_interactive', 'trajectory_planner_poly_interactive.rviz'),
        ('trajectory_planner_test_solver', 'rviz_trajectory_planner_test_solver', 'trajectory_planner_testSolver.rviz'),
    ]

    nodes = []
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='agipix_robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': robot_description,
                }
            ],
            condition=IfCondition(LaunchConfiguration('publish_agipix_urdf')),
        )
    )

    for toggle_suffix, node_name, config_name in rviz_targets:
        toggle_name = f'launch_{toggle_suffix}'
        launch_args.append(
            DeclareLaunchArgument(
                toggle_name,
                default_value=LaunchConfiguration('launch_all'),
                description=f'Launch RViz config agi_viz/rviz/{config_name}.',
            )
        )
        nodes.append(_rviz_node(toggle_name, node_name, config_name))

    return LaunchDescription(launch_args + nodes)
