from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare("trajectory_planner")
    map_location = LaunchConfiguration("map_location")

    params = [
        PathJoinSubstitution([pkg_share, "cfg", "planner_interactive.yaml"]),
    ]
    
    octomap_path = os.path.join(
        get_package_share_directory('global_planner'),
        'map',
        'maze.bt'
    )   
    

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_location",
                default_value=PathJoinSubstitution([pkg_share, "map", "box.bt"]),
            ),
            SetParameter(name="use_sim_time", value=False),
            Node(
                package="trajectory_planner",
                executable="poly_RRT_node",
                name="Poly_RRT_test_node",
                output="screen",
                parameters=params,
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server_node",
                output="log",
                parameters=[{'octomap_path': octomap_path}, {'frame_id': 'map'}]  # <--- Pass as param here
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution([pkg_share, "rviz", "poly_interactive.rviz"]),
                ],
            ),
        ]
    )
