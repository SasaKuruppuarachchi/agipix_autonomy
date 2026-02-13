from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("trajectory_planner")
    map_location = LaunchConfiguration("map_location")

    params = [
        PathJoinSubstitution([pkg_share, "cfg", "planner_interactive.yaml"]),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_location",
                default_value=PathJoinSubstitution([pkg_share, "map", "box.bt"]),
            ),
            SetParameter(name="use_sim_time", value=False),
            Node(
                package="trajectory_planner",
                executable="testBspline",
                name="test_bspline_node",
                output="screen",
                parameters=params,
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server_node",
                output="log",
                arguments=[map_location],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution([pkg_share, "rviz", "bspline_interactive.rviz"]),
                ],
            ),
        ]
    )
