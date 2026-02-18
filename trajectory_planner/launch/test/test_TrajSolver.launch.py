from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("trajectory_planner")

    params = [
        PathJoinSubstitution([pkg_share, "cfg", "test", "waypoint.yaml"]),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            Node(
                package="trajectory_planner",
                executable="testTrajSolver",
                name="test_solver",
                output="screen",
                parameters=params,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution([pkg_share, "rviz", "testSolver.rviz"]),
                ],
            ),
        ]
    )
