from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")

    agipix_root = Path(__file__).resolve().parents[2]
    mosek_x86_lib = str(agipix_root / "time_optimizer" / "include" / "time_optimizer" / "third_party" / "lib" / "x86")
    mosek_arm_lib = str(agipix_root / "time_optimizer" / "include" / "time_optimizer" / "third_party" / "lib" / "ARM")
    osqp_x86_lib = str(agipix_root / "trajectory_planner" / "include" / "trajectory_planner" / "third_party" / "lib" / "x86")
    osqp_arm_lib = str(agipix_root / "trajectory_planner" / "include" / "trajectory_planner" / "third_party" / "lib" / "ARM")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "rl_navigation", "flight_base.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "rl_navigation", "mapping_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "rl_navigation", "dynamic_detector_param.yaml"]),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=[
                    mosek_x86_lib,
                    ":",
                    mosek_arm_lib,
                    ":",
                    osqp_x86_lib,
                    ":",
                    osqp_arm_lib,
                    ":",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            Node(
                package="autonomous_flight",
                executable="rl_navigation_node",
                name="rl_navigation_node",
                output="screen",
                parameters=af_params
                + [{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="autonomous_flight",
                executable="mission_tracking_executor_node",
                name="mission_tracking_executor_node",
                output="screen",
                parameters=af_params
                + [
                    {"builtin_profile": "external_target"},
                    {"target_topic": "/autonomous_flight/target_state"},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
