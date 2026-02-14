from pathlib import Path

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")
    tc_share = FindPackageShare("tracking_controller")

    agipix_root = Path(__file__).resolve().parents[2]
    mosek_x86_lib = str(agipix_root / "time_optimizer" / "include" / "time_optimizer" / "third_party" / "lib" / "x86")
    mosek_arm_lib = str(agipix_root / "time_optimizer" / "include" / "time_optimizer" / "third_party" / "lib" / "ARM")
    osqp_x86_lib = str(agipix_root / "trajectory_planner" / "include" / "trajectory_planner" / "third_party" / "lib" / "x86")
    osqp_arm_lib = str(agipix_root / "trajectory_planner" / "include" / "trajectory_planner" / "third_party" / "lib" / "ARM")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "navigation", "flight_base.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "navigation", "planner_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "navigation", "mapping_param.yaml"]),
    ]
    controller_params = PathJoinSubstitution([tc_share, "cfg", "controller_param.yaml"])

    return LaunchDescription(
        [
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
                package="tracking_controller",
                executable="tracking_controller_node",
                name="tracking_controller_node",
                output="screen",
                parameters=[controller_params],
            ),
            Node(
                package="autonomous_flight",
                executable="navigation_node",
                name="navigation_node",
                output="screen",
                parameters=af_params,
            ),
        ]
    )
