from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "takeoff_and_track_circle", "flight_base.yaml"]),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drone_namespace",
                default_value="drone0",
                description="Namespace applied to autonomous flight nodes.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            Node(
                package="autonomous_flight",
                executable="takeoff_and_track_circle_executor_node",
                name="takeoff_and_track_circle_executor_node",
                output="screen",
                parameters=af_params + [
                    {"builtin_profile": "circle"},
                    {"drone_namespace": LaunchConfiguration("drone_namespace")},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
