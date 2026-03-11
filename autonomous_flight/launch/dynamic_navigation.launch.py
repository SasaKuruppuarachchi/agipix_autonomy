from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "flight_base.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "planner_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "mapping_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "dynamic_detector_param.yaml"]),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drone_namespace",
                default_value="drone0",
                description="Namespace applied to autonomous flight nodes and mission topics.",
            ),
            DeclareLaunchArgument(
                "target_topic",
                default_value="autonomous_flight/target_state",
                description="Target topic consumed by mission_tracking_executor_node.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            Node(
                package="autonomous_flight",
                executable="dynamic_navigation_node",
                namespace=LaunchConfiguration("drone_namespace"),
                name="dynamic_navigation_node",
                output="screen",
                parameters=af_params + [
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
            Node(
                package="autonomous_flight",
                executable="mission_tracking_executor_node",
                name="mission_tracking_executor_node",
                output="screen",
                parameters=af_params + [
                    {"builtin_profile": "external_target"},
                    {"drone_namespace": LaunchConfiguration("drone_namespace")},
                    {"target_topic": LaunchConfiguration("target_topic")},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
