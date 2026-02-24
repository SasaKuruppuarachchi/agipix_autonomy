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
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            # Node(
            #     package="map_manager",
            #     executable="dynamic_map_node",
            #     name="dynamic_map_node",
            #     output="screen",
            #     parameters=[map_params],
            # ),
            # Node(
            #     package="onboard_detector",
            #     executable="dynamic_detector_node",
            #     name="dynamic_detector_node",
            #     output="screen",
            #     parameters=[detector_params],
            # ),
            Node(
                package="autonomous_flight",
                executable="dynamic_navigation_node",
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
                    {"target_topic": "/autonomous_flight/target_state"},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
