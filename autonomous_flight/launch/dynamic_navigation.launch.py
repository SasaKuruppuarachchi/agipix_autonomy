from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")
    tc_share = FindPackageShare("tracking_controller")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "flight_base.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "planner_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "mapping_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "dynamic_detector_param.yaml"]),
    ]

    map_params = PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "mapping_param.yaml"])
    detector_params = PathJoinSubstitution([af_share, "cfg", "dynamic_navigation", "dynamic_detector_param.yaml"])

    controller_params = PathJoinSubstitution([tc_share, "cfg", "controller_param.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_backend",
                default_value="dds",
                description="tracking_controller backend (DDS-only)",
            ),
            DeclareLaunchArgument(
                "controller_dds_target_topic",
                default_value="/px4_control_interface/controller_target_state",
                description="DDS target topic used when controller_backend=dds",
            ),
               DeclareLaunchArgument(
                   "use_sim_time",
                   default_value="false",
                   description="Use simulation clock if true",
               ),
            Node(
                package="tracking_controller",
                executable="tracking_controller_node",
                name="tracking_controller_node",
                output="screen",
                parameters=[
                    controller_params,
                    {
                        "controller.backend": LaunchConfiguration("controller_backend"),
                        "controller.dds_target_topic": LaunchConfiguration("controller_dds_target_topic"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
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
                    {"controller.backend": LaunchConfiguration("controller_backend")},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
