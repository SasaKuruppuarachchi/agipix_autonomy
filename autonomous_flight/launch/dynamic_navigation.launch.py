from launch import LaunchDescription
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
            Node(
                package="tracking_controller",
                executable="tracking_controller_node",
                name="tracking_controller_node",
                output="screen",
                parameters=[controller_params],
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
                parameters=af_params,
            ),
        ]
    )
