from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")
    tc_share = FindPackageShare("tracking_controller")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "navigation", "flight_base.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "navigation", "planner_param.yaml"]),
    ]

    map_params = PathJoinSubstitution([af_share, "cfg", "navigation", "mapping_param.yaml"])
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
            Node(
                package="map_manager",
                executable="occupancy_map_node",
                name="occupancy_map_node",
                output="screen",
                parameters=[map_params],
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
