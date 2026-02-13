from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "inspection", "inspection_param.yaml"]),
    ]

    return LaunchDescription(
        [
            Node(
                package="autonomous_flight",
                executable="inspection_node",
                name="inspection_node",
                output="screen",
                parameters=af_params,
            )
        ]
    )
