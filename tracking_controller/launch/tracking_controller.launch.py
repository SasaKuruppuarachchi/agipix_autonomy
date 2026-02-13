from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tracking_controller")
    controller_params = PathJoinSubstitution([pkg_share, "cfg", "controller_param.yaml"])

    return LaunchDescription(
        [
            Node(
                package="tracking_controller",
                executable="tracking_controller_node",
                name="tracking_controller_node",
                output="screen",
                parameters=[controller_params],
            ),
        ]
    )
