from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")
    tc_share = FindPackageShare("tracking_controller")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "dynamic_exploration", "flight_base.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_exploration", "exploration_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_exploration", "planner_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_exploration", "mapping_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_exploration", "dynamic_detector_param.yaml"])
    ]

    
    controller_params = PathJoinSubstitution([tc_share, "cfg", "controller_param.yaml"])

    throttle_script = PathJoinSubstitution([af_share, "scripts", "throttle_topics.sh"])

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
            #ExecuteProcess(cmd=["bash", throttle_script], output="screen"),
            Node(
                package="autonomous_flight",
                executable="dynamic_exploration_node",
                name="dynamic_exploration_node",
                output="screen",
                parameters=af_params,
            ),
        ]
    )
