from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "dynamic_inspection", "flight_base.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_inspection", "inspection_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_inspection", "planner_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_inspection", "mapping_param.yaml"]),
        PathJoinSubstitution([af_share, "cfg", "dynamic_inspection", "dynamic_detector_param.yaml"])

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
            #ExecuteProcess(cmd=["bash", throttle_script], output="screen"),
            Node(
                package="autonomous_flight",
                executable="dynamic_inspection_node",
                name="dynamic_inspection_node",
                output="screen",
                parameters=af_params + [
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
