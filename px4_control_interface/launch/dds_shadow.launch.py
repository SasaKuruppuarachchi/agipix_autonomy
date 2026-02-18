from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include_legacy_stack(context):
    mission = LaunchConfiguration("mission").perform(context)

    mission_to_launch = {
        "takeoff_and_hover": "takeoff_and_hover.launch.py",
        "takeoff_and_track_circle": "takeoff_and_track_circle.launch.py",
        "navigation": "navigation.launch.py",
        "dynamic_navigation": "dynamic_navigation.launch.py",
        "dynamic_inspection": "dynamic_inspection.launch.py",
        "dynamic_exploration": "dynamic_exploration.launch.py",
        "inspection": "inspection.launch.py",
    }

    if mission not in mission_to_launch:
        raise RuntimeError(
            f"Unsupported mission '{mission}'. Supported: {', '.join(mission_to_launch.keys())}"
        )

    launch_file = mission_to_launch[mission]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("autonomous_flight"), "launch", launch_file]
                )
            ),
            condition=IfCondition(LaunchConfiguration("start_legacy_stack")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission",
                default_value="navigation",
                description=(
                    "Legacy mission stack to include in shadow run. "
                    "One of: takeoff_and_hover, takeoff_and_track_circle, navigation, "
                    "dynamic_navigation, dynamic_inspection, dynamic_exploration, inspection"
                ),
            ),
            DeclareLaunchArgument(
                "start_legacy_stack",
                default_value="true",
                description="If true, include autonomous_flight legacy mission stack in parallel.",
            ),
            DeclareLaunchArgument(
                "target_topic",
                default_value="/autonomous_flight/target_state",
                description=(
                    "Target topic consumed by px4_tracking_mode_node. "
                    "Use /autonomous_flight/target_state for mission shadow input or "
                    "/px4_control_interface/controller_target_state for tracking_controller DDS sink output."
                ),
            ),
            DeclareLaunchArgument(
                "start_tracking_controller",
                default_value="false",
                description=(
                    "If true, include tracking_controller in this launch. "
                    "Keep false when start_legacy_stack=true because mission launches already start tracking_controller."
                ),
            ),
            DeclareLaunchArgument(
                "tracking_backend",
                default_value="dds",
                description="tracking_controller backend when included: mavros or dds.",
            ),
            DeclareLaunchArgument(
                "tracking_dds_target_topic",
                default_value="/px4_control_interface/controller_target_state",
                description="tracking_controller DDS sink target topic.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "middle_level_controller",
                default_value="cascaded_pid",
                description="Middle-level controller inside px4_tracking_mode_node: pass_through or cascaded_pid.",
            ),
            OpaqueFunction(function=_include_legacy_stack),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tracking_controller"), "launch", "tracking_controller.launch.py"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_tracking_controller")),
                launch_arguments={
                    "controller_backend": LaunchConfiguration("tracking_backend"),
                    "controller_dds_target_topic": LaunchConfiguration("tracking_dds_target_topic"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            Node(
                package="px4_control_interface",
                executable="px4_tracking_mode_node",
                name="px4_tracking_mode_node",
                output="screen",
                parameters=[
                    {
                        "target_topic": LaunchConfiguration("target_topic"),
                        "target_timeout_s": 0.2,
                        "use_input_yaw": True,
                        "middle_level_controller": LaunchConfiguration("middle_level_controller"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
