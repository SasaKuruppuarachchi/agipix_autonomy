from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare a launch argument so the config filename can be passed on the CLI
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='map_param_sw.yaml',
        description='YAML config filename located in map_manager/cfg'
    )
    
    detector_config_arg = DeclareLaunchArgument(
        'detector_config_file',
        default_value='dynamic_detector_param_sw.yaml',
        description='YAML config filename located in map_manager/cfg'
    )

    # Build the parameter file path at runtime: <package_share>/cfg/<config_file>
    map_param_file = PathJoinSubstitution([
        FindPackageShare('map_manager'),
        'cfg',
        LaunchConfiguration('config_file')
    ])
    
    detector_param_file = PathJoinSubstitution([
        FindPackageShare('map_manager'),
        'cfg',
        LaunchConfiguration('detector_config_file')
    ])

    # Log the resolved parameter file (will print when substitutions are resolved)
    log = LogInfo(msg=['Using parameter file: ', map_param_file])
    log1 = LogInfo(msg=['Using parameter file: ', detector_param_file])

    # Create the node with the parameter file substitution
    dynamic_map_node = Node(
        package='map_manager',
        executable='dynamic_map_node',
        name='dynamic_map_node',
        output='screen',
        parameters=[map_param_file]
    )
    
    # dynamic_detector_node = Node(
    #     package='onboard_detector',
    #     executable='dynamic_detector_node',
    #     name='dynamic_detector_node',
    #     output='screen',
    #     parameters=[detector_param_file]
    # )

    return LaunchDescription([
        config_arg,
        log,
        dynamic_map_node,
        detector_config_arg,
        log1,
        #dynamic_detector_node
    ])