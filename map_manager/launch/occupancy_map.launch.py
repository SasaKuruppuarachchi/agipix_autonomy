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

    # Build the parameter file path at runtime: <package_share>/cfg/<config_file>
    param_file = PathJoinSubstitution([
        FindPackageShare('map_manager'),
        'cfg',
        LaunchConfiguration('config_file')
    ])

    # Log the resolved parameter file (will print when substitutions are resolved)
    log = LogInfo(msg=['Using parameter file: ', param_file])

    # Create the node with the parameter file substitution
    occupancy_map_node = Node(
        package='map_manager',
        executable='occupancy_map_node',
        name='occupancy_map_node',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        config_arg,
        log,
        occupancy_map_node
    ])