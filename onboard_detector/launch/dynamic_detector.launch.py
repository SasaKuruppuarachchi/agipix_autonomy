from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare a launch argument for the dynamic detector config filename
    dynamic_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='dynamic_detector_param_sw.yaml',
        description='YAML config filename for the dynamic detector, located in onboard_detector/cfg'
    )

    # Build the parameter file path at runtime for dynamic detector: <package_share>/cfg/<config_file>
    dynamic_param_file = PathJoinSubstitution([
        FindPackageShare('onboard_detector'),
        'cfg',
        LaunchConfiguration('config_file')
    ])

    # Keep YOLO param handling as-is (resolved at import/runtime via get_package_share_directory)
    yolo_detector_param_path = os.path.join(
        get_package_share_directory('onboard_detector'),
        'cfg',
        'yolo_detector_param.yaml'
    )

    # Log the resolved dynamic parameter file
    log = LogInfo(msg=['Using dynamic detector parameter file: ', dynamic_param_file])

    # Create the dynamic detector node using the substituted parameter path
    dynamic_detector_node = Node(
        package='onboard_detector',
        executable='dynamic_detector_node',
        name='dynamic_detector_node',
        output='screen',
        parameters=[dynamic_param_file]
    )

    # Create the yolo node (unchanged)
    yolo_detector_node = Node(
        package='onboard_detector',
        executable='yolo_detector_node.py',
        name='yolo_detector_node',
        output='screen',
        parameters=[yolo_detector_param_path]
    )

    return LaunchDescription([
        dynamic_config_arg,
        log,
        dynamic_detector_node,
        yolo_detector_node
    ])