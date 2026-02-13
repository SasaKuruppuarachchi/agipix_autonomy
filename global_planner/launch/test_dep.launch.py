import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'global_planner'  
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'navigation.rviz' 
    )

    # Declare the launch argument for RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Full path to the RViz config file'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    
    dep_param_arg = DeclareLaunchArgument(
        'dep_param_file',
        default_value='dep_param_ros2.yaml',
        description='DEP config YAML in global_planner/cfg'
    )

    dynamic_map_param_arg = DeclareLaunchArgument(
        'dynamic_map_param_file',
        default_value='dynamic_map_param_ros2.yaml',
        description='Dynamic map config YAML in global_planner/cfg'
    )

    detector_param_arg = DeclareLaunchArgument(
        'detector_param_file',
        default_value='dynamic_detector_param_ros2.yaml',
        description='Onboard detector config YAML in global_planner/cfg'
    )

    dep_param_file = PathJoinSubstitution([
        FindPackageShare('global_planner'),
        'cfg',
        LaunchConfiguration('dep_param_file')
    ])

    dynamic_map_param_file = PathJoinSubstitution([
        FindPackageShare('global_planner'),
        'cfg',
        LaunchConfiguration('dynamic_map_param_file')
    ])

    detector_param_file = PathJoinSubstitution([
        FindPackageShare('global_planner'),
        'cfg',
        LaunchConfiguration('detector_param_file')
    ])

    dep_node = Node(
        package='global_planner',
        executable='test_dep_node',
        name='dep_test_node',
        output='screen',
        parameters=[
            dep_param_file,
            dynamic_map_param_file,
            detector_param_file
        ]
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
        dep_param_arg,
        dynamic_map_param_arg,
        detector_param_arg,
        dep_node
    ])
