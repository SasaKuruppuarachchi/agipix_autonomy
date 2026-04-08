import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    package_name = 'onboard_detector'  
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'detector.rviz' 
    )

    # Declare the launch argument for RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Full path to the RViz config file'
    )
    
    drone_namespace_arg = DeclareLaunchArgument(
        "drone_namespace",
        default_value="drone0",
        description="Namespace applied to autonomous flight nodes and mission topics.",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=LaunchConfiguration('drone_namespace'),
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        drone_namespace_arg,
        use_sim_time_arg,
        rviz_config_arg,
        rviz_node
    ])
