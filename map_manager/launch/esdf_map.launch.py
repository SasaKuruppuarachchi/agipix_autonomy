from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Define the path to the YAML parameter file
    param_file_path = os.path.join(
        get_package_share_directory('map_manager'),
        'cfg',
        'map_param_sw.yaml'
    )

    # Create the node with the parameter file
    esdf_map_node = Node(
        package='map_manager',
        executable='esdf_map_node',
        name='map_manager_node',
        output='screen',
        parameters=[param_file_path]
    )

    return LaunchDescription([
        use_sim_time_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        esdf_map_node
    ])