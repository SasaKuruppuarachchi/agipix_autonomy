from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    af_share = FindPackageShare("autonomous_flight")

    af_params = [
        PathJoinSubstitution([af_share, "cfg", "inspection", "inspection_param.yaml"]),
    ]
    octomap_path = os.path.join(
        get_package_share_directory('global_planner'),
        'map',
        'maze.bt'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server_node',
                output='screen',
                #arguments=[LaunchConfiguration('map_location')]
                parameters=[
                    {'octomap_path': octomap_path},
                    {'frame_id': 'drone0/map'},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ]  # <--- Pass as param here
            ),
            Node(
                package="autonomous_flight",
                executable="inspection_node",
                name="inspection_node",
                output="screen",
                parameters=af_params + [
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            )
        ]
    )
