from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    traj_planner_share = get_package_share_directory('trajectory_planner')
    bspline_param_file = os.path.join(
        traj_planner_share, 'cfg', 'bspline_interactive', 'bspline_planner_param.yaml'
    )
    occupancy_map_param_file = os.path.join(
        traj_planner_share, 'cfg', 'bspline_interactive', 'occupancy_map.yaml'
    )
    rviz_config_file = os.path.join(
        traj_planner_share, 'rviz', 'bspline_interactive.rviz'
    )

    return LaunchDescription([
        Node(
            package='trajectory_planner',
            executable='bspline_node',
            name='bspline_node',
            output='screen',
            parameters=[bspline_param_file,occupancy_map_param_file]
        ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=[
                    "-d",
                    rviz_config_file
                ],
            ),
        ]
    )
