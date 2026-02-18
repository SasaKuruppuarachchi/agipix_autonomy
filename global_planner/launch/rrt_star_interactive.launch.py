from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    static_tf_parent_frame_arg = DeclareLaunchArgument(
        'static_tf_parent_frame',
        default_value='map',
        description='Static TF parent frame'
    )
    static_tf_child_frame_arg = DeclareLaunchArgument(
        'static_tf_child_frame',
        default_value='base_link',
        description='Static TF child frame'
    )
    static_tf_x_arg = DeclareLaunchArgument(
        'static_tf_x',
        default_value='0.0',
        description='Static TF translation x'
    )
    static_tf_y_arg = DeclareLaunchArgument(
        'static_tf_y',
        default_value='0.0',
        description='Static TF translation y'
    )
    static_tf_z_arg = DeclareLaunchArgument(
        'static_tf_z',
        default_value='0.0',
        description='Static TF translation z'
    )
    static_tf_qx_arg = DeclareLaunchArgument(
        'static_tf_qx',
        default_value='0.0',
        description='Static TF rotation qx'
    )
    static_tf_qy_arg = DeclareLaunchArgument(
        'static_tf_qy',
        default_value='0.0',
        description='Static TF rotation qy'
    )
    static_tf_qz_arg = DeclareLaunchArgument(
        'static_tf_qz',
        default_value='0.0',
        description='Static TF rotation qz'
    )
    static_tf_qw_arg = DeclareLaunchArgument(
        'static_tf_qw',
        default_value='1.0',
        description='Static TF rotation qw'
    )

    map_location_arg = DeclareLaunchArgument(
        'map_location',
        default_value=PathJoinSubstitution([
            FindPackageShare('global_planner'),
            'map',
            'maze.bt'
        ]),
        description='Path to the OctoMap binary file'
    )
    
    octomap_path = os.path.join(
        get_package_share_directory('global_planner'),
        'map',
        'maze.bt'
    )

    rrt_param_file = PathJoinSubstitution([
        FindPackageShare('global_planner'),
        'cfg',
        'rrt_planner_ros2.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('global_planner'),
        'rviz',
        'rrt_interactive.rviz'
    ])

    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time')
    )

    planner_node = Node(
        package='global_planner',
        executable='rrt_star_interactive_node',
        name='rrt_star_interactive_node',
        output='screen',
        parameters=[rrt_param_file]
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        output='screen',
        parameters=[{'octomap_path': octomap_path}, {'frame_id': 'drone0/map'}]  # <--- Pass as param here
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        output='screen',
        arguments=[
            LaunchConfiguration('static_tf_x'),
            LaunchConfiguration('static_tf_y'),
            LaunchConfiguration('static_tf_z'),
            LaunchConfiguration('static_tf_qx'),
            LaunchConfiguration('static_tf_qy'),
            LaunchConfiguration('static_tf_qz'),
            LaunchConfiguration('static_tf_qw'),
            LaunchConfiguration('static_tf_parent_frame'),
            LaunchConfiguration('static_tf_child_frame')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf_parent_frame_arg,
        static_tf_child_frame_arg,
        static_tf_x_arg,
        static_tf_y_arg,
        static_tf_z_arg,
        static_tf_qx_arg,
        static_tf_qy_arg,
        static_tf_qz_arg,
        static_tf_qw_arg,
        map_location_arg,
        set_use_sim_time,
        planner_node,
        octomap_server_node,
        static_tf_node,
        rviz_node
    ])
