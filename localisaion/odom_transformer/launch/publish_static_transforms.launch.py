#    Odom transformer - ROS 2 Node for performing odometry transformations.
#    Copyright (C) 2023  Karelics Oy
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program. If not, see <https://www.gnu.org/licenses/>.

"""Example launch file for tansform_pub"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    """Launch tansform_pub. -> translation + orientation + [parent, child]"""

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    drone_namespace_arg = DeclareLaunchArgument(
        'drone_namespace',
        default_value='drone0',
        description='Namespace prefix used to build TF frame ids.'
    )

    map_frame = [LaunchConfiguration('drone_namespace'), '/map']
    base_link_frame = [LaunchConfiguration('drone_namespace'), '/base_link']
    px4_frame = [LaunchConfiguration('drone_namespace'), '/px4_frame']
    livox_imu_frame = [LaunchConfiguration('drone_namespace'), '/livox_imu_frame']
    camera_frame = [LaunchConfiguration('drone_namespace'), '/camera']
    if use_sim_time_arg == 'true':
        livox_lidar_frame = [LaunchConfiguration('drone_namespace'), '/livox_lidar_frame']
    else:
        livox_lidar_frame  = [LaunchConfiguration('drone_namespace'), '/livox_lidar_frame']
    
    
    package_share_path = get_package_share_directory("odom_transformer")
    transformer_config_path = os.path.join(package_share_path, "params", "odom_transformer_params.yaml")
    
    px4_imu = Node(
        package="odom_transformer",
        executable="px4_imu_node.py",
        name="px4_imu",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[],
    )
    odom_transformer = Node(
        package="odom_transformer",
        executable="transformer_node.py",
        name="odom_transformer",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[transformer_config_path],
    )
    # Map
    transform_drone0map_to_earth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='earth_frame_pub',
        output='screen',
        arguments = [ '0', '0', '0', '0', '0', '0', '1.0', map_frame, 'earth'] 
    )
    # Map
    transform_drone0map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_frame_pub',
        output='screen',
        arguments = [ '0', '0', '0', '0', '0', '0', '1.0', map_frame, 'odom'] 
    )
    ## -------------- Sensror frames -------------- ##
    # Lidar
    transform_drone0base_link_to_livox_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_lidar_frame_pub',
        output='screen',
        arguments = [ '0.0905', '0.02329', '0.00182', '0', '0.3826834', '0', '0.9238795', base_link_frame, livox_lidar_frame]
    )
    
        
    #Lidar IMU
    transform_base_link_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_imu_frame_pub',
        output='screen',
        arguments=['0.0905', '0.02329', '0.00182', '0', '0.3826834', '0', '0.9238795', base_link_frame, 'livox_frame'] #'0.0905', '0.02329', '0.00182'
    )
    #Camera front
    transform_px4_to_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_pub',
        output='screen',
        arguments=['0.117', '0', '-0.025', '-0.5', '0.5', '-0.5', '0.5', base_link_frame, 'camera_frame']
    )
    # px4
    transform_baselink_to_px4_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='px4_frame_pub',
        output='screen',
        arguments=['0', '0', '0', '1', '0', '0', '0', base_link_frame, px4_frame]
    )
    
    return LaunchDescription([use_sim_time_arg,
        drone_namespace_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')), 
        #odom_transformer, 
        #px4_imu, 
        #transform_drone0map_to_earth, 
        transform_drone0map_to_odom, 
        transform_baselink_to_px4_frame,
        transform_drone0base_link_to_livox_lidar]
        )
