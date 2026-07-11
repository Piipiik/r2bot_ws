import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory('r2bot_bringup'))
    slam_toolbox_dir = Path(get_package_share_directory('slam_toolbox'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_camera = LaunchConfiguration('use_camera')
    lidar_port_name = LaunchConfiguration('lidar_port_name')
    lidar_keep_angle_ranges_deg_csv = LaunchConfiguration('lidar_keep_angle_ranges_deg_csv')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_dir / 'launch' / 'bringup.launch.py')),
        launch_arguments={
            'use_camera': use_camera,
            'enable_object_follower': 'false',
            'lidar_port_name': lidar_port_name,
            'lidar_keep_angle_ranges_deg_csv': lidar_keep_angle_ranges_deg_csv,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(str(slam_toolbox_dir), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('lidar_port_name', default_value='/dev/lidar_ld06'),
        DeclareLaunchArgument(
            'lidar_keep_angle_ranges_deg_csv',
            default_value='-20,30,60,90',
        ),
        bringup,
        slam,
    ])
