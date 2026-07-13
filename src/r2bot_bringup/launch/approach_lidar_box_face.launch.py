from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ldlidar_dir = get_package_share_directory('ldlidar_ros2')

    port_name = LaunchConfiguration('port_name')
    start_lidar = LaunchConfiguration('start_lidar')
    start_chassis = LaunchConfiguration('start_chassis')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    keep_angle_ranges_deg_csv = LaunchConfiguration('keep_angle_ranges_deg_csv')

    target_length_mm = LaunchConfiguration('target_length_mm')
    length_tolerance_mm = LaunchConfiguration('length_tolerance_mm')
    target_distance_mm = LaunchConfiguration('target_distance_mm')
    distance_tolerance_mm = LaunchConfiguration('distance_tolerance_mm')
    center_tolerance_mm = LaunchConfiguration('center_tolerance_mm')
    target_center_offset_mm = LaunchConfiguration('target_center_offset_mm')
    angle_tolerance_deg = LaunchConfiguration('angle_tolerance_deg')
    max_range_mm = LaunchConfiguration('max_range_mm')
    timeout_s = LaunchConfiguration('timeout_s')

    ldlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ldlidar_dir, 'launch', 'ld06.launch.py')),
        launch_arguments={
            'port_name': port_name,
            'keep_angle_ranges_deg_csv': keep_angle_ranges_deg_csv,
        }.items(),
        condition=IfCondition(start_lidar),
    )

    chassis_driver = Node(
        package='chassis_can_driver',
        executable='chassis_can_node',
        name='chassis_can_node',
        output='screen',
        parameters=[{
            'can_interface': 'can0',
            'send_period': 0.1,
            'reconnect_interval': 1.0,
        }],
        condition=IfCondition(start_chassis),
    )

    approach = Node(
        package='r2bot_bringup',
        executable='approach_lidar_box_face.py',
        name='approach_lidar_box_face',
        output='screen',
        arguments=[
            '--scan-topic', scan_topic,
            '--cmd-vel-topic', cmd_vel_topic,
            '--target-length-mm', target_length_mm,
            '--length-tolerance-mm', length_tolerance_mm,
            '--target-distance-mm', target_distance_mm,
            '--distance-tolerance-mm', distance_tolerance_mm,
            '--center-tolerance-mm', center_tolerance_mm,
            '--target-center-offset-mm', target_center_offset_mm,
            '--angle-tolerance-deg', angle_tolerance_deg,
            '--max-range-mm', max_range_mm,
            '--timeout-s', timeout_s,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='/dev/lidar_ld06'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_chassis', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('keep_angle_ranges_deg_csv', default_value='270,90'),
        DeclareLaunchArgument('target_length_mm', default_value='1200'),
        DeclareLaunchArgument('length_tolerance_mm', default_value='20'),
        DeclareLaunchArgument('target_distance_mm', default_value='200'),
        DeclareLaunchArgument('distance_tolerance_mm', default_value='20'),
        DeclareLaunchArgument('center_tolerance_mm', default_value='20'),
        DeclareLaunchArgument('target_center_offset_mm', default_value='0'),
        DeclareLaunchArgument('angle_tolerance_deg', default_value='2.0'),
        DeclareLaunchArgument('max_range_mm', default_value='3000'),
        DeclareLaunchArgument('timeout_s', default_value='30.0'),
        chassis_driver,
        TimerAction(period=2.0, actions=[ldlidar]),
        TimerAction(period=4.0, actions=[approach]),
    ])
