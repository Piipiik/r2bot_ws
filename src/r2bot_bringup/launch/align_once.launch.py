from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('r2bot_bringup')
    ldlidar_dir = get_package_share_directory('ldlidar_ros2')

    front_mm = LaunchConfiguration('front_mm')
    back_mm = LaunchConfiguration('back_mm')
    left_mm = LaunchConfiguration('left_mm')
    right_mm = LaunchConfiguration('right_mm')
    tolerance_mm = LaunchConfiguration('tolerance_mm')
    sector_deg = LaunchConfiguration('sector_deg')
    timeout_s = LaunchConfiguration('timeout_s')
    kp_linear = LaunchConfiguration('kp_linear')
    max_vx = LaunchConfiguration('max_vx')
    max_vy = LaunchConfiguration('max_vy')
    max_speed_xy = LaunchConfiguration('max_speed_xy')
    kp_yaw = LaunchConfiguration('kp_yaw')
    max_wz = LaunchConfiguration('max_wz')
    yaw_tolerance_deg = LaunchConfiguration('yaw_tolerance_deg')
    yaw_translation_gate_deg = LaunchConfiguration('yaw_translation_gate_deg')
    status_interval_s = LaunchConfiguration('status_interval_s')
    port_name = LaunchConfiguration('port_name')
    start_lidar = LaunchConfiguration('start_lidar')
    start_chassis = LaunchConfiguration('start_chassis')
    start_odom = LaunchConfiguration('start_odom')
    source = LaunchConfiguration('source')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    ldlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ldlidar_dir, 'launch', 'ld06.launch.py')),
        launch_arguments={
            'port_name': port_name,
        }.items(),
        condition=IfCondition(start_lidar),
    )
    ldlidar_delay = TimerAction(period=2.0, actions=[ldlidar])

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

    odom_calc = Node(
        package='r2bot_bringup',
        executable='odom_calculator.py',
        name='odom_calculator',
        output='screen',
        condition=IfCondition(start_odom),
    )

    align_node = Node(
        package='r2bot_bringup',
        executable='align_to_obstacles.py',
        name='align_to_obstacles',
        output='screen',
        arguments=[
            '--front-mm', front_mm,
            '--back-mm', back_mm,
            '--left-mm', left_mm,
            '--right-mm', right_mm,
            '--tolerance-mm', tolerance_mm,
            '--sector-deg', sector_deg,
            '--timeout-s', timeout_s,
            '--kp-linear', kp_linear,
            '--max-vx', max_vx,
            '--max-vy', max_vy,
            '--max-speed-xy', max_speed_xy,
            '--kp-yaw', kp_yaw,
            '--max-wz', max_wz,
            '--yaw-tolerance-deg', yaw_tolerance_deg,
            '--yaw-translation-gate-deg', yaw_translation_gate_deg,
            '--status-interval-s', status_interval_s,
            '--source', source,
            '--scan-topic', scan_topic,
            '--cmd-vel-topic', cmd_vel_topic,
            '--odom-topic', odom_topic,
            '--serial-port', port_name,
        ],
    )
    align_delay = TimerAction(period=4.0, actions=[align_node])

    return LaunchDescription([
        DeclareLaunchArgument('front_mm', default_value='-1'),
        DeclareLaunchArgument('back_mm', default_value='-1'),
        DeclareLaunchArgument('left_mm', default_value='-1'),
        DeclareLaunchArgument('right_mm', default_value='-1'),
        DeclareLaunchArgument('tolerance_mm', default_value='20'),
        DeclareLaunchArgument('sector_deg', default_value='8.0'),
        DeclareLaunchArgument('timeout_s', default_value='20.0'),
        DeclareLaunchArgument('kp_linear', default_value='0.003'),
        DeclareLaunchArgument('max_vx', default_value='0.15'),
        DeclareLaunchArgument('max_vy', default_value='0.15'),
        DeclareLaunchArgument('max_speed_xy', default_value='0.15'),
        DeclareLaunchArgument('kp_yaw', default_value='2.2'),
        DeclareLaunchArgument('max_wz', default_value='0.25'),
        DeclareLaunchArgument('yaw_tolerance_deg', default_value='1.0'),
        DeclareLaunchArgument('yaw_translation_gate_deg', default_value='1.0'),
        DeclareLaunchArgument('status_interval_s', default_value='0.5'),
        DeclareLaunchArgument('port_name', default_value='/dev/lidar_ld06'),
        DeclareLaunchArgument('source', default_value='serial'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('start_chassis', default_value='true'),
        DeclareLaunchArgument('start_odom', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        ldlidar_delay,
        chassis_driver,
        odom_calc,
        align_delay,
    ])
