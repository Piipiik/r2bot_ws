import os
from pathlib import Path

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory('r2bot_bringup'))
    nav2_dir = Path(get_package_share_directory('nav2_bringup'))
    ldlidar_ros2_dir = get_package_share_directory('ldlidar_ros2')
    astra_camera_dir = get_package_share_directory('astra_camera')

    default_map = str(bringup_dir / 'config' / 'maps' / 'r2_map.yaml')
    default_params = str(bringup_dir / 'config' / 'r2bot_nav2_params.yaml')
    default_rviz = str(bringup_dir / 'config' / 'nav2_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera = LaunchConfiguration('use_camera')

    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(str(bringup_dir), 'launch', 'urdf2tf.launch.py')
        )
    )

    odom_calc_node = launch_ros.actions.Node(
        package='r2bot_bringup',
        executable='odom_calculator.py',
        name='odom_calculator',
        output='screen'
    )

    chassis_driver = launch_ros.actions.Node(
        package='chassis_can_driver',
        executable='chassis_can_node',
        name='chassis_can_node',
        output='screen',
        parameters=[{
            'can_interface': 'can0',
            'send_period': 0.1,
            'reconnect_interval': 1.0
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_dir / 'launch' / 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    ldlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_ros2_dir, 'launch', 'ld06.launch.py')
        )
    )
    ldlidar_delay = TimerAction(period=5.0, actions=[ldlidar])

    astra_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(astra_camera_dir, 'launch', 'astra_pro.launch.xml')
        )
    )
    astra_camera_delay = TimerAction(
        period=3.0,
        actions=[astra_camera],
        condition=IfCondition(use_camera),
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        urdf2tf,
        odom_calc_node,
        chassis_driver,
        nav2,
        ldlidar_delay,
        astra_camera_delay,
        rviz_node,
    ])
