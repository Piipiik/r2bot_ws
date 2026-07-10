from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory('r2bot_bringup'))
    ldlidar_ros2_dir = Path(get_package_share_directory('ldlidar_ros2'))
    default_params = str(bringup_dir / 'config' / 'object_follower.yaml')

    params_file = LaunchConfiguration('params_file')

    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_dir / 'launch' / 'urdf2tf.launch.py'))
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
    )

    ldlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ldlidar_ros2_dir / 'launch' / 'ld06.launch.py'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        urdf2tf,
        chassis_driver,
        ldlidar,
        Node(
            package='r2bot_bringup',
            executable='d435_camera_node',
            name='d435_camera_node',
            output='screen',
        ),
        Node(
            package='r2bot_bringup',
            executable='rgbd_object_approach.py',
            name='rgbd_object_approach',
            output='screen',
            parameters=[params_file],
        ),
    ])
