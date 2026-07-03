from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = Path(get_package_share_directory('r2bot_description'))
    urdf_path = description_dir / 'urdf' / 'r2bot.urdf'
    robot_description = urdf_path.read_text()

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': False},
            ],
        ),
    ])
