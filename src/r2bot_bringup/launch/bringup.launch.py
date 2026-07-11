import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    r2bot_bringup_dir = get_package_share_directory('r2bot_bringup')
    ldlidar_ros2_dir = get_package_share_directory('ldlidar_ros2')
    object_params = os.path.join(r2bot_bringup_dir, 'config', 'object_follower.yaml')
    use_camera = LaunchConfiguration('use_camera')
    enable_object_follower = LaunchConfiguration('enable_object_follower')
    lidar_port_name = LaunchConfiguration('lidar_port_name')
    lidar_keep_angle_ranges_deg_csv = LaunchConfiguration('lidar_keep_angle_ranges_deg_csv')

    # 1. URDF 与 TF
    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(r2bot_bringup_dir, 'launch', 'urdf2tf.launch.py')
        )
    )

    # 2. 里程计计算节点（如有）
    odom_calc_node = launch_ros.actions.Node(
        package='r2bot_bringup',
        executable='odom_calculator.py',
        name='odom_calculator',
        output='screen'
    )

    # 3. 底盘 CAN 驱动节点
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

    # 4. 雷达（延时 5s）
    ldlidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_ros2_dir, 'launch', 'ld06.launch.py')
        ),
        launch_arguments={
            'port_name': lidar_port_name,
            'keep_angle_ranges_deg_csv': lidar_keep_angle_ranges_deg_csv,
        }.items(),
    )
    ldlidar_delay = launch.actions.TimerAction(period=5.0, actions=[ldlidar])

    # 5. D435 摄像头
    d435_camera = launch_ros.actions.Node(
        package='r2bot_bringup',
        executable='d435_camera_node',
        name='d435_camera_node',
        output='screen',
        condition=IfCondition(use_camera),
    )
    d435_camera_delay = launch.actions.TimerAction(period=3.0, actions=[d435_camera])

    object_follower = launch_ros.actions.Node(
        package='r2bot_bringup',
        executable='rgbd_object_approach.py',
        name='rgbd_object_approach',
        output='screen',
        parameters=[object_params],
        condition=IfCondition(enable_object_follower),
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_camera', default_value='true'),
        launch.actions.DeclareLaunchArgument('enable_object_follower', default_value='false'),
        launch.actions.DeclareLaunchArgument('lidar_port_name', default_value='/dev/lidar_ld06'),
        launch.actions.DeclareLaunchArgument('lidar_keep_angle_ranges_deg_csv', default_value=''),
        urdf2tf,
        odom_calc_node,
        chassis_driver,
        ldlidar_delay,
        d435_camera_delay,
        object_follower
    ])
