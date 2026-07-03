import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

def generate_launch_description():
    r2bot_bringup_dir = get_package_share_directory('r2bot_bringup')
    ldlidar_ros2_dir = get_package_share_directory('ldlidar_ros2')
    astra_camera_dir = get_package_share_directory('astra_camera')

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
        )
    )
    ldlidar_delay = launch.actions.TimerAction(period=5.0, actions=[ldlidar])

    # 5. 摄像头（延时 3s）
    astra_camera = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(astra_camera_dir, 'launch', 'astra_pro.launch.xml')
        )
    )
    astra_camera_delay = launch.actions.TimerAction(period=3.0, actions=[astra_camera])

    return launch.LaunchDescription([
        urdf2tf,
        odom_calc_node,
        chassis_driver,
        ldlidar_delay,
        astra_camera_delay
    ])
