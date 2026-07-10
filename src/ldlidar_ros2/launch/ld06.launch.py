#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  port_name = LaunchConfiguration('port_name')
  serial_baudrate = LaunchConfiguration('serial_baudrate')

  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='ldlidar_publisher_ld06',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'laser_link'},
        {'port_name': port_name},
        {'serial_baudrate': serial_baudrate},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},  # unit is degress
        {'angle_crop_max': 225.0},  # unit is degress
        {'range_min': 0.02}, # unit is meter
        {'range_max': 12.0}   # unit is meter
      ]
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(DeclareLaunchArgument('port_name', default_value='/dev/jlink_lidar'))
  ld.add_action(DeclareLaunchArgument('serial_baudrate', default_value='230400'))
  ld.add_action(ldlidar_node)

  return ld
