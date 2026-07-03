# r2bot_ws

robocon R2机器人

source /opt/ros/humble/setup.bash
source ~/r2bot_ws/install/setup.bash

###控制
source /opt/ros/humble/setup.bash
source /home/yzy/r2bot_ws/install/setup.bash
ros2 launch r2bot_bringup bringup.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

###建图
source /opt/ros/humble/setup.bash
source /home/yzy/r2bot_ws/install/setup.bash
ros2 launch r2bot_bringup bringup.launch.py

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
rviz2
ros2 run nav2_map_server map_saver_cli -f ~/r2bot_map/r2bot_map

###导航
建图后地图放在
r2bot_bringup/config/maps/r2_map.yaml 和 r2_map.pgm

ros2 launch r2bot_bringup nav2.launch.py
“
如果你不想开 RViz：
ros2 launch r2bot_bringup nav2.launch.py use_rviz:=false
如果以后要顺带开相机：
ros2 launch r2bot_bringup nav2.launch.py use_camera:=true
“
