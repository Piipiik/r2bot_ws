#!/bin/bash

# 切换到脚本所在目录
cd /home/yzy/r2bot_ws || exit 1

# 1. 先执行 CAN 命令
echo "1234" | ./can_command.py 0 1
if [ $? -ne 0 ]; then
    echo "can_command.py 执行失败，退出"
    exit 1
fi

# 2. 等待激光雷达设备就绪（可选但推荐）
echo "等待激光雷达设备 /dev/lidar_ld06..."
while [ ! -e /dev/lidar_ld06 ]; do
    sleep 1
done
echo "激光雷达已就绪"

# 3. 加载 ROS2 环境
source install/setup.bash

# 4. 启动 launch 文件
ros2 launch r2bot_bringup align_once.launch.py \
    front_mm:=1000 \
    left_mm:=2500 \
    port_name:=/dev/lidar_ld06 \
    tolerance_mm:=4 \
    timeout_s:=0
