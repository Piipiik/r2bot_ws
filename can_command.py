#!/bin/bash
cd /home/yzy/r2bot_ws || exit 1

# 依次输入：设备编号、波特率、sudo密码
# 每个输入用 \n 换行
printf "0\n1\n1234\n" | ./can_command.py
if [ $? -ne 0 ]; then
    echo "can_command.py 执行失败，退出"
    exit 1
fi

echo "等待激光雷达设备 /dev/lidar_ld06..."
while [ ! -e /dev/lidar_ld06 ]; do
    sleep 1
done
echo "激光雷达已就绪"

source install/setup.bash
ros2 launch r2bot_bringup align_once.launch.py \
    front_mm:=1000 \
    left_mm:=2500 \
    port_name:=/dev/lidar_ld06 \
    tolerance_mm:=4 \
    timeout_s:=0
