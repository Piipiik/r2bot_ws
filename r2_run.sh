#!/bin/bash
set -u

# 切换到脚本所在目录
cd /home/yzy/r2bot_ws || exit 1

CAN_IFACE="${CAN_IFACE:-can0}"
ARM_PICK_FRAME="${ARM_PICK_FRAME:-130#120EFA00C800B400}"
ARM_HOME_FRAME="${ARM_HOME_FRAME:-430#0100000000000000}"

# ROTATE_DIR: cw=顺时针, ccw=逆时针
ROTATE_DIR="${ROTATE_DIR:-cw}"
ROTATE_SPEED_CMD="${ROTATE_SPEED_CMD:-20}"
ROTATE_DURATION_S="${ROTATE_DURATION_S:-4.7}"

send_can() {
    local frame="$1"
    echo "[r2_run] cansend ${CAN_IFACE} ${frame}"
    cansend "${CAN_IFACE}" "${frame}"
}

int8_hex() {
    local value="$1"
    if [ "${value}" -lt 0 ]; then
        value=$((256 + value))
    fi
    printf "%02X" "${value}"
}

send_wheel_speed() {
    local speed="$1"
    local speed_hex
    speed_hex="$(int8_hex "${speed}")"

    send_can "123#11${speed_hex}000000000000"
    send_can "124#11${speed_hex}000000000000"
    send_can "125#11${speed_hex}000000000000"
    send_can "126#11${speed_hex}000000000000"
}

stop_chassis() {
    send_wheel_speed 0
}

rotate_chassis_180() {
    local speed="${ROTATE_SPEED_CMD}"

    case "${ROTATE_DIR}" in
        cw|CW|clockwise)
            speed="${ROTATE_SPEED_CMD}"
            ;;
        ccw|CCW|counterclockwise)
            speed=$((-ROTATE_SPEED_CMD))
            ;;
        *)
            echo "[r2_run] invalid ROTATE_DIR=${ROTATE_DIR}, expected cw or ccw"
            return 2
            ;;
    esac

    echo "[r2_run] rotate chassis 180 deg, dir=${ROTATE_DIR}, speed_cmd=${speed}, duration=${ROTATE_DURATION_S}s"
    send_wheel_speed "${speed}" || return $?
    sleep "${ROTATE_DURATION_S}"
    stop_chassis
}

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
# ROS setup.bash may read unset variables, so disable nounset while sourcing it.
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# 3.5 等待雷达触发条件：front/left < 20mm 持续 2s，然后 front/left > 20mm，再等待 5s
./wait_lidar_gate.py \
    --port /dev/lidar_ld06 \
    --threshold-mm 20 \
    --hold-s 2 \
    --post-delay-s 5
if [ $? -ne 0 ]; then
    echo "雷达触发脚本执行失败，退出"
    exit 1
fi

# 4. 启动 launch 文件
ros2 launch r2bot_bringup align_once.launch.py \
    front_mm:=1000 \
    port_name:=/dev/lidar_ld06 \
    tolerance_mm:=4 \
    timeout_s:=0
rc=$?
if [ "${rc}" -ne 0 ]; then
    echo "第一次对齐失败，退出"
    exit "${rc}"
fi

# 5. 第一次对齐成功后，继续左侧 500mm 对齐
ros2 launch r2bot_bringup align_once.launch.py \
    left_mm:=500 \
    port_name:=/dev/lidar_ld06 \
    tolerance_mm:=4 \
    timeout_s:=0
rc=$?
if [ "${rc}" -ne 0 ]; then
    echo "第二次对齐失败，退出"
    exit "${rc}"
fi

# 6. 第二次对齐成功后，等待 1s，移动机械臂
sleep 1
send_can "${ARM_PICK_FRAME}" || exit $?

# 7. 等待 2s，机械臂回中
sleep 2
send_can "${ARM_HOME_FRAME}" || exit $?

# 8. 回中命令发送成功后，等待 2s，让车体旋转 180°
sleep 2
rotate_chassis_180 || exit $?

# 9. 旋转完成后，再次移动机械臂
send_can "${ARM_PICK_FRAME}"
