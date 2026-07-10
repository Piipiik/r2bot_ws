#!/usr/bin/env python3

import math
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

CAN_FRAME_FORMAT = '=IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_ID_MASK = 0x1FFFFFFF

CMD_SET_SPEED = 0x11
FLAG_STALL = 0x01
FLAG_SATURATED = 0x02

SQRT2 = math.sqrt(2.0)
INV_SQRT2 = 1.0 / SQRT2

WHEEL_NAMES = ('lf', 'lr', 'rf', 'rr')
COMMAND_IDS = {
    'lf': 0x123,
    'lr': 0x124,
    'rf': 0x125,
    'rr': 0x126,
}
STATUS_IDS = {
    0x323: 'lf',
    0x324: 'lr',
    0x325: 'rf',
    0x326: 'rr',
}


@dataclass
class WheelState:
    name: str
    velocity_raw: int = 0
    encoder_low16: int = 0
    encoder_full: int = 0
    encoder_initialized: bool = False
    pwm: int = 0
    target_speed: int = 0
    flags: int = 0
    wheel_speed_mps: float = 0.0
    stamp: float = 0.0


@dataclass
class ChassisFeedback:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    wheel_speeds: Dict[str, float] = field(default_factory=dict)


class ChassisCanNode(Node):
    def __init__(self):
        super().__init__('chassis_can_node')

        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('send_period', 0.05)
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('wheel_diameter_m', 0.152)
        self.declare_parameter('wheel_center_radius_m', 0.33)
        self.declare_parameter('encoder_ticks_per_rev', 4320.0)
        self.declare_parameter('speed_sample_period_s', 0.01)
        self.declare_parameter('max_motor_speed_cmd', 100)

        self.can_interface = str(self.get_parameter('can_interface').value)
        self.send_period = float(self.get_parameter('send_period').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.wheel_diameter_m = float(self.get_parameter('wheel_diameter_m').value)
        self.wheel_center_radius_m = float(self.get_parameter('wheel_center_radius_m').value)
        self.encoder_ticks_per_rev = float(self.get_parameter('encoder_ticks_per_rev').value)
        self.speed_sample_period_s = float(self.get_parameter('speed_sample_period_s').value)
        self.max_motor_speed_cmd = int(self.get_parameter('max_motor_speed_cmd').value)

        if self.send_period <= 0.0:
            self.send_period = 0.05
        if self.reconnect_interval <= 0.0:
            self.reconnect_interval = 1.0
        if self.command_timeout < 0.0:
            self.command_timeout = 0.5
        if self.encoder_ticks_per_rev <= 0.0:
            raise ValueError('encoder_ticks_per_rev 必须大于 0')
        if self.speed_sample_period_s <= 0.0:
            raise ValueError('speed_sample_period_s 必须大于 0')
        if self.wheel_diameter_m <= 0.0:
            raise ValueError('wheel_diameter_m 必须大于 0')
        if self.wheel_center_radius_m <= 0.0:
            raise ValueError('wheel_center_radius_m 必须大于 0')
        if self.max_motor_speed_cmd <= 0:
            raise ValueError('max_motor_speed_cmd 必须大于 0')

        self.wheel_radius_m = self.wheel_diameter_m * 0.5
        self.wheel_circumference_m = math.pi * self.wheel_diameter_m

        self.target_cmd = Twist()
        self.last_cmd_time = time.monotonic()
        self.cmd_lock = threading.Lock()

        self.state_lock = threading.Lock()
        self.wheel_states = {name: WheelState(name=name) for name in WHEEL_NAMES}
        self.feedback = ChassisFeedback(wheel_speeds={name: 0.0 for name in WHEEL_NAMES})

        self.can_socket: Optional[socket.socket] = None
        self._running = threading.Event()
        self._running.set()

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.current_vel_pub = self.create_publisher(Twist, 'current_velocity', 10)
        self.wheel_speed_pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)
        self.wheel_encoder_pub = self.create_publisher(Int32MultiArray, 'wheel_encoders', 10)

        self.get_logger().info('=' * 60)
        self.get_logger().info('四轮底盘 CAN 驱动启动')
        self.get_logger().info(f'CAN 接口: {self.can_interface}')
        self.get_logger().info(f'轮径: {self.wheel_diameter_m:.3f} m')
        self.get_logger().info(f'轮心到车体中心: {self.wheel_center_radius_m:.3f} m')
        self.get_logger().info(f'编码器每圈: {self.encoder_ticks_per_rev:.1f}')
        self.get_logger().info(f'速度采样周期: {self.speed_sample_period_s:.3f} s')
        self.get_logger().info('命令映射: 0x123/124/125/126 + [0x11, speed]')

        if not self._open_can_socket():
            self.get_logger().warn('首次打开 CAN 失败，将自动重连')

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True, name='can_recv')
        self.recv_thread.start()
        self.send_timer = self.create_timer(self.send_period, self._send_cmd_loop)

    def _open_can_socket(self) -> bool:
        try:
            can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            can_socket.settimeout(0.1)
            can_socket.bind((self.can_interface,))
            self.can_socket = can_socket
            self.get_logger().info(f'CAN socket 打开成功: {self.can_interface}')
            return True
        except OSError as exc:
            self.can_socket = None
            self.get_logger().error(f'CAN socket 打开失败: {exc}')
            return False

    def _close_can_socket(self):
        if self.can_socket is not None:
            try:
                self.can_socket.close()
            except OSError:
                pass
        self.can_socket = None

    @staticmethod
    def _build_frame(can_id: int, payload: bytes) -> bytes:
        if len(payload) > 8:
            raise ValueError('CAN payload 长度不能超过 8')
        return struct.pack(CAN_FRAME_FORMAT, can_id, len(payload), payload.ljust(8, b'\x00'))

    def _send_frame(self, can_id: int, payload: bytes):
        if self.can_socket is None:
            raise OSError('CAN socket 未连接')
        self.can_socket.send(self._build_frame(can_id, payload))

    def cmd_vel_callback(self, msg: Twist):
        with self.cmd_lock:
            self.target_cmd = msg
            self.last_cmd_time = time.monotonic()

    def _send_cmd_loop(self):
        if self.can_socket is None:
            return

        with self.cmd_lock:
            cmd = Twist()
            cmd.linear.x = self.target_cmd.linear.x
            cmd.linear.y = self.target_cmd.linear.y
            cmd.angular.z = self.target_cmd.angular.z
            expired = (time.monotonic() - self.last_cmd_time) > self.command_timeout

        if expired:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        wheel_cmds = self._inverse_kinematics(cmd.linear.x, cmd.linear.y, cmd.angular.z)
        encoded = self._encode_wheel_commands(wheel_cmds)

        try:
            for name in WHEEL_NAMES:
                payload = bytes((CMD_SET_SPEED, encoded[name] & 0xFF))
                self._send_frame(COMMAND_IDS[name], payload)
        except OSError as exc:
            self.get_logger().error(f'CAN 发送失败: {exc}')
            self._close_can_socket()

    def _recv_loop(self):
        while self._running.is_set():
            if self.can_socket is None:
                if not self._open_can_socket():
                    time.sleep(self.reconnect_interval)
                    continue
            try:
                frame = self.can_socket.recv(CAN_FRAME_SIZE)
                if len(frame) < CAN_FRAME_SIZE:
                    continue
                can_id, dlc, payload = struct.unpack(CAN_FRAME_FORMAT, frame)
                self._handle_frame(can_id & CAN_ID_MASK, dlc, payload[:dlc])
            except socket.timeout:
                continue
            except OSError as exc:
                if self._running.is_set():
                    self.get_logger().error(f'CAN 接收失败: {exc}')
                self._close_can_socket()
                time.sleep(self.reconnect_interval)

    def _handle_frame(self, can_id: int, dlc: int, payload: bytes):
        wheel_name = STATUS_IDS.get(can_id)
        if wheel_name is None or dlc < 8:
            return

        velocity_raw, encoder_low16, pwm = struct.unpack('<hHh', payload[:6])
        target_speed = struct.unpack('<b', payload[6:7])[0]
        flags = payload[7]
        stamp = time.monotonic()

        with self.state_lock:
            state = self.wheel_states[wheel_name]
            state.velocity_raw = velocity_raw
            state.encoder_low16 = encoder_low16
            state.encoder_full = self._extend_encoder(state.encoder_full, encoder_low16, state.encoder_initialized)
            state.encoder_initialized = True
            state.pwm = pwm
            state.target_speed = target_speed
            state.flags = flags
            state.wheel_speed_mps = self._raw_speed_to_mps(velocity_raw)
            state.stamp = stamp

            feedback = self._compute_chassis_feedback_locked()

        self._publish_feedback(feedback)

    @staticmethod
    def _extend_encoder(previous_full: int, current_low16: int, initialized: bool) -> int:
        if not initialized:
            return current_low16
        previous_low16 = previous_full & 0xFFFF
        delta = current_low16 - previous_low16
        if delta > 32767:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return previous_full + delta

    def _raw_speed_to_mps(self, velocity_raw: int) -> float:
        rev_per_second = (
            float(velocity_raw)
            / self.encoder_ticks_per_rev
            / self.speed_sample_period_s
        )
        return rev_per_second * self.wheel_circumference_m

    def _inverse_kinematics(self, vx: float, vy: float, wz: float) -> Dict[str, float]:
        radius = self.wheel_center_radius_m
        return {
            # X-drive with 90 deg omni wheels: wheel axes point toward the chassis center.
            # For pure +vy motion, wheel commands should be lf-, lr+, rf+, rr-.
            'lf': (vx - vy) * INV_SQRT2 - radius * wz,
            'lr': (vx + vy) * INV_SQRT2 - radius * wz,
            'rf': -(vx - vy) * INV_SQRT2 - radius * wz,
            'rr': -(vx + vy) * INV_SQRT2 - radius * wz,
        }

    def _encode_wheel_commands(self, wheel_cmds: Dict[str, float]) -> Dict[str, int]:
        scaled = {
            name: (
                speed
                / self.wheel_circumference_m
                * self.encoder_ticks_per_rev
                * self.speed_sample_period_s
            )
            for name, speed in wheel_cmds.items()
        }
        peak = max(abs(value) for value in scaled.values()) if scaled else 0.0
        if peak > float(self.max_motor_speed_cmd):
            scale = float(self.max_motor_speed_cmd) / peak
            scaled = {name: value * scale for name, value in scaled.items()}
        return {
            name: int(max(-self.max_motor_speed_cmd, min(self.max_motor_speed_cmd, round(value))))
            for name, value in scaled.items()
        }

    def _compute_chassis_feedback_locked(self) -> ChassisFeedback:
        wheel_speeds = {name: self.wheel_states[name].wheel_speed_mps for name in WHEEL_NAMES}
        radius = self.wheel_center_radius_m

        vx = (
            wheel_speeds['lf']
            + wheel_speeds['lr']
            - wheel_speeds['rf']
            - wheel_speeds['rr']
        ) / (2.0 * SQRT2)
        vy = (
            -wheel_speeds['lf']
            + wheel_speeds['lr']
            + wheel_speeds['rf']
            - wheel_speeds['rr']
        ) / (2.0 * SQRT2)
        wz = -sum(wheel_speeds.values()) / (4.0 * radius)

        self.feedback.vx = vx
        self.feedback.vy = vy
        self.feedback.wz = wz
        self.feedback.wheel_speeds = wheel_speeds
        return ChassisFeedback(vx=vx, vy=vy, wz=wz, wheel_speeds=dict(wheel_speeds))

    def _publish_feedback(self, feedback: ChassisFeedback):
        twist = Twist()
        twist.linear.x = feedback.vx
        twist.linear.y = feedback.vy
        twist.angular.z = feedback.wz
        self.current_vel_pub.publish(twist)

        wheel_speed_msg = Float32MultiArray()
        wheel_speed_msg.data = [feedback.wheel_speeds[name] for name in WHEEL_NAMES]
        self.wheel_speed_pub.publish(wheel_speed_msg)

        with self.state_lock:
            encoder_msg = Int32MultiArray()
            encoder_msg.data = [int(self.wheel_states[name].encoder_full) for name in WHEEL_NAMES]
        self.wheel_encoder_pub.publish(encoder_msg)

    def destroy_node(self):
        self._running.clear()
        if hasattr(self, 'send_timer'):
            self.send_timer.cancel()

        if self.can_socket is not None:
            try:
                for name in WHEEL_NAMES:
                    self._send_frame(COMMAND_IDS[name], bytes((CMD_SET_SPEED, 0x00)))
                time.sleep(0.05)
            except OSError:
                pass

        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)

        self._close_can_socket()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
