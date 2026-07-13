#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformBroadcaster
import math
import serial
import struct
import threading
import time


G354_CONFIG_COMMANDS = (
    "FE 01 0D",
    "85 04 0D",
    "88 01 0D",
    "8C 06 0D",
    "8D F0 0D",
    "8F 70 0D",
    "FE 00 0D",
    "83 01 0D",
)
G354_FRAME_LEN = 36


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def blend_angles(primary: float, secondary: float, primary_weight: float) -> float:
    primary_weight = max(0.0, min(1.0, primary_weight))
    secondary_weight = 1.0 - primary_weight
    x = primary_weight * math.cos(primary) + secondary_weight * math.cos(secondary)
    y = primary_weight * math.sin(primary) + secondary_weight * math.sin(secondary)
    if abs(x) < 1e-12 and abs(y) < 1e-12:
        return normalize_angle(primary)
    return math.atan2(y, x)


class OdomCalculator(Node):
    def __init__(self):
        super().__init__('odom_calculator')
        self.declare_parameter('imu_port', '/dev/g354_imu')
        self.declare_parameter('imu_baudrate', 460800)
        self.declare_parameter('imu_weight', 0.98)
        self.declare_parameter('imu_timeout_s', 0.3)
        self.declare_parameter('enable_imu_fusion', True)

        self.imu_port = str(self.get_parameter('imu_port').value)
        self.imu_baudrate = int(self.get_parameter('imu_baudrate').value)
        self.imu_weight = float(self.get_parameter('imu_weight').value)
        self.imu_timeout_s = float(self.get_parameter('imu_timeout_s').value)
        self.enable_imu_fusion = bool(self.get_parameter('enable_imu_fusion').value)
        self.imu_weight = max(0.0, min(1.0, self.imu_weight))
        if self.imu_timeout_s <= 0.0:
            self.imu_timeout_s = 0.3

        self.sub = self.create_subscription(Twist, 'current_velocity', self.vel_callback, 10)
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.br = TransformBroadcaster(self)
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.vx = 0.0; self.vy = 0.0; self.vth = 0.0
        self.last_time = self.get_clock().now()
        self.vx_prev = 0.0; self.vy_prev = 0.0; self.vth_prev = 0.0
        self.pose_covariance = [
            0.0025, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0025, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        self.twist_covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0025,
        ]
        self.imu_lock = threading.Lock()
        self.imu_yaw = 0.0
        self.imu_last_time_monotonic = 0.0
        self.imu_previous_sample_time = None
        self.imu_initialized = False
        self.imu_packet_count = 0
        self.imu_stop = threading.Event()
        self.imu_thread = None
        if self.enable_imu_fusion:
            self.imu_thread = threading.Thread(target=self.imu_loop, daemon=True, name='g354_imu_reader')
            self.imu_thread.start()
            self.get_logger().info(
                'G354 IMU yaw fusion enabled: port=%s baud=%d imu_weight=%.3f odom_weight=%.3f'
                % (self.imu_port, self.imu_baudrate, self.imu_weight, 1.0 - self.imu_weight)
            )
        else:
            self.get_logger().info('G354 IMU yaw fusion disabled')
        self.create_timer(1.0 / 30.0, self.update_and_publish)

    def vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def imu_loop(self):
        while not self.imu_stop.is_set():
            try:
                self.read_g354_forever()
            except Exception as exc:
                self.get_logger().warn(f'G354 IMU read failed on {self.imu_port}: {exc}')
                time.sleep(1.0)

    def read_g354_forever(self):
        with serial.Serial(
            self.imu_port,
            self.imu_baudrate,
            timeout=1,
            rtscts=False,
            dsrdtr=False,
        ) as ser:
            self.get_logger().info(f'G354 IMU serial opened: {self.imu_port}')
            for cmd in G354_CONFIG_COMMANDS:
                ser.write(bytes.fromhex(cmd))
                time.sleep(0.06)

            buffer = bytearray()
            while not self.imu_stop.is_set():
                if ser.in_waiting > 0:
                    try:
                        rx_data = ser.read(ser.in_waiting)
                    except serial.SerialException as exc:
                        if 'returned no data' in str(exc):
                            time.sleep(0.001)
                            continue
                        raise
                    buffer.extend(rx_data)
                    while len(buffer) >= G354_FRAME_LEN:
                        if buffer[0] == 0x80 and buffer[G354_FRAME_LEN - 1] == 0x0D:
                            packet = bytes(buffer[:G354_FRAME_LEN])
                            del buffer[:G354_FRAME_LEN]
                            self.handle_g354_packet(packet)
                        else:
                            del buffer[0]
                else:
                    time.sleep(0.001)

    def handle_g354_packet(self, packet: bytes):
        gz_raw = struct.unpack('>h', packet[15:17])[0]
        gz_rad_s = math.radians(gz_raw * 0.016)
        now = time.monotonic()

        with self.imu_lock:
            if self.imu_previous_sample_time is None:
                dt = 0.008
            else:
                dt = max(0.001, min(now - self.imu_previous_sample_time, 0.05))
            self.imu_previous_sample_time = now

            if not self.imu_initialized:
                self.imu_yaw = self.th
                self.imu_initialized = True
            else:
                self.imu_yaw = normalize_angle(self.imu_yaw + gz_rad_s * dt)
            self.imu_last_time_monotonic = now
            self.imu_packet_count += 1

        if self.imu_packet_count == 1:
            self.get_logger().info('G354 IMU first valid frame received')

    def update_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now
        vx_avg = (self.vx_prev + self.vx) / 2.0
        vy_avg = (self.vy_prev + self.vy) / 2.0
        vth_avg = (self.vth_prev + self.vth) / 2.0
        theta_new = self.th + vth_avg * dt
        theta_avg = (self.th + theta_new) / 2.0
        self.x += (vx_avg * math.cos(theta_avg) - vy_avg * math.sin(theta_avg)) * dt
        self.y += (vx_avg * math.sin(theta_avg) + vy_avg * math.cos(theta_avg)) * dt
        self.th = normalize_angle(self.fuse_yaw(theta_new))
        self.vx_prev = self.vx; self.vy_prev = self.vy; self.vth_prev = self.vth
        q = yaw_to_quaternion(self.th)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.pose.covariance = self.pose_covariance
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        odom.twist.covariance = self.twist_covariance
        self.pub.publish(odom)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.br.sendTransform(t)

    def fuse_yaw(self, odom_yaw: float) -> float:
        if not self.enable_imu_fusion:
            return odom_yaw

        now = time.monotonic()
        with self.imu_lock:
            imu_ready = self.imu_initialized and now - self.imu_last_time_monotonic <= self.imu_timeout_s
            imu_yaw = self.imu_yaw

        if not imu_ready:
            return odom_yaw
        return blend_angles(imu_yaw, odom_yaw, self.imu_weight)

    def destroy_node(self):
        self.imu_stop.set()
        if self.imu_thread is not None and self.imu_thread.is_alive():
            self.imu_thread.join(timeout=1.0)
        super().destroy_node()

def main():
    rclpy.init()
    node = OdomCalculator()
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
