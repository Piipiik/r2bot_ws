#!/usr/bin/env python3
import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from direction import DirectionEstimator as SerialDirectionEstimator
from direction import LD06Reader


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def direction_center_rad(name: str) -> float:
    mapping = {
        'front': 0.0,
        'left': math.pi / 2.0,
        'back': math.pi,
        'right': -math.pi / 2.0,
    }
    return mapping[name]


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Move the holonomic chassis to target obstacle distances and exit automatically.'
    )
    parser.add_argument('--front-mm', type=int, default=-1, help='target distance to front obstacle')
    parser.add_argument('--back-mm', type=int, default=-1, help='target distance to back obstacle')
    parser.add_argument('--left-mm', type=int, default=-1, help='target distance to left obstacle')
    parser.add_argument('--right-mm', type=int, default=-1, help='target distance to right obstacle')
    parser.add_argument('--tolerance-mm', type=int, default=20, help='allowed error for each target distance')
    parser.add_argument('--sector-deg', type=float, default=8.0, help='half sector width used for each direction')
    parser.add_argument('--control-rate', type=float, default=15.0, help='control loop rate in Hz')
    parser.add_argument('--kp-linear', type=float, default=0.003, help='linear proportional gain in m/s per mm error')
    parser.add_argument('--max-vx', type=float, default=0.15, help='maximum forward/backward speed in m/s')
    parser.add_argument('--max-vy', type=float, default=0.15, help='maximum lateral speed in m/s')
    parser.add_argument('--kp-yaw', type=float, default=1.8, help='yaw lock proportional gain')
    parser.add_argument('--max-wz', type=float, default=0.5, help='maximum angular speed in rad/s')
    parser.add_argument('--yaw-tolerance-deg', type=float, default=3.0, help='allowed heading drift before finish')
    parser.add_argument('--scan-topic', default='/scan', help='LaserScan topic')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel', help='Twist command topic')
    parser.add_argument('--odom-topic', default='/odom', help='Odometry topic used for heading lock')
    parser.add_argument('--timeout-s', type=float, default=20.0, help='overall timeout in seconds')
    parser.add_argument('--data-timeout-s', type=float, default=1.0, help='scan/odom freshness timeout in seconds')
    parser.add_argument('--source', choices=['auto', 'ros', 'serial'], default='auto', help='distance source')
    parser.add_argument('--serial-port', default='/dev/jlink_lidar', help='LD06 serial device path')
    parser.add_argument('--serial-baudrate', type=int, default=230400, help='LD06 serial baudrate')
    parser.add_argument('--serial-timeout', type=float, default=0.1, help='LD06 serial read timeout in seconds')
    parser.add_argument('--serial-offset', type=float, default=0.0, help='LD06 angle offset in degrees')
    parser.add_argument('--serial-window', type=float, default=10.0, help='half window in degrees for direct serial direction sectors')
    parser.add_argument('--serial-min-intensity', type=int, default=0, help='minimum LD06 point intensity for direct serial mode')
    parser.add_argument('--serial-max-distance-mm', type=int, default=12000, help='maximum LD06 point distance for direct serial mode')
    return parser


def collect_targets(args: argparse.Namespace) -> Dict[str, int]:
    targets: Dict[str, int] = {}
    for name in ('front', 'back', 'left', 'right'):
        value = getattr(args, f'{name}_mm')
        if value >= 0:
            if value == 0:
                raise ValueError(f'{name} target must be > 0 mm when provided')
            targets[name] = value

    if not targets:
        raise ValueError('at least one target direction must be provided')
    if len(targets) > 2:
        raise ValueError('at most two target directions are allowed')
    if 'front' in targets and 'back' in targets:
        raise ValueError('front and back cannot be given together')
    if 'left' in targets and 'right' in targets:
        raise ValueError('left and right cannot be given together')
    return targets


@dataclass
class AlignResult:
    success: bool
    reason: str
    targets_mm: Dict[str, int]
    final_mm: Dict[str, Optional[int]]
    error_mm: Dict[str, Optional[int]]

    def to_json(self) -> str:
        return json.dumps(
            {
                'success': self.success,
                'reason': self.reason,
                'targets_mm': self.targets_mm,
                'final_mm': self.final_mm,
                'error_mm': self.error_mm,
            },
            ensure_ascii=False,
        )


class AlignToObstacles(Node):
    def __init__(self, args: argparse.Namespace, targets_mm: Dict[str, int]):
        super().__init__('align_to_obstacles')
        self.targets_mm = targets_mm
        self.tolerance_mm = int(args.tolerance_mm)
        self.sector_deg = float(args.sector_deg)
        self.sector_rad = math.radians(self.sector_deg)
        self.kp_linear = float(args.kp_linear)
        self.max_vx = float(args.max_vx)
        self.max_vy = float(args.max_vy)
        self.kp_yaw = float(args.kp_yaw)
        self.max_wz = float(args.max_wz)
        self.yaw_tolerance_rad = math.radians(float(args.yaw_tolerance_deg))
        self.timeout_s = float(args.timeout_s)
        self.data_timeout_s = float(args.data_timeout_s)
        self.source_mode = str(args.source)

        self.scan_msg: Optional[LaserScan] = None
        self.scan_time_monotonic: Optional[float] = None
        self.serial_distances_mm: Optional[Dict[str, Optional[int]]] = None
        self.serial_time_monotonic: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.odom_time_monotonic: Optional[float] = None
        self.lock_yaw: Optional[float] = None
        self.start_time_monotonic = time.monotonic()
        self.result: Optional[AlignResult] = None
        self.stop_sent = False
        self._serial_error: Optional[str] = None
        self._serial_stop = threading.Event()
        self._serial_thread: Optional[threading.Thread] = None

        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.scan_sub = self.create_subscription(LaserScan, args.scan_topic, self.on_scan, scan_qos)
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)

        period = 1.0 / max(args.control_rate, 1.0)
        self.control_timer = self.create_timer(period, self.on_control)

        self.get_logger().info(f'targets(mm): {self.targets_mm}, tolerance={self.tolerance_mm} mm')
        self.get_logger().info(
            'scan direction uses ROS convention: front=0 deg, left=90 deg, back=180 deg, right=270 deg'
        )
        self.get_logger().info(f'distance source mode: {self.source_mode}')

        if self.source_mode in ('auto', 'serial'):
            self.start_serial_reader(args)

    def start_serial_reader(self, args: argparse.Namespace) -> None:
        def worker() -> None:
            try:
                reader = LD06Reader(
                    port=args.serial_port,
                    baudrate=args.serial_baudrate,
                    timeout=args.serial_timeout,
                    angle_offset_deg=args.serial_offset,
                )
            except Exception as exc:  # pragma: no cover - hardware dependent
                self._serial_error = str(exc)
                self.get_logger().warn(f'direct serial mode unavailable on {args.serial_port}: {exc}')
                return

            estimator = SerialDirectionEstimator(
                window_deg=args.serial_window,
                min_intensity=args.serial_min_intensity,
                min_distance_mm=1,
                max_distance_mm=args.serial_max_distance_mm,
            )
            self.get_logger().info(f'direct LD06 serial reader opened on {args.serial_port}')
            try:
                for points in reader.frames():
                    if self._serial_stop.is_set():
                        break
                    distances = estimator.update(points)
                    if distances is None:
                        continue
                    self.serial_distances_mm = {
                        'front': distances['front'],
                        'left': distances['left'],
                        'back': distances['back'],
                        'right': distances['right'],
                    }
                    self.serial_time_monotonic = time.monotonic()
            finally:
                reader.close()

        self._serial_thread = threading.Thread(target=worker, daemon=True, name='ld06_serial_reader')
        self._serial_thread.start()

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg
        self.scan_time_monotonic = time.monotonic()

    def on_odom(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        self.odom_time_monotonic = time.monotonic()
        if self.lock_yaw is None:
            self.lock_yaw = self.current_yaw

    def on_control(self) -> None:
        if self.result is not None:
            return

        now = time.monotonic()
        if now - self.start_time_monotonic > self.timeout_s:
            self.finish(False, 'timeout')
            return

        distances_mm = self.get_distances(now)
        if distances_mm is None:
            if self.source_mode == 'serial' and self._serial_error is not None:
                self.finish(False, f'serial open failed: {self._serial_error}')
                return
            if self.source_mode == 'ros' and self.scan_time_monotonic is not None and now - self.scan_time_monotonic > self.data_timeout_s:
                self.finish(False, 'scan timeout')
                return
            return

        for name in self.targets_mm:
            if distances_mm.get(name) is None:
                self.publish_stop()
                return

        x_error_mm = self.compute_axis_error_mm(distances_mm, 'front', 'back')
        y_error_mm = self.compute_axis_error_mm(distances_mm, 'left', 'right')

        vx = 0.0
        vy = 0.0
        if x_error_mm is not None and abs(x_error_mm) > self.tolerance_mm:
            vx = clamp(self.kp_linear * x_error_mm, -self.max_vx, self.max_vx)
        if y_error_mm is not None and abs(y_error_mm) > self.tolerance_mm:
            vy = clamp(self.kp_linear * y_error_mm, -self.max_vy, self.max_vy)

        yaw_error = 0.0
        wz = 0.0
        yaw_locked = True
        if self.lock_yaw is not None and self.current_yaw is not None:
            yaw_error = normalize_angle_rad(self.lock_yaw - self.current_yaw)
            yaw_locked = abs(yaw_error) <= self.yaw_tolerance_rad
            wz = clamp(self.kp_yaw * yaw_error, -self.max_wz, self.max_wz)

        x_ok = x_error_mm is None or abs(x_error_mm) <= self.tolerance_mm
        y_ok = y_error_mm is None or abs(y_error_mm) <= self.tolerance_mm
        done = x_ok and y_ok and yaw_locked

        if done:
            self.finish(True, 'aligned', distances_mm)
            return

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)
        self.stop_sent = False

    def compute_axis_error_mm(
        self,
        distances_mm: Dict[str, Optional[int]],
        positive_name: str,
        negative_name: str,
    ) -> Optional[int]:
        if positive_name in self.targets_mm:
            measured = distances_mm[positive_name]
            if measured is None:
                return None
            return measured - self.targets_mm[positive_name]
        if negative_name in self.targets_mm:
            measured = distances_mm[negative_name]
            if measured is None:
                return None
            return self.targets_mm[negative_name] - measured
        return None

    def get_distances(self, now: float) -> Optional[Dict[str, Optional[int]]]:
        ros_distances: Optional[Dict[str, Optional[int]]] = None
        serial_distances: Optional[Dict[str, Optional[int]]] = None

        if self.scan_msg is not None and self.scan_time_monotonic is not None:
            if now - self.scan_time_monotonic <= self.data_timeout_s:
                ros_distances = self.extract_direction_distances(self.scan_msg)

        if self.serial_distances_mm is not None and self.serial_time_monotonic is not None:
            if now - self.serial_time_monotonic <= self.data_timeout_s:
                serial_distances = dict(self.serial_distances_mm)

        if self.source_mode == 'ros':
            return ros_distances
        if self.source_mode == 'serial':
            return serial_distances
        if ros_distances is not None:
            return ros_distances
        return serial_distances

    def extract_direction_distances(self, scan: LaserScan) -> Dict[str, Optional[int]]:
        result: Dict[str, Optional[int]] = {}
        if not scan.ranges:
            return {'front': None, 'left': None, 'back': None, 'right': None}

        valid: List[tuple[float, float]] = []
        for index, range_m in enumerate(scan.ranges):
            if not math.isfinite(range_m):
                continue
            if range_m < scan.range_min or range_m > scan.range_max:
                continue
            angle = scan.angle_min + scan.angle_increment * index
            angle = normalize_angle_rad(angle)
            valid.append((angle, range_m))

        for name in ('front', 'left', 'back', 'right'):
            center = direction_center_rad(name)
            sector_ranges_mm = [
                int(round(range_m * 1000.0))
                for angle, range_m in valid
                if abs(normalize_angle_rad(angle - center)) <= self.sector_rad
            ]
            result[name] = min(sector_ranges_mm) if sector_ranges_mm else None
        return result

    def publish_stop(self) -> None:
        if self.stop_sent:
            return
        self.cmd_pub.publish(Twist())
        self.stop_sent = True

    def finish(self, success: bool, reason: str, final_mm: Optional[Dict[str, Optional[int]]] = None) -> None:
        if final_mm is None:
            final_mm = self.get_distances(time.monotonic())
            if final_mm is None:
                final_mm = {
                    'front': None,
                    'left': None,
                    'back': None,
                    'right': None,
                }

        error_mm: Dict[str, Optional[int]] = {}
        for name in ('front', 'back', 'left', 'right'):
            target = self.targets_mm.get(name)
            measured = final_mm.get(name)
            error_mm[name] = None if target is None or measured is None else measured - target

        self.publish_stop()
        self.result = AlignResult(
            success=success,
            reason=reason,
            targets_mm=dict(self.targets_mm),
            final_mm=final_mm,
            error_mm=error_mm,
        )
        self.get_logger().info(self.result.to_json())


def parse_args(argv: List[str]) -> tuple[argparse.Namespace, Dict[str, int]]:
    parser = build_argparser()
    args = parser.parse_args(argv)
    targets = collect_targets(args)
    return args, targets


def main(args=None) -> int:
    del args
    cli_args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    try:
        parsed_args, targets = parse_args(cli_args)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    rclpy.init(args=sys.argv)
    node = AlignToObstacles(parsed_args, targets)
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.finish(False, 'interrupted')
    finally:
        node.publish_stop()
        node._serial_stop.set()
        if node._serial_thread is not None and node._serial_thread.is_alive():
            node._serial_thread.join(timeout=1.0)
        result = node.result
        if result is not None:
            print(result.to_json(), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if result is None:
        return 1
    return 0 if result.success else 1


if __name__ == '__main__':
    sys.exit(main())
