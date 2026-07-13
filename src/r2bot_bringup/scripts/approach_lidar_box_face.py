#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Point:
    x: float
    y: float
    angle: float
    range_m: float


@dataclass
class FaceCandidate:
    points: List[Point]
    center_x: float
    center_y: float
    normal_x: float
    normal_y: float
    distance_m: float
    length_m: float
    rms_error_m: float

    @property
    def normal_angle_rad(self) -> float:
        return math.atan2(self.normal_y, self.normal_x)


@dataclass
class ApproachResult:
    success: bool
    reason: str
    final: dict

    def to_json(self) -> str:
        return json.dumps({'success': self.success, 'reason': self.reason, 'final': self.final}, ensure_ascii=False)


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Use front lidar scan to approach the center of a 1200 mm rectangular face.')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--target-length-mm', type=float, default=1200.0)
    parser.add_argument('--length-tolerance-mm', type=float, default=20.0)
    parser.add_argument('--target-distance-mm', type=float, default=200.0)
    parser.add_argument('--distance-tolerance-mm', type=float, default=20.0)
    parser.add_argument('--center-tolerance-mm', type=float, default=20.0)
    parser.add_argument('--target-center-offset-mm', type=float, default=0.0)
    parser.add_argument('--angle-tolerance-deg', type=float, default=2.0)
    parser.add_argument('--front-half-angle-deg', type=float, default=90.0)
    parser.add_argument('--min-range-mm', type=float, default=80.0)
    parser.add_argument('--max-range-mm', type=float, default=3000.0)
    parser.add_argument('--cluster-gap-mm', type=float, default=80.0)
    parser.add_argument('--min-cluster-points', type=int, default=18)
    parser.add_argument('--max-line-rms-mm', type=float, default=18.0)
    parser.add_argument('--control-rate', type=float, default=15.0)
    parser.add_argument('--kp-distance', type=float, default=1.1)
    parser.add_argument('--kp-center', type=float, default=1.0)
    parser.add_argument('--kp-angle', type=float, default=1.8)
    parser.add_argument('--max-vx', type=float, default=0.12)
    parser.add_argument('--max-vy', type=float, default=0.12)
    parser.add_argument('--max-wz', type=float, default=0.45)
    parser.add_argument('--timeout-s', type=float, default=30.0)
    parser.add_argument('--data-timeout-s', type=float, default=1.0)
    parser.add_argument('--status-interval-s', type=float, default=0.5)
    return parser


class LidarBoxFaceApproach(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__('approach_lidar_box_face')
        self.args = args
        self.target_length_m = args.target_length_mm / 1000.0
        self.length_tolerance_m = args.length_tolerance_mm / 1000.0
        self.target_distance_m = args.target_distance_mm / 1000.0
        self.distance_tolerance_m = args.distance_tolerance_mm / 1000.0
        self.center_tolerance_m = args.center_tolerance_mm / 1000.0
        self.target_center_offset_m = args.target_center_offset_mm / 1000.0
        self.angle_tolerance_rad = math.radians(args.angle_tolerance_deg)
        self.front_half_angle_rad = math.radians(args.front_half_angle_deg)
        self.min_range_m = args.min_range_mm / 1000.0
        self.max_range_m = args.max_range_mm / 1000.0
        self.cluster_gap_m = args.cluster_gap_mm / 1000.0
        self.max_line_rms_m = args.max_line_rms_mm / 1000.0
        self.scan_msg: Optional[LaserScan] = None
        self.scan_time: Optional[float] = None
        self.start_time = time.monotonic()
        self.last_status_time = 0.0
        self.result: Optional[ApproachResult] = None
        self.stop_sent = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.scan_sub = self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos)
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / max(args.control_rate, 1.0), self.on_control)
        self.get_logger().info(
            'front lidar face approach: angles [-%.1f, %.1f] deg, length %.0f +/- %.0f mm, distance %.0f mm'
            % (args.front_half_angle_deg, args.front_half_angle_deg, args.target_length_mm, args.length_tolerance_mm, args.target_distance_mm)
        )

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg
        self.scan_time = time.monotonic()

    def on_control(self) -> None:
        if self.result is not None:
            return
        now = time.monotonic()
        if self.args.timeout_s > 0.0 and now - self.start_time > self.args.timeout_s:
            self.finish(False, 'timeout')
            return
        if self.scan_msg is None or self.scan_time is None:
            return
        if now - self.scan_time > self.args.data_timeout_s:
            self.publish_stop()
            self.finish(False, 'scan timeout')
            return

        candidate = self.detect_face(self.scan_msg)
        if candidate is None:
            self.publish_stop()
            self.emit_status(None, 0.0, 0.0, 0.0, 'no matching face')
            return

        distance_error = candidate.distance_m - self.target_distance_m
        center_error = candidate.center_y - self.target_center_offset_m
        angle_error = normalize_angle_rad(candidate.normal_angle_rad)
        done = (
            abs(distance_error) <= self.distance_tolerance_m
            and abs(center_error) <= self.center_tolerance_m
            and abs(angle_error) <= self.angle_tolerance_rad
        )
        if done:
            self.emit_status(candidate, 0.0, 0.0, 0.0, 'aligned', force=True)
            self.finish(True, 'aligned', candidate)
            return

        cmd = Twist()
        cmd.linear.x = clamp(self.args.kp_distance * distance_error, -self.args.max_vx, self.args.max_vx)
        cmd.linear.y = clamp(self.args.kp_center * center_error, -self.args.max_vy, self.args.max_vy)
        cmd.angular.z = clamp(self.args.kp_angle * angle_error, -self.args.max_wz, self.args.max_wz)
        self.cmd_pub.publish(cmd)
        self.stop_sent = False
        self.emit_status(candidate, cmd.linear.x, cmd.linear.y, cmd.angular.z, 'tracking')

    def detect_face(self, scan: LaserScan) -> Optional[FaceCandidate]:
        points = self.scan_to_front_points(scan)
        if len(points) < self.args.min_cluster_points:
            return None
        candidates: List[FaceCandidate] = []
        for cluster in self.cluster_points(points):
            if len(cluster) < self.args.min_cluster_points:
                continue
            candidate = self.fit_line_segment(cluster)
            if candidate is None:
                continue
            if abs(candidate.length_m - self.target_length_m) > self.length_tolerance_m:
                continue
            if candidate.rms_error_m > self.max_line_rms_m:
                continue
            if abs(candidate.normal_angle_rad) > self.front_half_angle_rad:
                continue
            candidates.append(candidate)
        if not candidates:
            return None
        return min(candidates, key=lambda c: (abs(c.length_m - self.target_length_m), abs(c.center_y), c.distance_m))

    def scan_to_front_points(self, scan: LaserScan) -> List[Point]:
        points: List[Point] = []
        range_min = max(scan.range_min, self.min_range_m)
        range_max = min(scan.range_max, self.max_range_m)
        for index, range_m in enumerate(scan.ranges):
            if not math.isfinite(range_m) or range_m < range_min or range_m > range_max:
                continue
            angle = normalize_angle_rad(scan.angle_min + index * scan.angle_increment)
            if abs(angle) > self.front_half_angle_rad:
                continue
            x = range_m * math.cos(angle)
            y = range_m * math.sin(angle)
            if x <= 0.0:
                continue
            points.append(Point(x=x, y=y, angle=angle, range_m=range_m))
        points.sort(key=lambda p: p.angle)
        return points

    def cluster_points(self, points: Sequence[Point]) -> List[List[Point]]:
        clusters: List[List[Point]] = []
        current: List[Point] = []
        previous: Optional[Point] = None
        for point in points:
            if previous is None:
                current = [point]
            else:
                gap = math.hypot(point.x - previous.x, point.y - previous.y)
                if gap <= self.cluster_gap_m:
                    current.append(point)
                else:
                    clusters.append(current)
                    current = [point]
            previous = point
        if current:
            clusters.append(current)
        return clusters

    def fit_line_segment(self, points: Sequence[Point]) -> Optional[FaceCandidate]:
        count = len(points)
        if count < 2:
            return None
        mean_x = sum(p.x for p in points) / count
        mean_y = sum(p.y for p in points) / count
        sxx = sum((p.x - mean_x) ** 2 for p in points) / count
        syy = sum((p.y - mean_y) ** 2 for p in points) / count
        sxy = sum((p.x - mean_x) * (p.y - mean_y) for p in points) / count
        direction_angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        tx = math.cos(direction_angle)
        ty = math.sin(direction_angle)
        nx = -ty
        ny = tx
        if nx * mean_x + ny * mean_y < 0.0:
            nx = -nx
            ny = -ny
        projections = [(p.x - mean_x) * tx + (p.y - mean_y) * ty for p in points]
        length_m = max(projections) - min(projections)
        signed_distances = [(p.x - mean_x) * nx + (p.y - mean_y) * ny for p in points]
        rms_error_m = math.sqrt(sum(d * d for d in signed_distances) / count)
        distance_m = nx * mean_x + ny * mean_y
        if distance_m <= 0.0 or length_m <= 0.0:
            return None
        return FaceCandidate(list(points), mean_x, mean_y, nx, ny, distance_m, length_m, rms_error_m)

    def publish_stop(self) -> None:
        if self.stop_sent:
            return
        self.cmd_pub.publish(Twist())
        self.stop_sent = True

    def emit_status(self, candidate: Optional[FaceCandidate], vx: float, vy: float, wz: float, state: str, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_status_time < self.args.status_interval_s:
            return
        self.last_status_time = now
        if candidate is None:
            self.get_logger().info('status state=%s cmd={vx: %.3f, vy: %.3f, wz: %.3f}' % (state, vx, vy, wz))
            return
        length_mm = candidate.length_m * 1000.0
        distance_mm = candidate.distance_m * 1000.0
        center_y_mm = candidate.center_y * 1000.0
        length_error_mm = length_mm - self.args.target_length_mm
        distance_error_mm = distance_mm - self.args.target_distance_mm
        target_center_offset_mm = self.args.target_center_offset_mm
        center_error_mm = center_y_mm - target_center_offset_mm
        self.get_logger().info(
            'status state=%s length={measured: %.0fmm, target: %.0fmm, error: %.0fmm} '
            'distance={measured: %.0fmm, target: %.0fmm, error: %.0fmm} '
            'center_offset={measured: %.0fmm, target: %.0fmm, error: %.0fmm} '
            'angle=%.2fdeg rms=%.1fmm points=%d cmd={vx: %.3f, vy: %.3f, wz: %.3f}'
            % (
                state,
                length_mm,
                self.args.target_length_mm,
                length_error_mm,
                distance_mm,
                self.args.target_distance_mm,
                distance_error_mm,
                center_y_mm,
                target_center_offset_mm,
                center_error_mm,
                math.degrees(candidate.normal_angle_rad),
                candidate.rms_error_m * 1000.0,
                len(candidate.points),
                vx,
                vy,
                wz,
            )
        )

    def finish(self, success: bool, reason: str, candidate: Optional[FaceCandidate] = None) -> None:
        self.publish_stop()
        final = {}
        if candidate is not None:
            length_mm = candidate.length_m * 1000.0
            distance_mm = candidate.distance_m * 1000.0
            center_y_mm = candidate.center_y * 1000.0
            normal_angle_deg = math.degrees(candidate.normal_angle_rad)
            final = {
                'target_mm': {
                    'object_length': round(self.args.target_length_mm),
                    'distance': round(self.args.target_distance_mm),
                    'center_offset': round(self.args.target_center_offset_mm),
                },
                'measured_mm': {
                    'object_length': round(length_mm),
                    'distance': round(distance_mm),
                    'center_offset': round(center_y_mm),
                },
                'error_mm': {
                    'object_length': round(length_mm - self.args.target_length_mm),
                    'distance': round(distance_mm - self.args.target_distance_mm),
                    'center_offset': round(center_y_mm - self.args.target_center_offset_mm),
                },
                'tolerance_mm': {
                    'object_length': round(self.args.length_tolerance_mm),
                    'distance': round(self.args.distance_tolerance_mm),
                    'center_offset': round(self.args.center_tolerance_mm),
                },
                'normal_angle_deg': round(normal_angle_deg, 2),
                'normal_angle_error_deg': round(normal_angle_deg, 2),
                'angle_tolerance_deg': round(self.args.angle_tolerance_deg, 2),
                'line_rms_mm': round(candidate.rms_error_m * 1000.0, 1),
                'points': len(candidate.points),
            }
            self.get_logger().info(
                'final_summary success=%s reason=%s length=%s distance=%s center_offset=%s angle_error=%.2fdeg'
                % (
                    success,
                    reason,
                    final['measured_mm']['object_length'],
                    final['measured_mm']['distance'],
                    final['measured_mm']['center_offset'],
                    final['normal_angle_error_deg'],
                )
            )
        self.result = ApproachResult(success, reason, final)
        self.get_logger().info(self.result.to_json())


def main(args=None) -> int:
    del args
    parsed_args = build_argparser().parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(args=sys.argv)
    node = LidarBoxFaceApproach(parsed_args)
    result: Optional[ApproachResult] = None
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.finish(False, 'interrupted')
    finally:
        node.publish_stop()
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
