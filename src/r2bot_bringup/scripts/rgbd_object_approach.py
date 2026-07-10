#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class RgbdObjectApproach(Node):
    def __init__(self):
        super().__init__('rgbd_object_approach')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.color_topic = self.declare_parameter('color_topic', '/camera/color/image_raw').value
        self.depth_topic = self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw').value
        self.info_topic = self.declare_parameter('camera_info_topic', '/camera/color/camera_info').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.target_frame = self.declare_parameter('target_frame', 'base_link').value

        self.hsv_lower = np.array(self.declare_parameter('hsv_lower', [0, 90, 70]).value, dtype=np.uint8)
        self.hsv_upper = np.array(self.declare_parameter('hsv_upper', [10, 255, 255]).value, dtype=np.uint8)
        self.hsv_lower_2 = np.array(self.declare_parameter('hsv_lower_2', [170, 90, 70]).value, dtype=np.uint8)
        self.hsv_upper_2 = np.array(self.declare_parameter('hsv_upper_2', [180, 255, 255]).value, dtype=np.uint8)
        self.enable_second_hsv_range = bool(self.declare_parameter('enable_second_hsv_range', True).value)
        self.min_area_px = float(self.declare_parameter('min_area_px', 1200.0).value)
        self.depth_min_m = float(self.declare_parameter('depth_min_m', 0.15).value)
        self.depth_max_m = float(self.declare_parameter('depth_max_m', 3.0).value)
        self.depth_scale_m = float(self.declare_parameter('depth_scale_m', 0.001).value)
        self.desired_distance_m = float(self.declare_parameter('desired_distance_m', 0.6).value)
        self.stop_tolerance_m = float(self.declare_parameter('stop_tolerance_m', 0.05).value)
        self.center_tolerance_px = float(self.declare_parameter('center_tolerance_px', 35.0).value)
        self.linear_kp = float(self.declare_parameter('linear_kp', 0.6).value)
        self.angular_kp = float(self.declare_parameter('angular_kp', 1.2).value)
        self.max_linear_speed = float(self.declare_parameter('max_linear_speed', 0.25).value)
        self.max_reverse_speed = float(self.declare_parameter('max_reverse_speed', 0.12).value)
        self.max_angular_speed = float(self.declare_parameter('max_angular_speed', 0.8).value)
        self.approach_only_when_centered = bool(self.declare_parameter('approach_only_when_centered', True).value)
        self.search_when_lost = bool(self.declare_parameter('search_when_lost', False).value)
        self.search_angular_speed = float(self.declare_parameter('search_angular_speed', 0.25).value)
        self.target_timeout_s = float(self.declare_parameter('target_timeout_s', 0.6).value)
        self.depth_window_px = int(self.declare_parameter('depth_window_px', 5).value)
        self.publish_debug_image = bool(self.declare_parameter('publish_debug_image', True).value)

        self.have_camera_info = False
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

        self.last_detection_time = self.get_clock().now() - Duration(seconds=10.0)
        self.last_cmd_was_stop = False

        self.info_sub = self.create_subscription(CameraInfo, self.info_topic, self.on_camera_info, 10)
        self.color_sub = Subscriber(self, Image, self.color_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.08)
        self.sync.registerCallback(self.on_images)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.target_pub = self.create_publisher(PointStamped, '~/target_point', 10)
        self.tracking_pub = self.create_publisher(Bool, '~/tracking', 10)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 10)
        self.safety_timer = self.create_timer(0.1, self.on_safety_timer)

    def on_camera_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.have_camera_info = self.fx > 1.0 and self.fy > 1.0

    def on_images(self, color_msg: Image, depth_msg: Image):
        color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        mask = self.segment_target(color)
        contour = self.find_largest_contour(mask)
        tracking = Bool()
        tracking.data = contour is not None
        self.tracking_pub.publish(tracking)

        debug = color.copy() if self.publish_debug_image else None

        if contour is None:
            self.handle_target_lost(debug, 'no contour')
            return

        x, y, w, h = cv2.boundingRect(contour)
        center_px = (x + w // 2, y + h // 2)
        depth_m = self.compute_depth(depth, center_px, mask)
        if depth_m is None:
            self.handle_target_lost(debug, 'invalid depth')
            return

        point = self.project_to_camera_point(center_px, depth_m, color_msg.header.frame_id)
        if point is not None:
            self.publish_target_point(point)

        image_center_x = color.shape[1] * 0.5
        pixel_error_x = float(center_px[0]) - image_center_x
        angular = self.clamp(-self.angular_kp * (pixel_error_x / image_center_x), -self.max_angular_speed, self.max_angular_speed)

        distance_error = depth_m - self.desired_distance_m
        linear = self.compute_linear_speed(distance_error, pixel_error_x)

        if abs(distance_error) <= self.stop_tolerance_m and abs(pixel_error_x) <= self.center_tolerance_px:
            linear = 0.0
            angular = 0.0

        self.publish_cmd(linear, angular)
        self.last_detection_time = self.get_clock().now()

        if debug is not None:
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug, center_px, 4, (0, 255, 255), -1)
            cv2.putText(debug, f'depth={depth_m:.2f}m', (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug, f'cmd vx={linear:.2f} wz={angular:.2f}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def on_safety_timer(self):
        age = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if age <= self.target_timeout_s:
            return
        if self.search_when_lost:
            self.publish_cmd(0.0, self.search_angular_speed)
        else:
            self.publish_stop_once()

    def segment_target(self, color: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        if self.enable_second_hsv_range:
            mask |= cv2.inRange(hsv, self.hsv_lower_2, self.hsv_upper_2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def find_largest_contour(self, mask: np.ndarray) -> Optional[np.ndarray]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self.min_area_px:
            return None
        return contour

    def compute_depth(self, depth: np.ndarray, center_px: Tuple[int, int], mask: np.ndarray) -> Optional[float]:
        cx, cy = center_px
        radius = max(1, self.depth_window_px)
        x0 = max(0, cx - radius)
        y0 = max(0, cy - radius)
        x1 = min(depth.shape[1], cx + radius + 1)
        y1 = min(depth.shape[0], cy + radius + 1)

        depth_roi = depth[y0:y1, x0:x1]
        mask_roi = mask[y0:y1, x0:x1] > 0
        if depth_roi.size == 0:
            return None

        if mask_roi.any():
            values = depth_roi[mask_roi]
        else:
            values = depth_roi.reshape(-1)

        values = values[np.isfinite(values)]
        if values.size == 0:
            return None

        if depth_roi.dtype == np.uint16:
            values_m = values.astype(np.float32) * self.depth_scale_m
        else:
            values_m = values.astype(np.float32)

        values_m = values_m[(values_m >= self.depth_min_m) & (values_m <= self.depth_max_m)]
        if values_m.size == 0:
            return None
        return float(np.median(values_m))

    def project_to_camera_point(self, center_px: Tuple[int, int], depth_m: float, frame_id: str) -> Optional[PointStamped]:
        if not self.have_camera_info:
            return None
        u, v = center_px
        x = (float(u) - self.cx) * depth_m / self.fx
        y = (float(v) - self.cy) * depth_m / self.fy

        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = frame_id
        point.point.x = x
        point.point.y = y
        point.point.z = depth_m
        return point

    def publish_target_point(self, camera_point: PointStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_point.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            target_point = do_transform_point(camera_point, transform)
            self.target_pub.publish(target_point)
        except TransformException:
            self.target_pub.publish(camera_point)

    def compute_linear_speed(self, distance_error: float, pixel_error_x: float) -> float:
        if self.approach_only_when_centered and abs(pixel_error_x) > self.center_tolerance_px:
            return 0.0

        raw = self.linear_kp * distance_error
        if raw >= 0.0:
            return self.clamp(raw, 0.0, self.max_linear_speed)
        return self.clamp(raw, -self.max_reverse_speed, 0.0)

    def handle_target_lost(self, debug: Optional[np.ndarray], reason: str):
        if debug is not None:
            cv2.putText(debug, f'lost: {reason}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        if self.search_when_lost:
            self.publish_cmd(0.0, self.search_angular_speed)
        else:
            self.publish_stop_once()

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        self.last_cmd_was_stop = math.isclose(linear_x, 0.0, abs_tol=1e-4) and math.isclose(angular_z, 0.0, abs_tol=1e-4)

    def publish_stop_once(self):
        if self.last_cmd_was_stop:
            return
        self.publish_cmd(0.0, 0.0)

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)
    node = RgbdObjectApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
