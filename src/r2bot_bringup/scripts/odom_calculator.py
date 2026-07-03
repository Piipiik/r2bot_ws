#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )

class OdomCalculator(Node):
    def __init__(self):
        super().__init__('odom_calculator')
        self.sub = self.create_subscription(Twist, 'current_velocity', self.vel_callback, 10)
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.br = TransformBroadcaster(self)
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.vx = 0.0; self.vy = 0.0; self.vth = 0.0
        self.last_time = self.get_clock().now()
        self.vx_prev = 0.0; self.vy_prev = 0.0; self.vth_prev = 0.0
        self.create_timer(1.0 / 30.0, self.update_and_publish)

    def vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

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
        self.th = theta_new
        self.vx_prev = self.vx; self.vy_prev = self.vy; self.vth_prev = self.vth
        q = yaw_to_quaternion(self.th)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        self.pub.publish(odom)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomCalculator()
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
