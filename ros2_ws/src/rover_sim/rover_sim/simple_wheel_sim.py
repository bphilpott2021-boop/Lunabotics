#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray


class SimpleWheelSim(Node):
    def __init__(self):
        super().__init__('simple_wheel_sim')

        # Parameters (tune as needed)
        self.declare_parameter('wheel_radius', 0.06)     # meters
        self.declare_parameter('wheel_base', 0.35)       # meters
        self.declare_parameter('ticks_per_rev', 2048)    # encoder counts per wheel rev
        self.declare_parameter('update_rate', 50.0)      # Hz
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.R = float(self.get_parameter('wheel_radius').value)
        self.B = float(self.get_parameter('wheel_base').value)
        self.TPR = int(self.get_parameter('ticks_per_rev').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self._left_tick_remainder = 0.0
        self._right_tick_remainder = 0.0
        self._last_time = time.monotonic()
        self._last_debug = self._last_time

        # IO (subscribe to absolute topic to avoid name resolution surprises)
        self.sub = self.create_subscription(Twist, '/motor/cmd_vel', self._on_cmd, 10)
        self.pub_ticks = self.create_publisher(Int32MultiArray, 'wheel_ticks', 10)
        self.pub_odom = self.create_publisher(Odometry, 'wheel_odom_raw', 10)

        period = 1.0 / max(1e-3, float(self.get_parameter('update_rate').value))
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info('SimpleWheelSim started: listening on /motor/cmd_vel')

    def _on_cmd(self, msg: Twist):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)
        # Debug: confirm we are receiving commands
        self.get_logger().info(f"RX cmd: v={self.v_cmd:.2f}, w={self.w_cmd:.2f}")

    def _on_timer(self):
        now = time.monotonic()
        dt = max(0.0, now - self._last_time)
        self._last_time = now

        # Differential drive kinematics
        v = self.v_cmd
        w = self.w_cmd
        v_l = v - (w * self.B / 2.0)
        v_r = v + (w * self.B / 2.0)
        w_l = v_l / self.R
        w_r = v_r / self.R

        # Integrate pose
        if abs(w) < 1e-6:
            # Straight line
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
        else:
            # Circular arc
            dtheta = w * dt
            R_icc = v / w if abs(w) > 1e-9 else 0.0
            self.x += R_icc * (math.sin(self.yaw + dtheta) - math.sin(self.yaw))
            self.y += -R_icc * (math.cos(self.yaw + dtheta) - math.cos(self.yaw))
            self.yaw = (self.yaw + dtheta + math.pi) % (2.0 * math.pi) - math.pi

        # Update ticks
        d_ticks_l = (w_l * dt) * (self.TPR / (2.0 * math.pi)) + self._left_tick_remainder
        d_ticks_r = (w_r * dt) * (self.TPR / (2.0 * math.pi)) + self._right_tick_remainder
        inc_l = int(math.floor(d_ticks_l) if d_ticks_l >= 0 else math.ceil(d_ticks_l))
        inc_r = int(math.floor(d_ticks_r) if d_ticks_r >= 0 else math.ceil(d_ticks_r))
        self._left_tick_remainder = d_ticks_l - inc_l
        self._right_tick_remainder = d_ticks_r - inc_r
        self.left_ticks += inc_l
        self.right_ticks += inc_r

        # Publish ticks
        tick_msg = Int32MultiArray()
        tick_msg.data = [self.left_ticks, self.right_ticks]
        self.pub_ticks.publish(tick_msg)

        # Publish basic odom (pose + twist in base frame)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # Simple yaw to quaternion (roll=pitch=0)
        half = self.yaw * 0.5
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)

        # Throttled debug: show integration state once per ~1s
        if now - self._last_debug > 1.0:
            self._last_debug = now
            self.get_logger().info(
                f"State x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f} v={v:.2f} w={w:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleWheelSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
