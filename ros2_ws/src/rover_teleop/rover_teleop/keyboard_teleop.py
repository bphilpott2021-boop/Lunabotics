#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# Cross-platform non-blocking keyboard input
class KeyReader:
    def __init__(self):
        self._posix = False
        self._win = False
        self._old_settings = None
        try:
            import msvcrt  # type: ignore
            self._msvcrt = msvcrt
            self._win = True
        except Exception:
            self._msvcrt = None
            try:
                import termios  # type: ignore
                import tty  # type: ignore
                import select  # type: ignore
                self._termios = termios
                self._tty = tty
                self._select = select
                self._posix = True
            except Exception:
                pass

    def enter_raw(self):
        if self._posix:
            self._old_settings = self._termios.tcgetattr(sys.stdin)
            self._tty.setcbreak(sys.stdin.fileno())

    def exit_raw(self):
        if self._posix and self._old_settings is not None:
            self._termios.tcsetattr(sys.stdin, self._termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    def get_key(self, timeout_sec: float = 0.05):
        # Returns a single-character string or None if nothing pressed
        if self._win and self._msvcrt is not None:
            # Poll up to timeout
            end = time.monotonic() + max(0.0, timeout_sec)
            while time.monotonic() < end:
                if self._msvcrt.kbhit():
                    try:
                        ch = self._msvcrt.getwch()
                    except Exception:
                        ch = self._msvcrt.getch().decode(errors='ignore')
                    return ch
                time.sleep(0.01)
            return None
        elif self._posix:
            rlist, _, _ = self._select.select([sys.stdin], [], [], timeout_sec)
            if rlist:
                try:
                    ch = sys.stdin.read(1)
                except Exception:
                    ch = ''
                return ch
            return None
        else:
            # Fallback: blocking input (not ideal). Only check once per timeout.
            time.sleep(timeout_sec)
            return None


HELP_TEXT = (
    "Keyboard Teleop (fixed-speed)\n\n"
    "Controls:\n"
    "  w: forward toggle\n"
    "  s or space: stop\n"
    "  a: turn left (in-place)\n"
    "  d: turn right (in-place)\n"
    "  x: stop rotation (keep current forward state)\n"
    "  q: quit\n\n"
    "Publishes geometry_msgs/Twist on /cmd_vel at a fixed rate.\n"
)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Parameters
        self.declare_parameter('forward_speed', 0.2)        # m/s
        self.declare_parameter('angular_speed', 0.6)        # rad/s
        self.declare_parameter('accel', 0.5)                # m/s^2
        self.declare_parameter('publish_hz', 20.0)          # Hz
        self.declare_parameter('topic', 'cmd_vel')

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.accel = float(self.get_parameter('accel').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.topic = str(self.get_parameter('topic').value)

        self.pub = self.create_publisher(Twist, self.topic, 10)

        # State
        self._target_linear = 0.0
        self._current_linear = 0.0
        self._angular = 0.0
        self._last_time = time.monotonic()
        self._shutdown_requested = False

        # Keyboard reader
        self._keys = KeyReader()
        self._keys.enter_raw()
        self.get_logger().info(HELP_TEXT)

        # Timer: poll keys and publish twist at fixed rate
        period = 1.0 / self.publish_hz if self.publish_hz > 0 else 0.05
        self.timer = self.create_timer(period, self._on_timer)

    def destroy_node(self):
        try:
            self._keys.exit_raw()
        except Exception:
            pass
        return super().destroy_node()

    def _handle_key(self, ch: str):
        ch = ch.lower()
        if ch == 'q':
            self.get_logger().info('Quit requested')
            self._shutdown_requested = True
        elif ch == 'w':
            # Toggle forward
            new_target = 0.0 if self._target_linear > 0.0 else self.forward_speed
            self._target_linear = new_target
            self.get_logger().info(f"Forward {'ON' if new_target > 0 else 'OFF'}")
        elif ch in ('s', ' '):
            # Stop linear immediately (target 0; ramp handles decay)
            self._target_linear = 0.0
            self.get_logger().info('Forward OFF')
        elif ch == 'a':
            self._angular = +self.angular_speed
            self.get_logger().info('Turning LEFT')
        elif ch == 'd':
            self._angular = -self.angular_speed
            self.get_logger().info('Turning RIGHT')
        elif ch == 'x':
            self._angular = 0.0
            self.get_logger().info('Rotation OFF')
        else:
            # Ignore other keys
            pass

    # Timer callback: poll keys, update ramp, and publish current twist
    def _on_timer(self):
        # Drain any pending keypresses without blocking
        while True:
            ch = self._keys.get_key(0.0)
            if not ch:
                break
            self._handle_key(ch)

        if self._shutdown_requested:
            # Request shutdown of the ROS context; spin() will exit
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        now = time.monotonic()
        dt = max(0.0, now - self._last_time)
        self._last_time = now

        # Ramp linear speed toward target
        if self.accel > 0.0:
            delta = self._target_linear - self._current_linear
            max_step = self.accel * dt
            if abs(delta) <= max_step:
                self._current_linear = self._target_linear
            else:
                self._current_linear += max_step if delta > 0 else -max_step
        else:
            self._current_linear = self._target_linear

        twist = Twist()
        twist.linear.x = self._current_linear
        twist.angular.z = self._angular
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # In case shutdown wasn't already requested from within the node
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

