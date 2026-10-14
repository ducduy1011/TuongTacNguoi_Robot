#!/usr/bin/env python3
"""WASD Twist publisher at 10 Hz with a wall-clock keyboard timeout."""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


def main():
    if not sys.stdin.isatty():
        raise SystemExit('Open this command in an interactive Terminal.')
    rclpy.init()
    node = rclpy.create_node('teleop_wasd')
    speed = float(node.declare_parameter('speed', 0.3).value)
    turn = float(node.declare_parameter('turn', 0.5).value)
    timeout = float(node.declare_parameter('key_timeout', 0.8).value)
    if speed <= 0 or turn <= 0 or timeout <= 0:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit('speed, turn and key_timeout must be positive.')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    bindings = {'w': (speed, 0.0), 's': (-speed, 0.0),
                'a': (0.0, turn), 'd': (0.0, -turn)}
    twist = Twist()
    settings = termios.tcgetattr(sys.stdin.fileno())
    last_key = time.monotonic()
    next_publish = last_key
    print('W/S: forward/backward; A/D: turn; Space/X: stop; Ctrl+C: exit.')
    print(f'Hold/repeat keys. Publish 10 Hz; stop after {timeout:.1f}s without a key.')
    print(f'Linear speed: {speed:.2f} m/s; angular speed: {turn:.2f} rad/s.')
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            poll_time = max(0.0, min(0.03, next_publish - time.monotonic()))
            if select.select([sys.stdin], [], [], poll_time)[0]:
                key = sys.stdin.read(1).lower()
                if key == '\x03' or not key:
                    break
                twist.linear.x, twist.angular.z = bindings.get(key, (0.0, 0.0))
                last_key = time.monotonic()
                if key not in bindings:
                    pub.publish(twist)
            now = time.monotonic()
            if now - last_key > timeout:
                twist.linear.x = twist.angular.z = 0.0
            if now >= next_publish:
                pub.publish(twist)
                next_publish = now + 0.1
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                pub.publish(Twist())
                time.sleep(0.03)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
