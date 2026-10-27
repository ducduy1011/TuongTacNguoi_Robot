"""/detected_gesture -> may trang thai -> /cmd_vel (20 Hz)."""
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .control import Behavior


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        self.control = Behavior()
        self.last_status = ''
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/robot_status', 1)
        self.create_subscription(String, '/detected_gesture', self.on_gesture, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        # Mac dinh dung wall clock; khong bat use_sim_time cho node nay.
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Ready: select gesture, then Thumb_Up. Open_Palm always stops.')

    def on_gesture(self, msg):
        was_returning = self.control.active == 'RETURN'
        self.control.gesture(msg.data, time.monotonic())
        # Dung hanh vi cu ngay; giu nam tay khong chen lenh zero khi dang ve.
        if msg.data == 'Open_Palm' or (msg.data == 'Closed_Fist' and not was_returning):
            self.send(0.0, 0.0)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.control.odometry(p.x, p.y, yaw, time.monotonic())

    def send(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.velocity.publish(msg)

    def tick(self):
        self.send(*self.control.tick(time.monotonic()))
        status = self.control.status()
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RobotBehaviorNode()
    running = True

    def request_stop(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    try:
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.send(0.0, 0.0)
            time.sleep(0.03)
        node.destroy_node()
        rclpy.shutdown()
