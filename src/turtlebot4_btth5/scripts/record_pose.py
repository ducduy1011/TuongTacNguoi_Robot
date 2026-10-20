#!/usr/bin/env python3
"""Print map x/y/yaw from AMCL; Publish Point alone has no yaw."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    rclpy.init()
    node = Node('record_pose')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    def receive(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        node.get_logger().info(f'frame={msg.header.frame_id}: '
                               f'{{x: {p.x:.3f}, y: {p.y:.3f}, yaw: {yaw:.5f}}}')
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', receive, qos)
    node.get_logger().info('Recording AMCL pose: metres and yaw in radians. Ctrl+C to stop.')
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
