#!/usr/bin/env python3
"""Wait for an AMCL pose and a valid map transform before starting Nav2."""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node('btth4_wait_localization')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    ready = [False]
    localized = False
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                   lambda msg: ready.__setitem__(0, True), qos)
    node.get_logger().info('Dat 2D Pose Estimate tren RViz. Nav2 se mo khi AMCL va TF san sang.')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if ready[0] and buffer.can_transform('map', 'base_link', Time()):
                node.get_logger().info('Localization ready.')
                localized = True
                break
    except KeyboardInterrupt:
        return
    finally:
        listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not localized:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
