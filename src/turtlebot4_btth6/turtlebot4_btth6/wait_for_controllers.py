#!/usr/bin/env python3
"""Chi mo GUI sau khi hai controller cua robot da active."""
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node('wait_for_simulation_controllers')
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    future = None
    sent = 0.0
    deadline = time.monotonic() + 60.0
    try:
        node.get_logger().info('Dang cho controller robot san sang truoc khi mo Gazebo GUI...')
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if future is not None and future.done():
                result = future.result()
                states = {c.name: c.state for c in result.controller} if result else {}
                if all(states.get(name) == 'active' for name in (
                        'joint_state_broadcaster', 'diffdrive_controller')):
                    node.get_logger().info('ROBOT READY: hai controller active. Mo Gazebo GUI.')
                    return 0
                future = None
            elif future is not None and now - sent > 3.0:
                future.cancel()
                future = None
            if future is None and now - sent >= 0.5 and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
                sent = now
        node.get_logger().error('Controller khong san sang sau 60 giay; dung phien mo phong nay.')
        return 1
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
