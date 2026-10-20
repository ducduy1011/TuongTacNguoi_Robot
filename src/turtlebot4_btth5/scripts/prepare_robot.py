#!/usr/bin/env python3
"""Initialize a fresh simulation at its dock, wait for readiness, then undock.

Only used by demo.launch.py. Navigation goals remain in navigate_goals.py.
Never reassign the origin after the robot has left its starting dock.
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter as RosParameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import DockStatus
from irobot_create_msgs.action import Undock
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener


class PrepareRobot(Node):
    def __init__(self):
        super().__init__('prepare_simulation_robot')
        self.declare_parameter('startup_pause', 5.0)
        self.odom = None
        self.dock = None
        self.pose = None
        self.yolo_ready = False
        self.goal_handle = None
        self.readiness_futures = {}
        self.readiness_values = {}
        self.readiness_poll_times = {}
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.create_subscription(DockStatus, '/dock_status', lambda m: setattr(self, 'dock', m), qos_profile_sensor_data)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: setattr(self, 'pose', m), qos)
        self.create_subscription(Image, '/yolo/image', lambda m: setattr(self, 'yolo_ready', True), 1)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.state_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                              for name in ('amcl', 'bt_navigator', 'planner_server', 'controller_server')}
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.parameters_client = self.create_client(SetParameters, '/motion_control/set_parameters')
        self.undock_client = ActionClient(self, Undock, '/undock')

    def wait(self, predicate, timeout=120.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('Readiness timed out. Check the preceding Gazebo/AMCL/Nav2/YOLO logs.')

    def poll_service(self, name, client, request, check):
        now = time.monotonic()
        entry = self.readiness_futures.get(name)
        if entry is not None:
            future, sent = entry
            if future.done():
                self.readiness_values[name] = check(future.result())
                del self.readiness_futures[name]
            elif now - sent > 15:
                future.cancel()
                del self.readiness_futures[name]
        if (name not in self.readiness_futures and client.service_is_ready()
                and now - self.readiness_poll_times.get(name, 0) >= 1):
            self.readiness_futures[name] = (client.call_async(request()), now)
            self.readiness_poll_times[name] = now
        return self.readiness_values.get(name, False)

    def state(self, name):
        return self.poll_service(name, self.state_clients[name], GetState.Request,
                                 lambda response: response is not None and response.current_state.id == 3)

    def control_ready(self):
        def check(response):
            if response is None:
                return False
            states = {c.name: c.state for c in response.controller}
            return all(states.get(n) == 'active' for n in ('joint_state_broadcaster', 'diffdrive_controller'))
        return self.poll_service('wheel_controllers', self.controllers, ListControllers.Request, check)

    def localization_ready(self):
        if self.pose is None or not self.tf.can_transform('map', 'base_link', rclpy.time.Time()):
            return False
        values = [self.pose.pose.covariance[i] for i in (0, 7, 35)]
        return all(math.isfinite(v) and 0 <= v <= limit for v, limit in zip(values, (.49, .49, .25)))

    def run(self):
        self.get_logger().info('Waiting for the fresh maze simulation, active wheel controllers and AMCL.')
        self.wait(lambda: self.odom is not None and self.dock is not None
                  and self.get_clock().now().nanoseconds > 200000000)
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        if not self.dock.is_docked or math.hypot(p.x, p.y) > .1 or abs(yaw) > .1:
            raise RuntimeError('Robot is not at its fresh starting dock. Stop the old session first; '
                               'refusing to reset its pose to the origin.')
        self.wait(self.control_ready)
        self.wait(lambda: self.state('amcl') and self.initial_pose_pub.get_subscription_count() > 0)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = (self.get_clock().now() - Duration(seconds=.1)).to_msg()
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = pose.pose.covariance[7] = .04
        pose.pose.covariance[35] = .01
        self.initial_pose_pub.publish(pose)
        self.get_logger().info('Initial pose (0,0,0) set at the starting dock. Waiting for AMCL/TF and Nav2.')
        self.wait(lambda: self.localization_ready() and all(self.state(n) for n in self.state_clients))
        self.wait(lambda: self.yolo_ready)
        self.get_logger().info('Localization, Nav2 and YOLO ready. Showing the loaded map before undocking.')
        end = time.monotonic() + self.get_parameter('startup_pause').value
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.1)
        self.wait(self.undock_client.server_is_ready, 15)
        future = self.undock_client.send_goal_async(Undock.Goal())
        self.wait(future.done, 15)
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            raise RuntimeError('Undock was rejected.')
        result = self.goal_handle.get_result_async()
        self.wait(result.done, 45)
        if result.result().status != GoalStatus.STATUS_SUCCEEDED or result.result().result.is_docked:
            raise RuntimeError('Undock failed; navigation will not start.')
        self.goal_handle = None
        self.wait(lambda: self.dock is not None and not self.dock.is_docked, 5)
        self.wait(self.parameters_client.service_is_ready, 10)
        request = SetParameters.Request()
        request.parameters = [RosParameter('safety_override', value='backup_only').to_parameter_msg()]
        future = self.parameters_client.call_async(request)
        self.wait(future.done, 10)
        if len(future.result().results) != 1 or not future.result().results[0].successful:
            raise RuntimeError('Could not set the existing simulation safety configuration.')
        self.get_logger().info('Undock SUCCEEDED. Starting the three-goal Python navigation program.')

    def cancel(self):
        if self.goal_handle is not None and self.goal_handle.accepted:
            future = self.goal_handle.cancel_goal_async()
            self.wait(future.done, 5)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PrepareRobot()
    code = 0
    try:
        node.run()
    except KeyboardInterrupt:
        code = 1
    except Exception as error:
        node.get_logger().error(str(error))
        code = 1
    finally:
        try:
            node.cancel()
        finally:
            node.listener.unregister()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
