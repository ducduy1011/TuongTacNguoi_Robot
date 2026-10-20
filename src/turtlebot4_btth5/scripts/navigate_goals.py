#!/usr/bin/env python3
"""Send one NavigateToPose at a time; stop the sequence on any failure."""
import math
from pathlib import Path
import time

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import DockStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException


def load_goals(path):
    data = yaml.safe_load(Path(path).expanduser().read_text())
    goals = data.get('goals') if isinstance(data, dict) else None
    if not isinstance(goals, list) or len(goals) < 3:
        raise ValueError('goals.yaml must contain at least 3 goals.')
    for goal in goals:
        if not isinstance(goal, dict) or not all(
                isinstance(goal.get(k), (float, int)) and not isinstance(goal[k], bool)
                and math.isfinite(goal[k]) for k in ('x', 'y', 'yaw')):
            raise ValueError('Every goal needs finite x, y, yaw (radians).')
    if len({(g['x'], g['y']) for g in goals}) < 3:
        raise ValueError('Choose at least 3 different positions.')
    return goals


class GoalSequence(Node):
    def __init__(self):
        super().__init__('navigate_goals')
        default = str(Path(get_package_share_directory('turtlebot4_btth5')) / 'config/goals.yaml')
        self.declare_parameter('goals_file', default)
        self.declare_parameter('ready_timeout', 120.0)
        self.declare_parameter('goal_timeout', 360.0)
        self.declare_parameter('pause_at_goal', 3.0)
        self.declare_parameter('max_position_std', 0.7)
        self.declare_parameter('max_yaw_std', 0.5)
        self.goals = load_goals(self.get_parameter('goals_file').value)
        for key in ('ready_timeout', 'goal_timeout', 'pause_at_goal', 'max_position_std', 'max_yaw_std'):
            value = self.get_parameter(key).value
            if not math.isfinite(value) or value < 0 or (key != 'pause_at_goal' and value == 0):
                raise ValueError(f'Invalid {key}.')
        self.pose = None
        self.docked = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, qos)
        self.create_subscription(DockStatus, '/dock_status', self.on_dock, qos_profile_sensor_data)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.state_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                         for name in ('amcl', 'bt_navigator', 'planner_server', 'controller_server')}
        self.pending = None
        self.goal_handle = None
        self.last_feedback = 0.0

    def on_pose(self, msg):
        if msg.header.frame_id == 'map':
            self.pose = msg

    def on_dock(self, msg):
        self.docked = msg.is_docked

    def pose_is_confident(self):
        if self.pose is None:
            return False
        covariance = self.pose.pose.covariance
        limits = [self.get_parameter('max_position_std').value ** 2] * 2
        limits.append(self.get_parameter('max_yaw_std').value ** 2)
        return all(math.isfinite(covariance[index]) and 0 <= covariance[index] <= limit
                   for index, limit in zip((0, 7, 35), limits))

    def wait_for(self, predicate, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(predicate())

    def wait_ready(self):
        self.get_logger().info('Waiting for AMCL, map->base_link, active Nav2 and undocked robot. '
                               'Set 2D Pose Estimate, check scan alignment, then undock first.')
        end = time.monotonic() + self.get_parameter('ready_timeout').value
        requests = {}
        states = {}
        last_poll = 0.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            for name, future in list(requests.items()):
                if future.done():
                    states[name] = future.result().current_state.id
                    del requests[name]
            if time.monotonic() - last_poll >= 1:
                for name, client in self.state_clients.items():
                    if name not in requests and client.service_is_ready():
                        requests[name] = client.call_async(GetState.Request())
                last_poll = time.monotonic()
            try:
                transform = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                stamp = rclpy.time.Time.from_msg(transform.header.stamp)
                fresh = 0 <= (self.get_clock().now() - stamp).nanoseconds / 1e9 < 3
            except TransformException:
                fresh = False
            if (self.pose_is_confident() and self.docked is False and fresh
                    and all(states.get(n) == 3 for n in self.state_clients)
                    and self.client.server_is_ready()):
                return
        raise RuntimeError('Not ready: check AMCL pose uncertainty/TF, active Nav2, Play and /dock_status. '
                           'No navigation goal was sent.')

    def feedback(self, msg):
        if time.monotonic() - self.last_feedback >= 5:
            self.get_logger().info(f'Distance remaining: {msg.feedback.distance_remaining:.2f} m')
            self.last_feedback = time.monotonic()

    def cancel(self):
        # Also handle Ctrl+C while the server is acknowledging a new goal.
        if self.pending is not None and self.goal_handle is None:
            if self.wait_for(self.pending.done, 5.0):
                self.goal_handle = self.pending.result()
            else:
                self.get_logger().error('No goal acknowledgement; cancel status unknown. Stop the session.')
        if self.goal_handle is not None and self.goal_handle.accepted:
            future = self.goal_handle.cancel_goal_async()
            if self.wait_for(future.done, 5.0) and future.result().goals_canceling:
                result = self.goal_handle.get_result_async()
                if not self.wait_for(result.done, 10.0):
                    self.get_logger().error('Cancel accepted but result timed out; stop the session.')
            elif not future.done():
                self.get_logger().error('Cancel reply timed out; stop the session.')
            elif future.result().return_code != 3:  # ERROR_GOAL_TERMINATED: already finished.
                self.get_logger().error('Cancel was rejected; check robot and stop the session.')
        self.goal_handle = None

    def run(self):
        self.wait_ready()
        for number, goal in enumerate(self.goals, 1):
            if not self.pose_is_confident():
                raise RuntimeError('AMCL pose is too uncertain. Sequence stopped before sending '
                                   'the next goal; check scan/map alignment and re-localize.')
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'map'
            request.pose.header.stamp = self.get_clock().now().to_msg()
            request.pose.pose.position.x = float(goal['x'])
            request.pose.pose.position.y = float(goal['y'])
            request.pose.pose.orientation.z = math.sin(goal['yaw'] / 2)
            request.pose.pose.orientation.w = math.cos(goal['yaw'] / 2)
            self.get_logger().info(f"Goal {number}/{len(self.goals)}: "
                                   f"x={goal['x']}, y={goal['y']}, yaw={goal['yaw']} rad")
            self.pending = self.client.send_goal_async(request, feedback_callback=self.feedback)
            if not self.wait_for(self.pending.done, 20.0):
                raise RuntimeError('Goal acknowledgement timed out; sequence stopped.')
            self.goal_handle = self.pending.result()
            self.pending = None
            if not self.goal_handle.accepted:
                raise RuntimeError(f'Goal {number} REJECTED; no next goal sent.')
            result = self.goal_handle.get_result_async()
            if not self.wait_for(result.done, self.get_parameter('goal_timeout').value):
                raise RuntimeError(f'Goal {number} timed out; canceling, no next goal sent.')
            status = result.result().status
            self.goal_handle = None
            if status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Goal {number} failed (action status={status}); sequence stopped.')
            self.get_logger().info(f'Goal {number} SUCCEEDED')
            self.wait_for(lambda: False, self.get_parameter('pause_at_goal').value)
        self.get_logger().info(f'COMPLETE: {len(self.goals)}/{len(self.goals)} goals SUCCEEDED.')


def main():
    # Keep the context alive on Ctrl+C long enough to cancel the active goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = GoalSequence()
        node.run()
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Ctrl+C: canceling current goal; no next goal.')
        code = 0  # User stop is a normal launch shutdown, not a process failure.
    except Exception as error:
        if node:
            node.get_logger().error(str(error))
        else:
            print(f'[navigate_goals] {error}', flush=True)
        code = 1
    finally:
        if node:
            try:
                node.cancel()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
