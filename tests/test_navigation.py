#!/usr/bin/env python3
"""Exercise the real node against fake Nav2 services/actions in isolated domain 147.

No Gazebo, robot process or velocity command is used. Source install/setup.bash
and run: ROS_DOMAIN_ID=147 python3 tests/test_navigation.py
"""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from irobot_create_msgs.msg import DockStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'install/turtlebot4_btth5/lib/turtlebot4_btth5/navigate_goals'


class NavigationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '147':
            raise RuntimeError('Use ROS_DOMAIN_ID=147: tests must not connect to the robot domain.')
        rclpy.init()
        cls.node = rclpy.create_node('fake_nav2_for_test')
        cls.group = ReentrantCallbackGroup()
        cls.received = []
        cls.canceled = 0
        cls.mode = 'success'
        cls.docked = False
        cls.pose_variance = 0.0
        cls.services = []
        for name in ('amcl', 'bt_navigator', 'planner_server', 'controller_server'):
            cls.services.append(cls.node.create_service(GetState, f'/{name}/get_state',
                                                        cls.state, callback_group=cls.group))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        cls.pose_pub = cls.node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', qos)
        cls.dock_pub = cls.node.create_publisher(DockStatus, '/dock_status', 10)
        cls.tf = TransformBroadcaster(cls.node)
        cls.timer = cls.node.create_timer(0.1, cls.publish, callback_group=cls.group)
        cls.server = ActionServer(cls.node, NavigateToPose, '/navigate_to_pose',
                                  execute_callback=cls.execute,
                                  goal_callback=cls.accept, cancel_callback=cls.cancel,
                                  callback_group=cls.group)
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def state(cls, request, response):
        response.current_state.id = 3
        return response

    @classmethod
    def publish(cls):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = cls.node.get_clock().now().to_msg()
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = cls.pose_variance
        pose.pose.covariance[7] = cls.pose_variance
        cls.pose_pub.publish(pose)
        dock = DockStatus()
        dock.is_docked = cls.docked
        cls.dock_pub.publish(dock)
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        cls.tf.sendTransform(transform)

    @classmethod
    def accept(cls, request):
        cls.received.append(request.pose)
        return GoalResponse.REJECT if cls.mode == 'reject' else GoalResponse.ACCEPT

    @classmethod
    def cancel(cls, handle):
        return CancelResponse.ACCEPT

    @classmethod
    def execute(cls, handle):
        if cls.mode == 'wait':
            end = time.monotonic() + 20
            while time.monotonic() < end:
                if handle.is_cancel_requested:
                    cls.canceled += 1
                    handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.02)
        if cls.mode == 'abort' and len(cls.received) == 2:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def setUp(self):
        type(self).received = []
        type(self).canceled = 0
        type(self).mode = 'success'
        type(self).docked = False
        type(self).pose_variance = 0.0

    def command(self, goal_timeout=10.0, ready_timeout=8.0):
        return ['/usr/bin/python3', str(SCRIPT), '--ros-args', '-p', 'use_sim_time:=false',
                '-p', 'pause_at_goal:=0.0', '-p', f'goal_timeout:={goal_timeout}',
                '-p', f'ready_timeout:={ready_timeout}']

    def run_node(self, **kwargs):
        result = subprocess.run(self.command(**kwargs), capture_output=True, text=True, timeout=30)
        self.assertNotIn('Traceback', result.stdout + result.stderr, result.stdout + result.stderr)
        return result

    def test_three_successes_in_order(self):
        result = self.run_node()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('COMPLETE: 3/3', result.stdout + result.stderr)
        expected = yaml.safe_load((ROOT / 'src/turtlebot4_btth5/config/goals.yaml').read_text())['goals']
        self.assertEqual([(p.pose.position.x, p.pose.position.y) for p in self.received],
                         [(g['x'], g['y']) for g in expected])
        self.assertTrue(all(p.header.frame_id == 'map' for p in self.received))
        for pose in self.received:
            q = pose.pose.orientation
            self.assertAlmostEqual(q.z*q.z + q.w*q.w, 1.0)

    def test_abort_stops_before_third_goal(self):
        type(self).mode = 'abort'
        result = self.run_node()
        self.assertEqual(result.returncode, 1)
        self.assertEqual(len(self.received), 2)
        self.assertNotIn('COMPLETE', result.stdout)

    def test_rejected_goal_stops_sequence(self):
        type(self).mode = 'reject'
        result = self.run_node()
        self.assertEqual(result.returncode, 1)
        self.assertEqual(len(self.received), 1)

    def test_timeout_cancels_and_stops(self):
        type(self).mode = 'wait'
        result = self.run_node(goal_timeout=0.5)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertEqual(len(self.received), 1)
        self.assertEqual(self.canceled, 1)

    def test_ctrl_c_cancels_and_stops(self):
        type(self).mode = 'wait'
        process = subprocess.Popen(self.command(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            end = time.monotonic() + 10
            while not self.received and time.monotonic() < end:
                time.sleep(0.05)
            self.assertTrue(self.received)
            time.sleep(0.1)
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=20)
            self.assertEqual(process.returncode, 0, output)
            self.assertNotIn('COMPLETE:', output)
            self.assertEqual(self.canceled, 1, output)
            self.assertEqual(len(self.received), 1)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            process.stdout.close()

    def test_docked_robot_sends_no_goal(self):
        type(self).docked = True
        result = self.run_node(ready_timeout=1.5)
        self.assertEqual(result.returncode, 1)
        self.assertFalse(self.received)

    def test_uncertain_localization_sends_no_goal(self):
        type(self).pose_variance = 1.0
        result = self.run_node(ready_timeout=1.5)
        self.assertEqual(result.returncode, 1)
        self.assertFalse(self.received)
        self.assertIn('uncertainty', result.stdout + result.stderr)

    def test_invalid_goal_file(self):
        # Executable has no .py extension: use source file to load the validator.
        path = ROOT / 'src/turtlebot4_btth5/scripts/navigate_goals.py'
        spec = importlib.util.spec_from_file_location('student_nav', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as temp:
            goal_file = Path(temp) / 'goals.yaml'
            for content in ['goals: []', 'goals: [{x: .nan, y: 1, yaw: 0}]',
                            'goals: [{x: 1, y: 1, yaw: 0}, {x: 1, y: 1, yaw: 0}, {x: 1, y: 1, yaw: 0}]']:
                goal_file.write_text(content)
                with self.assertRaises(ValueError):
                    module.load_goals(goal_file)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=5)
        cls.thread.join(timeout=5)
        cls.server.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(verbosity=2)
