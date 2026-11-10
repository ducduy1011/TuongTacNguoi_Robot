"""Kiem tra giao tiep action ROS 2 bang server gia, khong thay cho mo phong."""
from pathlib import Path
import time

import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import yaml

from voice_navigation.voice_nav import VoiceNavigator

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('command, outcome, expected, final', [
    ('Đi đến khu vực A.', 'success', ['A'], 'Da hoan thanh'),
    ('Di chuyển từ khu vực A đến khu vực B.', 'success', ['A', 'B'], 'Da hoan thanh'),
    ('Đi đến khu lưu trữ.', 'success', ['C'], 'Da hoan thanh'),
    ('Đi đến khu vực D.', 'success', [], 'khong ton tai'),
    ('Đi từ A đến D.', 'success', [], 'khong ton tai'),
    ('Đi từ A đến B.', 'abort', ['A'], 'khong thanh cong'),
    ('Đi từ A đến B.', 'reject', [], 'tu choi'),
    ('Đi đến A.', 'false_success', ['A'], 'chua xac nhan'),
    ('Đi đến A.', 'stale_success', ['A'], 'chua xac nhan'),
])
def test_action(command, outcome, expected, final):
    rclpy.init(args=['--ros-args', '-p', f'regions_file:={ROOT / "output/regions.yaml"}'])
    server = Node('test_nav2_server')
    received, statuses = [], []
    points = {r['id']: r['goal'] for r in yaml.safe_load(
        (ROOT / 'output/regions.yaml').read_text())['regions']}
    position = {'x': 0.0, 'y': 0.0}
    broadcaster = TransformBroadcaster(server)

    def publish_position():
        transform = TransformStamped()
        stamp = server.get_clock().now().nanoseconds
        if outcome == 'stale_success':
            stamp -= 60_000_000_000
        transform.header.stamp = rclpy.time.Time(nanoseconds=stamp).to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = position['x']
        transform.transform.translation.y = position['y']
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

    position_timer = server.create_timer(0.02, publish_position)

    def execute(handle):
        point = handle.request.pose
        assert point.header.frame_id == 'map'
        region = next(key for key, goal in points.items()
                      if abs(goal['x'] - point.pose.position.x) < 1e-6
                      and abs(goal['y'] - point.pose.position.y) < 1e-6)
        received.append(region)
        if outcome == 'abort':
            handle.abort()
        else:
            if outcome != 'false_success':
                position.update(x=point.pose.position.x, y=point.pose.position.y)
            publish_position()
            handle.succeed()
        return NavigateToPose.Result()

    action = ActionServer(server, NavigateToPose, 'navigate_to_pose', execute,
                          goal_callback=lambda _: GoalResponse.REJECT
                          if outcome == 'reject' else GoalResponse.ACCEPT)
    nav = VoiceNavigator()
    publisher = server.create_publisher(String, 'voice_command', 10)
    subscription = server.create_subscription(
        String, 'voice_status', lambda msg: statuses.append(msg.data), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(nav)

    def spin_until(predicate):
        deadline = time.monotonic() + 8
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate(), statuses

    try:
        spin_until(lambda: nav.client.server_is_ready()
                   and publisher.get_subscription_count() > 0
                   and nav.status_pub.get_subscription_count() > 0)
        publisher.publish(String(data=command))
        spin_until(lambda: any(final in message for message in statuses))
        assert received == expected
        assert not nav.busy
    finally:
        action.destroy()
        executor.shutdown()
        nav.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
