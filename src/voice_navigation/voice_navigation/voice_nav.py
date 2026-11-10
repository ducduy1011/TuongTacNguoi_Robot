"""Nhan van ban tu /voice_command, gui tung dich NavigateToPose."""
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .commands import parse_command


class VoiceNavigator(Node):
    def __init__(self):
        super().__init__('voice_navigator')
        self.declare_parameter('regions_file', '')
        path = self.get_parameter('regions_file').value
        data = yaml.safe_load(Path(path).read_text())
        self.regions = data['regions']
        # Kiem tra ten trung ngay khi khoi dong.
        parse_command('Di den ' + self.regions[0]['id'], self.regions)
        self.by_id = {region['id']: region for region in self.regions}
        self.frame_id = data.get('frame_id', 'map')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arrival_timer = None
        self.status_pub = self.create_publisher(String, 'voice_status', 10)
        self.subscription = self.create_subscription(String, 'voice_command', self.command, 10)
        self.route = []
        self.busy = False
        self.report('San sang nhan lenh.')

    def report(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def command(self, msg):
        try:
            route = parse_command(msg.data, self.regions)
        except ValueError as exc:
            self.report(str(exc))
            return
        if self.busy:
            self.report('Robot dang di chuyen. Hay cho hoan thanh roi ra lenh moi.')
            return
        if not self.client.server_is_ready():
            self.report('Nav2 chua san sang. Kiem tra localization va Nav2.')
            return
        self.route = route
        self.busy = True
        self.report('Lo trinh: ' + ' -> '.join(route))
        self.send_next()

    def stop_route(self, message):
        if self.arrival_timer is not None:
            self.arrival_timer.cancel()
            self.arrival_timer = None
        self.route.clear()
        self.busy = False
        self.report(message)

    def send_next(self):
        if not self.route:
            self.stop_route('Da hoan thanh lo trinh.')
            return
        region = self.by_id[self.route[0]]
        point = region['goal']
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point['x'])
        goal.pose.pose.position.y = float(point['y'])
        goal.pose.pose.orientation.z = math.sin(float(point['yaw']) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(point['yaw']) / 2.0)
        self.report('Dang di den ' + region['name'])
        try:
            self.client.send_goal_async(goal).add_done_callback(self.goal_response)
        except Exception as exc:
            self.stop_route(f'Khong gui duoc dich: {exc}')

    def goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.stop_route('Nav2 tu choi dich. Dung lo trinh.')
                return
            handle.get_result_async().add_done_callback(self.goal_result)
        except Exception as exc:
            self.stop_route(f'Loi nhan dich: {exc}')

    def goal_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.stop_route(f'Loi nhan ket qua: {exc}')
            return
        # Humble: dung status cua action, khong dung result.error_code.
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.stop_route(f'Dieu huong khong thanh cong (status={status}). Dung lo trinh.')
            return
        # Doi ngan de TF kip cap nhat; khong chi dua vao status cua Nav2.
        self.arrival_deadline = time.monotonic() + 2.0
        self.arrival_timer = self.create_timer(0.1, self.verify_arrival)
        self.verify_arrival()

    def verify_arrival(self):
        problem = 'chua co toa do robot trong ban do'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, 'base_link', rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(transform.header.stamp)
            age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            point = self.by_id[self.route[0]]['goal']
            position = transform.transform.translation
            distance = math.hypot(position.x - point['x'], position.y - point['y'])
            if abs(age) > 2.0:
                problem = f'du lieu toa do cu hoac lech gio ({age:.1f} giay)'
            elif distance > 0.4:
                problem = f'robot con cach dich {distance:.2f} m'
            else:
                self.arrival_timer.cancel()
                self.arrival_timer = None
                self.report('Da den khu vuc ' + self.route.pop(0))
                self.send_next()
                return
        except TransformException:
            pass
        if time.monotonic() >= self.arrival_deadline:
            self.stop_route('Nav2 bao thanh cong nhung chua xac nhan den dich: ' + problem)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f'Loi du lieu khu vuc: {exc}')
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
