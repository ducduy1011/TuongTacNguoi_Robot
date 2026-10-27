"""Webcam -> MediaPipe Gesture Recognizer -> /detected_gesture."""
import time
import signal
from pathlib import Path

import cv2
import mediapipe as mp
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .gesture_filter import GestureFilter, wave_features


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('show_window', True)
        self.declare_parameter('confidence', 0.65)
        self.show = self.get_parameter('show_window').value
        self.threshold = self.get_parameter('confidence').value
        self.publisher = self.create_publisher(String, '/detected_gesture', 1)
        self.status = 'WAITING FOR ROBOT NODE'
        self.create_subscription(String, '/robot_status', self.on_status, 1)
        self.filter = GestureFilter()
        self.cap = None
        self.recognizer = None
        self.last_timestamp = -1
        self.last_output = ''
        self.window = 'Bai 6 - Hand Gesture Control'

    def on_status(self, msg):
        self.status = msg.data

    def open(self):
        model = Path(get_package_share_directory('turtlebot4_btth6')) / 'models/gesture_recognizer.task'
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.6,
            min_tracking_confidence=0.6)
        self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        index = self.get_parameter('camera_index').value
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'Khong mo duoc webcam /dev/video{index}. Dong ung dung dang dung camera.')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.show:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window, 800, 600)
        self.get_logger().info('Webcam ready. Thumb_Up xac nhan mot lan; Closed_Fist huy va ve diem dau.')

    def frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.publisher.publish(String(data='No_Hand'))
            raise RuntimeError('Mat hinh webcam. Robot se dung; kiem tra camera roi chay lai.')
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        timestamp = max(self.last_timestamp + 1, time.monotonic_ns() // 1_000_000)
        self.last_timestamp = timestamp
        result = self.recognizer.recognize_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp)
        label, score, center = 'No_Hand', 0.0, None
        palm_width, palm_open = None, False
        raw_label = 'No_Hand'
        if result.hand_landmarks:
            label = 'Unknown'
            raw_label = 'Unknown'
            points = result.hand_landmarks[0]
            center, palm_width, palm_open = wave_features(points, frame.shape[0] / frame.shape[1])
            if result.gestures and result.gestures[0]:
                category = result.gestures[0][0]
                raw_label = category.category_name
                score = category.score
                # Uu tien nhan lenh dung, ke ca khi ban tay xoe chua that ro.
                threshold = 0.55 if category.category_name == 'Open_Palm' else self.threshold
                if score >= threshold and category.category_name in (
                        'Open_Palm', 'Closed_Fist', 'Pointing_Up', 'Victory', 'Thumb_Up'):
                    label = category.category_name
            height, width = frame.shape[:2]
            # Ve duong noi truoc, sau do ve cham tron tai tung khop.
            for start, end in mp.solutions.hands.HAND_CONNECTIONS:
                a, b = points[start], points[end]
                cv2.line(frame, (int(a.x * width), int(a.y * height)),
                         (int(b.x * width), int(b.y * height)),
                         (0, 255, 0), 2, cv2.LINE_AA)
            for point in points:
                position = (int(point.x * width), int(point.y * height))
                cv2.circle(frame, position, 5, (255, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(frame, position, 3, (0, 0, 255), -1, cv2.LINE_AA)
        output = self.filter.update(label, center, time.monotonic(), palm_width, palm_open)
        self.publisher.publish(String(data=output))
        if output == 'Wave' and self.last_output != 'Wave':
            self.get_logger().info('Wave detected -> chon WAVE. Gio Thumb_Up de robot chao lai.')
        self.last_output = output
        if self.show:
            # ASCII de OpenCV hien chu dung, khong loi font tieng Viet.
            wave_hint = ('Wave READY -> show THUMB UP' if self.filter.wave else
                         f'Wave: sway open hand left-right-left | turns {min(self.filter.turns, 2)}/2')
            lines = [f'AI: {raw_label}  confidence: {score:.2f}', f'Topic: {output}',
                     self.status, '1 finger: back/forth x4 | 2 fingers: circle',
                     wave_hint, 'Thumb Up: confirm once | No hand: continue',
                     'Open Palm: STOP | Closed Fist: CANCEL + RETURN']
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (frame.shape[1], 185), (0, 0, 0), -1)
            frame = cv2.addWeighted(overlay, 0.65, frame, 0.35, 0)
            for i, line in enumerate(lines):
                cv2.putText(frame, line, (8, 23 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX,
                            0.43, (0, 255, 255) if i == 2 else (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(self.window, frame)
            cv2.waitKey(1)  # Chi xu ly cua so; khong dung phim de dieu khien robot.
            if cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                return False
        return True

    def close(self):
        self.publisher.publish(String(data='No_Hand'))
        if self.cap is not None:
            self.cap.release()
        if self.recognizer is not None:
            self.recognizer.close()
        if self.show:
            cv2.destroyAllWindows()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GestureRecognitionNode()
    running = True

    def request_stop(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    try:
        node.open()
        while rclpy.ok() and running:
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            if not node.frame():
                break
            time.sleep(max(0.0, 0.05 - (time.monotonic() - start)))
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # SIGINT co the lam VideoCapture.read() tra False khi dang thoat.
        if running:
            node.get_logger().error(str(exc))
            raise
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
