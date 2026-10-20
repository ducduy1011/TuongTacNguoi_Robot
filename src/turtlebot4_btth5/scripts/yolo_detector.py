#!/usr/bin/env python3
"""ROS RGB -> cv_bridge/OpenCV -> YOLO -> annotated Image and JSON detections."""
import json
import math
import os
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        for key, value in [('image_topic', '/oakd/rgb/preview/image_raw'),
                           ('model', ''), ('confidence', 0.35), ('image_size', 416),
                           ('max_fps', 2.0), ('show_window', False), ('device', 'cpu'),
                           ('target_classes', ['person', 'chair'])]:
            self.declare_parameter(key, value)
        model_path = Path(self.get_parameter('model').value).expanduser()
        if not model_path.is_file():
            raise ValueError(f'Model not found: {model_path}. Run tools/prepare_yolo.py first.')
        fps = self.get_parameter('max_fps').value
        confidence = self.get_parameter('confidence').value
        if not math.isfinite(fps) or fps <= 0 or not 0 <= confidence <= 1:
            raise ValueError('max_fps must be positive; confidence must be in [0,1].')
        if self.get_parameter('image_size').value < 32:
            raise ValueError('image_size must be >=32.')
        self.show_window = self.get_parameter('show_window').value
        if self.show_window and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('No desktop display; use show_window:=false and RViz /yolo/image.')
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(2)
        cv2.setNumThreads(1)
        self.model = YOLO(str(model_path))
        targets = self.get_parameter('target_classes').value
        known = set(self.model.names.values())
        if not set(targets).issubset(known):
            raise ValueError(f'Unknown target classes: {set(targets) - known}')
        self.class_ids = [i for i, name in self.model.names.items() if name in targets] if targets else None
        self.bridge = CvBridge()
        self.latest = None
        self.last_received = time.monotonic()
        self.last_log = 0.0
        self.images = self.create_publisher(Image, '/yolo/image', 1)
        self.detections = self.create_publisher(String, '/yolo/detections', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.on_image, qos)
        # A single slot replaces old images. Inference cannot build a stale image queue.
        self.create_timer(1.0 / fps, self.infer)
        self.create_timer(5.0, self.check_camera)
        self.get_logger().info(f"YOLO ready: {self.get_parameter('image_topic').value} -> "
                               '/yolo/image. Waiting for camera; no fake detections.')

    def on_image(self, msg):
        self.latest = msg
        self.last_received = time.monotonic()

    def check_camera(self):
        if time.monotonic() - self.last_received > 5:
            self.get_logger().warning('No RGB received for 5 s. Check Play and image_topic/bridge.')

    def infer(self):
        if self.latest is None:
            return
        msg, self.latest = self.latest, None
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'cv_bridge: {error}')
            return
        result = self.model.predict(frame, conf=self.get_parameter('confidence').value,
                                    imgsz=self.get_parameter('image_size').value,
                                    device=self.get_parameter('device').value,
                                    classes=self.class_ids, verbose=False)[0]
        annotated = result.plot()  # Native YOLO boxes, class names and confidences.
        output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        output.header = msg.header
        self.images.publish(output)
        objects = []
        for box in result.boxes:
            class_id = int(box.cls.item())
            objects.append({'class_id': class_id, 'class': result.names[class_id],
                            'confidence': round(float(box.conf.item()), 4),
                            'xyxy': [round(float(v), 1) for v in box.xyxy[0].tolist()]})
        data = {'frame_id': msg.header.frame_id,
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'objects': objects}
        self.detections.publish(String(data=json.dumps(data)))
        if time.monotonic() - self.last_log >= 2:
            description = ', '.join(f"{o['class']} {o['confidence']:.2f}" for o in objects)
            self.get_logger().info(f'Detected: {description or "none (0 objects)"}')
            self.last_log = time.monotonic()
        if self.show_window:
            cv2.imshow('TurtleBot 4 - YOLO', annotated)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = None
    code = 0
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'[yolo_detector] {error}', flush=True)
        code = 1
    finally:
        cv2.destroyAllWindows()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
