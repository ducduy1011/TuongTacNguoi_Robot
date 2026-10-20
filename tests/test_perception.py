#!/usr/bin/env python3
"""Test actual ROS image -> cv_bridge -> YOLO -> Image/JSON in domain 148.

Use the official Ultralytics bus.jpg example (not Gazebo demo evidence):
ROS_DOMAIN_ID=148 .venv-yolo/bin/python tests/test_perception.py /path/to/bus.jpg
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = Path(sys.argv.pop(1)) if len(sys.argv) > 1 else ROOT / '.venv-yolo/lib/python3.10/site-packages/ultralytics/assets/bus.jpg'


class PerceptionTest(unittest.TestCase):
    def test_ros_image_to_yolo_and_back(self):
        if os.environ.get('ROS_DOMAIN_ID') != '148':
            self.fail('Use ROS_DOMAIN_ID=148, never the robot domain.')
        frame = cv2.imread(str(EXAMPLE))
        self.assertIsNotNone(frame, 'Supply official Ultralytics bus.jpg as the test image.')
        rclpy.init()
        node = rclpy.create_node('yolo_pipeline_test')
        bridge = CvBridge()
        received = {'image': None, 'data': None}
        node.create_subscription(Image, '/yolo/image', lambda m: received.update(image=m), 10)
        node.create_subscription(String, '/yolo/detections',
                                 lambda m: received.update(data=json.loads(m.data)), 10)
        publisher = node.create_publisher(Image, '/test/rgb', 1)
        log = tempfile.TemporaryFile(mode='w+')
        process = subprocess.Popen([
            str(ROOT / '.venv-yolo/bin/python'),
            str(ROOT / 'install/turtlebot4_btth5/lib/turtlebot4_btth5/yolo_detector'),
            '--ros-args', '-p', 'image_topic:=/test/rgb', '-p',
            f'model:={ROOT / ".models/yolov8n.pt"}', '-p', 'max_fps:=2.0',
            '-p', 'use_sim_time:=false'], stdout=log, stderr=subprocess.STDOUT)

        def send_until(image, condition, timeout):
            end = time.monotonic() + timeout
            last_sent = 0
            while time.monotonic() < end and process.poll() is None:
                if time.monotonic() - last_sent > 0.2:
                    msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
                    msg.header.frame_id = 'test_camera_optical'
                    msg.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(msg)
                    last_sent = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.05)
                if condition():
                    return True
            log.seek(0)
            self.fail('YOLO pipeline timed out: ' + log.read())

        try:
            send_until(frame, lambda: received['image'] is not None and received['data'] is not None
                       and len(received['data']['objects']) > 0, 40)
            result = received['data']
            self.assertEqual(result['frame_id'], 'test_camera_optical')
            self.assertTrue(any(o['class'] in ('bus', 'person') for o in result['objects']))
            for obj in result['objects']:
                self.assertGreaterEqual(obj['confidence'], 0.35)
                self.assertEqual(len(obj['xyxy']), 4)
            annotated = bridge.imgmsg_to_cv2(received['image'], 'bgr8')
            self.assertEqual(annotated.shape, frame.shape)
            self.assertFalse(np.array_equal(annotated, frame), 'Boxes/labels must alter the output image.')
            self.assertEqual(received['image'].header.frame_id, 'test_camera_optical')
            print('Detected on official example:', [(o['class'], o['confidence']) for o in result['objects']])
            # Empty scene must be reported honestly, without stale boxes or invented detections.
            send_until(np.zeros_like(frame), lambda: received['data']['objects'] == [], 15)
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
            log.seek(0)
            output = log.read()
            log.close()
        self.assertEqual(process.returncode, 0, output)


if __name__ == '__main__':
    unittest.main(verbosity=2)
