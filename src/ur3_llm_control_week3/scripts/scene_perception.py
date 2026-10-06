#!/usr/bin/env python3
"""Top-down calibrated RGB perception. Object XY comes exclusively from pixels."""
import json
import math
import os
import signal
import time
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ur3_llm_control_week3.srv import GetSceneState

COLORS = {'red_cube': (0, 0, 255), 'yellow_cube': (0, 255, 255), 'blue_cube': (255, 100, 30),
          'green_cube': (0, 255, 0), 'purple_cube': (255, 70, 190)}
HUE_RANGES = {'red_cube': [(0, 10), (170, 179)], 'yellow_cube': [(20, 38)],
              'blue_cube': [(110, 130)], 'green_cube': [(54, 62)], 'purple_cube': [(137, 145)]}


class ScenePerception(Node):
    def __init__(self):
        super().__init__('scene_perception')
        path = os.path.join(get_package_share_directory('ur3_llm_control_week3'), 'config', 'scene.yaml')
        with open(path) as stream:
            self.cfg = yaml.safe_load(stream)
        self.bridge = CvBridge()
        self.scene = None
        self.received = 0.0
        self.held = ''
        self.kernel = np.ones((3, 3), np.uint8)
        camera = self.cfg['camera']
        focal = camera['width'] / (2 * math.tan(camera['hfov'] / 2))
        self.scale = (camera['position'][2] - camera['surface_z']) / focal
        self.face_area = (self.cfg['cube_size']/self.scale)**2
        self.target_pixels = {name: (int((camera['width']-1)/2 - (pos[1]-camera['position'][1])/self.scale),
                                    int((camera['height']-1)/2 - (pos[0]-camera['position'][0])/self.scale))
                              for name, pos in self.cfg['targets'].items()}
        self.create_subscription(Image, '/camera/color/image_raw', self.image, qos_profile_sensor_data)
        self.create_subscription(String, '/held_object', self.holding, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(GetSceneState, '/get_scene_state', self.get_scene)
        self.annotated = self.create_publisher(Image, '/camera/color/image_annotated', 1)
        self.publisher = self.create_publisher(String, '/scene_state', 1)
        self.viewer = self.declare_parameter('viewer', False).value
        self.overlay = np.zeros((camera['height'], camera['width'], 3), np.uint8) if self.viewer else None
        self.task_status = 'READY'
        self.window = 'Week 3 - Live Camera Detection (Q to stop)'
        if self.viewer and not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
            self.get_logger().warning('No desktop display; camera detection continues without video window')
            self.viewer = False
        if self.viewer:
            import gi
            gi.require_version('Gtk', '3.0')
            from gi.repository import Gtk, GObject
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window, camera['width'], camera['height'])
            for window in Gtk.Window.list_toplevels():
                if window.get_title() == self.window:
                    # HighGUI's earlier handler destroys/recreates the window; replace it.
                    handler = GObject.signal_handler_find(window, GObject.SignalMatchType.ID,
                        GObject.signal_lookup('delete-event', Gtk.Window), 0, None, None, None)
                    if handler:
                        window.handler_block(handler)
                    window.connect('delete-event', self.close_video)
            self.create_subscription(String, '/task_status', self.status, 10)
            self.create_timer(.05, self.show_video, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('RGB perception ready; calibrated camera frame -> base_link')

    def holding(self, msg):
        self.held = msg.data

    def status(self, msg):
        self.task_status = msg.data

    def show_video(self):
        if not self.viewer:
            return
        try:
            frame = self.overlay.copy()
            status = self.task_status if time.monotonic()-self.received < 1.0 else 'WAITING FOR CAMERA / STALE IMAGE'
            cv2.rectangle(frame, (0, frame.shape[0]-32), (frame.shape[1], frame.shape[0]), (30, 30, 30), -1)
            cv2.putText(frame, status, (12, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(self.window, frame)
            if cv2.waitKey(1) & 255 in (27, ord('q')):
                rclpy.try_shutdown()
        except cv2.error as exc:
            self.get_logger().warning(f'Video window closed: {exc}')
            rclpy.try_shutdown()

    def close_video(self, window, event):
        # Stop before GTK destroys HighGUI's window and leaves its mutex blocked.
        rclpy.try_shutdown()
        return True

    def hide_video(self):
        self.viewer = False
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

    def image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            camera = self.cfg['camera']
            if msg.width != camera['width'] or msg.height != camera['height']:
                raise ValueError('image dimensions differ from calibration')
            scale = self.scale
            objects = {}
            subscribed = self.annotated.get_subscription_count() > 0
            draw = self.viewer or subscribed
            overlay = bgr.copy() if draw else None
            for name, intervals in HUE_RANGES.items():
                mask = np.zeros(hsv.shape[:2], np.uint8)
                for low, high in intervals:
                    mask |= cv2.inRange(hsv, (low, 120, 70), (high, 255, 255))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = [c for c in contours if cv2.contourArea(c) > 120]
                if len(contours) == 1:
                    bx, by, bw, bh = cv2.boundingRect(contours[0])
                obj = {'detected': False, 'position': None, 'location': 'unknown'}
                if name == self.held:
                    obj['location'] = 'held'
                elif len(contours) == 1:
                    # Side faces pull the color silhouette toward the camera; use the bright top.
                    region = mask[by:by+bh, bx:bx+bw]
                    value = hsv[by:by+bh, bx:bx+bw, 2]
                    peak = np.percentile(value[region > 0], 90)
                    top = region & cv2.inRange(value, math.ceil(.9*peak), 255)
                    top = cv2.morphologyEx(top, cv2.MORPH_OPEN, self.kernel)
                    faces, _ = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    faces = [c for c in faces if .7*self.face_area <= cv2.contourArea(c) <= 1.3*self.face_area]
                    if len(faces) == 1:
                        m = cv2.moments(faces[0])
                        u, v = bx+m['m10']/m['m00'], by+m['m01']/m['m00']
                        # Optical right=-world Y, down=-world X; calibrated plane is the cube top.
                        x = camera['position'][0] - (v-(msg.height-1)/2)*scale
                        y = camera['position'][1] - (u-(msg.width-1)/2)*scale
                        obj = {'detected': True, 'position': [x, y, self.cfg['table_z']+self.cfg['cube_size']/2], 'location': 'table'}
                objects[name] = obj
                if draw and len(contours) == 1:
                    cv2.rectangle(overlay, (bx, by), (bx+bw, by+bh), COLORS[name], 2)
                    label = name + (' [HELD]' if name == self.held else '')
                    cv2.putText(overlay, label, (max(0, bx-25), max(18, by-8)), cv2.FONT_HERSHEY_SIMPLEX, .45, COLORS[name], 1, cv2.LINE_AA)
            visible = all(v['detected'] or k == self.held for k, v in objects.items())
            occupancy = {name: 'empty' if visible else 'unknown' for name in self.cfg['targets']}
            for name, obj in objects.items():
                if not obj['detected']:
                    continue
                x, y, _ = obj['position']
                for target, pos in self.cfg['targets'].items():
                    # Count footprint overlap, including cubes crossing a region's edge.
                    clearance = self.cfg['cube_size']/2 + (.035 if target.startswith('zone_') else .04)
                    if max(abs(x-pos[0]), abs(y-pos[1])) <= clearance+1e-6:
                        occupancy[target] = name if occupancy[target] in ('empty', 'unknown') else 'blocked'
                        obj['location'] = target
            self.scene = {'frame_id': 'base_link', 'stamp': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                          'objects': objects, 'zones': {k:v for k,v in occupancy.items() if k.startswith('zone_')},
                          'temporary': {k:v for k,v in occupancy.items() if k.startswith('temporary_')}}
            if draw:
                for target, (px, py) in self.target_pixels.items():
                    cv2.putText(overlay, target+': '+occupancy[target], (max(0, px-45), py+38), cv2.FONT_HERSHEY_SIMPLEX, .38, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.rectangle(overlay, (0, 0), (msg.width, 62), (30, 30, 30), -1)
                cv2.putText(overlay, 'RGB LIVE | base_link | XY in meters', (12, 23), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(overlay, 'Holding: '+(self.held or 'none')+' | Unknown = occluded / not detected', (12, 48), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 230, 255), 1, cv2.LINE_AA)
                cv2.rectangle(overlay, (0, msg.height-158), (msg.width, msg.height), (30, 30, 30), -1)
                for row, (name, obj) in enumerate(objects.items()):
                    description = obj['location']
                    if obj['detected']:
                        description += ' | X={:.3f} Y={:.3f} m'.format(*obj['position'][:2])
                    cv2.putText(overlay, name+': '+description, (12, msg.height-135+row*23), cv2.FONT_HERSHEY_SIMPLEX, .50, COLORS[name], 1, cv2.LINE_AA)
                if subscribed:
                    annotated = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
                    annotated.header = msg.header
                    self.annotated.publish(annotated)
                self.overlay = overlay
            self.received = time.monotonic()
            self.publisher.publish(String(data=json.dumps(self.scene)))
        except Exception as exc:
            self.scene = None
            if self.context.ok():
                self.get_logger().error(str(exc))

    def get_scene(self, request, response):
        response.success = self.scene is not None and time.monotonic()-self.received < 1.0
        response.scene_json = json.dumps(self.scene) if response.success else '{}'
        response.status = 'SUCCESS' if response.success else 'CAMERA_STALE_OR_UNAVAILABLE'
        return response


def main():
    rclpy.init()
    node = ScenePerception()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: node.context.try_shutdown())
    try:
        while node.context.ok():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if node.context.ok():
            raise
    finally:
        executor.shutdown()
        node.hide_video()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
