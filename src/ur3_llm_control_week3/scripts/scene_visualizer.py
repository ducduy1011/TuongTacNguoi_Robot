#!/usr/bin/env python3
"""Display Gazebo poses in RViz; this topic is never used for task planning."""
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


class SceneVisualizer(Node):
    def __init__(self):
        super().__init__('scene_visualizer')
        world = ET.parse(Path(get_package_share_directory('ur3_llm_control_week3')) /
                         'worlds/ur3_task_week3.sdf').find('world')
        self.colors = {}
        self.poses = {}
        self.static = []
        for model in world.findall('model'):
            name = model.get('name')
            if name.endswith('_cube'):
                self.colors[name] = list(map(float, model.find('link/visual/material/diffuse').text.split()))
                continue
            if name == 'overhead_camera':
                continue
            for link in model.findall('link'):
                for visual in link.findall('visual'):
                    box = visual.find('geometry/box/size')
                    plane = visual.find('geometry/plane/size')
                    if box is None and plane is None:
                        continue
                    parts = [self.offset(e) for e in (model, link, visual)]
                    if any(any(abs(v) > 1e-9 for v in pose[3:]) for pose in parts):
                        raise ValueError('Static RViz geometry must be axis aligned')
                    marker = self.marker(len(self.static), 'environment')
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                        sum(pose[i] for pose in parts) for i in range(3))
                    size = list(map(float, (box if box is not None else plane).text.split()))
                    if plane is not None:
                        size.append(.01)
                        marker.pose.position.z -= .005
                    marker.scale.x, marker.scale.y, marker.scale.z = size
                    rgba = list(map(float, visual.find('material/diffuse').text.split()))
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
                    self.static.append(marker)
        self.publisher = self.create_publisher(MarkerArray, '/gazebo_scene_markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/world/ur3_task_week3/pose/info', self.receive, 10)
        self.create_timer(.05, self.publish)

    @staticmethod
    def offset(element):
        pose = element.find('pose')
        return list(map(float, pose.text.split())) if pose is not None else [0.0]*6

    @staticmethod
    def marker(index, namespace):
        marker = Marker()
        marker.header.frame_id = 'base_link'  # Fixed robot base coincides with Gazebo world.
        marker.ns = namespace
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def receive(self, message):
        for transform in message.transforms:
            if transform.child_frame_id in self.colors:
                self.poses[transform.child_frame_id] = transform.transform

    def publish(self):
        markers = list(self.static)
        for index, (name, rgba) in enumerate(self.colors.items()):
            if name not in self.poses:
                continue
            pose = self.poses[name]
            marker = self.marker(index, 'blocks')
            marker.pose.position.x = pose.translation.x
            marker.pose.position.y = pose.translation.y
            marker.pose.position.z = pose.translation.z
            marker.pose.orientation = pose.rotation
            marker.scale.x = marker.scale.y = marker.scale.z = .05
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
            markers.append(marker)
        stamp = self.get_clock().now().to_msg()
        for marker in markers:
            marker.header.stamp = stamp
        self.publisher.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = SceneVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
