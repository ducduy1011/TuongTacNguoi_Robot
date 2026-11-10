"""Hien thi ten khu vuc va diem dich tren ban do RViz."""
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .commands import normalize


class MapLabels(Node):
    def __init__(self):
        super().__init__('map_labels')
        self.declare_parameter('regions_file', '')
        data = yaml.safe_load(Path(self.get_parameter('regions_file').value).read_text())
        self.frame = data.get('frame_id', 'map')
        self.regions = data['regions']
        self.publisher = self.create_publisher(
            MarkerArray, '/region_labels',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(5.0, self.publish_labels)
        self.publish_labels()

    def publish_labels(self):
        markers = MarkerArray()
        colors = [(1.0, 0.7, 0.2), (0.3, 1.0, 0.3),
                  (0.4, 0.7, 1.0), (1.0, 0.5, 0.8)]
        for index, region in enumerate(self.regions):
            for is_text in (False, True):
                marker = Marker()
                marker.header.frame_id = self.frame
                marker.ns = 'regions'
                marker.id = index * 2 + int(is_text)
                marker.type = Marker.TEXT_VIEW_FACING if is_text else Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(region['goal']['x'])
                marker.pose.position.y = float(region['goal']['y'])
                marker.pose.position.z = 0.3
                marker.pose.orientation.w = 1.0
                marker.color.r, marker.color.g, marker.color.b = colors[index % len(colors)]
                marker.color.a = 1.0
                if is_text:
                    marker.pose.position.y += 1.0
                    marker.scale.z = 1.0
                    # Font RViz mac dinh khong hien du dau tieng Viet.
                    marker.text = f"{region['id']}: {normalize(region['name'])}"
                    marker.color.r *= 0.55
                    marker.color.g *= 0.55
                    marker.color.b *= 0.55
                else:
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.35
                markers.markers.append(marker)
        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapLabels()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
