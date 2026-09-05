from __future__ import annotations

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from robot_skills.semantic_map import SemanticMap


class SemanticMapMarkerNode(Node):
    def __init__(self):
        super().__init__("semantic_map_marker_node")
        self.semantic_map = SemanticMap()
        self.pub = self.create_publisher(MarkerArray, "/semantic_map/markers", 1)
        self.create_timer(1.0, self._publish)
        self.get_logger().info("Semantic map marker node ready.")

    def _publish(self):
        markers = MarkerArray()
        for index, target in enumerate(self.semantic_map.list_known_targets()):
            resolved = self.semantic_map.resolve(target["id"])
            if resolved is None:
                continue
            marker = Marker()
            marker.header.frame_id = resolved.pose.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "semantic_map"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = resolved.pose.x
            marker.pose.position.y = resolved.pose.y
            marker.pose.position.z = 0.15
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.r = 0.1
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.8
            markers.markers.append(marker)
        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
