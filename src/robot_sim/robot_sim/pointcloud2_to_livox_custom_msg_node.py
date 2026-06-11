#!/usr/bin/env python3
from collections import deque

import rclpy
from rclpy.node import Node

from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class PointCloud2ToLivoxCustomMsgNode(Node):
    def __init__(self):
        super().__init__('pointcloud2_to_livox_custom_msg_node')
        self.declare_parameter('input_topic', '/point_cloud')
        self.declare_parameter('output_topic', '/livox/lidar')
        self.declare_parameter('accumulated_pointcloud_topic', '/livox/lidar_pc2_accumulated')
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('lidar_id', 0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('target_points_per_packet', 20000)
        self.declare_parameter('max_buffer_points', 60000)
        self.declare_parameter('accumulate_packets', True)
        self.declare_parameter('throttle_latest_frame', False)
        self.declare_parameter('clear_after_publish', False)
        self.declare_parameter('drop_duplicate_stamps', True)
        self.declare_parameter('input_point_stride', 1)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        pc2_output_topic = self.get_parameter('accumulated_pointcloud_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._lidar_id = int(self.get_parameter('lidar_id').value)
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._target_points = int(self.get_parameter('target_points_per_packet').value)
        self._max_buffer_points = int(self.get_parameter('max_buffer_points').value)
        self._accumulate_packets = bool(self.get_parameter('accumulate_packets').value)
        self._throttle_latest_frame = bool(self.get_parameter('throttle_latest_frame').value)
        self._clear_after_publish = bool(self.get_parameter('clear_after_publish').value)
        self._drop_duplicate_stamps = bool(self.get_parameter('drop_duplicate_stamps').value)
        self._input_point_stride = max(1, int(self.get_parameter('input_point_stride').value))
        self._points = deque(maxlen=max(self._max_buffer_points, self._target_points))
        self._latest_points = None
        self._latest_stamp = None
        self._last_published_stamp_ns = None
        self._last_input_stamp_ns = None

        self.create_subscription(PointCloud2, input_topic, self._pc2_cb, 10)
        self._pub = self.create_publisher(CustomMsg, output_topic, 10)
        self._pc2_pub = self.create_publisher(PointCloud2, pc2_output_topic, 10)
        if self._accumulate_packets:
            self.create_timer(1.0 / self._publish_rate, self._publish_accumulated)
            accumulation = 'non-overlapping batches' if self._clear_after_publish else 'sliding window'
            mode = (
                f'accumulated {self._publish_rate:.1f} Hz {accumulation}, '
                f'target {self._target_points} points/packet, '
                f'input stride {self._input_point_stride}'
            )
        elif self._throttle_latest_frame:
            self.create_timer(1.0 / self._publish_rate, self._publish_latest_frame)
            mode = f'latest complete frame throttled to {self._publish_rate:.1f} Hz'
        else:
            mode = 'passthrough one CustomMsg per PointCloud2'
        self.get_logger().info(
            f'Converting {input_topic} -> {output_topic} and {pc2_output_topic} ({mode})'
        )

    def _pc2_cb(self, msg: PointCloud2):
        stamp_ns = self._stamp_ns(msg.header.stamp)
        if self._drop_duplicate_stamps and stamp_ns == self._last_input_stamp_ns:
            return
        self._last_input_stamp_ns = stamp_ns
        self._latest_stamp = msg.header.stamp
        incoming = []
        for i, p in enumerate(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)):
            if i % self._input_point_stride:
                continue
            incoming.append((float(p[0]), float(p[1]), float(p[2])))

        if self._accumulate_packets:
            self._points.extend(incoming)
            return
        if self._throttle_latest_frame:
            self._latest_points = incoming
            return

        self._publish_points(incoming, msg.header.stamp)

    @staticmethod
    def _stamp_ns(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    def _publish_latest_frame(self):
        if self._latest_points is None or self._latest_stamp is None:
            return

        stamp_ns = self._stamp_ns(self._latest_stamp)
        if stamp_ns == self._last_published_stamp_ns:
            return

        self._publish_points(self._latest_points, self._latest_stamp)
        self._last_published_stamp_ns = stamp_ns

    def _publish_accumulated(self):
        if not self._points or self._latest_stamp is None:
            return

        available = len(self._points)
        count = min(self._target_points, available)
        if count <= 0:
            return

        if available >= count:
            start = available - count
            points = list(self._points)[start:]
        else:
            points = list(self._points)

        self._publish_points(points, self._latest_stamp)
        if self._clear_after_publish:
            self._points.clear()

    def _publish_points(self, source_points, stamp):
        out = CustomMsg()
        out.header.stamp = stamp
        out.header.frame_id = self._frame_id
        out.timebase = stamp.sec * 1000000000 + stamp.nanosec
        out.lidar_id = self._lidar_id

        points = []
        point_count = len(source_points)
        packet_period_ns = int(1e9 / self._publish_rate) if self._publish_rate > 0.0 else 0
        for i, p in enumerate(source_points):
            pt = CustomPoint()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            pt.reflectivity = 0
            if point_count > 1 and packet_period_ns > 0:
                pt.offset_time = int(i * packet_period_ns / (point_count - 1))
            else:
                pt.offset_time = 0
            pt.tag = 0
            pt.line = i % 40
            points.append(pt)

        out.point_num = len(points)
        out.points = points
        self._pub.publish(out)
        self._publish_pointcloud2(source_points, stamp)

    def _publish_pointcloud2(self, source_points, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self._frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self._pc2_pub.publish(pc2.create_cloud(header, fields, source_points))


def main(args=None):
    rclpy.init(args=args)
    node = PointCloud2ToLivoxCustomMsgNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
