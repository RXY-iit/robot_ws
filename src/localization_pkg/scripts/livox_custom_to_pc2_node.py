#!/usr/bin/env python3
"""
livox_custom_to_pc2_node.py — Relay: Livox CustomMsg → PointCloud2.

Required when xfer_format=1 (fast_lio_mode): FAST-LIO needs CustomMsg on
/livox/lidar, but GICP and pointcloud_to_laserscan need PointCloud2.

Topics:
  Input:  /livox/lidar      livox_ros_driver2/msg/CustomMsg
  Output: /livox/lidar_pc2  sensor_msgs/PointCloud2  (fields: x, y, z, intensity)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField


_FIELDS_XYZI = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 16  # 4 × float32


class LivoxCustomToPc2Node(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pc2')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._sub = self.create_subscription(
            CustomMsg, '/livox/lidar', self._cb, sensor_qos
        )
        self._pub = self.create_publisher(
            PointCloud2, '/livox/lidar_pc2', sensor_qos
        )
        self.get_logger().info('livox_custom_to_pc2: relay ready  /livox/lidar → /livox/lidar_pc2')

    def _cb(self, msg: CustomMsg) -> None:
        points = msg.points
        n = len(points)
        if n == 0:
            return

        # Build Nx4 float32 array in one pass using list comprehension.
        # For 20 k points at 10 Hz this takes ~3 ms in CPython — acceptable.
        raw = np.array(
            [[p.x, p.y, p.z, float(p.reflectivity)] for p in points],
            dtype=np.float32,
        )

        # Drop points with non-finite coordinates (origin zeros from missed returns)
        valid = np.isfinite(raw[:, 0]) & np.isfinite(raw[:, 1]) & np.isfinite(raw[:, 2])
        raw = np.ascontiguousarray(raw[valid])
        n = raw.shape[0]
        if n == 0:
            return

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n
        out.fields = _FIELDS_XYZI
        out.is_bigendian = False
        out.point_step = _POINT_STEP
        out.row_step = _POINT_STEP * n
        out.is_dense = True
        out.data = raw.tobytes()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPc2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
