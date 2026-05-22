#!/usr/bin/env python3
"""Publish a binary little-endian PLY as a latched PointCloud2 topic."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from ply_to_nav2_map import read_ply_vertices


class PlyPointCloudPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ply_pointcloud_publisher")
        if args.use_sim_time:
            self.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
            )
        points = read_ply_vertices(Path(args.input))

        if args.z_min is not None or args.z_max is not None:
            z_min = -np.inf if args.z_min is None else args.z_min
            z_max = np.inf if args.z_max is None else args.z_max
            points = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]

        header = Header()
        header.frame_id = args.frame_id
        header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.msg = point_cloud2.create_cloud(header, fields, points.astype(np.float32))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PointCloud2, args.topic, qos)
        self.timer = self.create_timer(args.period, self.publish_cloud)
        self.publish_cloud()
        self.get_logger().info(
            f"Publishing {len(points)} PLY points on {args.topic} in frame {args.frame_id}"
        )

    def publish_cloud(self) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="Input binary little-endian PLY file")
    parser.add_argument("--topic", default="/l402_glim_points")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--z-min", type=float, default=None)
    parser.add_argument("--z-max", type=float, default=None)
    parser.add_argument("--use-sim-time", action="store_true")
    return parser.parse_args()


def main() -> None:
    rclpy.init()
    args = parse_args()
    node = PlyPointCloudPublisher(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
