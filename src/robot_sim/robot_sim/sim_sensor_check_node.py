#!/usr/bin/env python3
from collections import deque
import time

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from livox_ros_driver2.msg import CustomMsg
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class TopicStat:
    def __init__(self, name):
        self.name = name
        self.times = deque(maxlen=120)
        self.last_summary = "waiting"
        self.last_count = None

    def mark(self, summary, count=None):
        self.times.append(time.monotonic())
        self.last_summary = summary
        self.last_count = count

    def rate(self):
        if len(self.times) < 2:
            return 0.0
        duration = self.times[-1] - self.times[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.times) - 1) / duration

    def point_rate(self):
        if self.last_count is None:
            return None
        return self.last_count * self.rate()

    def age(self):
        if not self.times:
            return None
        return time.monotonic() - self.times[-1]


class SimSensorCheckNode(Node):
    def __init__(self):
        super().__init__("sim_sensor_check_node")
        self.declare_parameter("report_period", 2.0)

        self._stats = {}
        self._add_image("/camera/camera/color/image_raw", "color")
        self._add_image("/camera/camera/depth/image_rect_raw", "depth")
        self._add_camera_info("/camera/camera/color/camera_info", "color_info")
        self._add_camera_info("/camera/camera/depth/camera_info", "depth_info")
        self._add_pointcloud2("/point_cloud", "raw_pc2")
        self._add_pointcloud2("/livox/lidar_pc2_accumulated", "accum_pc2")
        self._add_livox("/livox/lidar", "livox")
        self._add_clock("/clock", "clock")
        self._add_joint_state("/joint_states", "joint_states")

        period = float(self.get_parameter("report_period").value)
        self.create_timer(period, self._report)
        self.get_logger().info("Checking Isaac sim sensor topics. Open RViz for visual inspection.")

    def _stat(self, key):
        stat = TopicStat(key)
        self._stats[key] = stat
        return stat

    def _add_image(self, topic, key):
        stat = self._stat(key)

        def cb(msg: Image):
            stat.mark(f"{msg.width}x{msg.height} {msg.encoding} frame={msg.header.frame_id}")

        self.create_subscription(Image, topic, cb, 10)

    def _add_camera_info(self, topic, key):
        stat = self._stat(key)

        def cb(msg: CameraInfo):
            stat.mark(f"{msg.width}x{msg.height} frame={msg.header.frame_id}")

        self.create_subscription(CameraInfo, topic, cb, 10)

    def _add_pointcloud2(self, topic, key):
        stat = self._stat(key)

        def cb(msg: PointCloud2):
            count = msg.width * msg.height
            z_min = None
            z_max = None
            for point in pc2.read_points(msg, field_names=('z',), skip_nans=True):
                z = float(point[0])
                z_min = z if z_min is None else min(z_min, z)
                z_max = z if z_max is None else max(z_max, z)
            z_text = ""
            if z_min is not None and z_max is not None:
                z_text = f" z=[{z_min:.2f},{z_max:.2f}]"
            stat.mark(f"points={count}{z_text} frame={msg.header.frame_id}", count=count)

        self.create_subscription(PointCloud2, topic, cb, 10)

    def _add_livox(self, topic, key):
        stat = self._stat(key)

        def cb(msg: CustomMsg):
            stat.mark(f"points={msg.point_num} frame={msg.header.frame_id}", count=msg.point_num)

        self.create_subscription(CustomMsg, topic, cb, 10)

    def _add_clock(self, topic, key):
        stat = self._stat(key)

        def cb(msg: Clock):
            stat.mark(_format_stamp(msg.clock))

        self.create_subscription(Clock, topic, cb, 10)

    def _add_joint_state(self, topic, key):
        stat = self._stat(key)

        def cb(msg: JointState):
            stat.mark(f"joints={len(msg.name)}")

        self.create_subscription(JointState, topic, cb, 10)

    def _report(self):
        lines = []
        missing = []
        for key, stat in self._stats.items():
            age = stat.age()
            if age is None:
                missing.append(key)
                lines.append(f"{key}: WAIT {stat.last_summary}")
                continue
            stale = " STALE" if age > 2.5 else ""
            point_rate = stat.point_rate()
            point_rate_text = ""
            if point_rate is not None:
                point_rate_text = f" point_rate={point_rate / 1000.0:5.1f}k/s"
            lines.append(
                f"{key}: {stat.rate():5.1f} Hz age={age:3.1f}s{stale}"
                f"{point_rate_text} {stat.last_summary}"
            )
        self.get_logger().info("\n" + "\n".join(lines))
        if missing:
            self.get_logger().warn("Waiting for: " + ", ".join(missing))


def _format_stamp(stamp: Time):
    return f"{stamp.sec}.{stamp.nanosec:09d}"


def main(args=None):
    rclpy.init(args=args)
    node = SimSensorCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
