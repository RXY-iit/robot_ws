"""Read-only B1 evidence capture and full frozen-route RViz overlay.

Physics odometry is preserved in its original frame, not relabelled map truth.
No motion or recovery commands are sent by this observer.
"""
import json
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/robot_skills"))
from robot_skills.waypoint_routes import read_route


class B1Evidence:
    def __init__(self, node, output_dir, route_path=None):
        self.node = node
        self.clock = None
        self.counts = {}
        self.last = {}
        self.last_source = {}
        self.clock_last_advance = None
        self.output = Path(output_dir)
        self.file = (self.output / "b1_evidence.jsonl").open("a", buffering=1)
        self.route = read_route(route_path) if route_path else None
        self.write("observer_started", {"route": self.route,
            "trajectory_source": "/wheel_odom: Isaac ComputeOdometry in odom; map/world calibration required for ground-truth error",
            "score_timestamp": "Float32 has no header; use receipt sim/wall time, not an invented source stamp"})
        self.subscriptions = [node.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)]
        for topic, kind in [("/wheel_odom", Odometry), ("/gicp_loc/pose", PoseStamped),
                            ("/initialpose", PoseWithCovarianceStamped), ("/operator/initialpose", PoseWithCovarianceStamped),
                            ("/gicp_loc/score", Float32), ("/agent/health_state", String),
                            ("/agent/health_dashboard_state", String), ("/agent/recovery_state", String),
                            ("/agent/mission_events", String), ("/agent/mission_state", String),
                            ("/agent/recovery_events", String), ("/agent/recovery_response", String),
                            ("/gicp_loc/accepted_state", String), ("/gicp_loc/recovery_status", String),
                            ("/b1/lidar_injection_status", String), ("/b1/lidar_injection_command", String)]:
            self.subscriptions.append(node.create_subscription(kind, topic,
                lambda msg, name=topic: self.receive(name, msg),
                QoSProfile(depth=100) if kind is String else QoSProfile(
                    depth=100, reliability=qos_profile_sensor_data.reliability)))
        self.publisher = node.create_publisher(MarkerArray, "/b1/waypoint_route",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = node.create_timer(1.0, self.publish)
        self.publish()

    def write(self, event, data, source_stamp=None):
        self.counts[event] = self.counts.get(event, 0) + 1
        self.file.write(json.dumps({"event": event, "wall_unix_sec": time.time(),
            "wall_monotonic_sec": time.monotonic(), "sim_sec": self.clock,
            "source_stamp_sec": source_stamp, "data": data}, ensure_ascii=False) + "\n")

    def on_clock(self, msg):
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        self.last["/clock"] = time.monotonic()
        if self.clock is None or stamp != self.clock:
            self.clock_last_advance = time.monotonic()
        if self.clock is not None and stamp < self.clock:
            self.write("clock_reset", {"previous": self.clock, "next": stamp})
        self.clock = stamp

    def receive(self, topic, msg):
        now = time.monotonic()
        # Preserve every delivered health transition, including sub-second changes.
        self.last[topic] = now
        data = message_to_ordereddict(msg)
        if isinstance(msg, String):
            try:
                data = json.loads(msg.data)
            except ValueError:
                data = {"raw": msg.data}
        header = getattr(msg, "header", None)
        stamp = header.stamp.sec + header.stamp.nanosec / 1e9 if header else None
        self.last_source[topic] = stamp
        self.write(topic, data, stamp)

    def publish(self):
        now = time.monotonic()
        required = ["/clock", "/wheel_odom", "/gicp_loc/pose", "/gicp_loc/score",
                    "/agent/health_state", "/agent/health_dashboard_state", "/agent/recovery_state"]
        ages = {topic: now - stamp for topic, stamp in self.last.items()}
        missing = [topic for topic in required if topic not in ages]
        stale = [topic for topic in required if ages.get(topic, 0) > 3.0]
        clock_age = None if self.clock_last_advance is None else now - self.clock_last_advance
        # Wall-time heartbeat preserves clock stalls and RTF history even without sensor traffic.
        self.write("capture_heartbeat", {"receive_age_wall_sec": ages,
            "clock_advance_age_wall_sec": clock_age, "missing_topics": missing, "stale_topics": stale})
        (self.output / "b1_capture_status.json").write_text(json.dumps({
            "wall_unix_sec": time.time(), "sim_sec": self.clock, "record_counts": self.counts,
            "route_sha256": self.route["sha256"] if self.route else None,
            "required_topics": required, "receive_age_wall_sec": ages,
            "last_source_stamp_sec": self.last_source,
            "missing_topics": missing, "stale_topics": stale,
            "clock_advance_age_wall_sec": clock_age,
            "capture_receiving": not missing and not stale and clock_age is not None and clock_age <= 3.0,
            "note": "Missing counts mean missing evidence; this observer does not record raw LiDAR/images."}, indent=2))
        if not self.route:
            return
        markers = MarkerArray()
        def marker(index, kind):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = "b1_route_a", index, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = .1, .65, 1.0, 1.0
            markers.markers.append(m)
            return m
        line = marker(0, Marker.LINE_STRIP)
        line.scale.x = .07
        for index, pose in enumerate(self.route["poses"]):
            line.points.append(Point(x=float(pose["x"]), y=float(pose["y"]), z=.12))
            arrow = marker(index * 2 + 1, Marker.ARROW)
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = float(pose["x"]), float(pose["y"]), .15
            arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(pose["yaw"] / 2), math.cos(pose["yaw"] / 2)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .65, .14, .14
            label = marker(index * 2 + 2, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = float(pose["x"]), float(pose["y"]), .65
            label.scale.z = .3
            label.text = f"{index}: {self.route['waypoints'][index].get('id', index)}"
        self.publisher.publish(markers)

    def close(self):
        self.write("observer_stopped", {"counts": dict(self.counts)})
        self.file.close()
