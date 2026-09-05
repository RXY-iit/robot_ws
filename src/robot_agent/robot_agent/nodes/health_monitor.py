from __future__ import annotations

import json
import math
import os
import shutil
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from my_messages.msg import DriveMotor, SteerMotor
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


BAD_ITEM_STATES = {"BLOCKED", "ERROR", "STALE"}
UNCERTAIN_ITEM_STATES = {"UNKNOWN", "NOT_INSTRUMENTED", "NOT_OBSERVED"}
DEGRADED_ITEM_STATES = {"DEGRADED", "WARN", "ESTIMATED", "REPORTED"}
NON_PROVING_ITEM_STATES = {"IDLE"}
MODBUS_RECOVERY_WINDOW_SEC = 10.0
SO101_ARM_MOTORS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def finite_or_none(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def diagnostic_level_number(value: Any) -> int:
    """Normalize ROS diagnostic level across generated Python representations."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError(f"Diagnostic level must contain exactly one byte, got {value!r}")
        return int(value[0])
    return int(value)


def json_safe(value: Any) -> Any:
    """Replace non-finite floats instead of emitting non-standard JSON tokens."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def aggregate_status(items: list[dict[str, Any]]) -> tuple[str, dict[str, Any] | None]:
    """Aggregate only critical facts while preserving uncertainty.

    This deliberately does not treat NOT_INSTRUMENTED as healthy and does not
    let optional/event-driven facts turn a domain red.
    """
    critical = [item for item in items if item.get("critical")]
    for item in critical:
        if item.get("status") == "BLOCKED":
            return "BLOCKED", item
    for item in critical:
        if item.get("status") in {"ERROR", "STALE"}:
            return "DEGRADED", item
    # An observed degradation must not be hidden by a different item whose
    # instrumentation is still unknown.  UNKNOWN remains conservative only
    # when there is no directly observed fault.
    for item in critical:
        if item.get("status") in DEGRADED_ITEM_STATES:
            return "DEGRADED", item
    for item in critical:
        if item.get("status") in UNCERTAIN_ITEM_STATES:
            return "UNKNOWN", item
    if not critical:
        return "UNKNOWN", None
    return "AVAILABLE", None


def combine_status(groups: list[tuple[str, dict[str, Any] | None]]) -> tuple[str, dict[str, Any] | None]:
    for wanted in ("BLOCKED", "DEGRADED", "UNKNOWN"):
        for status, reason in groups:
            if status == wanted:
                return status, reason
    return "AVAILABLE", None


@dataclass
class SignalObservation:
    source: str
    evidence: str
    count: int = 0
    last_receive_monotonic: float | None = None
    last_receive_utc: str | None = None
    source_stamp_sec: float | None = None
    value: Any = None
    receive_times: deque[float] = field(default_factory=lambda: deque(maxlen=256))

    def update(
        self,
        value: Any,
        *,
        now_monotonic: float | None = None,
        source_stamp_sec: float | None = None,
    ) -> None:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        self.count += 1
        self.last_receive_monotonic = now
        self.last_receive_utc = utc_now()
        self.source_stamp_sec = source_stamp_sec
        self.value = value
        self.receive_times.append(now)

    def snapshot(self, now_monotonic: float, ros_now_sec: float) -> dict[str, Any]:
        age = None
        if self.last_receive_monotonic is not None:
            age = max(0.0, now_monotonic - self.last_receive_monotonic)
        recent = [stamp for stamp in self.receive_times if now_monotonic - stamp <= 5.0]
        rate = None
        if len(recent) >= 2 and recent[-1] > recent[0]:
            rate = (len(recent) - 1) / (recent[-1] - recent[0])
        gaps = [later - earlier for earlier, later in zip(recent, recent[1:])]
        if self.last_receive_monotonic is not None:
            gaps.append(max(0.0, now_monotonic - self.last_receive_monotonic))
        max_gap = max(gaps) if gaps else None
        source_age = None
        if self.source_stamp_sec is not None:
            candidate = ros_now_sec - self.source_stamp_sec
            if math.isfinite(candidate) and candidate >= -0.05:
                source_age = max(0.0, candidate)
        return {
            "source": self.source,
            "evidence": self.evidence,
            "message_count": self.count,
            "last_receive_at": self.last_receive_utc,
            "receive_age_sec": age,
            "source_stamp_sec": self.source_stamp_sec,
            "source_age_sec": source_age,
            "receive_rate_hz_5s": rate,
            "max_receive_gap_sec_5s": max_gap,
            "observation_span_sec": (
                max(0.0, self.receive_times[-1] - self.receive_times[0])
                if len(self.receive_times) >= 2
                else 0.0
            ),
            "value": self.value,
        }


class HealthMonitor(Node):
    """Read-only V1 evidence collector for the seven-domain Health Dashboard."""

    def __init__(self) -> None:
        super().__init__("health_monitor")
        # Safety loops run in their hardware nodes; this page is supervisory.
        # Publishing the large evidence tree at 0.5 Hz keeps it useful while
        # avoiding unnecessary JSON/diagnostic work during Full bringup.
        self.declare_parameter("publish_rate_hz", 0.5)
        self.declare_parameter("input_sample_rate_hz", 5.0)
        self.declare_parameter("gicp_score_threshold", 0.25)
        self.declare_parameter("lifecycle_request_timeout_sec", 2.0)

        self.publisher = self.create_publisher(String, "/agent/health_dashboard_state", 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.signals: dict[str, SignalObservation] = {}
        self._signals_lock = threading.RLock()
        # Subscriptions stay in the node's default mutually-exclusive group.
        # Slow host/graph collection and the large JSON publish run in a second
        # group so they cannot starve direct topic callbacks.
        self._work_group = MutuallyExclusiveCallbackGroup()
        self.sequence = 0
        self._last_publish_duration_sec: float | None = None
        self._last_cpu_total: int | None = None
        self._last_cpu_idle: int | None = None
        self._process_cpu_previous: dict[int, int] = {}
        self._process_cpu_total_previous: int | None = None
        self._process_cpu_last_sample: float | None = None
        self._process_cpu_top: list[dict[str, Any]] = []
        self._process_cpu_window_sec: float | None = None
        self._host_metrics: dict[str, Any] = {}
        self._last_mark_monotonic: dict[str, float] = {}
        # Keep the Future identity as well as its start time.  A lifecycle
        # service request can otherwise remain pending forever and suppress all
        # later polls even though the lifecycle node itself is still active.
        self._lifecycle_pending: dict[str, tuple[Any, float]] = {}
        self._lifecycle_clients: dict[str, Any] = {}
        self._lifecycle_request_timeout_sec = max(
            0.5,
            float(self.get_parameter("lifecycle_request_timeout_sec").value),
        )
        sample_rate = max(1.0, float(self.get_parameter("input_sample_rate_hz").value))
        self._input_sample_interval_sec = 1.0 / sample_rate

        self._setup_subscriptions()
        self._setup_lifecycle_clients()
        self.create_timer(1.0, self._collect_host_metrics, callback_group=self._work_group)
        self.create_timer(1.0, self._poll_lifecycle, callback_group=self._work_group)
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish, callback_group=self._work_group)
        self.get_logger().info(
            "HealthMonitor V1 ready: publishing read-only evidence on "
            "/agent/health_dashboard_state"
        )

    def _setup_subscriptions(self) -> None:
        configured_sources = {
            "drive_feedback": ("/drive_odom", "DIRECT_TOPIC"),
            "steer_feedback": ("/steer_odom", "DIRECT_TOPIC"),
            "drive_health": ("/drive/health_summary", "DIRECT_DRIVER_SUMMARY"),
            "dynamixel_health": ("/dynamixel/health_summary", "DIRECT_DRIVER_SUMMARY"),
            "so101_arm_health": ("/arm/hardware_status", "DIRECT_DEVICE_DIAGNOSTIC"),
            "so101_arm_gateway": ("/arm/dashboard_state", "STRUCTURED_REPORT"),
            "so101_arm_visual_joints": ("/arm/visual_joint_states", "DERIVED_TF_INPUT"),
            "camera_pan": ("/chokudomotor/angle", "DIRECT_TOPIC"),
            "camera_tilt": ("/cameraswingmotor/angle", "DIRECT_TOPIC"),
            "lift_position": ("/lift/position", "SOFTWARE_ESTIMATE"),
            "lift_state": ("/lift/state", "STRUCTURED_REPORT"),
            "lift_health": ("/lift/health_summary", "DIRECT_DRIVER_SUMMARY"),
            "robot_mode": ("/robot_mode", "DIRECT_TOPIC"),
            "safety": ("/safety_status", "DIRECT_TOPIC"),
            "joy_source": ("/joy/source_status", "DIRECT_ARBITER_STATUS"),
            "cmd_vel_raw": ("/cmd_vel_raw", "DIRECT_TOPIC"),
            "cmd_vel": ("/cmd_vel", "DIRECT_TOPIC"),
            "nav2_cmd_vel": ("/nav2/cmd_vel", "DIRECT_TOPIC"),
            "motion_policy": ("/motion_policy/status", "DIRECT_TOPIC"),
            "lidar_device_health": ("/livox/device_health", "DIRECT_DEVICE_DIAGNOSTIC"),
            "camera_color_diagnostics": ("/diagnostics", "DRIVER_DIAGNOSTIC"),
            "camera_depth_diagnostics": ("/diagnostics", "DRIVER_DIAGNOSTIC"),
            "camera_temperatures": ("/diagnostics", "DIRECT_DEVICE_DIAGNOSTIC"),
            "gicp_score": ("/gicp_loc/score", "DIRECT_TOPIC"),
            "gicp_pose": ("/gicp_loc/pose", "ACCEPTED_ALGORITHM_OUTPUT"),
            "fast_lio_odom": ("/fast_lio/odometry", "DIRECT_TOPIC"),
            "global_plan": ("/plan", "DIRECT_TOPIC"),
            "navigation_progress": ("/agent/navigation_progress", "STRUCTURED_REPORT"),
            "navigation_route": ("/agent/navigation_route_state", "STRUCTURED_REPORT"),
            "agent_heartbeat": ("/agent/health_state", "STRUCTURED_REPORT"),
            "agent_status": ("/agent/status", "STRUCTURED_REPORT"),
            "mission_state": ("/agent/mission_state", "STRUCTURED_REPORT"),
            "real_robot_state": ("/agent/real_robot_state", "STRUCTURED_REPORT"),
        }
        for key, (source, evidence) in configured_sources.items():
            self._signal(key, source, evidence)

        self.create_subscription(DriveMotor, "/drive_odom", self._drive_cb, 10)
        self.create_subscription(SteerMotor, "/steer_odom", self._steer_cb, 10)
        self.create_subscription(
            String,
            "/drive/health_summary",
            self._json_or_text_cb("drive_health", "/drive/health_summary"),
            10,
        )
        self.create_subscription(
            String,
            "/dynamixel/health_summary",
            self._json_or_text_cb("dynamixel_health", "/dynamixel/health_summary"),
            10,
        )
        self.create_subscription(
            String,
            "/arm/hardware_status",
            self._json_or_text_cb("so101_arm_health", "/arm/hardware_status"),
            10,
        )
        self.create_subscription(
            String,
            "/arm/dashboard_state",
            self._json_or_text_cb("so101_arm_gateway", "/arm/dashboard_state"),
            10,
        )
        self.create_subscription(
            JointState,
            "/arm/visual_joint_states",
            self._arm_visual_joint_state_cb,
            10,
        )
        self.create_subscription(Float32, "/chokudomotor/angle", self._camera_pan_cb, 10)
        self.create_subscription(Float32, "/cameraswingmotor/angle", self._camera_tilt_cb, 10)
        self.create_subscription(Float32, "/lift/position", self._lift_position_cb, 10)
        self.create_subscription(String, "/lift/state", self._json_or_text_cb("lift_state", "/lift/state"), 10)
        self.create_subscription(
            String,
            "/lift/health_summary",
            self._json_or_text_cb("lift_health", "/lift/health_summary"),
            10,
        )

        self.create_subscription(String, "/robot_mode", self._text_cb("robot_mode", "/robot_mode"), 10)
        self.create_subscription(String, "/safety_status", self._text_cb("safety", "/safety_status"), 10)
        self.create_subscription(
            String,
            "/joy/source_status",
            self._json_or_text_cb("joy_source", "/joy/source_status"),
            10,
        )
        self.create_subscription(Twist, "/cmd_vel_raw", self._twist_cb("cmd_vel_raw", "/cmd_vel_raw"), 10)
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb("cmd_vel", "/cmd_vel"), 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._twist_cb("nav2_cmd_vel", "/nav2/cmd_vel"), 10)
        self.create_subscription(
            String, "/motion_policy/status", self._text_cb("motion_policy", "/motion_policy/status"), 10
        )

        self.create_subscription(
            String,
            "/livox/device_health",
            self._json_or_text_cb("lidar_device_health", "/livox/device_health"),
            10,
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._diagnostics_cb, 10
        )

        self.create_subscription(Float32, "/gicp_loc/score", self._gicp_score_cb, 10)
        self.create_subscription(PoseStamped, "/gicp_loc/pose", self._gicp_pose_cb, 10)
        self.create_subscription(
            Odometry, "/fast_lio/odometry", self._fast_lio_cb, qos_profile_sensor_data
        )

        self.create_subscription(NavPath, "/plan", self._plan_cb, 10)
        self.create_subscription(
            String,
            "/agent/navigation_progress",
            self._json_or_text_cb("navigation_progress", "/agent/navigation_progress"),
            10,
        )
        self.create_subscription(
            String,
            "/agent/navigation_route_state",
            self._json_or_text_cb("navigation_route", "/agent/navigation_route_state"),
            10,
        )

        self.create_subscription(
            String, "/agent/health_state", self._json_or_text_cb("agent_heartbeat", "/agent/health_state"), 10
        )
        self.create_subscription(String, "/agent/status", self._text_cb("agent_status", "/agent/status"), 10)
        self.create_subscription(
            String, "/agent/mission_state", self._json_or_text_cb("mission_state", "/agent/mission_state"), 10
        )
        self.create_subscription(
            String,
            "/agent/real_robot_state",
            self._json_or_text_cb("real_robot_state", "/agent/real_robot_state"),
            10,
        )

    def _setup_lifecycle_clients(self) -> None:
        services = {
            "controller_server": "/controller_server/get_state",
            "planner_server": "/planner_server/get_state",
            "bt_navigator": "/bt_navigator/get_state",
            "local_costmap": "/local_costmap/local_costmap/get_state",
        }
        for key, service in services.items():
            self._signal(f"lifecycle_{key}", service, "DIRECT_SERVICE_RESPONSE")
            self._lifecycle_clients[key] = (
                service,
                self.create_client(GetState, service, callback_group=self._work_group),
            )

    def _signal(self, key: str, source: str, evidence: str = "DIRECT_TOPIC") -> SignalObservation:
        signal = self.signals.get(key)
        if signal is None:
            signal = SignalObservation(source=source, evidence=evidence)
            self.signals[key] = signal
        return signal

    def _mark(
        self,
        key: str,
        source: str,
        value: Any,
        *,
        evidence: str = "DIRECT_TOPIC",
        source_stamp_sec: float | None = None,
        min_interval_sec: float = 0.0,
    ) -> None:
        now = time.monotonic()
        with self._signals_lock:
            last = self._last_mark_monotonic.get(key)
            if min_interval_sec > 0.0 and last is not None and now - last < min_interval_sec:
                return
            self._last_mark_monotonic[key] = now
            self._signal(key, source, evidence).update(
                value,
                now_monotonic=now,
                source_stamp_sec=source_stamp_sec,
            )

    @staticmethod
    def _header_stamp(msg: Any) -> float | None:
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return None
        sec = int(getattr(stamp, "sec", 0))
        nanosec = int(getattr(stamp, "nanosec", 0))
        if sec == 0 and nanosec == 0:
            return None
        return sec + nanosec / 1_000_000_000.0

    @staticmethod
    def _twist_value(msg: Twist) -> dict[str, float]:
        return {
            "linear_x": float(msg.linear.x),
            "linear_y": float(msg.linear.y),
            "angular_z": float(msg.angular.z),
        }

    def _text_cb(self, key: str, source: str) -> Callable[[String], None]:
        def callback(msg: String) -> None:
            self._mark(key, source, str(msg.data))

        return callback

    def _json_or_text_cb(self, key: str, source: str) -> Callable[[String], None]:
        def callback(msg: String) -> None:
            try:
                value = json.loads(msg.data)
            except (TypeError, ValueError):
                value = str(msg.data)
            self._mark(key, source, value, evidence="STRUCTURED_REPORT")

        return callback

    def _twist_cb(self, key: str, source: str) -> Callable[[Twist], None]:
        def callback(msg: Twist) -> None:
            self._mark(
                key,
                source,
                self._twist_value(msg),
                min_interval_sec=self._input_sample_interval_sec,
            )

        return callback

    def _drive_cb(self, msg: DriveMotor) -> None:
        self._mark(
            "drive_feedback",
            "/drive_odom",
            {"vel1_m_s": float(msg.vel1), "vel2_m_s": float(msg.vel2), "vel3_m_s": float(msg.vel3)},
            source_stamp_sec=self._header_stamp(msg),
            min_interval_sec=self._input_sample_interval_sec,
        )

    def _steer_cb(self, msg: SteerMotor) -> None:
        self._mark(
            "steer_feedback",
            "/steer_odom",
            {"phi1_rad": float(msg.phi1), "phi2_rad": float(msg.phi2), "phi3_rad": float(msg.phi3)},
            source_stamp_sec=self._header_stamp(msg),
            min_interval_sec=self._input_sample_interval_sec,
        )

    def _camera_pan_cb(self, msg: Float32) -> None:
        self._mark("camera_pan", "/chokudomotor/angle", {"angle_deg": float(msg.data)})

    def _arm_visual_joint_state_cb(self, msg: JointState) -> None:
        self._mark(
            "so101_arm_visual_joints",
            "/arm/visual_joint_states",
            {
                "names": list(msg.name),
                "positions": [float(value) for value in msg.position],
            },
            evidence="DERIVED_TF_INPUT",
            source_stamp_sec=self._header_stamp(msg),
            min_interval_sec=self._input_sample_interval_sec,
        )

    def _camera_tilt_cb(self, msg: Float32) -> None:
        self._mark("camera_tilt", "/cameraswingmotor/angle", {"angle_deg": float(msg.data)})

    def _lift_position_cb(self, msg: Float32) -> None:
        self._mark(
            "lift_position",
            "/lift/position",
            {"estimated_position_mm": float(msg.data)},
            evidence="SOFTWARE_ESTIMATE",
        )

    def _diagnostics_cb(self, msg: DiagnosticArray) -> None:
        """Keep only RealSense's low-rate native diagnostics.

        health_monitor also publishes to /diagnostics.  Match only the three
        exact status names observed from the real D435 on 2026-08-19 so its own
        messages (and unrelated Nav2 diagnostics) cannot become camera evidence.
        """
        for status in msg.status:
            name = str(status.name)
            key = self._realsense_diagnostic_key(name)
            if key is None:
                continue
            value = {
                "name": name,
                "hardware_id": str(status.hardware_id),
                "level": diagnostic_level_number(status.level),
                "message": str(status.message),
                "values": {str(item.key): str(item.value) for item in status.values},
            }
            self._mark(key, "/diagnostics", value, evidence="DRIVER_DIAGNOSTIC")

    @staticmethod
    def _realsense_diagnostic_key(name: str) -> str | None:
        return {
            "camera: color": "camera_color_diagnostics",
            "camera: depth": "camera_depth_diagnostics",
            "camera: temperatures": "camera_temperatures",
        }.get(str(name).strip().lower())

    def _gicp_score_cb(self, msg: Float32) -> None:
        self._mark(
            "gicp_score",
            "/gicp_loc/score",
            {"inlier_rmse": float(msg.data)},
            min_interval_sec=self._input_sample_interval_sec,
        )

    def _gicp_pose_cb(self, msg: PoseStamped) -> None:
        pose = msg.pose
        self._mark(
            "gicp_pose",
            "/gicp_loc/pose",
            {
                "frame_id": msg.header.frame_id,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "quaternion": {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                },
            },
            evidence="ACCEPTED_ALGORITHM_OUTPUT",
            source_stamp_sec=self._header_stamp(msg),
            min_interval_sec=self._input_sample_interval_sec,
        )

    def _fast_lio_cb(self, msg: Odometry) -> None:
        self._mark(
            "fast_lio_odom",
            "/fast_lio/odometry",
            {
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "z": float(msg.pose.pose.position.z),
            },
            source_stamp_sec=self._header_stamp(msg),
            min_interval_sec=self._input_sample_interval_sec,
        )

    def _plan_cb(self, msg: NavPath) -> None:
        self._mark(
            "global_plan",
            "/plan",
            {"frame_id": msg.header.frame_id, "pose_count": len(msg.poses)},
            source_stamp_sec=self._header_stamp(msg),
        )

    def _poll_lifecycle(self) -> None:
        now_monotonic = time.monotonic()
        for key, (service, client) in self._lifecycle_clients.items():
            pending = self._lifecycle_pending.get(key)
            if pending is not None:
                pending_future, started_monotonic = pending
                if now_monotonic - started_monotonic < self._lifecycle_request_timeout_sec:
                    continue
                # Remove before cancel(), because cancellation may invoke the
                # done callback immediately.  That callback must be treated as
                # a late result and must not remove a newer retry.
                self._lifecycle_pending.pop(key, None)
                pending_future.cancel()
                self.get_logger().warn(
                    f"Lifecycle state query timed out for {service}; retrying"
                )
            if not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            self._lifecycle_pending[key] = (future, now_monotonic)
            future.add_done_callback(
                lambda completed, item_key=key, item_service=service: self._lifecycle_result(
                    item_key, item_service, completed
                )
            )

    def _lifecycle_result(self, key: str, service: str, future: Any) -> None:
        pending = self._lifecycle_pending.get(key)
        if pending is None or pending[0] is not future:
            return
        self._lifecycle_pending.pop(key, None)
        if future.cancelled():
            return
        try:
            response = future.result()
            value = {
                "id": int(response.current_state.id),
                "label": str(response.current_state.label),
            }
            self._mark(f"lifecycle_{key}", service, value, evidence="DIRECT_SERVICE_RESPONSE")
        except Exception as exc:
            self.get_logger().warn(f"Lifecycle state query failed for {service}: {exc!r}")

    def _collect_host_metrics(self) -> None:
        metrics: dict[str, Any] = {
            "source": "/proc + filesystem + hwmon + systemd-timesyncd marker"
        }
        cpu_total_ticks = None
        try:
            cpu_fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()[1:]
            values = [int(value) for value in cpu_fields]
            total = sum(values)
            cpu_total_ticks = total
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            if self._last_cpu_total is not None and total > self._last_cpu_total:
                total_delta = total - self._last_cpu_total
                idle_delta = idle - (self._last_cpu_idle or 0)
                metrics["cpu_used_percent"] = 100.0 * (1.0 - idle_delta / total_delta)
            self._last_cpu_total = total
            self._last_cpu_idle = idle
        except (OSError, ValueError, IndexError):
            metrics["cpu_used_percent"] = None

        self._update_process_cpu(cpu_total_ticks)
        metrics["logical_cpu_count"] = os.cpu_count() or 1
        metrics["process_cpu_window_sec"] = self._process_cpu_window_sec
        metrics["top_processes"] = self._process_cpu_top

        try:
            load_1m, load_5m, load_15m = Path("/proc/loadavg").read_text(
                encoding="utf-8"
            ).split()[:3]
            metrics["load_average"] = {
                "1m": float(load_1m),
                "5m": float(load_5m),
                "15m": float(load_15m),
            }
        except (OSError, ValueError, IndexError):
            metrics["load_average"] = None

        try:
            meminfo: dict[str, int] = {}
            for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
                key, raw = line.split(":", 1)
                meminfo[key] = int(raw.strip().split()[0])
            total_kib = meminfo.get("MemTotal", 0)
            available_kib = meminfo.get("MemAvailable", 0)
            metrics["memory_total_bytes"] = total_kib * 1024
            metrics["memory_available_bytes"] = available_kib * 1024
            metrics["memory_used_percent"] = (
                100.0 * (total_kib - available_kib) / total_kib if total_kib else None
            )
        except (OSError, ValueError):
            metrics["memory_used_percent"] = None

        try:
            disk = shutil.disk_usage(Path.cwd())
            metrics.update(
                disk_total_bytes=disk.total,
                disk_free_bytes=disk.free,
                disk_used_percent=100.0 * disk.used / disk.total if disk.total else None,
            )
        except OSError:
            metrics["disk_used_percent"] = None

        temperatures: list[float] = []
        for temp_path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
            try:
                raw = float(temp_path.read_text(encoding="utf-8").strip())
                temperatures.append(raw / 1000.0 if abs(raw) > 1000.0 else raw)
            except (OSError, ValueError):
                continue
        metrics["thermal_zone_max_c"] = max(temperatures) if temperatures else None

        hwmon_sensors: list[dict[str, Any]] = []
        for hwmon_dir in Path("/sys/class/hwmon").glob("hwmon*"):
            try:
                device_name = (hwmon_dir / "name").read_text(encoding="utf-8").strip()
            except OSError:
                device_name = hwmon_dir.name
            for input_path in hwmon_dir.glob("temp*_input"):
                prefix = input_path.stem.removesuffix("_input")

                def read_temperature(suffix: str) -> float | None:
                    try:
                        raw = float(
                            (hwmon_dir / f"{prefix}_{suffix}")
                            .read_text(encoding="utf-8")
                            .strip()
                        )
                    except (OSError, ValueError):
                        return None
                    return raw / 1000.0 if abs(raw) > 1000.0 else raw

                try:
                    label = (hwmon_dir / f"{prefix}_label").read_text(
                        encoding="utf-8"
                    ).strip()
                except OSError:
                    label = prefix
                input_c = read_temperature("input")
                if input_c is None:
                    continue
                max_c = read_temperature("max")
                crit_c = read_temperature("crit")
                hwmon_sensors.append(
                    {
                        "device": device_name,
                        "label": label,
                        "input_c": input_c,
                        "max_c": max_c,
                        "crit_c": crit_c,
                        "headroom_to_max_c": max_c - input_c if max_c is not None else None,
                        "headroom_to_crit_c": crit_c - input_c if crit_c is not None else None,
                    }
                )
        metrics["hwmon_temperatures"] = hwmon_sensors
        metrics["hwmon_max_c"] = (
            max(sensor["input_c"] for sensor in hwmon_sensors) if hwmon_sensors else None
        )
        metrics["host_ntp_synchronized"] = Path(
            "/run/systemd/timesync/synchronized"
        ).exists()
        metrics["observed_at"] = utc_now()
        self._host_metrics = metrics

    @staticmethod
    def _process_display_name(command: str, comm: str) -> str:
        lowered = command.lower()
        labels = (
            ("gicp_localizer", "GICP localizer"),
            ("controller_server", "Nav2 controller"),
            ("rviz2", "RViz"),
            ("health_monitor", "Health Monitor"),
            ("agent_node", "Agent"),
            ("drive_motor.py", "BLVD drive bridge"),
            ("om_modbusrtu_node", "Modbus RTU"),
            ("livox_ros_driver", "Livox driver"),
            ("realsense2_camera", "RealSense driver"),
            ("nav_mode_switch", "Navigation mode switch"),
            ("component_container", "ROS component container"),
            ("chrome", "Chrome"),
            ("discord", "Discord"),
        )
        for marker, label in labels:
            if marker in lowered:
                return label
        return comm or command.split(" ", 1)[0] or "unknown"

    def _update_process_cpu(self, cpu_total_ticks: int | None) -> None:
        now = time.monotonic()
        if cpu_total_ticks is None:
            return
        if (
            self._process_cpu_last_sample is not None
            and now - self._process_cpu_last_sample < 5.0
        ):
            return
        current: dict[int, tuple[int, str, str]] = {}
        for proc_dir in Path("/proc").glob("[0-9]*"):
            try:
                pid = int(proc_dir.name)
                raw_stat = (proc_dir / "stat").read_text(encoding="utf-8")
                close = raw_stat.rfind(")")
                if close < 0:
                    continue
                comm = raw_stat[raw_stat.find("(") + 1 : close]
                fields = raw_stat[close + 2 :].split()
                ticks = int(fields[11]) + int(fields[12])
                raw_cmdline = (proc_dir / "cmdline").read_bytes()
                command = raw_cmdline.replace(b"\0", b" ").decode(
                    "utf-8", errors="replace"
                ).strip()
                current[pid] = (ticks, command, comm)
            except (OSError, ValueError, IndexError):
                continue
        previous_total = self._process_cpu_total_previous
        previous = self._process_cpu_previous
        previous_time = self._process_cpu_last_sample
        self._process_cpu_previous = {pid: item[0] for pid, item in current.items()}
        self._process_cpu_total_previous = cpu_total_ticks
        self._process_cpu_last_sample = now
        if previous_total is None or cpu_total_ticks <= previous_total or previous_time is None:
            return
        total_delta = cpu_total_ticks - previous_total
        cpu_count = max(1, os.cpu_count() or 1)
        rows = []
        for pid, (ticks, command, comm) in current.items():
            old_ticks = previous.get(pid)
            if old_ticks is None or ticks < old_ticks:
                continue
            delta = ticks - old_ticks
            if delta <= 0:
                continue
            host_capacity_percent = 100.0 * delta / total_delta
            core_equivalent = delta * cpu_count / total_delta
            rows.append(
                {
                    "pid": pid,
                    "name": self._process_display_name(command, comm),
                    "host_capacity_percent": host_capacity_percent,
                    "one_core_percent": core_equivalent * 100.0,
                    "cpu_core_equivalent": core_equivalent,
                    "command": command[:180] or comm,
                }
            )
        rows.sort(key=lambda item: item["host_capacity_percent"], reverse=True)
        self._process_cpu_top = rows[:10]
        self._process_cpu_window_sec = now - previous_time

    def _signal_item(
        self,
        key: str,
        label: str,
        *,
        stale_sec: float | None,
        standard: str,
        item_id: str | None = None,
        critical: bool = False,
        fresh_status: str = "OBSERVED",
        fresh_summary: str = "Fresh messages observed at this monitor",
        unobserved_status: str = "UNKNOWN",
        unobserved_summary: str = "No message has reached this monitor",
        stale_status: str = "STALE",
        validator: Callable[[Any], tuple[str, str]] | None = None,
        value_selector: Callable[[Any], Any] | None = None,
    ) -> dict[str, Any]:
        now_mono = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        with self._signals_lock:
            signal = self.signals.get(key)
            snapshot = signal.snapshot(now_mono, ros_now) if signal is not None else None
        if snapshot is None:
            return {
                "id": item_id or key,
                "label": label,
                "status": unobserved_status,
                "summary": unobserved_summary,
                "source": "configured ROS source",
                "evidence": "RECEIVER_MEASURED",
                "critical": critical,
                "standard": standard,
                "receive_age_sec": None,
                "source_age_sec": None,
                "receive_rate_hz_5s": None,
                "value": None,
            }
        age = snapshot["receive_age_sec"]
        status, summary = fresh_status, fresh_summary
        if age is None:
            status, summary = unobserved_status, unobserved_summary
        else:
            if validator is not None:
                status, summary = validator(snapshot["value"])
            if stale_sec is not None and age > stale_sec:
                stale_summary = f"Last message was received {age:.2f}s ago"
                # Do not erase a confirmed fault merely because the evidence
                # later became old.  Staleness replaces only a non-fault state.
                if status in {"BLOCKED", "ERROR"}:
                    summary = f"{summary}; {stale_summary}"
                else:
                    status, summary = stale_status, stale_summary
        max_gap = snapshot.get("max_receive_gap_sec_5s")
        if (
            stale_sec is not None
            and max_gap is not None
            and max_gap > stale_sec
            and status in {"OBSERVED", "OK", "AVAILABLE", "ESTIMATED", "REPORTED", "RECOVERED"}
        ):
            status = "DEGRADED"
            summary = (
                f"Maximum receiver-observed gap in the last 5s was {max_gap:.2f}s "
                f"(limit {stale_sec:.2f}s)"
            )
        if value_selector is not None:
            snapshot["value"] = value_selector(snapshot["value"])
        return {
            "id": item_id or key,
            "label": label,
            "status": status,
            "summary": summary,
            "critical": critical,
            "standard": standard,
            **snapshot,
        }

    @staticmethod
    def _fixed_item(
        key: str,
        label: str,
        status: str,
        summary: str,
        source: str,
        evidence: str,
        *,
        critical: bool = False,
        standard: str = "No automated rule in V1",
        value: Any = None,
    ) -> dict[str, Any]:
        return {
            "id": key,
            "label": label,
            "status": status,
            "summary": summary,
            "source": source,
            "evidence": evidence,
            "critical": critical,
            "standard": standard,
            "receive_age_sec": None,
            "source_age_sec": None,
            "receive_rate_hz_5s": None,
            "value": value,
        }

    @staticmethod
    def _modbus_direct_recovery_complete(value: dict[str, Any], event_age: float) -> bool:
        """Require post-error evidence from the transaction and all three drives."""
        last_read_age = finite_or_none(value.get("last_read_response_age_sec"))
        direct_io_age = finite_or_none(value.get("last_direct_io_age_sec"))
        electrical_age = finite_or_none(value.get("last_electrical_age_sec"))
        if any(
            age is None or age > 3.5 or age >= event_age
            for age in (last_read_age, direct_io_age, electrical_age)
        ):
            return False
        alarms = value.get("alarm_status")
        if not isinstance(alarms, list):
            return False
        by_id = {
            int(item.get("slave_id", -1)): item
            for item in alarms
            if isinstance(item, dict)
        }
        for slave_id in (1, 2, 3):
            sample = by_id.get(slave_id)
            if sample is None or int(sample.get("present_alarm", -1)) != 0:
                return False
            sample_age = finite_or_none(sample.get("sample_age_sec"))
            if sample_age is None or sample_age > 3.5 or sample_age >= event_age:
                return False
        return True

    @staticmethod
    def _modbus_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid drive health summary"
        state = value.get("state")
        if not isinstance(state, dict):
            return "UNKNOWN", "Drive summary has no /om_state0 state snapshot yet"
        error = int(state.get("state_error", -1))
        message = int(state.get("state_message", -1))
        if error != 0 or message == 2:
            return "ERROR", f"Modbus reports state_error={error}, state_message={message}"
        response_age = finite_or_none(value.get("last_response_age_sec"))
        if response_age is None:
            return "UNKNOWN", "No completed Modbus response has been observed"
        if response_age > 1.0:
            return "STALE", f"Last completed Modbus response is {response_age:.2f}s old"
        driver_errors = int(value.get("driver_error_event_count", 0))
        if driver_errors > 0:
            error_age = finite_or_none(value.get("last_driver_error_age_sec"))
            age_text = "unknown age" if error_age is None else f"{error_age:.1f}s ago"
            if (
                error_age is not None
                and error_age >= MODBUS_RECOVERY_WINDOW_SEC
                and HealthMonitor._modbus_direct_recovery_complete(value, error_age)
            ):
                return (
                    "RECOVERED",
                    f"Fresh post-error transactions and direct samples from IDs 1/2/3 "
                    f"have remained complete for at least {MODBUS_RECOVERY_WINDOW_SEC:.0f}s; "
                    f"{driver_errors} session error event(s) retained, last {age_text}",
                )
            return (
                "DEGRADED",
                f"{driver_errors} direct Modbus driver error event(s) occurred in this session; "
                f"last occurrence {age_text}; waiting for {MODBUS_RECOVERY_WINDOW_SEC:.0f}s "
                "of fresh post-error transaction, DIN3, electrical and three-driver alarm evidence",
            )
        recent_misses = int(
            value.get("completion_deadline_misses_5s", value.get("query_timeouts_5s", 0))
        )
        if recent_misses > 0:
            return (
                "DEGRADED",
                f"{recent_misses} completion callback deadline miss(es) observed in the last 5s; "
                "serial failure is not proven",
            )
        session_misses = int(
            value.get("completion_deadline_miss_count", value.get("query_timeout_count", 0))
        )
        if session_misses > 0:
            miss_age = finite_or_none(
                value.get(
                    "last_completion_deadline_miss_age_sec",
                    value.get("last_query_timeout_age_sec"),
                )
            )
            age_text = "unknown age" if miss_age is None else f"{miss_age:.1f}s ago"
            kind = value.get("last_completion_deadline_miss_query_kind")
            elapsed = finite_or_none(value.get("last_completion_deadline_miss_elapsed_sec"))
            context = ""
            if kind:
                context += f" during {kind}"
            if elapsed is not None:
                context += f" after {elapsed:.3f}s"
            last_read_age = finite_or_none(value.get("last_read_response_age_sec"))
            if (
                miss_age is not None
                and miss_age >= MODBUS_RECOVERY_WINDOW_SEC
                and last_read_age is not None
                and last_read_age <= 1.0
            ):
                return (
                    "RECOVERED",
                    f"Fresh read responses continued for at least "
                    f"{MODBUS_RECOVERY_WINDOW_SEC:.0f}s after {session_misses} callback "
                    f"deadline miss(es); last {age_text}{context}; event retained and "
                    "serial failure was not proven",
                )
            return (
                "DEGRADED",
                f"{session_misses} completion callback deadline miss(es) occurred in this drive session; "
                f"last occurrence {age_text}{context}; waiting for fresh read responses through "
                f"the {MODBUS_RECOVERY_WINDOW_SEC:.0f}s recovery window; serial failure is not proven",
            )
        return "OBSERVED", "Fresh completed Modbus responses observed by drive_motor"

    @staticmethod
    def _dynamixel_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid DYNAMIXEL health summary"
        motors = value.get("motors")
        if not isinstance(motors, list) or not motors:
            return "UNKNOWN", "No DYNAMIXEL read result has been summarized"
        if any(not bool(item.get("observed")) for item in motors if isinstance(item, dict)):
            return "UNKNOWN", "Not every configured DYNAMIXEL has a read result"
        failed = [
            item
            for item in motors
            if isinstance(item, dict)
            and (not bool(item.get("comm_success")) or int(item.get("device_error", 0)) != 0)
        ]
        if failed:
            ids = ", ".join(str(item.get("id")) for item in failed)
            return "ERROR", f"Latest direct DYNAMIXEL read failed or reported a device error: ID {ids}"
        ages = [
            finite_or_none(item.get("last_success_age_sec"))
            for item in motors
            if isinstance(item, dict)
        ]
        if any(age is None for age in ages):
            return "UNKNOWN", "DYNAMIXEL success age is unavailable"
        if any(age > 1.0 for age in ages if age is not None):
            return "STALE", "Latest successful DYNAMIXEL position read is more than 1s old"
        return "OBSERVED", "Fresh successful direct reads from all configured DYNAMIXEL IDs"

    @staticmethod
    def _blvd_direct_io_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid drive health summary"
        direct_io = value.get("direct_io")
        if not isinstance(direct_io, list) or len(direct_io) != 3:
            return "UNKNOWN", "No three-axis BLVD Direct I/O sample has been received"
        age = finite_or_none(value.get("last_direct_io_age_sec"))
        if age is None:
            return "UNKNOWN", "BLVD Direct I/O sample age is unavailable"
        if age > 3.5:
            return "STALE", f"Latest BLVD Direct I/O sample is {age:.2f}s old"
        states = ", ".join(
            f"ID {item.get('slave_id')} DIN3={int(bool(item.get('din3')))}"
            for item in direct_io
            if isinstance(item, dict)
        )
        return (
            "OBSERVED",
            f"Fresh raw Direct I/O received ({states}); installed wiring was qualified "
            "as released=1, pressed/locked=0 on 2026-08-19",
        )

    @staticmethod
    def _blvd_electrical_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid drive health summary"
        samples = value.get("electrical_status")
        if not isinstance(samples, list) or len(samples) != 3:
            return "UNKNOWN", "No three-axis BLVD electrical monitor sample has been received"
        age = finite_or_none(value.get("last_electrical_age_sec"))
        if age is None:
            return "UNKNOWN", "BLVD electrical monitor sample age is unavailable"
        if age > 3.5:
            return "STALE", f"Latest BLVD electrical monitor sample is {age:.2f}s old"
        fields = (
            "main_power_supply_current_a",
            "inverter_voltage_v",
            "main_power_supply_voltage_v",
        )
        if any(
            not isinstance(item, dict)
            or any(finite_or_none(item.get(field)) is None for field in fields)
            for item in samples
        ):
            return "ERROR", "BLVD electrical monitor contains missing or non-finite values"
        values = ", ".join(
            f"ID {item['slave_id']} {item['wheel_position']}: "
            f"{item['main_power_supply_current_a']:.3f}A, "
            f"inverter {item['inverter_voltage_v']:.1f}V, "
            f"main {item['main_power_supply_voltage_v']:.1f}V"
            for item in samples
        )
        return "OBSERVED", f"Fresh official BLVD monitor values received ({values})"

    @staticmethod
    def _blvd_alarm_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid drive health summary"
        samples = value.get("alarm_status")
        if not isinstance(samples, list) or not samples:
            return "UNKNOWN", "No direct Present alarm sample has been received"
        by_id = {
            int(item.get("slave_id")): item
            for item in samples
            if isinstance(item, dict) and item.get("slave_id") is not None
        }
        if set(by_id) != {1, 2, 3}:
            return "UNKNOWN", f"Expected BLVD alarm samples for IDs 1/2/3, observed {sorted(by_id)}"
        stale = [
            slave_id
            for slave_id, item in by_id.items()
            if finite_or_none(item.get("sample_age_sec")) is None
            or float(item["sample_age_sec"]) > 3.5
        ]
        if stale:
            return "STALE", f"BLVD alarm sample is stale for ID(s): {stale}"
        active = [
            (slave_id, int(item.get("present_alarm", 0)))
            for slave_id, item in by_id.items()
            if int(item.get("present_alarm", 0)) != 0
        ]
        if active:
            details = ", ".join(f"ID {slave_id}=0x{code:02X}" for slave_id, code in active)
            return "ERROR", f"BLVD Present alarm is active: {details}"
        changed = [
            (slave_id, int(item.get("alarm_history_1", 0)))
            for slave_id, item in by_id.items()
            if bool(item.get("alarm_history_changed_in_session"))
        ]
        if changed:
            details = ", ".join(f"ID {slave_id}=0x{code:02X}" for slave_id, code in changed)
            return (
                "RECOVERED",
                "BLVD Present alarms are zero and fresh; retained alarm history changed "
                f"during this drive session: {details}",
            )
        return "OBSERVED", "Direct Present alarm is 0 on BLVD IDs 1/2/3"

    @staticmethod
    def _drive_restart_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "UNKNOWN", "No real-robot recovery state is available"
        restart = value.get("drive_restart")
        if not isinstance(restart, dict):
            return "UNKNOWN", "No drive restart record is available"
        state = str(restart.get("state", "IDLE")).upper()
        message = str(restart.get("message") or "No drive restart requested")
        if state == "RUNNING":
            return "DEGRADED", message
        if state in {"FAILED", "REJECTED"}:
            return "ERROR", message
        if state == "SUCCEEDED":
            return "RECOVERED", f"Manual drive recovery succeeded: {message}"
        return "IDLE", message

    @staticmethod
    def _lift_health_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid lift health summary"
        serial_state = value.get("serial")
        motion = value.get("motion")
        if not isinstance(serial_state, dict) or not isinstance(motion, dict):
            return "UNKNOWN", "Lift summary has no serial or motion state"
        if not bool(serial_state.get("open")):
            return "ERROR", f"Lift serial port is not open: {serial_state.get('last_error') or 'unknown cause'}"
        if serial_state.get("usb_identity_verified") is False:
            return "ERROR", "Lift serial port is open but is not the verified Arduino UNO"
        write_errors = int(serial_state.get("write_error_count", 0))
        if write_errors > 0 and serial_state.get("last_error"):
            return "DEGRADED", f"Lift serial path recorded {write_errors} write/open error(s)"
        protocol = value.get("protocol")
        if isinstance(protocol, dict):
            if int(protocol.get("consecutive_ack_timeouts", 0)) > 0:
                return (
                    "DEGRADED",
                    "Lift Arduino did not acknowledge the latest command",
                )
            if not bool(protocol.get("ready")) or not bool(protocol.get("ack_supported")):
                return "DEGRADED", "Lift Arduino identity/command ACK is not verified"
        position = finite_or_none(motion.get("estimated_position_mm"))
        if position is None:
            return "UNKNOWN", "Lift software position estimate is unavailable"
        command = str(motion.get("command", "UNKNOWN"))
        reference = str(motion.get("position_reference", "UNREFERENCED"))
        return (
            "ESTIMATED",
            f"Software-integrated lift position {position:.1f} mm; command={command}; reference={reference}",
        )

    @staticmethod
    def _pick_fields(*keys: str) -> Callable[[Any], Any]:
        def select(value: Any) -> Any:
            if not isinstance(value, dict):
                return value
            return {key: value.get(key) for key in keys}

        return select

    @staticmethod
    def _physical_estop_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid drive health summary"
        direct_io = value.get("direct_io")
        if not isinstance(direct_io, list) or len(direct_io) != 3:
            return "UNKNOWN", "No three-axis BLVD Direct I/O sample has been received"
        age = finite_or_none(value.get("last_direct_io_age_sec"))
        if age is None:
            return "UNKNOWN", "BLVD Direct I/O sample age is unavailable"
        if age > 3.5:
            return "STALE", f"Latest BLVD Direct I/O sample is {age:.2f}s old"
        by_id = {
            int(item.get("slave_id")): bool(item.get("din3"))
            for item in direct_io
            if isinstance(item, dict) and item.get("slave_id") is not None
        }
        if set(by_id) != {1, 2, 3}:
            return "UNKNOWN", f"Expected BLVD IDs 1/2/3, observed {sorted(by_id)}"
        states = [by_id[dxl_id] for dxl_id in (1, 2, 3)]
        if all(states):
            return "OK", "Physical red stop is released (BLVD IDs 1/2/3 DIN3=1)"
        if not any(states):
            return "BLOCKED", "Physical red stop is pressed/locked (BLVD IDs 1/2/3 DIN3=0)"
        details = ", ".join(f"ID {dxl_id}={int(by_id[dxl_id])}" for dxl_id in (1, 2, 3))
        return "ERROR", f"Physical stop inputs disagree across BLVD drivers: {details}"

    @staticmethod
    def _dynamixel_identity_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid DYNAMIXEL health summary"
        motors = value.get("motors")
        if not isinstance(motors, list) or not motors:
            return "UNKNOWN", "No DYNAMIXEL identity result has been summarized"
        missing = [
            item.get("id")
            for item in motors
            if isinstance(item, dict) and not bool(item.get("identity_observed"))
        ]
        if missing:
            return "UNKNOWN", f"No successful model-number ping for DYNAMIXEL ID(s): {missing}"
        identities = [
            f"ID {item.get('id')}={item.get('model_number')}"
            + (f" ({item.get('model_name')})" if item.get("model_name") else "")
            for item in motors
            if isinstance(item, dict)
        ]
        return "OBSERVED", "Direct model-number ping: " + ", ".join(identities)

    @staticmethod
    def _dynamixel_hardware_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid DYNAMIXEL health summary"
        motors = value.get("motors")
        if not isinstance(motors, list) or not motors:
            return "UNKNOWN", "No DYNAMIXEL hardware-status result has been summarized"
        unsupported = [
            item.get("id")
            for item in motors
            if isinstance(item, dict) and not bool(item.get("hardware_status_supported"))
        ]
        if unsupported:
            return "UNKNOWN", f"No verified control table for DYNAMIXEL ID(s): {unsupported}"
        unobserved = [
            item.get("id")
            for item in motors
            if isinstance(item, dict) and not bool(item.get("hardware_observed"))
        ]
        if unobserved:
            return "UNKNOWN", f"No hardware-register sample for DYNAMIXEL ID(s): {unobserved}"
        failed = [
            item.get("id")
            for item in motors
            if isinstance(item, dict)
            and (
                not bool(item.get("hardware_comm_success"))
                or int(item.get("hardware_device_error", 0)) != 0
            )
        ]
        if failed:
            return "ERROR", f"DYNAMIXEL hardware-register read failed: ID(s) {failed}"
        stale = []
        for item in motors:
            if not isinstance(item, dict):
                continue
            age = finite_or_none(item.get("last_hardware_success_age_sec"))
            if age is None or age > 2.5:
                stale.append(item.get("id"))
        if stale:
            return "STALE", f"DYNAMIXEL hardware-register sample is stale: ID(s) {stale}"
        hardware_errors = [
            (item.get("id"), int(item.get("hardware_error_status", 0)))
            for item in motors
            if isinstance(item, dict) and int(item.get("hardware_error_status", 0)) != 0
        ]
        if hardware_errors:
            details = ", ".join(f"ID {dxl_id}=0x{error:02x}" for dxl_id, error in hardware_errors)
            return "ERROR", f"DYNAMIXEL Hardware Error Status is nonzero: {details}"
        details = ", ".join(
            f"ID {item.get('id')}: {float(item.get('input_voltage_v')):.1f}V, "
            f"{float(item.get('present_current_a')):.3f}A, "
            f"{int(item.get('temperature_c'))}C"
            for item in motors
            if isinstance(item, dict)
        )
        return "OK", f"Direct XH540-W270 hardware registers are clear ({details})"

    @staticmethod
    def _so101_arm_hardware_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid SO101 hardware status"
        required_maps = {
            key: value.get(key)
            for key in (
                "voltage_deci_v",
                "temperature_c",
                "current_raw",
                "torque_enable",
            )
        }
        missing_maps = [key for key, data in required_maps.items() if not isinstance(data, dict)]
        if missing_maps:
            return "UNKNOWN", f"SO101 status lacks register maps: {missing_maps}"
        missing_motors = {
            key: sorted(set(SO101_ARM_MOTORS) - set(data))
            for key, data in required_maps.items()
            if isinstance(data, dict) and set(SO101_ARM_MOTORS) - set(data)
        }
        if missing_motors:
            return "UNKNOWN", f"SO101 status lacks configured motors: {missing_motors}"
        invalid_torque = {
            name: required_maps["torque_enable"].get(name)
            for name in SO101_ARM_MOTORS
            if required_maps["torque_enable"].get(name) not in (0, 1)
        }
        if invalid_torque:
            return "ERROR", f"SO101 Torque_Enable contains invalid values: {invalid_torque}"
        unexpected = value.get("unexpected_torque")
        if isinstance(unexpected, dict) and unexpected:
            return "ERROR", f"SO101 reports unexpected torque: {unexpected}"
        hold = value.get("hold")
        if isinstance(hold, dict) and str(hold.get("state", "")).upper() == "FAULT":
            return "ERROR", f"SO101 HOLD protection is faulted: {hold.get('reason', 'unknown')}"
        if value.get("healthy") is not True:
            return "ERROR", "SO101 guarded hardware server reports unhealthy"
        temperatures = [
            finite_or_none(required_maps["temperature_c"].get(name))
            for name in SO101_ARM_MOTORS
        ]
        voltages = [
            finite_or_none(required_maps["voltage_deci_v"].get(name))
            for name in SO101_ARM_MOTORS
        ]
        currents = [
            finite_or_none(required_maps["current_raw"].get(name))
            for name in SO101_ARM_MOTORS
        ]
        if any(sample is None for sample in temperatures + voltages + currents):
            return "UNKNOWN", "SO101 electrical register sample contains non-finite values"
        max_temperature = max(sample for sample in temperatures if sample is not None)
        min_voltage = min(sample for sample in voltages if sample is not None) / 10.0
        max_voltage = max(sample for sample in voltages if sample is not None) / 10.0
        enabled = sum(
            int(required_maps["torque_enable"].get(name, 0))
            for name in SO101_ARM_MOTORS
        )
        hold_state = hold.get("state", "UNKNOWN") if isinstance(hold, dict) else "UNKNOWN"
        return (
            "OK",
            f"SO101 direct registers healthy: {min_voltage:.1f}-{max_voltage:.1f}V, "
            f"max {max_temperature:.0f}C, torque {enabled}/6, HOLD={hold_state}",
        )

    @staticmethod
    def _so101_arm_gateway_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid SO101 Arm Dashboard state"
        hardware = value.get("hardware")
        if not isinstance(hardware, dict):
            return "UNKNOWN", "SO101 Arm Dashboard state has no hardware declaration"
        if hardware.get("real_hardware_enabled") is not True:
            return "IDLE", "SO101 real hardware is not enabled for this launch"
        health = str(value.get("health", "UNKNOWN"))
        feedback_age = finite_or_none(value.get("feedback_age_sec"))
        controller = value.get("controller") if isinstance(value.get("controller"), dict) else {}
        if health in {"HOLD_FAULT", "CODE_TORQUE_INCOMPLETE", "TELEOP_TORQUE_INCOMPLETE"}:
            return "ERROR", f"SO101 gateway reports {health}"
        if health == "STALE" or (feedback_age is not None and feedback_age > 1.0):
            age_text = "unknown" if feedback_age is None else f"{feedback_age:.2f}s"
            return "STALE", f"SO101 joint feedback is stale ({age_text})"
        if health in {"MODE_SYNCING", "STARTING"}:
            return (
                "UNKNOWN",
                f"SO101 gateway remains {health}; no usable real joint feedback/controller is available",
            )
        if value.get("control_mode_synced") is not True:
            return "UNKNOWN", "SO101 requested mode has not been confirmed by the hardware owner"
        if controller.get("action_ready") is not True:
            return "UNKNOWN", "SO101 trajectory controller action is not ready"
        if feedback_age is None:
            return "UNKNOWN", "SO101 gateway has no measured joint feedback age"
        if health in {
            "REAL2SIM_READY",
            "REAL_READY",
            "REAL_HOLDING",
            "RECOVERY_REQUIRED",
            "TELEOP_ACTIVE",
            "TELEOP_ALIGNING",
            "TELEOP_WAITING",
        }:
            return "OK", f"SO101 gateway is {health}; joint feedback age {feedback_age:.2f}s"
        return "UNKNOWN", f"SO101 gateway reports unclassified state {health}"

    @staticmethod
    def _so101_visual_joint_state_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid SO101 visual JointState"
        names = value.get("names")
        positions = value.get("positions")
        if not isinstance(names, list) or not isinstance(positions, list):
            return "ERROR", "SO101 visual JointState lacks names or positions"
        expected = {f"{name}_joint" for name in SO101_ARM_MOTORS}
        missing = sorted(expected - set(str(name) for name in names))
        if missing:
            return "ERROR", f"SO101 visual JointState is missing joints: {missing}"
        if len(names) != len(positions) or not all(
            finite_or_none(position) is not None for position in positions
        ):
            return "ERROR", "SO101 visual JointState has malformed or non-finite positions"
        return "OK", "Fresh six-axis SO101 JointState is available to the Arm TF publisher"

    def _so101_real_hardware_expected(self) -> bool:
        with self._signals_lock:
            signal = self.signals.get("so101_arm_gateway")
            value = signal.value if signal is not None else None
        return bool(
            isinstance(value, dict)
            and isinstance(value.get("hardware"), dict)
            and value["hardware"].get("real_hardware_enabled") is True
        )

    @staticmethod
    def _diagnostic_status_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid diagnostic status payload"
        level = diagnostic_level_number(value.get("level", 3))
        message = str(value.get("message", ""))
        if level == 0:
            return "OK", f"Driver diagnostic reports OK: {message or 'no message'}"
        if level == 1:
            return "DEGRADED", f"Driver diagnostic warning: {message or 'no message'}"
        if level == 2:
            return "ERROR", f"Driver diagnostic error: {message or 'no message'}"
        return "STALE", f"Driver diagnostic stale: {message or 'no message'}"

    @staticmethod
    def _livox_status_bytes(value: Any) -> list[int] | None:
        """Parse SDK2's 256-bit status_code without assigning undocumented bit meanings."""
        if isinstance(value, str):
            try:
                fields = [int(field, 16) for field in value.split()]
            except ValueError:
                return None
            return fields if len(fields) == 32 and all(0 <= field <= 0xFF for field in fields) else None
        if isinstance(value, list):
            try:
                fields = [int(field) for field in value]
            except (TypeError, ValueError):
                return None
            return fields if len(fields) == 32 and all(0 <= field <= 0xFF for field in fields) else None
        return None

    @staticmethod
    def _livox_device_health_validator(value: Any) -> tuple[str, str]:
        """Judge only native MID-360 fields whose semantics are documented by Livox."""
        if not isinstance(value, dict):
            return "DEGRADED", "Livox SDK2 diagnostic payload is invalid; point-cloud state is not inferred"
        devices = value.get("devices")
        if not isinstance(devices, list) or not devices:
            return "UNKNOWN", "No Livox SDK2 device state has been observed"

        warnings: list[str] = []
        errors: list[str] = []
        information: list[str] = []
        identities: list[str] = []
        for device in devices:
            if not isinstance(device, dict):
                return "DEGRADED", "Livox SDK2 diagnostic entry is invalid; point-cloud state is not inferred"
            identity = str(device.get("ip") or device.get("handle") or "unknown device")
            identities.append(identity)
            age = finite_or_none(device.get("push_age_sec"))
            if age is None:
                warnings.append(f"{identity}: SDK diagnostic push age unavailable")
            elif age > 2.5:
                warnings.append(f"{identity}: native SDK diagnostic state is {age:.2f}s old")
            if not bool(device.get("state_json_valid", False)):
                warnings.append(f"{identity}: SDK diagnostic state JSON is invalid")
                continue
            state = device.get("state")
            if not isinstance(state, dict):
                warnings.append(f"{identity}: SDK diagnostic state object is missing")
                continue

            work_state = state.get("cur_work_state")
            if work_state is None:
                warnings.append(f"{identity}: cur_work_state missing")
            else:
                try:
                    work_state = int(work_state)
                except (TypeError, ValueError):
                    warnings.append(f"{identity}: invalid cur_work_state diagnostic field")
                else:
                    # MID-360 protocol: 1=SAMPLING, 9=READY, 4=ERROR.
                    if work_state == 4:
                        errors.append(f"{identity}: work state ERROR (4)")
                    elif work_state not in {1, 9}:
                        warnings.append(f"{identity}: work state {work_state} is not SAMPLING/READY")

            diag = state.get("lidar_diag_status")
            if diag is None:
                warnings.append(f"{identity}: lidar_diag_status missing")
            else:
                try:
                    diag_value = int(diag)
                except (TypeError, ValueError):
                    warnings.append(f"{identity}: invalid lidar_diag_status diagnostic field")
                else:
                    levels = [(diag_value >> shift) & 0xF for shift in (0, 4, 8, 12)]
                    if any(level >= 2 for level in levels):
                        errors.append(
                            f"{identity}: diagnostic module level(s) {levels} include error/safety_error"
                        )
                    elif any(level == 1 for level in levels):
                        warnings.append(f"{identity}: diagnostic module warning level(s) {levels}")
                    elif any(level != 0 for level in levels):
                        warnings.append(f"{identity}: undocumented diagnostic level(s) {levels}")

            hms = state.get("hms_code")
            if isinstance(hms, list):
                nonzero_hms = []
                for code in hms:
                    try:
                        parsed = int(code)
                    except (TypeError, ValueError):
                        warnings.append(f"{identity}: invalid HMS diagnostic code")
                        nonzero_hms = []
                        break
                    if parsed != 0:
                        nonzero_hms.append(parsed)
                if nonzero_hms:
                    for code in nonzero_hms:
                        level = code & 0xFF
                        detail = f"{identity}: HMS 0x{code:08X} level={level}"
                        if level in {3, 4}:
                            errors.append(detail)
                        elif level == 2:
                            warnings.append(detail)
                        elif level == 1:
                            information.append(detail)
                        else:
                            warnings.append(f"{detail} (undocumented level)")
            else:
                warnings.append(f"{identity}: hms_code missing")

            raw_status = state.get("status_code")
            if raw_status is None:
                # MID-360 protocol 1.4.11 and the installed SDK2 1.3.1 real
                # frame expose lidar_diag_status + hms_code but do not include
                # key 0x800D.  Absence is a capability fact, not a fault.
                information.append(f"{identity}: status_code not exposed")
            else:
                status_bytes = HealthMonitor._livox_status_bytes(raw_status)
                if status_bytes is None:
                    warnings.append(f"{identity}: exposed status_code is malformed")
                elif any(status_bytes):
                    warnings.append(f"{identity}: native 256-bit status_code is nonzero")

        if errors:
            return "ERROR", "; ".join(errors)
        if warnings:
            return "DEGRADED", "; ".join(warnings)
        summary = f"Fresh native Livox SDK2 state is clear for {', '.join(identities)}"
        if information:
            summary += "; " + "; ".join(information)
        return "OK", summary

    @staticmethod
    def _finite_dict_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Unexpected payload"
        numeric = [item for item in value.values() if isinstance(item, (int, float))]
        if not numeric or any(not math.isfinite(float(item)) for item in numeric):
            return "ERROR", "Feedback contains no finite numeric value"
        return "OBSERVED", "Fresh finite feedback received"

    @staticmethod
    def _safety_validator(value: Any) -> tuple[str, str]:
        status = str(value).strip().upper()
        if status == "ESTOP":
            return "BLOCKED", "Software safety node reports ESTOP"
        if status in {"OK", "SAFE"}:
            return "OK", f"Software safety gate reports {status}"
        if status in {"WATCHDOG", "WATCHDOG_OK"}:
            return "OK", "Software safety watchdog is intentionally holding zero output while idle"
        if status == "TOUCH_LOCK":
            return "OK", "Touch Mode intentionally locks all mobile-base velocity output"
        if status == "MANUAL_STOP":
            return "DEGRADED", "Software safety gate reports MANUAL_STOP; output should be zero"
        return "UNKNOWN", f"Unrecognized safety status: {status or 'empty'}"

    @staticmethod
    def _joy_source_validator(value: Any) -> tuple[str, str]:
        if not isinstance(value, dict):
            return "ERROR", "Invalid Joy source arbiter status"
        source = str(value.get("source", "UNKNOWN")).upper()
        acquired = bool(value.get("dashboard_acquired", False))
        reason = str(value.get("reason", ""))
        if source == "PHYSICAL" and not acquired:
            return "OK", f"Canonical /joy is owned by PHYSICAL; {reason}"
        if source == "DASHBOARD" and acquired:
            return "OK", f"Canonical /joy is explicitly owned by DASHBOARD; {reason}"
        return "ERROR", f"Inconsistent Joy source state: source={source}, acquired={acquired}"

    @staticmethod
    def _mode_validator(value: Any) -> tuple[str, str]:
        mode = str(value).strip().upper()
        if mode in {"MANUAL", "AUTO"}:
            return "OK", f"Mode authority reports {mode}"
        return "UNKNOWN", f"Unrecognized robot mode: {mode or 'empty'}"

    def _gicp_validator(self, value: Any) -> tuple[str, str]:
        score = finite_or_none(value.get("inlier_rmse") if isinstance(value, dict) else None)
        threshold = float(self.get_parameter("gicp_score_threshold").value)
        if score is None:
            return "ERROR", "GICP score is not finite"
        if score > threshold:
            return "DEGRADED", f"GICP attempt rejected by configured threshold ({score:.4f} > {threshold:.4f})"
        return "OK", f"GICP self-score passed configured acceptance threshold ({score:.4f} <= {threshold:.4f})"

    @staticmethod
    def _lifecycle_validator(value: Any) -> tuple[str, str]:
        label = str(value.get("label", "")) if isinstance(value, dict) else ""
        if label == "active":
            return "OK", "Lifecycle service reports active"
        if label:
            return "DEGRADED", f"Lifecycle service reports {label}"
        return "UNKNOWN", "Lifecycle response has no state label"

    def _publisher_item(
        self,
        topic: str,
        *,
        critical: bool,
        allowed_nodes: set[str] | None = None,
        require_single_endpoint: bool = True,
    ) -> dict[str, Any]:
        try:
            publishers = self.get_publishers_info_by_topic(topic)
            identities = [
                {
                    "node_name": info.node_name,
                    "node_namespace": info.node_namespace,
                    "topic_type": info.topic_type,
                }
                for info in publishers
            ]
        except Exception as exc:
            return self._fixed_item(
                f"publisher:{topic}",
                f"Publisher authority {topic}",
                "UNKNOWN",
                f"ROS graph query failed: {exc!r}",
                "ROS graph",
                "GRAPH_QUERY",
                critical=critical,
            )
        observed_nodes = {
            f"{identity['node_namespace'].rstrip('/')}/{identity['node_name']}".replace("//", "/")
            for identity in identities
        }
        if len(identities) == 0:
            status, summary = "UNKNOWN", "No publisher observed"
        elif allowed_nodes is not None:
            unexpected = sorted(observed_nodes - allowed_nodes)
            if unexpected:
                status = "ERROR"
                summary = f"Unexpected command publisher node(s): {', '.join(unexpected)}"
            else:
                status = "OK"
                summary = (
                    f"{len(identities)} publisher endpoint(s) belong to approved source node(s): "
                    f"{', '.join(sorted(observed_nodes))}"
                )
        elif require_single_endpoint and len(identities) == 1:
            status, summary = "OK", "Exactly one publisher endpoint observed"
        else:
            status, summary = "ERROR", f"{len(identities)} publishers observed; command authority is ambiguous"
        return self._fixed_item(
            f"publisher:{topic}",
            f"Publisher authority {topic}",
            status,
            summary,
            "ROS graph endpoint info",
            "GRAPH_QUERY",
            critical=critical,
            standard=(
                "All publisher endpoints must belong to the explicit source-node allowlist"
                if allowed_nodes is not None
                else "Exactly one final publisher endpoint is required"
            ),
            value=identities,
        )

    def _host_item(self) -> dict[str, Any]:
        if not self._host_metrics:
            return self._fixed_item(
                "host_resources",
                "Host resources",
                "UNKNOWN",
                "No host metrics collected yet",
                "/proc + filesystem",
                "DIRECT_HOST_COUNTER",
            )
        sensors = self._host_metrics.get("hwmon_temperatures") or []
        at_crit = [
            sensor
            for sensor in sensors
            if sensor.get("crit_c") is not None and sensor["input_c"] >= sensor["crit_c"]
        ]
        at_max = [
            sensor
            for sensor in sensors
            if sensor.get("max_c") is not None and sensor["input_c"] >= sensor["max_c"]
        ]
        if at_crit:
            status = "ERROR"
            summary = "A hardware temperature input reached its sysfs critical limit"
        elif at_max:
            status = "DEGRADED"
            summary = "A hardware temperature input reached its sysfs maximum limit"
        else:
            status = "OBSERVED"
            summary = "Host counters are measured; no workload baseline is inferred"
        return self._fixed_item(
            "host_resources",
            "Host resources",
            status,
            summary,
            "/proc/stat, /proc/loadavg, /proc/meminfo, filesystem, /sys/class/hwmon",
            "DIRECT_HOST_COUNTER",
            critical=True,
            standard="Only hardware-exported temp*_max/temp*_crit are evaluated; CPU/load/disk remain display-only",
            value=self._host_metrics,
        )

    def _host_clock_item(self) -> dict[str, Any]:
        if not self._host_metrics:
            return self._fixed_item(
                "host_clock_sync",
                "Host NTP synchronization",
                "UNKNOWN",
                "Host metrics have not been collected",
                "/run/systemd/timesync/synchronized",
                "DIRECT_HOST_COUNTER",
            )
        synchronized = bool(self._host_metrics.get("host_ntp_synchronized"))
        return self._fixed_item(
            "host_clock_sync",
            "Host NTP synchronization",
            "OK" if synchronized else "UNKNOWN",
            (
                "systemd-timesyncd synchronization marker exists"
                if synchronized
                else "systemd-timesyncd synchronization marker is absent"
            ),
            "/run/systemd/timesync/synchronized",
            "DIRECT_HOST_COUNTER",
            standard="Direct systemd-timesyncd synchronization marker; does not prove sensor hardware-clock sync",
            value={"synchronized": synchronized},
        )

    def _perception_runtime_item(self) -> dict[str, Any]:
        with self._signals_lock:
            signal = self.signals.get("mission_state")
            mission = signal.value if signal is not None else None
        if not isinstance(mission, dict):
            return self._fixed_item(
                "perception_runtime",
                "Latest perception execution",
                "IDLE",
                "No mission report contains a perception execution yet",
                "/agent/mission_state",
                "STRUCTURED_REPORT",
                standard="Event-driven evidence; IDLE is not proof that inference would succeed",
            )
        results = mission.get("skill_results")
        if not isinstance(results, list):
            results = []
        perception_names = {"capture_observation", "detect_known_object"}
        latest = next(
            (
                result
                for result in reversed(results)
                if isinstance(result, dict)
                and str(result.get("skill_name", result.get("name", ""))) in perception_names
            ),
            None,
        )
        if latest is None:
            return self._fixed_item(
                "perception_runtime",
                "Latest perception execution",
                "IDLE",
                "Latest mission has no reported perception step",
                "/agent/mission_state",
                "STRUCTURED_REPORT",
                standard="Event-driven evidence; IDLE is not proof that inference would succeed",
                value={"mission_status": mission.get("status")},
            )
        success = latest.get("success")
        status = "REPORTED" if success is True else ("ERROR" if success is False else "REPORTED")
        summary = (
            "Latest structured perception result reports success"
            if success is True
            else (
                "Latest structured perception result reports failure"
                if success is False
                else "Latest structured perception result has no boolean success field"
            )
        )
        return self._fixed_item(
            "perception_runtime",
            "Latest perception execution",
            status,
            summary,
            "/agent/mission_state skill_results",
            "STRUCTURED_REPORT",
            standard="Latest event report only; no continuous inference-runtime heartbeat exists",
            value=latest,
        )

    def _domain(self, domain_id: str, title: str, items: list[dict[str, Any]]) -> dict[str, Any]:
        status, reason = aggregate_status(items)
        summary = "All required V1 evidence is available"
        if reason is not None:
            summary = f"{reason['label']}: {reason['summary']}"
        return {
            "id": domain_id,
            "title": title,
            "status": status,
            "summary": summary,
            "reason_item_id": reason.get("id") if reason else None,
            "items": items,
        }

    def _build_domains(self) -> list[dict[str, Any]]:
        # Arm evidence is optional only when the integrated launch explicitly
        # disables real Arm hardware. Once the Arm gateway declares that real
        # hardware is enabled, loss of its direct register stream and TF input
        # must affect the seven-domain summary instead of remaining a gray,
        # non-critical detail.
        arm_expected = self._so101_real_hardware_expected()
        arm_gateway_item = self._signal_item(
            "so101_arm_gateway",
            "SO101 Arm gateway and real2sim delivery",
            stale_sec=2.5,
            standard=(
                "Direct /arm/dashboard_state gate: real hardware requires a synchronized mode, "
                "ready controller action and fresh measured joint feedback"
            ),
            item_id="so101_arm_gateway_delivery",
            critical=arm_expected,
            validator=self._so101_arm_gateway_validator,
        )
        arm_hardware_item = self._signal_item(
            "so101_arm_health",
            "SO101 arm voltage / current / temperature / torque",
            stale_sec=2.5,
            standard="1Hz direct Feetech register summary from the unique guarded follower bus owner; current remains in raw register units because no verified ampere conversion is configured",
            item_id="so101_arm_hardware_status",
            critical=arm_expected,
            validator=self._so101_arm_hardware_validator,
        )
        arm_visual_item = self._signal_item(
            "so101_arm_visual_joints",
            "SO101 real2sim joint and TF input",
            stale_sec=1.0,
            standard=(
                "Derived /arm/visual_joint_states must contain all six finite follower joints; "
                "the Arm robot_state_publisher cannot form its dynamic TF chain without this input"
            ),
            item_id="so101_arm_visual_joint_states",
            critical=arm_expected,
            validator=self._so101_visual_joint_state_validator,
        )
        # MODE_SYNCING is valid briefly while the serial handshake completes.
        # If the gateway has declared real hardware for at least five seconds,
        # continuing without either direct status or visual joints is a fault,
        # not open-ended UNKNOWN evidence.
        gateway_span = finite_or_none(arm_gateway_item.get("observation_span_sec")) or 0.0
        if arm_expected and gateway_span >= 5.0:
            if arm_gateway_item.get("status") == "UNKNOWN":
                arm_gateway_item["status"] = "ERROR"
                arm_gateway_item["summary"] = (
                    f"{arm_gateway_item['summary']}; persisted for at least {gateway_span:.1f}s"
                )
            if arm_hardware_item.get("status") == "UNKNOWN":
                arm_hardware_item["status"] = "ERROR"
                arm_hardware_item["summary"] = (
                    "Real SO101 hardware is enabled, but /arm/hardware_status has not arrived "
                    "after the startup grace period; check follower 12 V power, power switch, "
                    "servo-bus cable and guarded hardware server"
                )
            if arm_visual_item.get("status") == "UNKNOWN":
                arm_visual_item["status"] = "ERROR"
                arm_visual_item["summary"] = (
                    "Real SO101 hardware is enabled, but no complete six-axis TF input has arrived "
                    "after the startup grace period; real2sim and dynamic Arm TF are unavailable"
                )
        h1 = self._domain(
            "H1",
            "Physical, Device & Driver",
            [
                self._signal_item(
                    "drive_health",
                    "Physical E-stop circuit",
                    stale_sec=1.5,
                    item_id="physical_estop",
                    critical=True,
                    standard="Installed wiring qualified on 2026-08-19 against the physical button: all BLVD DIN3=1 released, all DIN3=0 pressed/locked; mixed inputs are an error",
                    validator=self._physical_estop_validator,
                    value_selector=self._pick_fields("direct_io", "last_direct_io_age_sec"),
                ),
                self._fixed_item(
                    "drive_hardware_identity",
                    "Drive hardware identity",
                    "VERIFIED",
                    "Nameplate photos identify BLVD-KRD / GFS5G30FR assemblies at ID 1 back, ID 2 front-left and ID 3 front-right",
                    "2026-08-19 real-robot nameplate photos + hardware/blv_r_drive_motors.md",
                    "PHYSICAL_INSPECTION_RECORD",
                    standard="Physically verified from six position-labelled real-robot nameplate photos; this proves installed identity and location, while live communication state is evaluated separately from /drive/health_summary",
                    value={
                        "installed_count": 3,
                        "nameplate_records": [
                            {
                                "modbus_id": 1,
                                "wheel_position": "back",
                                "driver_trace_code": "RX5 F178611",
                                "gear_head_trace_code": "RX9 A127705",
                                "photos": ["hardware/motor-image/1-back.jpeg", "hardware/motor-image/1-back.jpg"],
                            },
                            {
                                "modbus_id": 2,
                                "wheel_position": "front_left",
                                "driver_trace_code": "RX5 F178612",
                                "gear_head_trace_code": "RX9 A127704",
                                "photos": ["hardware/motor-image/2-front-left.jpeg", "hardware/motor-image/2-front-left.jpg"],
                            },
                            {
                                "modbus_id": 3,
                                "wheel_position": "front_right",
                                "driver_trace_code": "RX5 F178620",
                                "gear_head_trace_code": "RX9 A127706",
                                "photos": ["hardware/motor-image/3-front-right.jpeg", "hardware/motor-image/3-front-right.jpg"],
                            },
                        ],
                        "driver": {
                            "manufacturer": "Oriental Motor Co., Ltd.",
                            "model": "BLVD-KRD",
                            "rated_input": "24-48 VDC, 10.5 A",
                            "protection": "IP20 (self-declaration)",
                            "manufactured": "2024-10",
                            "country": "Made in Japan",
                        },
                        "gear_head": {
                            "manufacturer": "Oriental Motor Co., Ltd.",
                            "model": "GFS5G30FR",
                            "manufactured": "2024-10",
                            "country": "Made in Japan",
                        },
                        "motor": {
                            "model": "BLMR5100K-GFV-B",
                            "evidence": "deployment record; not visible in supplied photos",
                        },
                        "runtime_configuration": {
                            "id_to_wheel_position": {"1": "back", "2": "front_left", "3": "front_right"},
                            "global_id": 10,
                            "baudrate": 230400,
                            "interface": "/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FT7E9M3C-if00-port0",
                        },
                    },
                ),
                self._signal_item(
                    "drive_health",
                    "Drive bus voltage / current",
                    stale_sec=1.5,
                    item_id="drive_bus_voltage",
                    standard="Direct HP-5142-4E ID-share monitors: NET-ID 155 main current (0.001 A), 163 inverter voltage (0.1 V), 164 main supply voltage (0.1 V); 24-48 VDC / 10.5 A is a nameplate rating, not an automatic health threshold",
                    validator=self._blvd_electrical_validator,
                    value_selector=self._pick_fields("electrical_status", "last_electrical_age_sec"),
                ),
                self._signal_item(
                    "drive_health",
                    "BLVD per-driver alarm status",
                    stale_sec=1.5,
                    item_id="drive_alarm_status",
                    standard="Direct HP-5142-4E registers: Present alarm NET-ID 64 and Alarm history 1 NET-ID 65, sampled per physical driver without interpreting LED blink count",
                    critical=True,
                    validator=self._blvd_alarm_validator,
                    value_selector=self._pick_fields(
                        "alarm_status",
                        "alarm_event_count",
                        "last_alarm_event_age_sec",
                        "session_started_at",
                    ),
                ),
                self._signal_item(
                    "drive_health",
                    "Drive transaction and callback summary",
                    stale_sec=1.5,
                    standard=(
                        "2Hz summary generated inside drive_motor from actual /om_response0 and /om_state0 "
                        "callbacks. Incomplete startup evidence is UNKNOWN; an explicit driver error is "
                        "ERROR/DEGRADED and verified post-error evidence may become RECOVERED"
                    ),
                    critical=True,
                    validator=self._modbus_validator,
                    value_selector=self._pick_fields(
                        "state",
                        "state_snapshot_age_sec",
                        "response_count",
                        "runtime_query_count",
                        "response_rate_hz_5s",
                        "read_response_rate_hz_5s",
                        "last_response_age_sec",
                        "last_read_response_age_sec",
                        "completion_deadline_miss_count",
                        "completion_deadline_misses_5s",
                        "last_completion_deadline_miss_age_sec",
                        "last_completion_deadline_miss_query_kind",
                        "last_completion_deadline_miss_elapsed_sec",
                        "driver_error_event_count",
                        "driver_no_response_event_count",
                        "driver_exception_event_count",
                        "last_driver_error_age_sec",
                        "session_started_at",
                    ),
                ),
                self._signal_item(
                    "drive_health",
                    "BLVD Direct I/O raw DIN3",
                    stale_sec=1.5,
                    standard="Official BLV-R function manual HP-5142: ID-share NET-ID 106 Direct I/O, DIN3 bit 3; polarity must be qualified on the installed wiring",
                    item_id="drive_din3_raw",
                    validator=self._blvd_direct_io_validator,
                    value_selector=self._pick_fields("direct_io", "last_direct_io_age_sec"),
                ),
                self._signal_item(
                    "real_robot_state",
                    "Recent drive recovery",
                    stale_sec=3.0,
                    item_id="drive_recovery_event",
                    standard="Direct result of the guarded Dashboard restart workflow; recovery success does not erase the original unidentified fault",
                    validator=self._drive_restart_validator,
                    value_selector=lambda value: (
                        {"drive_restart": value.get("drive_restart")}
                        if isinstance(value, dict)
                        else value
                    ),
                ),
                self._signal_item(
                    "drive_feedback",
                    "Three-wheel velocity feedback",
                    stale_sec=1.0,
                    standard="V1 receiver threshold 1.00s; real recovery run on 2026-08-20 observed 4-5Hz with gaps up to 0.804s after 5Hz monitor-side sampling",
                    critical=True,
                    validator=self._finite_dict_validator,
                ),
                self._signal_item(
                    "steer_feedback",
                    "Three-axis steering feedback",
                    stale_sec=1.0,
                    standard="V1 receiver threshold 1.00s; freshness only, while tracking-error limits still require a real baseline",
                    critical=True,
                    validator=self._finite_dict_validator,
                ),
                self._signal_item(
                    "dynamixel_health",
                    "DYNAMIXEL direct read status",
                    stale_sec=1.0,
                    standard="2Hz summary of the existing position-read transaction result and device error byte",
                    critical=True,
                    validator=self._dynamixel_validator,
                ),
                self._signal_item(
                    "dynamixel_health",
                    "DYNAMIXEL model identity",
                    stale_sec=1.0,
                    standard="Protocol 2.0 ping returns the model number reported directly by each configured device",
                    item_id="dynamixel_identity",
                    validator=self._dynamixel_identity_validator,
                ),
                self._signal_item(
                    "camera_pan",
                    "Camera pan feedback",
                    stale_sec=1.5,
                    standard="Existing real_robot_monitor freshness threshold",
                    validator=self._finite_dict_validator,
                ),
                self._signal_item(
                    "camera_tilt",
                    "Camera tilt feedback",
                    stale_sec=1.5,
                    standard="Existing real_robot_monitor freshness threshold",
                    validator=self._finite_dict_validator,
                ),
                self._signal_item(
                    "lift_health",
                    "Lift estimated position",
                    stale_sec=2.0,
                    item_id="lift_position",
                    standard="1Hz lift_serial_node summary; USB VID/PID and Arduino command ACK verify delivery, while position remains jog_speed × command time because ABZO/limit/home/alarm feedback is not returned to ROS",
                    validator=self._lift_health_validator,
                ),
                self._signal_item(
                    "dynamixel_health",
                    "DYNAMIXEL voltage / current / temperature registers",
                    stale_sec=2.5,
                    standard="ROBOTIS XH540-W270 control table: Hardware Error Status(70), Present Current(126), Present Input Voltage(144), Present Temperature(146)",
                    item_id="dynamixel_hardware_status",
                    validator=self._dynamixel_hardware_validator,
                ),
                arm_hardware_item,
            ],
        )

        h2 = self._domain(
            "H2",
            "ROS & Platform",
            [
                self._publisher_item(
                    "/cmd_vel_raw",
                    critical=True,
                    allowed_nodes={"/agent_node", "/nav_mode_switch_node"},
                    require_single_endpoint=False,
                ),
                self._publisher_item("/cmd_vel", critical=True),
                self._publisher_item("/joy", critical=True),
                self._signal_item(
                    "joy_source",
                    "Manual control source arbitration",
                    stale_sec=1.5,
                    standard="joy_source_arbiter_node is the only canonical /joy publisher; PHYSICAL is default and Dashboard requires an explicit lease",
                    item_id="joy_source_authority",
                    critical=True,
                    validator=self._joy_source_validator,
                ),
                self._signal_item(
                    "robot_mode",
                    "ROS delivery /robot_mode",
                    stale_sec=3.0,
                    standard="Publisher timer is 2s; V1 max receive age is 3s",
                    item_id="delivery_robot_mode",
                    critical=True,
                ),
                self._signal_item(
                    "safety",
                    "ROS delivery /safety_status",
                    stale_sec=2.0,
                    standard="Publisher timer is 1s; V1 max receive age is 2s",
                    item_id="delivery_safety",
                    critical=True,
                ),
                self._signal_item(
                    "drive_feedback",
                    "ROS delivery /drive_odom",
                    stale_sec=1.0,
                    standard="Receiver-measured freshness at health_monitor; 1.00s threshold includes the observed 0.804s gap and 5Hz monitor-side sampling jitter",
                    item_id="delivery_drive_feedback",
                    critical=True,
                ),
                arm_gateway_item,
                arm_visual_item,
                self._signal_item(
                    "camera_color_diagnostics",
                    "RealSense color stream diagnostic delivery",
                    stale_sec=2.5,
                    standard="RealSense native frequency diagnostic published every 1s",
                    item_id="delivery_camera_color_diagnostics",
                    validator=self._diagnostic_status_validator,
                ),
                self._host_item(),
                self._host_clock_item(),
                self._fixed_item(
                    "sensor_clock_sync",
                    "Sensor hardware-clock synchronization",
                    "NOT_INSTRUMENTED",
                    "Host NTP is observable, but no PTP/device clock-lock status is exposed by current sensor drivers",
                    "No source available",
                    "NOT_INSTRUMENTED",
                    standard="Header age is not treated as absolute cross-device latency without direct device clock status",
                ),
            ],
        )

        h3 = self._domain(
            "H3",
            "Command & Motion",
            [
                self._signal_item(
                    "robot_mode",
                    "Motion authority mode",
                    stale_sec=3.0,
                    standard="Only MANUAL and AUTO are recognized",
                    critical=True,
                    validator=self._mode_validator,
                ),
                self._signal_item(
                    "safety",
                    "Software safety gate",
                    stale_sec=2.0,
                    standard="Direct /safety_status from cmd_vel_safety_node; not the physical E-stop circuit",
                    critical=True,
                    validator=self._safety_validator,
                ),
                self._signal_item(
                    "cmd_vel_raw",
                    "Command before safety gate",
                    stale_sec=0.5,
                    standard="Receive freshness and current value only",
                    critical=True,
                ),
                self._signal_item(
                    "cmd_vel",
                    "Command after safety gate",
                    stale_sec=0.5,
                    standard="Dashboard receiver threshold is 0.50s after 5Hz monitor-side sampling; the independent motor command watchdog remains 0.08s during motion",
                    critical=True,
                ),
                self._signal_item(
                    "drive_feedback",
                    "Observed wheel motion",
                    stale_sec=1.0,
                    standard="Direct driver feedback values with a 1.00s receiver threshold; no command-tracking tolerance is inferred in V1",
                    critical=True,
                ),
                self._signal_item(
                    "steer_feedback",
                    "Observed steering angles",
                    stale_sec=1.0,
                    standard="Direct steering feedback values with a 1.00s receiver threshold; no tracking tolerance is inferred in V1",
                    critical=True,
                ),
                self._signal_item(
                    "motion_policy",
                    "Motion policy report",
                    stale_sec=None,
                    standard="Reported by nav_mode_switch_node when policy is evaluated",
                    fresh_status="REPORTED",
                    fresh_summary="Latest policy decision reported by nav_mode_switch_node",
                    unobserved_status="IDLE",
                    unobserved_summary="No motion policy event is expected while no policy is being evaluated",
                ),
            ],
        )

        h4 = self._domain(
            "H4",
            "Sensor & Perception",
            [
                self._signal_item(
                    "camera_color_diagnostics",
                    "RealSense color frequency diagnostic",
                    stale_sec=2.5,
                    standard="Native RealSense FrequencyDiagnostics for configured color stream",
                    critical=True,
                    validator=self._diagnostic_status_validator,
                ),
                self._signal_item(
                    "camera_depth_diagnostics",
                    "RealSense depth frequency diagnostic",
                    stale_sec=2.5,
                    standard="Native RealSense FrequencyDiagnostics for configured depth stream",
                    critical=True,
                    validator=self._diagnostic_status_validator,
                ),
                self._signal_item(
                    "camera_temperatures",
                    "RealSense device temperatures",
                    stale_sec=2.5,
                    standard="Direct ASIC/projector temperature values from the RealSense diagnostic updater",
                    validator=self._diagnostic_status_validator,
                ),
                self._signal_item(
                    "lidar_device_health",
                    "Livox SDK diagnostic channel / device error bits",
                    item_id="lidar_device_status",
                    stale_sec=2.5,
                    standard=(
                        "Native Livox SDK2 diagnostic state is independent of the point-cloud stream. "
                        "A missing/stale/malformed diagnostic callback degrades monitoring confidence but "
                        "does not prove point-cloud loss. MID-360 cur_work_state must be "
                        "SAMPLING/READY and four lidar_diag_status module levels must be 0; HMS info is "
                        "retained while warning/error/fatal levels degrade or fail health. status_code is "
                        "optional for MID-360/SDK2 1.3.1, but is checked when exposed. time_sync_type=0 is "
                        "informational because "
                        "this robot has no configured external clock source"
                    ),
                    critical=True,
                    stale_status="DEGRADED",
                    validator=self._livox_device_health_validator,
                    unobserved_summary="No /livox/device_health message has reached this monitor",
                ),
                self._perception_runtime_item(),
            ],
        )

        score_threshold = float(self.get_parameter("gicp_score_threshold").value)
        h5 = self._domain(
            "H5",
            "Localization",
            [
                self._signal_item(
                    "gicp_score",
                    "GICP attempt self-score",
                    stale_sec=3.0,
                    standard=f"gicp_localizer.yaml fitness_score_threshold={score_threshold}",
                    critical=True,
                    validator=self._gicp_validator,
                ),
                self._signal_item(
                    "gicp_pose",
                    "Last accepted GICP pose",
                    stale_sec=3.0,
                    standard="Pose is only published after an accepted correction; receive age is acceptance age",
                    critical=True,
                ),
                self._signal_item(
                    "fast_lio_odom",
                    "FAST-LIO odometry (optional mode)",
                    stale_sec=1.0,
                    standard="Optional: absent in default GICP-only launch",
                ),
                self._fixed_item(
                    "localization_independent_validation",
                    "Independent localization consistency",
                    "NOT_INSTRUMENTED",
                    "V1 does not yet compare wheel odometry, FAST-LIO delta and GICP corrections",
                    "No combined monitor available",
                    "NOT_INSTRUMENTED",
                    standard="A good GICP self-score alone does not prove absolute pose correctness",
                ),
            ],
        )

        lifecycle_items = [
            self._signal_item(
                f"lifecycle_{key}",
                f"Lifecycle {key}",
                stale_sec=3.0,
                standard=f"Direct GetState response from {service}",
                critical=True,
                validator=self._lifecycle_validator,
            )
            for key, (service, _client) in self._lifecycle_clients.items()
        ]
        h6 = self._domain(
            "H6",
            "Navigation",
            lifecycle_items
            + [
                self._fixed_item(
                    "local_costmap_output_freshness",
                    "Local costmap output freshness",
                    "NOT_INSTRUMENTED",
                    "The high-bandwidth OccupancyGrid subscription was removed; lifecycle active does not prove output updates",
                    "No low-bandwidth source available",
                    "NOT_INSTRUMENTED",
                    standard="Add a source-side heartbeat/counter instead of resubscribing the full costmap",
                ),
                self._signal_item(
                    "global_plan",
                    "Latest global plan",
                    stale_sec=None,
                    standard="Event-driven evidence; not critical while navigation is idle",
                    unobserved_status="IDLE",
                    unobserved_summary="No global plan is expected while navigation is idle",
                ),
                self._signal_item(
                    "navigation_progress",
                    "Agent navigation progress",
                    stale_sec=None,
                    standard="Structured report from active goto_pose; not direct Nav2 action feedback",
                    fresh_status="REPORTED",
                    fresh_summary="Latest navigation progress report received",
                    unobserved_status="IDLE",
                    unobserved_summary="No navigation progress is expected before an active goto_pose",
                ),
                self._signal_item(
                    "navigation_route",
                    "Agent navigation route state",
                    stale_sec=None,
                    standard="Event-driven route report; not critical while navigation is idle",
                    fresh_status="REPORTED",
                    fresh_summary="Latest navigation route report received",
                    unobserved_status="IDLE",
                    unobserved_summary="No navigation route event is expected while navigation is idle",
                ),
            ],
        )

        h7 = self._domain(
            "H7",
            "Agent & Mission",
            [
                self._signal_item(
                    "agent_heartbeat",
                    "Agent heartbeat",
                    stale_sec=3.5,
                    standard="agent_node publishes /agent/health_state every 2s",
                    critical=True,
                    fresh_status="OBSERVED",
                    fresh_summary="Fresh Agent heartbeat report received",
                ),
                self._signal_item(
                    "agent_status",
                    "Latest Agent status",
                    stale_sec=None,
                    standard="Event-driven text; age does not indicate failure",
                    fresh_status="REPORTED",
                    fresh_summary="Latest event-driven Agent status",
                    unobserved_status="IDLE",
                    unobserved_summary="No Agent status event has been emitted yet",
                ),
                self._signal_item(
                    "mission_state",
                    "Latest mission state",
                    stale_sec=None,
                    standard="Event-driven mission state; not required before the first mission",
                    fresh_status="REPORTED",
                    fresh_summary="Latest structured mission state",
                    unobserved_status="IDLE",
                    unobserved_summary="No mission state is expected before the first mission",
                ),
            ],
        )
        return [h1, h2, h3, h4, h5, h6, h7]

    @staticmethod
    def _summary_group(
        group_id: str,
        title: str,
        domains: list[dict[str, Any]],
    ) -> dict[str, Any]:
        status, reason = combine_status(
            [
                (
                    str(domain.get("status", "UNKNOWN")),
                    next(
                        (item for item in domain.get("items", []) if item.get("id") == domain.get("reason_item_id")),
                        None,
                    ),
                )
                for domain in domains
            ]
        )
        return {
            "id": group_id,
            "title": title,
            "status": status,
            "summary": reason.get("summary") if reason else "Required V1 evidence is available",
            "reason_domain_id": next(
                (
                    domain.get("id")
                    for domain in domains
                    if reason is not None and domain.get("reason_item_id") == reason.get("id")
                ),
                None,
            ),
            "reason_item_id": reason.get("id") if reason else None,
        }

    @staticmethod
    def _find_item(domains: list[dict[str, Any]], item_id: str) -> dict[str, Any] | None:
        for domain in domains:
            for item in domain.get("items", []):
                if item.get("id") == item_id:
                    return item
        return None

    def _capability(
        self,
        capability_id: str,
        title: str,
        domains: list[dict[str, Any]],
        dependencies: list[str],
    ) -> dict[str, Any]:
        items = [self._find_item(domains, dependency) for dependency in dependencies]
        present = [item for item in items if item is not None]
        missing_ids = [dependency for dependency, item in zip(dependencies, items) if item is None]
        reason = next((item for item in present if item.get("status") == "BLOCKED"), None)
        status = "BLOCKED" if reason else "AVAILABLE"
        if reason is None:
            reason = next((item for item in present if item.get("status") in {"ERROR", "STALE"}), None)
            if reason is not None:
                status = "BLOCKED"
        if reason is None:
            reason = next(
                (
                    item
                    for item in present
                    if item.get("status") in UNCERTAIN_ITEM_STATES | NON_PROVING_ITEM_STATES
                ),
                None,
            )
            if reason is not None or missing_ids:
                status = "UNKNOWN"
        if reason is None and status == "AVAILABLE":
            reason = next((item for item in present if item.get("status") in DEGRADED_ITEM_STATES), None)
            if reason is not None:
                status = "SUPERVISED_ONLY"
        return {
            "id": capability_id,
            "title": title,
            "status": status,
            "summary": reason.get("summary") if reason else "All V1 dependencies are available",
            "blocked_by": reason.get("id") if reason else (missing_ids[0] if missing_ids else None),
            "dependencies": dependencies,
        }

    def _publish(self) -> None:
        publish_started = time.monotonic()
        domains = self._build_domains()
        by_id = {domain["id"]: domain for domain in domains}
        summaries = [
            self._summary_group("A1", "Device & Data Acquisition", [by_id["H1"], by_id["H4"]]),
            self._summary_group("A2", "ROS & Platform Delivery", [by_id["H2"]]),
            self._summary_group(
                "A3", "Functional & Behavior", [by_id[key] for key in ("H3", "H4", "H5", "H6", "H7")]
            ),
        ]
        capabilities = [
            self._capability(
                "manual_drive",
                "Manual drive",
                domains,
                [
                    "physical_estop",
                    "drive_health",
                    "drive_feedback",
                    "dynamixel_health",
                    "steer_feedback",
                    "safety",
                    "publisher:/cmd_vel",
                ],
            ),
            self._capability(
                "auto_navigation",
                "Auto navigation",
                domains,
                [
                    "physical_estop",
                    "drive_health",
                    "drive_feedback",
                    "dynamixel_health",
                    "steer_feedback",
                    "safety",
                    "camera_depth_diagnostics",
                    "gicp_score",
                    "gicp_pose",
                    "lifecycle_controller_server",
                    "lifecycle_planner_server",
                    "lifecycle_bt_navigator",
                    "lifecycle_local_costmap",
                ],
            ),
            self._capability(
                "stationary_perception",
                "On-demand perception (stationary)",
                domains,
                [
                    "camera_color_diagnostics",
                    "camera_depth_diagnostics",
                    "camera_pan",
                    "camera_tilt",
                    "perception_runtime",
                ],
            ),
            self._capability(
                "lift",
                "Lift",
                domains,
                ["physical_estop", "lift_position"],
            ),
            self._capability(
                "so101_arm",
                "SO101 arm",
                domains,
                [
                    "so101_arm_hardware_status",
                    "so101_arm_gateway_delivery",
                    "so101_arm_visual_joint_states",
                ],
            ),
        ]
        self.sequence += 1
        state = {
            "schema_version": 1,
            "generated_at": utc_now(),
            "sequence": self.sequence,
            "monitor": {
                "node": "/health_monitor",
                "publish_rate_hz": float(self.get_parameter("publish_rate_hz").value),
                "input_sample_rate_hz": float(
                    self.get_parameter("input_sample_rate_hz").value
                ),
                "previous_publish_duration_sec": self._last_publish_duration_sec,
                "executor_threads": 2,
                "mode": "read_only",
                "truth_policy": "unknown_is_not_ok",
                "high_bandwidth_subscriptions": False,
            },
            "summaries": summaries,
            "capabilities": capabilities,
            "domains": domains,
        }
        safe_state = json_safe(state)
        self.publisher.publish(String(data=json.dumps(safe_state, ensure_ascii=False, allow_nan=False)))
        self._publish_diagnostics(domains)
        self._last_publish_duration_sec = time.monotonic() - publish_started

    def _publish_diagnostics(self, domains: list[dict[str, Any]]) -> None:
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        for domain in domains:
            domain_id = str(domain.get("id", "unknown"))
            for item in domain.get("items", []):
                status_text = str(item.get("status", "UNKNOWN"))
                status = DiagnosticStatus()
                status.name = f"health_dashboard/{domain_id}/{item.get('id', 'unknown')}"
                status.hardware_id = str(item.get("source") or "not_available")
                status.message = str(item.get("summary") or status_text)
                if status_text == "STALE":
                    status.level = DiagnosticStatus.STALE
                elif status_text in {"BLOCKED", "ERROR"}:
                    status.level = DiagnosticStatus.ERROR
                elif status_text in UNCERTAIN_ITEM_STATES | DEGRADED_ITEM_STATES:
                    status.level = DiagnosticStatus.WARN
                else:
                    status.level = DiagnosticStatus.OK
                values = {
                    "native_status": status_text,
                    "evidence": item.get("evidence"),
                    "critical": item.get("critical"),
                    "receive_age_sec": item.get("receive_age_sec"),
                    "source_age_sec": item.get("source_age_sec"),
                    "receive_rate_hz_5s": item.get("receive_rate_hz_5s"),
                    "max_receive_gap_sec_5s": item.get("max_receive_gap_sec_5s"),
                    "standard": item.get("standard"),
                }
                status.values = [
                    KeyValue(key=key, value=json.dumps(json_safe(value), ensure_ascii=False))
                    for key, value in values.items()
                ]
                output.status.append(status)
        self.diagnostics_publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HealthMonitor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
