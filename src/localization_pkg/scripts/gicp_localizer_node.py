#!/usr/bin/python3
"""
gicp_localizer_node.py — GICP prior-map localization for ROS 2 Humble.

Replaces GLIM localization with deterministic scan-to-map matching.

Input topics:
  /livox/lidar       sensor_msgs/PointCloud2   (Livox xfer_format=0, XYZRTL)
  /initialpose       geometry_msgs/PoseWithCovarianceStamped  (RViz 2D Pose Estimate)
  /fast_lio/odometry nav_msgs/Odometry         (optional, Phase 2 FAST-LIO2 hint)

Output:
  /gicp_loc/pose     geometry_msgs/PoseStamped  (accepted planar navigation sensor pose)
  /gicp_loc/raw_pose geometry_msgs/PoseStamped  (accepted 3-D registration sensor pose)
  /gicp_loc/score    std_msgs/Float32           (GICP inlier RMSE — lower is better)
  TF broadcast:      map → odom

TF chain after this node:
  map → odom  (this node)
  odom → base_footprint  (wheel odom, robot_odom_node — unchanged)
  base_footprint → base_link → livox_frame  (URDF)

Algorithm:
  1. Get T_odom_lidar at the incoming scan timestamp
  2. Seed from the full 3-D registration prior and odom, or a fresh FAST-LIO delta
  3. Voxel-downsample incoming scan, estimate normals
  4. GICP: match scan against preloaded PLY map (fixed, never modified)
  5. Compute T_map_odom = T_map_lidar_corrected × inv(T_odom_lidar)
  6. Validate RMSE, overlap and correction limits; keep the full 3-D prior,
     but publish a planar map→odom TF and consistent accepted navigation pose

Backend (auto-detected at startup):
  open3d  — /usr/bin/python3 -m pip install --user open3d
  small_gicp — /usr/bin/python3 -m pip install --user small-gicp  (fallback)
"""

import os
import sys
import math
import threading
import time

import numpy as np
from initialpose_scan_window import InitialPoseScanWindow
from localization_evidence import hint_is_current, matching_seed, registration_valid, hint_agrees_with_odometry
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
from rosgraph_msgs.msg import Clock as ClockMsg
import json
from rclpy.clock import Clock, ClockType
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

# ── backend detection ─────────────────────────────────────────────────────────
try:
    import open3d as o3d
    _BACKEND = "open3d"
except ImportError:
    try:
        import small_gicp  # noqa: F401
        _BACKEND = "small_gicp"
    except ImportError:
        print(
            "[gicp_localizer] ERROR: no matching backend found.\n"
            "  Install open3d:    /usr/bin/python3 -m pip install --user open3d\n"
            "  or small_gicp:     /usr/bin/python3 -m pip install --user small-gicp",
            file=sys.stderr,
        )
        sys.exit(1)


# ── helper: TF ↔ 4×4 matrix ──────────────────────────────────────────────────

def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = q / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quat(R: np.ndarray) -> np.ndarray:
    m = np.asarray(R, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s,
                0.25 * s,
            ]
        )

    idx = int(np.argmax(np.diag(m)))
    if idx == 0:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = np.array(
            [
                0.25 * s,
                (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s,
                (m[2, 1] - m[1, 2]) / s,
            ]
        )
    elif idx == 1:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = np.array(
            [
                (m[0, 1] + m[1, 0]) / s,
                0.25 * s,
                (m[1, 2] + m[2, 1]) / s,
                (m[0, 2] - m[2, 0]) / s,
            ]
        )
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = np.array(
            [
                (m[0, 2] + m[2, 0]) / s,
                (m[1, 2] + m[2, 1]) / s,
                0.25 * s,
                (m[1, 0] - m[0, 1]) / s,
            ]
        )

    norm = np.linalg.norm(q)
    return q / norm if norm > 0.0 else np.array([0.0, 0.0, 0.0, 1.0])


def _yaw_to_matrix(yaw: float) -> np.ndarray:
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _matrix_yaw(T: np.ndarray) -> float:
    return math.atan2(float(T[1, 0]), float(T[0, 0]))


def _planarize_map_to_odom(T: np.ndarray) -> np.ndarray:
    """Constrain map->odom to 2-D x/y/yaw for Nav2 and RViz."""
    yaw = _matrix_yaw(T)
    out = np.eye(4)
    out[:3, :3] = _yaw_to_matrix(yaw)
    out[0, 3] = float(T[0, 3])
    out[1, 3] = float(T[1, 3])
    return out


def _blend_planar_transform(current: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
    alpha = max(0.0, min(1.0, float(alpha)))
    cur = _planarize_map_to_odom(current)
    tgt = _planarize_map_to_odom(target)
    yaw = _matrix_yaw(cur) + alpha * _normalize_angle(_matrix_yaw(tgt) - _matrix_yaw(cur))
    out = np.eye(4)
    out[:3, :3] = _yaw_to_matrix(yaw)
    out[0, 3] = float(cur[0, 3]) + alpha * (float(tgt[0, 3]) - float(cur[0, 3]))
    out[1, 3] = float(cur[1, 3]) + alpha * (float(tgt[1, 3]) - float(cur[1, 3]))
    return out


def _tf_to_matrix(tf: TransformStamped) -> np.ndarray:
    """TransformStamped → 4×4 homogeneous matrix (T_parent_child)."""
    t = tf.transform.translation
    r = tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = _quat_to_matrix(r.x, r.y, r.z, r.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def _matrix_to_tf(
    T: np.ndarray,
    stamp,
    parent_frame: str,
    child_frame: str,
) -> TransformStamped:
    """4×4 homogeneous matrix → TransformStamped."""
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])
    q = _matrix_to_quat(T[:3, :3])  # [x, y, z, w]
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])
    return msg


def _matrix_to_pose(T: np.ndarray, stamp, frame: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])
    q = _matrix_to_quat(T[:3, :3])
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    return msg


# ── PointCloud2 parsing ───────────────────────────────────────────────────────

def _pc2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Livox PointCloud2 (XYZRTL, xfer_format=0) → Nx3 float32 xyz array."""
    pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    if len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    if pts.dtype.names:
        arr = np.column_stack((pts["x"], pts["y"], pts["z"])).astype(
            np.float32, copy=False
        )
    else:
        arr = np.asarray(pts, dtype=np.float32).reshape(-1, 3)

    return arr[np.isfinite(arr).all(axis=1)]


# ── main node ─────────────────────────────────────────────────────────────────

class GicpLocalizer(Node):
    def __init__(self):
        super().__init__("gicp_localizer")

        # ── parameter declarations ────────────────────────────────────────────
        self.declare_parameter("map_path", "")
        self.declare_parameter("map_voxel_size", 0.25)
        self.declare_parameter("scan_voxel_size", 0.20)
        self.declare_parameter("max_correspondence_distance", 1.5)
        self.declare_parameter("fitness_score_threshold", 0.5)
        self.declare_parameter("normal_search_radius", 0.8)
        self.declare_parameter("localize_period", 0.5)
        self.declare_parameter("tf_broadcast_period", 1.0 / 30.0)
        self.declare_parameter("scan_min_height", float("-inf"))
        self.declare_parameter("scan_max_height", float("inf"))
        self.declare_parameter("max_correction_translation_m", 0.08)
        self.declare_parameter("max_correction_yaw_rad", 0.08)
        self.declare_parameter("correction_alpha", 0.35)
        self.declare_parameter("initialpose_relocalization_updates", 3)
        self.declare_parameter("initialpose_max_correction_translation_m", 1.5)
        self.declare_parameter("initialpose_max_correction_yaw_rad", 0.4)
        self.declare_parameter("initialpose_limit_correction", False)
        self.declare_parameter("auto_relocalization_reject_count", 0)
        self.declare_parameter("auto_relocalization_updates", 3)
        self.declare_parameter("auto_relocalization_max_correction_translation_m", 6.5)
        self.declare_parameter("auto_relocalization_max_correction_yaw_rad", 1.4)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("lidar_frame", "livox_frame")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("use_fast_lio_hint", False)
        self.declare_parameter("fast_lio_odom_topic", "/fast_lio/odometry")
        self.declare_parameter("initialpose_accumulation_sec", 2.5)
        self.declare_parameter("initialpose_min_scans", 10)
        self.declare_parameter("initialpose_max_scans", 30)
        self.declare_parameter("initialpose_sim_accumulation_sec", 10.0)
        self.declare_parameter("initialpose_accumulation_timeout_sec", 75.0)
        self.declare_parameter("initialpose_sim_azimuth_coverage", 0.90)
        self.declare_parameter("fast_lio_max_stamp_skew_sec", 0.5)
        self.declare_parameter("fast_lio_max_seed_disagreement_m", 0.75)
        self.declare_parameter("fast_lio_max_seed_disagreement_rad", 0.8)
        self.declare_parameter("scan_max_source_age_sec", 2.0)
        self.declare_parameter("minimum_match_fitness", 0.25)

        p = self.get_parameter
        self._map_path   = p("map_path").value
        self._map_voxel  = p("map_voxel_size").value
        self._scan_voxel = p("scan_voxel_size").value
        self._max_corr   = p("max_correspondence_distance").value
        self._fit_thr    = p("fitness_score_threshold").value
        self._normal_r   = p("normal_search_radius").value
        self._period     = p("localize_period").value
        self._tf_period  = p("tf_broadcast_period").value
        self._scan_min_height = float(p("scan_min_height").value)
        self._scan_max_height = float(p("scan_max_height").value)
        self._max_correction_translation = max(
            0.0, float(p("max_correction_translation_m").value)
        )
        self._max_correction_yaw = max(0.0, float(p("max_correction_yaw_rad").value))
        self._correction_alpha = max(0.0, min(1.0, float(p("correction_alpha").value)))
        self._initialpose_relocalization_updates = max(
            0, int(p("initialpose_relocalization_updates").value)
        )
        self._initialpose_max_correction_translation = max(
            self._max_correction_translation,
            float(p("initialpose_max_correction_translation_m").value),
        )
        self._initialpose_max_correction_yaw = max(
            self._max_correction_yaw,
            float(p("initialpose_max_correction_yaw_rad").value),
        )
        self._auto_relocalization_reject_count = max(
            0, int(p("auto_relocalization_reject_count").value)
        )
        self._auto_relocalization_updates = max(
            0, int(p("auto_relocalization_updates").value)
        )
        self._auto_relocalization_max_correction_translation = max(
            self._initialpose_max_correction_translation,
            float(p("auto_relocalization_max_correction_translation_m").value),
        )
        self._auto_relocalization_max_correction_yaw = max(
            self._initialpose_max_correction_yaw,
            float(p("auto_relocalization_max_correction_yaw_rad").value),
        )
        self._map_frame  = p("map_frame").value
        self._odom_frame = p("odom_frame").value
        self._base_frame = p("base_frame").value
        self._lidar_frame = p("lidar_frame").value
        self._initialpose_accumulation_sec = max(
            0.0, float(p("initialpose_accumulation_sec").value)
        )
        self._initialpose_min_scans = max(1, int(p("initialpose_min_scans").value))
        self._initialpose_max_scans = max(
            self._initialpose_min_scans,
            int(p("initialpose_max_scans").value),
        )

        # Initial T_map_odom from 2-D startup pose params
        x   = p("initial_x").value
        y   = p("initial_y").value
        yaw = p("initial_yaw").value
        T0 = np.eye(4)
        T0[:3, :3] = _yaw_to_matrix(yaw)
        T0[0, 3] = x
        T0[1, 3] = y
        self._T_map_odom: np.ndarray = T0
        # The 3-D prior map may have a different height/tilt origin from Nav2.
        # Do not feed the flattened navigation transform back into registration.
        self._T_matching_odom: np.ndarray = T0.copy()
        self._lock = threading.Lock()

        # ── TF infrastructure ─────────────────────────────────────────────────
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Separate timer callback groups let the 30 Hz TF rebroadcast keep
        # running while the heavier GICP registration callback is busy.
        self._gicp_group = MutuallyExclusiveCallbackGroup()
        self._tf_group = MutuallyExclusiveCallbackGroup()

        # ── publishers ────────────────────────────────────────────────────────
        self._pose_pub  = self.create_publisher(PoseStamped, "/gicp_loc/pose",  10)
        self._raw_pose_pub = self.create_publisher(PoseStamped, "/gicp_loc/raw_pose", 10)
        self._accepted_state_pub = self.create_publisher(String, "/gicp_loc/accepted_state", 10)
        self._score_pub = self.create_publisher(Float32,      "/gicp_loc/score", 10)
        self._recovery_status_pub = self.create_publisher(String, "/gicp_loc/recovery_status", 10)
        self._last_match_result = "not_attempted"
        self._match_details = {}
        self._hint_used = False
        self._hint_stamp_skew = None

        # ── subscribers ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._scan_sub = self.create_subscription(
            PointCloud2, "/livox/lidar", self._scan_cb, sensor_qos
        )
        self._init_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10,
            callback_group=self._gicp_group,
        )

        # FAST-LIO2 hint state (all under self._lock)
        self._fast_lio_T: np.ndarray | None = None               # current FAST-LIO pose
        self._fast_lio_stamp = 0.0
        self._fast_lio_received = 0.0
        self._fast_lio_reference_valid = False
        self._fast_lio_T_at_correction: np.ndarray | None = None # snapshot at last GICP accept
        self._T_map_lidar_at_correction: np.ndarray | None = None # T_map_lidar at last accept
        if p("use_fast_lio_hint").value:
            topic = p("fast_lio_odom_topic").value
            self.create_subscription(Odometry, topic, self._fast_lio_cb, sensor_qos)
            self.get_logger().info(f"[gicp_localizer] FAST-LIO2 hint enabled: {topic}")

        # latest scan buffer (consumed each localize cycle)
        self._latest_scan: np.ndarray | None = None
        self._latest_scan_stamp = None
        self._scan_lock = threading.Lock()
        self._initialpose_accumulation_started = 0.0
        self._initialpose_scans: list[np.ndarray] = []
        self._initialpose_window = None
        self._initialpose_generation = 0
        self._last_clock_sec = None
        self._clock_reset_needs_seed = False
        self.create_subscription(ClockMsg, "/clock", self._clock_cb, 10, callback_group=self._gicp_group)
        self._initialpose_odom_base = None
        self._relaxed_correction_remaining = 0
        self._relaxed_correction_source = ""
        self._consecutive_correction_rejects = 0

        # ── load map ──────────────────────────────────────────────────────────
        self._map_cloud = self._load_map(self._map_path)
        self.create_timer(1.0, self._publish_recovery_status,
                          clock=Clock(clock_type=ClockType.STEADY_TIME), callback_group=self._tf_group)

        # ── timers ────────────────────────────────────────────────────────────
        # GICP can be relatively expensive, so its correction rate is separate
        # from the cheap TF rebroadcast rate Nav2 needs for fresh lookups.
        self.create_timer(
            self._period,
            self._localize,
            callback_group=self._gicp_group,
        )
        self.create_timer(
            self._tf_period,
            self._broadcast,
            callback_group=self._tf_group,
        )

        self.get_logger().info(
            f"[gicp_localizer] started  backend={_BACKEND}  "
            f"period={self._period}s  "
            f"tf_period={self._tf_period}s  "
            f"map_voxel={self._map_voxel}m  scan_voxel={self._scan_voxel}m  "
            f"initialpose_accumulation={self._initialpose_accumulation_sec}s/"
            f"{self._initialpose_min_scans} scans"
        )

    # ── map loading ───────────────────────────────────────────────────────────

    def _load_map(self, path: str):
        if not path or not os.path.exists(path):
            self.get_logger().error(
                f"[gicp_localizer] map file not found: '{path}'\n"
                "  Set parameter map_path= or update gicp_localizer.yaml"
            )
            return None

        self.get_logger().info(f"[gicp_localizer] loading map: {path}")

        if _BACKEND == "open3d":
            read_kwargs = {}
            with open(path, "rb") as f:
                if f.read(3) == b"ply":
                    read_kwargs["format"] = "ply"
            cloud = o3d.io.read_point_cloud(path, **read_kwargs)
            if len(cloud.points) == 0:
                self.get_logger().error(
                    f"[gicp_localizer] map has 0 points after reading: '{path}'\n"
                    "  If this is a PLY file without .ply extension, keep the "
                    "PLY header or rename it to *.ply."
                )
                return None
            cloud = cloud.voxel_down_sample(self._map_voxel)
            if len(cloud.points) == 0:
                self.get_logger().error(
                    f"[gicp_localizer] map has 0 points after voxel downsample "
                    f"(voxel {self._map_voxel} m): '{path}'"
                )
                return None
            cloud.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self._normal_r, max_nn=30
                )
            )
            if not cloud.has_normals():
                self.get_logger().error(
                    f"[gicp_localizer] normal estimation failed for map: '{path}'\n"
                    f"  Try reducing normal_search_radius or map_voxel_size."
                )
                return None
            try:
                cloud.orient_normals_towards_camera_location(np.zeros(3))
            except RuntimeError as exc:
                self.get_logger().warn(
                    f"[gicp_localizer] normal orientation skipped: {exc}"
                )
            self.get_logger().info(
                f"[gicp_localizer] map ready: {len(cloud.points)} pts"
                f" (voxel {self._map_voxel} m)"
            )
            return cloud

        # small_gicp backend
        import small_gicp
        raw = small_gicp.read_point_cloud(path)
        cloud, tree = small_gicp.preprocess_points(
            raw, downsampling_resolution=self._map_voxel
        )
        self.get_logger().info(
            f"[gicp_localizer] map ready (small_gicp, voxel {self._map_voxel} m)"
        )
        return (cloud, tree)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _clock_cb(self, msg):
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        if self._last_clock_sec is not None and stamp < self._last_clock_sec:
            with self._scan_lock:
                self._latest_scan = None
                self._initialpose_window = None
                self._initialpose_accumulation_started = 0.0
            with self._lock:
                self._initialpose_generation += 1
                self._fast_lio_reference_valid = False
                self._fast_lio_stamp = 0.0
            self._clock_reset_needs_seed = True
            self._last_match_result = "clock_reset_requires_initialpose"
        self._last_clock_sec = stamp

    def _publish_recovery_status(self):
        with self._scan_lock:
            window = self._initialpose_window
            accumulating = bool(self._initialpose_accumulation_started)
            if accumulating and window and not window.failed and time.monotonic() - window.started > window.timeout_sec:
                window.failed = True
                window.failure_reason = "fresh_scan_coverage_timeout"
            value = {"phase": "awaiting_initialpose" if self._clock_reset_needs_seed else "failed" if window and window.failed else "accumulating" if accumulating else "matching",
                     "failure_reason": "clock_reset_requires_initialpose" if self._clock_reset_needs_seed else window.failure_reason if window else None,
                     "coverage": window.coverage_observed if window else None,
                     "coverage_required": window.coverage if window else None,
                     "scan_count": len(window.chunks) if window else 0,
                     "point_count": sum(len(p) for p in window.chunks) if window else 0,
                     "elapsed_wall_sec": time.monotonic() - window.started if window and accumulating else None,
                     "minimum_wall_sec": window.min_sec if window and not window.source_time else None,
                     "minimum_source_sec": window.min_sec if window and window.source_time else None,
                     "accumulation_time_basis": "scan_stamp" if window and window.source_time else "wall",
                     "accumulated_source_sec": window.source_span_sec if window else None,
                     "last_match_result": self._last_match_result,
                     "match": self._match_details,
                     "fast_lio_hint_used": self._hint_used,
                     "fast_lio_stamp_skew_sec": self._hint_stamp_skew,
                     "pose_representation": "accepted_planar_navigation_lidar_pose",
                     "map_path": self._map_path, "map_loaded": self._map_cloud is not None,
                     "map_frame": self._map_frame, "odom_frame": self._odom_frame,
                     "base_frame": self._base_frame, "lidar_frame": self._lidar_frame}
        self._recovery_status_pub.publish(String(data=json.dumps(value)))

    def _scan_cb(self, msg: PointCloud2):
        arr = _pc2_to_numpy(msg)
        coverage_points = arr
        if arr.shape[0] and (
            math.isfinite(self._scan_min_height) or math.isfinite(self._scan_max_height)
        ):
            keep = np.ones(arr.shape[0], dtype=bool)
            if math.isfinite(self._scan_min_height):
                keep &= arr[:, 2] >= self._scan_min_height
            if math.isfinite(self._scan_max_height):
                keep &= arr[:, 2] <= self._scan_max_height
            arr = arr[keep]
        if arr.shape[0] == 0:
            return
        with self._scan_lock:
            if self._initialpose_accumulation_started:
                window = self._initialpose_window
                if window.failed and window.failure_reason not in {"fresh_scan_coverage_timeout", "base_moved_during_accumulation"}:
                    return
                # Uncompensated accumulation is only valid while stationary.
                try:
                    current = _tf_to_matrix(self._tf_buf.lookup_transform(
                        self._odom_frame, self._base_frame, rclpy.time.Time()))
                    anchor = self._initialpose_odom_base
                    if anchor is None or np.linalg.norm(current[:2, 3] - anchor[:2, 3]) > 0.03 or abs(
                        _normalize_angle(_matrix_yaw(current) - _matrix_yaw(anchor))
                    ) > 0.05:
                        # Restart data accumulation, not the pose or the mission.
                        # A moving base cannot accumulate uncompensated scans.
                        self._initialpose_odom_base = current.copy()
                        self._initialpose_window = InitialPoseScanWindow(
                            time.monotonic(), window.min_sec, window.min_scans, window.max_scans,
                            window.timeout_sec, window.coverage,
                            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec, source_time=window.source_time)
                        self._last_match_result = "waiting_stationary_scan_window"
                        return
                except TransformException:
                    window.failed = True
                    window.failure_reason = "odom_base_tf_unavailable"
                if msg.header.frame_id.lstrip("/") != self._lidar_frame.lstrip("/"):
                    window.failed = True
                    window.failure_reason = "lidar_frame_mismatch"
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                scan = window.add(arr, stamp, time.monotonic(), coverage_points=coverage_points)
                if window.failed:
                    self.get_logger().warning(f"[gicp_localizer] Scan window: {window.failure_reason}; waiting for fresh stationary evidence.", throttle_duration_sec=10)
                    return
                if scan is None:
                    return
                self._latest_scan = scan
                self._latest_scan_stamp = msg.header.stamp
                self._initialpose_accumulation_started = 0.0
                self._last_match_result = "fresh_scan_window_ready_waiting_match"
                self.get_logger().info(
                    f"[gicp_localizer] /initialpose accumulated {len(window.chunks)} fresh scans, "
                    f"{scan.shape[0]} points across {window.source_span_sec:.2f} scan seconds "
                    f"({time.monotonic() - window.started:.2f} wall seconds); "
                    "starting GICP validation"
                )
                return
            self._latest_scan = arr
            self._latest_scan_stamp = msg.header.stamp

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id.lstrip("/") != self._map_frame.lstrip("/"):
            self.get_logger().error("[gicp_localizer] Ignoring initial pose outside map frame")
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]).all() or np.linalg.norm([q.x, q.y, q.z, q.w]) < 1e-6:
            self.get_logger().error("[gicp_localizer] Ignoring non-finite initial pose or invalid quaternion")
            return
        T_map_base = np.eye(4)
        T_map_base[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
        T_map_base[:3, 3] = [p.x, p.y, p.z]

        try:
            tf_stamped = self._tf_buf.lookup_transform(
                self._odom_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            T_odom_base = _tf_to_matrix(tf_stamped)
            T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
        except TransformException as exc:
            self.get_logger().warn(
                f"[gicp_localizer] /initialpose could not read "
                f"{self._odom_frame}->{self._base_frame}: {exc}; "
                "rejecting seed rather than guessing its frame"
            )
            return

        T_map_odom = _planarize_map_to_odom(T_map_odom)

        with self._lock:
            self._initialpose_generation += 1
            self._clock_reset_needs_seed = False
            self._T_map_odom = T_map_odom
            # Retain established map height/tilt while replacing the XY/yaw seed.
            old_yaw = _matrix_yaw(self._T_matching_odom)
            self._T_matching_odom[:3, :3] = _yaw_to_matrix(_matrix_yaw(T_map_odom) - old_yaw) @ self._T_matching_odom[:3, :3]
            self._T_matching_odom[:2, 3] = T_map_odom[:2, 3]
            # A manual pose is a new absolute anchor. Old FAST-LIO/GICP
            # correction snapshots must not override it on the next cycle.
            self._fast_lio_T_at_correction = None
            self._T_map_lidar_at_correction = None
            self._fast_lio_reference_valid = False
            self._relaxed_correction_remaining = self._initialpose_relocalization_updates
            self._relaxed_correction_source = "initialpose"
            self._consecutive_correction_rejects = 0
            self._last_match_result = "waiting_fresh_stationary_scans"
            self._match_details = {}
            self._hint_used = False
            self._hint_stamp_skew = None
        with self._scan_lock:
            self._latest_scan = None
            self._latest_scan_stamp = None
            self._initialpose_scans = []
            self._initialpose_accumulation_started = time.monotonic()
            self._initialpose_odom_base = T_odom_base.copy()
            sim = bool(self.get_parameter("use_sim_time").value)
            duration = max(self._initialpose_accumulation_sec, float(self.get_parameter("initialpose_sim_accumulation_sec").value)) if sim else self._initialpose_accumulation_sec
            self._initialpose_window = InitialPoseScanWindow(
                self._initialpose_accumulation_started, duration, self._initialpose_min_scans,
                self._initialpose_max_scans, float(self.get_parameter("initialpose_accumulation_timeout_sec").value),
                float(self.get_parameter("initialpose_sim_azimuth_coverage").value) if sim else 0.0,
                self.get_clock().now().nanoseconds,
                source_time=True,
            )
        self._broadcast()
        self.get_logger().info(
            f"[gicp_localizer] /initialpose reset {self._map_frame}->{self._odom_frame}: "
            f"robot x={p.x:.2f} y={p.y:.2f}; clearing old FAST-LIO hint and "
            f"accumulating at least {duration:.1f}s / {self._initialpose_min_scans} fresh scans; "
            f"azimuth coverage target={self._initialpose_window.coverage:.0%}, no base rotation"
        )

    def _fast_lio_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [p.x, p.y, p.z]
        with self._lock:
            self._fast_lio_T = T
            self._fast_lio_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self._fast_lio_received = time.monotonic()

    # ── localization timer ────────────────────────────────────────────────────

    def _localize(self):
        if self._map_cloud is None or self._clock_reset_needs_seed:
            return
        with self._scan_lock:
            if self._latest_scan is None:
                return
            scan_pts = self._latest_scan
            scan_stamp = self._latest_scan_stamp
            self._latest_scan = None
        if scan_stamp is None:
            return
        scan_time = scan_stamp.sec + scan_stamp.nanosec / 1e9
        source_age = self.get_clock().now().nanoseconds / 1e9 - scan_time
        if source_age < -0.1 or source_age > float(self.get_parameter("scan_max_source_age_sec").value):
            self._last_match_result = "stale_or_future_scan_rejected"
            return

        # Lookup odom → livox_frame
        try:
            tf_stamped = self._tf_buf.lookup_transform(
                self._odom_frame,
                self._lidar_frame,
                rclpy.time.Time.from_msg(scan_stamp),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            T_odom_lidar = _tf_to_matrix(tf_stamped)
        except TransformException as exc:
            self.get_logger().warn(
                f"[gicp_localizer] TF lookup failed: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        with self._lock:
            T_map_odom = self._T_map_odom.copy()
            T_matching_odom = self._T_matching_odom.copy()
            generation = self._initialpose_generation
            T_flio_cur = self._fast_lio_T.copy() if self._fast_lio_T is not None else None
            T_flio_ref = self._fast_lio_T_at_correction.copy() if self._fast_lio_T_at_correction is not None else None
            T_map_lidar_ref = self._T_map_lidar_at_correction.copy() if self._T_map_lidar_at_correction is not None else None
            hint_current = hint_is_current(scan_time, self._fast_lio_stamp, self._fast_lio_received,
                                           time.monotonic(), float(self.get_parameter("fast_lio_max_stamp_skew_sec").value))
            hint_reference_valid = self._fast_lio_reference_valid
            self._hint_stamp_skew = abs(scan_time - self._fast_lio_stamp) if self._fast_lio_stamp else None

        # Initial guess: prefer FAST-LIO delta hint when available
        self._hint_used = bool(hint_current and hint_reference_valid and T_flio_ref is not None)
        T_init = matching_seed(T_matching_odom, T_odom_lidar,
                               T_flio_cur if self._hint_used else None, T_flio_ref, T_map_lidar_ref)
        wheel_seed = matching_seed(T_matching_odom, T_odom_lidar)
        if self._hint_used and not hint_agrees_with_odometry(
                T_init, wheel_seed, float(self.get_parameter("fast_lio_max_seed_disagreement_m").value),
                float(self.get_parameter("fast_lio_max_seed_disagreement_rad").value)):
            T_init = wheel_seed
            self._hint_used = False
            self.get_logger().warning("FAST-LIO hint disagrees with wheel odometry; using wheel seed", throttle_duration_sec=5)

        # Run GICP
        started = time.monotonic()
        score, T_map_lidar = self._run_gicp(scan_pts, T_init)
        self._match_details.update(duration_wall_sec=time.monotonic() - started, scan_stamp_sec=scan_time,
                                   scan_source_age_sec=source_age, backend=_BACKEND)
        if not math.isfinite(score) or not np.isfinite(T_map_lidar).all():
            self._last_match_result = "non_finite_match_rejected"
            self.get_logger().warning("[gicp_localizer] Non-finite registration result rejected", throttle_duration_sec=5)
            return

        with self._lock:
            if generation != self._initialpose_generation:
                return  # A result computed against the old seed is not recovery evidence.

        # Open3D reports RMSE=0 when there are no correspondences. That is
        # invalid evidence, not a perfect match, for every score subscriber.
        valid = registration_valid(score, self._match_details["fitness"], self._match_details["correspondences"],
                                   T_map_lidar, float(self.get_parameter("minimum_match_fitness").value))
        score_msg = Float32()
        score_msg.data = float(score) if valid else float("inf")
        self._score_pub.publish(score_msg)

        if not valid:
            self._last_match_result = "insufficient_match_overlap"
            return

        if score > self._fit_thr:
            self._last_match_result = "score_rejected"
            with self._lock:
                self._consecutive_correction_rejects += 1
            self.get_logger().warn(
                f"[gicp_localizer] score {score:.4f} > thr {self._fit_thr} — keeping previous TF",
                throttle_duration_sec=3.0,
            )
            return

        # T_map_odom = T_map_lidar × inv(T_odom_lidar)
        T_map_odom_candidate = _planarize_map_to_odom(
            T_map_lidar @ np.linalg.inv(T_odom_lidar)
        )
        correction_xy = math.hypot(
            float(T_map_odom_candidate[0, 3]) - float(T_map_odom[0, 3]),
            float(T_map_odom_candidate[1, 3]) - float(T_map_odom[1, 3]),
        )
        correction_yaw = abs(
            _normalize_angle(_matrix_yaw(T_map_odom_candidate) - _matrix_yaw(T_map_odom))
        )
        with self._lock:
            relaxed_remaining = self._relaxed_correction_remaining
            relaxed_source = self._relaxed_correction_source
            reject_count = self._consecutive_correction_rejects
            if (
                relaxed_remaining <= 0
                and self._auto_relocalization_reject_count > 0
                and self._auto_relocalization_updates > 0
                and reject_count >= self._auto_relocalization_reject_count
            ):
                self._relaxed_correction_remaining = self._auto_relocalization_updates
                self._relaxed_correction_source = "auto"
                relaxed_remaining = self._relaxed_correction_remaining
                relaxed_source = self._relaxed_correction_source
                self._consecutive_correction_rejects = 0
                self.get_logger().warn(
                    "[gicp_localizer] opening bounded auto-relocalization window "
                    f"after {reject_count} rejected corrections"
                )
        if relaxed_remaining > 0 and relaxed_source == "auto":
            max_translation = self._auto_relocalization_max_correction_translation
            max_yaw = self._auto_relocalization_max_correction_yaw
        elif relaxed_remaining > 0:
            max_translation = self._initialpose_max_correction_translation
            max_yaw = self._initialpose_max_correction_yaw
            # An explicit human/recovery seed may intentionally move across the map.
            # Score/finite-transform checks remain; normal tracking stays bounded.
            if not self.get_parameter("initialpose_limit_correction").value:
                max_translation = max_yaw = 0.0
        else:
            max_translation = self._max_correction_translation
            max_yaw = self._max_correction_yaw
        if (
            max_translation > 0.0
            and correction_xy > max_translation
        ) or (
            max_yaw > 0.0
            and correction_yaw > max_yaw
        ):
            self._last_match_result = "correction_jump_rejected"
            with self._lock:
                self._consecutive_correction_rejects += 1
            self.get_logger().warn(
                f"[gicp_localizer] correction jump rejected: "
                f"dxy={correction_xy:.3f}m dyaw={math.degrees(correction_yaw):.2f}deg "
                f"(limits {max_translation:.3f}m/"
                f"{math.degrees(max_yaw):.2f}deg"
                f"{', ' + relaxed_source if relaxed_remaining > 0 else ''})",
                throttle_duration_sec=2.0,
            )
            return

        correction_alpha = 1.0 if relaxed_remaining > 0 else self._correction_alpha
        self._last_match_result = "correction_accepted"
        T_map_odom_new = _blend_planar_transform(
            T_map_odom,
            T_map_odom_candidate,
            correction_alpha,
        )

        with self._lock:
            self._T_map_odom = T_map_odom_new
            self._T_matching_odom = T_map_lidar @ np.linalg.inv(T_odom_lidar)
            self._consecutive_correction_rejects = 0
            if self._relaxed_correction_remaining > 0:
                self._relaxed_correction_remaining -= 1
                if self._relaxed_correction_remaining == 0:
                    self._relaxed_correction_source = ""
            else:
                self._relaxed_correction_source = ""
            remaining_after = self._relaxed_correction_remaining
            # Save reference snapshots for next FAST-LIO delta hint
            self._fast_lio_reference_valid = hint_current
            if T_flio_cur is not None and hint_current:
                self._fast_lio_T_at_correction = T_flio_cur
                self._T_map_lidar_at_correction = T_map_lidar.copy()

        message = (
            f"[gicp_localizer] correction accepted: "
            f"dxy={correction_xy:.3f}m dyaw={math.degrees(correction_yaw):.2f}deg "
            f"score={score:.4f} alpha={correction_alpha:.2f} "
            f"mode={relaxed_source or 'normal'} remaining={remaining_after}"
        )
        if relaxed_source:
            self.get_logger().info(message)
        else:
            self.get_logger().debug(message, throttle_duration_sec=10)

        self._raw_pose_pub.publish(_matrix_to_pose(T_map_lidar, scan_stamp, self._map_frame))
        self._pose_pub.publish(_matrix_to_pose(T_map_odom_new @ T_odom_lidar, scan_stamp, self._map_frame))
        try:
            T_lidar_base = _tf_to_matrix(self._tf_buf.lookup_transform(
                self._lidar_frame, self._base_frame, rclpy.time.Time.from_msg(scan_stamp)))
            T_odom_base = T_odom_lidar @ T_lidar_base
            T_map_base = T_map_odom_new @ T_odom_base
            def planar(T):
                return [float(T[0, 3]), float(T[1, 3]), _matrix_yaw(T)]
            self._accepted_state_pub.publish(String(data=json.dumps({
                "stamp_sec": scan_time, "map_frame": self._map_frame,
                "odom_frame": self._odom_frame, "base_frame": self._base_frame,
                "map_base_pose": planar(T_map_base), "odom_base_pose": planar(T_odom_base),
                "rmse_m": score, "fitness": self._match_details["fitness"],
                "correspondences": self._match_details["correspondences"],
                "generation": generation,
            })))
        except TransformException:
            self.get_logger().warning("Accepted base evidence unavailable: lidar/base TF missing", throttle_duration_sec=5)

    def _run_gicp(
        self, scan_pts: np.ndarray, T_init: np.ndarray
    ) -> tuple[float, np.ndarray]:
        """Return (inlier_rmse_m, T_map_lidar_corrected)."""
        if _BACKEND == "open3d":
            src = o3d.geometry.PointCloud()
            src.points = o3d.utility.Vector3dVector(scan_pts)
            src = src.voxel_down_sample(self._scan_voxel)
            src.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self._normal_r, max_nn=20
                )
            )
            result = o3d.pipelines.registration.registration_generalized_icp(
                source=src,
                target=self._map_cloud,
                max_correspondence_distance=self._max_corr,
                init=T_init,
                estimation_method=(
                    o3d.pipelines.registration
                    .TransformationEstimationForGeneralizedICP()
                ),
            )
            self._match_details = {"rmse_m": float(result.inlier_rmse), "fitness": float(result.fitness),
                                   "correspondences": len(result.correspondence_set)}
            return float(result.inlier_rmse), np.asarray(result.transformation)

        # small_gicp backend
        import small_gicp
        src_raw = small_gicp.PointCloud(scan_pts)
        src, _ = small_gicp.preprocess_points(
            src_raw, downsampling_resolution=self._scan_voxel
        )
        map_cloud, map_tree = self._map_cloud
        result = small_gicp.align(
            map_cloud,
            src,
            map_tree,
            init_T_target_source=T_init,
            registration_type="GICP",
            max_correspondence_distance=self._max_corr,
        )
        # small_gicp.error is an optimizer objective, not a metric RMSE.
        from scipy.spatial import cKDTree
        T = np.asarray(result.T_target_source)
        points = np.asarray(src.points())[:, :3]
        target = np.asarray(map_cloud.points())[:, :3]
        distances, _ = cKDTree(target).query(points @ T[:3, :3].T + T[:3, 3])
        inliers = distances[distances <= self._max_corr]
        rmse = float(np.sqrt(np.mean(inliers ** 2))) if len(inliers) else float("inf")
        self._match_details = {"rmse_m": rmse, "fitness": len(inliers) / max(1, len(points)),
                               "correspondences": len(inliers)}
        return rmse, T

    def _broadcast(self):
        with self._lock:
            T = _planarize_map_to_odom(self._T_map_odom.copy())
        stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(
            _matrix_to_tf(T, stamp, self._map_frame, self._odom_frame)
        )


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GicpLocalizer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
