#!/usr/bin/python3
"""
gicp_localizer_node.py — GICP prior-map localization for ROS 2 Humble.

Replaces GLIM localization with deterministic scan-to-map matching.

Input topics:
  /livox/lidar       sensor_msgs/PointCloud2   (Livox xfer_format=0, XYZRTL)
  /initialpose       geometry_msgs/PoseWithCovarianceStamped  (RViz 2D Pose Estimate)
  /fast_lio/odometry nav_msgs/Odometry         (optional, Phase 2 FAST-LIO2 hint)

Output:
  /gicp_loc/pose     geometry_msgs/PoseStamped  (corrected sensor pose in map)
  /gicp_loc/score    std_msgs/Float32           (GICP inlier RMSE — lower is better)
  TF broadcast:      map → odom

TF chain after this node:
  map → odom  (this node)
  odom → base_footprint  (wheel odom, robot_odom_node — unchanged)
  base_footprint → base_link → livox_frame  (URDF)

Algorithm:
  1. Get T_odom_lidar from TF lookup (odom → livox_frame)
  2. Compute initial guess: T_map_lidar = T_map_odom (current) × T_odom_lidar
  3. Voxel-downsample incoming scan, estimate normals
  4. GICP: match scan against preloaded PLY map (fixed, never modified)
  5. Compute T_map_odom = T_map_lidar_corrected × inv(T_odom_lidar)
  6. If fitness score < threshold: update and broadcast map→odom TF
     else: keep previous TF, log warning

Backend (auto-detected at startup):
  open3d  — /usr/bin/python3 -m pip install --user open3d
  small_gicp — /usr/bin/python3 -m pip install --user small-gicp  (fallback)
"""

import os
import sys
import threading

import numpy as np
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
from std_msgs.msg import Float32
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
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("lidar_frame", "livox_frame")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("use_fast_lio_hint", False)
        self.declare_parameter("fast_lio_odom_topic", "/fast_lio/odometry")

        p = self.get_parameter
        self._map_path   = p("map_path").value
        self._map_voxel  = p("map_voxel_size").value
        self._scan_voxel = p("scan_voxel_size").value
        self._max_corr   = p("max_correspondence_distance").value
        self._fit_thr    = p("fitness_score_threshold").value
        self._normal_r   = p("normal_search_radius").value
        self._period     = p("localize_period").value
        self._tf_period  = p("tf_broadcast_period").value
        self._map_frame  = p("map_frame").value
        self._odom_frame = p("odom_frame").value
        self._base_frame = p("base_frame").value
        self._lidar_frame = p("lidar_frame").value

        # Initial T_map_odom from 2-D startup pose params
        x   = p("initial_x").value
        y   = p("initial_y").value
        yaw = p("initial_yaw").value
        T0 = np.eye(4)
        T0[:3, :3] = _yaw_to_matrix(yaw)
        T0[0, 3] = x
        T0[1, 3] = y
        self._T_map_odom: np.ndarray = T0
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
        self._score_pub = self.create_publisher(Float32,      "/gicp_loc/score", 10)

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
            PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10
        )

        # FAST-LIO2 hint state (all under self._lock)
        self._fast_lio_T: np.ndarray | None = None               # current FAST-LIO pose
        self._fast_lio_T_at_correction: np.ndarray | None = None # snapshot at last GICP accept
        self._T_map_lidar_at_correction: np.ndarray | None = None # T_map_lidar at last accept
        if p("use_fast_lio_hint").value:
            topic = p("fast_lio_odom_topic").value
            self.create_subscription(Odometry, topic, self._fast_lio_cb, sensor_qos)
            self.get_logger().info(f"[gicp_localizer] FAST-LIO2 hint enabled: {topic}")

        # latest scan buffer (consumed each localize cycle)
        self._latest_scan: np.ndarray | None = None

        # ── load map ──────────────────────────────────────────────────────────
        self._map_cloud = self._load_map(self._map_path)

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
            f"map_voxel={self._map_voxel}m  scan_voxel={self._scan_voxel}m"
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

    def _scan_cb(self, msg: PointCloud2):
        arr = _pc2_to_numpy(msg)
        if arr.shape[0] > 0:
            self._latest_scan = arr

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
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
                "using clicked pose as map->odom fallback"
            )
            T_map_odom = T_map_base

        with self._lock:
            self._T_map_odom = T_map_odom
        self._latest_scan = None
        self._broadcast()
        self.get_logger().info(
            f"[gicp_localizer] /initialpose reset {self._map_frame}->{self._odom_frame}: "
            f"robot x={p.x:.2f} y={p.y:.2f}"
        )

    def _fast_lio_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [p.x, p.y, p.z]
        with self._lock:
            self._fast_lio_T = T

    # ── localization timer ────────────────────────────────────────────────────

    def _localize(self):
        if self._map_cloud is None or self._latest_scan is None:
            return

        scan_pts = self._latest_scan
        self._latest_scan = None

        # Lookup odom → livox_frame
        try:
            tf_stamped = self._tf_buf.lookup_transform(
                self._odom_frame,
                self._lidar_frame,
                rclpy.time.Time(),
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
            T_flio_cur = self._fast_lio_T.copy() if self._fast_lio_T is not None else None
            T_flio_ref = self._fast_lio_T_at_correction.copy() if self._fast_lio_T_at_correction is not None else None
            T_map_lidar_ref = self._T_map_lidar_at_correction.copy() if self._T_map_lidar_at_correction is not None else None

        # Initial guess: prefer FAST-LIO delta hint when available
        if T_flio_cur is not None and T_flio_ref is not None and T_map_lidar_ref is not None:
            # delta = motion since last GICP correction in FAST-LIO odom frame
            delta = np.linalg.inv(T_flio_ref) @ T_flio_cur
            T_init = T_map_lidar_ref @ delta
        else:
            T_init = T_map_odom @ T_odom_lidar

        # Run GICP
        score, T_map_lidar = self._run_gicp(scan_pts, T_init)

        # Publish score
        score_msg = Float32()
        score_msg.data = float(score)
        self._score_pub.publish(score_msg)

        if score > self._fit_thr:
            self.get_logger().warn(
                f"[gicp_localizer] score {score:.4f} > thr {self._fit_thr} — keeping previous TF",
                throttle_duration_sec=3.0,
            )
            return

        # T_map_odom = T_map_lidar × inv(T_odom_lidar)
        T_map_odom_new = T_map_lidar @ np.linalg.inv(T_odom_lidar)

        with self._lock:
            self._T_map_odom = T_map_odom_new
            # Save reference snapshots for next FAST-LIO delta hint
            if T_flio_cur is not None:
                self._fast_lio_T_at_correction = T_flio_cur
                self._T_map_lidar_at_correction = T_map_lidar.copy()

        stamp = self.get_clock().now().to_msg()
        self._pose_pub.publish(_matrix_to_pose(T_map_lidar, stamp, self._map_frame))

    def _run_gicp(
        self, scan_pts: np.ndarray, T_init: np.ndarray
    ) -> tuple[float, np.ndarray]:
        """Return (fitness_score, T_map_lidar_corrected)."""
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
            return float(result.inlier_rmse), np.asarray(result.transformation)

        # small_gicp backend
        import small_gicp
        src_raw = small_gicp.PointCloud(scan_pts)
        src, _ = small_gicp.preprocess_points(
            src_raw, downsampling_resolution=self._scan_voxel
        )
        map_cloud, map_tree = self._map_cloud
        result = small_gicp.align(
            src,
            map_cloud,
            map_tree,
            initial_guess=T_init,
            registration_type="GICP",
            max_correspondence_distance=self._max_corr,
        )
        return float(result.error), np.asarray(result.T_target_source)

    def _broadcast(self):
        with self._lock:
            T = self._T_map_odom.copy()
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
