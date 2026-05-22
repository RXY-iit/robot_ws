#!/usr/bin/env python3
"""
lidar_tf_tuner.py — Interactive LiDAR TF calibration tool.

Loads a PLY map cloud and LiDAR frames from a ROS2 bag.  Lets you
adjust the base_link → livox_frame transform with live sliders and
see both clouds overlaid in a top-down view.

Usage:
    python3 tools/lidar_tf_tuner.py \\
        --map maps/saved-map/l402-points/l402_points \\
        --bag slam_bag/l402_fixed_livox_tf

Optional:
    --max-frames N       Number of LiDAR frames to accumulate (default 15)
    --subsample M        Keep 1-in-M map points for display (default 8)
    --img-size PX        Render image size in pixels (default 900)
"""
from __future__ import annotations

# Clear OpenCV's bundled Qt plugin path before importing cv2 or PyQt5,
# otherwise PyQt5 picks up cv2's incompatible xcb plugin and crashes.
import os as _os
_os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import argparse
import sys
from pathlib import Path
from typing import Optional

import numpy as np

# ROS2 bag reading (requires sourced ROS2 environment)
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# GUI — PyQt5 must be imported before cv2 so our env fix takes effect first
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QScrollArea, QSplitter, QVBoxLayout, QWidget,
)
import cv2

# ---------------------------------------------------------------------------
# Current URDF values for base_link → livox_frame
# (change these if you edit the URDF and want correct "delta" display)
# ---------------------------------------------------------------------------
URDF_TX, URDF_TY, URDF_TZ = -0.240, 0.000, 0.6875
URDF_ROLL, URDF_PITCH, URDF_YAW = 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------
# PLY loader (binary little-endian, handles arbitrary vertex properties)
# ---------------------------------------------------------------------------
_PLY_DTYPE = {
    "float": "<f4", "float32": "<f4",
    "double": "<f8", "float64": "<f8",
    "uchar": "u1",  "uint8":  "u1",
    "char":  "i1",
    "short": "<i2", "int16":  "<i2",
    "ushort":"<u2", "uint16": "<u2",
    "int":   "<i4", "int32":  "<i4",
    "uint":  "<u4", "uint32": "<u4",
}

def load_ply(path: Path) -> np.ndarray:
    """Return Nx3 float32 (x, y, z) from a binary PLY file."""
    path = Path(path)
    with open(path, "rb") as fh:
        props: list[tuple[str, str]] = []
        n_vertices = 0
        in_vertex_element = False

        while True:
            line = fh.readline().decode("ascii", errors="ignore").strip()
            if line == "end_header":
                break
            tokens = line.split()
            if tokens[:2] == ["element", "vertex"]:
                n_vertices = int(tokens[2])
                in_vertex_element = True
            elif tokens[0] == "element":
                in_vertex_element = False
            elif tokens[0] == "property" and in_vertex_element and len(tokens) >= 3:
                props.append((tokens[2], tokens[1]))   # (name, type)

        # Build numpy structured dtype
        np_dtype = [(name, _PLY_DTYPE.get(typ, "u1")) for name, typ in props]
        raw = np.frombuffer(fh.read(n_vertices * np.dtype(np_dtype).itemsize),
                            dtype=np.dtype(np_dtype))

    pts = np.stack([raw["x"].astype(np.float32),
                    raw["y"].astype(np.float32),
                    raw["z"].astype(np.float32)], axis=1)
    return pts


# ---------------------------------------------------------------------------
# ROS2 bag utilities
# ---------------------------------------------------------------------------
def _storage_options(uri: str) -> rosbag2_py.StorageOptions:
    return rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3")

def _converter_options() -> rosbag2_py.ConverterOptions:
    return rosbag2_py.ConverterOptions("cdr", "cdr")


def _extract_xyz_pc2(msg) -> np.ndarray:
    """Extract xyz from sensor_msgs/msg/PointCloud2 using field offsets."""
    n = msg.width * msg.height
    step = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    pts = np.zeros((n, 3), dtype=np.float32)
    for axis, name in enumerate(("x", "y", "z")):
        field = next((f for f in msg.fields if f.name == name), None)
        if field is None:
            continue
        off = field.offset
        # Slice every point_step bytes starting at field offset
        byte_indices = (np.arange(n, dtype=np.int64) * step + off)
        b = np.stack([raw[byte_indices + k] for k in range(4)], axis=1)
        pts[:, axis] = b.view(np.float32).reshape(-1)

    valid = np.all(np.isfinite(pts), axis=1) & (np.abs(pts).sum(axis=1) > 1e-6)
    return pts[valid]


class TFBuffer:
    """Minimal TF lookup built from bag /tf and /tf_static messages."""

    def __init__(self):
        # {(parent, child): [(stamp_ns, 4x4 matrix), ...]}
        self._store: dict[tuple[str, str], list] = {}

    def add(self, transform, static: bool = False):
        key = (transform.header.frame_id, transform.child_frame_id)
        t = transform.transform.translation
        r = transform.transform.rotation
        mat = _pose_to_mat(t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        stamp_ns = (transform.header.stamp.sec * 10**9
                    + transform.header.stamp.nanosec)
        if key not in self._store:
            self._store[key] = []
        self._store[key].append((stamp_ns, mat))
        if static and len(self._store[key]) == 1:
            # Keep a copy stamped at 0 for lookup at any time
            self._store[key].insert(0, (0, mat))

    def lookup(self, parent: str, child: str,
               stamp_ns: int) -> Optional[np.ndarray]:
        key = (parent, child)
        entries = self._store.get(key)
        if not entries:
            return None
        # Find closest by timestamp
        stamps = [e[0] for e in entries]
        idx = np.searchsorted(stamps, stamp_ns)
        idx = min(idx, len(entries) - 1)
        return entries[idx][1]

    def lookup_chain(self, frames: list[str],
                     stamp_ns: int) -> Optional[np.ndarray]:
        """Compose transforms along a chain [A, B, C, …] → A→…→last."""
        T = np.eye(4)
        for i in range(len(frames) - 1):
            m = self.lookup(frames[i], frames[i + 1], stamp_ns)
            if m is None:
                return None
            T = T @ m
        return T


def load_bag(bag_dir: str, max_frames: int) -> tuple[list[np.ndarray], TFBuffer]:
    """
    Read up to max_frames LiDAR point clouds and build a TF buffer
    from the bag at bag_dir.

    Returns (lidar_frames, tf_buffer).
    Each lidar frame is Nx3 float32 in livox_frame.
    """
    PC2 = get_message("sensor_msgs/msg/PointCloud2")
    TFMsg = get_message("tf2_msgs/msg/TFMessage")

    reader = rosbag2_py.SequentialReader()
    reader.open(_storage_options(bag_dir), _converter_options())

    tf_buf = TFBuffer()
    lidar_frames: list[np.ndarray] = []
    lidar_stamps: list[int] = []

    total_lidar = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()

        if topic == "/tf_static":
            msg = deserialize_message(data, TFMsg)
            for tf in msg.transforms:
                tf_buf.add(tf, static=True)

        elif topic == "/tf":
            msg = deserialize_message(data, TFMsg)
            for tf in msg.transforms:
                tf_buf.add(tf)

        elif topic == "/livox/lidar":
            total_lidar += 1
            # Subsample: take every (total/max_frames)-th frame
            lidar_frames.append(deserialize_message(data, PC2))
            lidar_stamps.append(stamp)

    # Down-select to max_frames evenly spaced
    n = len(lidar_frames)
    if n > max_frames:
        idxs = np.linspace(0, n - 1, max_frames, dtype=int)
        lidar_frames = [lidar_frames[i] for i in idxs]
        lidar_stamps = [lidar_stamps[i] for i in idxs]

    # Deserialise xyz (stored as raw msg above)
    clouds: list[np.ndarray] = []
    for msg in lidar_frames:
        pts = _extract_xyz_pc2(msg)
        clouds.append(pts)

    return clouds, lidar_stamps, tf_buf


# ---------------------------------------------------------------------------
# Transform helpers (pure numpy — no scipy dependency)
# ---------------------------------------------------------------------------
def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """Unit quaternion → 3×3 rotation matrix."""
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)

def _euler_to_rot(roll, pitch, yaw) -> np.ndarray:
    """Intrinsic XYZ Euler angles → 3×3 rotation matrix."""
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx

def _rot_to_euler(R: np.ndarray):
    """3×3 rotation matrix → (roll, pitch, yaw) intrinsic XYZ."""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if abs(np.cos(pitch)) < 1e-6:
        roll = np.arctan2(R[0, 1], R[1, 1])
        yaw  = 0.0
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw  = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw

def _pose_to_mat(tx, ty, tz, qx, qy, qz, qw) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = _quat_to_rot(qx, qy, qz, qw)
    mat[:3, 3] = [tx, ty, tz]
    return mat

def make_transform(tx, ty, tz, roll, pitch, yaw) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = _euler_to_rot(roll, pitch, yaw)
    mat[:3, 3] = [tx, ty, tz]
    return mat

def transform_points(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply 4×4 homogeneous transform to Nx3 array."""
    ones = np.ones((len(pts), 1), dtype=np.float32)
    ph = np.hstack([pts, ones])           # Nx4
    return (T @ ph.T).T[:, :3].astype(np.float32)


# ---------------------------------------------------------------------------
# Top-down renderer (OpenCV)
# ---------------------------------------------------------------------------
class Renderer:
    """Renders map + accumulated LiDAR clouds in a top-down (X-Y) view."""

    def __init__(self, img_size: int = 900):
        self.W = self.H = img_size
        self.view_cx = 0.0
        self.view_cy = 0.0
        self.scale = 50.0          # pixels per metre
        self._map_img: Optional[np.ndarray] = None

    def fit_to_map(self, map_pts: np.ndarray, subsample: int = 1):
        sub = map_pts[::subsample]
        cx = float(sub[:, 0].mean())
        cy = float(sub[:, 1].mean())
        self.view_cx = cx
        self.view_cy = cy
        span = max(sub[:, 0].max() - sub[:, 0].min(),
                   sub[:, 1].max() - sub[:, 1].min(), 1.0)
        self.scale = (self.W - 40) / span
        # Pre-render map layer
        img = np.full((self.H, self.W, 3), 30, dtype=np.uint8)
        px, py = self._to_px(sub[:, :2])
        mask = (px >= 0) & (px < self.W) & (py >= 0) & (py < self.H)
        img[py[mask], px[mask]] = (120, 120, 120)
        self._map_img = img

    def _to_px(self, xy: np.ndarray):
        px = ((xy[:, 0] - self.view_cx) * self.scale + self.W / 2).astype(int)
        py = (-(xy[:, 1] - self.view_cy) * self.scale + self.H / 2).astype(int)
        return px, py

    def render(self, lidar_clouds_map: list[np.ndarray]) -> np.ndarray:
        if self._map_img is None:
            return np.zeros((self.H, self.W, 3), dtype=np.uint8)
        img = self._map_img.copy()

        # Colour ramp per cloud: blue → red over time
        n = max(len(lidar_clouds_map), 1)
        for i, pts in enumerate(lidar_clouds_map):
            if len(pts) == 0:
                continue
            t = i / (n - 1) if n > 1 else 0.5
            colour = (int(255 * (1 - t)), 80, int(255 * t))  # BGR: red→blue
            sub = pts[::4]   # subsample for speed
            px, py = self._to_px(sub[:, :2])
            mask = (px >= 0) & (px < self.W) & (py >= 0) & (py < self.H)
            img[py[mask], px[mask]] = colour

        # Compass / scale bar
        cv2.putText(img, "N", (self.W // 2 - 8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 50), 1)
        bar_m = 1.0 if self.scale < 50 else 5.0
        bar_px = int(bar_m * self.scale)
        cv2.line(img, (20, self.H - 20), (20 + bar_px, self.H - 20),
                 (200, 200, 50), 2)
        cv2.putText(img, f"{bar_m:.0f} m", (22, self.H - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 50), 1)
        return img


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------
class TunerWindow(QMainWindow):

    def __init__(self, map_pts: np.ndarray,
                 clouds: list[np.ndarray],
                 stamps: list[int],
                 tf_buf: TFBuffer,
                 subsample: int,
                 img_size: int):
        super().__init__()
        self.setWindowTitle("LiDAR TF Tuner")

        self._map_pts = map_pts
        self._clouds_livox = clouds
        self._stamps = stamps
        self._tf_buf = tf_buf

        self._renderer = Renderer(img_size)
        self._renderer.fit_to_map(map_pts, subsample)

        # --- Build UI ---
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)

        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter)

        # Left: rendered image
        self._view_label = QLabel()
        self._view_label.setAlignment(Qt.AlignCenter)
        self._view_label.setMinimumSize(img_size, img_size)
        splitter.addWidget(self._view_label)

        # Right: controls in a scroll area
        ctrl_widget = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_widget)

        scroll = QScrollArea()
        scroll.setWidget(ctrl_widget)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(340)
        splitter.addWidget(scroll)

        # --- TF delta group ---
        tf_group = QGroupBox("Δ  base_link → livox_frame  (delta from URDF)")
        tf_form = QFormLayout(tf_group)
        self._spins: dict[str, QDoubleSpinBox] = {}
        params = [
            ("dx [m]",    "dx",     -0.5,  0.5,  0.001),
            ("dy [m]",    "dy",     -0.5,  0.5,  0.001),
            ("dz [m]",    "dz",     -0.5,  0.5,  0.001),
            ("d_roll°",   "droll",  -45.0, 45.0, 0.1),
            ("d_pitch°",  "dpitch", -45.0, 45.0, 0.1),
            ("d_yaw°",    "dyaw",   -45.0, 45.0, 0.1),
        ]
        for label, key, lo, hi, step in params:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setSingleStep(step)
            sb.setDecimals(3)
            sb.setValue(0.0)
            sb.valueChanged.connect(self._on_param_changed)
            self._spins[key] = sb
            tf_form.addRow(label, sb)
        ctrl_layout.addWidget(tf_group)

        # --- Initial pose group ---
        pose_group = QGroupBox("Extra pose offset  (added on top of bag map→odom TF)")
        pose_form = QFormLayout(pose_group)
        pose_params = [
            ("pose_x [m]",   "pose_x",   -50.0, 50.0, 0.05),
            ("pose_y [m]",   "pose_y",   -50.0, 50.0, 0.05),
            ("pose_yaw°",    "pose_yaw", -180.0, 180.0, 0.5),
        ]
        for label, key, lo, hi, step in pose_params:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setSingleStep(step)
            sb.setDecimals(3)
            sb.setValue(0.0)
            sb.valueChanged.connect(self._on_param_changed)
            self._spins[key] = sb
            pose_form.addRow(label, sb)
        ctrl_layout.addWidget(pose_group)

        # --- Current URDF values display ---
        info_group = QGroupBox("Current URDF TF (base_link → livox_frame)")
        info_layout = QVBoxLayout(info_group)
        self._info_label = QLabel()
        self._info_label.setWordWrap(True)
        info_layout.addWidget(self._info_label)
        ctrl_layout.addWidget(info_group)

        # --- Result display ---
        result_group = QGroupBox("Adjusted TF (URDF + delta)")
        result_layout = QVBoxLayout(result_group)
        self._result_label = QLabel()
        self._result_label.setWordWrap(True)
        result_layout.addWidget(self._result_label)
        ctrl_layout.addWidget(result_group)

        # --- Buttons ---
        btn_layout = QHBoxLayout()
        print_btn = QPushButton("Print TF to terminal")
        print_btn.clicked.connect(self._print_tf)
        btn_layout.addWidget(print_btn)

        reset_btn = QPushButton("Reset all")
        reset_btn.clicked.connect(self._reset)
        btn_layout.addWidget(reset_btn)
        ctrl_layout.addLayout(btn_layout)

        ctrl_layout.addStretch()

        self._update_info_labels()
        self._refresh()

    # ------------------------------------------------------------------
    def _get_adjusted_tf(self) -> tuple[float, float, float, float, float, float]:
        dx    = self._spins["dx"].value()
        dy    = self._spins["dy"].value()
        dz    = self._spins["dz"].value()
        droll = np.deg2rad(self._spins["droll"].value())
        dpitch= np.deg2rad(self._spins["dpitch"].value())
        dyaw  = np.deg2rad(self._spins["dyaw"].value())

        tx = URDF_TX + dx
        ty = URDF_TY + dy
        tz = URDF_TZ + dz
        roll  = URDF_ROLL  + droll
        pitch = URDF_PITCH + dpitch
        yaw   = URDF_YAW   + dyaw
        return tx, ty, tz, roll, pitch, yaw

    def _build_lidar_clouds_in_map(self) -> list[np.ndarray]:
        tx, ty, tz, roll, pitch, yaw = self._get_adjusted_tf()
        T_base_livox = make_transform(tx, ty, tz, roll, pitch, yaw)

        # Optional extra pose offset on top of the bag's map→odom
        px  = self._spins["pose_x"].value()
        py  = self._spins["pose_y"].value()
        pyaw = np.deg2rad(self._spins["pose_yaw"].value())
        T_extra = make_transform(px, py, 0.0, 0.0, 0.0, pyaw)

        # Static fallbacks (from URDF / known geometry)
        T_bfp_base_fallback = make_transform(0.0, 0.0, 0.1125, 0.0, 0.0, 0.0)

        out = []
        for pts_livox, stamp in zip(self._clouds_livox, self._stamps):
            # map → odom  (static identity in this bag, or AMCL dynamic)
            T_map_odom = self._tf_buf.lookup("map", "odom", stamp)
            if T_map_odom is None:
                T_map_odom = np.eye(4)

            # odom → base_footprint  (wheel odometry)
            T_odom_bfp = self._tf_buf.lookup("odom", "base_footprint", stamp)
            if T_odom_bfp is None:
                T_odom_bfp = np.eye(4)

            # base_footprint → base_link  (from tf_static in bag)
            T_bfp_base = self._tf_buf.lookup("base_footprint", "base_link", stamp)
            if T_bfp_base is None:
                T_bfp_base = T_bfp_base_fallback

            # Full chain: map ← odom ← base_footprint ← base_link ← livox_frame
            T = T_extra @ T_map_odom @ T_odom_bfp @ T_bfp_base @ T_base_livox
            out.append(transform_points(pts_livox, T))
        return out

    def _refresh(self):
        clouds_map = self._build_lidar_clouds_in_map()
        img_bgr = self._renderer.render(clouds_map)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = img_rgb.shape
        qt_img = QImage(img_rgb.data, w, h, ch * w, QImage.Format_RGB888)
        self._view_label.setPixmap(QPixmap.fromImage(qt_img))
        self._update_result_label()

    def _update_info_labels(self):
        self._info_label.setText(
            f"xyz:  ({URDF_TX:.4f},  {URDF_TY:.4f},  {URDF_TZ:.4f}) m\n"
            f"rpy:  ({np.rad2deg(URDF_ROLL):.3f}°, "
            f"{np.rad2deg(URDF_PITCH):.3f}°, "
            f"{np.rad2deg(URDF_YAW):.3f}°)"
        )

    def _update_result_label(self):
        tx, ty, tz, r, p, y = self._get_adjusted_tf()
        self._result_label.setText(
            f"xyz:  ({tx:.4f},  {ty:.4f},  {tz:.4f}) m\n"
            f"rpy:  ({np.rad2deg(r):.3f}°, {np.rad2deg(p):.3f}°, {np.rad2deg(y):.3f}°)\n\n"
            f"URDF origin xyz:\n"
            f'  xyz="{tx:.4f} {ty:.4f} {tz:.4f}"\n'
            f'  rpy="{np.rad2deg(r):.4f} {np.rad2deg(p):.4f} {np.rad2deg(y):.4f}"'
        )

    def _on_param_changed(self):
        self._refresh()

    def _print_tf(self):
        tx, ty, tz, r, p, y = self._get_adjusted_tf()
        print("\n" + "=" * 60)
        print("Adjusted base_link → livox_frame TF")
        print("=" * 60)
        print(f"  Translation:  x={tx:.4f}  y={ty:.4f}  z={tz:.4f}  [m]")
        print(f"  Rotation RPY: roll={np.rad2deg(r):.4f}°  "
              f"pitch={np.rad2deg(p):.4f}°  yaw={np.rad2deg(y):.4f}°")
        print()
        print("URDF joint origin tag:")
        print(f'  <origin xyz="{tx:.4f} {ty:.4f} {tz:.4f}"'
              f' rpy="{r:.6f} {p:.6f} {y:.6f}"/>')
        print()
        print("robot.urdf.xacro properties to set:")
        print(f"  <xacro:property name=\"livox_x\" value=\"{tx:.4f}\"/>")
        print(f"  <xacro:property name=\"livox_y\" value=\"{ty:.4f}\"/>")
        print(f"  <xacro:property name=\"livox_z\" value=\"{tz:.4f}\"/>")
        rpy_str = f"{r:.6f} {p:.6f} {y:.6f}"
        print(f"  (add livox_roll/pitch/yaw properties if non-zero rotation)")
        print("=" * 60)

    def _reset(self):
        for sb in self._spins.values():
            sb.blockSignals(True)
            sb.setValue(0.0)
            sb.blockSignals(False)
        self._refresh()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", required=True,
                        help="Path to PLY map file")
    parser.add_argument("--bag", required=True,
                        help="Path to ROS2 bag directory")
    parser.add_argument("--max-frames", type=int, default=15,
                        help="Max LiDAR frames to load (default 15)")
    parser.add_argument("--subsample", type=int, default=8,
                        help="Map display subsample factor (default 8)")
    parser.add_argument("--img-size", type=int, default=900,
                        help="Render image size in pixels (default 900)")
    args = parser.parse_args()

    map_path = Path(args.map)
    bag_path = Path(args.bag)

    if not map_path.exists():
        print(f"ERROR: map file not found: {map_path}", file=sys.stderr)
        sys.exit(1)
    if not bag_path.is_dir():
        print(f"ERROR: bag directory not found: {bag_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading map from: {map_path}")
    map_pts = load_ply(map_path)
    print(f"  Loaded {len(map_pts):,} map points")
    print(f"  XY extent: x=[{map_pts[:,0].min():.2f}, {map_pts[:,0].max():.2f}]"
          f"  y=[{map_pts[:,1].min():.2f}, {map_pts[:,1].max():.2f}]")

    print(f"\nLoading bag: {bag_path}")
    clouds, stamps, tf_buf = load_bag(str(bag_path), args.max_frames)
    print(f"  Loaded {len(clouds)} LiDAR frames")
    for key in tf_buf._store:
        print(f"  TF edge: {key[0]} → {key[1]}  "
              f"({len(tf_buf._store[key])} entries)")

    app = QApplication(sys.argv)
    win = TunerWindow(map_pts, clouds, stamps, tf_buf,
                      args.subsample, args.img_size)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
