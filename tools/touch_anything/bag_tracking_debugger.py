#!/usr/bin/env python3
"""Interactive, read-only Touch Anything tracking debugger for rosbag2.

The tool replays the production ContinuousTargetTracker on every recorded Arm
Camera frame, then presents a seekable timeline with tracker health, action
windows, Touch state, online tracker/control telemetry (when recorded), and
per-frame geometry.  It never creates a ROS publisher or action client.
"""
from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
import importlib.util
import json
import math
import os
from pathlib import Path
import sqlite3
import sys
from typing import Any, Callable

# OpenCV wheels sometimes export a Qt plugin path incompatible with PyQt5.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import cv2
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
ARM_IMAGE_TOPIC = "/arm_camera/color/image_raw"
TRACKER_DEBUG_TOPIC = "/touch_anything/tracker_debug"
CONTROL_DEBUG_TOPIC = "/touch_anything/control_debug"


def _load_replay_module():
    path = Path(__file__).with_name("replay_continuous_tracker_bag.py")
    spec = importlib.util.spec_from_file_location("touch_tracking_replay", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


REPLAY = _load_replay_module()


@dataclass
class FrameAnalysis:
    row_id: int | None
    bag_stamp: float
    image_stamp: float
    result: dict[str, Any]
    contours: list[list[list[int]]]
    initial_rgb: np.ndarray | None = None


@dataclass
class DebugEvent:
    frame_index: int
    image_stamp: float
    kind: str
    label: str
    severity: str = "info"


class BagDatabase:
    """Small read-only sqlite index with random access to raw image messages."""

    def __init__(self, bag: Path) -> None:
        self.bag = bag.resolve()
        metadata = yaml.safe_load(
            (self.bag / "metadata.yaml").read_text(encoding="utf-8")
        )["rosbag2_bagfile_information"]
        paths = metadata.get("relative_file_paths", [])
        if len(paths) != 1 or not str(paths[0]).endswith(".db3"):
            raise RuntimeError(
                "debugger currently requires one sqlite3 .db3 file in the bag"
            )
        self.database_path = (self.bag / paths[0]).resolve()
        self.connection = sqlite3.connect(
            f"file:{self.database_path}?mode=ro", uri=True
        )
        self.connection.execute("PRAGMA query_only=ON")
        self.topics = {
            str(name): (int(identifier), str(message_type))
            for identifier, name, message_type in self.connection.execute(
                "SELECT id, name, type FROM topics"
            )
        }

    def close(self) -> None:
        self.connection.close()

    def rows(self, topic: str, *, include_data: bool = True):
        if topic not in self.topics:
            return iter(())
        topic_id = self.topics[topic][0]
        columns = "id, timestamp, data" if include_data else "id, timestamp"
        return self.connection.execute(
            f"SELECT {columns} FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,),
        )

    def message_count(self, topic: str) -> int:
        if topic not in self.topics:
            return 0
        row = self.connection.execute(
            "SELECT COUNT(*) FROM messages WHERE topic_id=?",
            (self.topics[topic][0],),
        ).fetchone()
        return 0 if row is None else int(row[0])

    def message_data(self, row_id: int) -> bytes:
        row = self.connection.execute(
            "SELECT data FROM messages WHERE id=?", (int(row_id),)
        ).fetchone()
        if row is None:
            raise KeyError(f"rosbag message row {row_id} disappeared")
        return bytes(row[0])

    def read_image(self, row_id: int) -> np.ndarray:
        return REPLAY.image_rgb(deserialize_message(self.message_data(row_id), Image))

    def json_series(self, topic: str) -> list[tuple[float, dict[str, Any]]]:
        output: list[tuple[float, dict[str, Any]]] = []
        for _, record_ns, data in self.rows(topic):
            try:
                payload = json.loads(deserialize_message(data, String).data)
            except (json.JSONDecodeError, TypeError, ValueError):
                continue
            if isinstance(payload, dict):
                output.append((float(record_ns) / 1.0e9, payload))
        return output

    def joint_series(self) -> list[tuple[float, dict[str, float]]]:
        output: list[tuple[float, dict[str, float]]] = []
        for _, record_ns, data in self.rows("/joint_states"):
            message = deserialize_message(data, JointState)
            output.append((
                float(record_ns) / 1.0e9,
                {
                    str(name): float(position)
                    for name, position in zip(message.name, message.position)
                },
            ))
        return output

    def controller_series(self) -> list[tuple[float, dict[str, Any]]]:
        output: list[tuple[float, dict[str, Any]]] = []
        for _, record_ns, data in self.rows("/arm_controller/controller_state"):
            message = deserialize_message(data, JointTrajectoryControllerState)
            names = [str(name) for name in message.joint_names]

            def positions(point) -> dict[str, float]:
                return {
                    name: float(value)
                    for name, value in zip(names, point.positions)
                }

            output.append((float(record_ns) / 1.0e9, {
                "desired_rad": positions(message.desired),
                "actual_rad": positions(message.actual),
                "error_rad": positions(message.error),
            }))
        return output


def load_shadow_control_series(
    bag: Path, record_path: Path
) -> list[tuple[float, dict[str, Any]]]:
    """Load the matching offline shadow report when one already exists."""
    wanted = str(record_path.resolve())
    for path in sorted(bag.glob("tracking_control_cycle_replay_*.json")):
        try:
            report = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if str(Path(report.get("initial_human_confirmation", "")).resolve()) != wanted:
            continue
        return [
            (float(item["tick"]), item)
            for item in report.get("shadow_control", {}).get("trace", [])
            if item.get("tick") is not None
        ]
    return []


def previous_sample(
    series: list[tuple[float, Any]], stamp: float
) -> tuple[float, Any] | None:
    if not series:
        return None
    stamps = [item[0] for item in series]
    index = bisect.bisect_right(stamps, stamp) - 1
    return None if index < 0 else series[index]


def _mask_contours(mask: np.ndarray | None) -> list[list[list[int]]]:
    if mask is None:
        return []
    contours, _ = cv2.findContours(
        (np.asarray(mask) > 0).astype(np.uint8),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    return [
        np.asarray(contour, dtype=np.int32).reshape(-1, 2).tolist()
        for contour in contours
        if len(contour) >= 3
    ]


def build_events(frames: list[FrameAnalysis]) -> list[DebugEvent]:
    events: list[DebugEvent] = []
    previous_status = None
    previous_method = None
    previous_stamp = None
    low_health_latched = False
    for index, frame in enumerate(frames):
        result = frame.result
        status = str(result.get("status", "UNKNOWN"))
        method = str(result.get("method", "UNKNOWN"))
        if index == 0:
            events.append(DebugEvent(index, frame.image_stamp, "START", "tracking start"))
        if previous_stamp is not None:
            gap = frame.image_stamp - previous_stamp
            if gap > 0.10:
                events.append(DebugEvent(
                    index, frame.image_stamp, "FRAME_GAP",
                    f"camera/tracker frame gap {gap * 1000.0:.1f} ms", "warning",
                ))
        normal_initial_handoff = (
            previous_status == "INITIALIZED" and status == "TRACKING"
        )
        if (
            status != previous_status
            and previous_status is not None
            and not normal_initial_handoff
        ):
            severity = "error" if status == "LOST" else "warning"
            events.append(DebugEvent(
                index, frame.image_stamp, "STATUS",
                f"tracker {previous_status} -> {status}: {result.get('reason', '')}",
                severity,
            ))
        if (
            method != previous_method
            and previous_method is not None
            and not normal_initial_handoff
        ):
            events.append(DebugEvent(
                index, frame.image_stamp, "METHOD",
                f"method {previous_method} -> {method}", "warning",
            ))
        points = int(result.get("tracked_points") or 0)
        inlier = float(result.get("inlier_ratio") or 0.0)
        low_health = status not in {"INITIALIZED", "TRACKING", "RECOVERED"} or (
            status != "INITIALIZED" and (points < 10 or inlier < 0.60)
        )
        if low_health and not low_health_latched:
            events.append(DebugEvent(
                index, frame.image_stamp, "LOW_HEALTH",
                f"low tracker health: points={points}, inlier={inlier:.3f}",
                "warning",
            ))
        low_health_latched = low_health
        previous_status = status
        previous_method = method
        previous_stamp = frame.image_stamp
    return events


def build_recorded_context_events(
    database: BagDatabase, frames: list[FrameAnalysis]
) -> list[DebugEvent]:
    """Align action/Touch/online-tracker transitions to the nearest video frame."""
    if not frames:
        return []
    bag_stamps = [frame.bag_stamp for frame in frames]
    minimum_stamp, maximum_stamp = bag_stamps[0], bag_stamps[-1]

    def frame_index(stamp: float) -> int:
        return min(bisect.bisect_left(bag_stamps, stamp), len(frames) - 1)

    events: list[DebugEvent] = []
    previous_execution: tuple[str, str] | None = None
    for stamp, payload in database.json_series("/arm/dashboard_state"):
        if stamp < minimum_stamp or stamp > maximum_stamp:
            continue
        execution = payload.get("execution") or {}
        state = str(execution.get("state", ""))
        request_id = str(execution.get("last_request_id", ""))
        current = (request_id, state)
        if request_id.startswith("touch-") and current != previous_execution:
            index = frame_index(stamp)
            events.append(DebugEvent(
                index, frames[index].image_stamp, "ACTION",
                f"{state or '--'} {request_id}",
                "warning" if state in {"FAILED", "CANCELLED", "BLOCKED"} else "info",
            ))
        previous_execution = current

    previous_state = None
    for stamp, payload in database.json_series("/touch_anything/state"):
        if stamp < minimum_stamp or stamp > maximum_stamp:
            continue
        state = str(payload.get("state", ""))
        if state and state != previous_state:
            index = frame_index(stamp)
            events.append(DebugEvent(
                index, frames[index].image_stamp, "TOUCH_STATE", state,
                "warning" if any(word in state for word in ("LOST", "FAILED", "BLOCKED")) else "info",
            ))
        previous_state = state

    previous_online = None
    for stamp, payload in database.json_series(TRACKER_DEBUG_TOPIC):
        if stamp < minimum_stamp or stamp > maximum_stamp:
            continue
        result = payload.get("result") or {}
        current = (str(result.get("status", "")), str(result.get("method", "")))
        if previous_online is not None and current != previous_online:
            index = frame_index(stamp)
            events.append(DebugEvent(
                index, frames[index].image_stamp, "ONLINE_TRACKER",
                f"online {previous_online[0]}/{previous_online[1]} -> "
                f"{current[0]}/{current[1]}",
                "error" if current[0] in {"LOST", "RUNTIME_ERROR"} else "warning",
            ))
        previous_online = current
    return events


def analyze_bag(
    database: BagDatabase,
    record_path: Path,
    *,
    duration: float = 0.0,
    progress: Callable[[int, int], None] | None = None,
) -> tuple[list[FrameAnalysis], list[DebugEvent], dict[str, Any]]:
    record, initial_rgb, initial_mask = REPLAY.load_initial(record_path)
    tracker_module = REPLAY.load_tracker_module()
    tracker = tracker_module.ContinuousTargetTracker()
    height, width = initial_rgb.shape[:2]
    fixed_tip = (0.496 * (width - 1), 0.735 * (height - 1))
    initial = tracker.initialize(initial_rgb, initial_mask, excluded_tip_uv=fixed_tip)
    start = float(record["image_stamp"])
    end = math.inf if duration <= 0.0 else start + duration
    frames = [FrameAnalysis(
        row_id=None,
        bag_stamp=start,
        image_stamp=start,
        result=initial,
        contours=_mask_contours(initial_mask),
        initial_rgb=initial_rgb,
    )]
    total = database.message_count(ARM_IMAGE_TOPIC)
    for ordinal, (row_id, record_ns, data) in enumerate(
        database.rows(ARM_IMAGE_TOPIC)
    ):
        message = deserialize_message(data, Image)
        stamp = REPLAY.image_stamp(message)
        if stamp <= start:
            continue
        if stamp > end:
            break
        rgb = REPLAY.image_rgb(message)
        try:
            result = tracker.update(rgb, excluded_tip_uv=fixed_tip)
        except Exception as exc:
            result = {
                "status": "RUNTIME_ERROR",
                "method": "NONE",
                "reason": f"{type(exc).__name__}: {exc}",
                "frame_index": len(frames),
                "centroid_uv": None,
                "mask_pixels": 0,
                "mask_fraction": 0.0,
                "tracked_points": 0,
                "inlier_ratio": 0.0,
                "processing_ms": None,
                "feature_points_uv": [],
                "previous_feature_points_uv": [],
                "affine_previous_to_current": None,
            }
        frames.append(FrameAnalysis(
            row_id=int(row_id),
            bag_stamp=float(record_ns) / 1.0e9,
            image_stamp=stamp,
            result=result,
            contours=_mask_contours(tracker.current_mask),
        ))
        if progress is not None and (len(frames) % 100 == 0 or ordinal + 1 == total):
            progress(ordinal + 1, total)
    events = build_events(frames)
    events.extend(build_recorded_context_events(database, frames))
    events.sort(key=lambda item: (item.frame_index, item.kind, item.label))
    summary = {
        "bag": str(database.bag),
        "initial_record": str(record_path),
        "frames": len(frames),
        "duration_sec": frames[-1].image_stamp - frames[0].image_stamp,
        "statuses": {},
        "methods": {},
        "event_counts": {},
        "first_loss": None,
        "online_tracker_telemetry_present": TRACKER_DEBUG_TOPIC in database.topics,
        "online_control_telemetry_present": CONTROL_DEBUG_TOPIC in database.topics,
    }
    for frame in frames:
        for field, key in (("status", "statuses"), ("method", "methods")):
            value = str(frame.result.get(field, "UNKNOWN"))
            summary[key][value] = summary[key].get(value, 0) + 1
        if summary["first_loss"] is None and frame.result.get("status") in {
            "LOST", "RUNTIME_ERROR"
        }:
            summary["first_loss"] = {
                "frame_index": int(frame.result.get("frame_index", 0)),
                "image_stamp": frame.image_stamp,
                "relative_sec": frame.image_stamp - start,
                "reason": frame.result.get("reason", ""),
            }
    for event in events:
        summary["event_counts"][event.kind] = (
            summary["event_counts"].get(event.kind, 0) + 1
        )
    return frames, events, summary


def annotate_frame(
    rgb: np.ndarray,
    frame: FrameAnalysis,
    *,
    action_text: str = "",
    online_result: dict[str, Any] | None = None,
) -> np.ndarray:
    canvas = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    tint = canvas.copy()
    contours = [np.asarray(item, dtype=np.int32).reshape(-1, 1, 2) for item in frame.contours]
    if contours:
        cv2.drawContours(tint, contours, -1, (0, 170, 0), thickness=-1)
        canvas = cv2.addWeighted(tint, 0.22, canvas, 0.78, 0.0)
        cv2.drawContours(canvas, contours, -1, (0, 255, 0), thickness=2)
    result = frame.result
    previous = result.get("previous_feature_points_uv") or []
    current = result.get("feature_points_uv") or []
    for old, new in zip(previous, current):
        p0 = tuple(int(round(value)) for value in old)
        p1 = tuple(int(round(value)) for value in new)
        cv2.line(canvas, p0, p1, (0, 180, 255), 1, cv2.LINE_AA)
    for point in current:
        p = tuple(int(round(value)) for value in point)
        cv2.circle(canvas, p, 2, (0, 255, 0), -1, cv2.LINE_AA)
    centroid = result.get("centroid_uv")
    if centroid is not None:
        p = tuple(int(round(value)) for value in centroid)
        cv2.drawMarker(canvas, p, (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
    height, width = canvas.shape[:2]
    cv2.drawMarker(
        canvas, (int(round(0.496 * (width - 1))), int(round(0.735 * (height - 1)))),
        (255, 255, 0), cv2.MARKER_TILTED_CROSS, 18, 2,
    )
    cv2.drawMarker(
        canvas, (int(round((width - 1) * 0.5)), int(round((height - 1) * 0.5))),
        (255, 0, 255), cv2.MARKER_CROSS, 14, 1,
    )
    status = str(result.get("status", "UNKNOWN"))
    color = (0, 210, 0) if status in {"INITIALIZED", "TRACKING", "RECOVERED"} else (0, 0, 255)
    lines = [
        f"OFFLINE {status} / {result.get('method', 'UNKNOWN')}",
        f"frame={result.get('frame_index')} points={result.get('tracked_points')} "
        f"inlier={float(result.get('inlier_ratio') or 0.0):.3f} "
        f"mask={float(result.get('mask_fraction') or 0.0) * 100.0:.2f}% "
        f"compute={result.get('processing_ms')} ms",
    ]
    if online_result is not None:
        online = online_result.get("result", online_result)
        lines.append(
            f"ONLINE {online.get('status', '--')} / {online.get('method', '--')} "
            f"points={online.get('tracked_points', '--')}"
        )
    if action_text:
        lines.append(action_text)
    if result.get("reason"):
        lines.append(str(result["reason"])[:110])
    banner_height = 8 + 24 * len(lines)
    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, 0), (width, banner_height), (15, 18, 20), -1)
    canvas = cv2.addWeighted(overlay, 0.82, canvas, 0.18, 0.0)
    for index, text in enumerate(lines):
        cv2.putText(
            canvas, text, (10, 22 + 24 * index), cv2.FONT_HERSHEY_SIMPLEX,
            0.52, color if index == 0 else (230, 230, 230), 1, cv2.LINE_AA,
        )
    return canvas


def launch_gui(
    database: BagDatabase,
    record_path: Path,
    frames: list[FrameAnalysis],
    events: list[DebugEvent],
) -> int:
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal
    from PyQt5.QtGui import QColor, QImage, QPainter, QPixmap
    from PyQt5.QtWidgets import (
        QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout,
        QLabel, QListWidget, QMainWindow, QPlainTextEdit, QPushButton,
        QScrollArea, QSplitter, QVBoxLayout, QWidget,
    )

    dashboard = database.json_series("/arm/dashboard_state")
    touch_state = database.json_series("/touch_anything/state")
    tracker_online = database.json_series(TRACKER_DEBUG_TOPIC)
    control_online = database.json_series(CONTROL_DEBUG_TOPIC)
    joints = database.joint_series()
    controller = database.controller_series()
    shadow_control = load_shadow_control_series(database.bag, record_path)
    camera_tf = REPLAY.load_arm_camera_tf_trace(database.bag)
    start_stamp = frames[0].image_stamp

    class Timeline(QWidget):
        selected = pyqtSignal(int)

        def __init__(self):
            super().__init__()
            self.current = 0
            self.setMinimumHeight(74)

        def paintEvent(self, _event):
            painter = QPainter(self)
            painter.fillRect(self.rect(), QColor(28, 31, 35))
            width = max(self.width() - 20, 1)
            y0, band_height = 20, 22
            count = max(len(frames) - 1, 1)
            for index, frame in enumerate(frames):
                x0 = 10 + int(index * width / count)
                x1 = 10 + int((index + 1) * width / count)
                status = frame.result.get("status")
                color = QColor(30, 180, 75)
                if status == "RECOVERED":
                    color = QColor(255, 175, 0)
                elif status not in {"INITIALIZED", "TRACKING"}:
                    color = QColor(220, 50, 47)
                painter.fillRect(x0, y0, max(1, x1 - x0), band_height, color)
            for event in events:
                if event.kind == "START":
                    continue
                x = 10 + int(event.frame_index * width / count)
                color = (
                    QColor(65, 145, 255)
                    if event.kind in {"ACTION", "TOUCH_STATE"}
                    else QColor(230, 60, 50)
                    if event.severity == "error"
                    else QColor(255, 190, 50)
                )
                painter.setPen(color)
                painter.drawLine(x, 8, x, 51)
            x = 10 + int(self.current * width / count)
            painter.setPen(QColor(80, 170, 255))
            painter.drawLine(x, 4, x, 62)
            painter.setPen(QColor(225, 225, 225))
            painter.drawText(10, 70, "green=tracking  amber=health  red=lost/error  blue=action/state/current")

        def mousePressEvent(self, event):
            ratio = min(max((event.x() - 10) / max(self.width() - 20, 1), 0.0), 1.0)
            self.selected.emit(int(round(ratio * (len(frames) - 1))))

    class Window(QMainWindow):
        def __init__(self):
            super().__init__()
            self.index = 0
            self.timer = QTimer(self)
            self.timer.timeout.connect(self._advance)
            self.setWindowTitle(f"Touch tracking bag debugger — {database.bag.name}")
            self.resize(1480, 920)

            self.image = QLabel(alignment=Qt.AlignCenter)
            self.image.setMinimumSize(700, 520)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(self.image)

            self.details = QPlainTextEdit()
            self.details.setReadOnly(True)
            self.details.setLineWrapMode(QPlainTextEdit.NoWrap)
            self.event_list = QListWidget()
            for event in events:
                relative = event.image_stamp - start_stamp
                self.event_list.addItem(
                    f"{relative:8.3f}s  [{event.kind}] {event.label}"
                )
            self.event_list.currentRowChanged.connect(self._jump_event)

            right = QSplitter(Qt.Vertical)
            right.addWidget(self.details)
            right.addWidget(self.event_list)
            right.setSizes([570, 270])
            body = QSplitter(Qt.Horizontal)
            body.addWidget(scroll)
            body.addWidget(right)
            body.setSizes([980, 480])

            self.play = QPushButton("Play")
            self.play.clicked.connect(self._toggle)
            previous = QPushButton("◀ frame")
            previous.clicked.connect(lambda: self.set_index(self.index - 1))
            following = QPushButton("frame ▶")
            following.clicked.connect(lambda: self.set_index(self.index + 1))
            self.speed = QComboBox()
            for value in (0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0):
                self.speed.addItem(f"{value:g}x", value)
            self.speed.setCurrentText("1x")
            self.position = QDoubleSpinBox()
            self.position.setDecimals(3)
            self.position.setRange(0.0, frames[-1].image_stamp - start_stamp)
            self.position.setSingleStep(0.067)
            go = QPushButton("Go to t")
            go.clicked.connect(self._go_time)
            self.break_lost = QCheckBox("break LOST/error")
            self.break_lost.setChecked(True)
            self.break_method = QCheckBox("break recovery/method")
            self.break_method.setChecked(True)
            self.break_health = QCheckBox("break gap/low health")
            self.break_health.setChecked(True)
            self.break_state = QCheckBox("break action/state")
            self.break_state.setChecked(True)

            controls = QHBoxLayout()
            for widget in (
                previous, self.play, following, QLabel("speed"), self.speed,
                QLabel("relative sec"), self.position, go,
                self.break_lost, self.break_method, self.break_health,
                self.break_state,
            ):
                controls.addWidget(widget)
            self.timeline = Timeline()
            self.timeline.selected.connect(self.set_index)
            layout = QVBoxLayout()
            layout.addLayout(controls)
            layout.addWidget(self.timeline)
            layout.addWidget(body)
            root = QWidget()
            root.setLayout(layout)
            self.setCentralWidget(root)
            self.set_index(0)

        def keyPressEvent(self, event):
            if event.key() == Qt.Key_Space:
                self._toggle()
            elif event.key() == Qt.Key_Left:
                self.set_index(self.index - 1)
            elif event.key() == Qt.Key_Right:
                self.set_index(self.index + 1)
            else:
                super().keyPressEvent(event)

        def _toggle(self):
            if self.timer.isActive():
                self.timer.stop()
                self.play.setText("Play")
            else:
                self.play.setText("Pause")
                self.timer.start(1)

        def _advance(self):
            if self.index >= len(frames) - 1:
                self._toggle()
                return
            next_index = self.index + 1
            event_kinds = {e.kind for e in events if e.frame_index == next_index}
            should_break = (
                self.break_lost.isChecked()
                and frames[next_index].result.get("status") in {"LOST", "RUNTIME_ERROR"}
            ) or (
                self.break_method.isChecked()
                and bool(event_kinds & {"STATUS", "METHOD"})
            ) or (
                self.break_health.isChecked()
                and bool(event_kinds & {"FRAME_GAP", "LOW_HEALTH"})
            ) or (
                self.break_state.isChecked()
                and bool(event_kinds & {"ACTION", "TOUCH_STATE"})
            )
            delta = max(
                frames[next_index].image_stamp - frames[self.index].image_stamp,
                0.001,
            )
            self.set_index(next_index)
            if should_break:
                self.timer.stop()
                self.play.setText("Play")
            else:
                speed = float(self.speed.currentData())
                self.timer.start(max(1, min(1000, int(delta * 1000.0 / speed))))

        def _go_time(self):
            stamp = start_stamp + self.position.value()
            stamps = [frame.image_stamp for frame in frames]
            self.set_index(min(bisect.bisect_left(stamps, stamp), len(frames) - 1))

        def _jump_event(self, row: int):
            if 0 <= row < len(events):
                self.set_index(events[row].frame_index)

        def set_index(self, index: int):
            self.index = min(max(int(index), 0), len(frames) - 1)
            frame = frames[self.index]
            rgb = (
                frame.initial_rgb.copy()
                if frame.row_id is None and frame.initial_rgb is not None
                else database.read_image(int(frame.row_id))
            )
            dashboard_sample = previous_sample(dashboard, frame.bag_stamp)
            touch_sample = previous_sample(touch_state, frame.bag_stamp)
            tracker_sample = previous_sample(tracker_online, frame.bag_stamp)
            control_sample = previous_sample(control_online, frame.bag_stamp)
            shadow_sample = previous_sample(shadow_control, frame.image_stamp)
            joint_sample = previous_sample(joints, frame.bag_stamp)
            controller_sample = previous_sample(controller, frame.bag_stamp)
            tf_sample = previous_sample(camera_tf, frame.image_stamp)
            execution = (
                dashboard_sample[1].get("execution", {}) if dashboard_sample else {}
            )
            action_text = (
                f"ACTION {execution.get('state', '--')} "
                f"{execution.get('last_request_id', '')}"
            )
            annotated = annotate_frame(
                rgb, frame, action_text=action_text,
                online_result=None if tracker_sample is None else tracker_sample[1],
            )
            height, width = annotated.shape[:2]
            qimage = QImage(
                annotated.data, width, height, int(annotated.strides[0]),
                QImage.Format_BGR888,
            ).copy()
            self.image.setPixmap(QPixmap.fromImage(qimage))
            self.position.blockSignals(True)
            self.position.setValue(frame.image_stamp - start_stamp)
            self.position.blockSignals(False)
            self.timeline.current = self.index
            self.timeline.update()
            detail = {
                "frame": {
                    "gui_index": self.index,
                    "relative_sec": frame.image_stamp - start_stamp,
                    "image_stamp": frame.image_stamp,
                    "bag_record_stamp": frame.bag_stamp,
                    "bag_image_transport_delay_ms": (
                        frame.bag_stamp - frame.image_stamp
                    ) * 1000.0,
                },
                "offline_recomputed_tracker": frame.result,
                "online_tracker_tick": None if tracker_sample is None else {
                    "age_ms": (frame.bag_stamp - tracker_sample[0]) * 1000.0,
                    "payload": tracker_sample[1],
                },
                "online_control_tick": None if control_sample is None else {
                    "age_ms": (frame.bag_stamp - control_sample[0]) * 1000.0,
                    "payload": control_sample[1],
                },
                "offline_shadow_control_tick": (
                    None if shadow_sample is None else {
                        "age_ms": (frame.image_stamp - shadow_sample[0]) * 1000.0,
                        "payload": shadow_sample[1],
                    }
                ),
                "touch_state": None if touch_sample is None else touch_sample[1],
                "arm_dashboard_execution": execution,
                "joint_positions_rad": None if joint_sample is None else joint_sample[1],
                "trajectory_controller": (
                    None if controller_sample is None else controller_sample[1]
                ),
                "base_from_arm_camera_rotation": (
                    None if tf_sample is None else {
                        "age_ms": (frame.image_stamp - tf_sample[0]) * 1000.0,
                        "matrix": np.asarray(tf_sample[1]).tolist(),
                    }
                ),
                "events_at_frame": [
                    {"kind": e.kind, "label": e.label, "severity": e.severity}
                    for e in events if e.frame_index == self.index
                ],
            }
            self.details.setPlainText(json.dumps(detail, indent=2, ensure_ascii=False))

    app = QApplication.instance() or QApplication(sys.argv)
    window = Window()
    window.show()
    return app.exec_()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--initial-arm-observation", type=Path)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--analyze-only", action="store_true",
        help="print summary JSON without opening a GUI",
    )
    parser.add_argument("--summary-json", type=Path)
    args = parser.parse_args()
    bag = args.bag.resolve()
    database = BagDatabase(bag)
    try:
        record_path = REPLAY.find_initial_record(
            bag,
            None if args.initial_arm_observation is None
            else args.initial_arm_observation.resolve(),
        )
        print(f"Initial human keyframe: {record_path}", flush=True)
        frames, events, summary = analyze_bag(
            database, record_path, duration=args.duration,
            progress=lambda done, total: print(
                f"Analyzing Arm frames: {done}/{total}", end="\r", flush=True
            ),
        )
        print(" " * 70, end="\r")
        output = json.dumps(summary, indent=2, ensure_ascii=False)
        print(output)
        if args.summary_json:
            args.summary_json.parent.mkdir(parents=True, exist_ok=True)
            args.summary_json.write_text(output + "\n", encoding="utf-8")
        if args.analyze_only:
            return 0
        return launch_gui(database, record_path, frames, events)
    finally:
        database.close()


if __name__ == "__main__":
    raise SystemExit(main())
