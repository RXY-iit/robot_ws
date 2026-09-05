#!/usr/bin/env python3
"""Save one truthful, stable ChArUco pan/tilt calibration sample per request."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import time
from typing import Any

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, Joy
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def stamp_seconds(msg: Any) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def camera_info_dict(msg: CameraInfo) -> dict[str, Any]:
    return {
        'header_stamp_sec': stamp_seconds(msg),
        'frame_id': msg.header.frame_id,
        'height': int(msg.height),
        'width': int(msg.width),
        'distortion_model': msg.distortion_model,
        'd': [float(value) for value in msg.d],
        'k': [float(value) for value in msg.k],
        'r': [float(value) for value in msg.r],
        'p': [float(value) for value in msg.p],
        'binning_x': int(msg.binning_x),
        'binning_y': int(msg.binning_y),
        'roi': {
            'x_offset': int(msg.roi.x_offset),
            'y_offset': int(msg.roi.y_offset),
            'height': int(msg.roi.height),
            'width': int(msg.roi.width),
            'do_rectify': bool(msg.roi.do_rectify),
        },
    }


def safe_label(value: str) -> str:
    cleaned = re.sub(r'[^A-Za-z0-9_.-]+', '_', value.strip()).strip('._')
    return cleaned[:64]


REQUIRED_ARM_JOINTS = {
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_flex_joint',
    'wrist_flex_joint',
    'wrist_roll_joint',
    'gripper_joint',
}

REQUIRED_ARM_MOTORS = {
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
}


class CaptureNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_pan_tilt_calibration_capture')
        output_root_value = str(self.declare_parameter('output_root', '').value).strip()
        if not output_root_value:
            raise RuntimeError('output_root must not be empty')
        self.output_root = Path(output_root_value).expanduser().resolve()

        self.squares_x = int(self.declare_parameter('squares_x', 7).value)
        self.squares_y = int(self.declare_parameter('squares_y', 5).value)
        self.square_length_m = float(
            self.declare_parameter('square_length_m', 0.0338).value
        )
        self.marker_length_m = float(
            self.declare_parameter('marker_length_m', 0.0165).value
        )
        self.dictionary_name = str(
            self.declare_parameter('dictionary', 'DICT_5X5_100').value
        )
        self.min_charuco_corners = int(
            self.declare_parameter('min_charuco_corners', 8).value
        )
        self.max_sensor_age_sec = float(
            self.declare_parameter('max_sensor_age_sec', 0.50).value
        )
        self.max_arm_hardware_age_sec = float(
            self.declare_parameter('max_arm_hardware_age_sec', 1.50).value
        )
        self.stability_window_sec = float(
            self.declare_parameter('stability_window_sec', 0.80).value
        )
        self.stability_tolerance_raw = float(
            self.declare_parameter('stability_tolerance_raw', 0.15).value
        )
        self.arm_stability_tolerance_rad = float(
            self.declare_parameter('arm_stability_tolerance_rad', 0.01).value
        )
        self.direction_delta_raw = float(
            self.declare_parameter('direction_delta_raw', 0.02).value
        )
        self.require_arm_joint_states = bool(
            self.declare_parameter('require_arm_joint_states', True).value
        )
        self.lift_physical_reference = str(
            self.declare_parameter(
                'lift_physical_reference', 'PHYSICAL_LOWER_LIMIT'
            ).value
        ).strip()
        self.board_pose_reference = str(
            self.declare_parameter('board_pose_reference', '').value
        ).strip()
        self.board_pose_base = {
            key: float(self.declare_parameter(parameter, float('nan')).value)
            for key, parameter in {
                'x_m': 'board_base_x_m',
                'y_m': 'board_base_y_m',
                'z_m': 'board_base_z_m',
                'roll_rad': 'board_base_roll_rad',
                'pitch_rad': 'board_base_pitch_rad',
                'yaw_rad': 'board_base_yaw_rad',
            }.items()
        }

        if self.squares_x < 3 or self.squares_y < 3:
            raise ValueError('ChArUco board must have at least 3x3 squares')
        if not 0.0 < self.marker_length_m < self.square_length_m:
            raise ValueError('marker_length_m must be positive and smaller than square_length_m')
        dictionary_id = getattr(cv2.aruco, self.dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f'Unknown OpenCV ArUco dictionary: {self.dictionary_name}')

        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.board = cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y),
            self.square_length_m,
            self.marker_length_m,
            dictionary,
        )
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.bridge = CvBridge()

        self.images: dict[str, deque] = {
            'color': deque(maxlen=20),
            'depth_raw': deque(maxlen=20),
            'depth_aligned': deque(maxlen=20),
            'arm_color': deque(maxlen=20),
        }
        self.camera_info: dict[str, tuple[CameraInfo, float]] = {}
        self.feedback: dict[str, float | None] = {'pan': None, 'tilt': None}
        self.feedback_received: dict[str, float | None] = {'pan': None, 'tilt': None}
        self.feedback_history: dict[str, deque] = {
            'pan': deque(maxlen=300),
            'tilt': deque(maxlen=300),
        }
        self.last_direction = {'pan': 'NONE_OBSERVED', 'tilt': 'NONE_OBSERVED'}
        self.last_direction_at = {'pan': None, 'tilt': None}
        self.targets: dict[str, dict[str, Any] | None] = {'pan': None, 'tilt': None}
        self.latest_joy: tuple[Joy, float] | None = None
        self.latest_joint_state: tuple[JointState, float] | None = None
        self.joint_state_history: deque = deque(maxlen=300)
        self.latest_arm_hardware_status: tuple[
            str, dict[str, Any] | None, float
        ] | None = None
        self.transforms: dict[str, dict[str, Any]] = {}
        self.sample_index = 0
        self.status_hold_until = 0.0

        session_stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.session_dir = self.output_root / f'session_{session_stamp}'
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self._write_json(self.session_dir / 'session.json', self._session_metadata())

        self.status_pub = self.create_publisher(String, '/camera_calibration/status', 10)
        self.create_subscription(
            String, '/camera_calibration/capture_request', self._capture_request, 10
        )
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            lambda msg: self._image_cb('color', msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            lambda msg: self._image_cb('depth_raw', msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            lambda msg: self._image_cb('depth_aligned', msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, '/arm_camera/color/image_raw',
            lambda msg: self._image_cb('arm_color', msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            lambda msg: self._camera_info_cb('color', msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info',
            lambda msg: self._camera_info_cb('depth_raw', msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
            lambda msg: self._camera_info_cb('depth_aligned', msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, '/arm_camera/color/camera_info',
            lambda msg: self._camera_info_cb('arm_color', msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32, '/chokudomotor/angle',
            lambda msg: self._feedback_cb('pan', msg), 10,
        )
        self.create_subscription(
            Float32, '/cameraswingmotor/angle',
            lambda msg: self._feedback_cb('tilt', msg), 10,
        )
        self.create_subscription(
            Float32, '/chokudomotor/target_angle',
            lambda msg: self._target_cb('pan', msg), 10,
        )
        self.create_subscription(
            Float32, '/cameraswingmotor/target_angle',
            lambda msg: self._target_cb('tilt', msg), 10,
        )
        self.create_subscription(Joy, '/joy', self._joy_cb, qos_profile_sensor_data)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            String, '/arm/hardware_status', self._arm_hardware_status_cb, 10
        )
        self.create_subscription(TFMessage, '/tf', self._tf_cb, 10)
        static_tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, '/tf_static', self._tf_cb, static_tf_qos)
        self.create_timer(0.5, self._publish_readiness)

        self.get_logger().info(f'Calibration output: {self.session_dir}')
        self.get_logger().info(
            f'ChArUco: {self.squares_x}x{self.squares_y}, '
            f'square={self.square_length_m}m, marker={self.marker_length_m}m, '
            f'{self.dictionary_name}'
        )

    def _session_metadata(self) -> dict[str, Any]:
        git_head = None
        try:
            git_head = subprocess.run(
                ['git', 'rev-parse', 'HEAD'],
                cwd='/home/matsunaga-h/robot_ws',
                check=True,
                capture_output=True,
                text=True,
                timeout=2,
            ).stdout.strip()
        except Exception:
            pass
        return {
            'schema_version': 1,
            'created_at_utc': utc_now(),
            'purpose': 'camera pan/tilt fixed-pose calibration capture',
            'output_directory': str(self.session_dir),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'host': platform.node(),
            'platform': platform.platform(),
            'workspace_git_head': git_head,
            'source_topics': {
                'color_image': '/camera/camera/color/image_raw',
                'raw_depth_image': '/camera/camera/depth/image_rect_raw',
                'aligned_depth_image': '/camera/camera/aligned_depth_to_color/image_raw',
                'color_camera_info': '/camera/camera/color/camera_info',
                'raw_depth_camera_info': '/camera/camera/depth/camera_info',
                'aligned_depth_camera_info': '/camera/camera/aligned_depth_to_color/camera_info',
                'arm_color_image': '/arm_camera/color/image_raw',
                'arm_color_camera_info': '/arm_camera/color/camera_info',
                'pan_raw_feedback': '/chokudomotor/angle',
                'tilt_raw_feedback': '/cameraswingmotor/angle',
                'pan_target': '/chokudomotor/target_angle',
                'tilt_target': '/cameraswingmotor/target_angle',
                'joy': '/joy',
                'arm_joint_state_required': '/joint_states',
                'arm_hardware_status_required': '/arm/hardware_status',
                'transforms': ['/tf', '/tf_static'],
            },
            'rig_configuration': {
                'lift_physical_reference': self.lift_physical_reference or None,
                'lift_motion_during_session': 'FIXED',
                'note': (
                    'Operator-confirmed physical reference. This is not an encoder '
                    'measurement and does not define the metric arm-base height.'
                ),
            },
            'board': self._board_metadata(),
            'board_pose_in_base': self._board_pose_metadata(),
            'capture_rules': {
                'min_charuco_corners': self.min_charuco_corners,
                'max_sensor_age_sec': self.max_sensor_age_sec,
                'max_arm_hardware_age_sec': self.max_arm_hardware_age_sec,
                'image_time_policy': (
                    'Each stream must be fresh at capture time. Header-stamp and '
                    'host receive-time differences are recorded but are not gates: '
                    'the two USB cameras have no verified shared hardware clock.'
                ),
                'stability_window_sec': self.stability_window_sec,
                'stability_tolerance_raw': self.stability_tolerance_raw,
                'direction_delta_raw': self.direction_delta_raw,
            },
            'truth_notes': [
                'pan/tilt values are raw values reported by the deployed OpenRB path',
                'arrival direction is inferred from raw feedback changes, not physical left/right/up/down',
                'feedback stability is an observed numeric rule, not proof of mechanical settling',
                'board printed dimensions must be physically measured before metric calibration',
            ],
        }

    def _board_metadata(self) -> dict[str, Any]:
        return {
            'type': 'ChArUco',
            'squares_x': self.squares_x,
            'squares_y': self.squares_y,
            'square_length_m_configured': self.square_length_m,
            'marker_length_m_configured': self.marker_length_m,
            'dictionary': self.dictionary_name,
        }

    def _board_pose_metadata(self) -> dict[str, Any]:
        complete = all(math.isfinite(value) for value in self.board_pose_base.values())
        return {
            'status': 'OPERATOR_PROVIDED' if complete else 'NOT_PROVIDED',
            'parent_frame': 'base_link',
            'child_frame': 'charuco_board',
            'xyz_rpy': self.board_pose_base if complete else None,
            'measurement_reference': self.board_pose_reference or None,
            'note': (
                'Values are operator-provided measurement records, not inferred by this capture tool.'
                if complete else
                'Absolute base_link-to-board pose was not supplied; metric '
                'base-frame extrinsics cannot be established from this field alone.'
            ),
        }

    def _image_cb(self, key: str, msg: Image) -> None:
        self.images[key].append((msg, time.monotonic()))

    def _camera_info_cb(self, key: str, msg: CameraInfo) -> None:
        self.camera_info[key] = (msg, time.monotonic())

    def _feedback_cb(self, axis: str, msg: Float32) -> None:
        now = time.monotonic()
        value = float(msg.data)
        previous = self.feedback[axis]
        if previous is not None:
            delta = value - previous
            if abs(delta) >= self.direction_delta_raw:
                self.last_direction[axis] = 'INCREASING' if delta > 0 else 'DECREASING'
                self.last_direction_at[axis] = utc_now()
        self.feedback[axis] = value
        self.feedback_received[axis] = now
        self.feedback_history[axis].append((now, time.time(), value))

    def _target_cb(self, axis: str, msg: Float32) -> None:
        self.targets[axis] = {
            'value_raw': float(msg.data),
            'received_at_utc': utc_now(),
            'received_monotonic': time.monotonic(),
        }

    def _joy_cb(self, msg: Joy) -> None:
        self.latest_joy = (msg, time.monotonic())

    def _joint_state_cb(self, msg: JointState) -> None:
        now = time.monotonic()
        self.latest_joint_state = (msg, now)
        if len(msg.name) == len(msg.position):
            positions = {
                str(name): float(position)
                for name, position in zip(msg.name, msg.position)
            }
            if REQUIRED_ARM_JOINTS.issubset(positions):
                self.joint_state_history.append((now, positions))

    def _arm_hardware_status_cb(self, msg: String) -> None:
        parsed = None
        try:
            candidate = json.loads(msg.data)
            if isinstance(candidate, dict):
                parsed = candidate
        except (json.JSONDecodeError, TypeError):
            pass
        self.latest_arm_hardware_status = (msg.data, parsed, time.monotonic())

    def _tf_cb(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            key = f'{transform.header.frame_id}->{transform.child_frame_id}'
            self.transforms[key] = {
                'header_stamp_sec': stamp_seconds(transform),
                'parent_frame': transform.header.frame_id,
                'child_frame': transform.child_frame_id,
                'translation_m': {
                    'x': float(transform.transform.translation.x),
                    'y': float(transform.transform.translation.y),
                    'z': float(transform.transform.translation.z),
                },
                'rotation_xyzw': {
                    'x': float(transform.transform.rotation.x),
                    'y': float(transform.transform.rotation.y),
                    'z': float(transform.transform.rotation.z),
                    'w': float(transform.transform.rotation.w),
                },
                'received_at_utc': utc_now(),
            }

    def _stability(self, axis: str, now: float) -> dict[str, Any]:
        recent = [
            entry for entry in self.feedback_history[axis]
            if now - entry[0] <= self.stability_window_sec
        ]
        coverage = 0.0 if len(recent) < 2 else recent[-1][0] - recent[0][0]
        values = [entry[2] for entry in recent]
        span = None if not values else max(values) - min(values)
        stable = (
            len(recent) >= 2
            and coverage >= self.stability_window_sec * 0.75
            and span is not None
            and span <= self.stability_tolerance_raw
        )
        return {
            'stable': stable,
            'sample_count': len(recent),
            'window_coverage_sec': coverage,
            'span_raw': span,
            'rule_window_sec': self.stability_window_sec,
            'rule_max_span_raw': self.stability_tolerance_raw,
            'observed_stable_since_utc': (
                datetime.fromtimestamp(recent[0][1], timezone.utc).isoformat()
                if stable else None
            ),
        }

    def _arm_stability(self, now: float) -> dict[str, Any]:
        # /joint_states is 5 Hz on the deployed arm. Clipping at exactly 0.8 s
        # normally leaves only four samples spanning about 0.6 s because of
        # scheduler phase. Keep one additional sample interval, while still
        # requiring the observations themselves to span the requested window.
        history_horizon = self.stability_window_sec + 0.25
        recent = [
            entry for entry in self.joint_state_history
            if now - entry[0] <= history_horizon
        ]
        coverage = 0.0 if len(recent) < 2 else recent[-1][0] - recent[0][0]
        spans = {
            name: (
                max(entry[1][name] for entry in recent)
                - min(entry[1][name] for entry in recent)
            )
            for name in sorted(REQUIRED_ARM_JOINTS)
        } if recent else {}
        stable = (
            len(recent) >= 2
            and coverage >= self.stability_window_sec - 0.03
            and all(
                span <= self.arm_stability_tolerance_rad
                for span in spans.values()
            )
        )
        return {
            'stable': stable,
            'sample_count': len(recent),
            'window_coverage_sec': coverage,
            'span_rad_by_joint': spans,
            'rule_window_sec': self.stability_window_sec,
            'rule_max_span_rad': self.arm_stability_tolerance_rad,
        }

    def _evidence(self) -> tuple[dict[str, Any], list[str]]:
        now = time.monotonic()
        errors = []
        evidence: dict[str, Any] = {'checked_at_utc': utc_now()}

        if not self.images['color']:
            errors.append('no color image')
            return evidence, errors
        color_msg, color_received = self.images['color'][-1]
        evidence['color'] = (color_msg, color_received)
        if now - color_received > self.max_sensor_age_sec:
            errors.append(f'color image stale: {now - color_received:.3f}s')

        for key in ('depth_raw', 'depth_aligned', 'arm_color'):
            # Use the freshest observed frame.  Comparing ROS header stamps across
            # the D435 and the independent UVC wrist camera is not a valid
            # synchronization test: no common hardware clock/trigger has been
            # verified.  The rig and board are required to be stationary, so the
            # truthful gate is host-observed freshness plus mechanism stability.
            selected = self.images[key][-1] if self.images[key] else None
            if selected is None:
                errors.append(f'no {key} image')
                continue
            msg, received = selected
            evidence[key] = selected
            if now - received > self.max_sensor_age_sec:
                errors.append(f'{key} image stale: {now - received:.3f}s')

        for key in ('color', 'depth_raw', 'depth_aligned', 'arm_color'):
            if key not in self.camera_info:
                errors.append(f'no {key} CameraInfo')
            else:
                msg, received = self.camera_info[key]
                evidence[f'{key}_info'] = (msg, received)
                if now - received > self.max_sensor_age_sec:
                    errors.append(f'{key} CameraInfo stale: {now - received:.3f}s')

        for axis in ('pan', 'tilt'):
            received = self.feedback_received[axis]
            if self.feedback[axis] is None or received is None:
                errors.append(f'no {axis} feedback')
            elif now - received > self.max_sensor_age_sec:
                errors.append(f'{axis} feedback stale: {now - received:.3f}s')
            stability = self._stability(axis, now)
            evidence[f'{axis}_stability'] = stability
            if not stability['stable']:
                errors.append(
                    f'{axis} not stable: span={stability["span_raw"]}, '
                    f'coverage={stability["window_coverage_sec"]:.3f}s'
                )

        if self.require_arm_joint_states:
            if self.latest_joint_state is None:
                errors.append('no required real /joint_states')
            else:
                joint_msg, joint_received = self.latest_joint_state
                joint_age = now - joint_received
                if joint_age > self.max_sensor_age_sec:
                    errors.append(f'/joint_states stale: {joint_age:.3f}s')
                elif not joint_msg.name or len(joint_msg.name) != len(joint_msg.position):
                    errors.append('/joint_states names/positions are incomplete')
                elif any(not math.isfinite(float(value)) for value in joint_msg.position):
                    errors.append('/joint_states contains a non-finite position')
                else:
                    missing = sorted(REQUIRED_ARM_JOINTS.difference(joint_msg.name))
                    if missing:
                        errors.append(
                            '/joint_states missing required joints: ' + ','.join(missing)
                        )
                    else:
                        arm_stability = self._arm_stability(now)
                        evidence['arm_joint_stability'] = arm_stability
                        if not arm_stability['stable']:
                            errors.append(
                                'arm joints not stable: coverage='
                                f'{arm_stability["window_coverage_sec"]:.3f}s'
                            )

            if self.latest_arm_hardware_status is None:
                errors.append('no required /arm/hardware_status')
            else:
                _raw_json, hardware, hardware_received = self.latest_arm_hardware_status
                hardware_age = now - hardware_received
                if hardware_age > self.max_arm_hardware_age_sec:
                    errors.append(f'/arm/hardware_status stale: {hardware_age:.3f}s')
                elif hardware is None:
                    errors.append('/arm/hardware_status is not a valid JSON object')
                else:
                    raw_position = hardware.get('raw_position')
                    if not isinstance(raw_position, dict):
                        errors.append('/arm/hardware_status has no raw_position object')
                    else:
                        missing_motors = sorted(
                            REQUIRED_ARM_MOTORS.difference(raw_position)
                        )
                        if missing_motors:
                            errors.append(
                                '/arm/hardware_status raw_position missing: '
                                + ','.join(missing_motors)
                            )

        # Detect the board during readiness checks as well as on capture.  A
        # generic READY followed by a corner-count rejection gave the operator
        # no actionable preview of framing/focus/board visibility.
        for key, label in (('color', 'RealSense'), ('arm_color', 'arm camera')):
            if key not in evidence:
                continue
            try:
                image_bgr = self.bridge.imgmsg_to_cv2(
                    evidence[key][0], desired_encoding='bgr8'
                )
                gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
                corners, ids, _marker_corners, _marker_ids = (
                    self.detector.detectBoard(gray)
                )
                count = 0 if ids is None else int(len(ids))
                evidence[f'{key}_charuco_corner_count'] = count
                if count < self.min_charuco_corners:
                    errors.append(
                        f'{label} ChArUco corners={count}; '
                        f'need {self.min_charuco_corners}'
                    )
            except Exception as exc:
                errors.append(f'{label} ChArUco preview failed: {exc}')
        return evidence, errors

    def _publish_readiness(self) -> None:
        if time.monotonic() < self.status_hold_until:
            return
        evidence, errors = self._evidence()
        if errors:
            self._status('WAITING: ' + '; '.join(errors[:4]))
        else:
            self._status(
                f'READY: stable pan={self.feedback["pan"]:.3f}, '
                f'tilt={self.feedback["tilt"]:.3f}; corners='
                f'{evidence["color_charuco_corner_count"]}/'
                f'{evidence["arm_color_charuco_corner_count"]}; press Capture'
            )

    def _capture_request(self, request: String) -> None:
        evidence, errors = self._evidence()
        if errors:
            message = 'REJECTED: ' + '; '.join(errors)
            self._status(message, hold_sec=4.0)
            self.get_logger().warning(message)
            return

        color_msg = evidence['color'][0]
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(
                evidence['depth_raw'][0], desired_encoding='passthrough'
            )
            depth_aligned = self.bridge.imgmsg_to_cv2(
                evidence['depth_aligned'][0], desired_encoding='passthrough'
            )
            arm_color_bgr = self.bridge.imgmsg_to_cv2(
                evidence['arm_color'][0], desired_encoding='bgr8'
            )
        except Exception as exc:
            self._status(f'REJECTED: image conversion failed: {exc}', hold_sec=4.0)
            return

        gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.detector.detectBoard(gray)
        )
        corner_count = 0 if charuco_ids is None else int(len(charuco_ids))
        arm_gray = cv2.cvtColor(arm_color_bgr, cv2.COLOR_BGR2GRAY)
        arm_charuco_corners, arm_charuco_ids, arm_marker_corners, arm_marker_ids = (
            self.detector.detectBoard(arm_gray)
        )
        arm_corner_count = (
            0 if arm_charuco_ids is None else int(len(arm_charuco_ids))
        )
        if corner_count < self.min_charuco_corners:
            self._status(
                f'REJECTED: RealSense has only {corner_count} ChArUco corners; '
                f'need at least {self.min_charuco_corners}',
                hold_sec=4.0,
            )
            return
        if arm_corner_count < self.min_charuco_corners:
            self._status(
                f'REJECTED: arm camera has only {arm_corner_count} ChArUco corners; '
                f'need at least {self.min_charuco_corners}',
                hold_sec=4.0,
            )
            return

        label = safe_label(request.data)
        self.sample_index += 1
        suffix = f'_{label}' if label else ''
        sample_name = f'sample_{self.sample_index:04d}{suffix}'
        partial_dir = self.session_dir / f'.{sample_name}.partial'
        sample_dir = self.session_dir / sample_name
        if partial_dir.exists():
            shutil.rmtree(partial_dir)
        partial_dir.mkdir()

        try:
            annotated = color_bgr.copy()
            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)
            cv2.aruco.drawDetectedCornersCharuco(
                annotated, charuco_corners, charuco_ids, (0, 255, 0)
            )
            arm_annotated = arm_color_bgr.copy()
            if arm_marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(
                    arm_annotated, arm_marker_corners, arm_marker_ids
                )
            cv2.aruco.drawDetectedCornersCharuco(
                arm_annotated, arm_charuco_corners, arm_charuco_ids, (0, 255, 0)
            )

            self._save_png(partial_dir / 'color.png', color_bgr)
            self._save_png(partial_dir / 'color_charuco.png', annotated)
            self._save_png(partial_dir / 'arm_color.png', arm_color_bgr)
            self._save_png(partial_dir / 'arm_color_charuco.png', arm_annotated)
            self._save_png(partial_dir / 'depth_raw.png', depth_raw)
            self._save_png(partial_dir / 'depth_aligned_to_color.png', depth_aligned)
            np.save(partial_dir / 'depth_raw.npy', np.asarray(depth_raw), allow_pickle=False)
            np.save(
                partial_dir / 'depth_aligned_to_color.npy',
                np.asarray(depth_aligned), allow_pickle=False,
            )

            board_points = np.asarray(self.board.getChessboardCorners(), dtype=float)
            ids = np.asarray(charuco_ids, dtype=int).reshape(-1)
            pixels = np.asarray(charuco_corners, dtype=float).reshape(-1, 2)
            charuco = {
                'detected_corner_count': corner_count,
                'minimum_required': self.min_charuco_corners,
                'corners': [
                    {
                        'id': int(corner_id),
                        'pixel_uv': [float(pixel[0]), float(pixel[1])],
                        'board_xyz_m': [float(v) for v in board_points[corner_id]],
                    }
                    for corner_id, pixel in zip(ids, pixels)
                ],
                'detected_marker_ids': (
                    [] if marker_ids is None
                    else [int(v) for v in np.asarray(marker_ids).reshape(-1)]
                ),
            }
            self._write_json(partial_dir / 'charuco.json', charuco)

            arm_ids = np.asarray(arm_charuco_ids, dtype=int).reshape(-1)
            arm_pixels = np.asarray(arm_charuco_corners, dtype=float).reshape(-1, 2)
            arm_charuco = {
                'detected_corner_count': arm_corner_count,
                'minimum_required': self.min_charuco_corners,
                'corners': [
                    {
                        'id': int(corner_id),
                        'pixel_uv': [float(pixel[0]), float(pixel[1])],
                        'board_xyz_m': [float(v) for v in board_points[corner_id]],
                    }
                    for corner_id, pixel in zip(arm_ids, arm_pixels)
                ],
                'detected_marker_ids': (
                    [] if arm_marker_ids is None
                    else [int(v) for v in np.asarray(arm_marker_ids).reshape(-1)]
                ),
            }
            self._write_json(partial_dir / 'arm_charuco.json', arm_charuco)

            now = time.monotonic()
            metadata = {
                'schema_version': 1,
                'sample_id': self.sample_index,
                'label': request.data.strip(),
                'directory_name': sample_name,
                'capture_completed_at_utc': utc_now(),
                'board': self._board_metadata(),
                'board_pose_in_base': self._board_pose_metadata(),
                'feedback': {
                    axis: {
                        'value_raw': self.feedback[axis],
                        'receive_age_sec': now - self.feedback_received[axis],
                        'arrival_direction_raw': self.last_direction[axis],
                        'last_direction_observed_at_utc': self.last_direction_at[axis],
                        'stability': evidence[f'{axis}_stability'],
                        'latest_target': self._target_metadata(axis, now),
                    }
                    for axis in ('pan', 'tilt')
                },
                'recent_feedback_trace': {
                    axis: self._feedback_trace(axis, now)
                    for axis in ('pan', 'tilt')
                },
                'images': {
                    key: self._image_metadata(evidence[key][0], evidence[key][1], now)
                    for key in ('color', 'depth_raw', 'depth_aligned', 'arm_color')
                },
                'image_stamp_skew_from_realsense_color_sec': {
                    key: abs(
                        stamp_seconds(evidence[key][0])
                        - stamp_seconds(evidence['color'][0])
                    )
                    for key in ('depth_raw', 'depth_aligned', 'arm_color')
                },
                'image_receive_skew_from_realsense_color_sec': {
                    key: abs(evidence[key][1] - evidence['color'][1])
                    for key in ('depth_raw', 'depth_aligned', 'arm_color')
                },
                'camera_info': {
                    key: camera_info_dict(evidence[f'{key}_info'][0])
                    for key in ('color', 'depth_raw', 'depth_aligned', 'arm_color')
                },
                'arm_camera_intrinsics_status': self._arm_intrinsics_status(evidence),
                'arm_joint_state_at_capture': self._joint_state_metadata(now),
                'arm_hardware_status_at_capture': self._arm_hardware_metadata(now),
                'arm_joint_stability': evidence.get('arm_joint_stability'),
                'joy_at_capture': self._joy_metadata(now),
                'tf_snapshot': dict(sorted(self.transforms.items())),
                'charuco_file': 'charuco.json',
                'arm_charuco_file': 'arm_charuco.json',
                'truth_notes': [
                    'raw arrival direction is increasing/decreasing feedback value, not a physical direction label',
                    'PNG and NPY depth values remain in the source ROS image encoding/units',
                    'no URDF pose or calibrated physical angle is inferred during capture',
                    'fresh arm joint states and direct raw motor positions are '
                    'required calibration evidence',
                    'the deployed arm CameraInfo is uncalibrated until a nonzero '
                    'measured intrinsic matrix is installed',
                    'the two USB cameras have no verified shared hardware clock or '
                    'trigger; header/receive-time differences are audit evidence, '
                    'not proof of simultaneous exposure',
                ],
            }
            self._write_json(partial_dir / 'metadata.json', metadata)
            partial_dir.rename(sample_dir)
            with (self.session_dir / 'manifest.jsonl').open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(metadata, ensure_ascii=False) + '\n')
                handle.flush()
                os.fsync(handle.fileno())
        except Exception as exc:
            self.get_logger().error(f'Failed to save {sample_name}: {exc}')
            self._status(f'ERROR: failed to save {sample_name}: {exc}', hold_sec=6.0)
            return

        self._status(
            f'SAVED: {sample_name}; RealSense/arm corners='
            f'{corner_count}/{arm_corner_count}; '
            f'pan={self.feedback["pan"]:.3f}; tilt={self.feedback["tilt"]:.3f}',
            hold_sec=4.0,
        )
        self.get_logger().info(f'Saved calibration sample: {sample_dir}')

    def _target_metadata(self, axis: str, now: float):
        target = self.targets[axis]
        if target is None:
            return None
        return {
            'value_raw': target['value_raw'],
            'received_at_utc': target['received_at_utc'],
            'receive_age_sec': now - target['received_monotonic'],
        }

    def _feedback_trace(
        self, axis: str, now: float, duration_sec: float = 5.0
    ) -> list[dict[str, float]]:
        """Preserve enough raw history to audit approach direction and settling."""
        return [
            {
                'seconds_before_capture': float(now - received),
                'value_raw': float(value),
            }
            for received, _wall_time, value in self.feedback_history[axis]
            if now - received <= duration_sec
        ]

    def _image_metadata(self, msg: Image, received: float, now: float) -> dict[str, Any]:
        return {
            'header_stamp_sec': stamp_seconds(msg),
            'frame_id': msg.header.frame_id,
            'receive_age_sec': now - received,
            'height': int(msg.height),
            'width': int(msg.width),
            'encoding': msg.encoding,
            'is_bigendian': int(msg.is_bigendian),
            'step': int(msg.step),
        }

    def _joy_metadata(self, now: float):
        if self.latest_joy is None:
            return None
        msg, received = self.latest_joy
        return {
            'receive_age_sec': now - received,
            'header_stamp_sec': stamp_seconds(msg),
            'axes': [float(v) for v in msg.axes],
            'buttons': [int(v) for v in msg.buttons],
        }

    @staticmethod
    def _arm_intrinsics_status(evidence: dict[str, Any]) -> dict[str, Any]:
        info = evidence['arm_color_info'][0]
        calibrated = bool(len(info.k) >= 5 and info.k[0] > 0.0 and info.k[4] > 0.0)
        return {
            'status': 'CALIBRATED_CAMERA_INFO' if calibrated else 'UNCALIBRATED',
            'usable_for_metric_projection': calibrated,
            'rule': 'sensor_msgs/CameraInfo K[0] and K[4] must both be positive',
        }

    def _joint_state_metadata(self, now: float) -> dict[str, Any]:
        if self.latest_joint_state is None:
            return {
                'status': 'NOT_AVAILABLE',
                'source_topic': '/joint_states',
                'note': (
                    'Camera intrinsics samples remain usable, but arm-base/hand-eye '
                    'estimation requires matching real joint states from multiple poses.'
                ),
            }
        msg, received = self.latest_joint_state
        age = now - received
        return {
            'status': 'OBSERVED' if age <= self.max_sensor_age_sec else 'STALE',
            'source_topic': '/joint_states',
            'receive_age_sec': age,
            'header_stamp_sec': stamp_seconds(msg),
            'name': list(msg.name),
            'position_rad': [float(value) for value in msg.position],
            'velocity_rad_s': [float(value) for value in msg.velocity],
            'effort': [float(value) for value in msg.effort],
        }

    def _arm_hardware_metadata(self, now: float) -> dict[str, Any]:
        if self.latest_arm_hardware_status is None:
            return {
                'status': 'NOT_AVAILABLE',
                'source_topic': '/arm/hardware_status',
            }
        raw_json, parsed, received = self.latest_arm_hardware_status
        age = now - received
        raw_position = parsed.get('raw_position') if isinstance(parsed, dict) else None
        complete = (
            isinstance(raw_position, dict)
            and REQUIRED_ARM_MOTORS.issubset(raw_position)
        )
        return {
            'status': (
                'OBSERVED'
                if parsed is not None and complete and age <= self.max_arm_hardware_age_sec
                else 'INVALID_OR_STALE'
            ),
            'source_topic': '/arm/hardware_status',
            'receive_age_sec': age,
            'raw_position': (
                {
                    name: int(raw_position[name])
                    for name in sorted(REQUIRED_ARM_MOTORS)
                }
                if complete else None
            ),
            'parsed_payload': parsed,
            'raw_json': raw_json,
        }

    @staticmethod
    def _save_png(path: Path, image: np.ndarray) -> None:
        if not cv2.imwrite(str(path), image):
            raise IOError(f'cv2.imwrite failed: {path}')

    @staticmethod
    def _write_json(path: Path, payload: dict[str, Any]) -> None:
        with path.open('w', encoding='utf-8') as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write('\n')

    def _status(self, text: str, hold_sec: float = 0.0) -> None:
        if hold_sec > 0.0:
            self.status_hold_until = max(
                self.status_hold_until, time.monotonic() + hold_sec
            )
        self.status_pub.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
