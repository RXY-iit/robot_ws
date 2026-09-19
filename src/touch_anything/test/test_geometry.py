import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import threading

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "touch_observation_node.py"
SPEC = importlib.util.spec_from_file_location("touch_observation_node", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_depth_units_are_explicit():
    raw = np.array([[1000, 2500]], dtype=np.uint16)
    np.testing.assert_allclose(
        MODULE.depth_to_meters(raw, "16UC1", 0.001), [[1.0, 2.5]]
    )


def test_deproject_center_pixel():
    k = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    result = MODULE.deproject_pixels(np.array([[320, 240]]), np.array([0.8]), k)
    np.testing.assert_allclose(result, [[0.0, 0.0, 0.8]])


def test_surface_target_rejects_depth_outliers_and_faces_camera():
    height, width = 80, 100
    depth = np.ones((height, width), dtype=np.float32)
    mask = np.ones((height, width), dtype=bool)
    depth[35:38, 45:48] = 4.0
    k = np.array([[500.0, 0.0, 50.0], [0.0, 500.0, 40.0], [0.0, 0.0, 1.0]])
    result = MODULE.robust_surface_target(mask, depth, (50, 40), k)
    assert abs(result["median_depth_m"] - 1.0) < 1e-6
    assert result["valid_points"] >= 30
    assert result["normal"][2] < -0.99


def test_quaternion_matrix_does_not_mutate_evidence():
    quaternion = np.array([0.1, -0.2, 0.3, 0.9], dtype=np.float64)
    before = quaternion.copy()
    matrix = MODULE.quaternion_matrix_xyzw(quaternion)
    np.testing.assert_array_equal(quaternion, before)
    np.testing.assert_allclose(matrix @ matrix.T, np.eye(3), atol=1e-12)


def test_motor_evidence_reports_per_motor_median_and_mad():
    samples = [
        {"monotonic_sec": 1.0, "current_raw": {"shoulder_lift": 4, "wrist_flex": 1}},
        {"monotonic_sec": 1.1, "current_raw": {"shoulder_lift": 6, "wrist_flex": 1}},
        {"monotonic_sec": 1.2, "current_raw": {"shoulder_lift": 8, "wrist_flex": 3}},
    ]
    result = MODULE.TouchObservationNode._summarize_motor_samples(samples)
    assert result["sample_count"] == 3
    assert np.isclose(result["duration_sec"], 0.2)
    assert result["motors"]["shoulder_lift"]["median_raw"] == 6.0
    assert result["motors"]["shoulder_lift"]["mad_raw"] == 2.0
    assert result["motors"]["wrist_flex"]["median_raw"] == 1.0
    assert result["motors"]["wrist_flex"]["mad_raw"] == 0.0


def test_post_motion_tf_rejects_pre_completion_sample_even_when_fresh():
    reason = MODULE.post_motion_tf_rejection_reason(
        transform_stamp_sec=100.10,
        now_ros_sec=100.20,
        minimum_completion_stamp=100.15,
        maximum_age_sec=0.75,
    )
    assert reason is not None
    assert "predates the completed motion" in reason


def test_post_motion_tf_accepts_fresh_sample_from_completion_window():
    reason = MODULE.post_motion_tf_rejection_reason(
        transform_stamp_sec=100.18,
        now_ros_sec=100.20,
        minimum_completion_stamp=100.15,
        maximum_age_sec=0.75,
    )
    assert reason is None


def test_post_motion_tf_rejects_stale_and_future_samples():
    stale = MODULE.post_motion_tf_rejection_reason(
        transform_stamp_sec=99.0,
        now_ros_sec=100.0,
        minimum_completion_stamp=None,
        maximum_age_sec=0.75,
    )
    future = MODULE.post_motion_tf_rejection_reason(
        transform_stamp_sec=101.0,
        now_ros_sec=100.0,
        minimum_completion_stamp=None,
        maximum_age_sec=0.75,
    )
    assert stale is not None and "stale" in stale
    assert future is not None and "future" in future


def test_motion_tracking_evidence_requires_samples_across_full_window():
    continuous = MODULE.motion_tracking_evidence(
        [10.1, 10.3, 10.5, 10.7, 10.9], 10.0, 11.0, 3, 0.30
    )
    clustered = MODULE.motion_tracking_evidence(
        [10.1, 10.2, 10.3, 11.1], 10.0, 11.0, 3, 0.30
    )
    assert continuous["continuous"] is True
    assert continuous["successful_frame_count"] == 5
    assert clustered["continuous"] is False
    assert clustered["maximum_gap_sec"] > 0.30


def test_appearance_signature_uses_masked_lab_median():
    rgb = np.zeros((20, 20, 3), dtype=np.uint8)
    rgb[:, :] = [10, 20, 30]
    rgb[5:15, 5:15] = [200, 30, 20]
    mask = np.zeros((20, 20), dtype=bool)
    mask[5:15, 5:15] = True
    result = MODULE.TouchObservationNode._appearance_signature(rgb, mask)
    expected = MODULE.cv2.cvtColor(
        np.array([[[200, 30, 20]]], dtype=np.uint8), MODULE.cv2.COLOR_RGB2LAB
    )[0, 0]
    np.testing.assert_allclose(result["median_lab"], expected)
    assert result["sample_count"] == 100


def test_continuous_arm_image_snapshot_does_not_require_info_or_tf():
    """KLT/ORB ingestion must survive CameraInfo/TF timing gaps."""
    node = object.__new__(MODULE.TouchObservationNode)

    class FakeBridge:
        @staticmethod
        def imgmsg_to_cv2(_message, desired_encoding):
            assert desired_encoding == "rgb8"
            return np.zeros((12, 16, 3), dtype=np.uint8)

    node._bridge = FakeBridge()
    image = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=123, nanosec=456_000_000),
        frame_id="arm_camera_color_optical_frame",
    ))

    frame = MODULE.TouchObservationNode._arm_image_snapshot(node, image)

    assert frame["rgb"].shape == (12, 16, 3)
    assert frame["frame_id"] == "arm_camera_color_optical_frame"
    assert np.isclose(frame["stamp"], 123.456)
    assert frame["geometry_ready"] is False
    assert "camera_matrix" not in frame
    assert "base_from_camera_rotation" not in frame


def test_overlay_dds_publish_happens_after_shared_lock_is_released():
    node = object.__new__(MODULE.TouchObservationNode)
    node._lock = threading.RLock()
    node._overlay_stage = "D435"
    stamp = SimpleNamespace(sec=12, nanosec=0)
    node._frozen = {
        "rgb": np.zeros((12, 16, 3), dtype=np.uint8),
        "stamp_msg": stamp,
        "frame_id": "camera_color_optical_frame",
    }
    node._mask = None
    node._prompts = []
    node._labels = []

    class FakeBridge:
        @staticmethod
        def cv2_to_imgmsg(_rgb, encoding):
            assert encoding == "rgb8"
            return SimpleNamespace(header=SimpleNamespace(stamp=None, frame_id=""))

    class LockCheckingPublisher:
        published = 0

        def publish(self, _message):
            # This captures the regression that blocked control, Dashboard and
            # tracker callbacks for multiple seconds in the 21:26 bag.
            assert not node._lock._is_owned()
            self.published += 1

    node._bridge = FakeBridge()
    node._overlay_pub = LockCheckingPublisher()

    MODULE.TouchObservationNode._republish_overlay(node)

    assert node._overlay_pub.published == 1


def test_overlay_tick_coalesces_latest_value_requests():
    node = object.__new__(MODULE.TouchObservationNode)
    node._lock = threading.RLock()
    node._overlay_dirty = True
    node._overlay_last_publish_duration_ms = None
    node._overlay_last_error = None
    node._overlay_published_count = 0
    node._overlay_deferred_for_motion_count = 0
    node._automatic_approach = False
    node._reach_execute_request_id = None
    node._approach_execute_request_id = None
    node._home_execute_request_id = None
    calls = []
    node._republish_overlay = lambda: calls.append("published") or True

    MODULE.TouchObservationNode._overlay_publish_tick(node)
    MODULE.TouchObservationNode._overlay_publish_tick(node)

    assert calls == ["published"]
    assert node._overlay_dirty is False
    assert node._overlay_published_count == 1
    assert node._overlay_last_publish_duration_ms is not None


def test_overlay_tick_defers_raw_image_while_automatic_approach_runs():
    node = object.__new__(MODULE.TouchObservationNode)
    node._lock = threading.RLock()
    node._overlay_dirty = True
    node._overlay_deferred_for_motion_count = 0
    node._automatic_approach = True
    node._reach_execute_request_id = None
    node._approach_execute_request_id = "approach-1"
    node._home_execute_request_id = None
    calls = []
    node._republish_overlay = lambda: calls.append("published") or True

    MODULE.TouchObservationNode._overlay_publish_tick(node)

    assert calls == []
    assert node._overlay_dirty is True
    assert node._overlay_deferred_for_motion_count == 1


def test_debug_telemetry_is_queued_before_independent_publish_tick():
    node = object.__new__(MODULE.TouchObservationNode)
    node._tracker_debug_queue = MODULE.deque(maxlen=2)
    node._control_debug_queue = MODULE.deque(maxlen=2)
    node._tracker_debug_dropped = 0
    node._control_debug_dropped = 0
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=123_000_000_000)
    )

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(json.loads(message.data))

    node._tracker_debug_pub = Publisher()
    node._control_debug_pub = Publisher()

    MODULE.TouchObservationNode._queue_debug_payload(
        node, "tracker", {"image_stamp": 122.9, "result": {"status": "TRACKING"}}
    )

    assert len(node._tracker_debug_queue) == 1
    assert node._tracker_debug_pub.messages == []

    MODULE.TouchObservationNode._debug_publish_tick(node)

    assert len(node._tracker_debug_queue) == 0
    assert node._tracker_debug_pub.messages[0]["kind"] == "TRACKER_TICK"
    assert node._tracker_debug_pub.messages[0]["queue_dropped_total"] == 0


def test_automatic_approach_waits_for_transient_stale_arm_state():
    node = object.__new__(MODULE.TouchObservationNode)
    node._lock = threading.RLock()
    node._active = True
    node._target = {}
    node._arm_target_confirmed = True
    node._arm_target = {}
    node._arm_frozen = {}
    node._state = "ARM_TARGET_CONFIRMED"
    node._automatic_approach = True
    node._automatic_reacquire_due = None
    node._arm_gate = lambda: (False, "Arm Dashboard state is missing or stale")
    transitions = []
    pauses = []
    node._set_state = lambda state, message: transitions.append((state, message))
    node._pause_automatic_approach = (
        lambda state, message: pauses.append((state, message))
    )

    MODULE.TouchObservationNode._plan_approach_step(node)

    assert node._automatic_approach is True
    assert node._automatic_reacquire_due is not None
    assert transitions[0][0] == "AUTO_APPROACH_ARM_STATE_WAIT"
    assert pauses == []


def test_automatic_approach_does_not_retry_non_stale_arm_fault():
    node = object.__new__(MODULE.TouchObservationNode)
    node._active = True
    node._target = {}
    node._arm_target_confirmed = True
    node._arm_target = {}
    node._arm_frozen = {}
    node._state = "ARM_TARGET_CONFIRMED"
    node._automatic_approach = True
    node._arm_gate = lambda: (False, "Arm must be in code2real&sim mode")
    pauses = []
    node._pause_automatic_approach = (
        lambda state, message: pauses.append((state, message))
    )

    MODULE.TouchObservationNode._plan_approach_step(node)

    assert pauses == [
        ("APPROACH_BLOCKED", "Arm must be in code2real&sim mode")
    ]


def test_tracker_frame_artifact_explicitly_reports_missing_geometry():
    frame = {
        "geometry_ready": False,
        "rgb": np.zeros((12, 16, 3), dtype=np.uint8),
    }
    evidence = MODULE.arm_frame_geometry_evidence(frame)
    assert evidence == {
        "geometry_ready": False,
        "camera_info": None,
        "tf_base_link_from_arm_camera": None,
    }


def test_full_arm_frame_artifact_keeps_camera_info_and_tf():
    frame = {
        "geometry_ready": True,
        "camera_matrix": np.array(
            [[500.0, 0.0, 320.0], [0.0, 501.0, 240.0], [0.0, 0.0, 1.0]]
        ),
        "distortion_model": "plumb_bob",
        "distortion": [0.1, 0.0, 0.0, 0.0, 0.0],
        "base_from_camera_translation": np.array([0.1, 0.2, 0.3]),
        "base_from_camera_rotation": np.eye(3),
    }
    evidence = MODULE.arm_frame_geometry_evidence(frame)
    assert evidence["geometry_ready"] is True
    assert evidence["camera_info"]["k"][0] == 500.0
    np.testing.assert_allclose(
        evidence["tf_base_link_from_arm_camera"]["translation_m"],
        [0.1, 0.2, 0.3],
    )


def test_preview_and_execute_both_protect_step_from_semantic_recovery():
    assert MODULE.approach_motion_request_in_flight("preview-id", None)
    assert MODULE.approach_motion_request_in_flight(None, "execute-id")
    assert MODULE.approach_motion_request_in_flight("preview-id", "execute-id")
    assert not MODULE.approach_motion_request_in_flight(None, None)
