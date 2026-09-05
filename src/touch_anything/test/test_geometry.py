import importlib.util
from pathlib import Path

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
