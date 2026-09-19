from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).parents[3]
    / "tools"
    / "touch_anything"
    / "replay_continuous_tracker_bag.py"
)
SPEC = importlib.util.spec_from_file_location(
    "touch_continuous_tracker_bag_replay", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _translation(x: float, y: float, z: float) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, 3] = [x, y, z]
    return result


def test_resolve_transform_composes_forward_and_inverse_edges() -> None:
    edges = {
        ("base_link", "wrist"): _translation(1.0, 2.0, 3.0),
        ("wrist", "camera"): _translation(0.1, 0.2, 0.3),
    }

    base_from_camera = MODULE.resolve_transform(edges, "base_link", "camera")
    camera_from_base = MODULE.resolve_transform(edges, "camera", "base_link")

    assert base_from_camera is not None
    assert camera_from_base is not None
    assert base_from_camera[:3, 3] == pytest.approx([1.1, 2.2, 3.3])
    assert camera_from_base @ base_from_camera == pytest.approx(np.eye(4))


def test_shadow_control_consumes_fresh_tracking_and_tf_at_ten_hz() -> None:
    start = 100.0
    tracker_trace = [
        {
            "stamp": start + index * 0.05,
            "status": "TRACKING",
            "centroid_uv": [330.0 + index * 0.1, 245.0],
            "processing_ms": 3.0,
        }
        for index in range(21)
    ]
    # A level optical frame: camera +Z looks along base +X, camera +X points
    # toward base -Y, and camera +Y points toward base -Z.
    base_from_camera = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    tf_trace = [
        (start + index * 0.1, base_from_camera)
        for index in range(11)
    ]
    record = {
        "camera_info": {
            "k": [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        "target": {"tracking_range_m": 0.40},
    }

    report = MODULE.simulate_shadow_control(
        tracker_trace, record, tf_trace, control_hz=10.0
    )

    assert report["ready"] is True
    assert report["ready_fraction"] == pytest.approx(1.0)
    assert report["stale_input_ticks"] == 0
    assert report["nonfinite_output_ticks"] == 0
    assert report["repeated_observation_ticks"] == 0
    assert report["compute_ms_p95"] < report["period_ms"] * 0.25
    assert all(
        np.all(np.isfinite(sample["bounded_delta_yz_m"]))
        for sample in report["trace"]
    )
