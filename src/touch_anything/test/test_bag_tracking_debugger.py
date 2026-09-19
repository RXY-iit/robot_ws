from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np


MODULE_PATH = (
    Path(__file__).parents[3]
    / "tools"
    / "touch_anything"
    / "bag_tracking_debugger.py"
)
SPEC = importlib.util.spec_from_file_location("touch_bag_tracking_debugger", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _frame(index: int, status: str, method: str, stamp: float):
    return MODULE.FrameAnalysis(
        row_id=index,
        bag_stamp=stamp,
        image_stamp=stamp,
        result={
            "status": status,
            "method": method,
            "frame_index": index,
            "centroid_uv": [10.0, 10.0] if status != "LOST" else None,
            "mask_fraction": 0.1 if status != "LOST" else 0.0,
            "tracked_points": 20 if status != "LOST" else 0,
            "inlier_ratio": 1.0 if status != "LOST" else 0.0,
            "processing_ms": 2.0,
            "feature_points_uv": [[8.0, 8.0]],
            "previous_feature_points_uv": [[7.0, 8.0]],
            "reason": "fixture loss" if status == "LOST" else "",
        },
        contours=[[[4, 4], [16, 4], [16, 16], [4, 16]]],
    )


def test_normal_initial_handoff_is_not_reported_as_anomaly() -> None:
    events = MODULE.build_events([
        _frame(0, "INITIALIZED", "SAM_HUMAN_CONFIRMED", 10.0),
        _frame(1, "TRACKING", "PYRAMID_KLT_RANSAC", 10.067),
    ])
    assert [(event.kind, event.frame_index) for event in events] == [("START", 0)]


def test_first_lost_frame_is_a_breakpoint_with_reason() -> None:
    events = MODULE.build_events([
        _frame(0, "INITIALIZED", "SAM_HUMAN_CONFIRMED", 10.0),
        _frame(1, "TRACKING", "PYRAMID_KLT_RANSAC", 10.067),
        _frame(2, "LOST", "SEMANTIC_RECOVERY_REQUIRED", 10.134),
    ])
    lost = [event for event in events if event.severity == "error"]
    assert len(lost) == 1
    assert lost[0].frame_index == 2
    assert "fixture loss" in lost[0].label


def test_annotation_renders_mask_flow_and_status_without_mutating_rgb() -> None:
    rgb = np.zeros((40, 50, 3), dtype=np.uint8)
    before = rgb.copy()
    output = MODULE.annotate_frame(rgb, _frame(1, "TRACKING", "KLT", 10.0))
    assert output.shape == rgb.shape
    assert np.array_equal(rgb, before)
    assert np.count_nonzero(output) > 0
