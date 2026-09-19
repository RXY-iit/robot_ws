import importlib.util
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "target_tracker.py"
SPEC = importlib.util.spec_from_file_location("target_tracker", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
ContinuousTargetTracker = MODULE.ContinuousTargetTracker
TrackerConfig = MODULE.TrackerConfig


def textured_button() -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(7)
    rgb = np.full((240, 320, 3), 230, dtype=np.uint8)
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[70:175, 95:225] = 1
    texture = rng.integers(10, 245, size=(105, 130, 3), dtype=np.uint8)
    rgb[70:175, 95:225] = texture
    cv2.rectangle(rgb, (95, 70), (224, 174), (10, 200, 40), 4)
    cv2.putText(
        rgb, "2", (135, 150), cv2.FONT_HERSHEY_SIMPLEX, 2.2,
        (250, 250, 250), 5, cv2.LINE_AA,
    )
    return rgb, mask


def test_pyramid_klt_propagates_confirmed_button_mask():
    rgb, mask = textured_button()
    tracker = ContinuousTargetTracker(TrackerConfig(minimum_tracks=8))
    initial = tracker.initialize(rgb, mask, excluded_tip_uv=(160.0, 215.0))
    transform = np.asarray([[1.0, 0.0, 9.0], [0.0, 1.0, -6.0]], dtype=np.float32)
    moved = cv2.warpAffine(rgb, transform, (320, 240))

    result = tracker.update(moved, excluded_tip_uv=(160.0, 215.0))

    assert initial["status"] == "INITIALIZED"
    assert result["status"] == "TRACKING"
    assert result["method"] == "PYRAMID_KLT_RANSAC"
    assert result["tracked_points"] >= 8
    assert len(initial["feature_points_uv"]) >= 8
    assert len(result["feature_points_uv"]) == result["tracked_points"]
    assert len(result["previous_feature_points_uv"]) == result["tracked_points"]
    np.testing.assert_allclose(
        result["centroid_uv"],
        np.asarray(initial["centroid_uv"]) + np.asarray([9.0, -6.0]),
        atol=1.5,
    )


def test_blank_frame_requests_semantic_recovery_instead_of_guessing():
    rgb, mask = textured_button()
    tracker = ContinuousTargetTracker(TrackerConfig(minimum_tracks=8))
    tracker.initialize(rgb, mask)

    result = tracker.update(np.full_like(rgb, 230))

    assert result["status"] == "LOST"
    assert result["method"] == "SEMANTIC_RECOVERY_REQUIRED"
    assert result["centroid_uv"] is None
    assert "KLT:" in result["reason"]
    assert "ORB:" in result["reason"]


def test_human_confirmed_semantic_recovery_preserves_immutable_identity():
    rgb, mask = textured_button()
    tracker = ContinuousTargetTracker(TrackerConfig(minimum_tracks=8))
    tracker.initialize(rgb, mask)
    original_reference = tracker.reference_rgb.copy()
    moved = np.roll(rgb, 5, axis=1)
    moved_mask = np.roll(mask, 5, axis=1)

    result = tracker.initialize(
        moved, moved_mask, preserve_identity_reference=True
    )

    assert result["status"] == "INITIALIZED"
    assert result["method"] == "SAM_HUMAN_CONFIRMED"
    np.testing.assert_array_equal(tracker.reference_rgb, original_reference)


def test_six_feature_floor_tracks_a_small_low_texture_target():
    rgb, mask = textured_button()
    # Bound the detector to the same small feature population observed on the
    # first 2026-09-10 button selection. The default six-track floor must let
    # the target start and retain redundant partial-affine correspondences.
    config = TrackerConfig(max_features=9)
    assert config.minimum_tracks == 6
    tracker = ContinuousTargetTracker(config)
    initial = tracker.initialize(rgb, mask)
    transform = np.asarray([[1.0, 0.0, 4.0], [0.0, 1.0, -3.0]], dtype=np.float32)
    moved = cv2.warpAffine(rgb, transform, (320, 240))

    result = tracker.update(moved)

    assert 6 <= initial["tracked_points"] <= 9
    assert result["status"] == "TRACKING"
    assert result["tracked_points"] >= 6
