from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np


MODULE_PATH = Path(__file__).parents[3] / "tools" / "touch_anything" / "offline_regression.py"
SPEC = importlib.util.spec_from_file_location("touch_offline_regression", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value), encoding="utf-8")


def test_audit_session_checks_saved_geometry_and_motor_evidence(tmp_path: Path) -> None:
    session = tmp_path / "observation_fixture"
    session.mkdir()
    rng = np.random.default_rng(7)
    rgb = rng.integers(0, 255, size=(180, 240, 3), dtype=np.uint8)
    mask = np.zeros((180, 240), dtype=np.uint8)
    mask[45:145, 65:185] = 255
    cv2.imwrite(str(session / "color_rgb.png"), rgb)
    cv2.imwrite(str(session / "sam_mask.png"), mask)
    _write_json(
        session / "observation.json",
        {"schema_version": 1, "files": {"rgb": "color_rgb.png", "mask": "sam_mask.png"}},
    )

    for index in range(2):
        identifier = f"fixture_{index}"
        rgb_name = f"arm_rgb_{identifier}.png"
        mask_name = f"arm_sam_mask_{identifier}.png"
        cv2.imwrite(str(session / rgb_name), rgb)
        cv2.imwrite(str(session / mask_name), mask)
        _write_json(
            session / f"arm_observation_{identifier}.json",
            {
                "schema_version": 1,
                "image_stamp": float(index),
                "target": {"centroid_uv": [124.5, 94.5]},
                "files": {"rgb": rgb_name, "mask": mask_name},
            },
        )
    _write_json(
        session / "motor_evidence.json",
        {
            "schema_version": 1,
            "far_baseline": {"source": "fixture"},
            "approach_steps": [
                {
                    "step_index": 1,
                    "step_outcome": "SUCCEEDED",
                    "planned_approach": {"step_kind": "FIXTURE"},
                    "cartesian_feedback_at_completion": {"waypoint_error_norm_m": 0.004},
                }
            ],
        },
    )

    report = MODULE.audit_session(
        session,
        {"min_arm_frames": 2, "min_approach_steps": 1, "require_motor_evidence": True},
    )
    assert report["passed"] is True
    assert report["arm"]["centroid_recompute_max_error_px"] == 0.0
    assert report["motor"]["outcomes"] == {"SUCCEEDED": 1}
    assert report["motor"]["cartesian_feedback"]["waypoint_error_m_max"] == 0.004
    assert report["tracker_replay"]["frames"] == 2


def test_audit_session_fails_missing_manifest_requirements(tmp_path: Path) -> None:
    session = tmp_path / "observation_empty"
    session.mkdir()
    report = MODULE.audit_session(
        session,
        {"min_arm_frames": 1, "require_motor_evidence": True},
        replay_tracker=False,
    )
    assert report["passed"] is False
    assert any("observation.json" in message for message in report["errors"])
    assert any("motor_evidence" in message for message in report["errors"])
