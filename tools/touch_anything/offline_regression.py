#!/usr/bin/env python3
"""Read-only regression checks for saved Touch Anything evidence.

The saved observation directories contain event snapshots, not a continuous
camera stream.  This tool therefore checks artifact integrity, recomputes mask
geometry, summarizes plan/execution evidence and performs a *sparse* KLT/ORB
replay.  It intentionally does not publish ROS commands or open arm hardware.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any

import cv2
import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_ROOT = WORKSPACE / "debug-output" / "touch-anything"
DEFAULT_MANIFEST = Path(__file__).with_name("regression_manifest.yaml")


def _load_tracker_module():
    path = WORKSPACE / "src" / "touch_anything" / "scripts" / "target_tracker.py"
    spec = importlib.util.spec_from_file_location("touch_anything_target_tracker", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import tracker from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("top level must be an object")
    return value


def _finite_vector(value: Any, length: int) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == length
        and all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in value)
    )


def _mask_centroid(mask: np.ndarray) -> tuple[float, float] | None:
    binary = (mask > 0).astype(np.uint8)
    moments = cv2.moments(binary, binaryImage=True)
    if moments["m00"] <= 0.0:
        return None
    return (
        float(moments["m10"] / moments["m00"]),
        float(moments["m01"] / moments["m00"]),
    )


def _arm_records(session: Path) -> list[tuple[Path, dict[str, Any]]]:
    records: list[tuple[Path, dict[str, Any]]] = []
    for path in session.glob("arm_observation_*.json"):
        # arm_observation.json is the mutable latest alias, not an event record.
        try:
            data = _read_json(path)
            stamp = float(data.get("image_stamp", 0.0))
        except Exception:
            stamp = float("inf")
            data = {}
        records.append((path, data))
    return sorted(records, key=lambda item: (float(item[1].get("image_stamp", 0.0)), item[0].name))


def audit_session(
    session: Path,
    expectations: dict[str, Any] | None = None,
    *,
    replay_tracker: bool = True,
) -> dict[str, Any]:
    """Audit one saved session without modifying it."""
    expectations = expectations or {}
    report: dict[str, Any] = {
        "session": str(session),
        "coverage": "SPARSE_EVENT_FRAMES",
        "errors": [],
        "warnings": [],
        "d435": {},
        "arm": {},
        "motor": {},
        "tracker_replay": {"attempted": False},
    }
    errors: list[str] = report["errors"]
    warnings: list[str] = report["warnings"]
    if not session.is_dir():
        errors.append("session directory does not exist")
        return report

    observation_path = session / "observation.json"
    if not observation_path.exists():
        errors.append("missing observation.json")
        observation: dict[str, Any] = {}
    else:
        try:
            observation = _read_json(observation_path)
        except Exception as exc:
            errors.append(f"invalid observation.json: {exc}")
            observation = {}
    report["d435"]["schema_version"] = observation.get("schema_version")
    d435_files = observation.get("files", {}) if isinstance(observation.get("files"), dict) else {}
    missing_d435: list[str] = []
    for key, filename in d435_files.items():
        if isinstance(filename, str) and not (session / filename).exists():
            missing_d435.append(f"{key}:{filename}")
    if missing_d435:
        errors.append("missing D435 artifacts: " + ", ".join(missing_d435))
    report["d435"]["referenced_files"] = len(d435_files)
    report["d435"]["missing_files"] = missing_d435

    records = _arm_records(session)
    valid_frames: list[tuple[np.ndarray, np.ndarray, dict[str, Any], str]] = []
    centroid_errors: list[float] = []
    bad_arm_files: list[str] = []
    for json_path, data in records:
        if not data:
            errors.append(f"invalid arm record: {json_path.name}")
            continue
        files = data.get("files", {})
        rgb_name = files.get("rgb") if isinstance(files, dict) else None
        mask_name = files.get("mask") if isinstance(files, dict) else None
        if not isinstance(rgb_name, str) or not isinstance(mask_name, str):
            bad_arm_files.append(f"{json_path.name}: missing rgb/mask references")
            continue
        rgb_bgr = cv2.imread(str(session / rgb_name), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(session / mask_name), cv2.IMREAD_GRAYSCALE)
        if rgb_bgr is None or mask is None:
            bad_arm_files.append(f"{json_path.name}: unreadable rgb/mask")
            continue
        if rgb_bgr.shape[:2] != mask.shape:
            bad_arm_files.append(f"{json_path.name}: rgb/mask shape mismatch")
            continue
        computed = _mask_centroid(mask)
        recorded = data.get("target", {}).get("centroid_uv") if isinstance(data.get("target"), dict) else None
        if computed is not None and _finite_vector(recorded, 2):
            centroid_errors.append(float(np.linalg.norm(np.asarray(computed) - np.asarray(recorded))))
        valid_frames.append((cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB), mask, data, json_path.name))
    if bad_arm_files:
        errors.extend(bad_arm_files)
    report["arm"] = {
        "event_records": len(records),
        "valid_rgb_mask_frames": len(valid_frames),
        "centroid_recompute_max_error_px": max(centroid_errors, default=None),
    }
    if centroid_errors and max(centroid_errors) > 3.0:
        errors.append(f"saved mask centroid mismatch: max {max(centroid_errors):.2f}px")

    motor_path = session / "motor_evidence.json"
    steps: list[dict[str, Any]] = []
    if motor_path.exists():
        try:
            motor = _read_json(motor_path)
            raw_steps = motor.get("approach_steps", [])
            if isinstance(raw_steps, list):
                steps = [item for item in raw_steps if isinstance(item, dict)]
            report["motor"] = {
                "present": True,
                "far_baseline_present": isinstance(motor.get("far_baseline"), dict),
                "approach_steps": len(steps),
                "outcomes": {},
                "step_kinds": {},
            }
            cartesian_errors: list[float] = []
            cartesian_missing = 0
            for step in steps:
                outcome = str(step.get("step_outcome", "UNKNOWN"))
                report["motor"]["outcomes"][outcome] = report["motor"]["outcomes"].get(outcome, 0) + 1
                planned = step.get("planned_approach", {})
                kind = str(planned.get("step_kind", "UNKNOWN")) if isinstance(planned, dict) else "UNKNOWN"
                report["motor"]["step_kinds"][kind] = report["motor"]["step_kinds"].get(kind, 0) + 1
                feedback = step.get("cartesian_feedback_at_completion")
                if not isinstance(feedback, dict):
                    cartesian_missing += 1
                    continue
                error = feedback.get("waypoint_error_norm_m")
                if isinstance(error, (int, float)) and math.isfinite(float(error)):
                    cartesian_errors.append(float(error))
                else:
                    cartesian_missing += 1
            report["motor"]["cartesian_feedback"] = {
                "available_steps": len(cartesian_errors),
                "missing_steps": cartesian_missing,
                "waypoint_error_m_median": statistics.median(cartesian_errors) if cartesian_errors else None,
                "waypoint_error_m_max": max(cartesian_errors, default=None),
            }
        except Exception as exc:
            errors.append(f"invalid motor_evidence.json: {exc}")
            report["motor"] = {"present": True, "invalid": True}
    else:
        report["motor"] = {"present": False, "approach_steps": 0}

    if replay_tracker and len(valid_frames) >= 2:
        tracker_module = _load_tracker_module()
        tracker = tracker_module.ContinuousTargetTracker()
        tracker_results: list[dict[str, Any]] = []
        first_rgb, first_mask, _, first_name = valid_frames[0]
        try:
            initialized = tracker.initialize(first_rgb, first_mask)
            tracker_results.append({"record": first_name, **initialized})
            for rgb, _, _, name in valid_frames[1:]:
                tracker_results.append({"record": name, **tracker.update(rgb)})
        except Exception as exc:
            warnings.append(f"sparse tracker replay stopped: {exc}")
        times = [float(item.get("processing_ms", 0.0)) for item in tracker_results[1:]]
        statuses: dict[str, int] = {}
        for item in tracker_results:
            status = str(item.get("status", "UNKNOWN"))
            statuses[status] = statuses.get(status, 0) + 1
        report["tracker_replay"] = {
            "attempted": True,
            "interpretation": "diagnostic only: source frames are sparse event snapshots",
            "frames": len(tracker_results),
            "statuses": statuses,
            "processing_ms_median": statistics.median(times) if times else None,
            "processing_ms_max": max(times, default=None),
            "results": tracker_results,
        }
    elif replay_tracker:
        warnings.append("fewer than two valid Arm Camera event frames; tracker replay skipped")

    min_arm = int(expectations.get("min_arm_frames", 0))
    min_steps = int(expectations.get("min_approach_steps", 0))
    if len(valid_frames) < min_arm:
        errors.append(f"expected at least {min_arm} Arm Camera frames, found {len(valid_frames)}")
    if len(steps) < min_steps:
        errors.append(f"expected at least {min_steps} approach steps, found {len(steps)}")
    if expectations.get("require_motor_evidence") and not motor_path.exists():
        errors.append("manifest requires motor_evidence.json")
    report["passed"] = not errors
    return report


def _load_manifest(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict) or not isinstance(value.get("sessions", []), list):
        raise ValueError("manifest must contain a sessions list")
    return value


def _discover(root: Path) -> list[Path]:
    return sorted(
        (path for path in root.glob("observation_*") if path.is_dir()),
        key=lambda path: path.stat().st_mtime,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--session", type=Path, action="append", default=[])
    parser.add_argument("--all", action="store_true", help="audit every discovered session")
    parser.add_argument("--max-sessions", type=int, default=0, help="newest N when used with --all")
    parser.add_argument("--no-tracker-replay", action="store_true")
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--list", action="store_true", help="list discovered sessions and exit")
    args = parser.parse_args()

    discovered = _discover(args.root)
    if args.list:
        for path in discovered:
            print(path)
        return 0

    selections: list[tuple[Path, dict[str, Any], str]] = []
    if args.session:
        selections = [((path if path.is_absolute() else WORKSPACE / path), {}, "manual") for path in args.session]
    elif args.all:
        chosen = discovered[-args.max_sessions :] if args.max_sessions > 0 else discovered
        selections = [(path, {}, "auto-discovered") for path in chosen]
    else:
        manifest = _load_manifest(args.manifest)
        for entry in manifest.get("sessions", []):
            if not isinstance(entry, dict) or not isinstance(entry.get("id"), str):
                continue
            selections.append(
                (args.root / entry["id"], entry.get("expect", {}), str(entry.get("purpose", "manifest")))
            )
    if not selections:
        print("No sessions selected. Use --all, --session, or populate the manifest.", file=sys.stderr)
        return 2

    reports = []
    for path, expect, purpose in selections:
        report = audit_session(path, expect, replay_tracker=not args.no_tracker_replay)
        report["purpose"] = purpose
        reports.append(report)
        status = "PASS" if report["passed"] else "FAIL"
        replay = report["tracker_replay"]
        replay_text = "skipped"
        if replay.get("attempted"):
            replay_text = f"{replay.get('statuses', {})}, median={replay.get('processing_ms_median')}ms"
        print(
            f"[{status}] {path.name}: arm={report['arm'].get('valid_rgb_mask_frames', 0)}, "
            f"steps={report['motor'].get('approach_steps', 0)}, sparse_tracker={replay_text}"
        )
        for message in report["errors"]:
            print(f"  ERROR: {message}")
        for message in report["warnings"]:
            print(f"  WARN: {message}")

    output = {
        "schema_version": 1,
        "coverage": "SPARSE_EVENT_FRAMES",
        "limitations": [
            "does not reproduce the continuous 25 Hz camera stream",
            "does not prove ROS timing, fresh post-motion TF, USB transport or real mechanics",
            "does not publish commands or execute the Arm",
        ],
        "sessions": reports,
        "passed": all(report["passed"] for report in reports),
    }
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(output, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"JSON report: {args.json_output}")
    return 0 if output["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
