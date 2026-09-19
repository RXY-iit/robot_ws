"""Pure validation and rate limiting for the guarded visual-servo stream."""
from __future__ import annotations

import math
import time
from typing import Any


PROTOCOL = "SO101_VISUAL_SERVO_V1"
OPS = {"START", "UPDATE", "STOP"}
MAX_ACTION_HORIZON_STEPS = 20


def validate_positions(
    joint_specs: list[dict[str, Any]], positions: object
) -> tuple[dict[str, float], list[str]]:
    expected = [str(spec["name"]) for spec in joint_specs]
    if not isinstance(positions, dict):
        return {}, ["positions must be an object"]
    if set(positions) != set(expected):
        return {}, [
            "positions must contain exactly configured joints; "
            f"expected={expected}, received={sorted(str(key) for key in positions)}"
        ]
    normalized: dict[str, float] = {}
    errors: list[str] = []
    for spec in joint_specs:
        name = str(spec["name"])
        try:
            value = float(positions[name])
        except (TypeError, ValueError):
            errors.append(f"{name}: position must be numeric")
            continue
        lower, upper = float(spec["lower"]), float(spec["upper"])
        if not math.isfinite(value):
            errors.append(f"{name}: position is non-finite")
        elif not lower <= value <= upper:
            errors.append(f"{name}: {value:.6f} outside [{lower:.6f}, {upper:.6f}]")
        else:
            normalized[name] = value
    return normalized, errors


def validate_command(
    payload: object, joint_specs: list[dict[str, Any]]
) -> tuple[dict[str, Any], list[str]]:
    if not isinstance(payload, dict):
        return {}, ["command must be a JSON object"]
    errors: list[str] = []
    if str(payload.get("protocol", "")) != PROTOCOL:
        errors.append(f"protocol must be {PROTOCOL}")
    op = str(payload.get("op", "")).upper()
    if op not in OPS:
        errors.append(f"op must be one of {sorted(OPS)}")
    session_id = str(payload.get("session_id", "")).strip()
    if len(session_id) < 8 or len(session_id) > 128:
        errors.append("session_id length must be 8..128")
    try:
        sequence = int(payload.get("sequence"))
    except (TypeError, ValueError):
        sequence = -1
        errors.append("sequence must be an integer")
    if sequence < 0:
        errors.append("sequence must be non-negative")
    positions: dict[str, float] = {}
    action_horizon: list[dict[str, float]] = []
    horizon_step_period_sec = 0.0
    horizon_plan_id = ""
    observation_age_sec = 0.0
    prediction_lead_sec = 0.0
    if op in {"START", "UPDATE"}:
        positions, position_errors = validate_positions(
            joint_specs, payload.get("positions")
        )
        errors.extend(position_errors)
        raw_horizon = payload.get("action_horizon")
        if raw_horizon is not None:
            if not isinstance(raw_horizon, list) or not (
                2 <= len(raw_horizon) <= MAX_ACTION_HORIZON_STEPS
            ):
                errors.append(
                    "action_horizon must contain 2.."
                    f"{MAX_ACTION_HORIZON_STEPS} joint targets"
                )
            else:
                for index, point in enumerate(raw_horizon):
                    normalized, point_errors = validate_positions(
                        joint_specs, point
                    )
                    if point_errors:
                        errors.extend(
                            f"action_horizon[{index}]: {error}"
                            for error in point_errors
                        )
                    action_horizon.append(normalized)
            try:
                horizon_step_period_sec = float(
                    payload.get("horizon_step_period_sec")
                )
            except (TypeError, ValueError):
                horizon_step_period_sec = math.nan
            if not math.isfinite(horizon_step_period_sec) or not (
                0.02 <= horizon_step_period_sec <= 0.20
            ):
                errors.append(
                    "horizon_step_period_sec must be within [0.02, 0.20]"
                )
            horizon_plan_id = str(payload.get("horizon_plan_id", "")).strip()
            if not 8 <= len(horizon_plan_id) <= 128:
                errors.append("horizon_plan_id length must be 8..128")
            if (
                action_horizon
                and positions
                and all(set(point) == set(positions) for point in action_horizon)
            ):
                terminal = action_horizon[-1]
                if any(
                    abs(float(terminal[name]) - float(positions[name])) > 1.0e-6
                    for name in positions
                ):
                    errors.append(
                        "positions must equal the terminal action_horizon target"
                    )
            for key, maximum in (
                ("observation_age_sec", 2.0),
                ("prediction_lead_sec", 0.5),
            ):
                try:
                    value = float(payload.get(key, 0.0))
                except (TypeError, ValueError):
                    value = math.nan
                if not math.isfinite(value) or not 0.0 <= value <= maximum:
                    errors.append(f"{key} must be within [0, {maximum}]")
                elif key == "observation_age_sec":
                    observation_age_sec = value
                else:
                    prediction_lead_sec = value
    try:
        issued_ros_sec = float(payload.get("issued_ros_sec"))
    except (TypeError, ValueError):
        issued_ros_sec = math.nan
        errors.append("issued_ros_sec must be numeric")
    if not math.isfinite(issued_ros_sec) or issued_ros_sec <= 0.0:
        errors.append("issued_ros_sec must be finite and positive")
    return {
        "protocol": PROTOCOL,
        "op": op,
        "session_id": session_id,
        "sequence": sequence,
        "positions": positions,
        "action_horizon": action_horizon,
        "horizon_step_period_sec": horizon_step_period_sec,
        "horizon_plan_id": horizon_plan_id,
        "observation_age_sec": observation_age_sec,
        "prediction_lead_sec": prediction_lead_sec,
        "issued_ros_sec": issued_ros_sec,
        "tracker_stamp": payload.get("tracker_stamp"),
        "reason": str(payload.get("reason", "")),
    }, errors


def select_action_horizon_target(
    action_horizon: list[dict[str, float]],
    *, elapsed_since_issue_sec: float, step_period_sec: float,
) -> tuple[dict[str, float], int]:
    """Select the time-indexed target from a replaceable action chunk."""
    if not action_horizon:
        raise ValueError("action horizon is empty")
    if not math.isfinite(elapsed_since_issue_sec) or elapsed_since_issue_sec < 0.0:
        raise ValueError("elapsed_since_issue_sec must be finite and non-negative")
    if not math.isfinite(step_period_sec) or step_period_sec <= 0.0:
        raise ValueError("step_period_sec must be finite and positive")
    index = min(
        len(action_horizon) - 1,
        int(math.floor(elapsed_since_issue_sec / step_period_sec)),
    )
    return dict(action_horizon[index]), index


def validate_sequence(
    *, op: str, session_id: str, sequence: int,
    active_session_id: str | None, last_sequence: int,
) -> str | None:
    if op == "START":
        if active_session_id is not None:
            return f"visual-servo session {active_session_id} is already active"
        if sequence != 0:
            return "START sequence must be zero"
        return None
    if active_session_id is None:
        return "no visual-servo session is active"
    if session_id != active_session_id:
        return f"session mismatch: active={active_session_id}, received={session_id}"
    if sequence <= last_sequence:
        return f"sequence {sequence} is not newer than {last_sequence}"
    return None


def velocity_limited_positions(
    previous: dict[str, float],
    requested: dict[str, float],
    joint_specs: list[dict[str, Any]],
    maximum_velocity_rad_s: dict[str, float],
    elapsed_sec: float,
) -> dict[str, float]:
    if not math.isfinite(elapsed_sec) or elapsed_sec <= 0.0:
        raise ValueError("elapsed_sec must be finite and positive")
    output: dict[str, float] = {}
    for spec in joint_specs:
        name = str(spec["name"])
        motor = str(spec.get("motor_name", name))
        speed = float(maximum_velocity_rad_s[motor])
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(f"invalid velocity limit for {motor}: {speed}")
        old = float(previous[name])
        wanted = float(requested[name])
        maximum_step = speed * elapsed_sec
        output[name] = min(old + maximum_step, max(old - maximum_step, wanted))
    return output


def watchdog_expired(
    last_update_monotonic: float | None,
    watchdog_sec: float,
    *, now_monotonic: float | None = None,
) -> bool:
    if last_update_monotonic is None:
        return True
    now = time.monotonic() if now_monotonic is None else float(now_monotonic)
    return now - float(last_update_monotonic) > float(watchdog_sec)
