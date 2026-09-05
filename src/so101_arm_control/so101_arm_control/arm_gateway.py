from __future__ import annotations

import json
import math
from pathlib import Path
import time
import uuid

import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from so101_arm_control.arm_visual_joint_state_calibrator import corrected_position


def validate_target(joint_specs: list[dict], positions: object) -> tuple[dict[str, float], list[str]]:
    """Validate a complete named joint target without silently filling values."""
    if not isinstance(positions, dict):
        return {}, ["positions must be an object"]

    expected = {str(spec["name"]) for spec in joint_specs}
    provided = {str(name) for name in positions}
    errors: list[str] = []
    missing = sorted(expected - provided)
    extra = sorted(provided - expected)
    if missing:
        errors.append("missing joints: " + ", ".join(missing))
    if extra:
        errors.append("unknown joints: " + ", ".join(extra))

    normalized: dict[str, float] = {}
    for spec in joint_specs:
        name = str(spec["name"])
        if name not in positions:
            continue
        try:
            value = float(positions[name])
        except (TypeError, ValueError):
            errors.append(f"{name}: value is not numeric")
            continue
        if not math.isfinite(value):
            errors.append(f"{name}: value must be finite")
            continue
        lower = float(spec["lower"])
        upper = float(spec["upper"])
        if value < lower or value > upper:
            errors.append(f"{name}: {value:.5f} outside [{lower:.5f}, {upper:.5f}]")
            continue
        normalized[name] = value
    return normalized, errors


def minimum_target_duration(
    joint_specs: list[dict],
    actual: dict[str, float],
    target: dict[str, float],
    max_velocity_rad_s: dict[str, float],
) -> float:
    required = 0.0
    for spec in joint_specs:
        name = str(spec["name"])
        motor_name = str(spec["motor_name"])
        velocity = float(max_velocity_rad_s[motor_name])
        if not math.isfinite(velocity) or velocity <= 0.0:
            raise ValueError(f"invalid max velocity for {motor_name}")
        required = max(required, abs(target[name] - actual[name]) / velocity)
    return required


def changed_command_limit_errors(
    effective_specs: list[dict],
    actual: dict[str, float],
    target: dict[str, float],
) -> list[str]:
    """Apply inset/learned limits only to joints that will actually be driven.

    A manually positioned, torque-off joint may sit a few ticks outside the
    command inset. It may remain there while another joint moves, but any new
    command for that joint must point inside the effective command range.
    """
    errors: list[str] = []
    for spec in effective_specs:
        name = str(spec["name"])
        if abs(target[name] - actual[name]) <= 0.002:
            continue
        lower = float(spec["lower"])
        upper = float(spec["upper"])
        if target[name] < lower or target[name] > upper:
            errors.append(
                f"{name}: commanded {target[name]:.5f} outside safe [{lower:.5f}, {upper:.5f}]"
            )
    return errors


class ArmGateway(Node):
    """Safe Dashboard adapter for mock or guarded real trajectory servers.

    This node has no serial or real-hardware code. It accepts JSON Dashboard
    requests, validates complete targets, publishes a separate candidate model,
    and sends a standard FollowJointTrajectory goal to the selected backend.
    """

    def __init__(self) -> None:
        super().__init__("so101_arm_gateway")
        default_config = Path(get_package_share_directory("so101_arm_control")) / "config" / "arm_mock.yaml"
        self.declare_parameter("config_file", str(default_config))
        config_path = Path(str(self.get_parameter("config_file").value))
        self._config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        self._mode = str(self._config.get("mode"))
        if self._mode not in {"MOCK", "REAL_DASHBOARD_TEST"}:
            raise RuntimeError(f"unsupported arm gateway mode: {self._mode}")
        self._is_real = self._mode == "REAL_DASHBOARD_TEST"
        real_enabled = bool(self._config.get("hardware", {}).get("real_hardware_enabled"))
        if real_enabled != self._is_real:
            raise RuntimeError("arm gateway mode and real_hardware_enabled disagree")

        self._joint_specs = list(self._config["joints"])
        self._control_modes = list(self._config.get("control_modes", []))
        self._control_mode = (
            str(self._config.get("default_control_mode", "real2sim"))
            if self._is_real
            else "mock"
        )
        self._valid_control_modes = {
            str(item["id"]) for item in self._control_modes
        } if self._is_real else {"mock"}
        if self._control_mode not in self._valid_control_modes:
            raise RuntimeError(f"invalid default control mode: {self._control_mode}")
        self._server_control_mode: str | None = None if self._is_real else "mock"
        self._joint_names = [str(spec["name"]) for spec in self._joint_specs]
        self._presets = {str(item["id"]): item for item in self._config.get("presets", [])}
        self._actual: dict[str, float] = {}
        self._candidate = {
            name: (0.85 if name == "gripper_joint" else 0.0) for name in self._joint_names
        }
        self._candidate_id: str | None = None
        self._candidate_valid = False
        self._candidate_errors: list[str] = ["candidate has not been previewed"]
        self._candidate_changed_joints: list[str] = []
        self._candidate_minimum_duration_sec = 0.5
        self._candidate_initialized = False
        self._last_joint_update: float | None = None
        self._last_controller_update: float | None = None
        self._controller_state = "STARTING"
        self._task_state = "IDLE"
        self._task_message = (
            "guarded real gateway started; waiting for fresh hardware feedback"
            if self._is_real
            else "mock gateway started; no hardware device is opened"
        )
        self._last_request_id: str | None = None
        self._seen_request_ids: list[str] = []
        self._active_goal = None
        self._hardware_status: dict = {}
        self._touch_state: dict = {}
        self._last_touch_state_update: float | None = None

        self._state_pub = self.create_publisher(String, "/arm/dashboard_state", 10)
        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._control_mode_pub = self.create_publisher(String, "/arm/control_mode", mode_qos)
        self._hold_command_pub = self.create_publisher(String, "/arm/hold_command", 10)
        self._touch_command_pub = self.create_publisher(String, "/touch_anything/command", 10)
        self._candidate_pub = self.create_publisher(JointState, "/arm/candidate_joint_states", 10)
        self._planned_path_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state",
            self._on_controller_state,
            10,
        )
        self.create_subscription(String, "/arm/dashboard_command", self._on_command, 20)
        self.create_subscription(String, "/arm/hardware_status", self._on_hardware_status, 10)
        self.create_subscription(String, "/touch_anything/state", self._on_touch_state, 10)
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.create_timer(0.1, self._publish_candidate)
        self.create_timer(0.2, self._publish_state)
        self.create_timer(0.5, self._publish_control_mode)
        if self._is_real:
            self.get_logger().warning(
                "SO101 gateway REAL_DASHBOARD_TEST: guarded multi-joint goals with "
                "velocity/current/stall protection"
            )
        else:
            self.get_logger().warning("SO101 gateway is MOCK ONLY; follower USB is never opened")

    def _on_joint_state(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in self._joint_names):
            return
        self._actual = {name: float(values[name]) for name in self._joint_names}
        self._last_joint_update = time.monotonic()
        if self._is_real and not self._candidate_initialized:
            self._candidate = dict(self._actual)
            self._candidate_id = None
            self._candidate_valid = False
            self._candidate_changed_joints = []
            self._candidate_minimum_duration_sec = 0.5
            self._candidate_errors = ["adjust one or more sliders, then Preview / Validate"]
            self._candidate_initialized = True
        elif self._is_real and self._control_mode != "code2real&sim":
            self._candidate = dict(self._actual)
            self._candidate_id = None
            self._candidate_valid = False
            self._candidate_changed_joints = []
            self._candidate_minimum_duration_sec = 0.5
            self._candidate_errors = [f"execution is disabled in {self._control_mode} mode"]

    def _on_hardware_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        observed = payload.get("control_mode")
        if isinstance(observed, str):
            self._server_control_mode = observed
        self._hardware_status = payload

    def _on_touch_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self._touch_state = payload
            self._last_touch_state_update = time.monotonic()

    def _effective_joint_specs(self) -> list[dict]:
        limits = self._hardware_status.get("protection", {}).get("command_limits_rad", {})
        effective: list[dict] = []
        for original in self._joint_specs:
            spec = dict(original)
            item = limits.get(str(spec["name"]))
            if isinstance(item, dict):
                try:
                    spec["lower"] = max(float(spec["lower"]), float(item["lower"]))
                    spec["upper"] = min(float(spec["upper"]), float(item["upper"]))
                except (KeyError, TypeError, ValueError):
                    pass
            effective.append(spec)
        return effective

    def _publish_control_mode(self) -> None:
        if self._is_real:
            self._control_mode_pub.publish(String(data=self._control_mode))

    def _on_controller_state(self, _msg: JointTrajectoryControllerState) -> None:
        self._last_controller_update = time.monotonic()
        self._controller_state = "ACTIVE"

    def _on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._set_event("BLOCKED", f"invalid JSON: {exc.msg}")
            return
        if not isinstance(payload, dict):
            self._set_event("BLOCKED", "command must be a JSON object")
            return
        if str(payload.get("mode", "")) != self._mode:
            self._set_event(
                "BLOCKED",
                f"command mode {payload.get('mode')!r} does not match gateway mode {self._mode}",
            )
            return

        request_id = str(payload.get("request_id", "")).strip()
        if not request_id:
            self._set_event("BLOCKED", "request_id is required")
            return
        if request_id in self._seen_request_ids:
            self._set_event("BLOCKED", f"duplicate request_id ignored: {request_id}")
            return
        self._seen_request_ids.append(request_id)
        self._seen_request_ids = self._seen_request_ids[-100:]
        self._last_request_id = request_id

        command = str(payload.get("command", "")).strip().lower()
        if command == "set_mode":
            self._set_control_mode(str(payload.get("control_mode", "")))
        elif command == "preview":
            self._preview(payload.get("positions"), source="slider")
        elif command == "diagnostic_preview":
            self._diagnostic_preview(payload.get("positions"))
        elif command == "select_preset":
            self._select_preset(str(payload.get("preset_id", "")))
        elif command == "execute_candidate":
            self._execute_candidate(str(payload.get("candidate_id", "")), payload.get("duration_sec", 2.0))
        elif command == "cancel":
            self._cancel_active_goal()
        elif command == "release_hold":
            self._release_hold()
        elif command == "reset_hold_fault":
            self._reset_hold_fault()
        elif command == "safe_shutdown":
            self._prepare_safe_shutdown()
        elif command == "trigger":
            self._trigger(str(payload.get("trigger_id", "")))
        else:
            self._set_event("BLOCKED", f"unsupported command: {command or '<empty>'}")

    def _set_control_mode(self, requested: str) -> None:
        if not self._is_real:
            self._set_event("BLOCKED", "runtime control modes are available only for real hardware")
            return
        if requested not in self._valid_control_modes:
            self._set_event("BLOCKED", f"unknown control mode: {requested}")
            return
        if self._active_goal is not None:
            self._set_event("BLOCKED", "cannot switch control mode while a goal is active")
            return
        mode_spec = next(item for item in self._control_modes if item["id"] == requested)
        if not bool(mode_spec.get("available")):
            self._set_event(
                "BLOCKED",
                f"control mode {requested} is not enabled by configuration",
            )
            return
        if requested == "teleoperate":
            teleoperation = self._hardware_status.get("teleoperation", {})
            if not bool(teleoperation.get("ready")):
                self._set_event(
                    "BLOCKED",
                    "teleoperate requires fresh calibrated leader feedback: "
                    + str(teleoperation.get("reason", "leader status unavailable")),
                )
                return
        self._control_mode = requested
        if requested != "code2real&sim" and bool(self._touch_state.get("active")):
            self._publish_touch_command("exit", "arm_control_mode_changed")
        self._candidate = dict(self._actual) if self._actual else dict(self._candidate)
        self._candidate_id = None
        self._candidate_valid = False
        self._candidate_changed_joints = []
        self._candidate_minimum_duration_sec = 0.5
        self._candidate_errors = (
            ["adjust one or more sliders, then Preview / Validate"]
            if requested == "code2real&sim"
            else [f"execution is disabled in {requested} mode"]
        )
        self._publish_control_mode()
        self._set_event("MODE_CHANGED", f"control mode set to {requested}")

    def _preview(self, positions: object, *, source: str) -> None:
        self._candidate_minimum_duration_sec = 0.5
        normalized, errors = validate_target(self._joint_specs, positions)
        changed: list[str] = []
        if self._is_real and not errors:
            if self._control_mode != "code2real&sim":
                errors.append(f"preview/execute is disabled in {self._control_mode} mode")
            now = time.monotonic()
            feedback_fresh = (
                self._last_joint_update is not None and now - self._last_joint_update < 0.5
            )
            if not feedback_fresh or not self._actual:
                errors.append("real hardware feedback is missing or stale")
            elif self._hardware_status.get("hold", {}).get("state") == "FAULT":
                errors.append("hold protection is faulted; press Reset HOLD fault before retry")
            if not errors:
                changed = [
                    name
                    for name in self._joint_names
                    if abs(normalized[name] - self._actual[name]) > 0.002
                ]
                if not changed:
                    errors.append("change at least one joint before Preview")
                else:
                    errors.extend(
                        changed_command_limit_errors(
                            self._effective_joint_specs(), self._actual, normalized
                        )
                    )
                if changed and not errors:
                    try:
                        self._candidate_minimum_duration_sec = max(
                            0.5,
                            minimum_target_duration(
                                self._joint_specs,
                                self._actual,
                                normalized,
                                self._config["safety"]["max_joint_velocity_rad_s"],
                            ),
                        )
                    except (KeyError, TypeError, ValueError) as exc:
                        errors.append(f"cannot compute guarded duration: {exc}")
        self._candidate_valid = not errors
        self._candidate_errors = errors
        self._candidate_changed_joints = changed if not errors else []
        if errors:
            self._candidate_id = None
            self._set_event("BLOCKED", "; ".join(errors))
            return
        self._candidate = normalized
        self._candidate_id = str(uuid.uuid4())
        self._publish_planned_path()
        self._set_event(
            "PLANNED",
            f"valid {self._mode} candidate from {source}; joints={changed}; "
            f"minimum duration={self._candidate_minimum_duration_sec:.2f}s; "
            "preview sent no hardware command",
        )

    def _select_preset(self, preset_id: str) -> None:
        preset = self._presets.get(preset_id)
        if preset is None:
            self._set_event("BLOCKED", f"unknown preset: {preset_id}")
            return
        if self._mode not in preset.get("allowed_modes", []):
            self._candidate_valid = False
            self._candidate_id = None
            self._candidate_errors = [f"preset {preset_id} is {preset.get('review_state', 'UNMEASURED')}"]
            self._set_event("BLOCKED", self._candidate_errors[0])
            return
        self._preview(preset.get("positions"), source=f"preset:{preset_id}")

    def _diagnostic_preview(self, positions: object) -> None:
        """Render a bounded rejected solution without making it executable."""
        normalized, errors = validate_target(self._effective_joint_specs(), positions)
        if errors:
            self._candidate_errors = errors
            self._candidate_valid = False
            self._candidate_id = None
            self._set_event("BLOCKED", "; ".join(errors))
            return
        self._candidate = normalized
        self._candidate_valid = False
        self._candidate_id = None
        self._candidate_changed_joints = []
        self._candidate_errors = [
            "diagnostic IK preview only; Touch acceptance criteria were not met"
        ]
        self._set_event(
            "DIAGNOSTIC_PREVIEW",
            "bounded rejected IK pose rendered in orange; execution is disabled",
        )

    def _execute_candidate(self, candidate_id: str, requested_duration: object) -> None:
        if self._active_goal is not None:
            self._set_event("BLOCKED", f"another {self._mode} trajectory is active")
            return
        if not self._candidate_valid or not self._candidate_id:
            self._set_event("BLOCKED", "preview a valid candidate before execution")
            return
        if candidate_id != self._candidate_id:
            self._set_event("BLOCKED", "candidate_id is stale or does not match the preview")
            return
        if not self._trajectory_client.server_is_ready():
            self._set_event("BLOCKED", "FollowJointTrajectory action server is not ready")
            return
        try:
            max_duration = (
                float(self._config["safety"]["max_duration_sec"])
                if self._is_real
                else 10.0
            )
            duration = max(0.5, float(requested_duration))
            if self._is_real:
                duration = max(duration, self._candidate_minimum_duration_sec)
            if duration > max_duration:
                self._set_event(
                    "BLOCKED",
                    f"guarded duration {duration:.2f}s exceeds maximum {max_duration:.2f}s",
                )
                return
        except (TypeError, ValueError):
            self._set_event("BLOCKED", "duration_sec must be numeric")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = [self._candidate[name] for name in self._joint_names]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1_000_000_000)
        goal.trajectory.points = [point]
        self._task_state = "SENDING"
        self._task_message = (
            f"submitting guarded {self._mode} goal for {self._candidate_changed_joints}; "
            f"duration={duration:.2f}s"
        )
        future = self._trajectory_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy transports exceptions through Future
            self._set_event("FAILED", f"goal transport failed: {exc}")
            return
        if not goal_handle.accepted:
            self._set_event("FAILED", f"{self._mode} trajectory controller rejected the goal")
            return
        self._active_goal = goal_handle
        self._task_state = "EXECUTING"
        self._task_message = f"{self._mode} trajectory executing"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if feedback.actual.positions:
            maximum_error = max(
                abs(float(value)) for value in feedback.error.positions
            ) if feedback.error.positions else 0.0
            self._task_message = f"{self._mode} trajectory executing; max error={maximum_error:.4f} rad"

    def _on_goal_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._active_goal = None
            self._set_event("FAILED", f"result transport failed: {exc}")
            return
        self._active_goal = None
        result = wrapped.result
        if self._is_real:
            self._candidate = dict(self._actual) if self._actual else dict(self._candidate)
            self._candidate_id = None
            self._candidate_valid = False
            self._candidate_changed_joints = []
            self._candidate_minimum_duration_sec = 0.5
            self._candidate_errors = ["adjust one or more sliders, then Preview / Validate"]
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == result.SUCCESSFUL:
            self._set_event("SUCCEEDED", f"{self._mode} FollowJointTrajectory completed")
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            self._set_event("CANCELLED", f"{self._mode} trajectory cancelled")
        else:
            self._set_event("FAILED", f"trajectory failed: {result.error_code} {result.error_string}")

    def _cancel_active_goal(self) -> None:
        if self._active_goal is None:
            self._set_event("IDLE", f"no active {self._mode} trajectory to cancel")
            return
        self._task_state = "CANCELLING"
        self._task_message = "cancel requested"
        self._active_goal.cancel_goal_async()

    def _release_hold(self) -> None:
        if not self._is_real:
            self._set_event("BLOCKED", "real hold is unavailable in MOCK mode")
            return
        if self._active_goal is not None:
            self._set_event("BLOCKED", "cancel the active trajectory before releasing hold")
            return
        self._hold_command_pub.publish(String(data="release"))
        self._set_event(
            "RELEASING_HOLD",
            "hold release requested; physically support the arm before torque becomes free",
        )

    def _reset_hold_fault(self) -> None:
        if not self._is_real:
            self._set_event("BLOCKED", "real hold is unavailable in MOCK mode")
            return
        if self._active_goal is not None:
            self._set_event("BLOCKED", "cancel the active trajectory before resetting HOLD")
            return
        if self._hardware_status.get("hold", {}).get("state") != "FAULT":
            self._set_event("IDLE", "no HOLD protection fault is latched")
            return
        self._hold_command_pub.publish(String(data="reset_fault"))
        self._set_event(
            "RESETTING_HOLD_FAULT",
            "HOLD fault acknowledgement requested; if hardware checks pass, "
            "code2real&sim will re-enable all six torques at the measured pose",
        )

    def _prepare_safe_shutdown(self) -> None:
        if not self._is_real:
            self._set_event("BLOCKED", "safe shutdown is available only for real hardware")
            return
        if self._active_goal is not None:
            self._set_event("BLOCKED", "cancel the active trajectory before safe shutdown")
            return
        self._candidate_valid = False
        self._candidate_id = None
        self._candidate_errors = ["safe shutdown requested; do not execute another goal"]
        self._hold_command_pub.publish(String(data="safe_shutdown"))
        self._set_event(
            "PREPARING_SHUTDOWN",
            "support the arm; disabling all six torques—wait for SAFE TO CLOSE",
        )

    def _trigger(self, trigger_id: str) -> None:
        trigger = next(
            (item for item in self._config.get("triggers", []) if item.get("id") == trigger_id),
            None,
        )
        if trigger is None:
            self._set_event("BLOCKED", f"unknown trigger: {trigger_id}")
            return
        if trigger_id == "touch_anything":
            touch_fresh = (
                self._last_touch_state_update is not None
                and time.monotonic() - self._last_touch_state_update < 1.0
            )
            if not touch_fresh:
                self._set_event(
                    "BLOCKED", "Touch Anything node is missing or its state is stale"
                )
                return
            if bool(self._touch_state.get("active")):
                self._publish_touch_command("exit", "arm_dashboard")
                self._set_event("TOUCH_EXITING", "Touch Mode exit requested")
                return
            if not bool(self._touch_state.get("sam", {}).get("available")):
                self._set_event(
                    "BLOCKED",
                    "SAM 2.1 is not ready: "
                    + str(self._touch_state.get("sam", {}).get("reason", "unknown reason")),
                )
                return
            if self._control_mode != "code2real&sim":
                self._set_event(
                    "BLOCKED", "select code2real&sim before entering Touch Anything"
                )
                return
            if self._active_goal is not None:
                self._set_event("BLOCKED", "cancel the active arm trajectory first")
                return
            if self._hardware_status.get("hold", {}).get("state") == "FAULT":
                self._set_event("BLOCKED", "HOLD protection is faulted")
                return
            self._publish_touch_command("enter", "arm_dashboard")
            self._set_event(
                "TOUCH_ENTERING",
                "Touch Mode requested; use the RViz Touch Anything panel after base lock is confirmed",
            )
            return
        self._set_event("BLOCKED", str(trigger.get("reason", "trigger is not implemented")))

    def _publish_touch_command(self, command: str, source: str) -> None:
        self._touch_command_pub.publish(
            String(
                data=json.dumps(
                    {
                        "command": command,
                        "source": source,
                        "request_id": str(uuid.uuid4()),
                    },
                    ensure_ascii=False,
                )
            )
        )

    def _publish_candidate(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["candidate_" + name for name in self._joint_names]
        # The cyan actual model consumes /arm/visual_joint_states with the
        # vision-derived calibration. Apply the identical visualization-only
        # mapping to the orange candidate so both models describe the same
        # physical joint geometry. Commands remain unmodified/raw.
        msg.position = [
            corrected_position(name, self._candidate[name])
            for name in self._joint_names
        ]
        self._candidate_pub.publish(msg)

    def _publish_planned_path(self) -> None:
        actual = [self._actual.get(name, self._candidate[name]) for name in self._joint_names]
        display = DisplayTrajectory()
        display.model_id = str(self._config.get("model_id", "so_arm101"))
        display.trajectory_start.joint_state.name = list(self._joint_names)
        display.trajectory_start.joint_state.position = actual
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(self._joint_names)
        start = JointTrajectoryPoint()
        start.positions = actual
        target = JointTrajectoryPoint()
        target.positions = [self._candidate[name] for name in self._joint_names]
        target.time_from_start.sec = 2
        trajectory.joint_trajectory.points = [start, target]
        display.trajectory = [trajectory]
        self._planned_path_pub.publish(display)

    def _set_event(self, state: str, message: str) -> None:
        self._task_state = state
        self._task_message = message
        self.get_logger().info(f"{state}: {message}")
        self._publish_state()

    def _publish_state(self) -> None:
        now = time.monotonic()
        joint_age = None if self._last_joint_update is None else now - self._last_joint_update
        controller_age = None if self._last_controller_update is None else now - self._last_controller_update
        action_ready = self._trajectory_client.server_is_ready()
        feedback_fresh = joint_age is not None and joint_age < 0.5
        mode_synced = self._server_control_mode == self._control_mode
        hold = self._hardware_status.get(
            "hold",
            {
                "state": "UNKNOWN" if self._is_real else "NOT_APPLICABLE",
                "holding": False,
                "reason": "waiting for hardware status" if self._is_real else "mock mode",
            },
        )
        hold_fault = self._is_real and hold.get("state") == "FAULT"
        required_torque_missing = self._hardware_status.get(
            "required_torque_missing", {}
        )
        code_torque_incomplete = bool(
            self._is_real
            and self._control_mode in {"code2real&sim", "teleoperate"}
            and required_torque_missing
        )
        teleoperation = self._hardware_status.get(
            "teleoperation",
            {
                "ready": False,
                "active": False,
                "reason": "waiting for leader status",
            },
        )
        recovery = self._hardware_status.get(
            "recovery", {"required": False, "joints": {}, "feedback_clamped": {}}
        )
        recovery_required = bool(self._is_real and recovery.get("required"))
        torque = self._hardware_status.get("torque_enable", {})
        torque_values = [int(value) for value in torque.values()] if isinstance(torque, dict) else []
        safe_to_close = bool(
            self._is_real
            and len(torque_values) == len(self._joint_names)
            and not any(torque_values)
            and not hold.get("holding")
            and self._active_goal is None
        )
        touch_age = (
            None
            if self._last_touch_state_update is None
            else now - self._last_touch_state_update
        )
        touch_fresh = touch_age is not None and touch_age < 1.0
        runtime_triggers = []
        for configured_trigger in self._config.get("triggers", []):
            trigger = dict(configured_trigger)
            if trigger.get("id") == "touch_anything":
                if not touch_fresh:
                    trigger.update(
                        state="BLOCKED",
                        reason="Touch Anything node is missing or its state is stale",
                    )
                elif bool(self._touch_state.get("active")):
                    trigger.update(
                        state="ACTIVE",
                        reason=(
                            f"{self._touch_state.get('state', 'UNKNOWN')}: "
                            f"{self._touch_state.get('message', '')}; click to exit"
                        ),
                    )
                elif not bool(self._touch_state.get("sam", {}).get("available")):
                    trigger.update(
                        state="BLOCKED",
                        reason=(
                            "SAM 2.1 is not ready: "
                            + str(
                                self._touch_state.get("sam", {}).get(
                                    "reason", "unknown reason"
                                )
                            )
                        ),
                    )
                elif self._control_mode != "code2real&sim":
                    trigger.update(
                        state="BLOCKED",
                        reason="Select code2real&sim first",
                    )
                else:
                    trigger.update(
                        state="AVAILABLE",
                        reason="Enter preview-only Touch Mode; target selection continues in RViz",
                    )
            runtime_triggers.append(trigger)
        if action_ready and feedback_fresh and mode_synced:
            if not self._is_real:
                health = "MOCK_OK"
            elif recovery_required:
                health = "RECOVERY_REQUIRED"
            elif self._control_mode == "real2sim":
                health = "REAL2SIM_READY"
            elif self._control_mode == "code2real&sim":
                health = "HOLD_FAULT" if hold_fault else (
                    "CODE_TORQUE_INCOMPLETE" if code_torque_incomplete else
                    "REAL_HOLDING" if hold.get("holding") else "REAL_READY"
                )
            else:
                health = "HOLD_FAULT" if hold_fault else (
                    "TELEOP_TORQUE_INCOMPLETE" if code_torque_incomplete else
                    "TELEOP_ACTIVE" if teleoperation.get("active") else
                    "TELEOP_ALIGNING" if teleoperation.get("aligning") else
                    "TELEOP_WAITING"
                )
            self._controller_state = "ACTIVE"
        elif self._is_real and not mode_synced:
            health = "MODE_SYNCING"
        elif self._last_joint_update is None:
            health = "STARTING"
        else:
            health = "STALE"

        joints = []
        for spec in self._effective_joint_specs():
            name = str(spec["name"])
            joints.append(
                {
                    "name": name,
                    "motor_name": spec.get("motor_name", name),
                    "label": spec.get("label", name),
                    "lower": float(spec["lower"]),
                    "upper": float(spec["upper"]),
                    "actual": self._actual.get(name),
                    "target": self._candidate[name],
                }
            )

        state = {
            "schema_version": int(self._config.get("schema_version", 1)),
            "mode": self._mode,
            "control_mode": self._control_mode,
            "control_mode_synced": mode_synced,
            "control_modes": [
                {
                    **mode,
                    "available": bool(mode.get("available"))
                    and (
                        str(mode.get("id")) != "teleoperate"
                        or bool(teleoperation.get("ready"))
                    ),
                    "runtime_reason": (
                        teleoperation.get("reason")
                        if str(mode.get("id")) == "teleoperate"
                        else None
                    ),
                }
                for mode in self._control_modes
            ],
            "safety_banner": (
                "RECOVERY_REQUIRED — measured pose is outside the conservative command "
                "envelope; feedback remains available, but only an inward goal is allowed: "
                f"{recovery.get('joints', {})}"
                if recovery_required
                else
                (
                    "REAL2SIM — manually move the follower; program goals are blocked and torque stays off"
                    if self._control_mode == "real2sim"
                    else (
                        "CODE2REAL&SIM — all six torques stay ON at the measured/commanded "
                        "pose except after a safety trip or explicit operator release; "
                        f"HOLD={hold.get('state', 'UNKNOWN')}"
                    )
                    if self._control_mode == "code2real&sim"
                    else (
                        "TELEOPERATE — follower first aligns to the calibrated absolute Leader "
                        "pose under velocity limits, then follows continuously; follower six-axis torque stays ON; "
                        f"leader={teleoperation.get('reason', 'UNKNOWN')}"
                    )
                )
                if self._is_real
                else "MOCK ONLY — no follower serial device is opened"
            ),
            "health": health,
            "controller": {
                "name": "/arm_controller/follow_joint_trajectory",
                "state": self._controller_state,
                "action_ready": action_ready,
                "state_age_sec": controller_age,
            },
            "feedback_age_sec": joint_age,
            "command_owner": (
                "HUMAN_REAL2SIM"
                if self._is_real and self._control_mode == "real2sim"
                else "CODE_REAL_SIM"
                if self._is_real and self._control_mode == "code2real&sim"
                else "LEADER_TELEOP"
                if self._is_real
                else "DASHBOARD_MOCK"
            ),
            "calibration": self._config.get("calibration", {}),
            "hardware": self._config.get("hardware", {}),
            "hold": hold,
            "teleoperation": teleoperation,
            "recovery": recovery,
            "joints": joints,
            "motor_health": {
                "voltage_deci_v": self._hardware_status.get("voltage_deci_v", {}),
                "temperature_c": self._hardware_status.get("temperature_c", {}),
                "current_raw": self._hardware_status.get("current_raw", {}),
                "torque_enable": self._hardware_status.get("torque_enable", {}),
                "unexpected_torque": self._hardware_status.get("unexpected_torque", {}),
                "code_torque_required": self._hardware_status.get(
                    "code_torque_required", False
                ),
                "powered_torque_required": self._hardware_status.get(
                    "powered_torque_required", False
                ),
                "required_torque_missing": required_torque_missing,
                "active_motors": self._hardware_status.get("active_motors", []),
            },
            "candidate": {
                "id": self._candidate_id,
                "valid": self._candidate_valid,
                "errors": self._candidate_errors,
                "positions": self._candidate,
                "changed_joints": self._candidate_changed_joints,
                "minimum_duration_sec": self._candidate_minimum_duration_sec,
            },
            "execution": {
                "allowed": bool(
                    action_ready
                    and feedback_fresh
                    and mode_synced
                    and not hold_fault
                    and not code_torque_incomplete
                    and self._candidate_valid
                    and (not self._is_real or self._control_mode == "code2real&sim")
                ),
                "state": self._task_state,
                "message": self._task_message,
                "last_request_id": self._last_request_id,
            },
            "shutdown": {
                "safe_to_close": safe_to_close,
                "torque_enabled_count": sum(1 for value in torque_values if value),
                "message": (
                    "SAFE TO CLOSE TERMINALS / ターミナルを閉じても安全です"
                    if safe_to_close
                    else "NOT SAFE TO CLOSE: torque or motion is still active"
                ),
            },
            "presets": list(self._presets.values()),
            "triggers": runtime_triggers,
            "touch_anything": {
                **self._touch_state,
                "state_age_sec": touch_age,
                "fresh": touch_fresh,
            },
            "protection": self._hardware_status.get("protection", {}),
        }
        self._state_pub.publish(String(data=json.dumps(state, ensure_ascii=False, allow_nan=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
