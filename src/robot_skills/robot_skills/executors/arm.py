from __future__ import annotations

import json
import time
import uuid
from typing import Any

from robot_skills import result_codes as codes
from robot_skills.executors.base import dry_run_result, make_result, now
from robot_skills.schemas import SkillContext, SkillResult

try:
    from std_msgs.msg import String
except Exception:  # pragma: no cover - keeps unit imports alive without ROS.
    String = None


class ArmSkills:
    def __init__(self, node):
        self.node = node
        self._command_pub = None
        self._touch_pub = None
        self._state: dict[str, Any] = {}
        self._state_received_at = 0.0
        self._touch_state: dict[str, Any] = {}
        if node is not None and String is not None and hasattr(node, "create_publisher"):
            self._command_pub = node.create_publisher(String, "/arm/dashboard_command", 20)
            self._touch_pub = node.create_publisher(String, "/touch_anything/command", 20)
            node.create_subscription(String, "/arm/dashboard_state", self._on_state, 20)
            node.create_subscription(String, "/touch_anything/state", self._on_touch_state, 20)

    def set_pose(self, context: SkillContext, name: str, repeat: int = 1) -> SkillResult:
        started = now()
        pose_name = self._normalize_pose_name(name)
        sequence = self._pose_sequence(pose_name, max(1, repeat))
        if context.mode != "dry_run":
            error = self.validate_pose(pose_name, context.mode)
            if error:
                return make_result("set_pose", False, codes.FAILED_PRECHECK, error, started)
        if context.mode == "dry_run":
            return dry_run_result(
                "set_pose",
                f'Arm pose "{pose_name}" accepted as dry-run.',
                {"pose": pose_name, "sequence": sequence, "mode": context.mode},
            )
        if self._command_pub is None:
            return make_result("set_pose", False, codes.FAILED_PRECHECK, "Arm gateway is unavailable.")

        gateway_mode = self._gateway_mode(context.mode)
        try:
            for preset_id in sequence:
                self._check_cancel(context)
                preview_request_id = self._publish_arm(gateway_mode, "select_preset", preset_id=preset_id)
                candidate = self._wait_for_candidate(context, preset_id, preview_request_id)
                request_id = self._publish_arm(
                    gateway_mode,
                    "execute_candidate",
                    candidate_id=candidate["id"],
                    duration_sec=2.0,
                )
                if not self._wait_for_idle(context, request_id):
                    return make_result(
                        "set_pose",
                        False,
                        codes.FAILED_TIMEOUT,
                        f'Timed out waiting for arm preset "{preset_id}" to finish.',
                        started,
                        {"pose": pose_name, "preset_id": preset_id, "last_state": self._state},
                    )
        except RuntimeError as exc:
            return make_result(
                "set_pose",
                False,
                codes.FAILED_PRECHECK,
                str(exc),
                started,
                {"pose": pose_name, "sequence": sequence, "last_state": self._state},
            )

        return make_result(
            "set_pose",
            True,
            codes.OK,
            f'Arm pose "{pose_name}" completed.',
            started,
            {"pose": pose_name, "sequence": sequence, "mode": context.mode},
        )

    def validate_pose(self, name: str, mode: str) -> str | None:
        if not self._state_received_at or time.monotonic() - self._state_received_at > 3.0:
            return "Fresh arm gateway preset evidence is unavailable."
        presets = {str(p.get("id")): p for p in self._state.get("presets", []) if isinstance(p, dict)}
        for preset_id in self._pose_sequence(self._normalize_pose_name(name), 1):
            preset = presets.get(preset_id)
            if preset is None:
                return f'Unknown arm pose "{preset_id}". Configured presets: {", ".join(sorted(presets))}.'
            if self._gateway_mode(mode) not in preset.get("allowed_modes", []):
                return f'Arm pose "{preset_id}" is not approved for {mode}.'
        return None

    def touch(
        self,
        context: SkillContext,
        target: str,
        mode: str = "touch",
        detection_result: dict[str, Any] | None = None,
    ) -> SkillResult:
        started = now()
        detection = self._select_detection(detection_result)
        if detection is None:
            return make_result(
                "touch",
                False,
                codes.NEEDS_HUMAN,
                f'No usable detection bbox is available for "{target}".',
                started,
                {"target": target, "mode": mode, "detection_result": detection_result or {}},
            )
        bbox = detection["bbox"]
        u = 0.5 * (float(bbox[0]) + float(bbox[2]))
        v = 0.5 * (float(bbox[1]) + float(bbox[3]))
        commands = [
            {"command": "enter", "source": "arm_agent"},
            {"command": "freeze", "source": "arm_agent"},
            {"command": "add_prompt", "u": u, "v": v, "label": str(target), "source": "arm_agent"},
            {"command": "confirm_preview", "source": "arm_agent"},
            {"command": "plan_pre_touch_far", "source": "arm_agent"},
            {"command": "execute_pre_touch_far", "source": "arm_agent"},
            {"command": "acquire_arm_target", "source": "arm_agent"},
            {"command": "confirm_arm_target", "source": "arm_agent"},
        ]
        if mode == "press":
            commands.append({"command": "confirm_press", "source": "arm_agent"})
        commands.extend(
            [
                {"command": "plan_approach_step", "source": "arm_agent"},
                {"command": "execute_approach_step", "source": "arm_agent"},
            ]
        )
        if context.mode == "dry_run":
            return dry_run_result(
                "touch",
                f'Touch Anything {mode} sequence prepared for "{target}".',
                {"target": target, "mode": mode, "u": u, "v": v, "commands": commands},
            )
        if self._touch_pub is None:
            return make_result("touch", False, codes.FAILED_PRECHECK, "Touch gateway is unavailable.")
        for command in commands:
            self._check_cancel(context)
            self._touch_pub.publish(String(data=json.dumps(command, ensure_ascii=False, allow_nan=False)))
            time.sleep(0.05)
        return make_result(
            "touch",
            True,
            codes.COMMAND_ACCEPTED,
            f'Touch Anything {mode} sequence sent for "{target}".',
            started,
            {"target": target, "mode": mode, "u": u, "v": v, "commands_sent": len(commands)},
        )

    def grasp(self, context: SkillContext, target: str, destination: str | None = None) -> SkillResult:
        message = "GR00T grasp policy endpoint is not configured yet."
        return make_result(
            "grasp",
            False,
            codes.NEEDS_HUMAN,
            message,
            metrics={"target": target, "destination": destination, "mode": context.mode},
        )

    def _on_state(self, message) -> None:
        try:
            data = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(data, dict):
            self._state = data
            self._state_received_at = time.monotonic()

    def _on_touch_state(self, message) -> None:
        try:
            data = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(data, dict):
            self._touch_state = data

    def _publish_arm(self, gateway_mode: str, command: str, **extra: Any) -> str:
        if self._command_pub is None or String is None:
            raise RuntimeError("Arm dashboard command publisher is unavailable.")
        request_id = uuid.uuid4().hex
        payload = {"mode": gateway_mode, "request_id": request_id, "command": command, **extra}
        self._command_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))
        return request_id

    def _wait_for_candidate(self, context: SkillContext, preset_id: str, request_id: str) -> dict[str, Any]:
        deadline = time.monotonic() + min(3.0, context.max_duration_sec)
        while time.monotonic() < deadline:
            self._check_cancel(context)
            candidate = self._state.get("candidate") or {}
            execution = self._state.get("execution") or {}
            if execution.get("last_request_id") != request_id:
                time.sleep(0.05)
                continue
            if execution.get("state") in {"BLOCKED", "FAILED", "CANCELLED"}:
                raise RuntimeError(f'Arm preset "{preset_id}" rejected: {execution.get("message")}')
            if execution.get("state") == "PLANNED" and candidate.get("valid") and candidate.get("id"):
                return candidate
            time.sleep(0.05)
        raise RuntimeError(f'Timed out waiting for valid arm preset "{preset_id}".')

    def _wait_for_idle(self, context: SkillContext, request_id: str) -> bool:
        deadline = time.monotonic() + min(context.max_duration_sec, 10.0)
        saw_request = False
        while time.monotonic() < deadline:
            self._check_cancel(context)
            execution = self._state.get("execution") or {}
            if execution.get("last_request_id") != request_id:
                time.sleep(0.05)
                continue
            saw_request = True
            state = str(execution.get("state") or "")
            if state == "SUCCEEDED":
                return True
            if state in {"BLOCKED", "FAILED", "CANCELLED"}:
                raise RuntimeError(f'Arm execution blocked: {execution.get("message")}')
            time.sleep(0.05)
        if not saw_request:
            raise RuntimeError(f"Timed out waiting for arm gateway acknowledgement for request_id={request_id}")
        return False

    @staticmethod
    def _check_cancel(context):
        if getattr(context, "cancel_check", lambda: False)():
            raise RuntimeError("Arm task was cancelled")

    def cancel(self, context):
        if context.mode == "dry_run":
            return
        if self._touch_pub is not None and String is not None:
            self._touch_pub.publish(String(data=json.dumps({"command": "exit", "source": "mission_cancel"})))
        if self._command_pub is not None:
            self._publish_arm(self._gateway_mode(context.mode), "visual_servo_stop")
            self._publish_arm(self._gateway_mode(context.mode), "cancel")

    @staticmethod
    def _gateway_mode(mode: str) -> str:
        return "REAL_DASHBOARD_TEST" if mode == "real" else "MOCK"

    @staticmethod
    def _normalize_pose_name(name: str) -> str:
        value = str(name or "").strip().lower().replace("-", "_").replace(" ", "_")
        aliases = {
            "home_pose": "home",
            "arm_home": "home",
            "ground": "see_ground",
            "check_ground": "see_ground",
            "front": "see_front",
            "left": "see_left",
            "right": "see_right",
            "hello": "wave",
            "wave_hello": "wave",
        }
        return aliases.get(value, value)

    @staticmethod
    def _pose_sequence(name: str, repeat: int) -> list[str]:
        if name == "wave":
            return ["see_right", "see_left"] * repeat + ["see_front"]
        return [name]

    @staticmethod
    def _select_detection(result: dict[str, Any] | None) -> dict[str, Any] | None:
        detections = list((result or {}).get("detections") or [])
        usable = [item for item in detections if item.get("bbox") and len(item.get("bbox")) == 4]
        if not usable:
            return None
        usable.sort(
            key=lambda item: (
                ArmSkills._center_distance(item.get("bbox")),
                -float(item.get("confidence", item.get("score", 0.0)) or 0.0),
            )
        )
        return usable[0]

    @staticmethod
    def _center_distance(bbox: Any) -> float:
        try:
            x1, y1, x2, y2 = [float(v) for v in bbox]
        except (TypeError, ValueError):
            return float("inf")
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        return (cx - 320.0) ** 2 + (cy - 240.0) ** 2
