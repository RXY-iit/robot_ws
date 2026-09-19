from __future__ import annotations

import json
import os
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


TARGET = {
    "shoulder_pan_joint": 0.20,
    "shoulder_lift_joint": -0.10,
    "elbow_flex_joint": 0.15,
    "wrist_flex_joint": 0.05,
    "wrist_roll_joint": -0.10,
    "gripper_joint": 0.80,
}

SERVO_TARGET = {
    "shoulder_pan_joint": -0.10,
    "shoulder_lift_joint": 0.08,
    "elbow_flex_joint": -0.08,
    "wrist_flex_joint": -0.04,
    "wrist_roll_joint": 0.08,
    "gripper_joint": 0.78,
}


class SmokeClient(Node):
    def __init__(self) -> None:
        super().__init__("so101_mock_smoke_test")
        self.state: dict | None = None
        self.joints: dict[str, float] = {}
        self.visual_servo_states: list[dict] = []
        self.command_pub = self.create_publisher(String, "/arm/dashboard_command", 10)
        self.create_subscription(String, "/arm/dashboard_state", self._on_state, 20)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self.create_subscription(
            String, "/arm/visual_servo_state", self._on_visual_servo_state, 50
        )

    def _on_state(self, msg: String) -> None:
        self.state = json.loads(msg.data)

    def _on_joints(self, msg: JointState) -> None:
        self.joints = {name: float(value) for name, value in zip(msg.name, msg.position)}

    def _on_visual_servo_state(self, msg: String) -> None:
        payload = json.loads(msg.data)
        if isinstance(payload, dict):
            self.visual_servo_states.append(payload)

    def wait_for(self, predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(f"timeout waiting for {label}")

    def send(self, command: dict) -> str:
        request_id = str(uuid.uuid4())
        command = {
            "request_id": request_id,
            "requested_by": "mock_smoke_test",
            "mode": "MOCK",
            **command,
        }
        self.command_pub.publish(String(data=json.dumps(command)))
        return request_id

    def spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def action_chunk(self, target: dict[str, float]) -> dict:
        """Return a 10 x 50 ms smooth tail from current mock feedback."""
        points = []
        for index in range(10):
            linear = float(index + 1) / 10.0
            blend = linear * linear * (3.0 - 2.0 * linear)
            points.append({
                name: float(self.joints[name])
                + (float(value) - float(self.joints[name])) * blend
                for name, value in target.items()
            })
        points[-1] = dict(target)
        return {
            "action_horizon": points,
            "horizon_step_period_sec": 0.05,
            "horizon_plan_id": f"mock-horizon-{uuid.uuid4()}",
            "observation_age_sec": 0.04,
            "prediction_lead_sec": 0.12,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmokeClient()
    try:
        node.wait_for(
            lambda: bool(
                node.state
                and node.state.get("mode") == "MOCK"
                and node.state.get("health") == "MOCK_OK"
                and node.state.get("controller", {}).get("action_ready")
            ),
            8.0,
            "MOCK_OK dashboard state",
        )
        if node.state["hardware"].get("real_hardware_enabled") is not False:
            raise RuntimeError("smoke test refuses a state that permits real hardware")

        node.send({"command": "preview", "positions": TARGET})
        node.wait_for(
            lambda: bool(node.state and node.state.get("candidate", {}).get("valid")),
            4.0,
            "valid candidate",
        )
        candidate_id = node.state["candidate"]["id"]
        node.send(
            {
                "command": "execute_candidate",
                "candidate_id": candidate_id,
                "duration_sec": 1.0,
            }
        )
        node.wait_for(
            lambda: bool(node.state and node.state.get("execution", {}).get("state") == "SUCCEEDED"),
            6.0,
            "mock trajectory success",
        )
        node.wait_for(
            lambda: all(abs(node.joints.get(name, 99.0) - target) < 0.015 for name, target in TARGET.items()),
            2.0,
            "joint feedback target",
        )

        # Normal rolling session: START once, then update the target while the
        # same session remains ACTIVE.  This proves there is no overlapping
        # FollowJointTrajectory goal and no normal-cycle HOLD transition.
        node.send({"command": "preview", "positions": SERVO_TARGET})
        node.wait_for(
            lambda: bool(
                node.state
                and node.state.get("candidate", {}).get("valid")
                and node.state.get("candidate", {}).get("positions") == SERVO_TARGET
            ),
            4.0,
            "rolling candidate",
        )
        servo_candidate_id = node.state["candidate"]["id"]
        session_id = f"mock-visual-servo-{uuid.uuid4()}"
        owner_request_id = node.send({
            "command": "visual_servo_start",
            "candidate_id": servo_candidate_id,
            "session_id": session_id,
            "positions": SERVO_TARGET,
            "tracker_stamp": 1.0,
            **node.action_chunk(SERVO_TARGET),
        })
        node.wait_for(
            lambda: bool(
                node.state
                and node.state.get("execution", {}).get("state") == "EXECUTING"
                and node.state.get("execution", {}).get("owner_request_id")
                == owner_request_id
            ),
            3.0,
            "visual-servo ACTIVE acknowledgement",
        )
        for sequence in range(1, 13):
            phase = sequence / 12.0
            positions = dict(SERVO_TARGET)
            positions["shoulder_pan_joint"] = -0.10 + 0.12 * phase
            positions["wrist_roll_joint"] = 0.08 - 0.10 * phase
            node.send({
                "command": "visual_servo_update",
                "session_id": session_id,
                "sequence": sequence,
                "positions": positions,
                "tracker_stamp": 1.0 + sequence * 0.04,
                **node.action_chunk(positions),
            })
            node.spin_for(0.08)
        pre_stop_positions = dict(node.joints)
        node.send({
            "command": "visual_servo_stop",
            "session_id": session_id,
            "sequence": 13,
            "reason": "mock rolling segment complete",
        })
        node.wait_for(
            lambda: bool(
                node.state
                and node.state.get("execution", {}).get("state") == "SUCCEEDED"
                and node.state.get("visual_servo", {}).get("state") == "HOLDING"
            ),
            3.0,
            "normal visual-servo HOLD acknowledgement",
        )
        node.spin_for(0.25)
        post_stop_maximum_drift = max(
            abs(float(node.joints[name]) - float(pre_stop_positions[name]))
            for name in SERVO_TARGET
        )
        if post_stop_maximum_drift > 1.0e-9:
            raise RuntimeError(
                "mock visual-servo continued after STOP: maximum drift="
                f"{post_stop_maximum_drift:.9f} rad"
            )
        normal_update_count = int(
            node.state.get("visual_servo", {}).get("update_count", 0)
        )
        if normal_update_count < 13:
            raise RuntimeError(
                f"server acknowledged only {normal_update_count} rolling updates"
            )
        normal_states = [
            state for state in node.visual_servo_states
            if state.get("session_id") == session_id
        ]
        sequences_observed = sorted({
            int(state.get("sequence_applied", -1)) for state in normal_states
            if state.get("state") == "ACTIVE"
        })
        requested_pan_values = {
            round(float(state.get("requested_positions", {}).get(
                "shoulder_pan_joint", 999.0
            )), 5)
            for state in normal_states
            if state.get("state") == "ACTIVE"
        }
        applied_pan_values = [
            float(state.get("applied_positions", {}).get(
                "shoulder_pan_joint", 0.0
            ))
            for state in normal_states
            if state.get("state") == "ACTIVE"
        ]
        applied_pan_span = (
            max(applied_pan_values) - min(applied_pan_values)
            if applied_pan_values else 0.0
        )
        horizon_indices_observed = sorted({
            int(state.get("action_horizon_index", -1))
            for state in normal_states
            if state.get("state") == "ACTIVE"
            and int(state.get("action_horizon_index", -1)) >= 0
        })
        maximum_horizon_replans = max(
            (
                int(state.get("action_horizon_replan_count", 0))
                for state in normal_states
            ),
            default=0,
        )
        realtime_correction_observed = bool(
            normal_update_count >= 13
            and len(sequences_observed) >= 5
            and len(requested_pan_values) >= 5
            and applied_pan_span >= 0.02
            and maximum_horizon_replans >= 13
            and len(horizon_indices_observed) >= 2
        )
        if not realtime_correction_observed:
            raise RuntimeError(
                "rolling target/application evidence is incomplete: "
                f"sequences={sequences_observed}, requested targets="
                f"{len(requested_pan_values)}, applied span={applied_pan_span:.4f}"
                f", horizon replans={maximum_horizon_replans}, "
                f"indices={horizon_indices_observed}"
            )

        # Watchdog injection: omit UPDATE/STOP after START. The mock backend
        # must leave motion ownership and report WATCHDOG_HOLD within 350 ms.
        watchdog_session = f"mock-watchdog-{uuid.uuid4()}"
        node.send({"command": "preview", "positions": TARGET})
        node.wait_for(
            lambda: bool(
                node.state
                and node.state.get("candidate", {}).get("valid")
                and node.state.get("candidate", {}).get("positions") == TARGET
            ),
            4.0,
            "watchdog candidate",
        )
        node.send({
            "command": "visual_servo_start",
            "candidate_id": node.state["candidate"]["id"],
            "session_id": watchdog_session,
            "positions": TARGET,
            "tracker_stamp": 2.0,
            **node.action_chunk(TARGET),
        })
        node.wait_for(
            lambda: bool(
                node.state
                and node.state.get("visual_servo", {}).get("state")
                == "WATCHDOG_HOLD"
                and node.state.get("execution", {}).get("state") == "FAILED"
            ),
            3.0,
            "visual-servo watchdog HOLD",
        )
        report = {
                    "result": "PASS",
                    "mode": node.state["mode"],
                    "candidate_id": candidate_id,
                    "execution": node.state["execution"],
                    "joint_positions": {name: node.joints[name] for name in TARGET},
                    "rolling_visual_servo": {
                        "normal_update_count": normal_update_count,
                        "normal_terminal_state": "HOLDING",
                        "watchdog_terminal_state": node.state["visual_servo"]["state"],
                        "overlapping_action_goal": False,
                        "active_sequences_observed": sequences_observed,
                        "distinct_requested_pan_targets": len(requested_pan_values),
                        "applied_pan_span_rad": applied_pan_span,
                        "action_horizon_steps": 10,
                        "action_horizon_step_period_sec": 0.05,
                        "action_horizon_duration_sec": 0.5,
                        "action_horizon_replans_observed": maximum_horizon_replans,
                        "action_horizon_indices_observed": horizon_indices_observed,
                        "post_stop_maximum_drift_rad": post_stop_maximum_drift,
                        "stop_policy": "CURRENT_POSE_HOLD",
                    },
                    "gates": {
                        "realtime_motion_correction_observed": (
                            realtime_correction_observed
                        ),
                        "next_real_motion_test_ready": bool(
                            realtime_correction_observed
                            and node.state["visual_servo"]["state"]
                            == "WATCHDOG_HOLD"
                        ),
                    },
        }
        output = json.dumps(report, ensure_ascii=False, indent=2)
        report_path = os.environ.get("TOUCH_MOCK_REPORT_PATH")
        if report_path:
            Path(report_path).write_text(output + "\n", encoding="utf-8")
        print(output)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
