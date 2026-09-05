from __future__ import annotations

import json
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


class SmokeClient(Node):
    def __init__(self) -> None:
        super().__init__("so101_mock_smoke_test")
        self.state: dict | None = None
        self.joints: dict[str, float] = {}
        self.command_pub = self.create_publisher(String, "/arm/dashboard_command", 10)
        self.create_subscription(String, "/arm/dashboard_state", self._on_state, 20)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)

    def _on_state(self, msg: String) -> None:
        self.state = json.loads(msg.data)

    def _on_joints(self, msg: JointState) -> None:
        self.joints = {name: float(value) for name, value in zip(msg.name, msg.position)}

    def wait_for(self, predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(f"timeout waiting for {label}")

    def send(self, command: dict) -> None:
        command = {
            "request_id": str(uuid.uuid4()),
            "requested_by": "mock_smoke_test",
            "mode": "MOCK",
            **command,
        }
        self.command_pub.publish(String(data=json.dumps(command)))


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
        print(
            json.dumps(
                {
                    "result": "PASS",
                    "mode": node.state["mode"],
                    "candidate_id": candidate_id,
                    "execution": node.state["execution"],
                    "joint_positions": {name: node.joints[name] for name in TARGET},
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
