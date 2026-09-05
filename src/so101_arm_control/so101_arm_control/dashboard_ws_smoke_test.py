from __future__ import annotations

import asyncio
import json
import uuid

from aiohttp import ClientSession, WSMsgType


TARGET = {
    "shoulder_pan_joint": -0.12,
    "shoulder_lift_joint": 0.08,
    "elbow_flex_joint": -0.10,
    "wrist_flex_joint": 0.06,
    "wrist_roll_joint": 0.10,
    "gripper_joint": 0.75,
}


async def next_arm_state(ws, timeout: float = 5.0) -> dict:
    async def receive() -> dict:
        while True:
            msg = await ws.receive()
            if msg.type != WSMsgType.TEXT:
                raise RuntimeError(f"websocket closed before arm state: {msg.type}")
            envelope = json.loads(msg.data)
            if envelope.get("topic") == "/arm/dashboard_state":
                return json.loads(envelope["data"])

    return await asyncio.wait_for(receive(), timeout=timeout)


async def send_arm(ws, command: dict) -> str:
    request_id = str(uuid.uuid4())
    await ws.send_json(
        {
            "type": "arm_command",
            "command": {
                "request_id": request_id,
                "requested_by": "dashboard_ws_smoke_test",
                "mode": "MOCK",
                **command,
            },
        }
    )
    return request_id


async def run() -> None:
    async with ClientSession() as session:
        async with session.ws_connect("http://127.0.0.1:8765/ws") as ws:
            state = await next_arm_state(ws, 8.0)
            if state.get("mode") != "MOCK":
                raise RuntimeError("websocket smoke test refuses non-MOCK mode")
            if state.get("hardware", {}).get("real_hardware_enabled") is not False:
                raise RuntimeError("websocket smoke test refuses real-hardware-enabled state")

            preview_request = await send_arm(ws, {"command": "preview", "positions": TARGET})
            while True:
                state = await next_arm_state(ws)
                if state.get("execution", {}).get("last_request_id") == preview_request:
                    break
            if not state.get("candidate", {}).get("valid"):
                raise RuntimeError(f"preview was blocked: {state['candidate'].get('errors')}")
            candidate_id = state["candidate"]["id"]

            execute_request = await send_arm(
                ws,
                {
                    "command": "execute_candidate",
                    "candidate_id": candidate_id,
                    "duration_sec": 1.0,
                },
            )
            while True:
                state = await next_arm_state(ws)
                execution = state.get("execution", {})
                if execution.get("last_request_id") == execute_request and execution.get("state") in {
                    "SUCCEEDED",
                    "FAILED",
                    "BLOCKED",
                }:
                    break
            if state["execution"]["state"] != "SUCCEEDED":
                raise RuntimeError(f"websocket execution failed: {state['execution']}")
            actual = {joint["name"]: joint["actual"] for joint in state["joints"]}
            if not all(abs(float(actual[name]) - target) < 0.015 for name, target in TARGET.items()):
                raise RuntimeError(f"websocket feedback mismatch: {actual}")
            print(
                json.dumps(
                    {
                        "result": "PASS",
                        "transport": "Dashboard WebSocket",
                        "mode": state["mode"],
                        "candidate_id": candidate_id,
                        "execution": state["execution"],
                        "joint_positions": actual,
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )


def main() -> None:
    asyncio.run(run())


if __name__ == "__main__":
    main()
