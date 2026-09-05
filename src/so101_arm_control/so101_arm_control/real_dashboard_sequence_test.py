from __future__ import annotations

import argparse
import asyncio
import json
import uuid

from aiohttp import ClientSession, WSMsgType


EXPECTED_CALIBRATION_SHA256 = (
    "276e40239a27baa4b910d9f1f384e934bf70265e8f558fbc51e4266ffcf5f68c"
)
MAX_TEST_SEGMENT_DELTA_RAD = 0.45
TERMINAL_STATES = {"SUCCEEDED", "FAILED", "BLOCKED", "CANCELLED"}
TEST_POSES = {
    "home": {
        "shoulder_pan_joint": -0.036,
        "shoulder_lift_joint": -1.583,
        "elbow_flex_joint": 1.367,
        "wrist_flex_joint": 0.783,
        "wrist_roll_joint": -0.124,
        "gripper_joint": 0.222,
    },
    "see_ground": {
        "shoulder_pan_joint": -0.181,
        "shoulder_lift_joint": -0.202,
        "elbow_flex_joint": 0.893,
        "wrist_flex_joint": 0.241,
        "wrist_roll_joint": -1.109,
        "gripper_joint": 0.223,
    },
    "see_front": {
        "shoulder_pan_joint": -0.014,
        "shoulder_lift_joint": -0.704,
        "elbow_flex_joint": 1.285,
        "wrist_flex_joint": -0.665,
        "wrist_roll_joint": -1.109,
        "gripper_joint": 0.222,
    },
    "see_right": {
        "shoulder_pan_joint": 0.933,
        "shoulder_lift_joint": -0.706,
        "elbow_flex_joint": 1.285,
        "wrist_flex_joint": -0.665,
        "wrist_roll_joint": -1.109,
        "gripper_joint": 0.222,
    },
    "see_left": {
        "shoulder_pan_joint": -1.050,
        "shoulder_lift_joint": -0.706,
        "elbow_flex_joint": 1.287,
        "wrist_flex_joint": -0.665,
        "wrist_roll_joint": -1.109,
        "gripper_joint": 0.222,
    },
    "see_upside": {
        "shoulder_pan_joint": 0.046,
        "shoulder_lift_joint": -0.845,
        "elbow_flex_joint": 0.770,
        "wrist_flex_joint": -0.432,
        "wrist_roll_joint": -1.108,
        "gripper_joint": 0.225,
    },
}


async def next_arm_state(ws, timeout: float = 5.0) -> dict:
    async def receive() -> dict:
        while True:
            message = await ws.receive()
            if message.type != WSMsgType.TEXT:
                raise RuntimeError(f"websocket closed before arm state: {message.type}")
            envelope = json.loads(message.data)
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
                "requested_by": "real_dashboard_sequence_test",
                "mode": "REAL_DASHBOARD_TEST",
                **command,
            },
        }
    )
    return request_id


async def wait_for_request(ws, request_id: str, timeout: float = 8.0) -> dict:
    deadline = asyncio.get_running_loop().time() + timeout
    while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            raise TimeoutError(f"request {request_id} did not appear in Dashboard state")
        state = await next_arm_state(ws, remaining)
        if state.get("execution", {}).get("last_request_id") == request_id:
            return state


def actual_positions(state: dict) -> dict[str, float]:
    return {str(item["name"]): float(item["actual"]) for item in state["joints"]}


def validate_start_and_target(state: dict, target: dict[str, float]) -> None:
    if state.get("mode") != "REAL_DASHBOARD_TEST":
        raise RuntimeError("refusing non-real-dashboard-test mode")
    if state.get("calibration", {}).get("sha256") != EXPECTED_CALIBRATION_SHA256:
        raise RuntimeError("refusing unexpected calibration hash")
    if not state.get("protection", {}).get("armed"):
        raise RuntimeError("refusing real motion without armed sampled protection")
    if state.get("health") not in {"REAL_READY", "REAL2SIM_READY"}:
        raise RuntimeError(f"arm health is not ready: {state.get('health')}")
    actual = actual_positions(state)
    if set(actual) != set(target):
        raise RuntimeError("target does not exactly cover current joints")
    large = {
        name: target[name] - actual[name]
        for name in target
        if abs(target[name] - actual[name]) > MAX_TEST_SEGMENT_DELTA_RAD
    }
    if large:
        raise RuntimeError(f"test segment exceeds {MAX_TEST_SEGMENT_DELTA_RAD} rad: {large}")


async def set_mode(ws, mode: str) -> dict:
    request_id = await send_arm(ws, {"command": "set_mode", "control_mode": mode})
    state = await wait_for_request(ws, request_id)
    deadline = asyncio.get_running_loop().time() + 5.0
    while state.get("control_mode") != mode or not state.get("control_mode_synced"):
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            raise TimeoutError(f"control mode failed to synchronize: {mode}")
        state = await next_arm_state(ws, remaining)
    return state


async def execute_pose(ws, pose_name: str) -> dict:
    target = TEST_POSES[pose_name]
    state = await next_arm_state(ws)
    validate_start_and_target(state, target)
    previous_trip = state.get("protection", {}).get("last_trip")

    preview_request = await send_arm(
        ws, {"command": "preview", "positions": target}
    )
    state = await wait_for_request(ws, preview_request)
    candidate = state.get("candidate", {})
    if not candidate.get("valid"):
        raise RuntimeError(f"preview blocked for {pose_name}: {candidate.get('errors')}")
    candidate_id = candidate["id"]
    duration = max(2.0, float(candidate.get("minimum_duration_sec", 0.5)))

    execute_request = await send_arm(
        ws,
        {
            "command": "execute_candidate",
            "candidate_id": candidate_id,
            "duration_sec": duration,
        },
    )
    deadline = asyncio.get_running_loop().time() + 30.0
    while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            raise TimeoutError(f"real execution timed out for {pose_name}")
        state = await next_arm_state(ws, remaining)
        execution = state.get("execution", {})
        if (
            execution.get("last_request_id") == execute_request
            and execution.get("state") in TERMINAL_STATES
        ):
            break
    if state["execution"]["state"] != "SUCCEEDED":
        raise RuntimeError(f"execution failed for {pose_name}: {state['execution']}")

    await asyncio.sleep(1.0)
    state = await next_arm_state(ws)
    protection = state.get("protection", {})
    if protection.get("last_trip") != previous_trip:
        raise RuntimeError(f"new protection trip after {pose_name}: {protection.get('last_trip')}")
    report = protection.get("last_motion") or {}
    if report.get("outcome") != "SUCCEEDED":
        raise RuntimeError(f"missing successful motion report for {pose_name}: {report}")
    return {
        "pose": pose_name,
        "duration_sec": duration,
        "actual_after_torque_off": actual_positions(state),
        "motion_report": report,
    }


async def run(sequence: list[str]) -> None:
    reports: list[dict] = []
    async with ClientSession() as session:
        async with session.ws_connect("http://127.0.0.1:8765/ws") as ws:
            initial = await next_arm_state(ws, 8.0)
            validate_start_and_target(initial, TEST_POSES[sequence[0]])
            try:
                await set_mode(ws, "code2real&sim")
                for pose_name in sequence:
                    reports.append(await execute_pose(ws, pose_name))
            except BaseException:
                await send_arm(ws, {"command": "cancel"})
                raise
            finally:
                await set_mode(ws, "real2sim")
    print(json.dumps({"result": "PASS", "reports": reports}, indent=2, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser(description="Explicit guarded SO101 real sequence test")
    parser.add_argument(
        "--sequence",
        default="home",
        help="comma-separated remount poses such as home,see_front,see_ground",
    )
    parser.add_argument(
        "--execute-real",
        action="store_true",
        help="required acknowledgement that this command will move the real arm",
    )
    args = parser.parse_args()
    if not args.execute_real:
        parser.error("--execute-real is required")
    sequence = [item.strip() for item in args.sequence.split(",") if item.strip()]
    unknown = [item for item in sequence if item not in TEST_POSES]
    if not sequence or unknown:
        parser.error(f"invalid sequence; unknown={unknown}, available={sorted(TEST_POSES)}")
    asyncio.run(run(sequence))


if __name__ == "__main__":
    main()
