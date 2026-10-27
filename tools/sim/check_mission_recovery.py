#!/usr/bin/env python3
"""Opt-in stationary sim Lift fault: Mission wait, scoped approval, and late reply."""
import argparse
import asyncio
import json
from pathlib import Path
import sys
import time

import aiohttp

from run_mission_case import decode, run


async def check(args):
    if not args.execute:
        raise RuntimeError("Requires --execute: MANUAL, stop Lift, run observation, approve Lift recovery")
    rows = []
    async with aiohttp.ClientSession() as session:
        config = await (await session.get("http://127.0.0.1:8765/api/config")).json()
        if config.get("stack_mode") != "sim" or str(config.get("stack_domain_id")) != "20":
            raise RuntimeError("Only sim domain 20 is supported")
        async with session.ws_connect("http://127.0.0.1:8765/ws") as ws:
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                try:
                    topic, data = decode(await asyncio.wait_for(ws.receive_json(), 1))
                except asyncio.TimeoutError:
                    continue
                if topic == "/agent/mission_state" and data.get("status") not in {
                    None, "", "IDLE", "DONE", "CANCELLED", "FAILED_PLAN", "FAILED_SKILL", "FAILED_PARSE"
                }:
                    raise RuntimeError("Existing Mission is active; do not change mode or inject")
                if topic == "/agent/recovery_state":
                    active = data.get("execution", {}).get("active", [])
                    if any(item.get("playbook_id") != "PB-HOST-RESOURCE" for item in active):
                        raise RuntimeError("Robot Recovery already active; inspect before injection")
            await ws.send_json({"type": "joy_control", "action": "ACQUIRE_DASHBOARD"})
            buttons = [0, 1, 0, 0, 0, 0, 0, 0]
            await ws.send_json({"type": "joy_state", "axes": [0] * 8, "buttons": buttons})
            await asyncio.sleep(.2)
            await ws.send_json({"type": "joy_state", "axes": [0] * 8, "buttons": [0] * 8})
            task = None
            try:
                process = await asyncio.create_subprocess_exec(
                    sys.executable, str(Path(__file__).with_name("inject_recovery_fault.py")),
                    "lift-exit", "--execute", stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
                output, _ = await process.communicate()
                rows.append({"type": "injection", "output": output.decode(), "returncode": process.returncode})
                print(output.decode(), flush=True)
                if process.returncode:
                    raise RuntimeError("Injection guard refused")
                case_args = argparse.Namespace(
                    url="http://127.0.0.1:8765/ws", mode="jev_primary",
                    case="R-Mission-lift-cancel" if args.cancel else "R-Mission-lift-resume",
                    command="look left, then set arm pose home", timeout=150, hold_seconds=40,
                    human_action=None, cancel_on_skill=None,
                    cancel_on_status="RECOVERING" if args.cancel else None, cancel_delay=0,
                    output=args.missions)
                task = asyncio.create_task(run(case_args))
                started, approved, recovering, verified = time.monotonic(), False, False, False
                own_id, cancelled, verified_at, execution_id = None, False, None, None
                while time.monotonic() - started < 180:
                    topic, data = decode(await asyncio.wait_for(ws.receive_json(), 5))
                    if topic == "/agent/mission_state" and isinstance(data, dict):
                        if data.get("command_text") == case_args.command or data.get("user_command") == case_args.command:
                            own_id = data.get("mission_id")
                        if data.get("status") == "RECOVERING":
                            recovering = True
                            own_id = data.get("mission_id")
                        if own_id and data.get("mission_id") == own_id:
                            cancelled |= data.get("status") == "CANCELLED"
                    if topic != "/agent/recovery_state" or not isinstance(data, dict):
                        continue
                    execution = data.get("execution") or {}
                    candidates = [r for r in execution.get("operator_requests", [])
                                  if r.get("incident_key") == "sim_lift" and r.get("kind") == "confirm_action"
                                  and "approve" in r.get("buttons", [])
                                  and (r.get("step") or {}).get("primitive") == "restart_component"]
                    # Restore even when Mission failed to request Recovery; record that as a failed handoff.
                    ready = (cancelled if args.cancel else recovering) or time.monotonic() - started > 45
                    if not approved and ready and len(candidates) == 1 and data.get("environment", {}).get("state") == "RUNNING":
                        request = candidates[0]
                        matches = [e for e in execution.get("active", [])
                                   if e.get("operator_request_id") == request["id"]]
                        if len(matches) != 1:
                            continue
                        execution_id = matches[0]["id"]
                        await ws.send_json({"type": "recovery_command", "command": {
                            "type": "operator_reply", "request_id": request["id"], "reply": "approve"}})
                        rows.append({"type": "approval", "request_id": request["id"],
                                     "mission_recovering_seen": recovering, "mission_cancelled_seen": cancelled})
                        approved = True
                    for result in execution.get("finished", []):
                        if approved and result.get("id") == execution_id and result.get("message") == "Verified":
                            if not verified:
                                rows.append({"type": "recovery_result", "result": result})
                                verified_at = time.monotonic()
                            verified = True
                    if task.done() and verified and time.monotonic() - verified_at > 4:
                        break
                if task.done():
                    rows.append({"type": "mission_result", "result": task.result()})
                else:
                    rows.append({"type": "mission_result", "result": await task})
                rows.append({"type": "summary", "approved": approved, "verified": verified,
                             "mission_recovering_seen": recovering, "cancelled_before_approval": cancelled})
                print(json.dumps(rows[-1]), flush=True)
            finally:
                if task is not None and not task.done():
                    task.cancel()
                    await asyncio.gather(task, return_exceptions=True)
                await ws.send_json({"type": "joy_control", "action": "RELEASE_DASHBOARD"})
                Path(args.output).write_text(json.dumps(rows, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--cancel", action="store_true")
    parser.add_argument("--output", required=True)
    parser.add_argument("--missions", required=True)
    asyncio.run(check(parser.parse_args()))
