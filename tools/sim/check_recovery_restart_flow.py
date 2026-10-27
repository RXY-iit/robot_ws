#!/usr/bin/env python3
"""Capture recovery evidence; optionally approve one currently offered sim start."""
import argparse
import asyncio
import json
from pathlib import Path

import aiohttp


async def check(args):
    async with aiohttp.ClientSession() as session:
        config = await (await session.get("http://127.0.0.1:8765/api/config")).json()
        if config.get("stack_mode") != "sim" or str(config.get("stack_domain_id")) != "20":
            raise RuntimeError("Expected sim Agent stack in domain 20; no action sent")
        async with session.ws_connect("http://127.0.0.1:8765/ws") as ws:
            while True:
                packet = await ws.receive_json()
                if packet.get("topic") != "/agent/recovery_state":
                    continue
                data = packet["data"]
                data = json.loads(data) if isinstance(data, str) else data
                output = Path(args.output)
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
                for unit in data.get("startup_units", []):
                    print(unit["id"], unit["status"], unit["evidence"])
                for ex in data.get("execution", {}).get("active", []):
                    print("ACTIVE", ex["incident_key"], ex["state"], ex["message"])
                for ex in data.get("execution", {}).get("finished", [])[:8]:
                    print("FINISHED", ex["incident_key"], ex["state"], ex["message"])
                requests = data.get("execution", {}).get("operator_requests", [])
                for req in requests:
                    print("REQUEST", req["id"], req["incident_key"], req["kind"], req["buttons"])
                if args.approve_component:
                    if data.get("environment", {}).get("state") != "RUNNING":
                        raise RuntimeError("Environment not running; no approval sent")
                    matching = [r for r in requests if r["incident_key"] == args.approve_component
                                and r["kind"] == "confirm_action" and "approve" in r["buttons"]
                                and (r.get("step") or {}).get("primitive") == args.approve_primitive]
                    if len(matching) != 1:
                        raise RuntimeError("No unique offered restart approval; no action sent")
                    request = matching[0]
                    await ws.send_json({"type": "recovery_command", "command": {
                        "type": "operator_reply", "request_id": request["id"], "reply": "approve"}})
                    print("APPROVAL_SENT", request["id"], args.approve_component)
                    await asyncio.sleep(1)
                return


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--approve-component", choices=["sim_lift", "sim_drive", "sim_lidar_adapter", "nav2", "fast_lio"])
    parser.add_argument("--approve-primitive", choices=["restart_component", "lifecycle_reset"], default="restart_component")
    asyncio.run(asyncio.wait_for(check(parser.parse_args()), 20))
