#!/usr/bin/env python3
"""Read-only, bounded-rate dashboard evidence collection; never sends commands."""
import argparse
import asyncio
import datetime
import json
import pathlib
import time

import aiohttp


async def observe(args):
    latest = {}
    started = time.monotonic()
    last_sample = -10.0
    output = pathlib.Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    samples = 0
    unhealthy = 0
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(args.url, heartbeat=10) as ws:
            with output.open("w") as log:
                while time.monotonic() - started < args.seconds:
                    try:
                        message = await asyncio.wait_for(ws.receive_json(), 3)
                    except asyncio.TimeoutError:
                        message = {}
                    topic, data = message.get("topic"), message.get("data")
                    if isinstance(data, str):
                        try:
                            data = json.loads(data)
                        except ValueError:
                            data = {}
                    if topic and isinstance(data, dict):
                        latest[topic] = data
                    elapsed = time.monotonic() - started
                    if elapsed - last_sample < 5:
                        continue
                    last_sample = elapsed
                    health = latest.get("/agent/health_state", {})
                    mission = latest.get("/agent/mission_state", {})
                    recovery = latest.get("/agent/recovery_state", {})
                    loc = health.get("components", {}).get("localization", {})
                    record = {
                        "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                        "elapsed_wall_sec": elapsed,
                        "health_timestamp": health.get("timestamp"),
                        "localization": loc,
                        "mission": {k: mission.get(k) for k in ("mission_id", "status", "user_text")},
                        "incidents": [{k: i.get(k) for k in ("id", "key", "status", "execution_state")}
                                      for i in recovery.get("incidents", [])],
                    }
                    log.write(json.dumps(record, ensure_ascii=False) + "\n")
                    log.flush()
                    samples += 1
                    unhealthy += loc.get("level") != "OK"
    print(json.dumps({"samples": samples, "non_ok_samples": unhealthy,
                      "elapsed_wall_sec": time.monotonic() - started, "output": str(output)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8765/ws")
    parser.add_argument("--seconds", type=float, default=1200)
    parser.add_argument("--output", required=True)
    asyncio.run(observe(parser.parse_args()))
