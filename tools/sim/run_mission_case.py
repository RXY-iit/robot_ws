#!/usr/bin/env python3
"""Run one explicitly selected sim case through the real Dashboard interface.

Does not inject detector results or disable safety gates. The caller must inspect
the scene before motion. Records compact references, not duplicate mission logs.
"""
import argparse
import asyncio
import json
from pathlib import Path
import time

import aiohttp


def decode(message):
    data = message.get("data")
    if isinstance(data, str):
        try:
            data = json.loads(data)
        except ValueError:
            pass
    return message.get("topic"), data


async def receive_event(ws):
    message = await ws.receive()
    if message.type == aiohttp.WSMsgType.TEXT:
        return decode(json.loads(message.data))
    if message.type in {
        aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED,
        aiohttp.WSMsgType.CLOSING, aiohttp.WSMsgType.ERROR,
    }:
        raise ConnectionError(f"Dashboard transport ended: {message.type.name}")
    return None, None


def is_terminal(status):
    return status in {"DONE", "DONE_WITH_STUB", "CANCELLED"} or str(status or "").startswith("FAILED")


class HumanWaitBudget:
    """Only the current unanswered human wait may expire, not a past wait."""
    def __init__(self):
        self.since = None
        self.action_sent = False

    def observe(self, status, now):
        if status != "NEEDS_HUMAN":
            self.since = None
            self.action_sent = False
        elif self.since is None:
            self.since = now

    def expired(self, now, limit):
        return self.since is not None and not self.action_sent and now - self.since > limit


async def run(args):
    latest, samples, mission, held = {}, [], None, None
    sent_action = False
    wait_budget = HumanWaitBudget()
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(args.url, heartbeat=10) as ws:
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                try:
                    topic, data = await asyncio.wait_for(receive_event(ws), 1)
                    latest[topic] = data
                except asyncio.TimeoutError:
                    continue
            previous = latest.get("/agent/mission_state", {})
            if previous.get("status") and not is_terminal(previous["status"]):
                raise RuntimeError("Existing mission is not terminal; do not supersede it")
            health = latest.get("/agent/health_state", {})
            if health.get("components", {}).get("localization", {}).get("details", {}).get("mode") != "sim":
                raise RuntimeError("Live sim evidence is required")
            await ws.send_json({"type": "agent_mode", "mode": args.mode})
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                try:
                    topic, data = await asyncio.wait_for(receive_event(ws), 1)
                except asyncio.TimeoutError:
                    continue
                if topic == "/agent/mode_state" and data.get("agent_mode") == args.mode:
                    break
            else:
                raise RuntimeError("Agent mode acknowledgement missing")
            start_pose = health.get("components", {}).get("localization", {}).get("details", {})
            await ws.send_json({"type": "command", "text": args.command})
            started, cancel_sent, cancel_due = time.monotonic(), False, None
            transport_error = None
            try:
                while time.monotonic() - started < args.timeout + 8:
                    try:
                        topic, data = await asyncio.wait_for(receive_event(ws), 1)
                    except asyncio.TimeoutError:
                        topic, data = None, None
                    if topic:
                        latest[topic] = data
                    if topic == "/agent/mission_state" and data.get("mission_id") != previous.get("mission_id"):
                        if mission and data.get("mission_id") != mission["mission_id"]:
                            raise RuntimeError("Another command superseded this test; leave the new mission alone")
                        mission = data
                        wait_budget.observe(mission.get("status"), time.monotonic())
                        if data.get("status") == getattr(args, "cancel_on_status", None) and cancel_due is None:
                            cancel_due = time.monotonic() + getattr(args, "cancel_delay", 0.0)
                        signature = (data.get("status"), data.get("current_step_index"), len(data.get("skill_results", [])))
                        if not samples or samples[-1]["signature"] != list(signature):
                            samples.append({"elapsed": round(time.monotonic() - started, 2), "signature": list(signature)})
                        if is_terminal(mission.get("status")):
                            break
                        wait = mission.get("human_wait") or {}
                        if args.human_action and not sent_action and args.human_action in wait.get("actions", []):
                            await ws.send_json({"type": "mission_control", "command": {**wait, "action": args.human_action}})
                            sent_action = True
                            wait_budget.action_sent = True
                        elif mission.get("status") == "NEEDS_HUMAN":
                            if held is None:
                                held = {"time": time.monotonic()}
                            # The first NEEDS_HUMAN frame can precede its wait/options.
                            ambiguity = ((mission.get("intent") or {}).get("constraints") or {}).get("ambiguity") or {}
                            held.update(question=mission.get("question_for_human"),
                                        task_count=len(mission.get("tasks", [])),
                                        human_wait=wait,
                                        choices=ambiguity.get("choices", []),
                                        status_before_test_cancel=mission.get("status"))
                    elapsed = time.monotonic() - started
                    is_cancel_skill = (topic == "/agent/mission_events" and isinstance(data, dict)
                        and mission and data.get("mission_id") == mission["mission_id"]
                        and data.get("type") == "skill_started"
                        and data.get("payload", {}).get("skill") == args.cancel_on_skill)
                    if is_cancel_skill and cancel_due is None:
                        cancel_due = time.monotonic() + getattr(args, "cancel_delay", 0.0)
                    own_mission = mission and latest.get("/agent/mission_state", {}).get("mission_id") == mission["mission_id"]
                    if own_mission and not cancel_sent and ((cancel_due is not None and time.monotonic() >= cancel_due) or elapsed > args.timeout or
                        wait_budget.expired(time.monotonic(), args.hold_seconds)):
                        await ws.send_json({"type": "cancel"})
                        cancel_sent = True
            except (ConnectionError, aiohttp.ClientError) as exc:
                transport_error = str(exc)
            finally:
                if not ws.closed and not transport_error and mission and not is_terminal(mission.get("status")) and latest.get("/agent/mission_state", {}).get("mission_id") == mission["mission_id"]:
                    try:
                        await ws.send_json({"type": "cancel"})
                        cancel_sent = True
                    except (ConnectionError, aiohttp.ClientError) as exc:
                        transport_error = str(exc)
            if not mission and not transport_error:
                raise RuntimeError("No mission response")
            # Allow the terminal health/report messages to settle without new commands.
            until = time.monotonic() + 2
            while not transport_error and time.monotonic() < until:
                try:
                    topic, data = await asyncio.wait_for(receive_event(ws), 1)
                    latest[topic] = data
                except asyncio.TimeoutError:
                    continue
                except (ConnectionError, aiohttp.ClientError) as exc:
                    transport_error = str(exc)
                    break
            mission = mission or {}
            settled = latest.get("/agent/mission_state", {})
            if mission.get("mission_id") and settled.get("mission_id") == mission["mission_id"]:
                mission = settled
            end_loc = latest.get("/agent/health_state", {}).get("components", {}).get("localization", {}).get("details", {})
            result = {"case": args.case, "mode": args.mode, "command": args.command,
                "mission_id": mission.get("mission_id"), "status": mission.get("status"), "outcome": mission.get("outcome"),
                "observation_status": "TRANSPORT_LOST" if transport_error else "OBSERVED",
                "transport_error": transport_error,
                "final_status_confirmed": is_terminal(mission.get("status")),
                "message": mission.get("final_message"), "held": held, "human_action_sent": sent_action,
                "cancel_sent": cancel_sent, "elapsed_sec": round(time.monotonic() - started, 2),
                "start_wheel_pose": start_pose.get("latest_wheel_odom_pose"), "end_wheel_pose": end_loc.get("latest_wheel_odom_pose"),
                "start_map_pose": start_pose.get("last_trusted_pose"), "end_map_pose": end_loc.get("last_trusted_pose"),
                "tasks": [{k: t.get(k) for k in ("task_type", "owner", "state", "outcome", "goal_satisfied", "navigation_arrival")} for t in mission.get("tasks", [])],
                "skills": [{"name": r.get("skill_name"), "code": r.get("code"), "success": r.get("success"), "message": r.get("message"),
                            "metrics": {k:v for k,v in (r.get("metrics") or {}).items() if k in {
                                "outcome", "goal_satisfied", "inference_completed", "distance_m", "yaw_deg",
                                "completion_class", "distance_traveled_m", "yaw_traveled_deg",
                                "final_xy_error_m", "final_yaw_error_rad", "final_pose_source",
                                "nav2_status", "global_plan_error"}}} for r in mission.get("skill_results", [])],
                "decisions": [{k:t.get(k) for k in ("stage", "provider", "adopted_decision", "latency_ms", "fallback_used")} for t in mission.get("jev_trace", {}).get("calls", [])],
                "transitions": samples}
            output = Path(args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open("a") as stream:
                stream.write(json.dumps(result, ensure_ascii=False) + "\n")
            print(json.dumps(result, ensure_ascii=False))
            return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8765/ws")
    parser.add_argument("--case", required=True)
    parser.add_argument("--command", required=True)
    parser.add_argument("--mode", choices=["classic", "jev_primary", "jev_shadow"], default="classic")
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--hold-seconds", type=float, default=5)
    parser.add_argument("--human-action")
    parser.add_argument("--cancel-on-skill")
    parser.add_argument("--cancel-on-status", help="Cancel during a Mission phase, e.g. PARSE, without waiting for a skill")
    parser.add_argument("--cancel-delay", type=float, default=0.0)
    parser.add_argument("--output", required=True)
    asyncio.run(run(parser.parse_args()))
