#!/usr/bin/env python3
"""One-off B1 baseline: sequential NavigateToPose tasks, no Mission/Recovery Agent.

Default only validates/prints the route. --execute sends real simulation goals.
Health gating scope is explicit (--health-gate route/waypoint); in-task recovery belongs to the configured Nav2 BT.
"""
import argparse
import json
import math
from pathlib import Path
import time

import yaml


# Allow validation from a plain Python shell without ROS sourcing.
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/robot_skills"))
from robot_skills.waypoint_routes import read_route


def load_route(path):
    return read_route(path)["poses"]


def dispatch_blockers(nodes, latest, now, *, check_navigation=True):
    blockers = []
    for name in ("agent_node", "agent_dry_run", "recovery_agent"):
        if any(node.rsplit("/", 1)[-1] == name for node in nodes):
            blockers.append(f"Agent-layer node still running: {name}")
    for topic in (("clock", "robot_mode", "safety_status", "health") if check_navigation else ("clock", "robot_mode", "safety_status")):
        if topic not in latest or not 0 <= now - latest[topic][0] <= 3:
            blockers.append(f"Missing/stale {topic}")
    if latest.get("robot_mode", (None, None))[1] != "AUTO":
        blockers.append("Select AUTO through the normal operator control")
    # Idle cmd_vel watchdog publishes zero; COR permits the same fresh status.
    # ESTOP/MANUAL_STOP/TOUCH_LOCK and stale evidence remain hard blocks.
    if latest.get("safety_status", (None, None))[1] not in {"OK", "WATCHDOG"}:
        blockers.append("Safety gate is not OK or idle WATCHDOG")
    if not latest.get("clock_advancing", False):
        blockers.append("Simulation clock has not advanced")
    health = latest.get("health", (None, {}))[1]
    capabilities = health.get("capabilities", []) if isinstance(health, dict) else []
    navigation = next((cap for cap in capabilities if cap.get("id") == "auto_navigation"), {})
    if check_navigation and navigation.get("status") != "AVAILABLE":
        blockers.append(f"Navigation precondition is {navigation.get('status', 'UNKNOWN')}: {navigation.get('summary', '')}")
    return blockers


def execute(args, route):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from nav2_msgs.action import NavigateToPose
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node("b1_tor_waypoint_runner", parameter_overrides=[Parameter("use_sim_time", value=True)])
    event_pub = node.create_publisher(String,"/b1/waypoint_events",10)
    latest, active, pending, stopping = {}, None, None, False
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        log = output.open("x", encoding="utf-8")
    except Exception:
        node.destroy_node()
        rclpy.shutdown()
        raise

    def record(event, **data):
        row = {"event": event, "wall_monotonic_sec": time.monotonic(),
               "wall_unix_sec": time.time(), "sim_sec": latest.get("clock", (None, None))[1], **data}
        log.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        log.flush()

    def route_event(event, point, handle):
        event_pub.publish(String(data=json.dumps(dict(event=event,goal_xy=[point['x'],point['y']],
            goal_id=bytes(handle.goal_id.uuid).hex(),frame_id='map',source='TOR',
            wall_unix_sec=time.time(),wall_monotonic_sec=time.monotonic(),sim_sec=latest.get('clock',(None,None))[1]))))

    def clock_cb(msg):
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        previous = latest.get("clock", (None, stamp))[1]
        latest["clock_advancing"] = stamp > previous
        latest["clock"] = (time.monotonic(), stamp)

    def string_cb(key, msg):
        value = msg.data
        if key == "health":
            try:
                value = json.loads(value)
            except (TypeError, ValueError):
                value = {}
        latest[key] = (time.monotonic(), value)

    node.create_subscription(Clock, "/clock", clock_cb, 10)
    for key, topic in (("robot_mode", "/robot_mode"), ("safety_status", "/safety_status"),
                       ("health", "/agent/health_dashboard_state")):
        node.create_subscription(String, topic, lambda msg, key=key: string_cb(key, msg), 10)

    def pump_until(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not future.done():
            raise TimeoutError("Nav2 acknowledgement/result timed out")
        return future.result()

    def goal_received(future):
        nonlocal active
        active = future.result()
        if stopping and active.accepted:
            active.cancel_goal_async()

    feedback_at = 0.0
    def feedback(msg):
        nonlocal feedback_at
        now = time.monotonic()
        if now - feedback_at >= .5:
            feedback_at = now
            f = msg.feedback
            record("nav2_feedback", distance_remaining=float(f.distance_remaining),
                   number_of_recoveries=int(f.number_of_recoveries),
                   x=float(f.current_pose.pose.position.x), y=float(f.current_pose.pose.position.y))

    exit_code = 1
    try:
        record("run_start", condition="TOR_waypoint_nav2", route=route,
               route_path=str(Path(args.route).resolve()), route_sha256=args.route_sha256, bt=args.bt or "server_default",
               task_timeout_wall_sec=args.timeout, health_gate=args.health_gate)
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError("NavigateToPose action server is unavailable")
        for index, point in enumerate(route):
            # This gate is intentionally not polled while a task is executing.
            preflight_started = time.monotonic()
            deadline = preflight_started + 4
            blockers = ["Waiting for pre-dispatch evidence"]
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                blockers = dispatch_blockers(node.get_node_names(), latest, time.monotonic(),
                                             check_navigation=(index == 0 or args.health_gate == "waypoint"))
                if not blockers:
                    break
            record("task_preflight", index=index, blockers=blockers,
                   elapsed_wall_sec=time.monotonic() - preflight_started)
            if blockers:
                raise RuntimeError("; ".join(blockers))
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(point["x"]), float(point["y"])
            goal.pose.pose.orientation.z = math.sin(point["yaw"] / 2)
            goal.pose.pose.orientation.w = math.cos(point["yaw"] / 2)
            goal.behavior_tree = args.bt
            record("task_dispatch", index=index, pose=point)
            pending = client.send_goal_async(goal, feedback_callback=feedback)
            pending.add_done_callback(goal_received)
            active = pump_until(pending, 10)
            if not active.accepted:
                raise RuntimeError(f"Nav2 rejected task {index}")
            route_event("accepted",point,active)
            result = pump_until(active.get_result_async(), args.timeout)
            route_event("succeeded" if result.status == 4 else "terminated",point,active)
            record("task_result", index=index, status=int(result.status))
            active = None
            if result.status != 4:
                raise RuntimeError(f"Nav2 task {index} did not succeed: status={result.status}")
        record("run_done")
        exit_code = 0
    except KeyboardInterrupt:
        record("operator_interrupt")
        exit_code = 130
    except Exception as exc:
        record("run_failed", reason=str(exc))
        print(str(exc))
    finally:
        stopping = True
        try:
            if pending is not None and not pending.done():
                pump_until(pending, 5)
            if active is not None and active.accepted:
                record("cancel_requested")
                pump_until(active.cancel_goal_async(), 5)
                result = pump_until(active.get_result_async(), 10)
                record("cancel_terminal", status=int(result.status))
        except Exception as exc:
            record("cancel_unconfirmed", reason=str(exc))
            print("Goal termination is unconfirmed; use the normal MANUAL/stop control and inspect Nav2.")
            exit_code = 1
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", required=True, help="YAML with frame_id: map and waypoints: [{x, y, yaw}]")
    parser.add_argument("--health-gate", choices=["route", "waypoint"], default="waypoint",
                        help="route: health only before the entire list; waypoint: legacy per-goal health gate")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", help="New JSONL evidence file, required with --execute")
    parser.add_argument("--bt", default="", help="Explicit Nav2 BT path; default uses the server's configured BT")
    parser.add_argument("--timeout", type=float, default=360, help="Wall-clock seconds per Nav2 task")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    try:
        document = read_route(args.route)
        route = document["poses"]
        args.route_sha256 = document["sha256"]
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    if not args.execute:
        print(json.dumps({"condition": "TOR_waypoint_nav2", "execute": False,
                          "waypoints": route, "route_sha256": args.route_sha256, "bt": args.bt or "server_default"}, ensure_ascii=False))
        return 0
    if not args.output:
        parser.error("--execute requires --output")
    return execute(args, route)


if __name__ == "__main__":
    raise SystemExit(main())
