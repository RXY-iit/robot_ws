#!/usr/bin/env python3
"""Opt-in isolated-domain check; never connects to the robot's domain 20."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from unittest.mock import patch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 200 <= args.domain <= 220:
        parser.error("Only dedicated test domains 200..220 are allowed")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ.pop("CYCLONEDDS_URI", None)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from robot_agent.recovery.backends import RosActionBackend
    from robot_agent.recovery.environment import load_env_profile

    workspace = Path(__file__).resolve().parents[2]
    env = load_env_profile(workspace / "src/robot_agent/config/recovery", "sim")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("isolated_recovery_bootstrap_check")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    children, observed, actions = [], {}, []
    nonzero = []
    subscriptions = []
    for topic in ("/robot_mode", "/safety_status"):
        subscriptions.append(node.create_subscription(String, topic,
            lambda m, topic=topic: observed.setdefault(topic, []).append(m.data), 10))
    def velocity(message):
        values = [message.linear.x, message.linear.y, message.angular.z]
        observed.setdefault("/cmd_vel", []).append(values)
        if any(abs(v) > 1e-9 for v in values):
            nonzero.append(values)
    subscriptions.append(node.create_subscription(Twist, "/cmd_vel", velocity, 10))
    original_popen = subprocess.Popen
    def launch(command, **kwargs):
        proc = original_popen(command, **kwargs)
        if command[0] == "ros2":
            children.append(proc)
        return proc
    result = {"domain": args.domain, "scope": "isolated bootstrap, not full robot recovery", "passed": False}
    try:
        time.sleep(2)
        if set(node.get_node_names()) - {node.get_name()}:
            raise RuntimeError("Test domain already has nodes; refusing to start anything")
        # No localization action clients/publishers are needed for this test.
        backend = RosActionBackend(node, env, {}, workspace=str(workspace), log_dir=str(args.output.parent))
        with patch("robot_agent.recovery.backends.subprocess.Popen", side_effect=launch):
            for component in ("mode_switch", "safety_node"):
                actions.append(backend._restart(component, lambda _: None))
                actions.append(backend._restart(component, lambda _: None))
        time.sleep(3)
        assert len(children) == 2, "Duplicate bootstrap"
        assert observed.get("/robot_mode") and set(observed["/robot_mode"]) == {"MANUAL"}
        assert observed.get("/safety_status")
        assert observed.get("/cmd_vel") and not nonzero
        result.update(passed=True, actions=actions, samples={k: len(v) for k, v in observed.items()},
                      modes=sorted(set(observed["/robot_mode"])), safety=sorted(set(observed["/safety_status"])),
                      child_pids=[p.pid for p in children], nonzero_velocity=nonzero)
    finally:
        cleanup = []
        for proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
                cleanup.append({"pid": proc.pid, "returncode": proc.returncode})
            except subprocess.TimeoutExpired:
                cleanup.append({"pid": proc.pid, "error": "SIGINT did not finish; inspect manually"})
                result["passed"] = False
        result["cleanup"] = cleanup
        executor.shutdown(timeout_sec=3)
        thread.join(3)
        node.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
