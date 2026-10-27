from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String

from robot_agent.recovery.backends import RosActionBackend
from robot_agent.recovery.engine import RecoveryEngine
from robot_agent.recovery.environment import load_env_profile
from robot_agent.recovery.playbook import load_playbooks
from robot_agent.recovery.primitives import LocalSystemAdapter, build_default_registry
from robot_agent.recovery.snapshot import HealthSnapshot
from robot_agent.stack_lifecycle import ShutdownParticipant


SIM_MODES = {"sim", "simulation", "isaac", "isaac_sim"}
# Where each quick-start launcher saves its last-started options, and the key
# holding the mode that matches envs/<kind>.yaml launch_profiles.
LAUNCHER_SETTINGS = {
    "sim": (Path.home() / ".config" / "robot-sim-launcher" / "settings.json", "ros_mode"),
    "real": (Path.home() / ".config" / "realbot-launcher" / "settings.json", "robot_mode"),
}


def profile_from_launcher_settings(kind: str, path_override: str = "") -> tuple[str, str]:
    """Fallback when no profile was passed: the mode the launcher last started."""
    path, key = LAUNCHER_SETTINGS[kind]
    if path_override:
        path = Path(path_override).expanduser()
    try:
        value = str(json.loads(path.read_text()).get(key) or "").strip()
    except (OSError, ValueError, AttributeError):
        return "", ""
    return (value, f"launcher settings {path} ({key})") if value else ("", "")


class RosGraphAdapter:
    def __init__(self, node: Node) -> None:
        self._node = node

    def node_names(self) -> set[str]:
        return {
            f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            for name, namespace in self._node.get_node_names_and_namespaces()
        }

    def publisher_count(self, topic: str) -> int:
        return self._node.count_publishers(topic)

    def node_counts(self) -> dict[str, int]:
        from collections import Counter
        return dict(Counter(f"{namespace.rstrip('/')}/{name}"
                            for name, namespace in self._node.get_node_names_and_namespaces()))


class RecoveryAgentNode(Node):
    """Recovery Agent.

    Heartbeat trigger: diagnoses every /agent/health_dashboard_state snapshot.
    Mission trigger:   /agent/recovery_request -> /agent/recovery_response
                       (IN_PROGRESS first, then a `final: true` follow-up).
    Operator channel:  /agent/recovery_command (Dashboard replies / acknowledge
                       / retry / abort), requests listed in /agent/recovery_state.
    """

    def __init__(self) -> None:
        super().__init__("recovery_agent")
        self.declare_parameter("mode", "real")
        self.declare_parameter("config_dir", "")
        # Launch profile from envs/<kind>.yaml launch_profiles, normally the
        # quick-start launcher mode (sim: nav2 | arm_test). Empty = everything
        # is expected. Items of components the mode does not start are ignored.
        self.declare_parameter("launch_profile", "")
        # "auto" = execute in sim, diagnose-only on the real robot. Dynamic type:
        # `execute:=false` arrives as BOOL, `execute:=auto` as STRING.
        self.declare_parameter("execute", "auto", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("workspace", os.environ.get("ROBOT_WS", "/home/ruan-x/robot_ws"))
        self.declare_parameter("map_ply", "")
        self.declare_parameter(
            "ack_store", ""
        )
        # Used only when launch_profile is empty ("" = default launcher path, "none" = disabled).
        self.declare_parameter("launcher_settings", "")
        self.declare_parameter("state_publish_min_interval_sec", 1.0)
        self.declare_parameter("tick_period_sec", 0.5)

        mode = str(self.get_parameter("mode").value).strip().lower()
        kind = "sim" if mode in SIM_MODES else "real"
        config_dir = str(self.get_parameter("config_dir").value) or str(
            Path(get_package_share_directory("robot_agent")) / "config" / "recovery"
        )
        env = load_env_profile(Path(config_dir), kind)
        registry = build_default_registry()
        library = load_playbooks(Path(config_dir) / "playbooks", env, registry)

        profile = str(self.get_parameter("launch_profile").value).strip()
        profile_source = "launch parameter" if profile else "none (all components expected)"
        launcher_settings = str(self.get_parameter("launcher_settings").value).strip()
        if not profile and launcher_settings not in {"", "none"}:
            # E.g. a quick-start GUI started before it passed RECOVERY_PROFILE.
            fallback, source = profile_from_launcher_settings(kind, launcher_settings)
            if fallback:
                profile, profile_source = fallback, source
                self.get_logger().warning(f"No launch_profile passed; using '{profile}' from {source}")
        launch_profile = None
        if profile:
            launch_profile = env.launch_profiles.get(profile)
            if launch_profile is None:
                # Do not crash the dashboard stack over a launcher/profile typo.
                self.get_logger().error(
                    f"Unknown launch_profile {profile!r}; envs/{kind}.yaml defines "
                    f"{sorted(env.launch_profiles)}. Expecting every component."
                )

        execute_param = str(self.get_parameter("execute").value).strip().lower()  # True -> "true"
        execute = kind == "sim" if execute_param == "auto" else execute_param in {"1", "true", "yes", "on"}
        backend = RosActionBackend(
            self,
            env,
            env.actions,
            workspace=str(self.get_parameter("workspace").value),
            map_ply=str(self.get_parameter("map_ply").value),
        )
        self._lock = threading.RLock()
        self.engine = RecoveryEngine(
            env,
            library,
            registry,
            graph=RosGraphAdapter(self),
            system=LocalSystemAdapter(),
            launch_profile=launch_profile,
            execution_enabled=execute,
            backend=backend,
            ack_store=(Path(str(self.get_parameter("ack_store").value).replace("{kind}", kind))
                       if self.get_parameter("ack_store").value else None),
        )
        self.engine.launch_profile_source = profile_source if launch_profile else "none (all components expected)"
        group = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, "/agent/recovery_state", 10)
        self.events_pub = self.create_publisher(String, "/agent/recovery_events", 10)
        self.response_pub = self.create_publisher(String, "/agent/recovery_response", 10)
        self.create_subscription(String, "/agent/health_dashboard_state", self._on_health, 10, callback_group=group)
        self.create_subscription(String, "/agent/recovery_request", self._on_request, 10, callback_group=group)
        self.create_subscription(String, "/agent/recovery_command", self._on_command, 10, callback_group=group)
        self.create_subscription(String, "/agent/mission_state", self._on_mission, 10, callback_group=group)
        self.create_timer(float(self.get_parameter("tick_period_sec").value), self._on_tick, callback_group=group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._last_state_publish = 0.0
        self._shutdown = ShutdownParticipant(self, "recovery_agent", self._prepare_shutdown, self._shutdown_ready)
        self.get_logger().info(
            f"Recovery agent env={kind} execute={self.engine.executor.enabled} "
            f"(require_verified={env.require_verified}) playbooks={len(library.playbooks)} active / "
            f"{len(library.inactive)} n/a, launch_profile={launch_profile.name if launch_profile else 'all'}"
        )

    def _on_mission(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            self.engine.mission_state = state

    def _prepare_shutdown(self):
        with self._lock:
            self.engine.prepare_shutdown(time.monotonic())
            self._flush(force_state=True)

    def _shutdown_ready(self):
        with self._lock:
            return self.engine.shutdown_ready()

    def _on_health(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning("Ignoring non-JSON health_dashboard_state")
            return
        with self._lock:
            self.engine.on_snapshot(HealthSnapshot.from_state(state, time.monotonic()))
            self._flush()

    def _on_tick(self) -> None:
        with self._lock:
            self.engine.tick(time.monotonic())
            self._flush()

    def _on_request(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            data = None
        if not isinstance(data, dict):
            self.response_pub.publish(String(data=json.dumps({"status": "INVALID_REQUEST", "raw": msg.data[:200]})))
            return
        with self._lock:
            response = self.engine.handle_request(data)
            self._publish_response(response)
            self._flush(force_state=True)

    def _on_command(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning(f"Ignoring non-JSON recovery_command: {msg.data[:120]}")
            return
        if not isinstance(data, dict):
            return
        with self._lock:
            result = self.engine.handle_command(data, time.monotonic())
            self.get_logger().info(f"recovery_command {data.get('type')}: {result['result']}")
            self._flush(force_state=True)

    def _publish_response(self, response: dict) -> None:
        self.response_pub.publish(String(data=json.dumps(response, ensure_ascii=False, default=str)))
        self.get_logger().info(
            f"recovery_response {response.get('request_id')}: {response.get('status')} "
            f"final={response.get('final')} ({response.get('summary', '')})"
        )

    def _flush(self, force_state: bool = False) -> None:
        for response in self.engine.drain_responses():
            self._publish_response(response)
        for event in self.engine.drain_events():
            self.events_pub.publish(String(data=json.dumps(event, ensure_ascii=False, default=str)))
            self.get_logger().info(
                f"{event['type']} "
                + " ".join(
                    f"{k}={event[k]}"
                    for k in ("incident_id", "key", "playbook_id", "state", "status", "message")
                    if event.get(k) not in (None, "")
                )
            )
        now = time.monotonic()
        min_interval = float(self.get_parameter("state_publish_min_interval_sec").value)
        if not force_state and now - self._last_state_publish < min_interval:
            return
        self._last_state_publish = now
        self.state_pub.publish(String(data=json.dumps(self.engine.state(), ensure_ascii=False, default=str)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryAgentNode()
    # Multi-threaded: action worker threads wait on service futures that
    # other executor threads complete.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
