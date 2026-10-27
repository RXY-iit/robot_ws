from __future__ import annotations

import os
import fcntl
import json
import math
import shlex
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from .environment import EnvProfile
from .executor import DONE, ERROR, RUNNING
from .primitives import same_domain_pids
from .trusted_pose import trusted_seed, propagate_base_pose


class ThreadHandle:
    """Runs a blocking action function in a thread; fn returns a detail string or raises."""

    def __init__(self, fn: Callable[[Callable[[str], None]], str]) -> None:
        self._state = RUNNING
        self._detail = "started"
        self._lock = threading.Lock()
        threading.Thread(target=self._run, args=(fn,), daemon=True).start()

    def _progress(self, detail: str) -> None:
        with self._lock:
            self._detail = detail

    def _run(self, fn: Callable[[Callable[[str], None]], str]) -> None:
        try:
            detail = fn(self._progress)
            with self._lock:
                self._state, self._detail = DONE, detail
        except Exception as exc:
            with self._lock:
                self._state, self._detail = ERROR, str(exc) or repr(exc)

    def poll(self) -> tuple[str, str]:
        with self._lock:
            return self._state, self._detail


class ImmediateHandle:
    def __init__(self, state: str, detail: str) -> None:
        self._result = (state, detail)

    def poll(self) -> tuple[str, str]:
        return self._result


def _signal_pids(pids: list[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except OSError:
            pass


def _wait_future(future: Any, timeout_sec: float, what: str) -> Any:
    deadline = time.monotonic() + timeout_sec
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError(f"{what} timed out after {timeout_sec:.0f}s")
        time.sleep(0.05)
    return future.result()


class RosActionBackend:
    """Executes recovery primitives against the live ROS graph / host.

    Requires the owning node to spin with a multi-threaded executor so that
    service futures complete while worker threads wait on them.

    Env config (envs/<kind>.yaml):
      components.<id>.restart: {kill_pattern, command: [...]}  -> restart_component
      actions: {lifecycle_manager, local_costmap_clear, global_costmap_clear,
                navigate_cancel, auto_initial_align: {script, topic, args}}
    """

    def __init__(
        self,
        node: Any,
        env: EnvProfile,
        actions_config: dict[str, Any],
        *,
        workspace: str,
        map_ply: str = "",
        log_dir: str = "",
    ) -> None:
        self.node = node
        self.env = env
        self.config = actions_config or {}
        self.workspace = workspace
        self.map_ply = map_ply
        self.log_dir = Path(log_dir or Path(workspace) / "debug-output" / "recovery")
        self._clients: dict[tuple[str, str], Any] = {}
        self._localization_metrics: dict = {}
        self._health_received = 0.0
        if self.config.get("last_trusted_pose"):
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from std_msgs.msg import String
            from tf2_ros import Buffer, TransformListener
            self._pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, node)
            self._health_sub = node.create_subscription(String, "/agent/health_state", self._on_agent_health, 10)

    def _on_agent_health(self, msg: Any) -> None:
        try:
            data = json.loads(msg.data)
            metrics = data["components"]["localization"]["details"]
            if isinstance(metrics, dict):
                self._localization_metrics = dict(metrics)
                self._health_received = time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    # ------------------------------------------------------------------ dispatch

    def supports(self, primitive: str, args: dict[str, Any]) -> tuple[bool, str]:
        if primitive == "restart_component":
            comp = self.env.components.get(str(args.get("component")))
            if comp is None:
                return False, f"unknown component {args.get('component')!r}"
            if not comp.restart:
                return False, f"no restart command configured for {comp.id} in {self.env.kind}"
            return True, ""
        if primitive == "lifecycle_reset":
            return bool(self.config.get("lifecycle_manager") or args.get("manager")), "no lifecycle manager configured"
        if primitive == "clear_costmap":
            key = f"{args.get('which', 'local')}_costmap_clear"
            return bool(self.config.get(key)), f"{key} not configured"
        if primitive == "cancel_motion":
            return bool(self.config.get("navigate_cancel")), "navigate_cancel not configured"
        if primitive == "publish_initialpose":
            source = str(args.get("source", ""))
            if source == "last_trusted_pose":
                return bool(self.config.get(source)), "last_trusted_pose not configured for this environment"
            if source == "auto_initial_align":
                if not (self.config.get("auto_initial_align") or {}).get("script"):
                    return False, "auto_initial_align not configured"
                if not self.map_ply:
                    return False, "no map PLY (launcher RECOVERY_MAP_PLY / map_ply parameter)"
                return True, ""
            return False, f"initial pose source {source!r} not implemented yet"
        if primitive == "notify_mission":
            return True, ""
        return False, f"{primitive} has no {self.env.kind} backend"

    def start(self, primitive: str, args: dict[str, Any]):
        if primitive == "restart_component":
            return ThreadHandle(lambda progress: self._restart(str(args["component"]), progress))
        if primitive == "lifecycle_reset":
            return ThreadHandle(lambda progress: self._lifecycle_reset(str(args.get("manager") or self.config["lifecycle_manager"]), progress))
        if primitive == "clear_costmap":
            service = self.config[f"{args.get('which', 'local')}_costmap_clear"]
            return ThreadHandle(lambda progress: self._clear_costmap(service, progress))
        if primitive == "cancel_motion":
            return ThreadHandle(lambda progress: self._cancel_navigation(self.config["navigate_cancel"], progress))
        if primitive == "publish_initialpose":
            if args.get("source") == "last_trusted_pose":
                # Synchronous publish: no delayed worker can reseed after Cancel.
                try:
                    return ImmediateHandle(DONE, self._publish_trusted_pose())
                except Exception as exc:
                    return ImmediateHandle(ERROR, str(exc))
            if args.get("source") != "auto_initial_align":
                return ImmediateHandle(ERROR, "Unsupported initial pose source")
            return ThreadHandle(lambda progress: self._auto_initial_align(progress))
        if primitive == "notify_mission":
            return ImmediateHandle(DONE, "mission notified via recovery events")
        return ImmediateHandle(ERROR, f"{primitive} unsupported")

    # ------------------------------------------------------------------ actions

    def _publish_trusted_pose(self) -> str:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.time import Time

        spec = self.config["last_trusted_pose"]
        x, y, yaw = trusted_seed(self._localization_metrics, spec, time.monotonic() - self._health_received)
        if spec.get("require_timestamped_base", False):
            stamped = self._tf_buffer.lookup_transform("odom", spec.get("base_frame", "base_footprint"), Time())
            source_stamp = stamped.header.stamp.sec + stamped.header.stamp.nanosec / 1e9
            # ROS TF timestamp is the geometry time. Report receipt uses monotonic time.
            clock_now = self.node.get_clock().now().nanoseconds / 1e9
            if not 0 <= clock_now - source_stamp <= float(spec.get("max_odom_age_sec", 3)):
                raise ValueError("Latest odometry TF source time is stale")
            if source_stamp < self._localization_metrics["trusted_pose_stamp_sec"]:
                raise ValueError("Odometry clock reset since trusted anchor")
            t, q = stamped.transform.translation, stamped.transform.rotation
            odom_now = (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            anchor = self._localization_metrics["trusted_odom_anchor"]
            if math.hypot(t.x-anchor[0], t.y-anchor[1]) > float(spec.get("max_odom_delta_m", 1)):
                raise ValueError("Current odometry exceeds recovery travel bound")
            if abs(math.atan2(math.sin(odom_now[2]-anchor[2]), math.cos(odom_now[2]-anchor[2]))) > float(spec.get("max_odom_delta_yaw_rad", .5)):
                raise ValueError("Current odometry exceeds recovery rotation bound")
            x, y, yaw = propagate_base_pose(self._localization_metrics["last_trusted_pose"], anchor, odom_now)
        else:
            # Compatibility for existing non-timestamped profiles only.
            raise ValueError("Legacy LiDAR recovery seed is unsupported; enable timestamped base evidence")
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError("Invalid LiDAR-to-base transform")
        if self._pose_pub.get_subscription_count() == 0:
            raise RuntimeError("No /initialpose subscriber; manual recovery required")
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = spec.get("map_frame", "map")
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06854
        self._pose_pub.publish(msg)
        return f"Trusted seed published: base x={x:.3f} y={y:.3f} yaw={yaw:.3f}; awaiting fresh GICP verification (no motion)"

    def _subst(self, value: str) -> str:
        return value.replace("${ws}", self.workspace).replace("${map_ply}", self.map_ply)

    def _log_path(self, name: str) -> Path:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        return self.log_dir / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{name}.log"

    def _restart(self, component_id: str, progress: Callable[[str], None]) -> str:
        # Serialize same-component starts across Recovery instances in this domain.
        self.log_dir.mkdir(parents=True, exist_ok=True)
        lock_path = self.log_dir / f"start-{os.getuid()}-{os.environ.get('ROS_DOMAIN_ID', '0')}-{component_id}.lock"
        with lock_path.open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            return self._restart_locked(component_id, progress)

    def _restart_locked(self, component_id: str, progress: Callable[[str], None]) -> str:
        comp = self.env.components[component_id]
        spec = comp.restart or {}
        pattern = str(spec.get("kill_pattern") or comp.process_pattern or "")
        command = [self._subst(str(part)) for part in spec.get("command", [])]
        if not command:
            raise RuntimeError(f"{component_id}: empty restart command")
        in_terminal = self.env.kind == "sim" and getattr(self, "config", {}).get("restart_in_terminal", False)
        if in_terminal and (not shutil.which("gnome-terminal") or not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))):
            raise RuntimeError("Recovery requires a desktop terminal, but no graphical session/gnome-terminal is available")
        if spec.get("start_if_absent"):
            nodes = {f"{ns.rstrip('/')}/{name}" for name, ns in self.node.get_node_names_and_namespaces()}
            # Group start is allowed only when ALL owned members are absent.
            # A partial group needs investigation, never a second full launch.
            conflicts = sorted(nodes.intersection(spec.get("exclusive_nodes", [])))
            processes = [p for p in spec.get("exclusive_process_patterns", []) if same_domain_pids(p)]
            if conflicts or processes:
                raise RuntimeError(f"{component_id}: group members already present; no start performed. "
                                   f"nodes={conflicts}, processes={processes}. Inspect partial group first.")
            pids = same_domain_pids(pattern) if pattern else []
            if comp.ros_node in nodes or pids:
                return f"{component_id} already present; no process killed or duplicate started; awaiting health verification"
            # Absence-only bootstrap must never fall through to the kill path.
            pattern = ""
        if pattern:
            pids = same_domain_pids(pattern)
            progress(f"stopping /{pattern}/ pids={pids}")
            _signal_pids(pids, signal.SIGINT)
            for _ in range(50):
                if not same_domain_pids(pattern):
                    break
                time.sleep(0.1)
            else:
                _signal_pids(same_domain_pids(pattern), signal.SIGKILL)
                time.sleep(0.5)
        log_path = self._log_path(f"restart_{component_id}")
        progress(f"starting {shlex.join(command)}")
        with log_path.open("w") as log:
            log.write(f"$ {shlex.join(command)}\n")
            log.flush()
            # New session: the restarted node outlives a recovery_agent restart.
            launch_command = command
            if in_terminal:
                launch_command = ["gnome-terminal", "--wait", f"--title=Recovery: {component_id}", "--",
                                  "/usr/bin/python3", str(Path(self.workspace) / "tools/recovery_terminal_runner.py"),
                                  "--workspace", self.workspace, "--domain", os.environ.get("ROS_DOMAIN_ID", "0"),
                                  "--log", str(log_path), "--", *command]
            proc = subprocess.Popen(launch_command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        if spec.get("oneshot"):
            # e.g. real tools/restart_drive_motor.sh: does its own guarded restart and exits.
            try:
                code = proc.wait(timeout=float(spec.get("timeout_sec", 90.0)))
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGINT)
                raise RuntimeError(f"{component_id} restart script timed out (log {log_path})")
            if code != 0:
                tail = log_path.read_text(errors="replace")[-400:]
                raise RuntimeError(f"{component_id} restart script exited {code} (log {log_path}): {tail}")
            return f"{component_id} restart script succeeded (log {log_path})"
        settle = float(spec.get("settle_sec", 3.0))
        time.sleep(settle)
        if proc.poll() is not None:
            tail = log_path.read_text(errors="replace")[-400:]
            raise RuntimeError(f"{component_id} exited with {proc.returncode} (log {log_path}): {tail}")
        return f"{component_id} start dispatched ({'terminal' if in_terminal else 'process'} pid {proc.pid}, log {log_path}); awaiting unique nodes and fresh outputs"

    def _client(self, srv_type: Any, name: str) -> Any:
        key = (srv_type.__name__, name)
        if key not in self._clients:
            self._clients[key] = self.node.create_client(srv_type, name)
        return self._clients[key]

    def _call(self, srv_type: Any, name: str, request: Any, timeout_sec: float) -> Any:
        client = self._client(srv_type, name)
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"service {name} not available")
        return _wait_future(client.call_async(request), timeout_sec, name)

    def _lifecycle_reset(self, manager: str, progress: Callable[[str], None]) -> str:
        from nav2_msgs.srv import ManageLifecycleNodes

        service = f"{manager.rstrip('/')}/manage_nodes"
        progress("RESET")
        reset = self._call(ManageLifecycleNodes, service, ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.RESET), 30.0)
        if not reset.success:
            raise RuntimeError(f"{service} RESET returned success=false; STARTUP not dispatched")
        progress("STARTUP")
        response = self._call(
            ManageLifecycleNodes, service, ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP), 60.0
        )
        if not response.success:
            raise RuntimeError(f"{service} STARTUP returned success=false")
        return f"{manager} reset + startup ok"

    def _clear_costmap(self, service: str, progress: Callable[[str], None]) -> str:
        from nav2_msgs.srv import ClearEntireCostmap

        progress(service)
        self._call(ClearEntireCostmap, service, ClearEntireCostmap.Request(), 10.0)
        return f"{service} cleared"

    def _cancel_navigation(self, service: str, progress: Callable[[str], None]) -> str:
        from action_msgs.srv import CancelGoal

        client = self._client(CancelGoal, service)
        if not client.wait_for_service(timeout_sec=2.0):
            return f"{service} not available: no navigation to cancel"
        # Zero goal id + zero stamp = cancel all goals.
        response = _wait_future(client.call_async(CancelGoal.Request()), 10.0, service)
        return f"cancel requested ({len(response.goals_canceling)} goal(s))"

    def _auto_initial_align(self, progress: Callable[[str], None]) -> str:
        spec = self.config.get("auto_initial_align") or {}
        command = [
            "python3",
            self._subst(str(spec["script"])),
            "--map",
            self.map_ply,
            "--topic",
            str(spec.get("topic", "/livox/lidar")),
            "--publish-initialpose",
            *[self._subst(str(a)) for a in spec.get("args", [])],
        ]
        log_path = self._log_path("auto_initial_align")
        progress("aligning live scan to map")
        with log_path.open("w") as log:
            log.write(f"$ {shlex.join(command)}\n")
            log.flush()
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = proc.wait(timeout=float(spec.get("timeout_sec", 60.0)))
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGINT)
                raise RuntimeError(f"auto_initial_align timed out (log {log_path})")
        if code != 0:
            raise RuntimeError(f"auto_initial_align exited {code}: {log_path.read_text(errors='replace')[-300:]}")
        return f"initial pose published (log {log_path})"
