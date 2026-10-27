"""Session-scoped, cooperative Agent shutdown; never discovers/kills robot nodes."""
from __future__ import annotations

import json
import os
import signal
from pathlib import Path

CONTROL_TOPIC = "/agent/stack_control"
ACK_TOPIC = "/agent/stack_ack"
STATE_TOPIC = "/agent/stack_state"


def process_identity(pid, proc=Path("/proc")):
    entry = proc / str(pid)
    stat = (entry / "stat").read_text().rsplit(")", 1)[1].split()
    return {"pid": int(pid), "ppid": int(stat[1]), "start": stat[19],
            "uid": entry.stat().st_uid,
            "args": (entry / "cmdline").read_bytes().decode().split("\0")}


def launch_parent(environ=None, proc=Path("/proc"), parent_pid=None):
    env = os.environ if environ is None else environ
    pid = int(env.get("ROBOT_AGENT_LAUNCH_PID", "0"))
    if pid <= 1 or pid != (os.getppid() if parent_pid is None else parent_pid):
        raise RuntimeError("Unmanaged Agent launch. Restart with the updated Agent launcher first.")
    identity = process_identity(pid, proc)
    if identity["uid"] != os.getuid() or identity["start"] != env.get("ROBOT_AGENT_LAUNCH_START"):
        raise RuntimeError("Agent launch identity changed; refusing to signal a different process.")
    args = identity["args"]
    if not any(args[n:n + 3] == ["launch", "robot_agent", "agent_dashboard.launch.py"]
               for n in range(len(args) - 2)):
        raise RuntimeError("Parent is not the Agent Dashboard launch.")
    return pid


def stop_launch():
    # A pidfd prevents PID reuse between identity checking and signalling.
    fd = os.pidfd_open(launch_parent())
    try:
        launch_parent()
        signal.pidfd_send_signal(fd, signal.SIGINT)
    finally:
        os.close(fd)


class ShutdownParticipant:
    """Wait on a wall/steady timer even when Isaac's /clock has stopped."""

    def __init__(self, node, name, prepare, ready):
        from rclpy.clock import Clock, ClockType
        from std_msgs.msg import String
        self.String = String
        self.session = os.environ.get("ROBOT_AGENT_SESSION_ID", "")
        self.name, self.prepare, self.ready = name, prepare, ready
        self.request = None
        self.stopping = False
        self._prepare_error = None
        self.publisher = node.create_publisher(String, ACK_TOPIC, 10)
        self.subscription = node.create_subscription(String, CONTROL_TOPIC, self._receive, 10)
        self.timer = node.create_timer(.2, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _receive(self, msg):
        try:
            request = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if (not isinstance(request, dict) or not self.session or request.get("session_id") != self.session
                or request.get("action") != "prepare_stop" or not request.get("request_id")):
            return
        self.request = request
        if not self.stopping:
            self.stopping = True
            try:
                self.prepare()
            except Exception as exc:
                self._prepare_error = str(exc)

    def _tick(self):
        if self.request is None:
            return
        try:
            if self._prepare_error:
                raise RuntimeError(self._prepare_error)
            ready, detail = self.ready()
        except Exception as exc:
            ready, detail = False, f"Shutdown check failed: {exc}"
        self.publisher.publish(self.String(data=json.dumps({
            **self.request, "node": self.name, "ready": ready, "detail": detail})))
