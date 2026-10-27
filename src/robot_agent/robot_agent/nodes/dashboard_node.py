from __future__ import annotations

import asyncio
import html
import ipaddress
import json
import mimetypes
import os
import threading
import time
from pathlib import Path
from uuid import uuid4
from urllib.parse import urlsplit

import rclpy
from aiohttp import web, WSMsgType
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String

from robot_agent.human_report import REPORT_TOPIC, HumanReportManager
from robot_agent.mission_test_commands import MISSION_TEST_COMMANDS
from robot_skills.semantic_map import SemanticMap
from robot_agent.stack_lifecycle import ACK_TOPIC, CONTROL_TOPIC, STATE_TOPIC, launch_parent, stop_launch

STACK_STOP_WAIT_SEC = 60.0


def _find_web_dist() -> Path:
    try:
        share_dir = Path(get_package_share_directory("robot_agent")) / "web"
        if (share_dir / "index.html").is_file():
            # With symlink-install the HTML tracks a rebuilt dist, but the
            # installed hashed asset links still refer to the previous build.
            return (share_dir / "index.html").resolve().parent
    except PackageNotFoundError:
        pass
    # Fallback for running straight out of the source tree (before `colcon build`
    # has installed the built assets into share/), e.g. during frontend development.
    source_dist = Path(__file__).resolve().parents[2] / "web" / "dist"
    return source_dist


SUBSCRIBED_TOPICS = [
    "/agent/status",
    "/agent/mission_events",
    "/agent/mission_state",
    "/agent/mode_state",
    "/agent/health_state",
    "/agent/navigation_progress",
    "/agent/navigation_route_state",
    "/agent/human_prompt",
    "/robot_mode",
    "/safety_status",
    "/lift/state",
    # Published by the real_robot_monitor node (real hardware only; absent in sim) --
    # forwarded like any other String topic, no special handling needed here.
    "/agent/real_robot_state",
    # Seven-domain read-only evidence produced by health_monitor.
    "/agent/health_dashboard_state",
    # Recovery Agent incidents / diagnosis (read-only v1) and mission request replies.
    "/agent/recovery_state",
    "/agent/recovery_events",
    "/agent/recovery_response",
    # Selected manual-control source from joy_source_arbiter_node.
    "/joy/source_status",
    # Mock/real arm gateway owns validation and execution. The dashboard only
    # forwards requests and renders this read-only state snapshot.
    "/arm/dashboard_state",
]

# Float32 readback topics broadcast to clients as plain numeric strings (not JSON),
# same convention as the plain std_msgs/String topics above.
SUBSCRIBED_FLOAT_TOPICS = [
    "/chokudomotor/angle",
    "/cameraswingmotor/angle",
    "/lift/position",
]


# Real Joy-Con / gamepad axis+button indices, taken directly from the existing
# joystick-consumer nodes so the dashboard's virtual controller is interpreted
# identically to a physical one -- see:
#   src/robot_bringup/config/joy_teleop.yaml            (axes 0/1/3, teleop_twist_joy)
#   src/serial_transciever/.../camera_motor_joy_node.py (axes 6/7, button 4)
#   src/serial_transciever/.../lift_joy_node.py          (trigger axes 2/5)
#   src/nav_pkg/scripts/nav_mode_switch_node.py          (buttons 0/1)
#   src/safety_layer/safety_layer/cmd_vel_safety_node.py (button 3, ESTOP)
JOY_AXIS_COUNT = 8
JOY_BUTTON_COUNT = 8
JOY_STATE_WATCHDOG_SEC = 0.3
JOY_PUBLISH_PERIOD_SEC = 0.05  # 20Hz -- mirrors joy_node's continuous real-device stream


class JoyManager:
    """Publishes Dashboard input on its own source topic.

    joy_source_arbiter_node is the only publisher on canonical /joy.  Browser
    control requires an explicit acquire command and one websocket owns the
    lease at a time; unrelated Health Dashboard clients cannot release it.

    Synthetic Joy messages are interpreted by the existing joystick-consumer nodes
    (teleop_twist_joy_node, nav_mode_switch_node, camera_motor_joy_node,
    lift_joy_node, cmd_vel_safety_node's ESTOP button) instead of dashboard_node
    after the arbiter selects this source; dashboard_node does not reimplement
    teleoperation, mode, camera, lift, or E-stop button logic.

    The lift uses independent hold-to-run trigger axes (LT=up, RT=down), so the
    Y safety ESTOP toggle no longer overlaps with a lift command.

    While acquired, publishes at JOY_PUBLISH_PERIOD_SEC. A
    watchdog resets to neutral if the browser stops sending updates for
    JOY_STATE_WATCHDOG_SEC (frozen tab, lost connection) -- a UI-responsiveness
    safety net on top of, not instead of, cmd_vel_safety_node's own watchdog.
    """

    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.Lock()
        self._axes = [0.0] * JOY_AXIS_COUNT
        self._buttons = [0] * JOY_BUTTON_COUNT
        self._last_update = time.monotonic()
        self._has_input = False
        self._enabled = False
        self._owner_id: int | None = None

        self._joy_pub = node.create_publisher(Joy, "/joy/dashboard", 10)
        self._control_pub = node.create_publisher(String, "/joy/source_command", 10)
        self._timers = [
            node.create_timer(JOY_PUBLISH_PERIOD_SEC, self._publish_tick),
            node.create_timer(0.1, self._watchdog_tick),
        ]

    def handle_control(self, owner_id: int, action: str) -> None:
        command = action.strip().upper()
        with self._lock:
            if command == "ACQUIRE_DASHBOARD":
                # A stale browser tab or refreshed websocket must not be able to
                # permanently lock out Dashboard control.  Re-acquire is safe:
                # it first resets the virtual Joy state to neutral, then starts
                # publishing heartbeats for the new owner.
                self._owner_id = owner_id
                self._enabled = True
                self._reset_locked(clear_owner=False)
            elif command == "RELEASE_DASHBOARD":
                # Release is safety-critical; accept it from any connected
                # Dashboard client instead of requiring the original owner tab.
                self._publish_neutral_locked()
                self._enabled = False
                self._reset_locked(clear_owner=True)
            else:
                return
        self._control_pub.publish(String(data=command))

    def handle_joy_state(self, owner_id: int, axes: list, buttons: list):
        with self._lock:
            if not self._enabled or self._owner_id != owner_id:
                return
            self._axes = self._sanitize(axes, JOY_AXIS_COUNT, float)
            self._buttons = self._sanitize(buttons, JOY_BUTTON_COUNT, int)
            self._last_update = time.monotonic()
            self._has_input = any(self._axes) or any(self._buttons)

    def handle_disconnect(self, owner_id: int):
        with self._lock:
            if self._owner_id != owner_id:
                return
            self._publish_neutral_locked()
            self._enabled = False
            self._reset_locked(clear_owner=True)
        self._control_pub.publish(String(data="RELEASE_DASHBOARD"))

    @staticmethod
    def _sanitize(values, size, cast):
        out = [cast(0)] * size
        for i, v in enumerate(values[:size]):
            try:
                out[i] = cast(v)
            except (TypeError, ValueError):
                pass
        return out

    def _reset_locked(self, *, clear_owner: bool = False):
        self._axes = [0.0] * JOY_AXIS_COUNT
        self._buttons = [0] * JOY_BUTTON_COUNT
        self._has_input = False
        self._last_update = time.monotonic()
        if clear_owner:
            self._owner_id = None

    def _watchdog_tick(self):
        now = time.monotonic()
        with self._lock:
            if self._enabled and self._has_input and now - self._last_update > JOY_STATE_WATCHDOG_SEC:
                self._reset_locked(clear_owner=False)

    def _publish_tick(self):
        with self._lock:
            if not self._enabled:
                return
            axes = list(self._axes)
            buttons = list(self._buttons)
            self._publish_locked(axes, buttons)

    def _publish_neutral_locked(self) -> None:
        self._publish_locked([0.0] * JOY_AXIS_COUNT, [0] * JOY_BUTTON_COUNT)

    def _publish_locked(self, axes: list, buttons: list) -> None:
        msg = Joy()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self._joy_pub.publish(msg)


class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.clients: set[web.WebSocketResponse] = set()
        self.latest: dict[str, str] = {}
        self.command_pub = None
        self.cancel_pub = None
        self.mode_command_pub = None
        self.mission_control_pub = None
        # Real hardware only (absent in sim): publishes to real_robot_monitor's
        # /agent/real_robot_command, e.g. for the drive-motor restart control.
        self.real_robot_command_pub = None
        # Operator replies / acknowledge / retry / abort for the Recovery Agent.
        self.recovery_command_pub = None
        self.arm_command_pub = None
        self.joy_manager: JoyManager | None = None
        self.foxglove_bridge_port = 8766
        self.foxglove_ui_port = 8080
        self.foxglove_ui_enabled = False
        self.quick_commands, self.default_command = self._build_quick_commands()
        self.human_report = HumanReportManager(Path.cwd())
        self._report_lock = threading.RLock()
        self.stack_control_pub = None
        self.stack_state = {"state": "RUNNING", "detail": "Agent stack running"}
        self.stack_task = None
        self.stack_acks = {}
        self.stack_token = uuid4().hex
        self.stack_participants = ("agent_node", "recovery_agent") + (
            ("real_robot_monitor",) if os.environ.get("ROBOT_AGENT_STACK_MODE") == "real" else ())

    def stack_ack(self, msg):
        try:
            ack = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if (isinstance(ack, dict) and ack.get("session_id") == os.environ.get("ROBOT_AGENT_SESSION_ID")
                and ack.get("request_id") == self.stack_state.get("request_id")
                and ack.get("node") in self.stack_participants):
            with self.lock:
                self.stack_acks[ack["node"]] = (time.monotonic(), ack)

    def stack_update(self, status, detail):
        self.stack_state = {**self.stack_state, "state": status, "detail": detail}
        self._publish_update(STATE_TOPIC, json.dumps(self.stack_state))

    async def stop_stack(self):
        try:
            launch_parent()
            request = {"action": "prepare_stop", "session_id": os.environ["ROBOT_AGENT_SESSION_ID"],
                       "request_id": uuid4().hex}
            self.stack_state = {**self.stack_state, "request_id": request["request_id"]}
            with self.lock:
                self.stack_acks.clear()
            self.stack_update("STOPPING", "Cancelling Mission and draining Recovery actions")
            if self.joy_manager is not None:
                self.joy_manager.handle_control(0, "RELEASE_DASHBOARD")
            deadline = time.monotonic() + STACK_STOP_WAIT_SEC
            while time.monotonic() < deadline:
                self.stack_control_pub.publish(String(data=json.dumps(request)))
                with self.lock:
                    acks = dict(self.stack_acks)
                ready = all(name in acks and time.monotonic() - acks[name][0] < 2
                            and acks[name][1].get("ready") is True for name in self.stack_participants)
                if ready:
                    self.stack_update("INTENTIONALLY_OFFLINE", "Agent stack stopping. Isaac and external ROS nodes remain running.")
                    await asyncio.sleep(.5)
                    stop_launch()
                    return
                detail = "; ".join(acks[name][1].get("detail", "Waiting") if name in acks
                                   else f"Waiting for {name} acknowledgement" for name in self.stack_participants)
                self.stack_update("STOPPING", detail)
                await asyncio.sleep(.5)
            self.stack_update("BLOCKED", "No forced termination. " + detail + ". Resolve the issue and retry Stop Agent stack.")
        except Exception as exc:
            self.stack_update("BLOCKED", str(exc))

    def publish_update(self, topic: str, data: str):
        report_update = None
        if topic != REPORT_TOPIC:
            with self._report_lock:
                report_update = self.human_report.on_topic(topic, data)
        self._publish_update(topic, data)
        if report_update is not None:
            self._publish_update(REPORT_TOPIC, json.dumps(report_update, ensure_ascii=False))

    def publish_human_report_command(self, kind: str, payload):
        with self._report_lock:
            report_update = self.human_report.on_dashboard_command(kind, payload)
        if report_update is not None:
            self._publish_update(REPORT_TOPIC, json.dumps(report_update, ensure_ascii=False))

    def _publish_update(self, topic: str, data: str):
        payload = json.dumps({"topic": topic, "data": data}, ensure_ascii=False)
        with self.lock:
            self.latest[topic] = data
            clients = list(self.clients)
            loop = self.loop
        if loop is None:
            return
        for ws in clients:
            asyncio.run_coroutine_threadsafe(self._safe_send(ws, payload), loop)

    @staticmethod
    async def _safe_send(ws: web.WebSocketResponse, payload: str):
        try:
            await ws.send_str(payload)
        except ConnectionResetError:
            pass

    @staticmethod
    def _build_quick_commands() -> tuple[list[tuple[str, str]], str]:
        commands: list[tuple[str, str]] = []
        try:
            semantic_map = SemanticMap()
            preferred_ids = ["sofa", "meeting_table", "desk_rxy", "trash_bin", "tool_box"]
            items = semantic_map.objects + semantic_map.places
            preferred_rank = {item_id: index for index, item_id in enumerate(preferred_ids)}
            sorted_items = sorted(
                enumerate(items),
                key=lambda pair: (preferred_rank.get(str(pair[1].get("id", "")), len(preferred_rank)), pair[0]),
            )
            for _, item in sorted_items:
                if item.get("navigation_ready") is False:
                    continue
                names = [str(name) for name in item.get("names", []) if name]
                target = names[0] if names else str(item.get("id", "")).strip()
                if not target:
                    continue
                label = target.replace("_", " ")
                commands.append((f"Go {label}", f"go to {target}"))
                if len(commands) >= 6:
                    break
        except Exception:
            commands = []

        commands.extend(
            [
                ("左を見る", "左を見て"),
                ("少し前進", "前進少し"),
                ("Turn around", "turn around"),
                ("ロボット状態", "ロボット状態"),
                ("既知の場所", "知っている場所を教えて"),
                ("キャンセル", "キャンセル"),
            ]
        )
        default_command = commands[0][1] if commands else "ロボット状態"
        return commands, default_command


def _serve_artifact(requested: str) -> tuple[bytes, str] | None:
    if not requested:
        return None
    try:
        path = Path(requested).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        path = path.resolve()
        workspace = Path.cwd().resolve()
        debug_root = (workspace / "debug-output").resolve()
        if not path.is_relative_to(debug_root):
            return None
        if not path.is_file():
            return None
        content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        return path.read_bytes(), content_type
    except Exception:
        return None


def _build_app(state: DashboardState) -> web.Application:
    dist_dir = _find_web_dist()
    app = web.Application()

    async def index(_request: web.Request) -> web.Response:
        index_path = dist_dir / "index.html"
        return web.Response(body=index_path.read_bytes(), content_type="text/html", headers={"Cache-Control": "no-cache"})

    async def config(_request: web.Request) -> web.Response:
        return web.json_response(
            {
                "quick_commands": state.quick_commands,
                "mission_test_commands": MISSION_TEST_COMMANDS,
                "default_command": state.default_command,
                "foxglove_bridge_port": state.foxglove_bridge_port,
                "foxglove_ui_port": state.foxglove_ui_port,
                "foxglove_ui_enabled": state.foxglove_ui_enabled,
                "stack_stop_token": state.stack_token,
                "stack_mode": os.environ.get("ROBOT_AGENT_STACK_MODE", ""),
                "stack_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            }
        )

    async def artifact(request: web.Request) -> web.Response:
        requested = request.query.get("path", "")
        result = _serve_artifact(requested)
        if result is None:
            raise web.HTTPNotFound()
        data, content_type = result
        return web.Response(body=data, content_type=content_type)

    async def report(request: web.Request) -> web.Response:
        requested = request.query.get("path", "")
        result = _serve_artifact(requested)
        if result is None:
            raise web.HTTPNotFound()
        data, _content_type = result
        try:
            parsed = json.loads(data.decode("utf-8"))
            rendered = json.dumps(parsed, ensure_ascii=False, indent=2)
            title = parsed.get("report_id") or "Human report"
            summary = parsed.get("summary_line") or ""
        except (UnicodeDecodeError, json.JSONDecodeError, AttributeError):
            rendered = data.decode("utf-8", errors="replace")
            title = "Human report"
            summary = ""
        body = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>{html.escape(str(title))}</title>
  <style>
    body {{ margin: 0; padding: 20px; background: #f6f7f9; color: #17202a; font: 14px/1.45 system-ui, sans-serif; }}
    h1 {{ margin: 0 0 6px; font-size: 18px; }}
    .summary {{ margin: 0 0 16px; color: #657386; }}
    pre {{ white-space: pre-wrap; overflow-wrap: anywhere; background: #fff; border: 1px solid #d8dde6; border-radius: 8px; padding: 14px; }}
  </style>
</head>
<body>
  <h1>{html.escape(str(title))}</h1>
  <div class="summary">{html.escape(str(summary))}</div>
  <pre>{html.escape(rendered)}</pre>
</body>
</html>"""
        return web.Response(text=body, content_type="text/html")

    async def stack_status(_request):
        return web.json_response(state.stack_state, headers={"Cache-Control": "no-store"})

    async def stack_stop(request):
        origin = urlsplit(request.headers.get("Origin", ""))
        try:
            local = ipaddress.ip_address(request.remote).is_loopback
        except ValueError:
            local = False
        if (not local or origin.hostname not in {"localhost", "127.0.0.1", "::1"}
                or origin.netloc != request.host or origin.scheme != request.scheme
                or request.headers.get("X-Agent-Stop-Token") != state.stack_token):
            raise web.HTTPForbidden(text="Local same-origin confirmation required")
        try:
            payload = await request.json()
        except (ValueError, TypeError):
            raise web.HTTPBadRequest(text="Expected JSON confirmation")
        if not isinstance(payload, dict) or payload.get("confirm") is not True:
            raise web.HTTPBadRequest(text="Explicit confirmation required")
        if state.stack_state["state"] == "INTENTIONALLY_OFFLINE":
            return web.json_response(state.stack_state, status=202)
        if state.stack_task is None or state.stack_task.done():
            try:
                launch_parent()
            except (RuntimeError, OSError, ValueError) as exc:
                return web.json_response({"state": "BLOCKED", "detail": str(exc)}, status=409)
            state.stack_update("STOPPING", "Preparing Agent stack shutdown")
            state.stack_task = asyncio.create_task(state.stop_stack())
        return web.json_response(state.stack_state, status=202)

    async def ws_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        client_id = id(ws)
        with state.lock:
            state.clients.add(ws)
            initial = list(state.latest.items())
        for topic, data in initial:
            await ws.send_str(json.dumps({"topic": topic, "data": data}, ensure_ascii=False))
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    payload = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                if not isinstance(payload, dict):
                    continue
                msg_type = payload.get("type")
                if state.stack_state["state"] != "RUNNING" and msg_type != "cancel":
                    continue
                if msg_type == "command":
                    text = str(payload.get("text", ""))
                    state.publish_human_report_command("command", text)
                    if state.command_pub is not None:
                        state.command_pub.publish(String(data=text))
                elif msg_type == "cancel":
                    state.publish_human_report_command("cancel", {"type": "cancel"})
                    if state.cancel_pub is not None:
                        state.cancel_pub.publish(String(data="cancel"))
                elif msg_type == "agent_mode":
                    mode = str(payload.get("mode", ""))
                    if state.mode_command_pub is not None:
                        state.mode_command_pub.publish(String(data=mode))
                elif msg_type == "mission_control":
                    command = payload.get("command")
                    if isinstance(command, dict) and state.mission_control_pub is not None:
                        state.publish_human_report_command("mission_control", command)
                        state.mission_control_pub.publish(String(data=json.dumps(command, ensure_ascii=False)))
                elif msg_type == "real_robot_command":
                    text = str(payload.get("text", ""))
                    if state.real_robot_command_pub is not None:
                        state.real_robot_command_pub.publish(String(data=text))
                elif msg_type == "recovery_command":
                    command = payload.get("command")
                    if isinstance(command, dict) and state.recovery_command_pub is not None:
                        state.publish_human_report_command("recovery_command", command)
                        state.recovery_command_pub.publish(String(data=json.dumps(command, ensure_ascii=False)))
                elif msg_type == "joy_state":
                    axes = payload.get("axes") or []
                    buttons = payload.get("buttons") or []
                    if state.joy_manager is not None:
                        state.joy_manager.handle_joy_state(client_id, axes, buttons)
                elif msg_type == "joy_control":
                    action = str(payload.get("action", ""))
                    if state.joy_manager is not None:
                        state.joy_manager.handle_control(client_id, action)
                elif msg_type == "arm_command":
                    command = payload.get("command")
                    if isinstance(command, dict) and state.arm_command_pub is not None:
                        state.arm_command_pub.publish(
                            String(data=json.dumps(command, ensure_ascii=False, allow_nan=False))
                        )
        finally:
            with state.lock:
                state.clients.discard(ws)
            if state.joy_manager is not None:
                state.joy_manager.handle_disconnect(client_id)
        return ws

    app.router.add_get("/", index)
    app.router.add_get("/health", index)
    app.router.add_get("/health/", index)
    app.router.add_get("/arm", index)
    app.router.add_get("/arm/", index)
    app.router.add_get("/api/config", config)
    app.router.add_get("/api/stack", stack_status)
    app.router.add_post("/api/stack/stop", stack_stop)
    app.router.add_get("/artifact", artifact)
    app.router.add_get("/report", report)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/assets/", dist_dir / "assets", follow_symlinks=True)
    return app


class DashboardNode(Node):
    def __init__(self):
        super().__init__("agent_dashboard_node")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("foxglove_bridge_port", 8766)
        self.declare_parameter("foxglove_ui_port", 8080)
        self.declare_parameter("foxglove_ui_enabled", False)
        self.state = DashboardState()
        self.state.command_pub = self.create_publisher(String, "/agent/text_command", 10)
        self.state.cancel_pub = self.create_publisher(String, "/agent/cancel", 10)
        self.state.mode_command_pub = self.create_publisher(String, "/agent/mode_command", 10)
        self.state.mission_control_pub = self.create_publisher(String, "/agent/mission_control", 10)
        self.state.real_robot_command_pub = self.create_publisher(String, "/agent/real_robot_command", 10)
        self.state.recovery_command_pub = self.create_publisher(String, "/agent/recovery_command", 10)
        self.state.arm_command_pub = self.create_publisher(String, "/arm/dashboard_command", 10)
        self.state.stack_control_pub = self.create_publisher(String, CONTROL_TOPIC, 10)
        self.create_subscription(String, ACK_TOPIC, self.state.stack_ack, 10)
        self.state.joy_manager = JoyManager(self)
        self.state.foxglove_bridge_port = int(self.get_parameter("foxglove_bridge_port").value)
        self.state.foxglove_ui_port = int(self.get_parameter("foxglove_ui_port").value)
        self.state.foxglove_ui_enabled = bool(self.get_parameter("foxglove_ui_enabled").value)

        self._topic_subscriptions = [
            self.create_subscription(
                String,
                topic,
                lambda msg, t=topic: self.state.publish_update(t, msg.data),
                10,
            )
            for topic in SUBSCRIBED_TOPICS
        ]
        self._float_topic_subscriptions = [
            self.create_subscription(
                Float32,
                topic,
                lambda msg, t=topic: self.state.publish_update(t, str(msg.data)),
                10,
            )
            for topic in SUBSCRIBED_FLOAT_TOPICS
        ]

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._runner: web.AppRunner | None = None
        self.thread = threading.Thread(target=self._run_server, args=(host, port), daemon=True)
        self.thread.start()

    def _run_server(self, host: str, port: int):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.state.loop = loop
        loop.run_until_complete(self._start_app(host, port))
        self.get_logger().info(f"Dashboard ready: http://{host}:{port}")
        loop.run_forever()

    async def _start_app(self, host: str, port: int):
        app = _build_app(self.state)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, host, port)
        await site.start()
        self._runner = runner

    def destroy_node(self):
        loop = self.state.loop
        if loop is not None and self._runner is not None:
            async def _cleanup():
                # AppRunner waits for open websocket handlers.  Close them first
                # so a browser left on /health or /arm cannot turn a normal ROS
                # shutdown into a five-second TimeoutError.
                with self.state.lock:
                    clients = list(self.state.clients)
                for ws in clients:
                    await ws.close(message=b"dashboard shutting down")
                await self._runner.cleanup()

            try:
                asyncio.run_coroutine_threadsafe(_cleanup(), loop).result(timeout=5)
            finally:
                loop.call_soon_threadsafe(loop.stop)
                self.thread.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
