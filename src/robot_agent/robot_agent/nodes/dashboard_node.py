from __future__ import annotations

import asyncio
import json
import mimetypes
import threading
import time
from pathlib import Path

import rclpy
from aiohttp import web, WSMsgType
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String

from robot_skills.semantic_map import SemanticMap


def _find_web_dist() -> Path:
    try:
        share_dir = Path(get_package_share_directory("robot_agent")) / "web"
        if (share_dir / "index.html").is_file():
            return share_dir
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
        node.create_timer(JOY_PUBLISH_PERIOD_SEC, self._publish_tick)
        node.create_timer(0.1, self._watchdog_tick)

    def handle_control(self, owner_id: int, action: str) -> None:
        command = action.strip().upper()
        with self._lock:
            if command == "ACQUIRE_DASHBOARD":
                if self._owner_id not in (None, owner_id):
                    return
                self._owner_id = owner_id
                self._enabled = True
                self._reset_locked(clear_owner=False)
            elif command == "RELEASE_DASHBOARD":
                if self._owner_id != owner_id:
                    return
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
        # Real hardware only (absent in sim): publishes to real_robot_monitor's
        # /agent/real_robot_command, e.g. for the drive-motor restart control.
        self.real_robot_command_pub = None
        self.arm_command_pub = None
        self.joy_manager: JoyManager | None = None
        self.foxglove_bridge_port = 8766
        self.foxglove_ui_port = 8080
        self.foxglove_ui_enabled = False
        self.quick_commands, self.default_command = self._build_quick_commands()

    def publish_update(self, topic: str, data: str):
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
        return web.Response(body=index_path.read_bytes(), content_type="text/html")

    async def config(_request: web.Request) -> web.Response:
        return web.json_response(
            {
                "quick_commands": state.quick_commands,
                "default_command": state.default_command,
                "foxglove_bridge_port": state.foxglove_bridge_port,
                "foxglove_ui_port": state.foxglove_ui_port,
                "foxglove_ui_enabled": state.foxglove_ui_enabled,
            }
        )

    async def artifact(request: web.Request) -> web.Response:
        requested = request.query.get("path", "")
        result = _serve_artifact(requested)
        if result is None:
            raise web.HTTPNotFound()
        data, content_type = result
        return web.Response(body=data, content_type=content_type)

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
                msg_type = payload.get("type")
                if msg_type == "command":
                    text = str(payload.get("text", ""))
                    if state.command_pub is not None:
                        state.command_pub.publish(String(data=text))
                elif msg_type == "cancel":
                    if state.cancel_pub is not None:
                        state.cancel_pub.publish(String(data="cancel"))
                elif msg_type == "real_robot_command":
                    text = str(payload.get("text", ""))
                    if state.real_robot_command_pub is not None:
                        state.real_robot_command_pub.publish(String(data=text))
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
    app.router.add_get("/artifact", artifact)
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
        self.state.real_robot_command_pub = self.create_publisher(String, "/agent/real_robot_command", 10)
        self.state.arm_command_pub = self.create_publisher(String, "/arm/dashboard_command", 10)
        self.state.joy_manager = JoyManager(self)
        self.state.foxglove_bridge_port = int(self.get_parameter("foxglove_bridge_port").value)
        self.state.foxglove_ui_port = int(self.get_parameter("foxglove_ui_port").value)
        self.state.foxglove_ui_enabled = bool(self.get_parameter("foxglove_ui_enabled").value)

        for topic in SUBSCRIBED_TOPICS:
            self.create_subscription(String, topic, lambda msg, t=topic: self.state.publish_update(t, msg.data), 10)
        for topic in SUBSCRIBED_FLOAT_TOPICS:
            self.create_subscription(
                Float32, topic, lambda msg, t=topic: self.state.publish_update(t, str(msg.data)), 10
            )

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
