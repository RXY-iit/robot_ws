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
from geometry_msgs.msg import Twist
from rclpy.node import Node
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
]

# Float32 readback topics broadcast to clients as plain numeric strings (not JSON),
# same convention as the plain std_msgs/String topics above.
SUBSCRIBED_FLOAT_TOPICS = [
    "/chokudomotor/angle",
    "/cameraswingmotor/angle",
    "/lift/position",
]


# Manual-control button scale/step -- mirrors the existing joystick nodes exactly so
# dashboard buttons feel the same as the Joy-Con:
#   chassis:  src/robot_bringup/config/joy_teleop.yaml (scale_linear/scale_angular)
#   camera:   serial_transciever/manipulator_control/camera_motor_joy_node.py
CHASSIS_MAX_VX = 0.3
CHASSIS_MAX_VY = 0.3
CHASSIS_MAX_YAW = 0.5
CAMERA_PAN_STEP_DEG = 2.5
CAMERA_TILT_STEP_DEG = 5.0
MANUAL_CONTROL_WATCHDOG_SEC = 0.3


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class ManualControlManager:
    """Owns the manual-control publishers and the heartbeat watchdog.

    The browser sends a `manual_control` message at a fixed interval (~100ms) only
    while a control button is held. If heartbeats for a given target (chassis/lift)
    stop arriving -- released button, lost connection, frozen tab -- this class
    force-publishes the stop state within MANUAL_CONTROL_WATCHDOG_SEC. This is a
    UI-responsiveness safety net on top of, not instead of, cmd_vel_safety_node's
    own 0.5s watchdog on /cmd_vel_raw.

    Camera pan/tilt is target-angle based (not velocity), so releasing the button
    simply stops incrementing the target -- no watchdog stop action needed there,
    exactly like camera_motor_joy_node's own behavior.
    """

    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.Lock()
        self._last_heartbeat: dict[str, float] = {}
        self._chassis_active = False
        self._lift_active = False
        self._target_pan: float | None = None
        self._target_tilt: float | None = None
        self._current_pan: float | None = None
        self._current_tilt: float | None = None

        self._teleop_cmd_vel_pub = node.create_publisher(Twist, "/teleop/cmd_vel", 10)
        self._nav_mode_pub = node.create_publisher(String, "/nav_mode/command", 10)
        self._pan_pub = node.create_publisher(Float32, "/chokudomotor/target_angle", 10)
        self._tilt_pub = node.create_publisher(Float32, "/cameraswingmotor/target_angle", 10)
        self._lift_pub = node.create_publisher(String, "/lift/command", 10)

        node.create_subscription(Float32, "/chokudomotor/angle", self._on_current_pan, 10)
        node.create_subscription(Float32, "/cameraswingmotor/angle", self._on_current_tilt, 10)

        node.create_timer(0.1, self._watchdog_tick)

    def _on_current_pan(self, msg: Float32):
        with self._lock:
            self._current_pan = float(msg.data)

    def _on_current_tilt(self, msg: Float32):
        with self._lock:
            self._current_tilt = float(msg.data)

    def handle_manual_control(self, target: str, value: dict):
        now = time.monotonic()
        with self._lock:
            self._last_heartbeat[target] = now
            if target == "chassis":
                self._publish_chassis_locked(value)
            elif target == "camera":
                self._publish_camera_locked(value)
            elif target == "lift":
                direction = str(value.get("direction", "")).upper()
                if direction in ("UP", "DOWN"):
                    self._lift_active = True
                    self._lift_pub.publish(String(data=direction))

    def handle_manual_control_stop(self, target: str):
        with self._lock:
            self._last_heartbeat.pop(target, None)
            self._stop_locked(target)

    def handle_disconnect(self):
        with self._lock:
            for target in ("chassis", "lift"):
                self._last_heartbeat.pop(target, None)
                self._stop_locked(target)

    def _publish_chassis_locked(self, value: dict):
        if not self._chassis_active:
            self._nav_mode_pub.publish(String(data="MANUAL"))
            self._chassis_active = True
        vx = _clamp(float(value.get("vx", 0.0)), CHASSIS_MAX_VX)
        vy = _clamp(float(value.get("vy", 0.0)), CHASSIS_MAX_VY)
        yaw = _clamp(float(value.get("yaw", 0.0)), CHASSIS_MAX_YAW)
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = yaw
        self._teleop_cmd_vel_pub.publish(twist)

    def _publish_camera_locked(self, value: dict):
        pan_dir = float(value.get("pan", 0.0))
        tilt_dir = float(value.get("tilt", 0.0))
        if pan_dir:
            if self._target_pan is None:
                self._target_pan = self._current_pan if self._current_pan is not None else 0.0
            self._target_pan += pan_dir * CAMERA_PAN_STEP_DEG
            self._pan_pub.publish(Float32(data=self._target_pan))
        if tilt_dir:
            if self._target_tilt is None:
                self._target_tilt = self._current_tilt if self._current_tilt is not None else 0.0
            self._target_tilt += tilt_dir * CAMERA_TILT_STEP_DEG
            self._tilt_pub.publish(Float32(data=self._target_tilt))

    def _stop_locked(self, target: str):
        if target == "chassis" and self._chassis_active:
            self._teleop_cmd_vel_pub.publish(Twist())
            self._chassis_active = False
        elif target == "lift" and self._lift_active:
            self._lift_pub.publish(String(data="STOP"))
            self._lift_active = False

    def _watchdog_tick(self):
        now = time.monotonic()
        with self._lock:
            stale = [
                target
                for target, last in self._last_heartbeat.items()
                if now - last > MANUAL_CONTROL_WATCHDOG_SEC
            ]
            for target in stale:
                del self._last_heartbeat[target]
                self._stop_locked(target)


class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.clients: set[web.WebSocketResponse] = set()
        self.latest: dict[str, str] = {}
        self.command_pub = None
        self.cancel_pub = None
        self.manual_control: ManualControlManager | None = None
        self.foxglove_bridge_port = 8766
        self.foxglove_ui_port = 8080
        self.foxglove_ui_enabled = True
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
                elif msg_type == "manual_control":
                    target = str(payload.get("target", ""))
                    value = payload.get("value") or {}
                    if state.manual_control is not None and target in ("chassis", "camera", "lift"):
                        state.manual_control.handle_manual_control(target, value)
                elif msg_type == "manual_control_stop":
                    target = str(payload.get("target", ""))
                    if state.manual_control is not None and target in ("chassis", "camera", "lift"):
                        state.manual_control.handle_manual_control_stop(target)
        finally:
            with state.lock:
                state.clients.discard(ws)
            if state.manual_control is not None:
                state.manual_control.handle_disconnect()
        return ws

    app.router.add_get("/", index)
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
        self.declare_parameter("foxglove_ui_enabled", True)
        self.state = DashboardState()
        self.state.command_pub = self.create_publisher(String, "/agent/text_command", 10)
        self.state.cancel_pub = self.create_publisher(String, "/agent/cancel", 10)
        self.state.manual_control = ManualControlManager(self)
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
                await self._runner.cleanup()

            asyncio.run_coroutine_threadsafe(_cleanup(), loop).result(timeout=5)
            loop.call_soon_threadsafe(loop.stop)
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
