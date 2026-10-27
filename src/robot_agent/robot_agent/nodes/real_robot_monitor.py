from __future__ import annotations

import importlib.util
import json
import math
import os
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from geometry_msgs.msg import Twist
from my_messages.msg import DriveMotor, SteerMotor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from robot_agent.stack_lifecycle import ShutdownParticipant


def drive_restart_guard_rejection(
    *,
    robot_mode: str,
    cmd_age: float,
    cmd_vel: tuple[float, float, float],
    drive_age: float,
    drive_values: object,
    feedback_stale_sec: float,
    wheel_speed_limit: float,
    joy_age: float,
    joy_source: object,
) -> str | None:
    """Return the first failed direct precondition for a Dashboard restart."""
    if robot_mode != "MANUAL":
        return "Drive restart requires MANUAL mode. Press B first."
    if cmd_age > 1.0 or max(abs(value) for value in cmd_vel) > 1e-3:
        return "Drive restart requires fresh zero /cmd_vel and a stationary robot."
    if (
        drive_age > feedback_stale_sec
        or not isinstance(drive_values, list)
        or len(drive_values) != 3
        or not all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in drive_values
        )
    ):
        return "Drive restart requires fresh finite /drive_odom feedback from all three wheels."
    measured_speed = max(abs(float(value)) for value in drive_values)
    if measured_speed > wheel_speed_limit:
        return (
            f"Drive restart requires measured wheel speed <= {wheel_speed_limit:.3f} m/s; "
            f"observed {measured_speed:.3f} m/s."
        )
    physical_active_age = (
        joy_source.get("physical_active_age_sec")
        if isinstance(joy_source, dict)
        else None
    )
    if (
        joy_age > 1.0
        or not isinstance(joy_source, dict)
        or joy_source.get("source") != "PHYSICAL"
        or joy_source.get("dashboard_acquired") is not False
        or (
            isinstance(physical_active_age, (int, float))
            and physical_active_age < 0.5
        )
    ):
        return (
            "Release Dashboard control and leave physical Joy neutral for at least "
            "0.5 seconds before drive restart."
        )
    return None


class RealRobotMonitor(Node):
    """Publish extensible real-hardware status and handle guarded recovery."""

    def __init__(self):
        super().__init__("real_robot_monitor")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("drive_stale_sec", 1.5)
        self.declare_parameter("steer_stale_sec", 1.0)
        self.declare_parameter("camera_stale_sec", 1.5)
        self.declare_parameter("lift_stale_sec", 2.0)
        self.declare_parameter("restart_feedback_stale_sec", 1.0)
        self.declare_parameter("restart_wheel_speed_limit_m_s", 0.01)
        self.declare_parameter("restart_timeout_sec", 90.0)
        self.declare_parameter("restart_script", "tools/restart_drive_motor.sh")

        self.workspace = Path(os.environ.get("ROBOT_WS", str(Path.cwd()))).expanduser().resolve()
        self.publisher = self.create_publisher(String, "/agent/real_robot_state", 10)
        self.latest_time: dict[str, float] = {}
        self.latest_values: dict[str, object] = {}
        self.robot_mode = "UNKNOWN"
        self.cmd_vel = (0.0, 0.0, 0.0)
        self.restart_lock = threading.Lock()
        self._shutdown = ShutdownParticipant(self, "real_robot_monitor", lambda: None,
            lambda: (not self.restart_lock.locked(), "Waiting for drive restart worker" if self.restart_lock.locked()
                     else "No drive restart worker running"))
        self.drive_virtual_mode = False
        self.restart_state = {
            "state": "IDLE",
            "message": "No drive motor restart requested.",
            "started_at": None,
            "finished_at": None,
            "log_tail": "",
        }
        self.model_state = self._inspect_models()

        self.create_subscription(DriveMotor, "/drive_odom", self._drive_cb, 10)
        self.create_subscription(SteerMotor, "/steer_odom", self._steer_cb, 10)
        self.create_subscription(String, "/drive/health_summary", self._drive_health_cb, 10)
        self.create_subscription(Bool, "/drive/virtual_mode", self._drive_virtual_mode_cb, 10)
        self.create_subscription(Float32, "/chokudomotor/angle", self._camera_pan_cb, 10)
        self.create_subscription(Float32, "/cameraswingmotor/angle", self._camera_tilt_cb, 10)
        self.create_subscription(Float32, "/lift/position", self._lift_cb, 10)
        self.create_subscription(String, "/robot_mode", self._mode_cb, 10)
        self.create_subscription(String, "/joy/source_status", self._joy_source_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(String, "/agent/real_robot_command", self._command_cb, 10)

        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)

    def _mark(self, key: str, value: object) -> None:
        self.latest_time[key] = time.monotonic()
        self.latest_values[key] = value

    def _drive_cb(self, msg: DriveMotor) -> None:
        self._mark("drive", [float(msg.vel1), float(msg.vel2), float(msg.vel3)])

    def _steer_cb(self, msg: SteerMotor) -> None:
        self._mark("steer", [float(msg.phi1), float(msg.phi2), float(msg.phi3)])

    def _drive_health_cb(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            value = {"parse_error": True, "raw": str(msg.data)}
        self._mark("drive_health", value)

    def _drive_virtual_mode_cb(self, msg: Bool) -> None:
        self.drive_virtual_mode = bool(msg.data)
        self._mark("drive_virtual_mode", self.drive_virtual_mode)

    def _camera_pan_cb(self, msg: Float32) -> None:
        self._mark("camera_pan", float(msg.data))

    def _camera_tilt_cb(self, msg: Float32) -> None:
        self._mark("camera_tilt", float(msg.data))

    def _lift_cb(self, msg: Float32) -> None:
        self._mark("lift", float(msg.data))

    def _mode_cb(self, msg: String) -> None:
        self.robot_mode = str(msg.data).strip().upper()
        self._mark("robot_mode", self.robot_mode)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self.cmd_vel = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        self._mark("cmd_vel", list(self.cmd_vel))

    def _joy_source_cb(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            value = {"parse_error": True, "raw": str(msg.data)}
        self._mark("joy_source", value)

    def _component(self, keys: list[str], stale_sec: float, details: dict | None = None) -> dict:
        ages = [self._age(key) for key in keys]
        age = max(ages) if ages else math.inf
        if any(math.isinf(item) for item in ages):
            level, summary = "UNKNOWN", "waiting for feedback"
        elif age > stale_sec:
            level, summary = "ERROR", f"feedback stale for {age:.2f}s"
        else:
            # Freshness proves that this monitor received a message. It does
            # not by itself prove device health or physical correctness.
            level, summary = "OBSERVED", f"feedback received {age:.2f}s ago"
        return {"level": level, "summary": summary, "age_sec": None if math.isinf(age) else age, **(details or {})}

    def _age(self, key: str) -> float:
        stamp = self.latest_time.get(key)
        return math.inf if stamp is None else max(0.0, time.monotonic() - stamp)

    def _publish(self) -> None:
        virtual_drive_active = (
            self.drive_virtual_mode and self._age("drive_virtual_mode") <= 2.0
        )
        drive = self._component(
            ["drive"] if virtual_drive_active else ["drive", "drive_health"],
            float(self.get_parameter("drive_stale_sec").value),
            {
                "wheel_velocity_m_s": self.latest_values.get("drive"),
                "driver_health": self.latest_values.get("drive_health"),
                "virtual_test_source": virtual_drive_active,
            },
        )
        if virtual_drive_active and drive["level"] == "OBSERVED":
            drive.update(
                level="SIMULATED",
                summary="Stationary virtual /drive_odom; Oriental Motor/Modbus is intentionally disabled",
                evidence="SIMULATED_TEST_SOURCE",
            )
        drive_health = self.latest_values.get("drive_health")
        if isinstance(drive_health, dict):
            state = drive_health.get("state")
            if isinstance(state, dict) and int(state.get("state_error", 0)) != 0:
                drive.update(level="ERROR", summary=f"Modbus state_error={state['state_error']}")
            response_age = drive_health.get("last_response_age_sec")
            if isinstance(response_age, (int, float)) and response_age > 1.0:
                drive.update(level="ERROR", summary=f"Modbus response stale for {response_age:.2f}s")
        steer = self._component(
            ["steer"],
            float(self.get_parameter("steer_stale_sec").value),
            {
                "angles_rad": self.latest_values.get("steer"),
                "note": "topic freshness; inspect driver log for read errors",
            },
        )
        camera = self._component(
            ["camera_pan", "camera_tilt"],
            float(self.get_parameter("camera_stale_sec").value),
            {"pan_deg": self.latest_values.get("camera_pan"), "tilt_deg": self.latest_values.get("camera_tilt")},
        )
        lift = self._component(
            ["lift"],
            float(self.get_parameter("lift_stale_sec").value),
            {"position_mm": self.latest_values.get("lift")},
        )
        if lift["level"] == "OBSERVED":
            lift.update(
                level="ESTIMATED",
                summary="fresh software-integrated estimate; no encoder/limit feedback",
                evidence="SOFTWARE_ESTIMATE",
            )
        components = {"drive_motor": drive, "steer_motor": steer, "camera_motor": camera, "lift": lift}
        levels = [item["level"] for item in components.values()]
        overall = (
            "ERROR"
            if "ERROR" in levels
            else ("UNKNOWN" if "UNKNOWN" in levels else "PARTIALLY_OBSERVED")
        )
        state = {
            "schema_version": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "overall": overall,
            "robot_mode": self.robot_mode,
            "drive_virtual_mode": virtual_drive_active,
            "cmd_vel": {
                "x": self.cmd_vel[0],
                "y": self.cmd_vel[1],
                "yaw": self.cmd_vel[2],
                "age_sec": self._finite_age("cmd_vel"),
            },
            "components": components,
            "perception": self.model_state,
            "drive_restart": dict(self.restart_state),
        }
        self.publisher.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def _finite_age(self, key: str) -> float | None:
        age = self._age(key)
        return None if math.isinf(age) else age

    def _command_cb(self, msg: String) -> None:
        if self._shutdown.stopping:
            return
        if str(msg.data).strip() != "restart_drive_motor":
            return
        if self.drive_virtual_mode and self._age("drive_virtual_mode") <= 2.0:
            self._set_restart(
                "REJECTED",
                "Oriental Motor is disabled in virtual-drive test mode; real drive restart is blocked.",
            )
            return
        drive_values = self.latest_values.get("drive")
        feedback_stale_sec = float(self.get_parameter("restart_feedback_stale_sec").value)
        wheel_speed_limit = float(self.get_parameter("restart_wheel_speed_limit_m_s").value)
        joy_source = self.latest_values.get("joy_source")
        rejection = drive_restart_guard_rejection(
            robot_mode=self.robot_mode,
            cmd_age=self._age("cmd_vel"),
            cmd_vel=self.cmd_vel,
            drive_age=self._age("drive"),
            drive_values=drive_values,
            feedback_stale_sec=feedback_stale_sec,
            wheel_speed_limit=wheel_speed_limit,
            joy_age=self._age("joy_source"),
            joy_source=joy_source,
        )
        if rejection is not None:
            self._set_restart("REJECTED", rejection)
            return
        if not self.restart_lock.acquire(blocking=False):
            self._set_restart("REJECTED", "A drive restart is already running.")
            return
        self.restart_state = {
            "state": "RUNNING",
            "message": "Restarting Modbus and drive_motor...",
            "started_at": datetime.now(timezone.utc).isoformat(),
            "finished_at": None,
            "log_tail": "",
        }
        threading.Thread(target=self._restart_worker, daemon=True).start()

    def _restart_worker(self) -> None:
        try:
            script = Path(str(self.get_parameter("restart_script").value)).expanduser()
            if not script.is_absolute():
                script = self.workspace / script
            result = subprocess.run(
                [str(script), "--confirmed-stationary"], cwd=str(self.workspace),
                env={**os.environ, "ROBOT_WS": str(self.workspace)}, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                timeout=float(self.get_parameter("restart_timeout_sec").value), check=False,
            )
            tail = "\n".join(result.stdout.splitlines()[-12:])
            if result.returncode == 0:
                self._set_restart("SUCCEEDED", "Drive motor feedback recovered.", tail)
            else:
                self._set_restart("FAILED", f"Restart program exited with code {result.returncode}.", tail)
        except Exception as exc:
            self._set_restart("FAILED", f"Restart program failed: {exc!r}")
        finally:
            self.restart_lock.release()

    def _set_restart(self, state: str, message: str, log_tail: str = "") -> None:
        self.restart_state = {
            "state": state,
            "message": message,
            "started_at": self.restart_state.get("started_at"),
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "log_tail": log_tail,
        }

    def _inspect_models(self) -> dict:
        paths = {
            "yolo11s": self.workspace / "models/perception/yolo11s.pt",
            "easyocr_detector": self.workspace / "models/perception/easyocr/craft_mlt_25k.pth",
            "easyocr_english": self.workspace / "models/perception/easyocr/english_g2.pth",
            "tesseract": self.workspace / "models/ocr/tesseract/usr/bin/tesseract",
            "tesseract_eng": self.workspace
            / "models/ocr/tesseract/usr/share/tesseract-ocr/4.00/tessdata/eng.traineddata",
        }
        files = {
            name: {
                "ready": path.is_file(),
                "path": str(path),
                "size_bytes": path.stat().st_size if path.is_file() else 0,
            }
            for name, path in paths.items()
        }
        module_names = ["ultralytics", "torch", "cv2", "pytesseract", "easyocr"]
        modules = {name: importlib.util.find_spec(name) is not None for name in module_names}
        installed = all(item["ready"] for item in files.values()) and all(modules.values())
        return {
            "level": "MODEL_INSTALLED" if installed else "MODEL_MISSING",
            "summary": (
                "model files and Python modules found; inference runtime not validated"
                if installed
                else "model file or Python dependency missing"
            ),
            "device": "cpu",
            "evidence": "STATIC_INSTALLATION_CHECK",
            "files": files,
            "modules": modules,
        }


def main(args=None):
    rclpy.init(args=args)
    node = RealRobotMonitor()
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
