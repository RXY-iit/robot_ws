#!/usr/bin/env python3
"""Desktop launcher for the Isaac Sim real-lab test stack."""

from __future__ import annotations

import json
import os
import runpy
import shlex
import shutil
import subprocess
import threading
from datetime import datetime
from pathlib import Path

from agent_stack_guard import find_existing
from agent_stack_stop import add_stop_control
from sim_startup_preflight import check_startup

import gi

GTK_MAJOR = 4
try:
    gi.require_version("Gtk", "3.0")
    gi.require_version("Gdk", "3.0")
    from gi.repository import Gdk, GLib, Gtk  # noqa: E402
    GTK_MAJOR = 3
except gi.RepositoryError:
    gi.require_version("Gtk", "4.0")
    gi.require_version("Gdk", "4.0")
    from gi.repository import Gdk, GLib, Gtk  # noqa: E402
    GTK_MAJOR = 4


WORKSPACE = Path("/home/ruan-x/robot_ws")
camera_environment = runpy.run_path(str(WORKSPACE / "src/robot_sim/robot_sim/camera_profile.py"))["camera_environment"]
ISAAC_DIR = Path("/home/ruan-x/isaac-sim")
ISAAC_BIN = ISAAC_DIR / "isaac-sim.sh"
SIM_NAV2_SCRIPT = WORKSPACE / "tools/sim/open_sim_nav2_windows.sh"
SIM_ARM_TEST_SCRIPT = WORKSPACE / "tools/sim/open_sim_arm_test_windows.sh"
DASHBOARD_SCRIPT = WORKSPACE / "tools/run_agent_dashboard_sim_clean.sh"
DEFAULT_SCENE = WORKSPACE / "src/robot_sim/usd/scenes/robot_sim_real_lab.usd"
DEFAULT_MAP_DIR = WORKSPACE / "maps/sim_real_lab_001"
DEFAULT_MAP = DEFAULT_MAP_DIR / "nav2.yaml"
DEFAULT_PLY = DEFAULT_MAP_DIR / "map_3d.ply"
DEFAULT_RVIZ = WORKSPACE / "rviz/nav2_navigation.rviz"
CONFIG_DIR = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "robot-sim-launcher"
CONFIG_FILE = CONFIG_DIR / "settings.json"
STATE_DIR = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state")) / "robot-sim-launcher"
LOG_FILE = STATE_DIR / "launcher.log"

SCENE_PROFILES = {
    "real_lab": {
        "label": "Real lab / sim_real_lab_001",
        "scene": str(WORKSPACE / "src/robot_sim/usd/scenes/robot_sim_real_lab.usd"),
        "map": str(WORKSPACE / "maps/sim_real_lab_001/nav2.yaml"),
        "ply": str(WORKSPACE / "maps/sim_real_lab_001/map_3d.ply"),
        "rviz": str(WORKSPACE / "rviz/nav2_navigation.rviz"),
        "robot_map_id": "sim_real_lab_001",
        "initial_x": "0.0",
        "initial_y": "0.0",
        "initial_yaw": "0.0",
        "scan_min_height": "-0.8",
        "scan_max_height": "-0.3",
    },
    "warehouse": {
        "label": "Full warehouse / sim_factory_mid360_like_gpu_trueimu_011",
        "scene": str(WORKSPACE / "src/robot_sim/usd/scenes/full_warehouse_arm_tasks.usda"),
        "map": str(WORKSPACE / "maps/sim_factory_mid360_like_gpu_trueimu_011/nav2.yaml"),
        "ply": str(WORKSPACE / "maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply"),
        "rviz": str(WORKSPACE / "rviz/sim_nav2_navigation.rviz"),
        "robot_map_id": "sim_factory_mid360_like_gpu_trueimu_011",
        "initial_x": "0.0",
        "initial_y": "0.0",
        "initial_yaw": "0.0",
        "scan_min_height": "-0.8",
        "scan_max_height": "-0.3",
    },
    "factory_mapping": {
        "label": "Factory mapping scene / sim_factory_mid360_like_gpu_trueimu_011",
        "scene": str(WORKSPACE / "src/robot_sim/usd/scenes/robot_sim_factory_mapping.usd"),
        "map": str(WORKSPACE / "maps/sim_factory_mid360_like_gpu_trueimu_011/nav2.yaml"),
        "ply": str(WORKSPACE / "maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply"),
        "rviz": str(WORKSPACE / "rviz/sim_gicp_nav2.rviz"),
        "robot_map_id": "sim_factory_mid360_like_gpu_trueimu_011",
        "initial_x": "0.0",
        "initial_y": "0.0",
        "initial_yaw": "0.0",
        "scan_min_height": "-0.8",
        "scan_max_height": "-0.3",
    },
}

PRE_PLAY_SCRIPT = """exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_real_lab_pre_play.py").read())"""

POST_PLAY_SCRIPT = """exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_real_lab_post_play.py").read())"""

ARM_TEST_PRE_PLAY_SCRIPT = """exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_arm_test_pre_play.py").read())"""

ARM_TEST_POST_PLAY_SCRIPT = """exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_arm_test_post_play.py").read())"""

POST_PLAY_EXPECTED = """Expected Isaac Console after Play:
  /clock is publishing from /World/ROS2_ActionGraph
  Created experimental MID360-like LiDAR
  Created /wheel_odom and /tf odom->base_footprint publishers.
  SO101 Isaac UDP joint bridge ready
  SO101 Arm Camera ROS2 publishers ready
  D455 ROS2 publishers ready
  SO101 Isaac visual synced from ROS

Do not expect rclpy inside Isaac.  Isaac Sim 5 uses Python 3.11, while ROS
Humble rclpy is Python 3.10; SO101 visual sync now uses udp://127.0.0.1:37601.
If an older callback is still active, rerun the post-Play wrapper once and expect
SO101 Isaac UDP joint bridge is stale; rebuilding.
If camera topics are missing, rerun the same post-Play wrapper while Isaac is
playing; stale D455/SO101 camera writers will be detached and reattached.
"""

ARM_TEST_POST_PLAY_EXPECTED = """Expected Isaac Console after Play:
  SO101 Isaac UDP joint bridge ready
  SO101 Arm Camera ROS2 publishers ready
  D455 ROS2 publishers ready
  Created /wheel_odom and /tf odom->base_footprint publishers.

Arm Test intentionally skips MID360/LiDAR/IMU/Nav2/localization publishers.
If /livox/lidar or RobotSimLivox render errors still appear, rerun the Arm Test
pre/post wrappers once after Reload/Reopen to detach stale full-mode publishers.
"""

ROS_CHECK_COMMANDS = """# After ROS/Dashboard start, quick checks:
ros2 node list --no-daemon | sort | rg 'gicp|isaac_joint|sim_lift|so101|arm_mount|touch|camera_pan|agent_dashboard|health_monitor'
ros2 topic info -v /clock --no-daemon
ros2 topic hz /clock --window 5
ros2 topic info -v /point_cloud --no-daemon
ros2 topic hz /point_cloud --window 5
ros2 topic hz /livox/lidar --window 5
ros2 topic hz /gicp_loc/pose --window 5
ros2 topic info -v /arm_camera/color/image_raw --no-daemon
ros2 topic hz /arm_camera/color/image_raw --window 5
ros2 topic info -v /camera/camera/aligned_depth_to_color/image_raw --no-daemon
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw --window 5
ros2 topic info -v /camera/camera/aligned_depth_to_color/camera_info --no-daemon
ros2 topic echo /arm/candidate_joint_states --once
ros2 topic echo /arm/visual_joint_states --once
ros2 topic echo /sim/arm_joint_states --once
ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true
ros2 lifecycle get /bt_navigator
"""

DEFAULTS = {
    "scene_profile": "real_lab",
    "start_isaac": True,
    "low_camera_resolution": False,
    "scene": str(DEFAULT_SCENE),
    "start_ros": True,
    "ros_mode": "nav2",
    "map": str(DEFAULT_MAP),
    "ply": str(DEFAULT_PLY),
    "rviz": str(DEFAULT_RVIZ),
    "initial_x": "0.0",
    "initial_y": "0.0",
    "initial_yaw": "0.0",
    "scan_min_height": "-0.8",
    "scan_max_height": "-0.3",
    "cleanup": False,
    "auto_mode": False,
    "motion_policy": False,
    "start_dashboard": True,
    "robot_map_id": "sim_real_lab_001",
    "start_llm": False,
    "llama_server": "",
    "gguf_model": "",
    "llm_port": "8090",
    "ros_domain_id": 20,
}

ROS_MODES = {
    "nav2": "完整导航：Nav2 + FAST-LIO/GICP + Arm/Touch",
    "arm_test": "Arm Test：底盘/Lift/Arm/Camera/Touch，不启动定位和 Nav2",
}


def load_settings() -> dict:
    settings = dict(DEFAULTS)
    try:
        data = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            settings.update({key: data[key] for key in DEFAULTS if key in data})
            if str(settings.get("llm_port")) == "8080":
                settings["llm_port"] = "8090"
    except (FileNotFoundError, OSError, TypeError, ValueError):
        pass
    # Safety migration: old launcher settings saved auto_mode=True, which used
    # to switch /robot_mode to AUTO during startup.  Keep startup MANUAL by
    # default; AUTO must come from an explicit Dashboard/Joy command.
    settings["auto_mode"] = False
    return settings


def save_settings(settings: dict) -> None:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    tmp = CONFIG_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(settings, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(CONFIG_FILE)


def append_log(message: str) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
    with LOG_FILE.open("a", encoding="utf-8") as stream:
        stream.write(f"[{timestamp}] {message}\n")


def discover_files(patterns: list[str], fallback: Path) -> list[tuple[str, str]]:
    paths: list[Path] = []
    for pattern in patterns:
        paths.extend(WORKSPACE.glob(pattern))
    unique = sorted({path.resolve() for path in paths if path.is_file()})
    if fallback.is_file() and fallback.resolve() not in unique:
        unique.insert(0, fallback.resolve())
    return [(str(path), str(path.relative_to(WORKSPACE)) if path.is_relative_to(WORKSPACE) else str(path)) for path in unique]


class LauncherWindow(Gtk.Window):
    def __init__(self) -> None:
        super().__init__(title="Real-Lab Isaac Sim Quick Start")
        self.set_border_width(16)
        self.set_default_size(860, 780)
        if GTK_MAJOR == 3:
            self.set_position(Gtk.WindowPosition.CENTER)
        self.settings = load_settings()
        self._restoring = False
        self._build_ui()
        self._restore_settings()
        self._update_sensitivity()
        self._update_preview()

    @staticmethod
    def _append(container: Gtk.Widget, child: Gtk.Widget) -> None:
        if GTK_MAJOR == 3:
            container.pack_start(child, False, False, 0)  # type: ignore[attr-defined]
        else:
            if isinstance(container, Gtk.Box):
                container.append(child)  # type: ignore[call-arg]
            else:
                container.set_child(child)  # type: ignore[call-arg]

    @staticmethod
    def _label(text: str) -> Gtk.Label:
        label = Gtk.Label(label=text, xalign=0)
        label.set_line_wrap(True)
        return label

    @staticmethod
    def _entry(text: str = "") -> Gtk.Entry:
        entry = Gtk.Entry()
        entry.set_text(text)
        return entry

    @staticmethod
    def _combo(entries: list[tuple[str, str]]) -> Gtk.ComboBoxText:
        combo = Gtk.ComboBoxText()
        for item_id, label in entries:
            combo.append(item_id, label)
        return combo

    def _section(self, title: str) -> tuple[Gtk.Frame, Gtk.Grid]:
        frame = Gtk.Frame(label=title)
        grid = Gtk.Grid(column_spacing=14, row_spacing=9, margin=12)
        if GTK_MAJOR == 3:
            frame.add(grid)
        else:
            frame.set_child(grid)
        return frame, grid

    def _add_row(self, grid: Gtk.Grid, row: int, title: str, widget: Gtk.Widget) -> None:
        grid.attach(self._label(title), 0, row, 1, 1)
        grid.attach(widget, 1, row, 1, 1)

    def _build_ui(self) -> None:
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        if GTK_MAJOR == 3:
            self.add(scroll)
            scroll.add(root)
        else:
            self.set_child(scroll)
            scroll.set_child(root)

        title = self._label("Real-lab 仿真快速启动：Isaac Sim、ROS Nav2/Arm/Touch、Dashboard、本地 LLM")
        self._append(root, title)
        self._append(
            root,
            self._label(
                "点击 Start 会保存当前选项，下次自动恢复。Script Editor 代码用复制按钮分步执行；"
                "SO101 同步使用 ROS 外部 UDP bridge，不在 Isaac 内 import rclpy。"
            ),
        )

        profile_frame, profile_grid = self._section("场景 / Map 配置")
        self._append(root, profile_frame)
        self.scene_profile = self._combo(
            [(profile_id, profile["label"]) for profile_id, profile in SCENE_PROFILES.items()]
        )
        self._add_row(profile_grid, 0, "选择配置", self.scene_profile)
        profile_grid.attach(
            self._label("选择后会自动填写下面 1/2/3 的 USD、Nav2 map、PLY、RViz、ROBOT_MAP_ID 和初始参数。"),
            0,
            1,
            2,
            1,
        )

        isaac_frame, isaac_grid = self._section("1. Isaac Sim")
        self._append(root, isaac_frame)
        self.start_isaac = Gtk.CheckButton(label="启动 Isaac Sim 并加载选中的 USD")
        isaac_grid.attach(self.start_isaac, 0, 0, 2, 1)
        self.scene = self._combo(discover_files([
            "src/robot_sim/usd/scenes/*.usd",
            "src/robot_sim/usd/scenes/*.usda",
            "src/robot_sim/usd/*.usd",
            "src/robot_sim/usd/*.usda",
        ], DEFAULT_SCENE))
        self._add_row(isaac_grid, 1, "USD 场景", self.scene)
        self.low_camera_resolution = Gtk.CheckButton(label="低分辨率：底盘 RGB 640×480 / Arm 320×240（默认 1280×720 / 640×480）")
        isaac_grid.attach(self.low_camera_resolution, 0, 5, 2, 1)
        full_pre_button = Gtk.Button(label="复制 Full Play 前代码")
        full_post_button = Gtk.Button(label="复制 Full Play 后代码")
        arm_pre_button = Gtk.Button(label="复制 Arm only Play 前代码")
        arm_post_button = Gtk.Button(label="复制 Arm only Play 后代码")
        isaac_grid.attach(full_pre_button, 0, 2, 1, 1)
        isaac_grid.attach(full_post_button, 1, 2, 1, 1)
        isaac_grid.attach(arm_pre_button, 0, 3, 1, 1)
        isaac_grid.attach(arm_post_button, 1, 3, 1, 1)
        full_pre_button.connect(
            "clicked",
            lambda _button: self._copy_script(
                self._camera_script(PRE_PLAY_SCRIPT, self._current_settings()),
                "已复制 Full Play 前脚本；请在 Stop 状态执行",
            ),
        )
        full_post_button.connect(
            "clicked",
            lambda _button: self._copy_script(
                self._camera_script(POST_PLAY_SCRIPT, self._current_settings()),
                "已复制 Full Play 后脚本；请在 Play 后执行",
            ),
        )
        arm_pre_button.connect(
            "clicked",
            lambda _button: self._copy_script(
                self._camera_script(ARM_TEST_PRE_PLAY_SCRIPT, self._current_settings()),
                "已复制 Arm only Play 前脚本；请在 Stop 状态执行",
            ),
        )
        arm_post_button.connect(
            "clicked",
            lambda _button: self._copy_script(
                self._camera_script(ARM_TEST_POST_PLAY_SCRIPT, self._current_settings()),
                "已复制 Arm only Play 后脚本；请在 Play 后执行",
            ),
        )
        isaac_grid.attach(
            self._label(
                "Full 模式：先 Stop 再粘贴 1)，按 Play 后粘贴 2)。Play 后 Console 应看到 UDP joint bridge / Arm Camera ready；若 stale bridge rebuilding，等它重新 ready。"
            ),
            0,
            4,
            2,
            1,
        )

        ros_frame, ros_grid = self._section("2. ROS sim / Nav2 / Arm / Touch / RViz")
        self._append(root, ros_frame)
        self.start_ros = Gtk.CheckButton(label="启动 ROS sim stack")
        ros_grid.attach(self.start_ros, 0, 0, 2, 1)
        self.ros_mode = self._combo(list(ROS_MODES.items()))
        self.map = self._combo(discover_files(["maps/*/nav2.yaml", "maps/*.yaml"], DEFAULT_MAP))
        self.ply = self._combo(discover_files(["maps/*/map_3d.ply", "maps/*.ply"], DEFAULT_PLY))
        self.rviz = self._combo(discover_files(["rviz/*.rviz", "src/*/rviz/*.rviz"], DEFAULT_RVIZ))
        self.initial_x = self._entry()
        self.initial_y = self._entry()
        self.initial_yaw = self._entry()
        self.scan_min_height = self._entry()
        self.scan_max_height = self._entry()
        self.cleanup = Gtk.CheckButton(label="Legacy process cleanup (disabled)")
        self.cleanup.set_sensitive(False)
        self.auto_mode = Gtk.CheckButton(label="--auto checks（不自动切 AUTO）")
        self.motion_policy = Gtk.CheckButton(label="--motion-policy")
        self._add_row(ros_grid, 1, "启动模式", self.ros_mode)
        self._add_row(ros_grid, 2, "Nav2 map yaml", self.map)
        self._add_row(ros_grid, 3, "FAST-LIO/GICP PLY", self.ply)
        self._add_row(ros_grid, 4, "RViz config", self.rviz)
        self._add_row(ros_grid, 5, "initial x", self.initial_x)
        self._add_row(ros_grid, 6, "initial y", self.initial_y)
        self._add_row(ros_grid, 7, "initial yaw rad", self.initial_yaw)
        self._add_row(ros_grid, 8, "scan min height", self.scan_min_height)
        self._add_row(ros_grid, 9, "scan max height", self.scan_max_height)
        ros_grid.attach(Gtk.Label(label="DDS: FastDDS | Startup policy: refuse duplicate processes"), 0, 10, 2, 1)
        ros_grid.attach(self.auto_mode, 1, 10, 1, 1)
        ros_grid.attach(self.motion_policy, 0, 11, 1, 1)

        dash_frame, dash_grid = self._section("3. Dashboard")
        self._append(root, dash_frame)
        self.start_dashboard = Gtk.CheckButton(label="启动 Agent Dashboard")
        dash_grid.attach(self.start_dashboard, 0, 0, 2, 1)
        self.robot_map_id = self._entry()
        self._add_row(dash_grid, 1, "ROBOT_MAP_ID", self.robot_map_id)
        self.stop_agent_button = add_stop_control(
            self, dash_grid, 2, "sim", lambda: self.domain_id.get_value_as_int())

        llm_frame, llm_grid = self._section("4. 本地 LLM Parser")
        self._append(root, llm_frame)
        self.start_llm = Gtk.CheckButton(label="启动 llama.cpp OpenAI-compatible server")
        llm_grid.attach(self.start_llm, 0, 0, 2, 1)
        self.llama_server = self._entry()
        self.gguf_model = self._entry()
        self.llm_port = self._entry()
        self._add_row(llm_grid, 1, "llama-server 路径", self.llama_server)
        self._add_row(llm_grid, 2, "GGUF 模型路径", self.gguf_model)
        self._add_row(llm_grid, 3, "端口", self.llm_port)

        common_frame, common_grid = self._section("5. Common")
        self._append(root, common_frame)
        self.domain_id = Gtk.SpinButton.new_with_range(0, 232, 1)
        self.domain_id.set_numeric(True)
        self._add_row(common_grid, 0, "ROS_DOMAIN_ID", self.domain_id)

        preview_frame = Gtk.Frame(label="将要执行的命令")
        preview_scroll = Gtk.ScrolledWindow()
        preview_scroll.set_min_content_height(120)
        self.preview = Gtk.TextView(editable=False, monospace=True, wrap_mode=Gtk.WrapMode.WORD_CHAR)
        if GTK_MAJOR == 3:
            self.preview.set_margin_start(8)
            self.preview.set_margin_end(8)
            self.preview.set_margin_top(8)
            self.preview.set_margin_bottom(8)
            preview_scroll.add(self.preview)
            preview_frame.add(preview_scroll)
        else:
            self.preview.set_margin_start(8)
            self.preview.set_margin_end(8)
            self.preview.set_margin_top(8)
            self.preview.set_margin_bottom(8)
            preview_scroll.set_child(self.preview)
            preview_frame.set_child(preview_scroll)
        if GTK_MAJOR == 3:
            root.pack_start(preview_frame, True, True, 0)
        else:
            root.append(preview_frame)

        self.status = self._label("Ready")
        self._append(root, self.status)

        if GTK_MAJOR == 3:
            buttons = Gtk.ButtonBox(orientation=Gtk.Orientation.HORIZONTAL)
            buttons.set_layout(Gtk.ButtonBoxStyle.END)
            buttons.set_spacing(10)
        else:
            buttons = Gtk.Box(spacing=10, orientation=Gtk.Orientation.HORIZONTAL)
        reset = Gtk.Button(label="恢复默认")
        close = Gtk.Button(label="关闭")
        self.start_button = Gtk.Button(label="Start")
        self.start_button.get_style_context().add_class(Gtk.STYLE_CLASS_SUGGESTED_ACTION)
        if GTK_MAJOR == 3:
            buttons.add(reset)
            buttons.add(close)
            buttons.add(self.start_button)
        else:
            buttons.append(reset)
            buttons.append(close)
            buttons.append(self.start_button)
        self._append(root, buttons)

        reset.connect("clicked", self._on_reset)
        close.connect("clicked", lambda _button: self.close())
        self.start_button.connect("clicked", self._on_start)
        self.scene_profile.connect("changed", self._on_profile_changed)
        for widget in (
            self.start_isaac,
            self.scene,
            self.start_ros,
            self.ros_mode,
            self.map,
            self.ply,
            self.rviz,
            self.initial_x,
            self.initial_y,
            self.initial_yaw,
            self.scan_min_height,
            self.scan_max_height,
            self.cleanup,
            self.auto_mode,
            self.motion_policy,
            self.start_dashboard,
            self.low_camera_resolution,
            self.robot_map_id,
            self.start_llm,
            self.llama_server,
            self.gguf_model,
            self.llm_port,
            self.domain_id,
        ):
            signal = "changed" if isinstance(widget, (Gtk.ComboBox, Gtk.Entry)) else "value-changed"
            if isinstance(widget, Gtk.ToggleButton):
                signal = "toggled"
            widget.connect(signal, self._on_option_changed)

    def _copy_script(self, text: str, message: str) -> None:
        if GTK_MAJOR == 3:
            clipboard = Gtk.Clipboard.get(Gdk.SELECTION_CLIPBOARD)
            clipboard.set_text(text, -1)
            clipboard.store()
        else:
            display = Gdk.Display.get_default()
            if display is not None:
                try:
                    clipboard = display.get_clipboard()
                    if hasattr(clipboard, "set_text"):
                        clipboard.set_text(text, -1)
                    elif hasattr(clipboard, "set"):
                        provider = Gdk.ContentProvider.new_for_value(text)
                        clipboard.set(provider)
                except Exception:
                    pass
        self.status.set_text(message if GTK_MAJOR == 3 else f"{message}（Gtk4 下请手动粘贴）")

    def _restore_settings(self) -> None:
        self._restoring = True
        s = self.settings
        if not self.scene_profile.set_active_id(str(s.get("scene_profile", "real_lab"))):
            self.scene_profile.set_active_id("real_lab")
        self.start_isaac.set_active(bool(s["start_isaac"]))
        self.low_camera_resolution.set_active(bool(s["low_camera_resolution"]))
        if not self.scene.set_active_id(str(s["scene"])):
            self.scene.set_active_id(str(DEFAULT_SCENE))
        self.start_ros.set_active(bool(s["start_ros"]))
        if not self.ros_mode.set_active_id(str(s.get("ros_mode", "nav2"))):
            self.ros_mode.set_active_id("nav2")
        if not self.map.set_active_id(str(s["map"])):
            self.map.set_active_id(str(DEFAULT_MAP))
        if not self.ply.set_active_id(str(s["ply"])):
            self.ply.set_active_id(str(DEFAULT_PLY))
        if not self.rviz.set_active_id(str(s["rviz"])):
            self.rviz.set_active_id(str(DEFAULT_RVIZ))
        self.initial_x.set_text(str(s["initial_x"]))
        self.initial_y.set_text(str(s["initial_y"]))
        self.initial_yaw.set_text(str(s["initial_yaw"]))
        self.scan_min_height.set_text(str(s["scan_min_height"]))
        self.scan_max_height.set_text(str(s["scan_max_height"]))
        self.cleanup.set_active(False)
        self.auto_mode.set_active(bool(s["auto_mode"]))
        self.motion_policy.set_active(bool(s["motion_policy"]))
        self.start_dashboard.set_active(bool(s["start_dashboard"]))
        self.robot_map_id.set_text(str(s["robot_map_id"]))
        self.start_llm.set_active(bool(s["start_llm"]))
        self.llama_server.set_text(str(s["llama_server"]))
        self.gguf_model.set_text(str(s["gguf_model"]))
        self.llm_port.set_text(str(s["llm_port"]))
        self.domain_id.set_value(int(s["ros_domain_id"]))
        self._restoring = False

    def _current_settings(self) -> dict:
        return {
            "scene_profile": self.scene_profile.get_active_id() or "real_lab",
            "start_isaac": self.start_isaac.get_active(),
            "low_camera_resolution": self.low_camera_resolution.get_active(),
            "scene": self.scene.get_active_id() or str(DEFAULT_SCENE),
            "start_ros": self.start_ros.get_active(),
            "ros_mode": self.ros_mode.get_active_id() or "nav2",
            "map": self.map.get_active_id() or str(DEFAULT_MAP),
            "ply": self.ply.get_active_id() or str(DEFAULT_PLY),
            "rviz": self.rviz.get_active_id() or str(DEFAULT_RVIZ),
            "initial_x": self.initial_x.get_text().strip() or "0.0",
            "initial_y": self.initial_y.get_text().strip() or "0.0",
            "initial_yaw": self.initial_yaw.get_text().strip() or "0.0",
            "scan_min_height": self.scan_min_height.get_text().strip() or "-0.8",
            "scan_max_height": self.scan_max_height.get_text().strip() or "-0.3",
            "cleanup": False,
            "auto_mode": self.auto_mode.get_active(),
            "motion_policy": self.motion_policy.get_active(),
            "start_dashboard": self.start_dashboard.get_active(),
            "robot_map_id": self.robot_map_id.get_text().strip() or "sim_real_lab_001",
            "start_llm": self.start_llm.get_active(),
            "llama_server": self.llama_server.get_text().strip(),
            "gguf_model": self.gguf_model.get_text().strip(),
            "llm_port": self.llm_port.get_text().strip() or "8090",
            "ros_domain_id": self.domain_id.get_value_as_int(),
        }

    def _update_preview(self) -> None:
        settings = self._current_settings()
        text_buffer = self.preview.get_buffer()
        text_buffer.set_text(self._preview_text(settings))

    def _isaac_command(self, settings: dict) -> list[str]:
        return [str(ISAAC_BIN), settings["scene"]]

    def _ros_command(self, settings: dict) -> list[str]:
        if settings.get("ros_mode") == "arm_test":
            command = [
                str(SIM_ARM_TEST_SCRIPT),
                "--rviz",
                settings["rviz"],
                "--ply",
                settings["ply"],
            ]
            return command
        command = [
            str(SIM_NAV2_SCRIPT),
            "--map",
            settings["map"],
            "--ply",
            settings["ply"],
            "--rviz",
            settings["rviz"],
            "--initial-x",
            settings["initial_x"],
            "--initial-y",
            settings["initial_y"],
            "--initial-yaw",
            settings["initial_yaw"],
        ]
        if settings["auto_mode"]:
            command.append("--auto")
        else:
            command.append("--no-auto")
        command.append("--motion-policy" if settings["motion_policy"] else "--no-motion-policy")
        return command

    @staticmethod
    def _camera_script(script: str, settings: dict) -> str:
        values = camera_environment(settings.get("low_camera_resolution", False))
        return "import os\nos.environ.update(" + repr(values) + ")\n" + script

    @staticmethod
    def _pre_play_script(settings: dict) -> str:
        script = ARM_TEST_PRE_PLAY_SCRIPT if settings.get("ros_mode") == "arm_test" else PRE_PLAY_SCRIPT
        return LauncherWindow._camera_script(script, settings)

    @staticmethod
    def _post_play_script(settings: dict) -> str:
        script = ARM_TEST_POST_PLAY_SCRIPT if settings.get("ros_mode") == "arm_test" else POST_PLAY_SCRIPT
        return LauncherWindow._camera_script(script, settings)

    @staticmethod
    def _post_play_expected(settings: dict) -> str:
        return ARM_TEST_POST_PLAY_EXPECTED if settings.get("ros_mode") == "arm_test" else POST_PLAY_EXPECTED

    def _script_editor_flow(self, settings: dict) -> str:
        mode_label = ROS_MODES.get(settings.get("ros_mode", "nav2"), str(settings.get("ros_mode")))
        return "\n".join(
            [
                f"# Isaac Script Editor / {mode_label}",
                "# 1) Stop playback first, then run this before pressing Play",
                self._pre_play_script(settings),
                "",
                "# 2) Press Isaac Play, then run this once while playing",
                self._post_play_script(settings),
                "",
            ]
        )

    def _llm_command(self, settings: dict) -> list[str]:
        return [
            settings["llama_server"],
            "-m",
            settings["gguf_model"],
            "--host",
            "127.0.0.1",
            "--port",
            settings["llm_port"],
            "--ctx-size",
            "4096",
        ]

    def _preview_text(self, settings: dict) -> str:
        mode_label = ROS_MODES.get(settings.get("ros_mode", "nav2"), str(settings.get("ros_mode")))
        lines = [
            f"export ROS_DOMAIN_ID={settings['ros_domain_id']}",
            "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
            "unset CYCLONEDDS_URI",
        ]
        starting = [group for group, key in (("isaac", "start_isaac"), ("ros", "start_ros"),
                                             ("agent", "start_dashboard")) if settings[key]]
        if starting:
            lines.extend(["# Read-only preflight; does not stop existing processes.",
                          shlex.join(["/usr/bin/python3", str(WORKSPACE / "tools/sim_startup_preflight.py"),
                                      "--domain", str(settings["ros_domain_id"]), "--starting", *starting])])
        if settings["start_isaac"]:
            lines.extend([
                "",
                "# Isaac Sim",
                "export PYTHONNOUSERSITE=1",
                *["export " + key + "=" + shlex.quote(value)
                  for key, value in camera_environment(settings.get("low_camera_resolution", False)).items()],
                f"cd {shlex.quote(str(ISAAC_DIR))}",
                shlex.join(self._isaac_command(settings)),
            ])
        lines.extend(["", self._script_editor_flow(settings).rstrip(), "", "# Play 后预期 Console 输出", self._post_play_expected(settings).strip()])
        if settings["start_ros"]:
            lines.extend([
                "",
                f"# ROS sim / {mode_label}",
                f"cd {shlex.quote(str(WORKSPACE))}",
                "export PYTHONNOUSERSITE=1",
                "export SCAN_MIN_HEIGHT=" + shlex.quote(settings["scan_min_height"]),
                "export SCAN_MAX_HEIGHT=" + shlex.quote(settings["scan_max_height"]),
                shlex.join(self._ros_command(settings)),
            ])
        if settings["start_dashboard"]:
            lines.extend(["", "# Dashboard", f"ROBOT_MAP_ID={shlex.quote(settings['robot_map_id'])} RECOVERY_PROFILE={shlex.quote(settings.get('ros_mode', 'nav2'))} RECOVERY_MAP_PLY={shlex.quote(settings.get('ply', ''))} {shlex.quote(str(DASHBOARD_SCRIPT))}"])
        if settings["start_llm"]:
            lines.extend(["", "# Local LLM Parser", shlex.join(self._llm_command(settings))])
        if settings["start_ros"] or settings["start_dashboard"]:
            lines.extend(["", ROS_CHECK_COMMANDS.strip()])
        if not any(settings[key] for key in ("start_isaac", "start_ros", "start_dashboard", "start_llm")):
            lines.append("# 没有选择启动项")
        return "\n".join(lines)

    def _update_sensitivity(self) -> None:
        for widget in (self.scene,):
            widget.set_sensitive(self.start_isaac.get_active())
        ros_enabled = self.start_ros.get_active()
        nav2_mode = (self.ros_mode.get_active_id() or "nav2") == "nav2"
        self.ros_mode.set_sensitive(ros_enabled)
        for widget in (self.ply, self.rviz, self.cleanup):
            widget.set_sensitive(ros_enabled)
        for widget in (self.map, self.initial_x, self.initial_y, self.initial_yaw, self.scan_min_height, self.scan_max_height, self.auto_mode, self.motion_policy):
            widget.set_sensitive(ros_enabled and nav2_mode)
        self.robot_map_id.set_sensitive(self.start_dashboard.get_active())
        for widget in (self.llama_server, self.gguf_model, self.llm_port):
            widget.set_sensitive(self.start_llm.get_active())

    def _on_option_changed(self, _widget: Gtk.Widget) -> None:
        self._update_sensitivity()
        self._update_preview()

    def _on_reset(self, _button: Gtk.Button) -> None:
        self.settings = dict(DEFAULTS)
        self._restore_settings()
        self.status.set_text("已恢复默认")

    def _show_error(self, title: str, detail: str) -> None:
        if GTK_MAJOR == 3:
            dialog = Gtk.MessageDialog(
                transient_for=self,
                modal=True,
                message_type=Gtk.MessageType.ERROR,
                buttons=Gtk.ButtonsType.CLOSE,
                text=title,
            )
            dialog.format_secondary_text(detail)
            dialog.run()
            dialog.destroy()
            return
        self.status.set_text(f"{title}: {detail}")

    def _validate(self, settings: dict) -> str | None:
        if not any(settings[key] for key in ("start_isaac", "start_ros", "start_dashboard", "start_llm")):
            return "至少选择一个启动项。"
        if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
            return (
                "当前 quick launcher 没有图形 DISPLAY/WAYLAND_DISPLAY，无法打开 gnome-terminal。\n"
                "请从本机图形桌面里的 terminal 运行 tools/run_sim_robot_quick_start.sh，"
                "或从系统应用菜单启动 Real-Lab Sim Quick Start。"
            )
        if shutil.which("gnome-terminal") is None:
            return "找不到 gnome-terminal。"
        if settings["start_isaac"] and not ISAAC_BIN.is_file():
            return f"找不到 Isaac Sim 启动脚本: {ISAAC_BIN}"
        for key in ("scene", "map", "ply", "rviz"):
            if settings["start_isaac"] and key == "scene" and not Path(settings[key]).is_file():
                return f"找不到 USD: {settings[key]}"
            if (
                settings["start_ros"]
                and settings.get("ros_mode") == "nav2"
                and key in {"map", "ply", "rviz"}
                and not Path(settings[key]).is_file()
            ):
                return f"找不到 {key}: {settings[key]}"
            if (
                settings["start_ros"]
                and settings.get("ros_mode") == "arm_test"
                and key in {"ply", "rviz"}
                and not Path(settings[key]).is_file()
            ):
                return f"找不到 {key}: {settings[key]}"
        if settings["start_ros"] and settings.get("ros_mode") == "nav2" and not SIM_NAV2_SCRIPT.is_file():
            return f"找不到 ROS sim launcher: {SIM_NAV2_SCRIPT}"
        if settings["start_ros"] and settings.get("ros_mode") == "arm_test" and not SIM_ARM_TEST_SCRIPT.is_file():
            return f"找不到 Arm Test launcher: {SIM_ARM_TEST_SCRIPT}"
        if settings["start_dashboard"] and not DASHBOARD_SCRIPT.is_file():
            return f"找不到 Dashboard launcher: {DASHBOARD_SCRIPT}"
        if settings["start_llm"]:
            if not settings["llama_server"] or not Path(settings["llama_server"]).is_file():
                return "请填写有效的 llama-server 路径。"
            if not settings["gguf_model"] or not Path(settings["gguf_model"]).is_file():
                return "请填写有效的 GGUF 模型路径。"
        try:
            float(settings["initial_x"])
            float(settings["initial_y"])
            float(settings["initial_yaw"])
            float(settings["scan_min_height"])
            float(settings["scan_max_height"])
            int(settings["llm_port"])
        except ValueError:
            return "initial pose 和 LLM port 必须是数字。"
        return None

    def _set_combo_or_status(self, combo: Gtk.ComboBoxText, item_id: str, label: str) -> None:
        if not combo.set_active_id(item_id):
            self.status.set_text(f"配置里找不到 {label}: {item_id}")

    def _apply_profile(self, profile_id: str) -> None:
        profile = SCENE_PROFILES.get(profile_id)
        if not profile:
            return
        self._set_combo_or_status(self.scene, profile["scene"], "USD")
        self._set_combo_or_status(self.map, profile["map"], "Nav2 map")
        self._set_combo_or_status(self.ply, profile["ply"], "PLY")
        self._set_combo_or_status(self.rviz, profile["rviz"], "RViz")
        self.initial_x.set_text(profile["initial_x"])
        self.initial_y.set_text(profile["initial_y"])
        self.initial_yaw.set_text(profile["initial_yaw"])
        self.scan_min_height.set_text(profile["scan_min_height"])
        self.scan_max_height.set_text(profile["scan_max_height"])
        self.robot_map_id.set_text(profile["robot_map_id"])

    def _on_profile_changed(self, _widget: Gtk.Widget) -> None:
        if self._restoring:
            return
        profile_id = self.scene_profile.get_active_id() or "real_lab"
        self._apply_profile(profile_id)
        self._update_sensitivity()
        self._update_preview()

    def _on_start(self, _button: Gtk.Button) -> None:
        if not self.start_button.get_sensitive():
            return
        settings = self._current_settings()
        starting = [group for group, key in (("isaac", "start_isaac"), ("ros", "start_ros"),
                                             ("agent", "start_dashboard")) if settings[key]]
        issues = check_startup(WORKSPACE, settings["ros_domain_id"], starting) if starting else []
        if issues:
            self._show_error("Simulation startup blocked", "\n\n".join(issues))
            return
        if settings["start_dashboard"]:
            existing = find_existing(WORKSPACE, settings["ros_domain_id"])
            if existing:
                self._show_error("Agent Dashboard 已在运行", "请先使用 Stop Agent stack，或取消勾选启动 Dashboard 以复用当前服务。\n"
                                 + "\n".join(f"PID {pid}: {name}" for pid, name in existing))
                return
        error = self._validate(settings)
        if error:
            self._show_error("无法启动", error)
            return
        save_settings(settings)
        append_log("requested\n" + self._preview_text(settings))
        self.start_button.set_sensitive(False)
        self.status.set_text("正在启动...")
        threading.Thread(target=self._launch, args=(settings,), daemon=True).start()

    def _launch_terminal(self, title: str, command: list[str], settings: dict, cwd: Path, extra_env: dict[str, str] | None = None) -> None:
        env = os.environ.copy()
        env.update(
            {
                "ROS_DOMAIN_ID": str(settings["ros_domain_id"]),
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "ROBOT_MAP_ID": settings["robot_map_id"],
            }
        )
        env.pop("CYCLONEDDS_URI", None)
        if extra_env:
            env.update(extra_env)
        export_extra = [
            "export " + key + "=" + shlex.quote(str(value))
            for key, value in (extra_env or {}).items()
        ]
        safe_title = title.replace("'", "'\"'\"'")
        safe_command = shlex.join(command).replace("'", "'\"'\"'")
        shell = "\n".join(
            [
                "export ROS_DOMAIN_ID=" + shlex.quote(str(settings["ros_domain_id"])),
                "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
                "unset CYCLONEDDS_URI",
                *export_extra,
                "cd " + shlex.quote(str(cwd)),
                "echo '===== " + safe_title + " ====='",
                "echo '[launcher] cwd: " + shlex.quote(str(cwd)) + "'",
                "echo '[launcher] command: " + safe_command + "'",
                "set +e",
                shlex.join(command),
                "rc=$?",
                "echo",
                "echo \"[launcher] command exited with code $rc\"",
                "if [ \"$rc\" -ne 0 ]; then echo '[launcher] 启动失败；按 Enter 关闭窗口。'; read -r _; fi",
                "exit $rc",
            ]
        )
        proc = subprocess.Popen(
            ["gnome-terminal", f"--title={title}", "--", "bash", "-lc", shell],
            cwd=cwd,
            env=env,
            start_new_session=True,
        )
        append_log(
            f"launch terminal: title={title!r} pid={proc.pid} cwd={cwd} "
            f"display={env.get('DISPLAY', '')!r} wayland={env.get('WAYLAND_DISPLAY', '')!r} "
            f"command={shlex.join(command)}"
        )

    def _launch(self, settings: dict) -> None:
        try:
            starting = [group for group, key in (("isaac", "start_isaac"), ("ros", "start_ros"),
                                                 ("agent", "start_dashboard")) if settings[key]]
            issues = check_startup(WORKSPACE, settings["ros_domain_id"], starting) if starting else []
            if issues:
                GLib.idle_add(self._launch_failed, "\n\n".join(issues))
                return
            if settings["start_llm"]:
                self._launch_terminal("Local LLM Parser", self._llm_command(settings), settings, Path(settings["llama_server"]).parent)
            if settings["start_isaac"]:
                self._launch_terminal("Isaac Sim Real Lab", self._isaac_command(settings), settings, ISAAC_DIR,
                                      {"PYTHONNOUSERSITE": "1", **camera_environment(settings.get("low_camera_resolution", False))})
            if settings["start_ros"]:
                ros_title = (
                    "ROS Sim Arm Test"
                    if settings.get("ros_mode") == "arm_test"
                    else "ROS Sim Nav2 Arm Touch RViz"
                )
                self._launch_terminal(
                    ros_title,
                    self._ros_command(settings),
                    settings,
                    WORKSPACE,
                    {
                        "PYTHONNOUSERSITE": "1",
                        "SCAN_MIN_HEIGHT": settings["scan_min_height"],
                        "SCAN_MAX_HEIGHT": settings["scan_max_height"],
                    },
                )
            if settings["start_dashboard"]:
                self._launch_terminal(
                    "Agent Dashboard Sim",
                    [str(DASHBOARD_SCRIPT)],
                    settings,
                    WORKSPACE,
                    {
                        "RECOVERY_PROFILE": settings.get("ros_mode", "nav2"),
                        "RECOVERY_MAP_PLY": settings.get("ply", ""),
                        "RECOVERY_LAUNCH_SETTINGS": json.dumps({key: settings[key] for key in (
                            "ros_domain_id", "ros_mode", "robot_map_id", "map", "ply", "motion_policy",
                            "initial_x", "initial_y", "initial_yaw")}),
                    },
                )
        except OSError as exc:
            append_log(f"ERROR: {exc}")
            GLib.idle_add(self._launch_failed, str(exc))
            return
        GLib.idle_add(self._launch_succeeded)

    def _launch_failed(self, detail: str) -> bool:
        self.start_button.set_sensitive(True)
        self.status.set_text("启动失败")
        self._show_error("启动失败", f"{detail}\n\nLog: {LOG_FILE}")
        return GLib.SOURCE_REMOVE

    def _launch_succeeded(self) -> bool:
        self.status.set_text(f"启动请求已发出。配置已保存，Log: {LOG_FILE}")
        if GTK_MAJOR == 3:
            dialog = Gtk.MessageDialog(
                transient_for=self,
                modal=True,
                message_type=Gtk.MessageType.INFO,
                buttons=Gtk.ButtonsType.OK,
                text="启动窗口已打开",
            )
            dialog.format_secondary_text(
                "配置已保存。\n"
                "Script Editor 仍需按按钮复制并在 Isaac 内执行。\n"
                "Play 后等待：SO101 Isaac UDP joint bridge ready / Arm Camera ROS2 publishers ready。\n"
                f"Log: {LOG_FILE}"
            )
            dialog.run()
            dialog.destroy()
        self.start_button.set_sensitive(True)
        return GLib.SOURCE_REMOVE


def main() -> int:
    window = LauncherWindow()
    window.connect("destroy", Gtk.main_quit)
    if GTK_MAJOR == 3:
        window.show_all()
    else:
        window.show()
    Gtk.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
