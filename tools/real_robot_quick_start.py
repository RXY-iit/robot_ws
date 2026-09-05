#!/usr/bin/env python3
"""Desktop launcher for the full robot, Arm-only, and Agent Dashboard stacks."""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import threading
from datetime import datetime
from pathlib import Path

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk  # noqa: E402


WORKSPACE = Path("/home/matsunaga-h/robot_ws")
NAV2_SCRIPT = WORKSPACE / "tools/open_nav2_windows.sh"
DASHBOARD_SCRIPT = WORKSPACE / "tools/run_agent_dashboard_real.sh"
DEFAULT_MAP = WORKSPACE / "maps/l402_2d_map_clean_0509.yaml"
CONFIG_DIR = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "realbot-launcher"
CONFIG_FILE = CONFIG_DIR / "settings.json"
STATE_DIR = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state")) / "realbot-launcher"
LOG_FILE = STATE_DIR / "launcher.log"

DEFAULTS = {
    "start_nav2": True,
    "robot_mode": "full",
    "localization": "gicp",
    "motion_policy": True,
    "cleanup": True,
    "debug_recorder": False,
    "start_arm": True,
    "nav2_map": str(DEFAULT_MAP),
    "start_dashboard": True,
    "ros_domain_id": 13,
    "robot_map_id": "l402_2d_map_clean_0509",
}


def load_settings() -> dict:
    settings = dict(DEFAULTS)
    try:
        data = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            settings.update({key: data[key] for key in DEFAULTS if key in data})
    except (FileNotFoundError, OSError, ValueError, TypeError):
        pass
    return settings


def save_settings(settings: dict) -> None:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    temporary = CONFIG_FILE.with_suffix(".tmp")
    temporary.write_text(json.dumps(settings, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(CONFIG_FILE)


def append_log(message: str) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
    with LOG_FILE.open("a", encoding="utf-8") as stream:
        stream.write(f"[{timestamp}] {message}\n")


class LauncherWindow(Gtk.Window):
    def __init__(self) -> None:
        super().__init__(title="Real Robot Quick Start / 実機クイックスタート")
        self.set_border_width(18)
        self.set_default_size(720, 650)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.settings = load_settings()
        self._build_ui()
        self._restore_settings()
        self._update_sensitivity()
        self._update_preview()

    @staticmethod
    def _label(text: str) -> Gtk.Label:
        label = Gtk.Label(label=text, xalign=0)
        label.set_line_wrap(True)
        return label

    @staticmethod
    def _combo(entries: list[tuple[str, str]]) -> Gtk.ComboBoxText:
        combo = Gtk.ComboBoxText()
        for item_id, label in entries:
            combo.append(item_id, label)
        return combo

    def _add_row(self, grid: Gtk.Grid, row: int, title: str, widget: Gtk.Widget) -> None:
        label = self._label(title)
        label.set_hexpand(True)
        grid.attach(label, 0, row, 1, 1)
        grid.attach(widget, 1, row, 1, 1)

    def _section(self, title: str) -> tuple[Gtk.Frame, Gtk.Grid]:
        frame = Gtk.Frame(label=title)
        grid = Gtk.Grid(column_spacing=18, row_spacing=10, margin=12)
        frame.add(grid)
        return frame, grid

    def _build_ui(self) -> None:
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14)
        self.add(root)

        title = self._label("Start the verified real-robot stack / 検証済み実機構成を起動")
        title.get_style_context().add_class("title")
        root.pack_start(title, False, False, 0)
        root.pack_start(
            self._label(
                "Selections are saved when Start is pressed and restored next time. "
                "/ 起動時に選択内容を保存し、次回に復元します。"
            ),
            False,
            False,
            0,
        )

        nav_frame, nav_grid = self._section("Nav2 and robot stack / Nav2・実機スタック")
        root.pack_start(nav_frame, False, False, 0)
        self.start_nav2 = Gtk.CheckButton(
            label="Start selected robot stack / 選択した実機構成を起動"
        )
        nav_grid.attach(self.start_nav2, 0, 0, 2, 1)
        self.robot_mode = self._combo(
            [
                ("full", "Full robot / 実機フル構成"),
                ("safe", "Phase 1 safe (no motors) / モーターなし安全確認"),
                (
                    "no_oriental",
                    "No Oriental Motor (virtual drive) / Oriental Motorなし（仮想駆動）",
                ),
                ("arm_only", "Arm only / SO101アームのみ"),
            ]
        )
        self.localization = self._combo(
            [
                ("gicp", "GICP only (--no-fast-lio) [verified / 検証済み]"),
                ("fast_lio", "FAST-LIO + GICP (--fast-lio) [test / 試験用]"),
            ]
        )
        self.motion_policy = Gtk.CheckButton(label="Enabled / 有効")
        self.cleanup = Gtk.CheckButton(label="Enabled / 有効")
        self.debug_recorder = Gtk.CheckButton(label="Enabled / 有効")
        self.nav2_map = self._combo(self._discover_maps())
        self.start_arm = Gtk.CheckButton(label="Start SO101 arm / SO101アームを起動")
        self._add_row(nav_grid, 1, "Startup mode / 起動モード", self.robot_mode)
        self._add_row(nav_grid, 2, "Localization / 自己位置推定", self.localization)
        self._add_row(nav_grid, 3, "Motion policy / モーションポリシー", self.motion_policy)
        self._add_row(nav_grid, 4, "Cleanup old processes / 既存プロセスを整理", self.cleanup)
        self._add_row(nav_grid, 5, "Persistent debug recorder / 永続デバッグ記録", self.debug_recorder)
        self._add_row(nav_grid, 6, "Nav2 map / Nav2 マップ", self.nav2_map)
        self._add_row(nav_grid, 7, "Follower arm / フォロワーアーム", self.start_arm)

        dashboard_frame, dashboard_grid = self._section("Agent Dashboard")
        root.pack_start(dashboard_frame, False, False, 0)
        self.start_dashboard = Gtk.CheckButton(label="Start Agent Dashboard / Agent Dashboard を起動")
        dashboard_grid.attach(self.start_dashboard, 0, 0, 2, 1)
        self.robot_map_id = Gtk.Entry()
        self.robot_map_id.set_placeholder_text("l402_2d_map_clean_0509")
        self._add_row(dashboard_grid, 1, "ROBOT_MAP_ID", self.robot_map_id)

        common_frame, common_grid = self._section("Common / 共通")
        root.pack_start(common_frame, False, False, 0)
        self.domain_id = Gtk.SpinButton.new_with_range(0, 232, 1)
        self.domain_id.set_numeric(True)
        self._add_row(common_grid, 0, "ROS_DOMAIN_ID", self.domain_id)

        preview_frame = Gtk.Frame(label="Command preview / コマンド確認")
        preview_scroll = Gtk.ScrolledWindow()
        preview_scroll.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        preview_scroll.set_min_content_height(92)
        self.preview = Gtk.TextView()
        self.preview.set_editable(False)
        self.preview.set_cursor_visible(False)
        self.preview.set_monospace(True)
        self.preview.set_wrap_mode(Gtk.WrapMode.WORD_CHAR)
        self.preview.set_margin_left(8)
        self.preview.set_margin_right(8)
        self.preview.set_margin_top(8)
        self.preview.set_margin_bottom(8)
        preview_scroll.add(self.preview)
        preview_frame.add(preview_scroll)
        root.pack_start(preview_frame, True, True, 0)

        self.status = self._label("Ready / 起動準備完了")
        root.pack_start(self.status, False, False, 0)

        buttons = Gtk.ButtonBox(orientation=Gtk.Orientation.HORIZONTAL)
        buttons.set_layout(Gtk.ButtonBoxStyle.END)
        buttons.set_spacing(10)
        reset = Gtk.Button(label="Restore defaults / 既定値に戻す")
        cancel = Gtk.Button(label="Cancel / キャンセル")
        self.start_button = Gtk.Button(label="Start selected / 選択した構成を起動")
        self.start_button.get_style_context().add_class(Gtk.STYLE_CLASS_SUGGESTED_ACTION)
        buttons.add(reset)
        buttons.add(cancel)
        buttons.add(self.start_button)
        root.pack_start(buttons, False, False, 0)

        reset.connect("clicked", self._on_reset)
        cancel.connect("clicked", lambda _button: self.close())
        self.start_button.connect("clicked", self._on_start)
        for widget in (
            self.start_nav2,
            self.robot_mode,
            self.localization,
            self.motion_policy,
            self.cleanup,
            self.debug_recorder,
            self.nav2_map,
            self.start_arm,
            self.start_dashboard,
            self.robot_map_id,
            self.domain_id,
        ):
            signal = "changed" if isinstance(widget, (Gtk.ComboBox, Gtk.Entry)) else "value-changed"
            if isinstance(widget, Gtk.ToggleButton):
                signal = "toggled"
            widget.connect(signal, self._on_option_changed)

    @staticmethod
    def _discover_maps() -> list[tuple[str, str]]:
        paths = sorted((WORKSPACE / "maps").glob("*.yaml"))
        if DEFAULT_MAP not in paths:
            paths.insert(0, DEFAULT_MAP)
        return [(str(path), path.name) for path in paths]

    def _restore_settings(self) -> None:
        value = self.settings
        self.start_nav2.set_active(bool(value["start_nav2"]))
        self.robot_mode.set_active_id(str(value["robot_mode"]))
        self.localization.set_active_id(str(value["localization"]))
        self.motion_policy.set_active(bool(value["motion_policy"]))
        self.cleanup.set_active(bool(value["cleanup"]))
        self.debug_recorder.set_active(bool(value["debug_recorder"]))
        self.start_arm.set_active(bool(value["start_arm"]))
        if not self.nav2_map.set_active_id(str(value["nav2_map"])):
            self.nav2_map.set_active_id(str(DEFAULT_MAP))
        self.start_dashboard.set_active(bool(value["start_dashboard"]))
        self.domain_id.set_value(int(value["ros_domain_id"]))
        self.robot_map_id.set_text(str(value["robot_map_id"]))

    def _current_settings(self) -> dict:
        return {
            "start_nav2": self.start_nav2.get_active(),
            "robot_mode": self.robot_mode.get_active_id() or "full",
            "localization": self.localization.get_active_id() or "gicp",
            "motion_policy": self.motion_policy.get_active(),
            "cleanup": self.cleanup.get_active(),
            "debug_recorder": self.debug_recorder.get_active(),
            "start_arm": self.start_arm.get_active(),
            "nav2_map": self.nav2_map.get_active_id() or str(DEFAULT_MAP),
            "start_dashboard": self.start_dashboard.get_active(),
            "ros_domain_id": self.domain_id.get_value_as_int(),
            "robot_map_id": self.robot_map_id.get_text().strip(),
        }

    def _nav2_command(self, settings: dict) -> list[str]:
        command = [str(NAV2_SCRIPT)]
        command.append("--fast-lio" if settings["localization"] == "fast_lio" else "--no-fast-lio")
        command.append("--motion-policy" if settings["motion_policy"] else "--no-motion-policy")
        if settings["robot_mode"] == "safe":
            command.append("--phase1-safe")
        elif settings["robot_mode"] == "no_oriental":
            command.append("--no-oriental-motor")
        if settings["cleanup"]:
            command.append("--cleanup")
        if settings["debug_recorder"]:
            command.append("--debug-recorder")
        arm_enabled = settings["start_arm"] and settings["robot_mode"] in {"full", "no_oriental"}
        command.append("--arm" if arm_enabled else "--no-arm")
        if Path(settings["nav2_map"]) != DEFAULT_MAP:
            command.extend(["--map", settings["nav2_map"]])
        return command

    @staticmethod
    def _arm_only_command() -> list[str]:
        return ["ros2", "launch", "robot_bringup", "arm_only.launch.py"]

    def _preview_text(self, settings: dict) -> str:
        lines = [f"cd {shlex.quote(str(WORKSPACE))}", f"export ROS_DOMAIN_ID={settings['ros_domain_id']}"]
        arm_only = settings["start_nav2"] and settings["robot_mode"] == "arm_only"
        if arm_only:
            lines.append(shlex.join(self._arm_only_command()))
            lines.append(
                "# Complete robot URDF + Arm actual/preview RViz + Arm/Health Dashboard; "
                "no Nav2 or mobile hardware"
            )
        elif settings["start_nav2"]:
            lines.append(shlex.join(self._nav2_command(settings)))
        if settings["start_dashboard"] and not arm_only:
            lines.append(f"export ROBOT_MAP_ID={shlex.quote(settings['robot_map_id'] or '<required>')}")
            lines.append(str(DASHBOARD_SCRIPT))
        if not settings["start_nav2"] and not settings["start_dashboard"]:
            lines.append("# Nothing selected / 起動対象が選択されていません")
        return "\n".join(lines)

    def _update_preview(self) -> None:
        text = self._preview_text(self._current_settings())
        self.preview.get_buffer().set_text(text)

    def _update_sensitivity(self) -> None:
        nav_enabled = self.start_nav2.get_active()
        arm_only = nav_enabled and self.robot_mode.get_active_id() == "arm_only"
        for widget in (
            self.localization,
            self.motion_policy,
            self.debug_recorder,
            self.nav2_map,
        ):
            widget.set_sensitive(nav_enabled and not arm_only)
        self.robot_mode.set_sensitive(nav_enabled)
        # Arm-only intentionally does not terminate an existing Arm owner: the
        # serial-owner lock will reject a duplicate without risking a held arm.
        self.cleanup.set_sensitive(nav_enabled and not arm_only)
        self.start_arm.set_sensitive(
            nav_enabled
            and self.robot_mode.get_active_id() in {"full", "no_oriental"}
            and not arm_only
        )
        self.start_dashboard.set_sensitive(not arm_only)
        self.robot_map_id.set_sensitive(self.start_dashboard.get_active() and not arm_only)

    def _on_option_changed(self, _widget: Gtk.Widget) -> None:
        self._update_sensitivity()
        self._update_preview()

    def _on_reset(self, _button: Gtk.Button) -> None:
        self.settings = dict(DEFAULTS)
        self._restore_settings()
        self.status.set_text("Defaults restored / 既定値を復元しました")

    def _show_error(self, primary: str, secondary: str = "") -> None:
        dialog = Gtk.MessageDialog(
            transient_for=self,
            modal=True,
            message_type=Gtk.MessageType.ERROR,
            buttons=Gtk.ButtonsType.CLOSE,
            text=primary,
        )
        if secondary:
            dialog.format_secondary_text(secondary)
        dialog.run()
        dialog.destroy()

    def _validate(self, settings: dict) -> str | None:
        if not settings["start_nav2"] and not settings["start_dashboard"]:
            return "Select at least one target. / 起動対象を1つ以上選択してください。"
        arm_only = settings["start_nav2"] and settings["robot_mode"] == "arm_only"
        if settings["start_nav2"] and not arm_only:
            if not NAV2_SCRIPT.is_file():
                return f"Missing launcher / 起動スクリプトがありません: {NAV2_SCRIPT}"
            if not Path(settings["nav2_map"]).is_file():
                return f"Missing Nav2 map / Nav2マップがありません: {settings['nav2_map']}"
        if settings["start_dashboard"] and not arm_only:
            if not DASHBOARD_SCRIPT.is_file():
                return f"Missing launcher / 起動スクリプトがありません: {DASHBOARD_SCRIPT}"
            map_id = settings["robot_map_id"]
            if not map_id or any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-" for character in map_id):
                return "ROBOT_MAP_ID may contain only letters, digits, '.', '_' and '-'."
        if shutil.which("gnome-terminal") is None:
            return "gnome-terminal is not installed. / gnome-terminal が見つかりません。"
        return None

    def _on_start(self, _button: Gtk.Button) -> None:
        settings = self._current_settings()
        error = self._validate(settings)
        if error:
            self._show_error("Cannot start / 起動できません", error)
            return
        try:
            save_settings(settings)
        except OSError as exc:
            self._show_error("Cannot save settings / 設定を保存できません", str(exc))
            return
        self.start_button.set_sensitive(False)
        self.status.set_text("Starting... / 起動中...")
        append_log("requested\n" + self._preview_text(settings))
        threading.Thread(target=self._launch, args=(settings,), daemon=True).start()

    def _launch(self, settings: dict) -> None:
        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = str(settings["ros_domain_id"])
        arm_only = settings["start_nav2"] and settings["robot_mode"] == "arm_only"
        try:
            if arm_only:
                terminal_script = (
                    'cd "$1"; set +u; '
                    'source /opt/ros/humble/setup.bash; source "$1/install/setup.bash"; '
                    'set -u; export ROS_DOMAIN_ID="$2" ROS_LOG_DIR=/tmp/arm_only_ros_logs; '
                    'mkdir -p "$ROS_LOG_DIR"; '
                    'exec ros2 launch robot_bringup arm_only.launch.py'
                )
                subprocess.Popen(
                    [
                        "gnome-terminal",
                        "--title=SO101 Arm Only",
                        "--",
                        "bash",
                        "-lc",
                        terminal_script,
                        "_",
                        str(WORKSPACE),
                        str(settings["ros_domain_id"]),
                    ],
                    cwd=WORKSPACE,
                    env=environment,
                    start_new_session=True,
                )
                append_log("SO101 Arm-only terminal requested")
            elif settings["start_nav2"]:
                result = subprocess.run(
                    self._nav2_command(settings),
                    cwd=WORKSPACE,
                    env=environment,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=90,
                    check=False,
                )
                append_log(f"Nav2 launcher exit={result.returncode}\n{result.stdout.rstrip()}")
                if result.returncode != 0:
                    raise RuntimeError(result.stdout.strip() or f"Nav2 launcher exited with {result.returncode}")
            if settings["start_dashboard"] and not arm_only:
                terminal_script = (
                    'cd "$1" && export ROS_DOMAIN_ID="$2" ROBOT_MAP_ID="$3"; '
                    '"$1/tools/run_agent_dashboard_real.sh"; status=$?; '
                    'echo; echo "[Agent Dashboard finished: exit $status]"; '
                    'echo "Press Enter to close / Enterで閉じる"; read -r; exit "$status"'
                )
                subprocess.Popen(
                    [
                        "gnome-terminal",
                        "--title=Real Robot Agent Dashboard",
                        "--",
                        "bash",
                        "-lc",
                        terminal_script,
                        "_",
                        str(WORKSPACE),
                        str(settings["ros_domain_id"]),
                        settings["robot_map_id"],
                    ],
                    cwd=WORKSPACE,
                    env=environment,
                    start_new_session=True,
                )
                append_log("Agent Dashboard terminal requested")
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            append_log(f"ERROR: {exc}")
            GLib.idle_add(self._launch_failed, str(exc))
            return
        GLib.idle_add(self._launch_succeeded)

    def _launch_failed(self, details: str) -> bool:
        self.start_button.set_sensitive(True)
        self.status.set_text("Start failed / 起動失敗")
        self._show_error("Start failed / 起動に失敗しました", f"{details}\n\nLog: {LOG_FILE}")
        return GLib.SOURCE_REMOVE

    def _launch_succeeded(self) -> bool:
        self.status.set_text("Start requested successfully / 起動要求が完了しました")
        dialog = Gtk.MessageDialog(
            transient_for=self,
            modal=True,
            message_type=Gtk.MessageType.INFO,
            buttons=Gtk.ButtonsType.OK,
            text="Startup windows requested / 起動ウィンドウを開きました",
        )
        dialog.format_secondary_text(f"Selections saved for next time.\nLog: {LOG_FILE}")
        dialog.run()
        dialog.destroy()
        self.close()
        return GLib.SOURCE_REMOVE


def main() -> int:
    window = LauncherWindow()
    window.connect("destroy", Gtk.main_quit)
    window.show_all()
    Gtk.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
