import importlib.util
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("sim_preflight", ROOT / "tools/sim_startup_preflight.py")
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


def process(tmp_path, pid, argv, domain="20", rmw="rmw_fastrtps_cpp"):
    entry = tmp_path / str(pid)
    entry.mkdir()
    (entry / "cmdline").write_bytes(("\0".join(argv) + "\0").encode())
    env = [f"ROS_DOMAIN_ID={domain}"]
    if rmw is not None:
        env.append(f"RMW_IMPLEMENTATION={rmw}")
    (entry / "environ").write_bytes(("\0".join(env) + "\0").encode())
    return entry


def test_same_domain_mixed_rmw_blocks_dashboard_even_when_ros_reused(tmp_path):
    process(tmp_path, 1001, ["/usr/bin/python3", str(ROOT / "install/localization_pkg/lib/localization_pkg/gicp_localizer_node")],
            rmw="rmw_cyclonedds_cpp")
    issues = preflight.check_startup(ROOT, 20, ["agent"], tmp_path)
    assert len(issues) == 1 and "rmw_cyclonedds_cpp" in issues[0]
    assert "PID 1001" in issues[0]


def test_existing_gicp_scope_process_blocks_ros_but_can_be_reused(tmp_path):
    process(tmp_path, 1002, ["/usr/bin/python3", "-u", str(ROOT / "install/localization_pkg/lib/localization_pkg/gicp_localizer_node")])
    assert preflight.check_startup(ROOT, 20, ["ros"], tmp_path)
    assert not preflight.check_startup(ROOT, 20, ["agent"], tmp_path)


def test_isaac_uses_its_own_process_environment(tmp_path):
    process(tmp_path, 1003, ["/home/user/isaac-sim/kit/kit", "isaacsim.exp.full.kit"],
            rmw="rmw_cyclonedds_cpp")
    rows = preflight.process_inventory(ROOT, 20, tmp_path)
    assert rows[0]["group"] == "isaac"
    assert len(preflight.startup_issues(rows, ["isaac"])) == 2


def test_other_domain_and_shell_text_are_not_robot_processes(tmp_path):
    target = str(ROOT / "install/robot_agent/lib/robot_agent/agent_node")
    process(tmp_path, 1004, ["/usr/bin/python3", target], domain="21", rmw="rmw_cyclonedds_cpp")
    process(tmp_path, 1005, ["/bin/bash", "-lc", target])
    process(tmp_path, 1006, ["/usr/bin/python3", "-c", "print('x')", target])
    assert preflight.process_inventory(ROOT, 20, tmp_path) == []


def test_unknown_rmw_is_not_silently_assumed_fastdds(tmp_path):
    process(tmp_path, 1007, ["/opt/ros/humble/lib/nav2_controller/controller_server"], rmw=None)
    assert "cannot verify" in preflight.check_startup(ROOT, 20, [], tmp_path)[0]


def test_unreadable_robot_environment_blocks_without_claiming_domain(tmp_path, monkeypatch):
    entry = process(tmp_path, 1008, ["/opt/ros/humble/lib/nav2_controller/controller_server"])
    original = Path.read_bytes
    def read(path):
        if path == entry / "environ":
            raise PermissionError()
        return original(path)
    monkeypatch.setattr(Path, "read_bytes", read)
    rows = preflight.process_inventory(ROOT, 20, tmp_path)
    assert rows[0]["domain"] is None
    assert preflight.startup_issues(rows)


def test_exited_process_is_ignored(tmp_path):
    (tmp_path / "1009").mkdir()
    assert preflight.process_inventory(ROOT, 20, tmp_path) == []


@pytest.mark.parametrize("relative", [
    "tools/run_sim_robot_quick_start.sh",
    "tools/sim/open_sim_nav2_windows.sh",
    "tools/sim/open_sim_arm_test_windows.sh",
    "tools/run_agent_dashboard_sim_clean.sh",
])
def test_all_sim_shell_entries_pin_rmw(relative):
    text = (ROOT / relative).read_text()
    assert "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in text
    assert 'RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-' not in text
    subprocess.run(["bash", "-n", str(ROOT / relative)], check=True)


@pytest.mark.parametrize("relative", [
    "tools/sim/open_sim_nav2_windows.sh", "tools/sim/open_sim_arm_test_windows.sh",
])
def test_no_global_process_kill_and_ros_preflight_required(relative):
    text = (ROOT / relative).read_text()
    assert "pkill" not in text
    assert "sim_startup_preflight.py" in text and "--starting ros" in text


def test_quickstart_ignores_saved_legacy_cleanup(monkeypatch):
    import importlib
    monkeypatch.syspath_prepend(str(ROOT / "tools"))
    launcher = importlib.import_module("sim_robot_quick_start")
    settings = dict(launcher.DEFAULTS, cleanup=True)
    for mode in ["nav2", "arm_test"]:
        settings["ros_mode"] = mode
        command = launcher.LauncherWindow._ros_command(None, settings)
        assert "--cleanup" not in command
    assert launcher.DEFAULTS["cleanup"] is False
