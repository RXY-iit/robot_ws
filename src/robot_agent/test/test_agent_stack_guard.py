import importlib.util
from pathlib import Path
import pytest


spec = importlib.util.spec_from_file_location("agent_stack_guard", Path(__file__).resolve().parents[3] / "tools/agent_stack_guard.py")
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


def test_python_orphan_same_workspace_and_domain(tmp_path):
    proc = tmp_path / "proc"
    proc.mkdir()
    for pid, domain, workspace in [(101, "20", "/ws"), (102, "21", "/ws"), (103, "20", "/other")]:
        entry = proc / str(pid)
        entry.mkdir()
        (entry / "cmdline").write_bytes(f"python3\0{workspace}/install/robot_agent/lib/robot_agent/agent_node\0".encode())
        (entry / "environ").write_bytes(f"ROS_DOMAIN_ID={domain}\0".encode())
    assert guard.find_existing("/ws", "20", proc) == [(101, "agent_node")]


def test_starting_launch_is_detected_before_nodes_exist(tmp_path):
    entry = tmp_path / "101"
    entry.mkdir()
    (entry / "cmdline").write_bytes(b"/usr/bin/python3\0/opt/ros/humble/bin/ros2\0launch\0robot_agent\0agent_dashboard.launch.py\0")
    (entry / "environ").write_bytes(b"ROS_DOMAIN_ID=20\0")
    (entry / "cwd").symlink_to("/ws")
    assert guard.find_existing("/ws", "20", tmp_path) == [(101, "agent_dashboard.launch.py (starting/running)")]


def test_launch_lock_is_exclusive_and_released(tmp_path):
    with guard.acquire_launch_lock("/ws", "20", tmp_path):
        with pytest.raises(RuntimeError, match="already starting"):
            guard.acquire_launch_lock("/ws", "20", tmp_path)
        with guard.acquire_launch_lock("/ws", "21", tmp_path):
            pass
    with guard.acquire_launch_lock("/ws", "20", tmp_path):
        pass


def test_lock_survives_exec_until_launch_exits(tmp_path):
    import os
    import subprocess
    import sys
    # Use a tiny child instead of starting any ROS nodes.
    with guard.acquire_launch_lock("/ws", "20", tmp_path) as lock:
        child = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(30)"], pass_fds=(lock.fileno(),))
    try:
        with pytest.raises(RuntimeError):
            guard.acquire_launch_lock("/ws", "20", tmp_path)
    finally:
        child.terminate()
        child.wait(timeout=5)
    with guard.acquire_launch_lock("/ws", "20", tmp_path):
        pass
