from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_agent.recovery.primitives import matches_node_process, same_domain_pids


@pytest.mark.parametrize("argv,expected", [
    (["/usr/bin/python3", "/ws/install/robot_sim/lib/robot_sim/sim_lift_command_node", "--ros-args"], True),
    (["/bin/bash", "-lc", "ros2 run robot_sim sim_lift_command_node & wait"], False),
    (["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "run", "robot_sim", "sim_lift_command_node"], False),
    (["/usr/bin/python3", "/ws/tools/recovery_terminal_runner.py", "--", "ros2", "run", "robot_sim", "sim_lift_command_node"], False),
    (["gnome-terminal", "--title=sim_lift_command_node"], False),
    (["python3", "-c", "print('sim_lift_command_node')"], False),
    (["/usr/bin/python3", "/ws/unrelated.py", "--topic", "sim_lift_command_node"], False),
    ([], False),
])
def test_matches_executable_not_embedded_command(argv, expected):
    assert matches_node_process(argv, "sim_lift_command_node") is expected


def test_container_remap_is_supported_without_matching_shell_payload():
    pattern = "__node:=depth_proc_container"
    assert matches_node_process(["/opt/ros/humble/lib/rclcpp_components/component_container", "-r", pattern], pattern)
    assert not matches_node_process(["bash", "-c", pattern], pattern)


def test_domain_filter_and_parent_script_are_both_checked(monkeypatch):
    from robot_agent.recovery import primitives
    monkeypatch.setenv("ROS_DOMAIN_ID", "20")
    monkeypatch.setattr(primitives.subprocess, "run", lambda *a, **k: SimpleNamespace(stdout="101 102 103"))
    def read_bytes(path):
        pid = int(path.parts[2])
        if path.name == "environ":
            return b"ROS_DOMAIN_ID=21\0" if pid == 103 else b"ROS_DOMAIN_ID=20\0"
        return (b"bash\0-lc\0ros2 run robot_sim sim_lift_command_node\0" if pid == 101 else
                b"python3\0/ws/lib/robot_sim/sim_lift_command_node\0")
    monkeypatch.setattr(Path, "read_bytes", read_bytes)
    assert same_domain_pids("sim_lift_command_node") == [102]
