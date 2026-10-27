import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools/sim/run_b1_tor_waypoints.py"
spec = importlib.util.spec_from_file_location("b1_tor", SCRIPT)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def latest(status="AVAILABLE"):
    return {"clock": (10, 1), "clock_advancing": True, "robot_mode": (10, "AUTO"),
            "safety_status": (10, "OK"), "health": (10, {"capabilities": [
                {"id": "auto_navigation", "status": status}]})}


def test_baseline_allows_read_only_health_but_rejects_agent_layer():
    assert not runner.dispatch_blockers(["health_monitor", "bt_navigator"], latest(), 10)
    assert runner.dispatch_blockers(["agent_node"], latest(), 10)
    assert runner.dispatch_blockers(["/namespace/recovery_agent"], latest(), 10)


@pytest.mark.parametrize("status", ["BLOCKED", "UNKNOWN", "SUPERVISED_ONLY"])
def test_bad_pre_dispatch_navigation_never_starts(status):
    assert runner.dispatch_blockers([], latest(status), 10)


def test_stale_preconditions_refuse_dispatch():
    assert runner.dispatch_blockers([], latest(), 14)


@pytest.mark.parametrize("content", ["frame_id: odom\nwaypoints: [{x: 1, y: 0, yaw: 0}]",
    "frame_id: map\nwaypoints: []", "frame_id: map\nwaypoints: [{x: .nan, y: 0, yaw: 0}]",
    "frame_id: map\nwaypoints: [{x: true, y: 0, yaw: 0}]"])
def test_unfrozen_or_invalid_route_is_rejected(tmp_path, content):
    route = tmp_path / "route.yaml"
    route.write_text(content)
    with pytest.raises(ValueError):
        runner.load_route(route)


def test_default_is_route_validation_without_ros_or_motion(tmp_path):
    route = tmp_path / "route.yaml"
    route.write_text("frame_id: map\nwaypoints: [{x: 1, y: 2, yaw: 0}]")
    result = subprocess.run([sys.executable, str(SCRIPT), "--route", str(route)],
                            capture_output=True, text=True, check=True)
    assert '"execute": false' in result.stdout
    assert runner.load_route(route) == [{"x": 1, "y": 2, "yaw": 0}]


def test_idle_watchdog_matches_cor_gate_but_stop_states_block():
    evidence=latest();evidence['safety_status']=(10,'WATCHDOG')
    assert not runner.dispatch_blockers([],evidence,10)
    for status in ['ESTOP','MANUAL_STOP','TOUCH_LOCK','UNKNOWN']:
        evidence['safety_status']=(10,status)
        assert runner.dispatch_blockers([],evidence,10)


def test_route_task_does_not_reintroduce_health_gates_between_waypoints():
    evidence=latest('BLOCKED')
    assert not runner.dispatch_blockers([],evidence,10,check_navigation=False)
    del evidence['health']
    assert not runner.dispatch_blockers([],evidence,10,check_navigation=False)
    assert runner.dispatch_blockers([],evidence,10,check_navigation=True)
    evidence['safety_status']=(10,'ESTOP')
    assert runner.dispatch_blockers([],evidence,10,check_navigation=False)
    evidence['safety_status']=(10,'OK')
    assert runner.dispatch_blockers(['recovery_agent'],evidence,10,check_navigation=False)
