import asyncio
import json
import os
import threading
from types import SimpleNamespace

import pytest

from robot_agent import stack_lifecycle
from robot_agent.nodes.health_monitor import HealthMonitor, SignalObservation
from robot_agent.recovery.snapshot import HealthSnapshot, classify_status
from robot_skills.executors.system import SystemSkills
from test_recovery_engine import make_engine, make_state, run
from test_recovery_executor import make, feed, DRIVE_DOWN, FakeBackend, FakeHandle


@pytest.mark.parametrize("clock_age", [0, 60000])
def test_silence_never_downgraded_to_source_warning(monkeypatch, clock_age):
    now = 60100
    signal = SignalObservation("/fast_lio/odometry", "DIRECT_TOPIC")
    signal.update({}, now_monotonic=100, source_stamp_sec=90)
    clock = SignalObservation("/clock", "DIRECT_TOPIC")
    clock.update({"sim_time_sec": 100}, now_monotonic=now - clock_age)
    monitor = SimpleNamespace(_mode="sim", _signals_lock=threading.RLock(),
                              signals={"fast_lio_odom": signal, "sim_clock": clock},
                              get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    monkeypatch.setattr("robot_agent.nodes.health_monitor.time.monotonic", lambda: now)
    result = HealthMonitor._signal_item(monitor, "fast_lio_odom", "FAST-LIO", stale_sec=1, standard="test")
    assert result["status"] == "STALE"
    assert result["receive_age_sec"] == 60000
    assert result["source_age_sec"] == (None if clock_age else 10)


def test_legacy_mode_and_safety_expire_and_recover(monkeypatch):
    now = [100.]
    monkeypatch.setattr("robot_skills.executors.system.time.monotonic", lambda: now[0])
    system = SystemSkills(None)
    system.update_robot_mode("AUTO")
    system.update_safety_status("WATCHDOG")
    assert system.health_summary()["components"]["mode"]["level"] == "OK"
    now[0] += 60000
    for name in ("mode", "safety"):
        item = system.health_summary()["components"][name]
        assert item["summary"] == "STALE" and item["level"] == "BAD"
    system.update_robot_mode("AUTO")
    system.update_safety_status("OK")
    assert system.health_summary()["components"]["safety"]["level"] == "OK"


def test_unknown_status_is_not_healthy():
    assert classify_status("TYPO_OK") == "UNCERTAIN"


def test_upstream_subsumes_old_diagnosis_without_false_success():
    engine = make_engine()
    run(engine, {"camera_color_diagnostics": "STALE"})
    assert "isaac_camera" in engine.incidents
    engine._pending_requests["mission-request"] = {"keys": ["isaac_camera"], "mission_id": "m"}
    engine.system.simulation_evidence = lambda now: {"process_alive": False}
    run(engine, {"camera_color_diagnostics": "STALE", "sim_clock": "STALE", "sim_rtf": "WARN"},
        {"sim_rtf": {"rtf": None, "clock_receive_age_sec": 100, "not_advancing_sec": 100}}, times=(10, 14))
    assert "isaac" not in engine.incidents
    assert "isaac_camera" not in engine.incidents
    assert "mission-request" not in engine._pending_requests
    assert engine.drain_responses()[0]["status"] == "NEEDS_HUMAN"
    assert any(i.key == "isaac_camera" and i.status == "ENVIRONMENT_SUSPENDED" for i in engine.closed)
    run(engine, {"camera_color_diagnostics": "STALE"}, times=(20, 24))
    assert engine.incidents["isaac_camera"].status == "NEEDS_INVESTIGATION"


@pytest.mark.parametrize("rtf,advance,confirmed", [(None, 500, False), (.45, .01, True), (0, 500, False)])
def test_rtf_requires_measurable_advancing_clock(rtf, advance, confirmed):
    engine = make_engine()
    run(engine, {"sim_rtf": "WARN"}, {"sim_rtf": {"rtf": rtf, "not_advancing_sec": advance,
          "clock_receive_age_sec": .01}}, times=(0, 15))
    if confirmed:
        assert engine.incidents["simulation_performance"].status == "CONFIRMED"
    else:
        assert "simulation_performance" not in engine.incidents
        assert not engine.environment_state["recovery_allowed"]


def test_lidar_fault_without_clock_support_is_not_confirmed():
    engine = make_engine()
    run(engine, {"lidar_device_status": "STALE"}, {"sim_rtf": None})
    assert "isaac_lidar" not in engine.incidents
    assert engine.environment_state["state"] == "UNKNOWN"


def test_sim_lift_does_not_match_arm_link_playbook():
    engine = make_engine()
    run(engine, {"sim_lift_joint_targets": "STALE"})
    assert all(i.diagnosis.playbook_id != "PB-ARM-LINK-LOST" for i in engine.incidents.values())


def test_sim_arm_link_requires_injected_provenance():
    engine = make_engine()
    run(engine, {"so101_arm_hardware_status": "STALE"})
    assert not any(i.status == "CONFIRMED" and i.diagnosis.playbook_id == "PB-ARM-LINK-LOST"
                   for i in engine.incidents.values())


def test_shutdown_disables_retry_and_waits_for_backend():
    backend = FakeBackend(results={"restart_component": ("RUNNING", "still running")})
    engine, _ = make(backend=backend)
    feed(engine, 0, DRIVE_DOWN)
    feed(engine, 4, DRIVE_DOWN)
    feed(engine, 5, DRIVE_DOWN)
    handles = [e.handle for e in engine.executor.active.values() if e.handle]
    assert handles
    engine.prepare_shutdown(6)
    assert engine.lifecycle == "STOPPING"
    assert engine.shutdown_ready()[0] is False
    count = len(backend.calls)
    engine.handle_command({"type": "retry", "incident_key": "sim_drive"}, 7)
    feed(engine, 8, DRIVE_DOWN)
    assert len(backend.calls) == count
    for handle in handles:
        handle.result = ("DONE", "settled")
    assert engine.shutdown_ready()[0] is True
    assert engine.lifecycle == "INTENTIONALLY_OFFLINE"
    assert engine.handle_request({"request_id": "x"})["status"] == "CANCELLED"


def fake_parent(tmp_path):
    entry = tmp_path / "123"
    entry.mkdir()
    fields = ["S", "1"] + ["0"] * 17 + ["42"]
    (entry / "stat").write_text("123 (ros2 launch) " + " ".join(fields))
    (entry / "cmdline").write_bytes(b"python3\0ros2\0launch\0robot_agent\0agent_dashboard.launch.py\0")
    return {"ROBOT_AGENT_LAUNCH_PID": "123", "ROBOT_AGENT_LAUNCH_START": "42"}


def test_stop_targets_exact_launch_identity_only(tmp_path):
    env = fake_parent(tmp_path)
    assert stack_lifecycle.launch_parent(env, tmp_path, 123) == 123
    for bad in ({**env, "ROBOT_AGENT_LAUNCH_START": "43"}, {**env, "ROBOT_AGENT_LAUNCH_PID": "1"}):
        with pytest.raises(RuntimeError):
            stack_lifecycle.launch_parent(bad, tmp_path, 123)
    with pytest.raises(RuntimeError):
        stack_lifecycle.launch_parent(env, tmp_path, 456)
    (tmp_path / "123" / "cmdline").write_bytes(b"isaac-sim\0")
    with pytest.raises(RuntimeError):
        stack_lifecycle.launch_parent(env, tmp_path, 123)


def test_participant_rejects_wrong_session_and_prepares_once():
    participant = object.__new__(stack_lifecycle.ShutdownParticipant)
    participant.session = "current"
    participant.request = None
    participant.stopping = False
    calls = []
    participant.prepare = lambda: calls.append("prepare")
    request = {"session_id": "old", "action": "prepare_stop", "request_id": "one"}
    participant._receive(SimpleNamespace(data=json.dumps(request)))
    assert not calls
    request["session_id"] = "current"
    for _ in range(3):
        participant._receive(SimpleNamespace(data=json.dumps(request)))
    assert calls == ["prepare"]
