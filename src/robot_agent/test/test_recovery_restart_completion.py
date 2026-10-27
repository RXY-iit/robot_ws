from types import SimpleNamespace

import pytest

from robot_agent.recovery.backends import RosActionBackend
from robot_agent.recovery.environment import load_env_profile
from robot_agent.recovery.executor import DONE, RUNNING
from test_recovery_engine import ALL_SIM_NODES, CONFIG_DIR, FakeGraph, make_engine, run, snap
from test_recovery_executor import FakeBackend, feed, make
from robot_agent.recovery.snapshot import HealthSnapshot
from test_recovery_engine import make_state


def test_start_then_node_present_does_not_abort_before_first_output():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_lift_command_node"})
    engine, backend = make(graph=graph, backend=FakeBackend(results={"restart_component": (RUNNING, "starting")}))
    missing = {"sim_lift_joint_targets": "STALE", "lift_position": "STALE"}
    feed(engine, 0, missing)
    feed(engine, 4, missing)
    ex = engine.executor.active["sim_lift"]
    graph.nodes.add("/sim_lift_command_node")
    engine.system.processes.add("sim_lift_command_node")
    feed(engine, 5, missing)
    assert engine.executor.active["sim_lift"] is ex
    assert ex.state == "RUNNING_ACTION"
    ex.handle.result = (DONE, "spawned")
    feed(engine, 6, missing)
    assert ex.state == "VERIFYING"
    feed(engine, 7, missing)
    assert ex.state == "VERIFYING"
    feed(engine, 8, {"sim_lift_joint_targets": "OK", "lift_position": "OBSERVED"})
    assert ex.state == "SUCCEEDED" and ex.message == "Verified"
    assert len(backend.calls) == 1


def test_waiting_recovery_does_not_lock_but_two_approvals_cannot_dispatch_together():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_lift_command_node", "/sim_omni_drive_command_node"})
    from test_recovery_executor import DRIVE_DOWN
    engine, backend = make(graph=graph, restart_confirmation=True,
                           backend=FakeBackend(results={"restart_component": (RUNNING, "starting")}))
    missing = {**DRIVE_DOWN, "sim_lift_joint_targets": "STALE", "lift_position": "STALE"}
    feed(engine, 0, missing)
    feed(engine, 4, missing)
    lift = engine.executor.active["sim_lift"]
    drive = engine.executor.active["sim_drive"]
    assert lift.state == drive.state == "WAITING_CONFIRM"
    lift.state = "WAITING_PRECONDITIONS"
    assert engine.executor._actuator_lock_reason() is None
    lift.state = "WAITING_CONFIRM"
    for execution in (lift, drive):
        engine.handle_command({"type": "operator_reply", "request_id": execution.request.id, "reply": "approve"}, 4)
    assert len(backend.calls) == 1
    assert lift.state == "RUNNING_ACTION"
    assert drive.state == "WAITING_CONFIRM"
    assert "actuator lock" in drive.request.note
    assert drive.request.reply is None


def test_restart_with_no_output_remains_bounded_and_can_be_aborted():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_lift_command_node"})
    engine, _ = make(graph=graph)
    missing = {"sim_lift_joint_targets": "STALE", "lift_position": "STALE"}
    feed(engine, 0, missing)
    feed(engine, 4, missing)
    ex = engine.executor.active["sim_lift"]
    graph.nodes.add("/sim_lift_command_node")
    engine.system.processes.add("sim_lift_command_node")
    feed(engine, 25, missing)
    assert any(event["event"] == "retry" for event in ex.log)
    engine.handle_command({"type": "abort", "incident_key": "sim_lift"}, 25)
    assert ex.state == "ABORTED"
    assert engine.state()["incidents"][0]["last_execution"]["state"] == "ABORTED"


def test_missing_adapter_subsumes_localization_without_requesting_initial_pose():
    engine = make_engine(graph=FakeGraph(nodes=ALL_SIM_NODES))
    missing = {"sim_lidar_adapter_cloud": "STALE", "localization_input_cloud": "STALE",
               "gicp_pose": "STALE", "gicp_score": "STALE", "fast_lio_odom": "STALE"}
    run(engine, missing, times=(0, 6))
    assert set(engine.incidents) == {"sim_lidar_adapter"}
    d = engine.incidents["sim_lidar_adapter"].diagnosis
    assert d.playbook_id == "PB-NODE-DOWN"
    assert d.plan["steps"][0]["args"]["component"] == "sim_lidar_adapter"
    assert "gicp_pose" in d.candidate.consequences


def test_low_rtf_is_degraded_startup_not_environment_offline():
    engine = make_engine()
    engine.on_snapshot(snap(0, {"sim_rtf": "WARN"}))
    assert engine.state()["environment"]["state"] == "RUNNING"
    assert engine.state()["startup_units"][0]["status"] == "DEGRADED"


def test_natural_localization_recovery_revokes_unexecuted_seed_approval():
    engine, backend = make(profile="nav2", restart_confirmation=True)
    feed(engine, 0, {"gicp_score": "DEGRADED"})
    feed(engine, 6, {"gicp_score": "DEGRADED"})
    ex = engine.executor.active["gicp"]
    assert ex.state == "WAITING_CONFIRM"
    request_id = ex.request.id
    assert ex.evidence_after is None
    feed(engine, 8)
    assert "gicp" not in engine.executor.active
    assert request_id not in engine.executor.requests
    assert ex.message == "Incident resolved by health evidence"
    assert not any(primitive == "publish_initialpose" for primitive, _ in backend.calls)
    assert "unknown request" in engine.executor.operator_reply(request_id, "approve", 9)


def test_lifecycle_reset_requires_fresh_all_member_verification():
    engine, backend = make(backend=FakeBackend(results={"lifecycle_reset": (RUNNING, "resetting")}))
    fault = {"lifecycle_controller_server": "DEGRADED"}
    feed(engine, 0, fault)
    feed(engine, 4, fault)
    ex = engine.executor.active["nav2"]
    assert ex.playbook.max_action_risk == "A2"
    ex.handle.result = (DONE, "started")
    state = make_state(values={"robot_mode": "MANUAL"})
    for item in state['domains'][0]['items']:
        if item['id'].startswith('lifecycle_'):
            item['receive_age_sec'] = 4
    engine.on_snapshot(HealthSnapshot.from_state(state, received_at=5))
    assert ex.state == "VERIFYING", "Old healthy data must not complete the reset"
    feed(engine, 6)
    assert ex.state == "SUCCEEDED" and ex.message == "Verified"
    assert {v.get('args', {}).get('item') for v in ex.playbook.verify} >= {
        'lifecycle_controller_server', 'lifecycle_planner_server', 'lifecycle_bt_navigator', 'lifecycle_local_costmap'}


def test_lifecycle_reset_failure_never_dispatches_startup():
    backend = RosActionBackend.__new__(RosActionBackend)
    calls = []
    def call(_type, _service, request, _timeout):
        calls.append(request.command)
        return SimpleNamespace(success=False)
    backend._call = call
    with pytest.raises(RuntimeError, match='RESET returned success=false'):
        backend._lifecycle_reset('/lifecycle_manager_navigation', lambda _: None)
    assert calls == [3]


def test_lifecycle_approval_does_not_claim_to_open_terminal():
    engine, _ = make(restart_confirmation=True)
    fault = {"lifecycle_controller_server": "DEGRADED"}
    feed(engine, 0, fault)
    feed(engine, 4, fault)
    request = engine.executor.active['nav2'].request
    assert request.action_context['opens_terminal'] is False
    assert 'RESET then STARTUP' in request.action_context['scope']


def test_fresh_monitor_preserves_unknown_upstream_evidence_of_failed_capability():
    engine = make_engine(profile="nav2")
    state = make_state({"sim_lidar_adapter_cloud": "UNKNOWN", "localization_input_cloud": "UNKNOWN",
                        "gicp_pose": "UNKNOWN", "gicp_score": "UNKNOWN"})
    for item in state["domains"][0]["items"]:
        if item["id"] in {"sim_lidar_adapter_cloud", "localization_input_cloud"}:
            item["critical"] = False
    for t in (0, 6):
        engine.on_snapshot(HealthSnapshot.from_state(state, received_at=t))
    assert set(engine.incidents) == {"sim_lidar_adapter"}
    assert engine.incidents["sim_lidar_adapter"].diagnosis.playbook_id == "PB-NODE-DOWN"


@pytest.mark.parametrize("collision", ["node", "process"])
def test_partial_nav_group_rejected_before_any_spawn(tmp_path, monkeypatch, collision):
    from robot_agent.recovery import backends
    backend = RosActionBackend.__new__(RosActionBackend)
    backend.env = load_env_profile(CONFIG_DIR, "sim")
    backend.workspace, backend.map_ply, backend.log_dir = str(tmp_path), "", tmp_path
    backend.node = SimpleNamespace(get_node_names_and_namespaces=lambda: [("planner_server", "/")] if collision == "node" else [])
    monkeypatch.setattr(backends, "same_domain_pids", lambda _: [12] if collision == "process" else [])
    monkeypatch.setattr(backends.subprocess, "Popen", lambda *a, **k: pytest.fail("duplicate spawn"))
    with pytest.raises(RuntimeError, match="group members already present"):
        backend._restart("nav2", lambda _: None)


def test_nav_approval_includes_scope_and_verifies_all_lifecycle_outputs():
    class Backend(FakeBackend):
        def supports(self, primitive, args):
            return True, ""
    engine, backend = make(graph=FakeGraph(nodes=ALL_SIM_NODES - {"/bt_navigator"}),
                           backend=Backend(), restart_confirmation=True)
    missing = {"lifecycle_bt_navigator": "STALE"}
    feed(engine, 0, missing)
    feed(engine, 4, missing)
    ex = engine.executor.active["nav2"]
    request = ex.request
    assert request.kind == "confirm_action" and not backend.calls
    assert "start_mode_switch:=false" in request.action_context["command_template"]
    assert "start_safety_node:=false" in request.action_context["command_template"]
    assert "mode_manual" in request.action_context["preconditions"]
    assert {v.get("args", {}).get("item") for v in request.done_when} >= {
        "lifecycle_controller_server", "lifecycle_planner_server", "lifecycle_bt_navigator", "lifecycle_local_costmap"}


def test_only_controller_missing_is_identified_even_with_bt_navigator_alive():
    engine = make_engine(graph=FakeGraph(nodes=ALL_SIM_NODES - {"/controller_server"}))
    run(engine, {"lifecycle_controller_server": "STALE"}, times=(0, 6))
    diagnosis = engine.incidents["nav2"].diagnosis
    assert diagnosis.playbook_id == "PB-NODE-DOWN"
    assert diagnosis.steps[-1]["value"]["missing"] == ["/controller_server"]


def test_duplicate_nodes_are_not_reported_as_success():
    graph = FakeGraph()
    graph.node_counts = lambda: {name: 2 if name == "/controller_server" else 1 for name in graph.nodes}
    engine = make_engine(graph=graph)
    engine.on_snapshot(snap(0))
    result = engine.registry.run_probe("node_alive", engine._ctx(engine.last_snapshot), {"component": "nav2"})
    assert result.outcome == "UNKNOWN" and "Duplicate" in result.detail


def test_quickstart_snapshot_overrides_default_motion_policy(monkeypatch):
    import json
    monkeypatch.setenv("ROS_DOMAIN_ID", "20")
    monkeypatch.setenv("RECOVERY_PROFILE", "nav2")
    monkeypatch.setenv("RECOVERY_LAUNCH_SETTINGS", json.dumps({"ros_domain_id": 20, "ros_mode": "nav2", "motion_policy": False}))
    env = load_env_profile(CONFIG_DIR, "sim")
    assert "enable_motion_policy:=false" in env.components["mode_switch"].restart["command"]
    assert env.actions["launch_context"]["source"] == "Quick Start launch snapshot"
    monkeypatch.setenv("ROS_DOMAIN_ID", "21")
    env = load_env_profile(CONFIG_DIR, "sim")
    assert "enable_motion_policy:=true" in env.components["mode_switch"].restart["command"]


def test_sim_restart_dispatches_visible_terminal_with_explicit_domain(tmp_path, monkeypatch):
    from robot_agent.recovery import backends
    backend = RosActionBackend.__new__(RosActionBackend)
    backend.env = load_env_profile(CONFIG_DIR, "sim")
    backend.config = backend.env.actions
    backend.workspace, backend.map_ply, backend.log_dir = str(tmp_path), "", tmp_path
    backend.node = SimpleNamespace(get_node_names_and_namespaces=lambda: [])
    monkeypatch.setenv("DISPLAY", ":1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "20")
    monkeypatch.setattr(backends.shutil, "which", lambda _: "/usr/bin/gnome-terminal")
    monkeypatch.setattr(backends, "same_domain_pids", lambda _: [])
    monkeypatch.setattr(backends.time, "sleep", lambda _: None)
    calls = []
    monkeypatch.setattr(backends.subprocess, "Popen", lambda command, **kwargs:
                        (calls.append(command) or SimpleNamespace(pid=123, poll=lambda: None)))
    backend._restart("sim_lidar_adapter", lambda _: None)
    assert calls[0][:2] == ["gnome-terminal", "--wait"]
    assert calls[0][calls[0].index("--domain") + 1] == "20"
    assert "pointcloud2_to_livox_custom_msg_node" in calls[0]


def test_fast_lio_reset_requires_explicit_fresh_flag_and_healthy_inputs():
    engine = make_engine()
    run(engine, {"fast_lio_timing": "ERROR", "fast_lio_odom": "STALE"}, times=(0, 6))
    assert engine.incidents["fast_lio"].diagnosis.playbook_id == "PB-FAST-LIO-RESET"
    engine = make_engine()
    run(engine, {"fast_lio_timing": "ERROR", "fast_lio_odom": "STALE", "sim_lidar_adapter_cloud": "STALE"}, times=(0, 6))
    assert "fast_lio" not in engine.incidents
