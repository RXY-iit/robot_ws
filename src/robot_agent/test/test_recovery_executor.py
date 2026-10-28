from pathlib import Path

from robot_agent.recovery.engine import RecoveryEngine
from robot_agent.recovery.environment import load_env_profile
from robot_agent.recovery.executor import DONE, ERROR
from robot_agent.recovery.playbook import load_playbooks
from robot_agent.recovery.primitives import build_default_registry
from robot_agent.recovery.snapshot import HealthSnapshot

from test_recovery_engine import ALL_SIM_NODES, CONFIG_DIR, FakeGraph, FakeSystem, make_state


IDLE_VALUES = {"robot_mode": "MANUAL", "cmd_vel": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}}
DRIVE_DOWN = {k: "STALE" for k in ("drive_health", "drive_feedback", "steer_feedback", "dynamixel_health")}


class FakeHandle:
    def __init__(self, result):
        self.result = result

    def poll(self):
        return self.result


class FakeBackend:
    def __init__(self, supported=None, results=None):
        self.supported = supported if supported is not None else {"restart_component", "cancel_motion", "publish_initialpose", "clear_costmap", "lifecycle_reset"}
        self.results = results or {}
        self.calls = []

    def supports(self, primitive, args):
        if primitive == "restart_component" and args.get("component") not in {"sim_drive", "sim_lift"}:
            return False, f"no restart for {args.get('component')}"
        if primitive == "publish_initialpose" and args.get("source") != "last_trusted_pose":
            return False, "not implemented"
        return (primitive in self.supported), "unsupported in test"

    def start(self, primitive, args):
        self.calls.append((primitive, args))
        return FakeHandle(self.results.get(primitive, (DONE, "ok")))


def make(graph=None, backend=None, profile=None, ack_store=None, approval=None, restart_confirmation=False):
    env = load_env_profile(CONFIG_DIR, "sim")
    # Legacy policy tests explicitly exercise auto execution. Production sim
    # requires confirmation; approval tests opt in with restart_confirmation.
    env.actions["restart_requires_approval"] = restart_confirmation
    if not restart_confirmation:
        env.approval.update(A1="auto", A2="auto", A3="auto")
    if approval:
        env.approval.update(approval)
    registry = build_default_registry()
    library = load_playbooks(CONFIG_DIR / "playbooks", env, registry)
    backend = backend or FakeBackend()
    engine = RecoveryEngine(
        env,
        library,
        registry,
        graph=graph or FakeGraph(),
        system=FakeSystem(),
        launch_profile=env.launch_profiles[profile] if profile else None,
        execution_enabled=True,
        backend=backend,
        ack_store=ack_store,
    )
    return engine, backend


def feed(engine, t, overrides=None, values=None):
    vals = {**IDLE_VALUES, "localization_consistency": {"components": {"localization": {"details": {"reference_verified": True}}}}, **(values or {})}
    engine.on_snapshot(HealthSnapshot.from_state(make_state(overrides, vals), received_at=t))


def events(engine):
    return [e["type"] for e in engine.drain_events()]


def test_drive_restart_runs_and_verifies():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph)
    feed(engine, 0.0, DRIVE_DOWN)
    feed(engine, 4.0, DRIVE_DOWN)
    assert backend.calls == [("restart_component", {"component": "sim_drive"})]
    ex = engine.executor.active["sim_drive"]
    assert ex.state == "VERIFYING"
    graph.nodes.add("/sim_omni_drive_command_node")
    feed(engine, 6.0)  # healthy -> verify passes
    assert "sim_drive" not in engine.executor.active
    assert engine.executor.finished[-1].state == "SUCCEEDED"
    assert "execution_succeeded" in events(engine)


def test_unmet_precondition_waits_then_asks_operator():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph)
    auto = {"robot_mode": "AUTO"}
    feed(engine, 0.0, DRIVE_DOWN, auto)
    feed(engine, 4.0, DRIVE_DOWN, auto)
    assert backend.calls == []
    assert engine.executor.active["sim_drive"].state == "WAITING_PRECONDITIONS"
    feed(engine, 16.0, DRIVE_DOWN, auto)
    requests = list(engine.executor.requests.values())
    assert requests and requests[0].kind == "precondition"
    assert requests[0].last_check[0]["name"] == "mode_manual"
    feed(engine, 18.0, DRIVE_DOWN)  # operator switched to MANUAL
    assert backend.calls == [("restart_component", {"component": "sim_drive"})]
    assert engine.executor.requests == {}


def test_unsupported_restart_falls_back_to_manual_request_and_click_is_not_evidence():
    # nav2 has no restart command in sim -> operator is asked to do it.
    lifecycle_unknown = {k: "UNKNOWN" for k in (
        "lifecycle_controller_server", "lifecycle_planner_server", "lifecycle_bt_navigator", "lifecycle_local_costmap")}
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/bt_navigator"})
    engine, backend = make(graph=graph, profile="nav2")
    feed(engine, 0.0, lifecycle_unknown)
    feed(engine, 4.0, lifecycle_unknown)
    request = next(iter(engine.executor.requests.values()))
    assert request.kind == "manual_fallback"
    assert backend.calls == []
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "done"}, 5.0)
    assert request.note.startswith("Not detected yet")
    assert request.id in engine.executor.requests
    graph.nodes.add("/bt_navigator")
    feed(engine, 6.0)
    feed(engine, 17.0)  # incident resolves after hysteresis
    assert engine.executor.finished[-1].state == "SUCCEEDED"


def test_operator_step_cannot_escalates():
    graph = FakeGraph(publishers={"/livox/lidar_pc2_from_sim": 0})
    engine, _ = make(graph=graph, profile="nav2")
    feed(engine, 0.0, {"lidar_device_status": "STALE"})
    feed(engine, 4.0, {"lidar_device_status": "STALE"})
    request = next(iter(engine.executor.requests.values()))
    assert request.kind == "operator_action"
    assert "LiDAR" in request.text["en"]
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "cannot"}, 5.0)
    assert engine.executor.finished[-1].state == "ESCALATED"


def test_operator_step_completes_on_evidence_without_click():
    graph = FakeGraph(publishers={"/livox/lidar_pc2_from_sim": 0})
    engine, _ = make(graph=graph, profile="nav2")
    feed(engine, 0.0, {"lidar_device_status": "STALE"})
    feed(engine, 4.0, {"lidar_device_status": "STALE"})
    feed(engine, 6.0)  # operator fixed it; done_when passes
    assert engine.executor.finished[-1].state == "SUCCEEDED"


def test_confirm_approval_gates_actuator_action():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph, approval={"A2": "confirm"})
    feed(engine, 0.0, DRIVE_DOWN)
    feed(engine, 4.0, DRIVE_DOWN)
    request = next(iter(engine.executor.requests.values()))
    assert request.kind == "confirm_action" and backend.calls == []
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "approve"}, 5.0)
    assert backend.calls == [("restart_component", {"component": "sim_drive"})]


def test_verify_failure_retries_then_fails_and_is_held_until_retry():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph)
    t = 0.0
    while t <= 60.0:  # restart "succeeds" but the node never appears
        feed(engine, t, DRIVE_DOWN)
        t += 2.0
    assert len(backend.calls) == 2  # attempt 1 + retry_budget 1
    assert engine.executor.finished[-1].state == "FAILED"
    # No automatic re-run after a failed playbook, however long we wait.
    while t <= 400.0:
        feed(engine, t, DRIVE_DOWN)
        t += 4.0
    assert len(backend.calls) == 2
    assert "held after FAILED" in engine.incidents["sim_drive"].execution_blocked
    engine.handle_command({"type": "retry", "incident_key": "sim_drive"}, t)
    assert len(backend.calls) == 3


def test_chronic_after_repeated_success_then_refault():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph)
    t = 0.0
    for _ in range(4):  # node comes back after each restart, then dies again
        for _ in range(3):
            feed(engine, t, DRIVE_DOWN)
            t += 2.0
        graph.nodes.add("/sim_omni_drive_command_node")
        for _ in range(8):
            feed(engine, t)
            t += 2.0
        graph.nodes.discard("/sim_omni_drive_command_node")
        t += 130.0  # past cooldown
    for _ in range(3):  # it breaks again: no 5th automatic run, operator is asked
        feed(engine, t, DRIVE_DOWN)
        t += 2.0
    assert "sim_drive" in engine.executor.state(t)["human_only"]
    assert "chronic" in [r.kind for r in engine.executor.requests.values()]
    assert len(backend.calls) == 3  # chronic.count = 3 runs per hour


def test_mission_request_gets_in_progress_then_final_resolved():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, _ = make(graph=graph)
    feed(engine, 0.0, DRIVE_DOWN)
    response = engine.handle_request(
        {"request_id": "rr-1", "mission_id": "m-1", "required_capabilities": ["manual_drive"]}
    )
    assert response["status"] == "IN_PROGRESS" and response["final"] is False
    assert engine.drain_responses() == []
    graph.nodes.add("/sim_omni_drive_command_node")
    feed(engine, 2.0)
    final = engine.drain_responses()
    assert len(final) == 1
    assert final[0]["request_id"] == "rr-1" and final[0]["final"] is True
    assert final[0]["status"] == "RESOLVED" and final[0]["advice"] == "resume_from_step"


def test_mission_aliases_map_to_health_capabilities():
    engine, _ = make()
    feed(engine, 0.0, values={"robot_mode": "MANUAL"})
    response = engine.handle_request({"request_id": "rr-2", "required_capabilities": ["relative_motion"]})
    assert response["root_cause"]["playbook"] == "PB-MODE-MANUAL"
    assert "auto_navigation" not in engine.env.normalize_capabilities(["relative_motion"])
    assert engine.env.normalize_capabilities(["relative_motion"]) == ["relative_motion", "manual_drive"]


def test_localization_chain_stops_after_first_successful_method():
    engine, backend = make(profile="nav2")
    feed(engine, 0.0, {"gicp_score": "DEGRADED"})
    feed(engine, 6.0, {"gicp_score": "DEGRADED"})
    assert [c[0] for c in backend.calls] == ["cancel_motion", "publish_initialpose"]
    feed(engine, 8.0)  # GICP and consistency healthy after trusted seed
    feed(engine, 9.0)
    assert engine.executor.finished[-1].state == "SUCCEEDED"
    assert all(c[1].get("source") != "semantic_landmark" for c in backend.calls)
    assert backend.calls[1][1] == {"source": "last_trusted_pose"}


def test_stationary_drift_triggers_recovery_despite_good_score():
    engine, backend = make(profile="nav2")
    feed(engine, 0, {"localization_consistency": "ERROR"})
    feed(engine, 6, {"localization_consistency": "ERROR"})
    assert backend.calls[-1] == ("publish_initialpose", {"source": "last_trusted_pose"})
    feed(engine, 20, {"localization_consistency": "UNKNOWN"})
    assert engine.executor.active["gicp"].state == "WAITING_SUCCESS"
    feed(engine, 30)
    assert engine.executor.finished[-1].state == "SUCCEEDED"


def test_trusted_pose_timeout_asks_manual_without_reseeding():
    engine, backend = make(profile="nav2")
    feed(engine, 0, {"gicp_score": "DEGRADED"})
    feed(engine, 6, {"gicp_score": "DEGRADED"})
    feed(engine, 107, {"gicp_score": "DEGRADED"})
    assert engine.executor.active["gicp"].state == "WAITING_OPERATOR"
    request = next(iter(engine.executor.requests.values()))
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "done"}, 108)
    assert request.note.startswith("Not detected yet")
    assert len([c for c in backend.calls if c[0] == "publish_initialpose"]) == 1


def test_retry_restarts_a_run_waiting_for_operator():
    engine, backend = make(profile="nav2")
    feed(engine, 0, {"gicp_score": "DEGRADED"})
    feed(engine, 6, {"gicp_score": "DEGRADED"})
    feed(engine, 107, {"gicp_score": "DEGRADED"})
    previous = engine.executor.active['gicp'].id
    old_requests = set(engine.executor.requests)
    engine.handle_command({'type': 'retry', 'incident_key': 'gicp'}, 108)
    feed(engine, 109, {"gicp_score": "DEGRADED"})
    assert engine.executor.active['gicp'].id != previous
    assert old_requests.isdisjoint(engine.executor.requests)
    assert len([c for c in backend.calls if c[0] == 'publish_initialpose']) == 2


def test_localization_cannot_verify_with_pre_seed_evidence():
    engine, _ = make(profile="nav2")
    feed(engine, 0, {"gicp_score": "DEGRADED"})
    feed(engine, 6, {"gicp_score": "DEGRADED"})
    state = make_state(values=IDLE_VALUES)
    for item in state["domains"][0]["items"]:
        if item["id"] in {"gicp_score", "gicp_pose", "localization_consistency"}:
            item["receive_age_sec"] = 4
    engine.on_snapshot(HealthSnapshot.from_state(state, received_at=8))
    assert engine.executor.active["gicp"].state == "WAITING_SUCCESS"
    for item in state["domains"][0]["items"]:
        if item["id"] in {"gicp_score", "gicp_pose", "localization_consistency"}:
            item["receive_age_sec"] = 11
    engine.on_snapshot(HealthSnapshot.from_state(state, received_at=15))
    assert engine.executor.active["gicp"].state == "WAITING_SUCCESS"
    feed(engine, 16)
    assert engine.executor.finished[-1].state == "SUCCEEDED"


def test_localization_chain_falls_through_to_operator():
    backend = FakeBackend(results={"publish_initialpose": (ERROR, "align failed")})
    engine, backend = make(backend=backend, profile="nav2")
    feed(engine, 0.0, {"gicp_score": "DEGRADED"})
    feed(engine, 6.0, {"gicp_score": "DEGRADED"})
    request = next(iter(engine.executor.requests.values()))
    assert request.kind == "operator_action" and request.step["template"] == "rviz_2d_pose_estimate"


def test_sim_localization_waits_longer_for_scan_acceptance():
    engine, _ = make(profile="nav2")
    feed(engine, 0.0, {"gicp_score": "DEGRADED"})
    feed(engine, 6.0, {"gicp_score": "DEGRADED"})
    feed(engine, 30.0, {"gicp_score": "DEGRADED"})
    assert engine.executor.active["gicp"].state == "WAITING_SUCCESS"
    assert not engine.executor.requests
    feed(engine, 35.0)
    assert engine.executor.finished[-1].state == "SUCCEEDED"


def test_healthy_snapshot_does_not_release_running_alignment():
    backend = FakeBackend(results={"publish_initialpose": ("RUNNING", "collecting scan")})
    engine, _ = make(profile="nav2", backend=backend)
    feed(engine, 0.0, {"gicp_score": "DEGRADED"})
    feed(engine, 6.0, {"gicp_score": "DEGRADED"})
    feed(engine, 8.0)
    feed(engine, 20.0)
    assert engine.executor.active["gicp"].state == "RUNNING_ACTION"
    assert "gicp" in engine.incidents
    engine.executor.active["gicp"].handle.result = (DONE, "aligned")
    feed(engine, 21.0)
    feed(engine, 22.0)
    assert engine.executor.finished[-1].state == "SUCCEEDED"


def test_acknowledge_blocks_execution_persists_and_clears_on_change(tmp_path: Path):
    store = tmp_path / "acks.json"
    engine, _ = make(profile="nav2", ack_store=store)
    feed(engine, 0.0, {"host_resources": "DEGRADED"})
    feed(engine, 12.0, {"host_resources": "DEGRADED"})
    assert engine.handle_command({"type": "ignore", "incident_key": "host", "note": "known"}, 13.0)["result"] == "ok"
    assert "host" not in engine.executor.active
    assert engine.incidents["host"].execution_blocked == "acknowledged"
    assert store.exists()
    # Reload: the acknowledgement survives a restart.
    engine2, _ = make(profile="nav2", ack_store=store)
    feed(engine2, 0.0, {"host_resources": "DEGRADED"})
    feed(engine2, 12.0, {"host_resources": "DEGRADED"})
    assert engine2.incidents["host"].acknowledged
    assert "host" not in engine2.executor.active
    feed(engine2, 14.0, {"host_resources": "ERROR"})
    assert not engine2.incidents["host"].acknowledged
    assert not engine2.state()["ignored_warnings"]


def test_never_started_without_profile_is_advisory():
    engine, _ = make()
    unknown = {"so101_arm_gateway_delivery": "UNKNOWN"}
    for t in range(0, 40, 4):
        feed(engine, float(t), unknown)
    incident = engine.incidents["arm_gateway"]
    assert incident.diagnosis.playbook_id == "PB-COMPONENT-NOT-STARTED"
    state = engine.state()["incidents"][0]
    assert state["severity"] == "advisory" and state["playbook_no"] == 52


def test_expected_component_never_started_with_profile_is_node_down():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/so101_arm_gateway"})
    engine, _ = make(graph=graph, profile="nav2")
    unknown = {"so101_arm_gateway_delivery": "UNKNOWN"}
    for t in range(0, 40, 4):
        feed(engine, float(t), unknown)
    assert engine.incidents["arm_gateway"].diagnosis.playbook_id == "PB-NODE-DOWN"


def test_dead_process_confirmed_even_while_graph_still_lists_node():
    # Live e2e 2026-10-06: after SIGKILL the DDS graph kept the node ~12 s.
    engine, backend = make(graph=FakeGraph())  # graph still has /sim_omni_drive_command_node
    feed(engine, 0.0, DRIVE_DOWN)
    feed(engine, 4.0, DRIVE_DOWN)
    assert engine.incidents["sim_drive"].diagnosis.playbook_id == "PB-DRIVE-PROC-DOWN"
    assert engine.incidents["sim_drive"].status == "CONFIRMED"
    assert backend.calls == [("restart_component", {"component": "sim_drive"})]


def test_operator_only_playbook_does_not_hold_actuator_lock():
    # Live e2e: PB-CAMERA-STALE (operator step = A4) blocked the drive restart.
    graph = FakeGraph(publishers={"/camera/camera/color/image_raw": 0})
    engine, backend = make(graph=graph)
    overrides = {**DRIVE_DOWN, "camera_color_diagnostics": "STALE"}
    feed(engine, 0.0, overrides)
    feed(engine, 4.0, overrides)
    assert "isaac_camera" in engine.executor.active
    assert ("restart_component", {"component": "sim_drive"}) in backend.calls


def test_r0_safety_block_holds_actuator_actions():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph)
    blocked = {**DRIVE_DOWN, "safety": "BLOCKED"}
    feed(engine, 0.0, blocked, {"safety": "ESTOP"})
    feed(engine, 4.0, blocked, {"safety": "ESTOP"})
    assert backend.calls == []
    assert "safety not OK" in engine.executor.active["sim_drive"].message
    feed(engine, 6.0, DRIVE_DOWN)  # safety released
    assert backend.calls == [("restart_component", {"component": "sim_drive"})]


def _lidar_engine():
    graph = FakeGraph(publishers={"/livox/lidar_pc2_from_sim": 0})
    engine, backend = make(graph=graph, profile="nav2")
    feed(engine, 0.0, {"lidar_device_status": "STALE"})
    feed(engine, 4.0, {"lidar_device_status": "STALE"})
    return engine


def test_abort_is_not_reopened_until_retry():
    # Live sim 2026-10-06: Abort -> the same request came back on the next tick.
    engine = _lidar_engine()
    request = next(iter(engine.executor.requests.values()))
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "abort"}, 5.0)
    for t in range(6, 300, 4):
        feed(engine, float(t), {"lidar_device_status": "STALE"})
    assert engine.executor.requests == {}
    assert "held after ABORTED" in engine.incidents["isaac_lidar"].execution_blocked
    engine.handle_command({"type": "retry", "incident_key": "isaac_lidar"}, 301.0)
    assert len(engine.executor.requests) == 1


def test_skip_then_failed_verify_is_held():
    engine = _lidar_engine()
    request = next(iter(engine.executor.requests.values()))
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "skip"}, 5.0)
    for t in range(6, 200, 4):
        feed(engine, float(t), {"lidar_device_status": "STALE"})
    assert engine.executor.finished[-1].state == "FAILED"
    assert engine.executor.requests == {}
    assert len(engine.executor.finished) == 1


def test_resolution_releases_hold():
    engine = _lidar_engine()
    request = next(iter(engine.executor.requests.values()))
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "abort"}, 5.0)
    feed(engine, 6.0)
    feed(engine, 17.0)  # incident resolved
    assert "isaac_lidar" not in engine.executor.state(17.0)["held"]


def test_diagnosis_change_supersedes_old_run_and_request():
    # Live sim: nav2 opened as #50 (3 s window), became #52 (30 s window) and
    # the stale "#50 restart nav2 manually" request stayed open.
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/bt_navigator"})
    engine, _ = make(graph=graph)
    lifecycle_unknown = {k: "UNKNOWN" for k in (
        "lifecycle_controller_server", "lifecycle_planner_server", "lifecycle_bt_navigator", "lifecycle_local_costmap")}
    feed(engine, 0.0, lifecycle_unknown)
    feed(engine, 4.0, lifecycle_unknown)
    # Without a profile the more specific #52 is preferred: wait, no #50 run.
    assert engine.executor.requests == {}
    assert "nav2" not in engine.incidents
    for t in range(8, 40, 4):
        feed(engine, float(t), lifecycle_unknown)
    assert engine.incidents["nav2"].diagnosis.playbook_id == "PB-COMPONENT-NOT-STARTED"
    # #52 is advisory: shown, not auto-run.
    assert engine.executor.requests == {}
    assert engine.incidents["nav2"].execution_blocked.startswith("advisory")


def test_fault_cannot_be_ignored_and_healthy_evidence_closes_request():
    graph = FakeGraph(publishers={"/livox/lidar_pc2_from_sim": 0})
    engine, _ = make(graph=graph, profile="nav2")
    feed(engine, 0.0, {"lidar_device_status": "STALE"})
    feed(engine, 4.0, {"lidar_device_status": "STALE"})
    assert len(engine.executor.requests) == 1
    result = engine.handle_command({"type": "ignore", "incident_key": "isaac_lidar"}, 5.0)
    assert result["result"] != "ok"
    assert len(engine.executor.requests) == 1
    graph.publishers["/livox/lidar_pc2_from_sim"] = 1
    feed(engine, 6.0)
    assert engine.executor.requests == {}
    assert engine.executor.finished[-1].state == "SUCCEEDED"
    feed(engine, 17.0)
    assert "isaac_lidar" not in engine.incidents


def test_skip_warning_suppresses_recurrence_until_restored():
    engine, _ = make()
    feed(engine, 0.0, {"host_resources": "DEGRADED"})
    feed(engine, 12.0, {"host_resources": "DEGRADED"})
    request = next(iter(engine.executor.requests.values()))
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "skip"}, 13.0)
    assert not engine.executor.requests
    feed(engine, 14.0)
    feed(engine, 25.0)
    assert not engine.state()["ignored_warnings"][0]["active"]
    feed(engine, 30.0, {"host_resources": "DEGRADED"})
    feed(engine, 42.0, {"host_resources": "DEGRADED"})
    assert engine.state()["ignored_warnings"][0]["active"]
    assert not engine.executor.requests
    engine.handle_command({"type": "unacknowledge", "incident_key": "host"}, 43.0)
    assert engine.executor.requests


def test_low_rate_warning_does_not_start_recovery():
    engine, backend = make()
    for t in (0.0, 12.0):
        state = make_state({"lidar_device_status": "WARN"})
        item = next(i for i in state["domains"][0]["items"] if i["id"] == "lidar_device_status")
        item["warning_kind"] = "low_rate"
        engine.on_snapshot(HealthSnapshot.from_state(state, received_at=t))
    assert not backend.calls
    assert not engine.executor.requests
    response = engine.handle_request({"request_id": "slow-sensor", "required_capabilities": ["auto_navigation"]})
    assert response["status"] == "NOT_RELATED"
    assert engine.handle_command({"type": "ignore", "incident_key": "isaac_lidar"}, 13.0)["result"] == "ok"


def test_advisory_runs_only_on_operator_run():
    engine, _ = make()
    state = make_state()
    state["domains"][0]["items"].append(
        {"id": "arm_camera_model", "status": "DEGRADED", "critical": True,
         "receive_age_sec": .1,
         "summary": "Arm Camera should match real 640x480; observed 320.0x240.0"})
    for t in (0.0, 4.0, 8.0):
        engine.on_snapshot(HealthSnapshot.from_state(state, received_at=t))
    assert engine.executor.requests == {}
    engine.handle_command({"type": "retry", "incident_key": "isaac_arm_camera"}, 9.0)
    assert len(engine.executor.requests) == 1


def test_dashboard_auto_approve_pending_action_and_audit():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph, approval={"A2": "confirm"})
    assert engine.state()["auto_approve"] is False
    feed(engine, 0.0, DRIVE_DOWN)
    feed(engine, 4.0, DRIVE_DOWN)
    assert backend.calls == []
    result = engine.handle_command({"type": "set_auto_approve", "enabled": True}, 5.0)
    assert result["result"] == "ok"
    assert backend.calls == [("restart_component", {"component": "sim_drive"})]
    assert any(e["type"] == "action_auto_approved" for e in engine.drain_events())


def test_auto_approve_disable_and_strict_boolean():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph, approval={"A2": "confirm"})
    engine.handle_command({"type": "set_auto_approve", "enabled": True}, 0.0)
    engine.handle_command({"type": "set_auto_approve", "enabled": False}, 0.1)
    engine.handle_command({"type": "set_auto_approve", "enabled": "true"}, 0.2)
    assert not engine.executor.auto_approve
    feed(engine, 1.0, DRIVE_DOWN)
    feed(engine, 5.0, DRIVE_DOWN)
    assert backend.calls == []
    assert next(iter(engine.executor.requests.values())).kind == "confirm_action"


def test_auto_approve_does_not_override_forbidden_or_manual_steps():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine, backend = make(graph=graph, approval={"A2": "forbidden"})
    engine.handle_command({"type": "set_auto_approve", "enabled": True}, 0.0)
    feed(engine, 0.0, DRIVE_DOWN)
    feed(engine, 4.0, DRIVE_DOWN)
    assert not backend.calls
    engine, backend = make(graph=FakeGraph(publishers={"/livox/lidar_pc2_from_sim": 0}), profile="nav2")
    engine.handle_command({"type": "set_auto_approve", "enabled": True}, 0.0)
    feed(engine, 0.0, {"lidar_device_status": "STALE"})
    feed(engine, 4.0, {"lidar_device_status": "STALE"})
    assert not backend.calls
    assert engine.executor.requests


def test_auto_approve_rejects_real_environment():
    engine, backend = make()
    engine.env.kind = "real"
    result = engine.handle_command({"type": "set_auto_approve", "enabled": True}, 0.0)
    assert result["result"] != "ok"
    assert not engine.executor.auto_approve


def test_bad_recovery_improving_to_warning_keeps_single_bounded_verification():
    engine, backend = make(profile="nav2")
    feed(engine, 0, {"gicp_score": "DEGRADED"})
    feed(engine, 6, {"gicp_score": "DEGRADED"})
    execution_id = engine.executor.active['gicp'].id
    state = make_state({"localization_consistency": "WARN"}, values=IDLE_VALUES)
    for item in state['domains'][0]['items']:
        if item['id'] == 'localization_consistency':
            item['response_policy'] = {'level': 'WARN'}
    engine.on_snapshot(HealthSnapshot.from_state(state, received_at=8))
    assert engine.executor.active['gicp'].id == execution_id
    assert engine.executor.active['gicp'].state == 'WAITING_SUCCESS'
    feed(engine, 12)
    assert engine.executor.finished[-1].state == 'SUCCEEDED'
    assert len([c for c in backend.calls if c[0] == 'publish_initialpose']) == 1


def test_localization_low_score_without_reference_verification_cannot_resume():
    engine,backend=make(profile="nav2")
    feed(engine,0,{"gicp_score":"ERROR"});feed(engine,6,{"gicp_score":"ERROR"})
    unverified={"localization_consistency":{"components":{"localization":{"details":{"reference_verified":False}}}}}
    for t in (8,20,35):feed(engine,t,values=unverified)
    assert "auto_navigation" not in engine.state()["task_supervision"]["verified_capabilities"]
    assert not any(ex.state=="SUCCEEDED" for ex in engine.executor.finished)
