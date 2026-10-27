from types import SimpleNamespace

import pytest

from robot_agent.recovery.backends import RosActionBackend
from robot_agent.recovery.environment import load_env_profile
from test_recovery_engine import ALL_SIM_NODES, CONFIG_DIR, FakeGraph, make_engine, run
from test_recovery_executor import DRIVE_DOWN, FakeBackend, feed, make


def test_production_sim_restarts_require_confirmation_without_changing_real_policy():
    env = load_env_profile(CONFIG_DIR, "sim")
    assert env.action_approval("A1", "restart_component") == "confirm"
    assert env.action_approval("A2", "restart_component") == "confirm"
    assert env.action_approval("A1", "cancel_motion") == "auto"
    env.approval["A2"] = "forbidden"
    assert env.action_approval("A2", "restart_component") == "forbidden"


def test_dead_localizer_hands_off_instead_of_stopping_at_support_gate():
    engine = make_engine(graph=FakeGraph(nodes=ALL_SIM_NODES - {"/gicp_localizer"}))
    run(engine, {"gicp_score": "STALE", "gicp_pose": "STALE"}, times=(0, 6))
    d = engine.incidents["gicp"].diagnosis
    assert d.outcome == "CONFIRMED" and d.playbook_id == "PB-NODE-DOWN"
    assert [s["primitive"] for s in d.plan["steps"]] == ["restart_component"]
    assert d.plan["steps"][0]["args"]["component"] == "gicp"


def test_fast_lio_absence_has_a_producer_and_node_diagnosis():
    engine = make_engine(graph=FakeGraph(nodes=ALL_SIM_NODES - {"/fast_lio"}))
    run(engine, {"fast_lio_odom": "STALE"}, times=(0, 6))
    d = engine.incidents["fast_lio"].diagnosis
    assert d.playbook_id == "PB-NODE-DOWN" and d.outcome == "CONFIRMED"


def test_lift_restart_raises_generic_pb_risk_and_requires_approval():
    engine, backend = make(graph=FakeGraph(nodes=ALL_SIM_NODES - {"/sim_lift_command_node"}),
                           restart_confirmation=True)
    missing = {"sim_lift_joint_targets": "STALE", "lift_position": "STALE"}
    feed(engine, 0, missing)
    feed(engine, 4, missing)
    assert not backend.calls
    ex = engine.executor.active["sim_lift"]
    assert ex.state == "WAITING_CONFIRM" and ex.playbook.max_action_risk == "A2"
    assert ex.request.buttons == ["approve", "reject"]
    plan = engine.incidents["sim_lift"].diagnosis.plan
    assert plan["steps"][0]["risk"] == "A2" and plan["steps"][0]["approval"] == "confirm"
    assert engine.library.get("PB-NODE-DOWN").max_action_risk == "A1", "Do not mutate the shared template"
    assert engine.executor.blocked_reason("sim_drive", engine.library.get("PB-DRIVE-PROC-DOWN"), 4) is None


@pytest.mark.parametrize("changed", ["safety_stale", "snapshot_stale", "mode_auto"])
def test_approval_rechecks_evidence_and_does_not_latch_after_block(changed):
    engine, backend = make(graph=FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"}),
                           restart_confirmation=True)
    feed(engine, 0, DRIVE_DOWN)
    feed(engine, 4, DRIVE_DOWN)
    request = engine.executor.active["sim_drive"].request
    if changed == "safety_stale":
        feed(engine, 5, {**DRIVE_DOWN, "safety": "STALE"})
    elif changed == "mode_auto":
        feed(engine, 5, DRIVE_DOWN, {"robot_mode": "AUTO"})
    now = 11 if changed == "snapshot_stale" else 5
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "approve"}, now)
    if changed == "snapshot_stale":
        assert not backend.calls
        assert request.id not in engine.executor.requests
        assert engine.closed[-1].status == "ENVIRONMENT_SUSPENDED"
        feed(engine, 12, DRIVE_DOWN)
        assert not backend.calls
        assert "unknown request" in engine.executor.operator_reply(request.id, "approve", 12)
        return
    assert not backend.calls and "Not executed" in request.note
    feed(engine, 12, DRIVE_DOWN)
    assert not backend.calls, "Fresh evidence alone must not reuse an earlier blocked approval"
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "approve"}, 12)
    assert backend.calls == [("restart_component", {"component": "sim_drive"})]


def test_stale_request_from_previous_stack_cannot_authorize_restart():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    first, _ = make(graph=graph, restart_confirmation=True)
    second, backend = make(graph=graph, restart_confirmation=True)
    for engine in (first, second):
        feed(engine, 0, DRIVE_DOWN)
        feed(engine, 4, DRIVE_DOWN)
    old = first.executor.active["sim_drive"].request
    new = second.executor.active["sim_drive"].request
    assert old.id != new.id
    assert "unknown request" in second.executor.operator_reply(old.id, "approve", 4)
    assert "not offered" in second.executor.operator_reply(new.id, "done", 4)
    feed(second, 5, DRIVE_DOWN)
    assert not backend.calls


@pytest.mark.parametrize("component", ["safety_node", "mode_switch"])
def test_bootstrap_produces_authorization_card(component):
    env = load_env_profile(CONFIG_DIR, "sim")
    class Backend(FakeBackend):
        def supports(self, primitive, args):
            return (True, "")
    engine, backend = make(graph=FakeGraph(nodes=ALL_SIM_NODES - {env.components[component].ros_node}),
                           backend=Backend(), restart_confirmation=True)
    missing = {"delivery_safety" if component == "safety_node" else "delivery_robot_mode": "STALE"}
    missing["cmd_vel_raw"] = "OBSERVED"
    values = {"cmd_vel_raw": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}}
    feed(engine, 0, missing, values)
    feed(engine, 4, missing, values)
    request = engine.executor.active[component].request
    assert request.kind == "confirm_action" and not backend.calls
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "approve"}, 4)
    assert backend.calls == [("restart_component", {"component": component})]


@pytest.mark.parametrize("present", ["graph", "process", "absent"])
def test_bootstrap_never_kills_or_duplicates_component(tmp_path, monkeypatch, present):
    from robot_agent.recovery import backends
    backend = RosActionBackend.__new__(RosActionBackend)
    backend.env = load_env_profile(CONFIG_DIR, "sim")
    backend.workspace = str(tmp_path)
    backend.map_ply = ""
    backend.log_dir = tmp_path
    nodes = [("cmd_vel_safety_node", "/")] if present == "graph" else []
    backend.node = SimpleNamespace(get_node_names_and_namespaces=lambda: nodes)
    monkeypatch.setattr(backends, "same_domain_pids", lambda _: [123] if present == "process" else [])
    monkeypatch.setattr(backends, "_signal_pids", lambda *_: pytest.fail("absence-only start must never signal a process"))
    monkeypatch.setattr(backends.time, "sleep", lambda _: None)
    spawned = []
    def spawn(command, **kwargs):
        spawned.append(command)
        nodes.append(("cmd_vel_safety_node", "/"))
        return SimpleNamespace(pid=456, poll=lambda: None)
    monkeypatch.setattr(backends.subprocess, "Popen", spawn)
    backend._restart("safety_node", lambda _: None)
    backend._restart("safety_node", lambda _: None)
    assert len(spawned) == (1 if present == "absent" else 0)
