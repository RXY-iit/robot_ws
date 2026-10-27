"""No live ROS, simulator, LLM calls or robot actions in these tests."""
import pytest

from robot_agent.recovery.runtime_environment import simulation_environment
from robot_agent.recovery.snapshot import HealthSnapshot
from test_recovery_engine import ALL_SIM_NODES, FakeGraph, make_engine, make_state, snap
from test_recovery_executor import DRIVE_DOWN, FakeBackend, feed, make


class EnvironmentSystem:
    def __init__(self, process=None, timeline=None, age=0):
        self.evidence = {"process_alive": process, "timeline": timeline, "timeline_age_sec": age}

    def simulation_evidence(self, now):
        return dict(self.evidence)

    def process_alive(self, pattern):
        return False

    def path_exists(self, path):
        return False


NO_CLOCK = {"rtf": None, "clock_receive_age_sec": 100, "not_advancing_sec": 100}


@pytest.mark.parametrize("evidence,expected", [
    ({"process_alive": False}, "OFFLINE"),
    ({"process_alive": None}, "UNKNOWN"),
    ({"process_alive": True}, "UNKNOWN"),
    ({"process_alive": True, "timeline": "PAUSED", "timeline_age_sec": .1}, "PAUSED"),
    ({"process_alive": True, "timeline": "STOPPED", "timeline_age_sec": .1}, "STOPPED"),
    ({"process_alive": True, "timeline": "PLAYING", "timeline_age_sec": .1}, "DELIVERY_UNKNOWN"),
    ({"process_alive": True, "timeline": "PAUSED", "timeline_age_sec": 20}, "UNKNOWN"),
    ({"process_alive": False, "timeline": "PAUSED", "timeline_age_sec": .1}, "OFFLINE"),
])
def test_clock_loss_does_not_mean_hardware_failure(evidence, expected):
    result = simulation_environment(snap(10, values={"sim_rtf": NO_CLOCK}), evidence, 10)
    assert result["state"] == expected and not result["recovery_allowed"]


def test_stale_snapshot_cannot_prove_running_even_with_old_good_rtf():
    assert simulation_environment(snap(0), {"process_alive": True}, 10)["state"] == "UNKNOWN"
    assert simulation_environment(snap(0), {"process_alive": True, "timeline_conflict": True}, 0)["state"] == "UNKNOWN"


def test_pausing_before_hysteresis_cannot_turn_old_healthy_items_into_recovery_success():
    engine, backend = make(graph=FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"}), restart_confirmation=True)
    engine.system = EnvironmentSystem(True, "PLAYING")
    feed(engine, 0, DRIVE_DOWN)
    feed(engine, 4, DRIVE_DOWN)
    engine.system.evidence["timeline"] = "PAUSED"
    feed(engine, 20)
    assert engine.closed[-1].status == "ENVIRONMENT_SUSPENDED"
    assert engine.executor.finished[-1].state == "ABORTED"
    assert not backend.calls


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1, "0.8", None])
def test_invalid_clock_values_never_enable_actions(value):
    result = simulation_environment(snap(0, values={"sim_rtf": {"rtf": value, "clock_receive_age_sec": 0, "not_advancing_sec": 0}}), {}, 0)
    assert not result["recovery_allowed"]


@pytest.mark.parametrize("timeline", ["PAUSED", "STOPPED", None])
def test_environment_unavailable_is_not_a_forest_of_pb_errors(timeline):
    engine, backend = make(profile="nav2", restart_confirmation=True)
    engine.system = EnvironmentSystem(process=bool(timeline), timeline=timeline)
    missing = {**DRIVE_DOWN, "sim_clock": "STALE", "gicp_score": "STALE", "camera_color_diagnostics": "STALE",
               "delivery_safety": "STALE", "safety": "STALE", "delivery_robot_mode": "STALE"}
    for now in (0, 10):
        feed(engine, now, missing, {"sim_rtf": NO_CLOCK})
    assert not engine.incidents and not backend.calls and not engine.executor.requests
    state = engine.state()
    assert state["last_cycle"]["result"] == "ENVIRONMENT_UNAVAILABLE"
    assert "gicp_score" in state["environment"]["suppressed_items"]
    response = engine.handle_request({"request_id": "r1", "required_capabilities": ["auto_navigation"]})
    assert response["final"] and response["status"] == "NEEDS_HUMAN"


def test_offline_does_not_hide_host_fault_or_asserted_safety_stop():
    engine = make_engine(system=EnvironmentSystem(False))
    for t in (0, 15):
        engine.on_snapshot(snap(t, {"physical_estop": "BLOCKED", "host_resources": "DEGRADED"}, {"sim_rtf": NO_CLOCK}))
    assert any(i.diagnosis.playbook_id == "PB-SAFE-ESTOP-PRESSED" for i in engine.incidents.values())
    assert any(i.diagnosis.playbook_id == "PB-HOST-RESOURCE" for i in engine.incidents.values())
    assert next(i for i in engine.state()["incidents"] if i["playbook_id"] == "PB-SAFE-ESTOP-PRESSED")["handling_class"] == "safety_event"


def test_environment_loss_retires_pending_approval_without_fake_success():
    engine, backend = make(graph=FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"}), restart_confirmation=True)
    engine.system = EnvironmentSystem(True, "PLAYING")
    feed(engine, 0, DRIVE_DOWN)
    feed(engine, 4, DRIVE_DOWN)
    request = engine.executor.active["sim_drive"].request
    engine.system.evidence["process_alive"] = False
    feed(engine, 5, DRIVE_DOWN, {"sim_rtf": NO_CLOCK})
    assert not backend.calls and request.id not in engine.executor.requests
    assert engine.closed[-1].status == "ENVIRONMENT_SUSPENDED"
    assert engine.executor.finished[-1].state == "ABORTED"
    assert "unknown request" in engine.executor.operator_reply(request.id, "approve", 5)
    engine.system.evidence["process_alive"] = True
    feed(engine, 130, DRIVE_DOWN)
    assert engine.executor.active["sim_drive"].request.id != request.id
    assert not backend.calls


def test_sim_running_ros_off_offers_approval_then_verifies_output():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/nav_mode_switch_node"})
    class ModeBackend(FakeBackend):
        def supports(self, primitive, args):
            return True, ""
    engine, backend = make(graph=graph, backend=ModeBackend(), restart_confirmation=True)
    engine.system = EnvironmentSystem(True, "PLAYING")
    for t in (0, 4):
        feed(engine, t, {"delivery_robot_mode": "STALE"})
    request = engine.executor.active["mode_switch"].request
    assert request.buttons == ["approve", "reject"] and not backend.calls
    engine.handle_command({"type": "operator_reply", "request_id": request.id, "reply": "approve"}, 4)
    assert backend.calls == [("restart_component", {"component": "mode_switch"})]
    graph.nodes.add("/nav_mode_switch_node")
    feed(engine, 5, {"delivery_robot_mode": "STALE"})
    assert engine.executor.active["mode_switch"].state == "VERIFYING"
    feed(engine, 6)
    assert engine.executor.finished[-1].state == "SUCCEEDED"


@pytest.mark.parametrize("item,topic,pb", [
    ("lidar_device_status", "/livox/lidar_pc2_from_sim", "PB-LIDAR-STALE"),
    ("camera_color_diagnostics", "/camera/camera/color/image_raw", "PB-CAMERA-STALE"),
])
def test_missing_publisher_known_but_present_stale_publisher_unknown(item, topic, pb):
    for count in (0, 1):
        engine = make_engine(graph=FakeGraph(publishers={topic: count}))
        for now in (0, 4):
            engine.on_snapshot(snap(now, {item: "STALE"}))
        incident = next(i for i in engine.state()["incidents"] if i["playbook_id"] == pb)
        assert incident["handling_class"] == ("known_recovery" if count == 0 else "unknown_investigation")
        assert not incident["investigator_available"]


def test_startup_coverage_is_not_full_startup_success():
    engine = make_engine(profile="arm_test")
    engine.on_snapshot(snap(0))
    units = {u["id"]: u for u in engine.state()["startup_units"]}
    assert units["localization"]["status"] == "NOT_REQUIRED"
    assert units["support_tools"]["status"] == "UNVERIFIED"
    assert units["navigation"]["recovery"] == "authorized_start"
    assert units["navigation"]["status"] == "NOT_REQUIRED"
    assert units["mode_switch"]["recovery"] == "authorized_start"
    engine.tick(10)
    assert next(u for u in engine.state()["startup_units"] if u["id"] == "agent_stack")["status"] == "UNKNOWN"


def test_real_diagnostics_are_not_suppressed_by_sim_environment():
    engine = make_engine(kind="real", system=EnvironmentSystem(False))
    engine.on_snapshot(snap(0, {"physical_estop": "BLOCKED"}))
    assert engine.environment_state["recovery_allowed"]
    assert engine.environment_state["state"] == "NOT_APPLICABLE"
