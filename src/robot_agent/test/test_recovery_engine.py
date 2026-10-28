from pathlib import Path

import pytest

from robot_agent.recovery.engine import CONFIRMED, NEEDS_INVESTIGATION, RecoveryEngine
from robot_agent.recovery.environment import load_env_profile
from robot_agent.recovery.playbook import PlaybookError, build_playbook, load_playbooks
from robot_agent.recovery.primitives import build_default_registry
from robot_agent.recovery.snapshot import HealthSnapshot


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config" / "recovery"

# Mirrors HealthMonitor._publish capability dependencies.
CAPABILITIES = {
    "manual_drive": [
        "physical_estop", "drive_health", "drive_feedback", "dynamixel_health",
        "steer_feedback", "safety", "publisher:/cmd_vel",
    ],
    "auto_navigation": [
        "physical_estop", "drive_health", "drive_feedback", "dynamixel_health", "steer_feedback",
        "safety", "camera_depth_diagnostics", "gicp_score", "gicp_pose",
        "lifecycle_controller_server", "lifecycle_planner_server", "lifecycle_bt_navigator",
        "lifecycle_local_costmap",
    ],
    "stationary_perception": [
        "camera_color_diagnostics", "camera_depth_diagnostics", "camera_pan", "camera_tilt",
        "perception_runtime",
    ],
    "lift": ["physical_estop", "lift_position"],
    "so101_arm": ["so101_arm_hardware_status", "so101_arm_gateway_delivery", "so101_arm_visual_joint_states"],
}

HEALTHY_SIM_ITEMS = {
    "physical_estop": "OK",
    "drive_health": "OBSERVED",
    "drive_feedback": "OBSERVED",
    "steer_feedback": "OBSERVED",
    "dynamixel_health": "OBSERVED",
    "camera_pan": "OBSERVED",
    "camera_tilt": "OBSERVED",
    "lift_position": "OBSERVED",
    "so101_arm_hardware_status": "OBSERVED",
    "so101_arm_gateway_delivery": "OBSERVED",
    "so101_arm_visual_joint_states": "OBSERVED",
    "publisher:/cmd_vel": "OK",
    "delivery_robot_mode": "OBSERVED",
    "delivery_safety": "OBSERVED",
    "sim_clock": "OBSERVED",
    "sim_rtf": "OK",
    "host_resources": "OK",
    "robot_mode": "OK",
    "safety": "OK",
    "cmd_vel": "OBSERVED",
    "camera_color_diagnostics": "OBSERVED",
    "camera_depth_diagnostics": "OBSERVED",
    "lidar_device_status": "OBSERVED",
    "sim_lidar_adapter_cloud": "OBSERVED",
    "localization_input_cloud": "OBSERVED",
    "gicp_score": "OK",
    "gicp_pose": "OBSERVED",
    "localization_consistency": "OK",
    "lifecycle_controller_server": "OK",
    "lifecycle_planner_server": "OK",
    "lifecycle_bt_navigator": "OK",
    "lifecycle_local_costmap": "OK",
    "perception_runtime": "IDLE",
    "agent_heartbeat": "OBSERVED",
}

ALL_SIM_NODES = {
    "/sim_omni_drive_command_node", "/sim_lift_command_node", "/so101_arm_gateway",
    "/arm_visual_joint_state_calibrator", "/cmd_vel_safety_node", "/nav_mode_switch_node",
    "/gicp_localizer", "/bt_navigator", "/agent_node",
    "/controller_server", "/planner_server", "/behavior_server", "/local_costmap/local_costmap",
    "/global_costmap/global_costmap", "/lifecycle_manager_navigation", "/joy_source_arbiter_node",
    "/depth_camera_info_stamp_bridge", "/depth_proc_container", "/point_cloud_xyz_node",
}


class FakeGraph:
    def __init__(self, nodes=None, publishers=None):
        self.nodes = set(ALL_SIM_NODES if nodes is None else nodes)
        self.publishers = publishers or {}

    def node_names(self):
        return set(self.nodes)

    def publisher_count(self, topic):
        return self.publishers.get(topic, 1)


class FakeSystem:
    def __init__(self, processes=()):
        self.processes = set(processes)

    def process_alive(self, pattern):
        return pattern in self.processes

    def path_exists(self, path):
        return False


def make_state(overrides=None, values=None, sequence=1):
    statuses = {**HEALTHY_SIM_ITEMS, **(overrides or {})}
    values = {"localization_consistency": {"components": {"localization": {"details": {"reference_verified": True}}}}, "sim_rtf": {"rtf": .8, "clock_receive_age_sec": .01, "not_advancing_sec": .01}, **(values or {})}
    items = [
        {
            "id": item_id,
            "status": status,
            "summary": f"{item_id} {status}",
            "critical": item_id not in {"camera_pan", "camera_tilt", "perception_runtime", "host_resources"},
            # health_monitor reports no age for never-observed items.
            "receive_age_sec": None if status == "UNKNOWN" else 0.1,
            "value": values.get(item_id),
        }
        for item_id, status in statuses.items()
    ]
    capabilities = [
        {"id": cap_id, "status": "AVAILABLE", "dependencies": deps} for cap_id, deps in CAPABILITIES.items()
    ]
    return {"sequence": sequence, "domains": [{"id": "H1", "items": items}], "capabilities": capabilities}


def snap(t, overrides=None, values=None):
    return HealthSnapshot.from_state(make_state(overrides, values), received_at=t)


def make_engine(kind="sim", graph=None, system=None, profile=None):
    env = load_env_profile(CONFIG_DIR, kind)
    registry = build_default_registry()
    library = load_playbooks(CONFIG_DIR / "playbooks", env, registry)
    return RecoveryEngine(
        env,
        library,
        registry,
        graph=graph or FakeGraph(),
        system=system or FakeSystem(),
        launch_profile=env.launch_profiles[profile] if profile else None,
    )


def run(engine, overrides, values=None, times=(0.0, 4.0)):
    cycle = None
    for t in times:
        cycle = engine.on_snapshot(snap(t, overrides, values))
    return cycle


@pytest.mark.parametrize("kind", ["sim", "real"])
def test_env_profiles_and_playbooks_load(kind):
    engine = make_engine(kind)
    assert "PB-NODE-DOWN" in engine.library.playbooks
    if kind == "real":
        assert "PB-SIM-CLOCK-STALL" in engine.library.inactive


def test_healthy_snapshot_stops_at_stage0():
    engine = make_engine()
    cycle = engine.on_snapshot(snap(0.0))
    assert cycle["result"] == "NO_FAULT_FOUND"
    assert cycle["stage_reached"] == 0
    assert engine.incidents == {}


def test_dead_drive_node_is_one_incident_confirmed_as_proc_down():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine = make_engine(graph=graph)
    stale = {k: "STALE" for k in ("drive_health", "drive_feedback", "steer_feedback", "dynamixel_health")}
    run(engine, stale)

    assert list(engine.incidents) == ["sim_drive"]
    incident = engine.incidents["sim_drive"]
    assert incident.status == CONFIRMED
    assert incident.diagnosis.playbook_id == "PB-DRIVE-PROC-DOWN"
    plan = incident.diagnosis.plan
    assert plan["steps"][0]["args"] == {"component": "sim_drive"}
    assert plan["steps"][0]["risk"] == "A2"
    assert plan["steps"][0]["approval"] == "confirm"  # sim node restarts require explicit consent
    assert plan["execution"] == "disabled_read_only_v1"
    events = engine.drain_events()
    assert [e["type"] for e in events if e["type"] != "environment_changed"] == ["incident_opened"]


def test_drive_node_alive_but_stale_escalates_to_investigation():
    engine = make_engine(system=FakeSystem(processes={"sim_omni_drive_command_node"}))
    run(engine, {"drive_health": "STALE", "drive_feedback": "STALE"})
    incident = engine.incidents["sim_drive"]
    assert incident.status == NEEDS_INVESTIGATION
    assert "escalated" in incident.diagnosis.reason


def test_clock_stall_is_single_root_with_consequences():
    engine = make_engine()
    overrides = {
        "sim_clock": "STALE",
        "drive_health": "STALE",
        "drive_feedback": "STALE",
        "lidar_device_status": "STALE",
        "camera_color_diagnostics": "STALE",
        "gicp_score": "STALE",
    }
    run(engine, overrides)
    assert list(engine.incidents) == ["isaac"]
    diagnosis = engine.incidents["isaac"].diagnosis
    assert diagnosis.playbook_id == "PB-SIM-CLOCK-STALL"
    assert {"drive_health", "lidar_device_status", "gicp_score"} <= set(diagnosis.candidate.consequences)


def test_upstream_lidar_fault_hides_downstream_localization():
    engine = make_engine()
    run(engine, {"lidar_device_status": "STALE", "gicp_score": "DEGRADED", "gicp_pose": "STALE"})
    assert list(engine.incidents) == ["isaac_lidar"]
    incident = engine.incidents["isaac_lidar"]
    assert incident.diagnosis.playbook_id == "PB-LIDAR-STALE"
    assert set(incident.diagnosis.candidate.consequences) == {"gicp_score", "gicp_pose"}
    # Stage 2 recorded the healthy upstream it skipped.
    checks = incident.diagnosis.candidate.upstream_checks
    assert {"for": "lidar_device_status", "upstream": "sim_clock"}.items() <= checks[0].items()


def test_localization_lost_with_healthy_lidar():
    engine = make_engine()
    run(engine, {"gicp_score": "DEGRADED"}, times=(0.0, 6.0))
    incident = engine.incidents["gicp"]
    assert incident.diagnosis.playbook_id == "PB-LOC-LOST"
    primitives = [step["primitive"] for step in incident.diagnosis.plan["steps"]]
    assert "publish_initialpose" in primitives
    assert all("spin" not in str(p) for p in primitives)


def test_debounce_then_resolve():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_lift_command_node"})
    engine = make_engine(graph=graph)
    overrides = {"lift_position": "STALE", "camera_pan": "STALE"}
    first = engine.on_snapshot(snap(0.0, overrides))
    assert first["diagnoses"][0].outcome == "PENDING"
    assert engine.incidents == {}

    engine.on_snapshot(snap(4.0, overrides))
    incident = engine.incidents["sim_lift"]
    assert incident.diagnosis.playbook_id == "PB-NODE-DOWN"
    assert incident.diagnosis.plan["steps"][0]["args"] == {"component": "sim_lift"}

    # Hysteresis: healthy for less than resolve_after_sec keeps it open.
    engine.on_snapshot(snap(6.0))
    assert engine.incidents["sim_lift"].healthy_since == 6.0
    engine.on_snapshot(snap(17.0))
    assert engine.incidents == {}
    types = [e["type"] for e in engine.drain_events()]
    assert [t for t in types if t != "environment_changed"] == ["incident_opened", "incident_resolved"]
    assert engine.state()["recently_closed"][0]["status"] == "RESOLVED"


def test_software_estop_skips_debounce_and_is_safety_priority():
    engine = make_engine()
    engine.on_snapshot(snap(0.0, {"safety": "BLOCKED"}, values={"safety": "ESTOP"}))
    incident = engine.incidents["safety"]
    assert incident.diagnosis.candidate.priority == 0
    assert incident.diagnosis.playbook_id == "PB-SAFE-SOFTWARE-STOP"


def test_arm_test_profile_ignores_lidar_localization_and_nav2():
    # Live sim 2026-10-06, quick-start ros_mode=arm_test.
    engine = make_engine(profile="arm_test")
    not_started = {
        "lidar_device_status": "UNKNOWN",
        "gicp_score": "UNKNOWN",
        "gicp_pose": "UNKNOWN",
        "lifecycle_controller_server": "UNKNOWN",
        "lifecycle_planner_server": "UNKNOWN",
        "lifecycle_bt_navigator": "UNKNOWN",
        "lifecycle_local_costmap": "UNKNOWN",
    }
    cycle = run(engine, not_started)
    assert cycle["result"] == "NO_FAULT_FOUND"
    assert set(cycle["ignored"].values()) == {"not_launched_in_profile"}
    assert engine.state()["launch_profile"] == "arm_test"


def test_nav2_profile_still_reports_missing_lidar():
    engine = make_engine(profile="nav2", graph=FakeGraph(publishers={"/livox/lidar_pc2_from_sim": 0}))
    run(engine, {"lidar_device_status": "UNKNOWN"})
    assert engine.incidents["isaac_lidar"].diagnosis.playbook_id == "PB-LIDAR-STALE"


def test_event_driven_cmd_vel_raw_is_idle_not_stale():
    engine = make_engine()
    cycle = run(engine, {"cmd_vel_raw": "STALE"})
    assert cycle["result"] == "NO_FAULT_FOUND"
    assert cycle["ignored"]["cmd_vel_raw"] == "event_driven_idle"
    # A real error on the same item still counts.
    engine2 = make_engine()
    run(engine2, {"cmd_vel_raw": "ERROR"})
    assert engine2.incidents


def test_trigger_window_pending_instead_of_unmatched_incident():
    # host_resources: debounce is 3s but PB-HOST-RESOURCE waits 10s.
    engine = make_engine()
    engine.on_snapshot(snap(0.0, {"host_resources": "DEGRADED"}))
    cycle = engine.on_snapshot(snap(4.0, {"host_resources": "DEGRADED"}))
    assert cycle["diagnoses"][0].outcome == "PENDING"
    assert "PB-HOST-RESOURCE" in cycle["diagnoses"][0].reason
    assert engine.incidents == {}
    engine.on_snapshot(snap(11.0, {"host_resources": "DEGRADED"}))
    assert engine.incidents["host"].diagnosis.playbook_id == "PB-HOST-RESOURCE"


def test_intermittent_fault_does_not_flap_and_counts_reopens():
    engine = make_engine()
    bad = {"host_resources": "DEGRADED"}
    t = 0.0
    for _ in range(2):  # opens at 10s, short healthy gaps keep it open
        for _ in range(6):
            engine.on_snapshot(snap(t, bad))
            t += 2.0
        engine.on_snapshot(snap(t))
        t += 2.0
    assert list(engine.incidents) == ["host"]
    assert len(engine.closed) == 0
    # Long healthy period closes it; recurring soon after counts as a reopen.
    for _ in range(7):
        engine.on_snapshot(snap(t))
        t += 2.0
    assert engine.incidents == {}
    for _ in range(7):
        engine.on_snapshot(snap(t, bad))
        t += 2.0
    assert engine.incidents["host"].reopen_count == 1
    assert "incident_reopened" in [e["type"] for e in engine.drain_events()]


def test_camera_resolution_mismatch_is_config_deviation():
    engine = make_engine()
    state = make_state()
    state["domains"][0]["items"].append(
        {"id": "arm_camera_model", "status": "DEGRADED", "critical": True,
         "receive_age_sec": .1,
         "summary": "Arm Camera should match real 640x480; observed 320.0x240.0"}
    )
    for t in (0.0, 4.0):
        engine.on_snapshot(HealthSnapshot.from_state(state, received_at=t))
    incident = engine.incidents["isaac_arm_camera"]
    assert incident.diagnosis.playbook_id == "PB-CAMERA-MODEL-MISMATCH"
    assert incident.status == "CONFIRMED"
    assert engine.state()["incidents"][0]["playbook_no"] == 42


def test_playbook_numbers_unique_and_shown():
    engine = make_engine()
    catalog = engine.state()["playbooks"]["catalog"]
    numbers = [entry["number"] for entry in catalog]
    assert numbers == sorted(numbers) and len(numbers) == len(set(numbers))


def test_mission_request_no_fault():
    engine = make_engine()
    engine.on_snapshot(snap(0.0, values={"robot_mode": "AUTO"}))
    response = engine.handle_request({"request_id": "rr-1", "required_capabilities": ["auto_navigation"]})
    assert response["status"] == "NO_FAULT_FOUND"
    assert response["advice"] == "handle_as_task_failure"


def test_mission_request_manual_mode_blocks_navigation():
    engine = make_engine()
    engine.on_snapshot(snap(0.0, values={"robot_mode": "MANUAL"}))
    response = engine.handle_request({"request_id": "rr-2", "required_capabilities": ["auto_navigation"]})
    assert response["status"] == "NEEDS_HUMAN"
    assert response["root_cause"]["playbook"] == "PB-MODE-MANUAL"
    # The mission incident survives heartbeats while still MANUAL, closes once AUTO.
    engine.on_snapshot(snap(1.0, values={"robot_mode": "MANUAL"}))
    assert "mission:PB-MODE-MANUAL" in engine.incidents
    engine.on_snapshot(snap(2.0, values={"robot_mode": "AUTO"}))
    assert engine.incidents == {}


def test_mission_request_unrelated_fault_does_not_pause_mission():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_lift_command_node"})
    engine = make_engine(graph=graph)
    run(engine, {"lift_position": "STALE"})
    response = engine.handle_request(
        {"request_id": "rr-3", "required_capabilities": ["auto_navigation"], "result": {"code": "FAILED_SKILL"}}
    )
    assert response["status"] == "NOT_RELATED"
    # The background fault is still tracked.
    assert "sim_lift" in engine.incidents


def test_mission_request_nav_failure_with_healthy_system_is_nav_blocked():
    engine = make_engine()
    engine.on_snapshot(snap(0.0, values={"robot_mode": "AUTO"}))
    response = engine.handle_request(
        {
            "request_id": "rr-4",
            "required_capabilities": ["auto_navigation"],
            "step": {"skill": "goto_pose"},
            "result": {"code": "FAILED_NAVIGATION"},
        }
    )
    assert response["root_cause"]["playbook"] == "PB-NAV-BLOCKED"


def test_mission_request_bypasses_debounce():
    graph = FakeGraph(nodes=ALL_SIM_NODES - {"/sim_omni_drive_command_node"})
    engine = make_engine(graph=graph)
    engine.on_snapshot(snap(0.0, {"drive_health": "STALE", "drive_feedback": "STALE"}))
    assert engine.incidents == {}
    response = engine.handle_request({"request_id": "rr-5", "required_capabilities": ["manual_drive"]})
    assert response["root_cause"]["playbook"] == "PB-DRIVE-PROC-DOWN"


def test_playbook_validation_rejects_unknown_probe_and_items():
    env = load_env_profile(CONFIG_DIR, "sim")
    registry = build_default_registry()
    base = {
        "id": "PB-TEST",
        "number": 999,
        "envs": {"sim": {"status": "supported"}},
        "trigger": {"any": [{"item": "drive_health"}]},
    }
    with pytest.raises(PlaybookError, match="unknown probe"):
        build_playbook({**base, "diagnosis": [{"id": "x", "probe": "nope"}]}, env, registry)
    with pytest.raises(PlaybookError, match="not in envs/sim.yaml"):
        build_playbook({**base, "trigger": {"any": [{"item": "no_such_item"}]}}, env, registry)
    with pytest.raises(PlaybookError, match="unknown action"):
        build_playbook({**base, "recovery": [{"type": "action", "primitive": "nope"}]}, env, registry)
    # Steps can raise but not lower the declared risk.
    pb = build_playbook(
        {**base, "recovery": [{"type": "action", "primitive": "restart_component", "risk": "A0"}]}, env, registry
    )
    assert pb.recovery[0]["risk"] == "A1"
