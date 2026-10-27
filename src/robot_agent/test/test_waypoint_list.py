from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_agent.agents.navigation_agent import NavigationAgent
from robot_agent.parsers.deterministic_parser import DeterministicParser
from robot_agent.schemas import MissionIntent
from robot_agent.task_catalog import TaskCatalog
from robot_skills.waypoint_routes import named_route, read_route


MAP = "sim_factory_mid360_like_gpu_trueimu_011"


@pytest.mark.parametrize("command", ["沿着waypoint list a走行", "沿着 waypoint list A 走行。", "follow waypoint list a"])
def test_named_route_command(command):
    intent = TaskCatalog().normalize(DeterministicParser().parse(command))
    assert (intent.task_type, intent.target) == ("follow_waypoint_list", "a")
    assert TaskCatalog().tasks[intent.task_type]["owner"] == "navigation"


def test_shared_route_expands_to_exact_goals_with_recovery_visible_steps():
    route = named_route("a", MAP)
    client = SimpleNamespace(semantic=SimpleNamespace(semantic_map=SimpleNamespace(data={"map_id": MAP})))
    refinement = NavigationAgent().refine(MissionIntent("follow_waypoint_list", target="a"), client, None, "classic")
    assert not refinement.question
    goals = [s for s in refinement.plan if s.name == "goto_pose"]
    assert len(goals) == 7
    assert [s.payload["pose"] for s in goals] == [{**p, "frame_id": "map"} for p in read_route(route["path"])["poses"]]
    assert [s.catalog["waypoint_route"]["index"] for s in goals] == list(range(7))
    assert all(s.catalog["waypoint_route"]["sha256"] == route["sha256"] for s in goals)
    assert all(s.payload["apply_navigation_annotations"] is False for s in goals)
    assert all(not any(s.payload["recovery_policy"].values()) for s in goals)
    assert [s.name for s in refinement.plan[:4]] == ["await_robot_stationary", "set_camera_pose", "await_robot_auto_mode", "check_localization_health"]


@pytest.mark.parametrize("name,map_id", [("a", "sim_real_lab_001"), ("a", None), ("../a", MAP), ("missing", MAP)])
def test_route_cannot_cross_maps_or_accept_arbitrary_path(name, map_id):
    with pytest.raises((ValueError, OSError)):
        named_route(name, map_id)


def test_changed_route_changes_evidence_hash(tmp_path):
    route = named_route("a", MAP)
    copy = tmp_path / "route.yaml"
    copy.write_bytes(Path(route["path"]).read_bytes() + b"\n# revised\n")
    assert read_route(copy)["sha256"] != route["sha256"]


def test_real_mission_task_expansion_dry_run(monkeypatch):
    from robot_agent.agents.mission_agent import MissionAgent
    from robot_agent.dry_run_skill_client import DryRunSkillClient
    monkeypatch.setenv("ROBOT_MAP_ID", MAP)
    monkeypatch.delenv("ROBOT_SEMANTIC_MAP", raising=False)
    state = MissionAgent(DryRunSkillClient()).run("沿着 waypoint list a 走行", mode="dry_run")
    assert state.status == "DONE"
    assert state.tasks[0]["goal_satisfied"] is True
    assert len([r for r in state.skill_results if r["skill_name"] == "goto_pose"]) == 7


def test_move_as_waypoint_skill_keeps_goto_contract_and_fixed_goal_policy():
    from robot_skills.registry import SkillRegistry
    from robot_skills.extensions import load_extensions, resolve_alias
    assert SkillRegistry().get("move_as_waypoint")["alias_of"] == "goto_pose"
    name, args = resolve_alias(load_extensions(), "move_as_waypoint", {"pose": {"x": 1, "y": 2, "yaw": 0}})
    assert name == "goto_pose"
    assert args["apply_navigation_annotations"] is False
    assert args["recovery_policy"] == {"allow_alternate_goal": False, "allow_relaxed_tolerance": False}


def test_navigation_exception_cancels_goal_and_does_not_dispatch_later_waypoints():
    from test_domain_tasks import FakeClient
    from robot_agent.agents.mission_agent import MissionAgent
    client = FakeClient()
    client.semantic.semantic_map.data["map_id"] = MAP
    def fail(name, *_):
        if name == "goto_pose":
            raise TypeError("injected failure after goal acceptance")
    client.hook = fail
    state = MissionAgent(client).run("沿着 waypoint list a 走行")
    assert state.status == "FAILED_SKILL"
    names = [c[0] for c in client.calls]
    assert names[-2:] == ["goto_pose", "cancel_motion"]
    assert names.count("goto_pose") == 1


def test_recovery_at_second_waypoint_retries_it_without_restarting_or_skipping():
    from test_recovery_supervision import waiter_agent, update, HOLD
    from robot_skills.executors.base import make_result
    from robot_skills import result_codes as codes
    agent = waiter_agent()
    agent.agent_mode = "classic"
    agent.parser = DeterministicParser()
    agent.task_catalog = TaskCatalog()
    client = SimpleNamespace(semantic=SimpleNamespace(semantic_map=SimpleNamespace(data={"map_id": MAP})))
    plan = NavigationAgent().refine(MissionIntent("follow_waypoint_list", target="a"), client, None, "classic").plan
    agent._build_plan = lambda _: plan
    agent._annotate_plan_with_catalog = lambda *_: None
    agent._annotate_call_with_catalog = lambda *_: None
    agent._handle_health_before_step = lambda *_: True
    agent._final_message = lambda _: "done"
    agent._event = lambda *_: None
    def publish(state):
        if state.recovery_pause and state.recovery_pause["motion_stopped"]:
            update(agent.recovery_supervision, verified=["auto_navigation"])
    agent._state = publish
    seen = []
    def execute(name, payload, context):
        if name == "goto_pose":
            seen.append(payload["pose"])
            if len(seen) == 2:
                update(agent.recovery_supervision, [HOLD])
                assert context.recovery_check()
                return make_result(name, False, codes.CANCELLED, "paused")
        return make_result(name, True, codes.OK, "done")
    agent.skill_client.execute_skill = execute
    agent.skill_client.context = lambda *_: SimpleNamespace()
    result = agent.run("沿着 waypoint list a 走行")
    assert result.status == "DONE"
    expected = [{**p, "frame_id": "map"} for p in named_route("a", MAP)["poses"]]
    assert seen == expected[:2] + expected[1:]
    assert result.parser_trace["source"] == "registered_waypoint_command"
