from copy import deepcopy
from types import SimpleNamespace
import threading
import time

import pytest

from robot_agent.agents.mission_agent import MissionAgent
from robot_agent.agents.navigation_agent import NavigationAgent
from robot_agent.agents.perception_agent import PerceptionAgent
from robot_agent.agents.arm_agent import ArmAgent
from robot_agent.jev_agent import JevCallTrace, JevStrengthenAgent
from robot_agent.schemas import MissionIntent, MissionState, SkillCall
from robot_agent.task_catalog import TaskCatalog
from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result
from robot_skills.schemas import SkillContext
from robot_skills.semantic_map import SemanticMap


class FakeClient:
    def __init__(self):
        self.calls = []
        self.hook = None
        self.node = None
        semantic = SemanticMap.__new__(SemanticMap)
        semantic.data = {}
        semantic.frame_id = "map"
        semantic.objects = [{"id": "shelf_06", "names": ["shelf 06"], "category": "shelf",
                             "pose": {"x": 1.0, "y": 0.0, "yaw": 0.0}}]
        semantic.places = []
        semantic.places.append({"id": "door", "names": ["door"], "pose": {"x": 2.0, "y": 0.0}})
        self.semantic = SimpleNamespace(semantic_map=semantic)

    def context(self, run_id, max_duration_sec=120):
        return SkillContext(run_id, robot_mode="AUTO", max_duration_sec=max_duration_sec)

    def execute_skill(self, name, payload, context):
        self.calls.append((name, deepcopy(payload), context.evidence_id))
        if self.hook:
            result = self.hook(name, payload, context)
            if result:
                return result
        metrics = {"completed_time_monotonic": 10.0}
        if name == "resolve_semantic_target":
            metrics["pose"] = {"x": 1.0, "y": 0.0, "yaw": 0.0}
        return make_result(name, True, codes.OK, "completed", metrics=metrics)

    def cancel_all(self, run_id):
        return make_result("cancel_motion", True, codes.OK, "cancelled")

    def stop_all(self, run_id):
        return make_result("cancel_motion", True, codes.OK, "stopped")


@pytest.fixture
def agent(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    client = FakeClient()
    events = []
    result = MissionAgent(client, event_cb=lambda e: events.append(deepcopy(e)))
    result.events = events
    return result


def use_intent(agent, intent):
    agent.parser = SimpleNamespace(parse=lambda _: deepcopy(intent), last_metrics={})


def primary_failure(agent, monkeypatch, skill, code, action):
    from robot_agent.jev_agent import JevSupervisionDecision
    agent.set_agent_mode("jev_primary")
    attempts = []
    def fail_once(name, payload, context):
        if name == skill:
            attempts.append(name)
            if len(attempts) == 1:
                return make_result(name, False, code, "Injected failure")
    agent.skill_client.hook = fail_once
    decision = JevSupervisionDecision(action, JevCallTrace(
        call_id="fault", stage="step_supervision", adopted_decision=action), message="Injected supervision")
    monkeypatch.setattr(agent.jev_agent, "supervise_step", lambda **_: decision)
    return attempts


def test_primary_motion_failure_routes_to_requested_recovery(agent, monkeypatch):
    primary_failure(agent, monkeypatch, "move_relative", codes.FAILED_NAVIGATION, "call_recovery_agent")
    requests = []
    def recover(state, *args):
        requests.append(args)
        state.status = "NEEDS_HUMAN"
        state.final_message = "Recovery needs fresh odometry"
        return False
    monkeypatch.setattr(agent, "_run_recovery_for_supervision", recover)
    state = agent.run("go back 20cm")
    assert len(requests) == 1
    assert state.status == "NEEDS_HUMAN"


def test_primary_recovery_retries_failed_prerequisite_not_next_skill(agent, monkeypatch):
    attempts = primary_failure(agent, monkeypatch, "await_robot_stationary", codes.NEEDS_HUMAN, "call_recovery_agent")
    monkeypatch.setattr(agent, "_run_recovery_for_supervision", lambda *args: True)
    state = agent.run("look left")
    assert state.status == "DONE"
    assert len(attempts) == 2
    assert state.outcome == "SATISFIED"


def test_primary_recovery_retry_is_bounded(agent, monkeypatch):
    primary_failure(agent, monkeypatch, "await_robot_stationary", codes.NEEDS_HUMAN, "call_recovery_agent")
    agent.skill_client.hook = lambda name, *_: make_result(name, False, codes.NEEDS_HUMAN, "Still faulty") if name == "await_robot_stationary" else None
    recoveries = []
    def recover(state, *args):
        recoveries.append(args)
        assert len(recoveries) < 3, "Unbounded recovery loop"
        return True
    monkeypatch.setattr(agent, "_run_recovery_for_supervision", recover)
    state = agent.run("look left")
    assert len(recoveries) == 1
    assert state.status == "NEEDS_HUMAN"
    assert "capture_observation" not in [c[0] for c in agent.skill_client.calls]


def test_primary_relative_motion_not_replayed_after_recovery(agent, monkeypatch):
    attempts = primary_failure(agent, monkeypatch, "move_relative", codes.FAILED_NAVIGATION, "call_recovery_agent")
    monkeypatch.setattr(agent, "_run_recovery_for_supervision", lambda *args: True)
    state = agent.run("go back 20cm")
    assert state.status == "NEEDS_HUMAN"
    assert len(attempts) == 1
    assert "will not be replayed" in state.final_message


def test_primary_failed_skill_stop_has_terminal_outcome(agent, monkeypatch):
    primary_failure(agent, monkeypatch, "move_relative", codes.FAILED_NAVIGATION, "stop_failed")
    state = agent.run("go back 20cm")
    assert state.status == "FAILED_SKILL"
    assert state.outcome == "FAILED"


def test_supervision_preserves_direct_motion_evidence_and_task_risk(agent):
    failure = {"skill_name": "move_relative", "code": codes.FAILED_NAVIGATION,
               "metrics": {"timeout_reason": "wheel_odom_stale", "travelled_m": .08,
                           "wheel_odom_rejection": {"reason": "source_stale", "source_age_sec": 2.0}}}
    contract = agent.skill_catalog.compact_skill("move_relative")
    compact = agent.jev_agent._compact_skill_result_for_jev(failure, contract["supervision"]["evidence_fields"])
    assert compact["metrics"]["wheel_odom_rejection"]["reason"] == "source_stale"
    assert agent._compact_result_for_recovery(failure)["metrics"]["travelled_m"] == .08
    assert agent.jev_agent._failure_category("move_relative", codes.FAILED_NAVIGATION, {}) == "relative_motion_fault"
    assert agent.jev_agent._recovery_target_for_category("relative_motion_fault") == "drive"
    home = SkillCall("set_arm_pose", kind="task", owner="arm")
    assert agent.jev_agent._next_step_group(home)["risk"] == "physical_motion"
    plan = [SkillCall("move_relative"), SkillCall("move_relative", kind="task_end"), home]
    window = agent.jev_agent._compact_plan_window_for_jev(plan, 0)
    assert window["total_steps"] == 2
    assert len(window["window"]) == 2


def test_live_trace_never_borrows_local_probabilities():
    trace = JevStrengthenAgent._questions_from_response({"answers": {
        "complete": {"type": "noul", "noul": .05}}}, [
        {"id": "complete", "type": "noul", "result": True, "probabilities": {"true": .9}},
        {"id": "missing", "type": "choice", "result": "continue"}])
    assert trace[0]["result"] == .05
    assert "probabilities" not in trace[0]
    assert trace[1]["result"] is None
    assert trace[1]["answer_source"] == "missing"


def test_missing_live_supervision_action_fails_closed(agent, monkeypatch):
    monkeypatch.setattr(agent.jev_agent, "_decide", lambda *args: {"answers": {}})
    decision = agent.jev_agent.supervise_step(user_text="look left", intent=None,
        plan=[SkillCall("capture_observation")], step_index=0,
        result={"skill_name": "capture_observation", "success": False, "code": codes.FAILED_PERCEPTION}, health={})
    assert decision.action == "ask_human_continue_retry_stop"


def test_cancel_in_parse_does_not_start_plan_review(agent, monkeypatch):
    agent.set_agent_mode("jev_primary")
    current = []
    agent.state_cb = lambda state: current.append(state.mission_id)
    original = agent.jev_agent.parse_command
    def parse(text, **kwargs):
        result = original(text, **kwargs)
        agent.cancel(current[-1])
        return result
    monkeypatch.setattr(agent.jev_agent, "parse_command", parse)
    monkeypatch.setattr(agent.jev_agent, "plan_review", lambda *a, **kw: pytest.fail("Already cancelled"))
    state = agent.run("go back 20cm")
    assert state.status == "CANCELLED"
    assert not agent.skill_client.calls


@pytest.mark.parametrize("choice", ["set_arm_pose", "navigate_to", "cancel"])
@pytest.mark.parametrize("duplicate", [False, True])
def test_task_interpretation_wait_preserves_mission_and_cancel_scope(agent, monkeypatch, choice, duplicate):
    from test_command_grounding import span, response
    text = "go back 20cm then set pose home"
    stages = []
    def decide(stage, state, questions):
        stages.append(stage)
        if stage == "command_parse":
            return response({"command_shape": "linear", "task_count": "two",
                "task_1_type": "move_relative", "task_1_alternative_type": "none",
                "task_2_type": "navigate_to", "task_2_alternative_type": "navigate_to" if duplicate else "set_arm_pose",
                "task_3_type": "absent", "task_3_alternative_type": "none"})
        if stage == "command_disambiguation":
            return response({"task_2_alternative_type": "set_arm_pose"})
        if stage == "command_bind":
            return response({**span(text, "go back 20cm", "task_1"), **span(text, "set pose home", "task_2"),
                **span(text, "home", "task_2_goal"), "task_1_dependency": "independent",
                "task_1_direction": "back", "task_1_distance": "explicit", "task_2_dependency": "independent"})
        if stage == "plan_review":
            assert state["human_confirmed_task_types"] == {"2": choice}
            return response({"jev_graph_faithful": "faithful", "classic_graph_faithful": "incorrect", "plan_source_decision": "use_jev_plan"})
        return response({key: "execute" for key in questions})
    monkeypatch.setattr(agent.jev_agent, "_decide", decide)
    agent.set_agent_mode("jev_primary")
    agent.enable_human_resume = True
    ready, states = threading.Event(), []
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    worker = threading.Thread(target=lambda: states.append(agent.run(text)), daemon=True)
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        assert set(wait["actions"]) == {"interpret_as_set_arm_pose", "interpret_as_navigate_to"}
        assert not agent.skill_client.calls
        expected_parse = ["command_parse", "command_disambiguation"] if duplicate else ["command_parse"]
        assert stages == expected_parse
        assert not agent.human_action({**wait, "wait_id": "stale", "action": "interpret_as_set_arm_pose"})
        if choice == "cancel":
            agent.cancel(wait["mission_id"])
        else:
            assert agent.human_action({**wait, "action": "interpret_as_" + choice})
        worker.join(3)
        assert not worker.is_alive()
        final = states[0]
        assert final.mission_id == wait["mission_id"]
        assert len([e for e in agent.events if e["type"] == "mission_received"]) == 1
        assert not agent.human_action({**wait, "action": "interpret_as_set_arm_pose"})
        if choice == "cancel":
            assert final.status == "CANCELLED" and stages == expected_parse
            assert not agent.skill_client.calls
        else:
            assert final.intent.constraints["steps"][1]["task_type"] == choice
            assert stages[:len(expected_parse) + 2] == expected_parse + ["command_bind", "plan_review"]
            assert [c[0] for c in agent.skill_client.calls].count("move_relative") == 1
            if choice == "set_arm_pose":
                assert final.status == "DONE"
                assert [c[0] for c in agent.skill_client.calls].count("set_pose") == 1
            else:
                # The fake map has no home. Choosing navigation must not invent one.
                assert final.status == "NEEDS_HUMAN"
                assert "goto_pose" not in [c[0] for c in agent.skill_client.calls]
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


def test_cancel_in_review_does_not_publish_human_wait(agent, monkeypatch):
    agent.set_agent_mode("jev_primary")
    snapshots = []
    agent.state_cb = lambda state: snapshots.append(deepcopy(state.to_dict()))
    def review(*args, **kwargs):
        agent.cancel(snapshots[-1]["mission_id"])
        return JevCallTrace(call_id="review", stage="plan_review", adopted_decision="ask_human")
    monkeypatch.setattr(agent.jev_agent, "plan_review", review)
    state = agent.run("go back 20cm")
    assert state.status == "CANCELLED"
    assert not any(s["status"] == "NEEDS_HUMAN" for s in snapshots)
    assert not agent.skill_client.calls


@pytest.mark.parametrize("action", ["perception_replace_0", "perception_supported_only", "cancel"])
def test_perception_correction_resumes_same_task_or_cancels(agent, monkeypatch, action):
    from test_command_grounding import response
    def decide(stage, state, questions):
        if stage == "perception_command_parse":
            return response({"coverage": "complete", "relation": "all", "unit_count": "2",
                             "kind_0": "composite", "object_0": "suitcase", "color_0": "red"})
        if stage == "perception_plan_review":
            return response({key: "supported" for key in questions})
        return None
    monkeypatch.setattr(agent.jev_agent, "_decide", decide)
    def execute(name, payload, context):
        if name == "search_at_place":
            partial = bool(payload["unresolved_requirements"])
            return make_result(name, True, codes.OK, "recorded fixture", metrics={
                "outcome": "PARTIAL" if partial else "FOUND", "goal_satisfied": not partial,
                "inference_completed": True})
    agent.skill_client.hook = execute
    agent.enable_human_resume = True
    agent.set_agent_mode("jev_primary")
    ready, results = threading.Event(), []
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    worker = threading.Thread(target=lambda: results.append(agent.run("find red suitecase")), daemon=True)
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        assert not agent.skill_client.calls
        assert set(wait["actions"]) == {"perception_replace_0", "perception_supported_only"}
        if action == "cancel":
            agent.cancel(wait["mission_id"])
        else:
            assert agent.human_action({**wait, "action": action})
        worker.join(3)
        assert not worker.is_alive()
        state = results[0]
        assert state.mission_id == wait["mission_id"]
        if action == "cancel":
            assert state.status == "CANCELLED" and not agent.skill_client.calls
        else:
            searches = [c for c in agent.skill_client.calls if c[0] == "search_at_place"]
            assert len(searches) == 1
            assert state.status == "DONE"
            assert state.outcome == ("SATISFIED" if action == "perception_replace_0" else "PARTIAL")
            expected_query = "red suitcase" if action == "perception_replace_0" else "color_red"
            assert searches[0][1]["checks"][0]["query"] == expected_query
            assert len(state.tasks) == 1
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


@pytest.mark.parametrize("action", ["retry_failed_step", "continue_after_failed_task", "cancel"])
def test_primary_failed_step_human_choice_stays_in_mission(agent, monkeypatch, action):
    primary_failure(agent, monkeypatch, "capture_observation", codes.NEEDS_HUMAN, "ask_human_continue_retry_stop")
    agent.enable_human_resume = True
    ready, results = threading.Event(), []
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    worker = threading.Thread(target=lambda: results.append(agent.run("look left, then set pose home")))
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        assert "retry_failed_step" in wait["actions"]
        assert "continue_after_failed_task" in wait["actions"]
        if action == "cancel":
            agent.cancel(wait["mission_id"])
        else:
            assert agent.human_action({**wait, "action": action})
        worker.join(3)
        assert not worker.is_alive()
        state = results[0]
        assert state.mission_id == wait["mission_id"]
        calls = [c[0] for c in agent.skill_client.calls]
        if action == "cancel":
            assert state.status == "CANCELLED"
            assert "set_pose" not in calls
        else:
            assert calls.count("set_pose") == 1
            assert calls.count("capture_observation") == (2 if action == "retry_failed_step" else 1)
            assert state.outcome == ("SATISFIED" if action == "retry_failed_step" else "PARTIAL")
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


def test_catalog_merged_names_and_composites():
    catalog = TaskCatalog()
    assert len(catalog.tasks) == 13
    assert catalog.normalize(MissionIntent("press_object")).constraints["operation"] == "press"
    assert catalog.normalize(MissionIntent("release_object")).constraints["desired_end_state"] == "released"
    tasks = catalog.flatten(MissionIntent("search_object_near_place", target="shelf_06", object_query="red box"))
    assert [t.task_type for t in tasks] == ["navigate_to", "search_object"]
    assert tasks[1].constraints["dependency_policy"] == "required_condition"


def test_domain_delegation_and_feedback(agent):
    use_intent(agent, MissionIntent("sequence", constraints={"steps": [
        MissionIntent("look_direction", constraints={"direction": "left"}).to_dict(),
        MissionIntent("set_pose", target="home").to_dict()]}))
    state = agent.run("look left then arm home")
    assert state.status == "DONE"
    assert [t["owner"] for t in state.tasks] == ["perception", "arm"]
    assert all(t["goal_satisfied"] for t in state.tasks)
    assert [e["type"] for e in agent.events].count("task_delegated") == 2
    assert [e["type"] for e in agent.events].count("task_feedback") == 2
    assert all(x.get("task_id") and x.get("step_id") for x in state.skill_results)


def test_dashboard_plan_snapshot_survives_domain_expansion(agent):
    states = []
    agent.state_cb = lambda state: states.append(deepcopy(state.to_dict()))
    use_intent(agent, MissionIntent("sequence", constraints={"steps": [
        MissionIntent("look_direction", constraints={"direction": "left"}).to_dict(),
        MissionIntent("set_pose", target="home").to_dict()]}))
    state = agent.run("look left then arm home")
    assert any(s["status"] == "PARSE" for s in states)
    assert any(s["status"] == "PLAN" for s in states)
    assert any(any(t["state"] == "REFINING" for t in s["tasks"]) for s in states)
    assert len(state.initial_plan) == 2
    assert all(item["kind"] == "task" for item in state.initial_plan)
    assert any(item.kind == "skill" for item in state.plan)
    snapshot = deepcopy(state.initial_plan)
    state.plan[0].payload["test_mutation"] = True
    assert state.initial_plan == snapshot
    original_parse = deepcopy(state.parsed_intent)
    state.intent.constraints["test_mutation"] = True
    assert state.parsed_intent == original_parse


def test_dashboard_domain_decisions_include_task_identity(agent):
    agent.set_agent_mode("jev_primary")
    state = agent.run("look left")
    calls = [c for c in state.jev_trace["calls"] if c["stage"].startswith("perception")]
    assert calls
    assert all(c["task_id"] in {t["task_id"] for t in state.tasks} for c in calls)


def test_dashboard_preserves_jev_parse_when_review_selects_classic(agent):
    use_intent(agent, MissionIntent("set_arm_pose", target="home"))
    agent.set_agent_mode("jev_primary")
    agent.jev_agent.plan_review = lambda *args, **kwargs: JevCallTrace(
        call_id="review", stage="plan_review", adopted_decision="use_classic_plan")
    state = agent.run("look left")
    assert state.parsed_intent["task_type"] == "observe_scene"
    assert state.intent.task_type == "set_arm_pose"
    assert state.initial_plan[0]["name"] == "set_arm_pose"


def test_dashboard_rejected_plan_has_no_accepted_snapshot(agent):
    agent.set_agent_mode("jev_primary")
    agent.jev_agent.plan_review = lambda *args, **kwargs: JevCallTrace(
        call_id="review", stage="plan_review", adopted_decision="reject")
    state = agent.run("look left")
    assert state.status == "FAILED_SKILL"
    assert state.initial_plan == []
    assert state.plan
    assert not agent.skill_client.calls


def prepare_waiting_agent(agent):
    agent.enable_human_resume = True
    agent.skill_client.system = SimpleNamespace(robot_mode="MANUAL", _last_robot_mode_time=time.monotonic())
    agent.skill_client.hook = lambda name, *_: make_result(name, False, codes.NEEDS_HUMAN, "Need AUTO") if (
        name == "await_robot_auto_mode" and agent.skill_client.system.robot_mode != "AUTO") else None
    ready = threading.Event()
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    results = []
    worker = threading.Thread(target=lambda: results.append(agent.run("back after arm home")), daemon=True)
    return ready, results, worker


def test_auto_approval_waits_for_fresh_feedback_then_resumes_same_mission(agent):
    use_intent(agent, MissionIntent("sequence", constraints={"steps": [
        MissionIntent("set_arm_pose", target="home").to_dict(),
        MissionIntent("move_relative", constraints={"direction": "back", "distance_m": .2}).to_dict()]}))
    ready, results, worker = prepare_waiting_agent(agent)
    requested = threading.Event()
    agent.request_auto_mode = requested.set
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        assert not agent.human_action({**wait, "wait_id": "old", "action": "set_auto"})
        assert agent.human_action({**wait, "action": "set_auto"})
        assert requested.wait(1)
        assert worker.is_alive()
        assert "move_relative" not in [c[0] for c in agent.skill_client.calls]
        agent.skill_client.system.robot_mode = "AUTO"
        agent.skill_client.system._last_robot_mode_time = time.monotonic() - 10
        time.sleep(.3)
        assert worker.is_alive(), "Stale AUTO must not resume"
        agent.skill_client.system._last_robot_mode_time = time.monotonic()
        worker.join(3)
        assert not worker.is_alive()
        state = results[0]
        assert state.status == "DONE"
        assert state.mission_id == wait["mission_id"]
        assert [c[0] for c in agent.skill_client.calls].count("set_pose") == 1
        assert [c[0] for c in agent.skill_client.calls].count("move_relative") == 1
        assert len([e for e in agent.events if e["type"] == "mission_received"]) == 1
        assert not agent.human_action({**wait, "action": "set_auto"})
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


def test_cancel_waiting_prerequisite_never_resurrects_on_auto(agent):
    use_intent(agent, MissionIntent("move_relative", constraints={"distance_m": .2}))
    ready, results, worker = prepare_waiting_agent(agent)
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        agent.cancel(wait["mission_id"])
        worker.join(2)
        assert not worker.is_alive()
        assert results[0].status == "CANCELLED"
        assert results[0].tasks[0]["state"] == "CANCELLED"
        agent.skill_client.system.robot_mode = "AUTO"
        assert not agent.human_action({**wait, "action": "resume"})
        assert "move_relative" not in [c[0] for c in agent.skill_client.calls]
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


@pytest.mark.parametrize('action', ['resume', 'cancel'])
def test_navigation_review_hold_rechecks_same_step_without_failure_supervision(agent, action):
    use_intent(agent, MissionIntent('navigate_to', target='shelf_06'))
    agent.enable_human_resume = True
    agent.set_agent_mode('jev_primary')
    ready = threading.Event()
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    attempts = []
    def hook(name, payload, context):
        if name == 'goto_pose':
            attempts.append(name)
            if len(attempts) == 1:
                return make_result(name, False, codes.NEEDS_HUMAN, 'Route policy needs clarification',
                                   metrics={'navigation_review': {'decision': 'ask_human'}})
    agent.skill_client.hook = hook
    states = []
    worker = threading.Thread(target=lambda: states.append(agent.run('go to shelf_06')), daemon=True)
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        assert wait['actions'] == ['resume']
        assert not any(e['type'] == 'jev_supervision_decision' for e in agent.events)
        assert not any(c[0] == 'search_at_place' for c in agent.skill_client.calls)
        if action == 'resume':
            assert agent.human_action({**wait, 'action': 'resume'})
        else:
            agent.cancel(wait['mission_id'])
        worker.join(3)
        assert not worker.is_alive()
        assert states[0].mission_id == wait['mission_id']
        assert states[0].status == ('DONE' if action == 'resume' else 'CANCELLED')
        assert len(attempts) == (2 if action == 'resume' else 1)
        assert [c[0] for c in agent.skill_client.calls].count('resolve_semantic_target') == 1
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait['mission_id'])
        worker.join(2)


def test_jev_offers_auto_only_for_control_mode_prerequisite(agent):
    jev = agent.jev_agent
    requests = []
    jev._decide = lambda *args: requests.append(args) or None
    step = SkillCall("await_robot_auto_mode")
    decision = jev.supervise_step(user_text="go back", intent=MissionIntent("move_relative"),
        plan=[step], step_index=0, result={"skill_name": step.name, "code": codes.NEEDS_HUMAN, "success": False}, health=None)
    assert decision.action == "ask_human_enable_auto"
    assert "ask_human_enable_auto" in requests[0][2]["supervision_action"]["criteria"]


def test_arm_primary_reviews_but_shadow_never_adopts_review_rejection(agent):
    jev = SimpleNamespace(domain_review=lambda *args: ("ask_human", JevCallTrace(call_id="arm", stage="arm_task_review")))
    intent = MissionIntent("set_arm_pose", target="home")
    assert agent.arm_agent.refine(intent, agent.skill_client, jev, "jev_primary").question
    shadow = agent.arm_agent.refine(intent, agent.skill_client, jev, "jev_shadow")
    assert not shadow.question
    assert shadow.plan[-1].name == "set_pose"
    assert len(shadow.traces) == 1


def test_manual_repair_rechecks_health_without_replaying_completed_steps(agent):
    use_intent(agent, MissionIntent("move_relative", constraints={"direction": "back", "distance_m": .2}))
    agent.enable_human_resume = True
    health = {"overall": "BAD", "components": {"safety": {"level": "BAD"}}}
    agent.skill_client.system = SimpleNamespace(health_summary=lambda _: deepcopy(health))
    waits = []
    ready = threading.Event()
    agent.state_cb = lambda state: (waits.append(deepcopy(state.human_wait)), ready.set()) if state.human_wait else None
    results = []
    worker = threading.Thread(target=lambda: results.append(agent.run("go back 20cm")), daemon=True)
    worker.start()
    try:
        assert ready.wait(3)
        first = dict(agent._human_wait)
        ready.clear()
        assert agent.human_action({**first, "action": "resume"})
        assert ready.wait(3)
        second = dict(agent._human_wait)
        assert first["wait_id"] != second["wait_id"]
        assert "move_relative" not in [c[0] for c in agent.skill_client.calls]
        health["overall"] = "OK"
        health["components"]["safety"]["level"] = "OK"
        assert agent.human_action({**second, "action": "resume"})
        worker.join(3)
        assert not worker.is_alive()
        assert results[0].status == "DONE"
        assert [c[0] for c in agent.skill_client.calls].count("await_robot_auto_mode") == 1
        assert [c[0] for c in agent.skill_client.calls].count("move_relative") == 1
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


@pytest.mark.parametrize("next_policy,expected", [(None, "DONE"), ("independent_goals", "DONE"),
                                                   ("required_condition", "NEEDS_HUMAN"),
                                                   ("ambiguous_dependency", "NEEDS_HUMAN")])
def test_not_found_is_a_completed_search_not_an_execution_fault(agent, next_policy, expected):
    search = MissionIntent("search_object", object_query="red box")
    intent = search if next_policy is None else MissionIntent("sequence", constraints={"steps": [
        search.to_dict(), MissionIntent("set_arm_pose", target="home", constraints={"dependency_policy": next_policy}).to_dict()]})
    use_intent(agent, intent)
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, "not found", metrics={
        "inference_completed": True, "goal_satisfied": False, "outcome": "NOT_FOUND"}) if name == "search_at_place" else None
    state = agent.run("search")
    assert state.status == expected
    assert state.tasks[0]["goal_satisfied"] is False
    assert not agent._should_call_jev_supervision(state.skill_results[0])
    if expected == "DONE":
        assert state.outcome == "PARTIAL"
    else:
        assert [c[0] for c in agent.skill_client.calls] == ["search_at_place"]
        assert len(state.tasks) == 2
        assert state.tasks[1]["outcome"] == "DEPENDENCY_UNCONFIRMED"


def test_redaction_keeps_measurements_not_credentials():
    value = JevStrengthenAgent._redact_for_log({"input_tokens": 123, "required_tokens": ["06"],
                                              "access_token": "secret-value", "api_key": "key"})
    assert value == {"input_tokens": 123, "required_tokens": ["06"],
                     "access_token": "<redacted>", "api_key": "<redacted>"}


@pytest.mark.parametrize("cancel", [False, True])
@pytest.mark.parametrize("mode", ["classic", "jev_primary", "jev_shadow"])
def test_m13_motion_needs_scoped_approval_after_not_found(agent, cancel, mode):
    agent.agent_mode = mode
    use_intent(agent, MissionIntent("sequence", constraints={"dependency_policy": "ordered_but_independent", "steps": [
        MissionIntent("search_object", object_query="red object").to_dict(),
        MissionIntent("move_relative", constraints={"direction": "back", "distance_m": .2}).to_dict()]}))
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, "not found", metrics={
        "inference_completed": True, "goal_satisfied": False, "outcome": "NOT_FOUND"}) if name == "search_at_place" else None
    agent.enable_human_resume = True
    ready, results = threading.Event(), []
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    worker = threading.Thread(target=lambda: results.append(agent.run("check red object and go back a little")), daemon=True)
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        assert not any(c[0] == "move_relative" for c in agent.skill_client.calls)
        if cancel:
            agent.cancel(wait["mission_id"])
        else:
            assert not agent.human_action({**wait, "wait_id": "old", "action": "continue_unmet_goal"})
            assert agent.human_action({**wait, "action": "continue_unmet_goal"})
        worker.join(3)
        assert not worker.is_alive()
        assert results[0].status == ("CANCELLED" if cancel else "DONE")
        assert results[0].mission_id == wait["mission_id"]
        assert any(c[0] == "move_relative" for c in agent.skill_client.calls) != cancel
        assert len(results[0].tasks) == 2
        if not cancel:
            assert len([e for e in agent.events if e["type"] == "task_delegated"]) == 2
        assert not agent.human_action({**wait, "action": "continue_unmet_goal"})
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


@pytest.mark.parametrize("outcome,goal,status,moved", [("FOUND", True, "DONE", True),
    ("NOT_FOUND", False, "DONE", False), ("PARTIAL", False, "NEEDS_HUMAN", False),
    ("FOUND", False, "NEEDS_HUMAN", False)])
def test_m12_detection_condition_uses_complete_current_task_evidence(agent, outcome, goal, status, moved):
    from robot_agent.parsers.deterministic_parser import DeterministicParser
    command = "if you find a red box, go back 20cm; otherwise stay still"
    use_intent(agent, DeterministicParser().parse(command))
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, outcome, metrics={
        "inference_completed": True, "goal_satisfied": goal, "outcome": outcome}) if name == "search_at_place" else None
    state = agent.run(command)
    assert state.status == status
    assert any(c[0] == "move_relative" for c in agent.skill_client.calls) == moved
    if status == "NEEDS_HUMAN":
        assert len(state.tasks) == 2
        assert state.tasks[1]["state"] == "WAITING_HUMAN"
        assert state.tasks[1]["outcome"] == "CONDITION_UNKNOWN"
        assert state.tasks[1]["message"] == state.question_for_human
    if outcome == "NOT_FOUND":
        assert state.outcome == "SATISFIED"
        assert state.tasks[0]["goal_satisfied"] is False
        assert state.tasks[1]["outcome"] == "CONDITION_FALSE"


@pytest.mark.parametrize("first_satisfied,second_satisfied,expected", [
    (True, False, "DONE"), (False, True, "NEEDS_HUMAN"),
])
def test_explicit_dependency_uses_referenced_goal_not_last_goal(agent, first_satisfied, second_satisfied, expected):
    steps = [MissionIntent("search_object", object_query="red", constraints={"task_ordinal": 1}),
             MissionIntent("search_object", object_query="blue", constraints={"task_ordinal": 2, "dependency_policy": "independent_goals"}),
             MissionIntent("set_arm_pose", target="home", constraints={"task_ordinal": 3, "depends_on": [1], "dependency_policy": "required_condition"})]
    use_intent(agent, MissionIntent("sequence", constraints={"steps": [x.to_dict() for x in steps]}))
    outcomes = iter([first_satisfied, second_satisfied])
    def detect(name, *_):
        if name == "search_at_place":
            found = next(outcomes)
            return make_result(name, True, codes.OK, "completed", metrics={
                "inference_completed": True, "goal_satisfied": found, "outcome": "FOUND" if found else "NOT_FOUND"})
    agent.skill_client.hook = detect
    state = agent.run("test explicit graph references")
    assert state.status == expected
    assert any(c[0] == "set_pose" for c in agent.skill_client.calls) == first_satisfied


@pytest.mark.parametrize("outcome,satisfied,status,arm_runs", [
    ("FOUND", True, "DONE", True),
    ("NOT_FOUND", False, "DONE", False),
    ("PARTIAL", False, "NEEDS_HUMAN", False),
    ("FOUND", False, "NEEDS_HUMAN", False),
])
def test_conditional_arm_requires_complete_detection(agent, outcome, satisfied, status, arm_runs):
    steps = [
        MissionIntent("search_object", object_query="red", constraints={"task_ordinal": 1}),
        MissionIntent("set_arm_pose", target="home", constraints={
            "task_ordinal": 2, "depends_on": [1], "dependency_policy": "required_condition",
            "run_if": "previous_search_found"}),
    ]
    use_intent(agent, MissionIntent("sequence", constraints={
        "steps": [step.to_dict() for step in steps], "else_action": "stay_still"}))
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, outcome, metrics={
        "inference_completed": True, "goal_satisfied": satisfied, "outcome": outcome,
    }) if name == "search_at_place" else None
    state = agent.run("conditional arm evidence test")
    assert state.status == status
    assert any(call[0] == "set_pose" for call in agent.skill_client.calls) == arm_runs
    if outcome == "NOT_FOUND":
        assert state.tasks[1]["outcome"] == "CONDITION_FALSE"
        assert state.outcome == "SATISFIED"
        assert "conditional task skipped" in state.final_message
        assert "conditional motion skipped" not in state.final_message


def test_explicit_after_attempt_preserves_partial_goal_and_executes_cleanup(agent):
    use_intent(agent, MissionIntent("sequence", constraints={"dependency_policy": "per_task", "steps": [
        MissionIntent("search_object", object_query="red box", constraints={"task_ordinal": 1}).to_dict(),
        MissionIntent("set_arm_pose", target="home", constraints={"task_ordinal": 2, "depends_on": [],
            "dependency_policy": "cleanup_after_attempt", "explicit_after_attempt": True}).to_dict()]}))
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, "only color supported", metrics={
        "inference_completed": True, "goal_satisfied": False, "outcome": "PARTIAL"}) if name == "search_at_place" else None
    state = agent.run("search and unconditional cleanup")
    assert state.status == "DONE" and state.outcome == "PARTIAL"
    assert any(c[0] == "set_pose" for c in agent.skill_client.calls)


def test_status_task_returns_measurements_after_rename(agent):
    use_intent(agent, MissionIntent("get_robot_status"))
    assert "completed_time_monotonic" in agent.run("robot status").final_message


def test_m12_incomplete_inference_never_triggers_motion(agent):
    from robot_agent.parsers.deterministic_parser import DeterministicParser
    command = "if you find bottle, go back 20cm; otherwise stay still"
    use_intent(agent, DeterministicParser().parse(command))
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, "incomplete", metrics={
        "inference_completed": False, "goal_satisfied": True, "outcome": "FOUND"}) if name == "search_at_place" else None
    state = agent.run(command)
    assert state.status == "NEEDS_HUMAN"
    assert not any(c[0] == "move_relative" for c in agent.skill_client.calls)


def test_internal_navigation_retry_does_not_call_exception_supervision(agent):
    use_intent(agent, MissionIntent("navigate_to", target="shelf_06", constraints={"visual_confirmation": False}))
    agent.set_agent_mode("jev_shadow")
    agent.jev_agent.supervise_step = lambda **kwargs: pytest.fail("handled retry must not escalate")
    poses = [{"x": 1.0, "y": 0.0, "yaw": 0.0}, {"x": 1.2, "y": 0.0, "yaw": 0.0}]
    attempts = []
    def hook(name, payload, context):
        if name == "resolve_semantic_target":
            return make_result(name, True, codes.OK, "resolved", metrics={
                "pose": poses[0], "approach_candidates": [{"pose": p} for p in poses]})
        if name == "goto_pose":
            attempts.append(payload["pose"])
            if len(attempts) == 1:
                return make_result(name, False, codes.FAILED_NAVIGATION, "first approach failed")
    agent.skill_client.hook = hook
    state = agent.run("go to shelf_06")
    assert state.status == "DONE"
    assert len(attempts) == 2 and attempts[0] != attempts[1]
    assert any(e["type"] == "step_supervision_skipped" for e in agent.events)


def test_navigation_candidate_exhaustion_does_not_retry_failed_original(agent):
    state = MissionState("navigate")
    pose = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    failed = {}
    result = make_result("goto_pose", False, codes.FAILED_NAVIGATION, "failed")
    assert not agent._try_next_navigation_candidate(state, 0, pose, pose, [{"pose": pose}], failed, result)


def test_task_is_refined_at_execution_not_plan_build(agent):
    agent.perception_agent.refine = lambda *a: pytest.fail("must not refine before dispatch")
    state = MissionState("find red box", intent=MissionIntent("search_object", object_query="red box"))
    plan = agent._build_plan(state)
    assert len(plan) == 1 and plan[0].kind == "task"


def test_classic_comparison_plan_keeps_current_mission_identity(agent):
    state = MissionState("look left", mission_id="actual-mission")
    plan = agent._build_plan_for_intent(MissionIntent("observe_scene", constraints={"direction": "left"}), state)
    assert plan[0].task_id == "actual-mission:t1"


def test_cancel_during_domain_decision_never_executes(agent):
    use_intent(agent, MissionIntent("observe_scene", constraints={"direction": "left"}))
    original = agent.perception_agent.refine
    def refine(*args):
        agent._cancelled_mission_ids.add(agent.events[0]["mission_id"])
        return original(*args)
    agent.perception_agent.refine = refine
    state = agent.run("look left")
    assert state.status == "CANCELLED"
    assert agent.skill_client.calls == []


def test_cancel_after_first_skill_never_returns_camera_home(agent):
    use_intent(agent, MissionIntent("observe_scene", constraints={"direction": "left"}))
    def hook(name, payload, context):
        if name == "set_camera_pose":
            agent.cancel(context.run_id)
    agent.skill_client.hook = hook
    state = agent.run("look left")
    assert state.status == "CANCELLED"
    assert [c[0] for c in agent.skill_client.calls] == ["await_robot_stationary", "set_camera_pose"]


def test_each_search_has_a_separate_evidence_epoch(agent):
    use_intent(agent, MissionIntent("sequence", constraints={"steps": [
        MissionIntent("search_object", object_query="red box").to_dict(),
        MissionIntent("search_object", object_query="blue box").to_dict()]}))
    state = agent.run("find red box then blue box")
    searches = [c for c in agent.skill_client.calls if c[0] == "search_at_place"]
    assert state.status == "DONE" and len(searches) == 2
    assert searches[0][2] != searches[1][2]


def test_navigation_owns_nested_perception_feedback(agent):
    use_intent(agent, MissionIntent("navigate_to", target="shelf_06"))
    state = agent.run("go to shelf_06")
    assert state.status == "DONE"
    assert [t["owner"] for t in state.tasks] == ["navigation", "perception"]
    assert state.tasks[1]["parent_task_id"] == state.tasks[0]["task_id"]
    assert all(t["goal_satisfied"] for t in state.tasks)


def test_navigation_arrival_survives_failed_identity_confirmation(agent):
    use_intent(agent, MissionIntent("navigate_to", target="shelf_06"))
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, "not found", metrics={
        "goal_satisfied": False, "outcome": "NOT_FOUND"}) if name == "search_at_place" else None
    state = agent.run("go to shelf_06")
    nav = state.tasks[0]
    assert state.status == "DONE" and state.outcome == "PARTIAL"
    assert nav["navigation_arrival"]["reached"] is True
    assert nav["visual_confirmation"] == "UNCONFIRMED" and nav["goal_satisfied"] is False


@pytest.mark.parametrize("cancel", [False, True])
@pytest.mark.parametrize("mode", ["classic", "jev_primary"])
def test_navigation_target_choice_resumes_same_mission(agent, cancel, mode):
    agent.set_agent_mode(mode)
    semantic = agent.skill_client.semantic.semantic_map
    semantic.places = [{"id": "front_door", "names": ["door"], "pose": {"x": 1, "y": 0}},
                       {"id": "back_door", "names": ["door"], "pose": {"x": 2, "y": 0}}]
    use_intent(agent, MissionIntent("navigate_to", target="door"))
    agent.enable_human_resume = True
    ready, results = threading.Event(), []
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    worker = threading.Thread(target=lambda: results.append(agent.run("go to door")), daemon=True)
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        if cancel:
            agent.cancel(wait["mission_id"])
        else:
            assert agent.human_action({**wait, "action": "navigation_target_0"})
        worker.join(3)
        assert not worker.is_alive()
        assert results[0].status == ("CANCELLED" if cancel else "DONE")
        assert results[0].mission_id == wait["mission_id"]
        assert len(results[0].tasks) == 1
        assert bool(agent.skill_client.calls) != cancel
        assert not agent.human_action({**wait, "action": "navigation_target_0"})
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


def test_primary_navigation_choice_then_label_checks_then_red_object(agent, monkeypatch):
    from pathlib import Path
    from robot_agent.jev_agent import JevParseDecision
    semantic = SemanticMap(Path(__file__).resolve().parents[3] /
        "maps/sim_factory_mid360_like_gpu_trueimu_011/semantic_map.yaml")
    agent.skill_client.semantic.semantic_map = semantic
    intent = MissionIntent("sequence", constraints={"steps": [
        MissionIntent("navigate_to", target="shelf 05", raw_text="go to shelf 05",
                      constraints={"task_ordinal": 1}).to_dict(),
        MissionIntent("search_object", object_query="red object", constraints={"task_ordinal": 2, "depends_on": [1],
            "dependency_policy": "required_condition"}, raw_text="find red object").to_dict()]})
    monkeypatch.setattr(agent.jev_agent, "parse_command", lambda *a, **k: JevParseDecision(
        deepcopy(intent), JevCallTrace(call_id="parse", stage="command_parse", adopted_decision="task_graph_grounded")))
    def decide(stage, state, questions):
        if stage == "navigation_command_parse":
            assert "goal_binding" not in questions
            assert state["evidence"]["target_binding"]["source"] == "human_selection"
            values = {"goal_kind": "region", "visual_confirmation": "required"}
        elif stage == "navigation_plan_review":
            values = {"decision": "execute"}
        elif stage == "perception_command_parse":
            is_label = state["evidence"]["recognition_contract"] is not None
            values = {"coverage": "complete", "relation": "all", "unit_count": "1",
                      "kind_0": "ocr" if is_label else "color", "object_0": "none",
                      "color_0": "none" if is_label else "red"}
        elif stage == "perception_plan_review":
            values = {key: "supported" for key in questions}
        else:
            return None
        return {"answers": {key: {"choice": values[key]} for key in questions}}
    monkeypatch.setattr(agent.jev_agent, "_decide", decide)
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, "fixture found", metrics={
        "outcome": "FOUND", "goal_satisfied": True, "inference_completed": True}) if name == "search_at_place" else None
    agent.set_agent_mode("jev_primary")
    agent.enable_human_resume = True
    ready, results = threading.Event(), []
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    worker = threading.Thread(target=lambda: results.append(agent.run("go to shelf 05，then find red object")), daemon=True)
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        assert not agent.skill_client.calls
        assert not agent.human_action({**wait, "wait_id": "stale", "action": "navigation_target_0"})
        assert agent.human_action({**wait, "action": "navigation_target_0"})
        worker.join(5)
        assert not worker.is_alive()
        state = results[0]
        assert state.mission_id == wait["mission_id"] and state.status == "DONE", (state.final_message, state.tasks)
        assert state.outcome == "SATISFIED"
        assert len([e for e in agent.events if e["type"] == "mission_waiting_for_domain_choice"]) == 1
        assert len([c for c in agent.skill_client.calls if c[0] == "goto_pose"]) == 1
        searches = [c[1] for c in agent.skill_client.calls if c[0] == "search_at_place"]
        assert [[c["query"] for c in s["checks"]] for s in searches] == [["text_05", "text_06"], ["color_red"]]
        assert len(state.tasks) == 3
        assert not state.question_for_human and not state.human_wait
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


def test_navigation_callback_is_bound_to_expanded_step_and_records_path(agent):
    use_intent(agent, MissionIntent("navigate_to", target="door"))
    def hook(name, payload, context):
        if name == "goto_pose":
            assert context.navigation_review is not None
            decision = context.navigation_review({"path": [{"x": 0, "y": 0}, {"x": 1, "y": 0}],
                "target_pose": {"x": 1, "y": 0, "frame_id": "map"}, "annotation": None})
            assert decision["decision"] == "execute"
    agent.skill_client.hook = hook
    state = agent.run("go to door")
    assert state.status == "DONE"
    assert any(e["type"] == "navigation_path_review" for e in agent.events)


def test_missing_manipulation_is_reported_without_motion(agent):
    use_intent(agent, MissionIntent("manipulate_object", object_query="box", constraints={"desired_end_state": "placed_at"}))
    state = agent.run("put box on table")
    assert state.status == "NEEDS_HUMAN"
    assert state.tasks[0]["outcome"] == "UNSUPPORTED"
    assert agent.skill_client.calls == []


def test_primary_review_uses_new_catalog(monkeypatch, agent):
    jev = agent.jev_agent
    monkeypatch.setattr(jev, "_decide", lambda *args: None)
    intent = jev.parse_command("go back a little then set pose home").intent
    assert [x["task_type"] for x in intent.constraints["steps"]] == ["move_relative", "set_arm_pose"]
    state = MissionState("go back a little then set pose home", intent=intent)
    trace = jev.plan_review(state.user_text, intent, agent._build_plan(state), None)
    assert trace.adopted_decision == "use_jev_plan"
    assert {x["task_type"] for x in jev._supported_task_types()} == set(TaskCatalog().tasks)


def test_primary_adopts_domain_review_shadow_does_not(agent):
    class Jev:
        def domain_questions(self, stage, intent, questions, defaults, evidence):
            answers = {**defaults, "check_0_compatible": "unsupported"} if stage == "perception_plan_review" else defaults
            return answers, JevCallTrace(call_id="parse", stage=stage)
        def domain_review(self, *args):
            return "ask_human", JevCallTrace(call_id="review", stage=args[0])
    intent = MissionIntent("search_object", object_query="bottle")
    primary = PerceptionAgent().refine(intent, agent.skill_client, Jev(), "jev_primary")
    shadow = PerceptionAgent().refine(intent, agent.skill_client, Jev(), "jev_shadow")
    assert primary.question and not primary.plan
    assert shadow.plan[0].payload["provider"] == "yolo11s"
    assert agent.skill_client.calls == []


def test_domain_ask_human_prevents_execution(agent):
    use_intent(agent, MissionIntent("observe_scene", constraints={"direction": "left"}))
    agent.agent_mode = "jev_shadow"
    # A shadow rejection must never replace the deterministic domain plan.
    agent.jev_agent.domain_review = lambda *args: ("ask_human", SimpleNamespace(to_dict=lambda: {}))
    # Exercise the domain directly: real trace aggregation requires JevCallTrace.
    result = PerceptionAgent().refine(MissionIntent("observe_scene", constraints={"direction": "left"}),
                                     agent.skill_client, agent.jev_agent, "jev_primary")
    assert result.question and not result.plan


def test_failed_camera_step_does_not_advance_or_return_home(agent):
    use_intent(agent, MissionIntent("observe_scene", constraints={"direction": "left"}))
    agent.skill_client.hook = lambda name, *_: make_result(name, False, codes.FAILED_TIMEOUT, "no feedback") if name == "set_camera_pose" else None
    state = agent.run("look left")
    assert state.status == "FAILED_SKILL"
    assert len(agent.skill_client.calls) == 2
    assert state.tasks[0]["goal_satisfied"] is False


def test_explicit_cleanup_preserves_unmet_search_result(agent):
    use_intent(agent, MissionIntent("sequence", constraints={"explicit_continue_after_not_found": True,
        "steps": [MissionIntent("search_object", object_query="red box").to_dict(),
                  MissionIntent("set_arm_pose", target="home").to_dict()]}))
    agent.skill_client.hook = lambda name, *_: make_result(name, False, codes.NEEDS_HUMAN, "not found", metrics={"inference_completed": True}) if name == "search_at_place" else None
    state = agent.run("find red box then arm home regardless")
    assert state.status == "DONE" and state.outcome == "PARTIAL"
    assert [t["goal_satisfied"] for t in state.tasks] == [False, True]


def test_delayed_mode_change_does_not_change_current_mission(agent):
    use_intent(agent, MissionIntent("sequence", constraints={"steps": [
        MissionIntent("observe_scene", constraints={"direction": "left"}).to_dict(),
        MissionIntent("set_arm_pose", target="home").to_dict()]}))
    agent.skill_client.hook = lambda *_: setattr(agent, "agent_mode", "jev_primary")
    state = agent.run("look left then home")
    assert state.status == "DONE" and state.agent_mode == "classic"
    assert state.jev_trace["total"]["calls"] == 0


def test_unregistered_navigation_target_is_delegated_without_motion(agent):
    agent.skill_client.semantic = SimpleNamespace(semantic_map=SimpleNamespace(
        find_matches=lambda _: [], resolve=lambda *_: None))
    state = agent.run("go to unknown door")
    assert state.status == "NEEDS_HUMAN"
    assert state.tasks[0]["owner"] == "navigation"
    assert not agent.skill_client.calls


def test_task_completion_uses_latest_attempt_without_losing_history(agent):
    state = MissionState("status")
    state.tasks = [{"task_id": "t1"}]
    state.skill_results = [
        {"task_id": "t1", "step_id": "t1:s1", "success": False, "code": codes.FAILED_TIMEOUT},
        {"task_id": "t1", "step_id": "t1:s1", "success": True, "code": codes.OK}]
    agent._complete_task(state, "t1")
    assert state.tasks[0]["goal_satisfied"] is True
    assert len(state.skill_results) == 2


def test_recovery_result_does_not_skip_failed_navigation_gate(agent):
    use_intent(agent, MissionIntent("navigate_to", target="door", constraints={"visual_confirmation": False}))
    attempts = []
    def hook(name, *_):
        if name == "check_localization_health":
            attempts.append(name)
            if len(attempts) == 1:
                return make_result(name, False, codes.FAILED_LOCALIZATION, "lost")
    agent.skill_client.hook = hook
    agent._localization_requires_recovery_from_metrics = lambda _: True
    agent._run_health_recovery = lambda *a, **kw: True
    state = agent.run("go to door")
    assert state.status == "DONE" and len(attempts) == 2
    assert state.tasks[0]["goal_satisfied"] is True


@pytest.mark.parametrize("cancel", [False, True])
def test_perception_substitute_resumes_same_mission_or_cancels(agent, cancel, monkeypatch):
    # Test an explicitly supplied proposal, not a production box->suitcase alias.
    from robot_agent.agents import perception_agent as module
    config = module.target_config()
    config["objects"]["box"]["fallback_classes"] = ["suitcase"]
    monkeypatch.setattr(module, "target_config", lambda: config)
    use_intent(agent, MissionIntent("sequence", constraints={"steps": [
        MissionIntent("set_arm_pose", target="home").to_dict(),
        MissionIntent("search_object", object_query="box").to_dict()]}))
    agent.enable_human_resume = True
    ready = threading.Event()
    agent.state_cb = lambda state: ready.set() if state.human_wait else None
    results = []
    worker = threading.Thread(target=lambda: results.append(agent.run("home then box")), daemon=True)
    worker.start()
    try:
        assert ready.wait(3)
        wait = dict(agent._human_wait)
        assert not agent.human_action({**wait, "wait_id": "old", "action": wait["actions"][0]})
        if cancel:
            agent.cancel(wait["mission_id"])
        else:
            assert agent.human_action({**wait, "action": wait["actions"][0]})
        worker.join(3)
        assert not worker.is_alive()
        assert results[0].mission_id == wait["mission_id"]
        assert results[0].status == ("CANCELLED" if cancel else "DONE")
        assert len(results[0].tasks) == 2
        assert [c[0] for c in agent.skill_client.calls].count("set_pose") == 1
        searches = [c for c in agent.skill_client.calls if c[0] == "search_at_place"]
        assert len(searches) == (0 if cancel else 1)
        if searches:
            assert searches[0][1]["checks"][0]["query"] == "suitcase"
        assert not agent.human_action({**wait, "action": wait["actions"][0]})
    finally:
        if agent._human_wait:
            agent.cancel(agent._human_wait["mission_id"])
        worker.join(2)


def test_scene_interpretation_routes_to_perception_without_false_success(agent):
    state = agent.run("check left side of robot if anything dangerous?")
    assert state.status == "NEEDS_HUMAN"
    assert state.tasks[0]["owner"] == "perception"
    assert state.tasks[0]["outcome"] == "UNSUPPORTED"
    assert "API not configured" in state.question_for_human
    assert not agent.skill_client.calls


def test_primary_partial_perception_does_not_wake_error_supervision(agent):
    agent.set_agent_mode("jev_primary")
    agent.jev_agent.supervise_step = lambda **kw: pytest.fail("PARTIAL is not an execution fault")
    agent.skill_client.hook = lambda name, *_: make_result(name, True, codes.OK, "partial", metrics={
        "outcome": "PARTIAL", "goal_satisfied": False, "inference_completed": True}) if name == "search_at_place" else None
    state = agent.run("find red box")
    assert state.status == "DONE" and state.outcome == "PARTIAL"
