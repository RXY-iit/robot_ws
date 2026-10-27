import threading
from types import SimpleNamespace
import pytest

from robot_agent.recovery_supervision import RecoverySupervision
from robot_agent.schemas import MissionState, SkillCall
from test_mission_recovery_handoff import make_agent


HOLD = {"incident_id": "loc-1", "capabilities": ["auto_navigation"], "action": "pause_dependent"}


def update(supervision, constraints=(), ready=True, verified=()):
    supervision.update({"task_supervision": {"ready": ready, "constraints": list(constraints),
                                           "verified_capabilities": list(verified)}})


def test_only_dependent_skills_are_held_and_verification_is_required_to_release():
    supervision = RecoverySupervision()
    update(supervision, [HOLD])
    assert supervision.blockers(["localization"])
    assert not supervision.blockers(["perception", "arm", "relative_motion"])
    update(supervision)  # An empty incident list alone is not proof of recovery.
    assert supervision.blockers(["auto_navigation"])
    update(supervision, verified=["auto_navigation"])
    assert not supervision.blockers(["auto_navigation"])


def test_suspended_or_malformed_state_cannot_release_hold():
    supervision = RecoverySupervision()
    update(supervision, [HOLD])
    update(supervision, ready=False, verified=["auto_navigation"])
    supervision.update({})
    supervision.update({"task_supervision": {"ready": True, "constraints": "invalid"}})
    assert supervision.blockers(["auto_navigation"])
    assert not supervision.fresh()


def test_heartbeat_expiry_is_not_recovery():
    supervision = RecoverySupervision()
    supervision.update({"task_supervision": {"ready": True, "constraints": [HOLD],
                                           "verified_capabilities": []}}, now=10)
    assert supervision.fresh(now=14)
    assert not supervision.fresh(now=16)
    assert supervision.blockers(["auto_navigation"])


def waiter_agent():
    agent = make_agent()
    agent.recovery_supervision = RecoverySupervision()
    agent.skill_client = SimpleNamespace(navigation=SimpleNamespace(recovery_motion_stopped=lambda: True))
    return agent


def test_release_waits_for_motion_ack_and_keeps_original_step():
    agent = waiter_agent()
    update(agent.recovery_supervision, [HOLD])
    state = MissionState(user_text="navigate", plan=[SkillCall(name="goto_pose", payload={"pose": {"x": 1}})])
    paused = threading.Event()
    states = []
    def publish(s):
        states.append(s.to_dict())
        if s.recovery_pause and s.recovery_pause["motion_stopped"]:
            paused.set()
    agent._state = publish
    result = []
    worker = threading.Thread(target=lambda: result.append(agent._wait_for_capability_recovery(
        state, state.plan[0], timeout_sec=2)))
    worker.start()
    assert paused.wait(1)
    update(agent.recovery_supervision, verified=["auto_navigation"])
    worker.join(1)
    assert result == [True] and state.status == "RESUMING"
    assert state.plan[0].payload == {"pose": {"x": 1}}
    assert state.current_step_index == 0 and state.recovery_pause is None
    assert states[0]["recovery_pause"]["motion_stopped"] is False


def test_clear_constraints_without_goal_terminal_ack_cannot_resume():
    agent = waiter_agent()
    agent.skill_client.navigation.recovery_motion_stopped = lambda: False
    update(agent.recovery_supervision, verified=["auto_navigation"])
    state = MissionState(user_text="navigate")
    assert not agent._wait_for_capability_recovery(state, SkillCall(name="goto_pose"),
                                                  interrupted=[HOLD], timeout_sec=.01)
    assert state.status == "NEEDS_HUMAN"


def test_cancelled_mission_is_never_released_by_later_verified_state():
    agent = waiter_agent()
    update(agent.recovery_supervision, [HOLD])
    state = MissionState(user_text="navigate")
    agent._cancelled_mission_ids.add(state.mission_id)
    assert not agent._wait_for_capability_recovery(state, SkillCall(name="goto_pose"), timeout_sec=.1)
    assert state.status == "CANCELLED"
    update(agent.recovery_supervision, verified=["auto_navigation"])
    assert state.status == "CANCELLED"


def test_unrelated_task_does_not_wait_for_localization():
    agent = waiter_agent()
    update(agent.recovery_supervision, [HOLD])
    state = MissionState(user_text="observe", status="EXECUTE_STEP")
    assert agent._wait_for_capability_recovery(state, SkillCall(name="capture_observation"))
    assert state.status == "EXECUTE_STEP"


@pytest.mark.parametrize("always_interrupt", [False, True])
def test_mission_run_retries_same_goal_after_proactive_pause_without_failure_recovery(always_interrupt):
    from robot_agent.schemas import MissionIntent
    from robot_skills import result_codes as codes
    from robot_skills.executors.base import make_result
    agent = waiter_agent()
    agent.agent_mode = "classic"
    agent.parser = SimpleNamespace(parse=lambda _: MissionIntent(task_type="navigate_to"))
    agent.task_catalog = SimpleNamespace(normalize=lambda intent: intent)
    call = SkillCall(name="goto_pose", payload={"pose": {"x": 1, "y": 2, "yaw": 0}})
    agent._build_plan = lambda _: [call]
    agent._annotate_plan_with_catalog = lambda *_: None
    agent._annotate_call_with_catalog = lambda *_: None
    agent._handle_health_before_step = lambda *_: True
    agent._final_message = lambda _: "done"
    events, calls, states = [], [], []
    agent._event = lambda _, kind, data: events.append(kind)
    def publish(state):
        states.append(state.to_dict())
        if state.recovery_pause and state.recovery_pause["motion_stopped"]:
            update(agent.recovery_supervision, verified=["auto_navigation"])
    agent._state = publish
    def execute(name, payload, context):
        calls.append((name, payload.copy()))
        if len(calls) == 1 or always_interrupt:
            update(agent.recovery_supervision, [HOLD])
            assert context.recovery_check()
            return make_result(name, False, codes.CANCELLED, "supervisor pause")
        return make_result(name, True, codes.OK, "arrived")
    agent.skill_client.execute_skill = execute
    agent.skill_client.context = lambda *_: SimpleNamespace()
    result = agent.run("navigate")
    assert result.status == ("NEEDS_HUMAN" if always_interrupt else "DONE")
    assert len(calls) == (3 if always_interrupt else 2)
    assert all(call == calls[0] for call in calls)
    assert "recovery_task_interrupted" in events and "recovery_task_released" in events
    assert "recovery_request" not in events
    assert any(s["status"] == "PAUSED_HEALTH" for s in states)
