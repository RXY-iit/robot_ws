import threading
import time
from types import SimpleNamespace

import pytest

from robot_agent.schemas import MissionState, SkillCall
from test_recovery_supervision import waiter_agent, update


def test_resume_verdict_requires_new_explicit_capability_evidence():
    agent = waiter_agent()
    supervision = agent.recovery_supervision
    before = time.monotonic()
    update(supervision, verified=["auto_navigation"])
    assert supervision.resume_verified(["auto_navigation"], before)
    assert not supervision.resume_verified(["auto_navigation"], time.monotonic())
    update(supervision)  # No incident is not a positive healthy verdict.
    assert not supervision.resume_verified(["auto_navigation"], before)
    update(supervision, verified=["auto_navigation"], ready=False)
    assert not supervision.resume_verified(["auto_navigation"], before)


@pytest.mark.parametrize("recovery_allowed,stopped,cancel", [(True, True, False), (False, True, False), (True, False, True)])
def test_wait_accepts_recovery_or_human_but_requires_stop_for_automatic_resume(recovery_allowed, stopped, cancel):
    agent = waiter_agent()
    agent.enable_human_resume = True
    agent._human_condition = threading.Condition()
    agent._human_wait = agent._human_action = None
    agent.request_auto_mode = None
    agent.skill_client.system = SimpleNamespace(robot_mode="AUTO", _last_robot_mode_time=time.monotonic())
    agent.skill_client.navigation.recovery_motion_stopped = lambda: stopped
    agent._health_summary = lambda: {"components": {"localization": {"level": "OK"}}}
    events = []
    agent._event = lambda state, kind, payload: events.append(kind)
    state = MissionState(user_text="waypoints")
    call = SkillCall("goto_pose", step_id="s1", payload={"pose": {"x": 1, "y": 2, "yaw": 0}})
    result = []
    worker = threading.Thread(target=lambda: result.append(agent._wait_for_step_repair(
        state, call, recovery_resume=recovery_allowed)), daemon=True)
    worker.start()
    try:
        deadline = time.monotonic() + 1
        while agent._human_wait is None and time.monotonic() < deadline:
            time.sleep(.01)
        update(agent.recovery_supervision, verified=["auto_navigation"])
        if recovery_allowed and stopped:
            worker.join(2.8)
            assert result == [True]
            assert "mission_recovery_resume_authorized" in events
            assert "mission_human_action" not in events
            assert call.catalog["recovery_verified_resume_count"] == 1
        else:
            time.sleep(.25)
            assert not result
            if cancel:
                agent._cancelled_mission_ids.add(state.mission_id)
            else:
                agent._human_action = "resume"
            worker.join(1)
            assert result == [not cancel]
            assert "mission_recovery_resume_authorized" not in events
    finally:
        agent._cancelled_mission_ids.add(state.mission_id)
        worker.join(1)


def test_recovery_auto_resume_budget_exhaustion_stays_human_only():
    agent = waiter_agent()
    agent.enable_human_resume = False
    call = SkillCall("goto_pose", catalog={"recovery_verified_resume_count": 2})
    assert not agent._wait_for_step_repair(MissionState(user_text="route"), call, recovery_resume=True)


def test_full_waypoint_mission_continues_after_stale_localization_without_human_action():
    from test_domain_tasks import FakeClient
    from robot_agent.agents.mission_agent import MissionAgent
    from robot_skills.executors.base import make_result
    from robot_skills import result_codes as codes
    client = FakeClient()
    client.semantic.semantic_map.data["map_id"] = "sim_factory_mid360_like_gpu_trueimu_011"
    client.system = SimpleNamespace(robot_mode="AUTO", _last_robot_mode_time=time.monotonic())
    client.navigation = SimpleNamespace(recovery_motion_stopped=lambda: True)
    attempts = []
    def inject(name, payload, context):
        if name == "goto_pose":
            attempts.append(payload["pose"])
            if len(attempts) == 1:
                return make_result(name, False, codes.FAILED_LOCALIZATION, "GICP_SCORE_STALE")
    client.hook = inject
    events = []
    agent = MissionAgent(client, event_cb=lambda event: events.append(event["type"]))
    agent.enable_human_resume = True
    def on_state(state):
        if state.human_wait and state.human_wait.get("recovery_resume_allowed"):
            update(agent.recovery_supervision, verified=["auto_navigation"])
    agent.state_cb = on_state
    state = agent.run("沿着 waypoint list a 走行")
    assert state.status == "DONE"
    assert len(attempts) == 8 and attempts[0] == attempts[1]
    assert "mission_recovery_resume_authorized" in events
    assert "mission_human_action" not in events
