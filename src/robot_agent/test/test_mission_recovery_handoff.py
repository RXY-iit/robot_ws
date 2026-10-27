from robot_agent.agents.mission_agent import MissionAgent
from robot_agent.schemas import MissionState, SkillCall
from robot_skills import result_codes as codes


def make_agent():
    import threading
    agent = object.__new__(MissionAgent)
    agent._cancel_lock = threading.Lock()
    agent._cancelled_mission_ids = set()
    agent._recovery_lock = threading.Condition()
    agent._recovery_responses = {}
    agent._recovery_pending = {}
    agent._state = lambda _: None
    agent._event = lambda *_: None
    agent._health_summary = lambda: {"components": {"localization": {"details": {
        "localization_warning": "POSSIBLE_STATIONARY_DRIFT",
        "localization_requires_recovery": True,
        "stationary_drift_m": .4,
    }}}}
    return agent


def test_wait_consumes_intermediate_then_final_and_ignores_late_response():
    import json
    from types import SimpleNamespace
    agent = make_agent()
    state = MissionState(user_text="navigate")
    def publish(msg):
        req = json.loads(msg.data)
        for response in ({"request_id": "wrong", "status": "RESOLVED", "final": True},
                         {"request_id": req["request_id"], "mission_id": "wrong-mission", "status": "RESOLVED", "final": True},
                         {"request_id": req["request_id"], "status": "IN_PROGRESS", "final": False},
                         {"request_id": req["request_id"], "status": "RESOLVED", "final": True}):
            agent._on_recovery_response(SimpleNamespace(data=json.dumps(response)))
    agent._recovery_request_pub = SimpleNamespace(publish=publish)
    result = agent._publish_recovery_request_and_wait(state, {"request_id": "r1"}, .5)
    assert result["status"] == "RESOLVED"
    agent._on_recovery_response(SimpleNamespace(data='{"request_id":"r1","status":"RESOLVED"}'))
    assert agent._recovery_responses == {}
    assert agent._recovery_pending == {}


def test_cancel_during_recovery_wait_exits_without_resuming():
    import json
    import time
    from types import SimpleNamespace
    agent = make_agent()
    state = MissionState(user_text="navigate")
    def publish(msg):
        agent._on_recovery_response(SimpleNamespace(data=json.dumps({"request_id": "r1", "status": "IN_PROGRESS", "final": False})))
        agent._cancelled_mission_ids.add(state.mission_id)
    agent._recovery_request_pub = SimpleNamespace(publish=publish)
    started = time.monotonic()
    result = agent._publish_recovery_request_and_wait(state, {"request_id": "r1"}, 180)
    assert time.monotonic() - started < .5
    assert result["status"] == "CANCELLED"
    assert not agent._handle_recovery_response(state, {"status": "RESOLVED", "advice": "resume_from_step"})
    assert state.status == "CANCELLED"
    assert agent._recovery_pending == {}


def test_health_recovery_includes_skill_metrics_and_preserves_response():
    agent = make_agent()
    state = MissionState(user_text="go to shelf_06", plan=[SkillCall(name="goto_pose")])
    requests = []

    def respond(_state, request):
        requests.append(request)
        return {"status": "NEEDS_HUMAN", "summary": "Trusted pose missing; set RViz pose"}

    agent._publish_recovery_request_and_wait = respond
    assert agent._run_health_recovery(state) is False
    assert requests[0]["step"]["skill"] == "goto_pose"
    assert requests[0]["result"]["metrics"]["stationary_drift_m"] == .4
    assert state.final_message == "Trusted pose missing; set RViz pose"


def test_missing_recovery_agent_does_not_call_legacy_reset_skill():
    agent = make_agent()  # Deliberately has no skill client: must not command anything.
    state = MissionState(user_text="go to shelf_06")
    result = agent._run_localization_recovery(state)
    assert result.code == codes.NEEDS_HUMAN
    assert not result.success
    assert state.skill_results == []


def test_cancel_text_uses_control_path_even_without_worker():
    import threading
    from types import SimpleNamespace
    from std_msgs.msg import String
    from robot_agent.nodes.agent_node import AgentNode
    requests = []
    node = SimpleNamespace(_active_lock=threading.Lock(), _active_thread=None,
                           _is_cancel_text=AgentNode._is_cancel_text,
                           _request_cancel_locked=requests.append)
    commands = ["cancel", "キャンセル", "stop", "Cancel task", "Cancel all", "全部取消"]
    for text in commands:
        AgentNode._command_cb(node, String(data=text))
    assert requests == commands
    assert not AgentNode._is_cancel_text("cancel step")
    assert not AgentNode._is_cancel_text("cancel current step")
    assert not AgentNode._is_cancel_text("go to bus stop")
    assert not AgentNode._is_cancel_text("find red box then stop")


def test_cancel_waiting_human_without_worker_closes_mission():
    import threading
    from types import SimpleNamespace
    from robot_agent.nodes.agent_node import AgentNode
    state = MissionState(user_text='go to shelf_06', status='NEEDS_HUMAN', question_for_human='Set pose')
    agent = make_agent()
    agent._cancel_lock = threading.Lock()
    agent._cancelled_mission_ids = {state.mission_id}
    published = []
    agent._state = published.append
    agent.cancel = lambda _: SimpleNamespace(to_dict=lambda: {})
    node = SimpleNamespace(_current_mission_id=state.mission_id, _last_mission_id=None,
                           _last_mission_state=state, _active_thread=None,
                           human_prompt_pub=SimpleNamespace(publish=lambda _: None),
                           mission_agent=agent, _publish_event=lambda _: None, _publish_status=lambda _: None)
    AgentNode._request_cancel_locked(node)
    assert published[-1].status == 'CANCELLED'
    assert published[-1].question_for_human is None
    assert state.status == 'NEEDS_HUMAN'  # copy, no mutation of old published state
