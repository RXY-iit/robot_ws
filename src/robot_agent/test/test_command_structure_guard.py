import pytest

from robot_agent.parsers.deterministic_parser import DeterministicParser, command_guard_reason
from robot_agent.parsers.hybrid_parser import HybridParser
from robot_agent.jev_agent import JevStrengthenAgent


@pytest.mark.parametrize("command", ["go to", "navigate to", "find", "search for", "set pose",
    "go back ...", "if you find a red box, go back 20cm; otherwise turn left",
    "if you find bottle, go back 20cm and do something else; otherwise stay still",
    "go back 20cm, then turn left, then set pose home, then look right"])
def test_unsupported_structure_never_defaults_to_motion(monkeypatch, command):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    assert command_guard_reason(command)
    assert DeterministicParser().parse(command).task_type == "ask_human"
    # Early guard must not touch embedding/LLM providers.
    assert object.__new__(HybridParser).parse(command).task_type == "ask_human"
    assert JevStrengthenAgent().parse_command(command).intent.task_type == "ask_human"


@pytest.mark.parametrize("command", ["go back a little, 左を見て", "go back a little and see left"])
def test_mixed_language_sequence_preserves_both_tasks(command):
    intent = DeterministicParser().parse(command)
    assert intent.task_type == "sequence"
    assert [s["task_type"] for s in intent.constraints["steps"]] == ["move_relative", "look_direction"]


def test_unconditional_cleanup_is_policy_not_pose_name():
    from robot_agent.agents.mission_agent import MissionAgent
    from robot_agent.schemas import MissionState, SkillCall
    intent = DeterministicParser().parse("find red box, then set pose home regardless of whether it is found")
    assert intent.constraints["steps"][1]["target"] == "home"
    state = MissionState(user_text=intent.raw_text, intent=intent, plan=[SkillCall("detect_known_object"),
        SkillCall("await_robot_stationary"), SkillCall("set_pose", {"name": "home"})])
    result = {"skill_name": "detect_known_object", "metrics": {"inference_completed": True}}
    assert MissionAgent._explicit_cleanup_after_not_found(state, 0, result)
    result["metrics"]["inference_completed"] = False
    assert not MissionAgent._explicit_cleanup_after_not_found(state, 0, result)
    state.skill_results = [{"success": False, "continued_after_failure": True}]
    assert "Some goals remain unmet" in MissionAgent._final_message(state)


def test_supported_detection_condition_is_preserved(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    command = "if you find a red box, go back 20cm; otherwise stay still"
    for intent in (DeterministicParser().parse(command), JevStrengthenAgent().parse_command(command).intent):
        assert intent.task_type == "sequence"
        assert intent.constraints["else_action"] == "stay_still"
        assert intent.constraints["steps"][1]["constraints"]["run_if"] == "previous_search_found"
    assert JevStrengthenAgent._split_command_steps(command, 2) == ["find a red box", "go back 20cm"]
    assert JevStrengthenAgent._split_command_steps(command, 1) == []
    graph = JevStrengthenAgent._task_graph_from_intent(intent)
    assert graph["connector"] == "if_found"
    assert graph["steps"][1]["dependency_role"] == "conditional_on_previous_search_found"


def test_live_non_robot_rejection_does_not_keep_local_fallback_reason(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    jev = JevStrengthenAgent()
    monkeypatch.setattr(jev, "_decide", lambda *a: {"answers": {
        "is_robot_command": {"probabilities": {"true": .04, "false": .96}},
        "command_shape": {"choice": "ambiguous_or_unsupported"}}})
    result = jev.parse_command("今日の天気は？")
    assert result.intent.task_type == "ask_human"
    assert result.intent.constraints["ambiguity"]["reason"] != "jev_local_fallback_low_confidence"


def test_jev_slot_extraction_strips_cleanup_qualifier(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    intent = agent._intent_for_jev_task("set_pose", "set pose home regardless of whether it is found", {}, 2)
    assert intent.target == "home"
    assert agent._split_command_steps("go back a little, 左を見て", 2) == ["go back a little", "左を見て"]
    assert agent._split_command_steps("go back, then turn left, then set pose home", 2) == []
