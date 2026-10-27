import pytest

from robot_agent.command_grounding import tokens, binding_questions, numeric_parameter
from robot_agent.jev_agent import JevStrengthenAgent
from robot_agent.schemas import MissionIntent, SkillCall


def span(text, fragment, prefix):
    start = text.index(fragment)
    end = start + len(fragment)
    words = tokens(text)
    return {prefix + "_start": next(w["id"] for w in words if w["start"] == start),
            prefix + "_end": next(w["id"] for w in words if w["end"] == end)}


def response(answers):
    return {"answers": {k: {"type": "choice", "choice": v} for k, v in answers.items()},
            "usage": {"input_tokens": 123}}


@pytest.mark.parametrize("text,task", [
    ("robot status", "get_robot_status"),
    ("list known targets", "list_known_targets"),
])
def test_read_only_queries_are_linear_tasks(monkeypatch, text, task):
    result, calls = parse(monkeypatch, text, [task], [text], {})
    assert result.intent.task_type == task
    shape = calls[0][2]["command_shape"]
    assert "read-only" in shape["criteria"]["linear"]
    assert "Read-only robot status" in shape["instructions"]
    assert not any("goal_start" in key for key in calls[1][2])


def test_task_count_and_positions_share_action_boundaries():
    from robot_agent.command_grounding import structure_questions
    from robot_agent.task_catalog import TaskCatalog
    questions = structure_questions(TaskCatalog())
    for key in ["task_count", "task_1_type", "task_2_type", "task_3_type"]:
        assert "Count requested actions, not target nouns." in questions[key]["instructions"]
        assert "OR alternatives is one" in questions[key]["instructions"]


@pytest.mark.parametrize("goal", [
    "cup or bottle", "yellow car or red bottle", "a yellow vehicle and a red bottle",
])
def test_one_perception_task_preserves_all_target_alternatives(monkeypatch, goal):
    text = "find " + goal
    result, calls = parse(monkeypatch, text, ["search_object"], [text],
                          span(text, goal, "task_1_goal"))
    assert result.intent.object_query == goal
    assert "ENTIRE AND/OR expression" in calls[1][2]["task_1_goal_end"]["instructions"]
    assert result.intent.constraints["source_span"]["end"] == len(text)


def parse(monkeypatch, command, types, fragments, extras, flow="linear", alternatives=None, clarify=None):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    struct = {"command_shape": flow, "task_count": {1: "one", 2: "two", 3: "three"}[len(types)]}
    bindings = {}
    for i in range(1, 4):
        struct[f"task_{i}_type"] = types[i - 1] if i <= len(types) else "absent"
        struct[f"task_{i}_alternative_type"] = (alternatives or {}).get(i, "none")
    for i, fragment in enumerate(fragments, 1):
        bindings.update(span(command, fragment, f"task_{i}"))
        bindings[f"task_{i}_dependency"] = "independent"
    bindings.update(extras)
    calls = []

    def decide(stage, state, questions):
        calls.append((stage, state, questions))
        return response(struct if stage == "command_parse" else bindings)

    monkeypatch.setattr(agent, "_decide", decide)
    return agent.parse_command(command, clarify_task=clarify), calls


@pytest.mark.parametrize("selected", ["set_arm_pose", "navigate_to"])
def test_ambiguity_selection_is_authoritative_but_still_requires_binding(monkeypatch, selected):
    text = "set pose home"
    prompts = []
    def clarify(ordinal, candidates, trace):
        prompts.append((ordinal, candidates, trace.to_dict()))
        return selected
    result, calls = parse(monkeypatch, text, ["navigate_to"], [text],
        span(text, "home", "task_1_goal"), alternatives={1: "set_arm_pose"}, clarify=clarify)
    assert result.intent.task_type == selected
    assert result.intent.target == "home"
    assert len(prompts) == 1
    assert {c["task_type"] for c in prompts[0][1]} == {"set_arm_pose", "navigate_to"}
    assert not any("type_choice_probability" in c for c in prompts[0][1])
    assert calls[1][1]["human_confirmed_task_types"] == {"1": selected}
    assert result.intent.constraints["human_confirmed_task_type"] == selected


def test_ambiguity_without_operator_never_binds_or_picks_winner(monkeypatch):
    result, calls = parse(monkeypatch, "set pose home", ["navigate_to"], ["set pose home"], {}, alternatives={1: "set_arm_pose"})
    assert result.intent.task_type == "ask_human"
    assert len(calls) == 1
    assert len(result.intent.constraints["ambiguity"]["interpretations"]) == 2


def test_near_tied_distribution_prompts_without_claiming_two_high_confidences(monkeypatch):
    from robot_agent.command_grounding import interpretation_candidates
    from robot_agent.task_catalog import TaskCatalog
    result = interpretation_candidates(TaskCatalog(), 1, "set_arm_pose", "none", {"answers": {
        "task_1_type": {"probabilities": {"set_arm_pose": .48, "navigate_to": .45, "unknown": .07}}}})
    assert {r["task_type"] for r in result} == {"set_arm_pose", "navigate_to"}
    assert [r["type_choice_probability"] for r in result] == [.48, .45]


@pytest.mark.parametrize("alternative", ["navigate_to", "none", "set_arm_pose", "unclear", None])
def test_duplicate_interpretation_uses_bounded_followup(monkeypatch, alternative):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    calls = []
    text = "set pose home"
    def decide(stage, state, questions):
        calls.append(stage)
        if stage == "command_parse":
            return response({"command_shape": "linear", "task_count": "one",
                "task_1_type": "set_arm_pose", "task_1_alternative_type": "set_arm_pose",
                "task_2_type": "absent", "task_2_alternative_type": "none",
                "task_3_type": "absent", "task_3_alternative_type": "none"})
        if stage == "command_disambiguation":
            assert state["primary_task_types"] == ["set_arm_pose"]
            assert "set_arm_pose" not in questions["task_1_alternative_type"]["criteria"]
            return response({"task_1_alternative_type": alternative}) if alternative else {"_error": "timeout"}
        assert stage == "command_bind"
        return response({**span(text, text, "task_1"), **span(text, "home", "task_1_goal"), "task_1_dependency": "independent"})
    monkeypatch.setattr(agent, "_decide", decide)
    result = agent.parse_command(text)
    assert calls[:2] == ["command_parse", "command_disambiguation"]
    assert result.additional_traces[0].stage == "command_disambiguation"
    if alternative == "none":
        assert result.intent.task_type == "set_arm_pose"
        assert calls[-1] == "command_bind"
    else:
        assert result.intent.task_type == "ask_human" and len(calls) == 2
        if alternative == "navigate_to":
            assert {c["task_type"] for c in result.intent.constraints["ambiguity"]["interpretations"]} == {"set_arm_pose", "navigate_to"}


def test_cancel_after_structure_does_not_start_binding(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    calls = []
    monkeypatch.setattr(agent, "_decide", lambda stage, *_: calls.append(stage) or response({}))
    result = agent.parse_command("go back 20cm", is_cancelled=lambda: True)
    assert calls == ["command_parse"]
    assert result.intent.task_type == "ask_human"
    assert result.intent.constraints["ambiguity"]["reason"] == "parse_cancelled"


def test_review_cannot_select_classic_that_reverses_human_choice(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    selected = MissionIntent("set_arm_pose", target="home", constraints={"human_confirmed_task_types": {"1": "set_arm_pose"}})
    wrong = MissionIntent("navigate_to", target="home")
    plan = lambda i: [SkillCall(i.task_type, kind="task", payload=i.to_dict())]
    monkeypatch.setattr(agent, "_decide", lambda *_: response({
        "classic_graph_faithful": "faithful", "plan_source_decision": "use_classic_plan"}))
    assert agent.plan_review("set pose home", selected, plan(selected), {}, classic_intent=wrong,
                             classic_plan=plan(wrong)).adopted_decision == "ask_human"


@pytest.mark.parametrize("connectors", [("next", "after that"), ("subsequently", "finally"), ("その後", "最后")])
def test_three_tasks_are_bound_without_connector_rules(monkeypatch, connectors):
    command = f"go back 0.2 meter, {connectors[0]} turn left, {connectors[1]} set pose home"
    result, calls = parse(monkeypatch, command, ["move_relative", "rotate_relative", "set_arm_pose"],
        ["go back 0.2 meter", "turn left", "set pose home"], {
            "task_1_direction": "back", "task_1_distance": "explicit", "task_2_rotation": "left", "task_2_turn_direction": "left",
            **span(command, "home", "task_3_goal")})
    assert result.intent.task_type == "sequence"
    steps = result.intent.constraints["steps"]
    assert steps[0]["constraints"]["distance_m"] == .2
    assert steps[1]["constraints"]["yaw_deg"] == 90
    assert steps[2]["target"] == "home"
    assert [c[0] for c in calls] == ["command_parse", "command_bind"]
    assert not any("pose_name" in q or "single_step" in q or "parse_confidence" in q for c in calls for q in c[2])
    assert len(result.additional_traces) == 1
    assert sum(t.input_tokens_est for t in [result.trace, *result.additional_traces]) == 246
    assert "classic" not in calls[0][1]


def test_conditional_is_graph_not_fixed_if_else_regex(monkeypatch):
    text = "search red box; provided it is found, retreat 20cm; if not, remain still"
    result, _ = parse(monkeypatch, text, ["search_object", "move_relative"],
        ["search red box", "retreat 20cm"], {**span(text, "red box", "task_1_goal"),
            "task_2_direction": "back", "task_2_distance": "explicit", "task_2_dependency": "success_1"}, "if_found_else_hold")
    assert result.intent.constraints["else_action"] == "stay_still"
    second = result.intent.constraints["steps"][1]["constraints"]
    assert second["run_if"] == "previous_search_found" and second["depends_on"] == [1]
    assert second["distance_m"] == .2


@pytest.mark.parametrize("task,action,target", [
    ("set_arm_pose", "set arm pose home", "home"),
    ("navigate_to", "go to shelf_06", "shelf_06"),
    ("search_object", "find a bottle", "a bottle"),
])
def test_conditional_delegated_task_uses_same_evidence_gate(monkeypatch, task, action, target):
    text = f"if you find a red thing, {action}; otherwise stay still"
    result, calls = parse(monkeypatch, text, ["search_object", task],
        ["find a red thing", action], {
            **span(text, "a red thing", "task_1_goal"),
            **span(text, target, "task_2_goal"),
            "task_2_dependency": "success_1"}, "if_found_else_hold")
    assert result.intent.task_type == "sequence"
    second = result.intent.constraints["steps"][1]
    assert second["task_type"] == task
    assert second["constraints"]["depends_on"] == [1]
    assert second["constraints"]["run_if"] == "previous_search_found"
    assert "arm, navigation" in calls[0][2]["command_shape"]["criteria"]["if_found_else_hold"]


def test_conditional_arm_never_drops_success_dependency(monkeypatch):
    text = "if you find red, set arm pose home; otherwise stay still"
    result, _ = parse(monkeypatch, text, ["search_object", "set_arm_pose"],
        ["find red", "set arm pose home"], {
            **span(text, "red", "task_1_goal"), **span(text, "home", "task_2_goal"),
            "task_2_dependency": "independent"}, "if_found_else_hold")
    assert result.intent.task_type == "ask_human"


def test_per_task_success_and_cleanup_are_not_global(monkeypatch):
    text = "go to shelf_06, search red box there, set pose home"
    result, _ = parse(monkeypatch, text, ["navigate_to", "search_object", "set_arm_pose"],
        ["go to shelf_06", "search red box there", "set pose home"], {
            **span(text, "shelf_06", "task_1_goal"), **span(text, "red box", "task_2_goal"), **span(text, "home", "task_3_goal"),
            "task_2_dependency": "success_1", "task_3_dependency": "after_attempt"})
    steps = result.intent.constraints["steps"]
    assert steps[1]["constraints"]["depends_on"] == [1]
    assert steps[2]["constraints"]["depends_on"] == []
    assert steps[2]["constraints"]["explicit_after_attempt"]


@pytest.mark.parametrize("changes", [
    {"task_1_start": "w999"}, {"task_1_end": "w0"},
    {"task_1_goal_start": "w0"}, {"task_1_dependency": "success_3"},
])
def test_malformed_binding_never_executes(monkeypatch, changes):
    text = "please find red box"
    result, _ = parse(monkeypatch, text, ["search_object"], ["find red box"],
                      {**span(text, "red box", "task_1_goal"), **changes})
    assert result.intent.task_type == "ask_human"


def test_overlap_rejected(monkeypatch):
    text = "go back 20cm and turn left"
    result, _ = parse(monkeypatch, text, ["move_relative", "rotate_relative"], [text, "turn left"], {
        "task_1_direction": "back", "task_1_distance": "explicit", "task_2_rotation": "left", "task_2_turn_direction": "left"})
    assert result.intent.task_type == "ask_human"


@pytest.mark.parametrize("text,value", [("20cm", .2), ("200 mm", .2), ("0.2 meter", .2), ("２０センチメートル", .2)])
def test_numeric_units(text, value):
    assert numeric_parameter(text, "distance") == value


def test_multiple_numeric_values_not_silently_selected():
    with pytest.raises(ValueError):
        numeric_parameter("20cm or 40cm", "distance")


def test_partial_api_answer_and_outage_never_become_local_plan(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    monkeypatch.setattr(agent, "_decide", lambda *_: {"answers": {}})
    assert agent.parse_command("go back 20cm").intent.task_type == "ask_human"
    agent.api_configured = True
    monkeypatch.setattr(agent, "_decide", lambda *_: None)
    assert agent.parse_command("go back 20cm").intent.task_type == "ask_human"


def test_review_requires_concrete_graph_verdict_not_only_source_vote(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    intent = MissionIntent("move_relative", constraints={"direction": "back", "distance_m": .2})
    plan = [SkillCall("move_relative", payload=intent.to_dict(), kind="task")]
    monkeypatch.setattr(agent, "_decide", lambda *_: response({"plan_source_decision": "use_jev_plan", "jev_graph_faithful": "incorrect"}))
    assert agent.plan_review("go back 20cm then home", intent, plan, {}).adopted_decision == "ask_human"


def test_domain_tasks_do_not_get_motion_or_pose_slot_questions():
    questions = binding_questions(["navigate_to", "search_object", "set_arm_pose"], tokens("go to shelf and find red box then home"))
    assert not any("rotation" in k or "distance" in k or "pose_name" in k for k in questions)


def test_domain_api_outage_does_not_adopt_local_default(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    agent.api_configured = True
    monkeypatch.setattr(agent, "_decide", lambda *_: None)
    intent = MissionIntent("set_arm_pose", target="home")
    action, _ = agent.domain_review("arm_task_review", intent, {"execute": "execute", "ask_human": "ask"}, "execute", {})
    assert action == "ask_human"
    answers, _ = agent.domain_questions("test", intent, {"decision": {"type": "choice", "criteria": {"execute": "execute"}}}, {"decision": "execute"}, {})
    assert answers["decision"] == "unclear"


def test_failed_api_trace_does_not_claim_local_answers():
    entries = JevStrengthenAgent._questions_from_response({"_error": "timeout"},
        [{"id": "decision", "type": "choice", "result": "execute", "confidence": .99}])
    assert entries == [{"id": "decision", "type": "choice", "result": None, "answer_source": "api_error"}]


def test_non_scalar_api_answer_is_rejected_without_exception():
    assert JevStrengthenAgent._answer_value({"answers": {"decision": {"choice": ["execute"]}}}, "decision") is None


def test_local_parse_has_no_api_cost_and_source_words_survive_redaction(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    trace = agent.parse_command("go back 20cm").trace
    assert trace.provider == "local" and trace.input_cost_usd == 0 and trace.input_tokens_est == 0
    words = tokens("go back 20cm")
    assert agent._redact_for_log({"source_words": words, "api_key": "secret"}) == {
        "source_words": words, "api_key": "<redacted>"}


def test_review_compares_graphs_once_and_keeps_source_arguments(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    captured = []
    monkeypatch.setattr(agent, "_decide", lambda *args: captured.append(args) or response({
        "jev_graph_faithful": "faithful", "classic_graph_faithful": "incorrect", "plan_source_decision": "use_jev_plan"}))
    intent = MissionIntent("move_relative", raw_text="go back 20cm", constraints={"direction": "back", "distance_m": .2})
    plan = [SkillCall("move_relative", kind="task", payload=intent.to_dict())]
    assert agent.plan_review(intent.raw_text, intent, plan, {}).adopted_decision == "use_jev_plan"
    _, state, qs = captured[0]
    assert len(qs) == 3
    assert not {"jev_plan", "jev_slot_resolution", "health"} & state.keys()
    assert state["jev_task_graph"]["steps"][0]["source_text"] == intent.raw_text


@pytest.mark.parametrize("source", ["jev", "classic"])
def test_model_cannot_override_explicit_confirmation_requirement(monkeypatch, source):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    agent = JevStrengthenAgent()
    intent = MissionIntent("set_arm_pose", target="home", requires_confirmation=True)
    plan = [SkillCall("set_arm_pose", kind="task", payload=intent.to_dict())]
    monkeypatch.setattr(agent, "_decide", lambda *_: response({
        f"{source}_graph_faithful": "faithful", "plan_source_decision": f"use_{source}_plan"}))
    assert agent.plan_review("home", intent, plan, {}, classic_intent=intent, classic_plan=plan).adopted_decision == "ask_human"


def test_conflicting_rotation_choices_do_not_move(monkeypatch):
    text = "turn left"
    result, _ = parse(monkeypatch, text, ["rotate_relative"], [text],
                      {"task_1_rotation": "left", "task_1_turn_direction": "right"})
    assert result.intent.task_type == "ask_human"


@pytest.mark.parametrize("policy,refs,meaningful", [
    ("cleanup_after_attempt", [], True), ("ordered_but_independent", [], True),
    ("required_condition", [1], False), ("ordered_but_independent", [1], False),
])
def test_failed_task_supervision_uses_per_task_dependencies(policy, refs, meaningful):
    intent = MissionIntent("sequence", constraints={"dependency_policy": "per_task"})
    call = SkillCall("set_arm_pose", kind="task", payload={"constraints": {"dependency_policy": policy, "depends_on": refs}})
    assert JevStrengthenAgent._next_step_still_meaningful(intent, call) == meaningful
