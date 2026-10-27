from types import SimpleNamespace

import pytest

from robot_agent.agents.perception_agent import PerceptionAgent
from robot_agent.jev_agent import JevStrengthenAgent
from robot_agent.parsers.deterministic_parser import DeterministicParser
from robot_agent.schemas import MissionIntent, SkillCall
from robot_skills.skill_catalog import SkillCatalog


@pytest.fixture
def jev(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    return JevStrengthenAgent()


def refine(query, jev, mode="jev_primary", client=None):
    return PerceptionAgent().refine(MissionIntent("search_object", object_query=query),
                                    client or SimpleNamespace(), jev, mode)


def test_primary_directional_acquisition_requires_confirmed_camera_pose(jev):
    intent = MissionIntent("observe_scene", constraints={"direction": "left", "profile": "directional"})
    plan = PerceptionAgent().refine(intent, SimpleNamespace(), jev, "jev_primary").plan
    poses = [call for call in plan if call.name == "set_camera_pose"]
    assert len(poses) == 2
    assert all(call.payload["require_feedback"] and call.payload["wait_timeout_sec"] == 4 for call in poses)
    classic = PerceptionAgent().refine(intent, SimpleNamespace(), jev, "classic").plan
    assert all("require_feedback" not in call.payload for call in classic if call.name == "set_camera_pose")


def test_health_failure_skip_requests_recovery_instead_of_killing_mission(jev, monkeypatch):
    monkeypatch.setattr(jev, "_decide", lambda *args: {
        "answers": {"supervision_action": {"choice": "skip_failed_subgoal_continue_independent_goal"}}})
    result = jev.supervise_step(
        user_text="look left, then set arm pose home", intent=MissionIntent("observe_scene"),
        plan=[SkillCall("set_camera_pose"), SkillCall("set_pose")], step_index=0,
        result={"skill_name": "set_camera_pose", "success": False, "code": "FAILED_PRECHECK"},
        health={})
    assert result.action == "call_recovery_agent"
    assert result.trace.execution_gate
    assert next(q["result"] for q in result.trace.questions if q["id"] == "supervision_action") == "skip_failed_subgoal_continue_independent_goal"


@pytest.mark.parametrize("skill,component", [
    ("set_camera_pose", "camera_control"), ("capture_observation", "camera"),
    ("set_pose", "arm_gateway"), ("rotate_relative", "drive"), ("goto_pose", "local_planner"),
])
def test_timeout_recovery_routing_respects_failed_resource(skill, component):
    from robot_agent.agents.mission_agent import MissionAgent
    assert MissionAgent._suspected_components_from_result(
        {"skill_name": skill, "code": "FAILED_TIMEOUT"}) == [component]


def test_composite_camera_failure_routes_to_control_not_image_stream():
    from robot_agent.agents.mission_agent import MissionAgent
    assert MissionAgent._suspected_components_from_result({
        "skill_name": "search_at_place", "code": "FAILED_PRECHECK",
        "metrics": {"failed_child": "set_camera_pose"}}) == ["camera_control"]


def test_three_phase_contract_and_box_gap(jev):
    result = refine("red box", jev)
    assert [t.stage for t in result.traces] == ["perception_command_parse", "perception_plan_review"]
    payload = result.plan[0].payload
    assert [c["query"] for c in payload["checks"]] == ["color_red"]
    assert payload["unresolved_requirements"]
    assert any(q["id"] == "unit_count" and q["result"] == "2" for q in result.traces[0].questions)


@pytest.mark.parametrize("query,expected", [("shelf_label_06", "shelf_label_06"),
    ("06 letter", "text_06"), ("letter 06", "text_06"), ("text_AB", "text_AB"),
    ("the text 987654321", "text_987654321"), ("a label AB", "text_AB")])
def test_literal_ocr_contract_and_scoped_domain_command(jev, monkeypatch, query, expected):
    requests = []
    monkeypatch.setattr(jev, "_decide", lambda *args: requests.append(args) or None)
    intent = MissionIntent("search_object", object_query=query, raw_text="go to shelf_06")
    result = PerceptionAgent().refine(intent, SimpleNamespace(), jev, "jev_primary")
    assert result.plan[0].payload["checks"][0]["query"] == expected
    assert not result.plan[0].payload["unresolved_requirements"]
    assert requests[0][1]["task"]["raw_text"] == query
    assert set(requests[0][2]["kind_0"]["criteria"]) == {"ocr", "unknown"}
    assert intent.raw_text == "go to shelf_06"


@pytest.mark.parametrize("query", [
    "the red text 06", "the text 06 on the left", "a damaged label AB",
])
def test_ocr_determiner_support_preserves_extra_qualifiers(query):
    from robot_agent.agents.perception_plan import literal_ocr_query, target_config
    assert literal_ocr_query(query, target_config()) is None


def test_reviewed_ocr_kind_supplies_bare_numeric_context():
    from robot_agent.agents.perception_plan import compile_checks, parse_seed, target_config
    config = target_config()
    units, _, _ = parse_seed("987654321", config)
    checks, gaps, _ = compile_checks(units, config)
    assert not checks and gaps
    units[0]["kind"] = "ocr"
    checks, gaps, _ = compile_checks(units, config)
    assert not gaps and checks[0]["query"] == "text_987654321"
    units[0]["source"] = "987654321 on the left"
    checks, gaps, _ = compile_checks(units, config)
    assert not checks and gaps


def test_wrong_live_object_answer_cannot_override_literal_ocr_contract(jev, monkeypatch):
    def decide(stage, state, questions):
        choices = {"coverage": "complete", "relation": "all", "unit_count": "1",
                   "kind_0": "object", "object_0": "none", "color_0": "none"}
        return {"answers": {k: {"choice": choices[k]} for k in questions}}
    monkeypatch.setattr(jev, "_decide", decide)
    result = refine("shelf_label_06", jev)
    assert result.question and not result.plan


def test_mapped_label_contract_compiles_alternatives_once(jev):
    intent = MissionIntent("search_object", object_query="rack_identity",
        constraints={"recognition_contract": {"kind": "ocr", "tokens": ["06", "6"], "aggregation": "any"}})
    result = PerceptionAgent().refine(intent, SimpleNamespace(), jev, "jev_primary")
    assert len(result.plan) == 1
    assert [c["query"] for c in result.plan[0].payload["checks"]] == ["text_06", "text_6"]
    assert result.plan[0].payload["aggregation"] == "any"


def test_region_confirmation_uses_map_contract_and_one_capture(jev):
    from pathlib import Path
    from robot_agent.agents.navigation_agent import NavigationAgent
    from robot_skills.semantic_map import SemanticMap
    semantic = SemanticMap(Path(__file__).resolve().parents[3] /
        "maps/sim_factory_mid360_like_gpu_trueimu_011/semantic_map.yaml")
    client = SimpleNamespace(semantic=SimpleNamespace(semantic_map=semantic))
    nav = NavigationAgent().refine(MissionIntent("navigate_to", target="shelf_05_06_region"),
                                   client, jev, "jev_primary")
    assert nav.plan and nav.plan[-1].name == "search_object"
    payload = nav.plan[-1].payload
    result = PerceptionAgent().refine(MissionIntent(**payload), client, jev, "jev_primary")
    assert not result.question and len(result.plan) == 1
    checks = result.plan[0].payload
    assert [c["query"] for c in checks["checks"]] == ["text_05", "text_06"]
    assert checks["aggregation"] == "all" and not checks["unresolved_requirements"]


def test_legacy_multi_label_fallback_has_explicit_grouping(jev):
    query = PerceptionAgent._arrival_confirmation_query("shelf_04_05_region")
    assert query == "shelf_label_04 and shelf_label_05"
    result = refine(query, jev)
    assert not result.question and len(result.plan) == 1
    assert [c["query"] for c in result.plan[0].payload["checks"]] == ["shelf_label_04", "shelf_label_05"]
    assert result.plan[0].payload["aggregation"] == "all"


def test_generic_red_object_is_color_evidence_not_an_invented_class(jev):
    result = refine("red object", jev)
    assert not result.question
    assert [c["query"] for c in result.plan[0].payload["checks"]] == ["color_red"]
    assert not result.plan[0].payload["unresolved_requirements"]


def test_recorded_supported_ocr_answers_compile_without_redundant_global_vote(jev, monkeypatch):
    def decide(stage, state, questions):
        if stage == "perception_command_parse":
            choices = {"coverage": "complete", "relation": "any", "unit_count": "1",
                       "kind_0": "ocr", "object_0": "none", "color_0": "none"}
        else:
            assert "decision" not in questions
            choices = {"check_0_compatible": "supported", "check_1_compatible": "supported"}
        return {"answers": {k: {"choice": v} for k, v in choices.items()}}
    monkeypatch.setattr(jev, "_decide", decide)
    intent = MissionIntent("search_object", object_query="shelf_label_06", constraints={
        "recognition_contract": {"kind": "ocr", "tokens": ["06", "6"], "aggregation": "any"}})
    result = PerceptionAgent().refine(intent, SimpleNamespace(), jev, "jev_primary")
    assert len(result.plan) == 1
    assert result.traces[-1].adopted_decision == "execute_supported_checks"


def test_missing_check_review_answer_blocks_that_check(jev, monkeypatch):
    monkeypatch.setattr(jev, "_decide", lambda stage, *_: None if stage == "perception_command_parse" else {})
    result = refine("06 letter", jev)
    assert result.question and not result.plan


def test_ocr_extra_attributes_not_silently_discarded(jev):
    result = refine("red label 06", jev)
    assert result.question and not result.plan


@pytest.mark.parametrize("query,checks,relation", [
    ("red bottle", ["red bottle"], "all"),
    ("cup and bottle", ["cup", "bottle"], "all"),
    ("cup or bottle", ["cup", "bottle"], "any"),
    ("shelf_label_06 or 6", ["shelf_label_06", "text_6"], "any"),
])
def test_shared_capture_compound_and_alternatives(jev, query, checks, relation):
    result = refine(query, jev)
    assert len(result.plan) == 1
    assert [c["query"] for c in result.plan[0].payload["checks"]] == checks
    assert result.plan[0].payload["aggregation"] == relation
    assert not result.plan[0].payload["unresolved_requirements"]


def test_unsupported_shade_is_not_erased(jev):
    result = refine("light blue bottle", jev)
    assert result.plan[0].payload["unresolved_requirements"]


def test_missing_model_keeps_independent_color(jev):
    client = SimpleNamespace(capability_snapshot=lambda domain, args: {
        "available": False if args["kind"] == "object" else None})
    result = refine("red and bottle", jev, client=client)
    assert [c["kind"] for c in result.plan[0].payload["checks"]] == ["color"]
    assert result.plan[0].payload["unresolved_requirements"]


def test_unsupported_object_does_not_invent_substitute(jev):
    result = refine("box", jev)
    assert result.question and not result.plan
    assert not result.choices
    assert "box" in result.question


def test_object_approval_does_not_erase_unsupported_color_qualifier(jev):
    intent = MissionIntent("search_object", object_query="light blue box",
                           constraints={"perception_approved_objects": {"0": "suitcase"}})
    result = PerceptionAgent().refine(intent, SimpleNamespace(), jev, "jev_primary")
    assert result.plan
    assert any("light" in gap["reason"] for gap in result.plan[0].payload["unresolved_requirements"])


def test_image_description_not_reduced_to_capture(jev):
    intent = DeterministicParser().parse("check left side of robot if anything dangerous?")
    assert intent.task_type == "observe_scene"
    assert intent.constraints["analysis_kind"] == "image_describe"
    result = PerceptionAgent().refine(intent, SimpleNamespace(), jev, "jev_primary")
    assert result.outcome == "UNSUPPORTED" and not result.plan
    assert "API not configured" in result.question


def test_malformed_live_parse_does_not_use_local_defaults(jev, monkeypatch):
    monkeypatch.setattr(jev, "_decide", lambda *a: {})
    result = refine("red bottle", jev)
    assert result.question and not result.plan


def test_shadow_does_not_adopt_live_parse_or_start_actions(jev, monkeypatch):
    monkeypatch.setattr(jev, "_decide", lambda *a: {})
    result = refine("bottle", jev, "jev_shadow")
    assert result.plan[0].payload["checks"][0]["query"] == "bottle"


def test_view_details_are_resolved_by_perception_not_mission(jev, monkeypatch):
    stages = []
    def decide(stage, _state, _questions):
        stages.append(stage)
        values = {"profile": "directional", "view": "middle_left", "analysis": "capture_only"} if stage == "perception_view_parse" else {
            "decision": "capture_only" if stage == "perception_command_parse" else "execute"}
        return {"answers": {key: {"choice": value} for key, value in values.items()}}
    monkeypatch.setattr(jev, "_decide", decide)
    intent = MissionIntent("observe_scene", raw_text="look left", constraints={"unresolved_view": "left"})
    result = PerceptionAgent().refine(intent, SimpleNamespace(), jev, "jev_primary")
    assert not result.question
    assert stages[0] == "perception_view_parse"
    assert any(s.name == "capture_observation" for s in result.plan)
    assert intent.constraints == {"unresolved_view": "left"}


@pytest.mark.parametrize("view", list(PerceptionAgent._configured_views()))
def test_view_questions_and_execution_use_same_configured_pose(jev, monkeypatch, view):
    def decide(stage, state, questions):
        if stage == "perception_view_parse":
            assert view in questions["view"]["criteria"]
            choices = {"profile": "directional", "view": view, "analysis": "capture_only"}
        else:
            choices = {"decision": "capture_only" if stage == "perception_command_parse" else "execute"}
        return {"answers": {k: {"choice": v} for k, v in choices.items()}}
    monkeypatch.setattr(jev, "_decide", decide)
    result = PerceptionAgent().refine(MissionIntent("observe_scene", raw_text="look ground side",
        constraints={"unresolved_view": "ground side"}), SimpleNamespace(), jev, "jev_primary")
    assert not result.question
    pose = next(s.payload for s in result.plan if s.name == "set_camera_pose")
    configured = PerceptionAgent._configured_views()[view]
    assert (pose["pan_deg"], pose["tilt_deg"], pose["scan_band"]) == (configured["pan_deg"], configured["tilt_deg"], view)


def test_unknown_camera_band_never_defaults_to_middle():
    with pytest.raises(ValueError, match="Unconfigured"):
        PerceptionAgent._direction_camera_pose("front", "unknown_band")


@pytest.mark.parametrize("resolution", [None, "approve", "supported_only"])
def test_composite_typo_proposal_survives_partial_capability(jev, monkeypatch, resolution):
    def decide(stage, state, questions):
        choices = ({"coverage": "complete", "relation": "all", "unit_count": "2",
                    "kind_0": "composite", "object_0": "suitcase", "color_0": "red"}
                   if stage == "perception_command_parse" else {"check_0_compatible": "supported"})
        return {"answers": {k: {"choice": v} for k, v in choices.items()}}
    monkeypatch.setattr(jev, "_decide", decide)
    constraints = ({"perception_approved_objects": {"0": "suitcase"}} if resolution == "approve"
                   else {"perception_use_supported_only": True} if resolution == "supported_only" else {})
    result = PerceptionAgent().refine(MissionIntent("search_object", object_query="red suitecase", constraints=constraints),
                                    SimpleNamespace(), jev, "jev_primary")
    if resolution is None:
        assert result.question and not result.plan
        assert {c["action"] for c in result.choices} == {"perception_replace_0", "perception_supported_only"}
    else:
        assert not result.question
        payload = result.plan[0].payload
        assert payload["checks"][0]["query"] == ("red suitcase" if resolution == "approve" else "color_red")
        assert bool(payload["unresolved_requirements"]) == (resolution == "supported_only")


def test_unknown_view_classic_does_not_call_jev(jev, monkeypatch):
    def unexpected(*_):
        raise AssertionError("Classic must not call JEV")
    monkeypatch.setattr(jev, "domain_questions", unexpected)
    intent = MissionIntent("observe_scene", constraints={"unresolved_view": "left"})
    assert PerceptionAgent().refine(intent, SimpleNamespace(), jev, "classic").question


def test_exception_questions_come_from_catalog(jev, monkeypatch):
    requests = []
    monkeypatch.setattr(jev, "_decide", lambda *args: requests.append(args) or None)
    jev.supervise_step(user_text="find bottle", intent=MissionIntent("search_object", object_query="bottle"),
                       plan=[SkillCall("search_at_place")], step_index=0, health=None,
                       result={"owner": "perception", "skill_name": "search_at_place", "code": "FAILED_PERCEPTION",
                               "success": False, "metrics": {"failed_child": "detect_known_object", "inference_completed": False}})
    stage, state, questions = requests[0]
    assert stage == "perception_step_supervision"
    assert "skill_backend_completed" in questions
    assert state["skill_result"]["metrics"]["inference_completed"] is False
    assert SkillCatalog().compact_skill("search_at_place")["supervision"]["questions"]["fresh_image"]


def test_live_choice_requires_approval_for_new_object_word(jev, monkeypatch):
    def decide(stage, state, questions):
        choices = {"coverage": "complete", "relation": "all", "unit_count": "1",
                   "kind_0": "object", "object_0": "cup", "color_0": "none", "decision": "ask_human"}
        return {"answers": {key: {"choice": choices[key]} for key in questions}}
    monkeypatch.setattr(jev, "_decide", decide)
    result = refine("beaker", jev)
    assert result.question and not result.plan
    assert result.choices[0]["constraints"]["perception_approved_objects"] == {"0": "cup"}


def test_plan_review_can_reject_one_check_without_discarding_other(jev, monkeypatch):
    def decide(stage, state, questions):
        if stage == "perception_command_parse":
            return None
        assert "capability_reviews" in state["evidence"]
        return {"answers": {"decision": {"choice": "execute"},
                            "check_0_compatible": {"choice": "supported"},
                            "check_1_compatible": {"choice": "unsupported"}}}
    monkeypatch.setattr(jev, "_decide", decide)
    result = refine("red and bottle", jev)
    assert [c["query"] for c in result.plan[0].payload["checks"]] == ["color_red"]
    assert any(g["reason"] == "Plan review: unsupported" for g in result.plan[0].payload["unresolved_requirements"])
