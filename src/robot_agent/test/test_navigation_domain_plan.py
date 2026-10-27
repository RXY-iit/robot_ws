from copy import deepcopy
from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from robot_agent.agents.navigation_agent import NavigationAgent
from robot_agent.agents.navigation_plan import applicable_rules, path_evidence
from robot_agent.jev_agent import JevCallTrace, JevStrengthenAgent
from robot_agent.schemas import MissionIntent, SkillCall
from robot_skills.semantic_map import SemanticMap
from robot_skills.skill_catalog import SkillCatalog


@pytest.fixture
def client():
    semantic = SemanticMap.__new__(SemanticMap)
    semantic.data, semantic.frame_id, semantic.places = {}, "map", []
    semantic.objects = [{"id": "door_front", "names": ["front door", "door"], "pose": {"x": 1, "y": 2}},
                        {"id": "door_back", "names": ["back door", "door"], "pose": {"x": 2, "y": 2}}]
    return SimpleNamespace(semantic=SimpleNamespace(semantic_map=semantic))


class Jev:
    def __init__(self, override=None):
        self.calls, self.override = [], override or {}

    def domain_questions(self, stage, intent, questions, defaults, evidence):
        self.calls.append((stage, deepcopy(evidence), questions))
        return {**defaults, **self.override}, JevCallTrace(call_id=str(len(self.calls)), stage=stage)


def test_missing_map_fails_before_any_plan():
    result = NavigationAgent().refine(MissionIntent("navigate_to", target="door"), object(), Jev(), "jev_primary")
    assert not result.plan and result.outcome == "CAPABILITY_UNAVAILABLE"


def test_unknown_destination_is_not_an_empty_selection(client):
    result = NavigationAgent().refine(MissionIntent("navigate_to", target="shelf_9999"), client, Jev(), "classic")
    assert result.outcome == "UNKNOWN_TARGET" and not result.plan
    assert "not registered" in result.question


def test_navigation_passes_target_yaml_confirmation_contract(client):
    spec = {"enabled": True, "query": "rack_identity", "recognition": {
        "kind": "ocr", "tokens": ["06", "6"], "aggregation": "any"}}
    client.semantic.semantic_map.objects[0]["visual_arrival_confirm"] = spec
    result = NavigationAgent().refine(MissionIntent("navigate_to", target="door_front"), client, Jev(), "classic")
    sub = result.plan[-1]
    assert sub.kind == "task" and sub.owner == "perception"
    assert sub.payload["object_query"] == "rack_identity"
    assert sub.payload["constraints"]["recognition_contract"] == spec["recognition"]


def test_ambiguous_target_has_scoped_answers(client):
    nav, jev = NavigationAgent(), Jev()
    intent = MissionIntent("navigate_to", target="door")
    answer = nav.refine(intent, client, jev, "jev_primary")
    assert not answer.plan and len(answer.choices) == 2
    assert not jev.calls
    assert all("command" not in c and c["action"] for c in answer.choices)
    intent.constraints.update(answer.choices[0]["constraints"])
    result = nav.refine(intent, client, jev, "jev_primary")
    assert result.plan[0].payload["target_name"] == "door_front"
    assert [t.stage for t in result.traces] == ["navigation_command_parse", "navigation_plan_review"]
    assert all("health" not in evidence for _, evidence, _ in jev.calls)
    assert "goal_binding" not in jev.calls[0][2]
    assert jev.calls[0][1]["target_binding"] == {
        "source": "human_selection", "original_request": "", "resolved_target_id": "door_front",
        "destination_ambiguity_resolved": True, "registered_match_count": 1}


def test_unique_map_match_is_not_reported_as_unresolved_ambiguity(client):
    jev = Jev()
    result = NavigationAgent().refine(
        MissionIntent("navigate_to", target="door_front", raw_text="go to front door"),
        client, jev, "jev_primary")
    assert result.plan
    binding = jev.calls[0][1]["target_binding"]
    assert binding["source"] == "semantic_map"
    assert binding["destination_ambiguity_resolved"] is True
    assert binding["registered_match_count"] == 1


def test_selected_target_does_not_repeat_original_identity_question(client, monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    jev = JevStrengthenAgent()
    requests = []
    def decide(stage, state, questions):
        requests.append((stage, state, questions))
        values = {"goal_binding": "ask_human", "goal_kind": "point",
                  "visual_confirmation": "not_required", "decision": "execute"}
        return {"answers": {k: {"choice": values[k]} for k in questions}}
    monkeypatch.setattr(jev, "_decide", decide)
    intent = MissionIntent("navigate_to", target="door", raw_text="go to door",
        constraints={"navigation_selected_target": "door_front", "requested_goal": "door"})
    result = NavigationAgent().refine(intent, client, jev, "jev_primary")
    assert result.plan and not result.question
    assert result.traces[0].adopted_decision == "accept_navigation_goal"
    assert "goal_binding" not in requests[0][2]
    assert requests[0][1]["evidence"]["target_binding"]["original_request"] == "door"
    client.semantic.semantic_map.data["navigation_rules"] = [
        {"id": "blocked", "revision": 1, "target_ids": ["door_front"], "effect": "deny_destination"}]
    assert NavigationAgent().refine(intent, client, jev, "jev_primary").outcome == "RULE_BLOCKED"


def test_human_selection_must_still_be_an_exact_registered_id(client):
    result = NavigationAgent().refine(MissionIntent("navigate_to", target="door",
        constraints={"navigation_selected_target": "front door"}), client, Jev(), "jev_primary")
    assert not result.plan and result.outcome == "UNKNOWN_TARGET"


@pytest.mark.parametrize("effect", ["deny_destination", "forbid_polygon", "arbitrary_action"])
def test_hard_or_unsupported_rules_cannot_be_overridden(client, effect):
    client.semantic.semantic_map.data["navigation_rules"] = [
        {"id": "maintenance", "revision": 1, "target_ids": ["door_front"], "effect": effect}]
    jev = Jev({"decision": "execute"})
    result = NavigationAgent().refine(MissionIntent("navigate_to", target="door_front"), client, jev, "jev_primary")
    assert result.outcome == "RULE_BLOCKED" and not result.plan and not jev.calls


def test_rule_validity_and_scope(client):
    semantic = client.semantic.semantic_map
    semantic.data["navigation_rules"] = [
        {"id": "expired", "revision": 1, "effect": "deny_destination", "valid_until": "2026-10-11T00:00:00Z"},
        {"id": "other", "revision": 1, "target_ids": ["door_back"], "effect": "deny_destination"}]
    now = datetime(2026, 10, 12, tzinfo=timezone.utc)
    assert applicable_rules(semantic, "door_front", now) == ([], [])
    semantic.data["navigation_rules"][0]["valid_until"] = "2026-10-13"
    assert applicable_rules(semantic, "door_front", now)[1]


def test_explicit_unavailable_approach_is_not_silently_near(client):
    result = NavigationAgent().refine(MissionIntent("navigate_to", target="door_front", relation="back"), client, Jev(), "classic")
    assert result.outcome == "UNSUPPORTED_APPROACH" and not result.plan


@pytest.mark.parametrize("mode,blocked", [("jev_primary", True), ("jev_shadow", False)])
def test_primary_adopts_and_shadow_observes(client, mode, blocked):
    result = NavigationAgent().refine(MissionIntent("navigate_to", target="door_front"),
                                    client, Jev({"goal_binding": "ask_human"}), mode)
    assert bool(result.question) == blocked


def test_computed_path_review_records_actual_path_and_revalidates_rules(client):
    nav, jev = NavigationAgent(), Jev()
    intent = MissionIntent("navigate_to", target="door_front")
    refinement = nav.refine(intent, client, jev, "jev_primary")
    contract = next(s.catalog["navigation_contract"] for s in refinement.plan if s.name == "goto_pose")
    evidence = {"path": [{"x": i / 10, "y": 0} for i in range(100)],
                "target_pose": {"x": 9.9, "y": 0, "frame_id": "map"},
                "annotation": {"passage_id": "narrow_1", "expected_width_m": .8}}
    decision, traces = nav.review_path(intent, contract, evidence, client, jev, "jev_primary")
    assert decision["decision"] == "execute" and decision["route_strategy"] == "annotated_segments"
    assert traces[0].stage == "navigation_plan_review"
    path = jev.calls[-1][1]["path"]
    assert path["point_count"] == 100 and len(path["sampled_points"]) <= 32
    assert path["annotation"]["passage_id"] == "narrow_1"
    client.semantic.semantic_map.data["navigation_rules"] = [{"id": "stop", "revision": 1, "effect": "deny_destination"}]
    blocked, traces = nav.review_path(intent, contract, evidence, client, jev, "jev_primary")
    assert blocked["decision"] == "ask_human" and not traces


def test_invalid_path_rejected():
    with pytest.raises(ValueError):
        path_evidence([{"x": float("nan"), "y": 1}], {"frame_id": "map"}, {})


def test_path_conflict_blocks_primary_but_shadow_only_observes(client):
    nav = NavigationAgent()
    intent = MissionIntent('navigate_to', target='door_front')
    contract = {'target_id': 'door_front', 'rules': []}
    evidence = {'path': [{'x': 0, 'y': 0}, {'x': 1, 'y': 2}],
                'target_pose': {'x': 1, 'y': 2, 'frame_id': 'map'}}
    for mode, expected in [('jev_primary', 'ask_human'), ('jev_shadow', 'execute')]:
        jev = Jev({'decision': 'ask_human', 'blocking_issue': 'rule_conflict', 'route_strategy': 'ask_human'})
        decision, _ = nav.review_path(intent, contract, evidence, client, jev, mode)
        assert decision['decision'] == expected
        if mode == 'jev_primary':
            assert 'rule conflict' in decision['reason']
            assert decision['route_strategy'] == 'ask_human'
            assert decision['implemented_route_strategy'] == 'nav2'
        assert 'pre_dispatch_recheck' in jev.calls[-1][1]['execution_contract']


def test_failed_navigation_never_offers_dependent_search(monkeypatch):
    monkeypatch.delenv('JEV_API_KEY', raising=False)
    jev = JevStrengthenAgent()
    plan = [SkillCall('goto_pose', task_id='nav'),
            SkillCall('search_object', kind='task', task_id='verify', catalog={'parent_task_id': 'nav'},
                      payload={'constraints': {'dependency_policy': 'required_condition'}}),
            SkillCall('navigate_to', kind='task_end', task_id='nav')]
    assert jev._next_meaningful_call(plan, 0) is None
    assert all('Continue' not in c['label'] for c in jev._human_choices_for_next(None, 'go to shelf_06', 'unknown'))
    assert 'search_object' not in jev._message_for_action('ask_human_continue_retry_stop', {}, None)
    assert not any(c['label'].startswith('Continue') for c in jev._human_choices_for_next(
        SkillCall('search_object', kind='task'), 'go to shelf_06', 'unknown'))
    independent = SkillCall('set_pose', payload={'name': 'home'}, task_id='cleanup')
    plan.append(independent)
    assert jev._next_meaningful_call(plan, 0) is independent


def test_expanded_descendant_is_not_an_independent_next_goal():
    plan = [SkillCall('goto_pose', task_id='nav'), SkillCall('search_at_place', task_id='verify'),
            SkillCall('search_object', kind='task_end', task_id='verify', catalog={'parent_task_id': 'nav'}),
            SkillCall('navigate_to', kind='task_end', task_id='nav')]
    assert JevStrengthenAgent._next_meaningful_call(plan, 0) is None


def test_every_registered_skill_has_unified_supervision(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    catalog = SkillCatalog()
    assert catalog.validate_registry() == []
    jev = JevStrengthenAgent()
    requests = []
    jev._decide = lambda *args: requests.append(args) or None
    for name in catalog.list_names():
        spec = catalog.get(name)["supervision"]
        assert spec["questions"] and spec["evidence_fields"], name
        jev.supervise_step(user_text="test", intent=MissionIntent("navigate_to"), plan=[SkillCall(name)],
                          step_index=0, result={"skill_name": name, "success": False, "code": "FAILED_TIMEOUT",
                                                "owner": "navigation"}, health=None)
        stage, _, questions = requests[-1]
        assert stage == "navigation_step_supervision"
        for key, prompt in spec["questions"].items():
            assert prompt in questions[f"skill_{key}"]["instructions"]
        assert questions["failure_category"] == catalog.data["common_supervision_questions"]["failure_category"]
    assert "ask_human_enable_auto" in catalog.data["common_supervision_questions"]["supervision_action"]["criteria"]
