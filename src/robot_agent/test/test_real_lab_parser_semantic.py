from __future__ import annotations

from pathlib import Path

from robot_agent.parsers.deterministic_parser import DeterministicParser
from robot_skills.executors.navigation import NavigationSkills
from robot_skills.semantic_map import SemanticMap


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
REAL_LAB_BUNDLE = WORKSPACE_ROOT / "maps" / "sim_real_lab_001"
REAL_LAB_SEMANTIC_MAP = REAL_LAB_BUNDLE / "semantic_map.yaml"


def test_real_lab_navigation_targets_reach_semantic_resolver():
    parser = DeterministicParser()
    semantic_map = SemanticMap(str(REAL_LAB_SEMANTIC_MAP))

    cases = [
        ("go to the sofa", "navigate_to_object", "sofa"),
        ("go to meeting table", "navigate_to_object", "meeting_table"),
        ("go to rxy table", "navigate_to_object", "desk_rxy"),
        ("desk_rxy に行って", "navigate_to_object", "desk_rxy"),
        ("inspect tool box", "inspect_place", "tool_box"),
    ]

    for command, task_type, object_id in cases:
        intent = parser.parse(command)
        assert intent.task_type == task_type
        assert intent.target
        resolved = semantic_map.resolve(intent.target)
        assert resolved is not None
        assert resolved.object_id == object_id


def test_real_lab_generic_table_navigation_is_deferred_to_semantic_map():
    parser = DeterministicParser()
    semantic_map = SemanticMap(str(REAL_LAB_SEMANTIC_MAP))

    intent = parser.parse("go to table")

    assert intent.task_type == "navigate_to_object"
    assert intent.target == "table"
    assert len(semantic_map.find_matches(intent.target)) > 1


def test_semantic_map_can_be_selected_by_environment(monkeypatch):
    monkeypatch.setenv("ROBOT_SEMANTIC_MAP", str(REAL_LAB_SEMANTIC_MAP))
    monkeypatch.delenv("ROBOT_MAP_ID", raising=False)

    semantic_map = SemanticMap()

    assert semantic_map.path == str(REAL_LAB_SEMANTIC_MAP)
    assert semantic_map.data.get("map_id") == "sim_real_lab_001"


def test_real_lab_semantic_map_can_be_selected_by_map_id(monkeypatch):
    monkeypatch.delenv("ROBOT_SEMANTIC_MAP", raising=False)
    monkeypatch.setenv("ROBOT_MAP_ID", "sim_real_lab_001")

    semantic_map = SemanticMap()

    assert semantic_map.path.endswith("maps/sim_real_lab_001/semantic_map.yaml")
    assert semantic_map.data.get("map_id") == "sim_real_lab_001"


def test_real_lab_alias_selects_bundle_semantic_map(monkeypatch):
    monkeypatch.delenv("ROBOT_SEMANTIC_MAP", raising=False)
    monkeypatch.setenv("ROBOT_MAP_ID", "real_lab")

    semantic_map = SemanticMap()

    assert semantic_map.path.endswith("maps/sim_real_lab_001/semantic_map.yaml")
    assert semantic_map.data.get("map_id") == "sim_real_lab_001"


def test_real_lab_region_targets_provide_multiple_approach_candidates():
    semantic_map = SemanticMap(str(REAL_LAB_SEMANTIC_MAP))

    candidates = semantic_map.approach_candidates("sofa")

    assert len(candidates) >= 4
    assert candidates[0]["source"] == "generated_from_region"
    assert any(candidate.get("preferred") for candidate in candidates)


def test_dynamic_navigation_timeout_scales_down_for_near_targets():
    near_timeout = NavigationSkills._estimate_navigation_timeout(1.0, 90.0, 120.0)
    far_timeout = NavigationSkills._estimate_navigation_timeout(6.0, 90.0, 120.0)

    assert 18.0 <= near_timeout < 30.0
    assert far_timeout > near_timeout
    assert far_timeout < 90.0
