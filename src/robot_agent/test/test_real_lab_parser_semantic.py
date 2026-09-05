from __future__ import annotations

from pathlib import Path

from robot_agent.parsers.deterministic_parser import DeterministicParser
from robot_skills.executors.navigation import NavigationSkills
from robot_skills.semantic_map import SemanticMap


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SIM_REAL_LAB_FIXTURE = Path(__file__).parent / "fixtures" / "sim_real_lab_001_semantic_map.yaml"
REAL_MAP_ID = "l402_2d_map_clean_0509"
REAL_SEMANTIC_MAP = WORKSPACE_ROOT / "maps" / REAL_MAP_ID / "semantic_map.yaml"
REAL_SEMANTIC_SEGMENTS = WORKSPACE_ROOT / "maps" / REAL_MAP_ID / "semantic_segments.yaml"
REAL_NAVIGATION_ANNOTATIONS = WORKSPACE_ROOT / "maps" / REAL_MAP_ID / "navigation_annotations.yaml"


def test_real_lab_navigation_targets_reach_semantic_resolver():
    parser = DeterministicParser()
    semantic_map = SemanticMap(str(SIM_REAL_LAB_FIXTURE))

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
    semantic_map = SemanticMap(str(SIM_REAL_LAB_FIXTURE))

    intent = parser.parse("go to table")

    assert intent.task_type == "navigate_to_object"
    assert intent.target == "table"
    assert len(semantic_map.find_matches(intent.target)) > 1


def test_semantic_map_can_be_selected_by_environment(monkeypatch):
    monkeypatch.setenv("ROBOT_SEMANTIC_MAP", str(REAL_SEMANTIC_MAP))
    monkeypatch.delenv("ROBOT_MAP_ID", raising=False)

    semantic_map = SemanticMap()

    assert semantic_map.path == str(REAL_SEMANTIC_MAP)
    assert semantic_map.data.get("map_id") == REAL_MAP_ID


def test_real_lab_semantic_map_can_be_selected_by_map_id(monkeypatch):
    monkeypatch.delenv("ROBOT_SEMANTIC_MAP", raising=False)
    monkeypatch.setenv("ROBOT_MAP_ID", REAL_MAP_ID)

    semantic_map = SemanticMap()

    assert semantic_map.path.endswith(f"maps/{REAL_MAP_ID}/semantic_map.yaml")
    assert semantic_map.data.get("map_id") == REAL_MAP_ID


def test_real_lab_alias_selects_bundle_semantic_map(monkeypatch):
    monkeypatch.delenv("ROBOT_SEMANTIC_MAP", raising=False)
    monkeypatch.setenv("ROBOT_MAP_ID", "real_lab")

    semantic_map = SemanticMap()

    assert semantic_map.path.endswith(f"maps/{REAL_MAP_ID}/semantic_map.yaml")
    assert semantic_map.data.get("map_id") == REAL_MAP_ID


def test_real_lab_region_targets_provide_multiple_approach_candidates():
    semantic_map = SemanticMap(str(SIM_REAL_LAB_FIXTURE))

    candidates = semantic_map.approach_candidates("sofa")

    assert len(candidates) >= 4
    assert candidates[0]["source"] == "generated_from_region"
    assert any(candidate.get("preferred") for candidate in candidates)


def test_navigation_ready_false_blocks_region_generated_navigation(tmp_path):
    semantic_path = tmp_path / "semantic_map.yaml"
    semantic_path.write_text(
        """
map_id: provisional
frame_id: map
places: []
objects:
  - id: provisional_table
    names: [provisional table]
    category: table
    navigation_ready: false
    map_region:
      type: polygon
      polygon:
        - {x: 0.0, y: 0.0}
        - {x: 1.0, y: 0.0}
        - {x: 1.0, y: 1.0}
        - {x: 0.0, y: 1.0}
      front_side: south
    approach_policy:
      generator: region_side_offset
      preferred_side: south
      distance_m: 0.75
""",
        encoding="utf-8",
    )
    semantic_map = SemanticMap(str(semantic_path))

    assert semantic_map.resolve("provisional table") is None
    assert semantic_map.approach_candidates("provisional table") == []


def test_real_map_manual_and_provisional_semantics_are_consistent():
    import yaml

    semantic_data = yaml.safe_load(REAL_SEMANTIC_MAP.read_text(encoding="utf-8"))
    segment_data = yaml.safe_load(REAL_SEMANTIC_SEGMENTS.read_text(encoding="utf-8"))
    annotation_data = yaml.safe_load(REAL_NAVIGATION_ANNOTATIONS.read_text(encoding="utf-8"))
    objects = semantic_data["objects"]
    segments = segment_data["segments"]
    object_ids = [item["id"] for item in objects]
    segment_ids = [item["id"] for item in segments]

    assert len(objects) == 22
    assert len(segments) == 22
    assert len(set(object_ids)) == len(object_ids)
    assert len(set(segment_ids)) == len(segment_ids)
    assert all(item.get("segment_id") in set(segment_ids) for item in objects)
    assert {item["id"] for item in objects if item.get("navigation_ready")} == {
        "front-door",
        "rxy-table",
        "sofa",
        "tool-box",
        "back-door",
        "meeting_table",
        "wall_cabinet_left_upper",
        "wall_cabinet_left_lower",
        "desk_ire",
        "desk_tkd",
        "desk_tcb",
        "desk_min",
        "desk_hyk",
        "desk_mkr",
    }
    assert len(annotation_data["narrow_passages"]) == 3
    assert all(passage.get("enabled") for passage in annotation_data["narrow_passages"])
    assert all(passage.get("calibrated") for passage in annotation_data["narrow_passages"])
    assert all(
        zone.get("enabled")
        for passage in annotation_data["narrow_passages"]
        for zone in passage["alignment_zones"]
    )


def test_real_map_provisional_target_is_listable_but_not_navigable():
    semantic_map = SemanticMap(str(REAL_SEMANTIC_MAP))

    assert any(item["id"] == "meeting_table" for item in semantic_map.list_known_targets())
    assert semantic_map.resolve("trash bin") is None
    assert semantic_map.resolve("desk tks") is None
    assert semantic_map.resolve("meeting table") is not None
    assert semantic_map.resolve("front door") is not None


def test_real_map_passage_poses_follow_alignment_polygon_centers():
    import math
    import yaml

    annotation_data = yaml.safe_load(REAL_NAVIGATION_ANNOTATIONS.read_text(encoding="utf-8"))
    for passage in annotation_data["narrow_passages"]:
        zones = {zone["side"]: zone for zone in passage["alignment_zones"]}
        centers = {}
        for side in ("entry", "exit"):
            polygon = zones[side]["polygon"]
            centers[side] = (
                sum(point["x"] for point in polygon) / len(polygon),
                sum(point["y"] for point in polygon) / len(polygon),
            )
            assert math.isclose(zones[side]["pose"]["x"], centers[side][0], abs_tol=1e-4)
            assert math.isclose(zones[side]["pose"]["y"], centers[side][1], abs_tol=1e-4)
            assert math.isclose(passage[side]["x"], centers[side][0], abs_tol=1e-4)
            assert math.isclose(passage[side]["y"], centers[side][1], abs_tol=1e-4)
        assert passage["centerline"][0] == {
            "x": round(centers["entry"][0], 4),
            "y": round(centers["entry"][1], 4),
        }
        assert passage["centerline"][-1] == {
            "x": round(centers["exit"][0], 4),
            "y": round(centers["exit"][1], 4),
        }


def test_real_camera_logical_pose_maps_to_real_hardware_neutral_offsets():
    from robot_skills.executors.camera import CameraSkills

    camera = object.__new__(CameraSkills)
    camera._real_pan_neutral_deg = 267.0
    camera._real_tilt_neutral_deg = 102.0
    camera._logical_pan_neutral_deg = 0.0
    camera._logical_tilt_neutral_deg = 67.0

    assert camera._hardware_targets("real", 0.0, 67.0) == (267.0, 102.0)
    assert camera._hardware_targets("real", -35.0, 25.0) == (232.0, 60.0)
    assert camera._hardware_targets("sim", -35.0, 25.0) == (-35.0, 25.0)


def test_real_camera_uses_named_calibrated_pose_when_enabled():
    from robot_skills.executors.camera import CameraSkills

    camera = object.__new__(CameraSkills)
    camera._real_pan_neutral_deg = 267.0
    camera._real_tilt_neutral_deg = 102.0
    camera._logical_pan_neutral_deg = 0.0
    camera._logical_tilt_neutral_deg = 67.0
    camera._real_pose_config = {
        "calibration": {"use_calibrated_poses": True},
        "poses": {"upper_left": {"hardware_pan_deg": 235.0, "hardware_tilt_deg": 22.0}},
    }

    assert camera._hardware_targets_for_pose("real", -35.0, -20.0, "upper_left") == (
        235.0,
        22.0,
        "real_camera_poses",
    )
    assert camera._hardware_targets_for_pose("real", 0.0, 25.0, "middle_center") == (
        267.0,
        60.0,
        "neutral_offset_fallback",
    )


def test_look_around_builds_camera_scan_without_detector_dependency():
    from robot_agent.agents.perception_agent import PerceptionAgent

    plan = PerceptionAgent().build_search_plan("around")

    assert [call.name for call in plan] == [
        "set_camera_pose",
        "capture_observation",
        "set_camera_pose",
        "capture_observation",
        "set_camera_pose",
        "capture_observation",
        "set_camera_pose",
    ]
    assert plan[-1].payload == {
        "pan_deg": 0.0,
        "tilt_deg": 67.0,
        "scan_band": "return_home",
    }


def test_directional_camera_commands_use_nine_pose_calibration_ids():
    from robot_agent.agents.perception_agent import PerceptionAgent

    assert PerceptionAgent._direction_camera_pose("front", "up")[2] == "upper_center"
    assert PerceptionAgent._direction_camera_pose("left", "middle")[2] == "middle_left"
    assert PerceptionAgent._direction_camera_pose("front_right", "down")[2] == "ground_right"


def test_real_camera_pose_config_contains_nine_measured_views():
    import yaml

    config_path = WORKSPACE_ROOT / "src" / "robot_skills" / "config" / "real_camera_poses.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    assert config["calibration"]["use_calibrated_poses"] is True
    assert config["calibration"]["status"] == "calibrated_9_of_9"
    assert config["poses"]["middle_center"]["hardware_pan_deg"] == 267.0
    assert config["poses"]["middle_center"]["hardware_tilt_deg"] == 65.0
    assert config["poses"]["upper_left"]["hardware_pan_deg"] == 334.0
    assert config["poses"]["ground_right"]["hardware_tilt_deg"] == 107.0
    assert config["poses"]["upper_center"]["hardware_pan_deg"] == 267.0
    assert config["poses"]["upper_center"]["hardware_tilt_deg"] == 33.0
    assert config["poses"]["return_home"] == {
        "hardware_pan_deg": 267.0,
        "hardware_tilt_deg": 102.0,
        "note": "verified neutral",
    }


def test_absolute_navigation_restores_camera_home_but_relative_motion_does_not():
    from robot_agent.agents.mission_agent import MissionAgent
    from robot_agent.agents.navigation_agent import NavigationAgent
    from robot_agent.schemas import MissionIntent

    absolute_plan = NavigationAgent().build_plan(
        MissionIntent(task_type="navigate_to_place", target="meeting_table")
    )
    assert [call.name for call in absolute_plan] == [
        "resolve_semantic_target",
        "await_robot_stationary",
        "set_camera_pose",
        "await_robot_auto_mode",
        "check_localization_health",
        "goto_pose",
    ]
    camera_call = absolute_plan[2]
    assert camera_call.payload["scan_band"] == "return_home"
    assert camera_call.payload["require_feedback"] is True
    assert absolute_plan[4].payload["settle_timeout_sec"] == 15.0

    assert "set_camera_pose" not in {
        call.name for call in MissionAgent._relative_motion_preflight()
    }


def test_manual_initialpose_requires_fresh_repeated_good_gicp_before_trust():
    from robot_skills.executors.system import SystemSkills

    system = SystemSkills(object(), mode="real")
    system.update_gicp_score(0.1)
    system.update_odom_pose(1.0, 2.0, 0.3)
    system.update_initialpose()

    assert system._gicp_score is None
    assert system._latest_odom_pose is None
    assert not system.localization_is_healthy()

    # Simulate completion of the explicit initial-pose settling period.
    system._last_initialpose_time -= system._initialpose_settle_sec + 0.1
    for _ in range(system._gicp_min_good_samples):
        system.update_gicp_score(0.1)
        system.update_odom_pose(1.0, 2.0, 0.3)

    assert system.localization_is_healthy()
    assert system.last_trusted_pose() == (1.0, 2.0, 0.3)

    system._last_gicp_score_time -= system._localization_data_stale_sec + 0.1
    assert not system.localization_is_healthy()


def test_dynamic_navigation_timeout_scales_down_for_near_targets():
    near_timeout = NavigationSkills._estimate_navigation_timeout(1.0, 90.0, 120.0)
    far_timeout = NavigationSkills._estimate_navigation_timeout(6.0, 90.0, 120.0)

    assert 18.0 <= near_timeout < 30.0
    assert far_timeout > near_timeout
    assert far_timeout < 90.0
