import pytest

from robot_skills.perception_capabilities import load_targets, normalize_targets
from robot_skills.executors.perception import PerceptionSkills


def test_box_is_semantic_gap_not_detector_alias():
    config = load_targets()
    assert len(config["detector_classes"]) == 80
    assert "box" not in config["detector_classes"]
    assert config["objects"]["box"]["classes"] == []
    plan = PerceptionSkills(None)._build_detection_plan("red box")
    assert plan["yolo_classes"] == [] and plan["fallback_classes"] == []
    assert plan["colors"] == ["red"] and plan["unsupported"]


def test_alias_cannot_expand_model_capabilities():
    config = load_targets()
    config["object_aliases"]["fake"] = {"classes": ["box"]}
    with pytest.raises(ValueError, match="unsupported detector classes"):
        normalize_targets(config)


def test_color_and_ocr_claim_only_configured_capabilities():
    config = load_targets()
    assert "light_blue" not in config["colors"]
    assert config["text_contract"]["scripts"] == ["latin", "digits"]
    assert config["text_contract"]["arbitrary_scene_description"] is False
    config["colors"]["red"]["hsv_ranges"][0]["upper"][0] = 300
    with pytest.raises(ValueError, match="HSV"):
        normalize_targets(config)


def test_typed_class_and_free_text_have_identical_capability():
    skills = PerceptionSkills(None)
    assert skills._build_detection_plan("object_chair")["yolo_classes"] == skills._build_detection_plan("chair")["yolo_classes"]


def test_specific_car_query_does_not_silently_widen_to_all_vehicles():
    skills = PerceptionSkills(None)
    assert skills._build_detection_plan("car")["yolo_classes"] == ["car"]
    assert set(skills._build_detection_plan("vehicle")["yolo_classes"]) == {"car", "truck", "bus"}
