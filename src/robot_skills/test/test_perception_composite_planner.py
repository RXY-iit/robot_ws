from __future__ import annotations

import pytest

from robot_skills.executors.perception import PerceptionSkills


def _planner() -> PerceptionSkills:
    return PerceptionSkills(node=None)


def primitives():
    common = {"confidence": .9, "bbox": [0, 0, 100, 100], "image_path": "observation_a.jpg", "matched_query": True}
    return [{**common, "primitive": "color", "label": "color_region_red", "color": "red"},
            {**common, "primitive": "object", "label": "suitcase"}]


@pytest.mark.parametrize("index,changes", [
    (1, {"label": "bus"}), (1, {"label": "truck", "matched_query": False}),
    (1, {"image_path": "observation_b.jpg"}), (0, {"color": "blue"}),
    (1, {"matched_query": False}), (1, {"approximate_match": True}),
    (1, {"fallback_requires_human": True}), (1, {"image_path": None}),
    (1, {"confidence": float("nan")}), (0, {"bbox": [0, 0, float("inf"), 100]}),
    (1, {"bbox": [100, 0, 0, 100]}), (1, {"bbox": [200, 200, 300, 300]}),
])
def test_composite_rejects_unrelated_or_invalid_evidence(index, changes):
    items = primitives()
    items[index].update(changes)
    assert _planner()._match_composite_detections(items, "red suitcase", .5) == []


def test_composite_requires_same_image_and_retains_both_evidence_boxes():
    items = primitives()
    matches = _planner()._match_composite_detections(items, "red suitcase", .5)
    assert len(matches) == 1
    assert matches[0]["matched_query"]
    assert matches[0]["object_image_path"] == matches[0]["color_image_path"] == items[0]["image_path"]
    assert matches[0]["object_bbox"] == items[1]["bbox"]
    assert matches[0]["color_bbox"] == items[0]["bbox"]


def test_car_does_not_accept_bus_or_truck_as_exact_match():
    planner = _planner()
    assert planner._build_detection_plan("car")["yolo_classes"] == ["car"]
    assert set(planner._build_detection_plan("vehicle")["yolo_classes"]) == {"car", "truck", "bus"}


def test_open_vocab_composite_matches_real_model_class_without_preconfigured_object():
    planner = _planner()

    # "chair" is not curated in perception_targets.yaml's objects: registry, but it
    # is one of the yolo11s.pt checkpoint's real COCO classes, so it should be
    # attempted without needing to be pre-registered.
    plan = planner._build_detection_plan("green_chair")  # noqa: SLF001 - planner regression

    assert plan["type"] == "composite"
    assert plan["colors"] == ["green"]
    assert plan["objects"] == ["chair"]
    assert plan["open_objects"] == ["chair"]
    assert plan["yolo_classes"] == ["chair"]
    assert plan["text_kind"] is None
    assert plan["unsupported"] == []


def test_open_vocab_composite_strips_action_words():
    planner = _planner()

    plan = planner._build_detection_plan("find green chair")  # noqa: SLF001 - planner regression

    assert plan["type"] == "composite"
    assert plan["colors"] == ["green"]
    assert plan["objects"] == ["chair"]
    assert plan["open_objects"] == ["chair"]
    assert plan["yolo_classes"] == ["chair"]


def test_unreal_object_word_does_not_get_treated_as_detectable():
    """"door" and "cart" are physically plausible nouns but are not classes the
    configured yolo11s.pt (a closed-set COCO-80 checkpoint) can actually output.
    They must not be silently promoted to a detectable object just because they
    sound object-like -- that used to happen via a hardcoded suffix-word list.
    """
    planner = _planner()

    door_plan = planner._build_detection_plan("green_door")  # noqa: SLF001 - planner regression
    assert door_plan["type"] == "color"
    assert door_plan["colors"] == ["green"]
    assert door_plan["objects"] == []
    assert any("door" in reason for reason in door_plan["unsupported"])

    cart_plan = planner._build_detection_plan("blue_cart")  # noqa: SLF001 - planner regression
    assert cart_plan["type"] == "color"
    assert cart_plan["colors"] == ["blue"]
    assert cart_plan["objects"] == []
    assert "car" not in cart_plan["objects"]
    assert "car" not in cart_plan["yolo_classes"]
    assert any("cart" in reason for reason in cart_plan["unsupported"])


def test_registered_object_with_no_real_model_capability_is_flagged_distinctly():
    """"pallet" is a real registry entry (perception_targets.yaml), but none of
    its configured classes/fallback_classes overlap the checkpoint's real class
    list -- it is recognized by name yet currently undetectable. That must read
    differently from a totally unknown word like "salfjogasf".
    """
    planner = _planner()

    plan = planner._build_detection_plan("pallet")  # noqa: SLF001 - planner regression

    assert plan["type"] == "unsupported"
    assert any("pallet" in reason and "known target" in reason for reason in plan["unsupported"])


def test_non_object_like_free_text_requires_human():
    planner = _planner()

    plan = planner._build_detection_plan("love")  # noqa: SLF001 - planner regression

    assert plan["type"] == "unsupported"
    assert plan["colors"] == []
    assert plan["objects"] == []
    assert len(plan["unsupported"]) >= 1


def test_plain_color_object_composite_does_not_call_ocr(monkeypatch):
    planner = _planner()
    plan = planner._build_detection_plan("find green chair")  # noqa: SLF001 - planner regression

    def fail_ocr(*_args, **_kwargs):
        raise AssertionError("plain color+object composites must not call OCR")

    monkeypatch.setattr(planner, "_ocr_text_detections", fail_ocr)
    monkeypatch.setattr(planner, "_detect_color_regions", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(planner, "_run_yolo_provider", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(planner, "_match_composite_detections", lambda *_args, **_kwargs: [])
    monkeypatch.setattr("cv2.imread", lambda *_: object())

    detections = planner._run_detection_plan(  # noqa: SLF001 - planner regression
        "yolo11s_ocr",
        [{"image_path": "/tmp/nonexistent.jpg"}],
        "find green chair",
        "find green chair",
        plan,
        0.5,
    )

    assert detections == []
