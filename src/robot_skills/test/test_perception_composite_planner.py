from __future__ import annotations

from robot_skills.executors.perception import PerceptionSkills


def _planner() -> PerceptionSkills:
    return PerceptionSkills(node=None)


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

    detections = planner._run_detection_plan(  # noqa: SLF001 - planner regression
        "yolo11s_ocr",
        [{"image_path": "/tmp/nonexistent.jpg"}],
        "find green chair",
        "find green chair",
        plan,
        0.5,
    )

    assert detections == []
