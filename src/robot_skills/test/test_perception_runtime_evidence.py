from types import SimpleNamespace
import pytest

from robot_skills.executors.perception import PerceptionSkills


def broken(*args):
    raise ImportError("backend unavailable")


def test_all_ocr_errors_are_not_empty_success():
    perception = SimpleNamespace(_tesseract_detections=broken, _easyocr_detections=broken)
    with pytest.raises(RuntimeError, match="All OCR backends failed"):
        PerceptionSkills._ocr_text_detections(perception, "saved.jpg", "06", .5)
    assert len(perception._provider_errors) == 2


def test_one_working_ocr_backend_can_report_no_match():
    perception = SimpleNamespace(_tesseract_detections=lambda *args: [], _easyocr_detections=broken)
    assert PerceptionSkills._ocr_text_detections(perception, "saved.jpg", "06", .5) == []
    assert len(perception._provider_errors) == 1


@pytest.mark.parametrize("text,expected", [("06", True), ("6", True), ("106", False),
                                            ("60", False), ("07", False), ("0 6", True)])
def test_shelf_numbers_are_whole_tokens_with_optional_zero_padding(text, expected):
    assert bool(PerceptionSkills._matched_digit_tokens("shelf_label_06", text, .9, .7)) == expected


def test_generic_text_stays_exact_and_query_tokens_are_not_truncated():
    assert PerceptionSkills._matched_digit_tokens("text_06", "6", .9, .7) == []
    assert PerceptionSkills._required_digit_tokens("shelf_label_106") == ["106"]
    assert PerceptionSkills._matches_query("06 shelf_label_06 text_06", "ocr_text", "06", .9, .7)
    assert not PerceptionSkills._matches_query("shelf_label_06", "ocr_text", "06", .2, .7)


def test_completed_empty_detection_is_not_a_skill_failure(monkeypatch):
    from robot_skills.schemas import SkillContext
    p = PerceptionSkills(None)
    p._observations_by_run["test"] = [{"image_path": "saved.jpg"}]
    monkeypatch.setattr(p, "_run_detection_plan", lambda *args: [])
    monkeypatch.setattr(p, "_write_annotated_images", lambda *args, **kwargs: [])
    result = p.detect_known_object(SkillContext("test"), "shelf_label_06", provider="yolo11s_ocr")
    assert result.success and result.code == "OK"
    assert result.metrics["outcome"] == "NOT_FOUND"
    assert result.metrics["goal_satisfied"] is False


def test_failed_inference_does_not_become_not_found(monkeypatch):
    from robot_skills.schemas import SkillContext
    p = PerceptionSkills(None)
    p._observations_by_run["test"] = [{"image_path": "saved.jpg"}]
    monkeypatch.setattr(p, "_run_detection_plan", broken)
    result = p.detect_known_object(SkillContext("test"), "shelf_label_06", provider="yolo11s_ocr")
    assert not result.success and result.code == "FAILED_PERCEPTION"
    assert result.metrics["inference_completed"] is False


def test_unreadable_image_is_not_a_negative_detection(monkeypatch):
    from robot_skills.schemas import SkillContext
    p = PerceptionSkills(None)
    p._observations_by_run["test"] = [{"image_path": "missing.jpg"}]
    monkeypatch.setattr("cv2.imread", lambda *_: None)
    result = p.detect_known_object(SkillContext("test"), "color_red", provider="yolo11s")
    assert result.code == "FAILED_PERCEPTION" and not result.success
    assert result.metrics["inference_completed"] is False


def test_box_gap_is_not_an_implicit_substitute():
    p = PerceptionSkills(None)
    plan = p._build_detection_plan("red box")
    assert "suitcase" not in plan["yolo_classes"]
    assert plan["fallback_classes"] == []
    assert plan["unsupported"]


def test_explicit_single_digit_ocr_is_not_ambiguous():
    p = PerceptionSkills(None)
    assert p._build_detection_plan("text_6")["type"] == "text"


def test_multi_query_annotations_do_not_overwrite(monkeypatch, tmp_path):
    p = PerceptionSkills(None)
    monkeypatch.setattr(p, "_run_dir", lambda _: tmp_path)
    monkeypatch.setattr(p, "_draw_detections", lambda *args: True)
    detections = [{"image_path": "observation_001.jpg", "label": "cup", "confidence": .9}]
    a = p._write_annotated_images("test", "cup", detections)
    b = p._write_annotated_images("test", "bottle", detections)
    assert a[0]["annotated_image_path"] != b[0]["annotated_image_path"]


def test_small_label_resampling_preserves_exact_match_and_threshold(monkeypatch):
    import cv2
    import numpy as np
    import pytesseract
    p = PerceptionSkills(None)
    monkeypatch.setattr("cv2.imread", lambda *_: np.zeros((40, 80, 3), dtype=np.uint8))
    resize = cv2.resize
    def resample(*args, **kwargs):
        assert kwargs["interpolation"] == cv2.INTER_LINEAR
        return resize(*args, **kwargs)
    monkeypatch.setattr(cv2, "resize", resample)
    def recognize(image, config, output_type):
        assert config == "--psm 11 -c tessedit_char_whitelist=" + p._ocr_allowlist("text_06")
        assert image.shape[:2] == (80, 160)
        return {"text": ["06", "106", "BOXES"], "conf": [85, 90, 91],
                "left": [20, 50, 80], "top": [10, 10, 10],
                "width": [20, 20, 30], "height": [10, 10, 10]}
    monkeypatch.setattr(pytesseract, "image_to_data", recognize)
    result = p._tesseract_at_scale("saved.jpg", "text_06", .7, 2)
    assert [d["text"] for d in result if d["matched_query"]] == ["06"]
    assert result[0]["confidence"] == .85
    assert result[0]["bbox"] == [10, 5, 20, 10]
