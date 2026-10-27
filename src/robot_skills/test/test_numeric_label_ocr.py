import cv2
import numpy as np
import pytest

from robot_skills.executors.perception import PerceptionSkills


def label_image(tmp_path):
    image = np.zeros((160, 200, 3), dtype=np.uint8)
    cv2.rectangle(image, (40, 40), (110, 100), (240, 240, 240), -1)
    path = tmp_path / "label.png"
    assert cv2.imwrite(str(path), image)
    return str(path)


@pytest.mark.parametrize("text,confidence,matched", [
    ("06", 88, True), ("05", 88, False), ("106", 88, False), ("06", 20, False),
    ("O6", 90, False),
])
def test_numeric_label_ocr_does_not_force_target_or_relax_confidence(tmp_path, monkeypatch, text, confidence, matched):
    pytesseract = pytest.importorskip("pytesseract")
    calls = []

    def recognize(image, **kwargs):
        calls.append(kwargs)
        return {"text": [text], "conf": [confidence]}

    monkeypatch.setattr(pytesseract, "image_to_data", recognize)
    p = PerceptionSkills(None)
    result = p._tesseract_numeric_labels(label_image(tmp_path), "text_06", .5)
    assert calls and calls[0]["config"] == "--psm 7"
    assert calls[0]["timeout"] == 2
    assert any(d["matched_query"] for d in result) == matched
    for item in result:
        assert item["bbox_scope"] == "label_region"
        assert 0 <= item["bbox"][0] < item["bbox"][2] <= 200


def test_empty_image_has_no_candidate_ocr_calls(tmp_path, monkeypatch):
    pytesseract = pytest.importorskip("pytesseract")
    monkeypatch.setattr(pytesseract, "image_to_data", lambda *a, **kw: pytest.fail("no labels"))
    path = tmp_path / "empty.png"
    assert cv2.imwrite(str(path), np.zeros((160, 200, 3), dtype=np.uint8))
    assert PerceptionSkills(None)._tesseract_numeric_labels(str(path), "text_06", .5) == []
