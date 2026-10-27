from __future__ import annotations

import json
import math
import os
import re
import time
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.qos import qos_profile_sensor_data

from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result, now
from robot_skills.schemas import SkillContext, SkillResult

os.environ.setdefault("MPLCONFIGDIR", "/tmp/robot_harness_matplotlib")
os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/robot_harness_ultralytics")

try:
    from sensor_msgs.msg import Image
except Exception:  # pragma: no cover - lets non-ROS unit imports survive.
    Image = None


class PerceptionQueryAnalyzer:
    def __init__(self, target_config: dict[str, Any]):
        from robot_skills.perception_capabilities import normalize_targets
        self.target_config = normalize_targets(target_config)

    def analyze(self, query: str) -> dict[str, Any]:
        raw = str(query or "").strip()
        normalized = raw.lower().replace(" ", "_").replace("-", "_")
        shelf_label = re.fullmatch(r"shelf_label_(0?[1-6])", normalized)
        if shelf_label:
            token = f"{int(shelf_label.group(1)):02d}"
            return self._text_plan(raw, token, "shelf_label", [token])

        legacy_shelf_label = re.fullmatch(r"(0?[1-6])(?:番号|棚ラベル|label)", raw, re.IGNORECASE)
        if legacy_shelf_label:
            token = f"{int(legacy_shelf_label.group(1)):02d}"
            return self._text_plan(raw, token, "shelf_label", [token])

        text_query = re.fullmatch(r"(?:text|label|word|letter)_([A-Za-z0-9][A-Za-z0-9_]*)", normalized)
        if text_query:
            target_text = text_query.group(1).replace("_", "")
            # The explicit text_ prefix resolves the ambiguity of a bare digit.
            return self._text_plan(raw, target_text, "text", [])

        object_query = re.fullmatch(r"object_([a-z0-9_]+)", normalized)
        if object_query:
            object_name = object_query.group(1)
            if object_name in (self.target_config.get("objects") or {}):
                return self._object_plan(raw, [object_name])
            classes = {str(c).replace(" ", "_"): c for c in self.target_config.get("yolo_open_vocab_classes", [])}
            if object_name in classes:
                return {"type": "legacy", "query": classes[object_name]}
            return self._unsupported(raw, f'Object "{object_name}" is not configured in perception_targets.yaml.')

        color_query = re.fullmatch(r"color_([a-z0-9_]+)", normalized)
        if color_query:
            color_name = color_query.group(1)
            if color_name in (self.target_config.get("colors") or {}):
                return self._color_plan(raw, [color_name])
            return self._unsupported(raw, f'Color "{color_name}" is not configured in perception_targets.yaml.')

        composite = self._typed_composite(normalized)
        if composite:
            colors, objects = composite
            return self._composite_plan(raw, colors, objects)

        if re.fullmatch(r"0?[1-6]", raw) or re.fullmatch(r"[A-Za-z0-9]", raw):
            return self._unsupported(raw, f'Bare perception query "{raw}" is ambiguous. Use shelf_label_06, text_BOX, object_box, or color_red.')

        return {"type": "legacy", "query": raw}

    def _typed_composite(self, normalized: str) -> tuple[list[str], list[str]] | None:
        colors = self.target_config.get("colors") or {}
        objects = self.target_config.get("objects") or {}
        for color_name in colors:
            prefix = f"{color_name}_"
            if not normalized.startswith(prefix):
                continue
            object_name = normalized[len(prefix) :]
            if object_name in objects:
                return [color_name], [object_name]
        return None

    @staticmethod
    def _text_plan(query: str, target_text: str, text_kind: str, required_digits: list[str]) -> dict[str, Any]:
        return {
            "type": "text",
            "query": query,
            "normalized_query": f"text_{target_text}",
            "target_text": target_text,
            "text_kind": text_kind,
            "text_match": "exact_token",
            "colors": [],
            "objects": [],
            "yolo_classes": [],
            "fallback_classes": [],
            "fallback_requires_human": False,
            "unsupported": [],
            "required_digits": required_digits,
            "display_min_confidence": 0.70,
        }

    def _object_plan(self, query: str, objects: list[str]) -> dict[str, Any]:
        yolo_classes: set[str] = set()
        fallback_classes: set[str] = set()
        fallback_requires_human = False
        for object_name in objects:
            cfg = (self.target_config.get("objects") or {}).get(object_name, {})
            yolo_classes.update(str(item).lower().replace(" ", "_") for item in cfg.get("classes") or [])
            fallback_classes.update(str(item).lower().replace(" ", "_") for item in cfg.get("fallback_classes") or [])
            fallback_requires_human = fallback_requires_human or bool(cfg.get("fallback_requires_human"))
        return {
            "type": "object",
            "query": query,
            "normalized_query": " ".join(objects),
            "colors": [],
            "objects": objects,
            "text_kind": None,
            "yolo_classes": sorted(yolo_classes),
            "fallback_classes": sorted(fallback_classes),
            "fallback_requires_human": fallback_requires_human,
            "unsupported": [],
            "required_digits": [],
            "display_min_confidence": 0.60,
        }

    @staticmethod
    def _color_plan(query: str, colors: list[str]) -> dict[str, Any]:
        return {
            "type": "color",
            "query": query,
            "normalized_query": " ".join(f"color_region_{color}" for color in colors),
            "colors": colors,
            "objects": [],
            "text_kind": None,
            "yolo_classes": [],
            "fallback_classes": [],
            "fallback_requires_human": False,
            "unsupported": [],
            "required_digits": [],
            "display_min_confidence": 0.60,
        }

    def _composite_plan(self, query: str, colors: list[str], objects: list[str]) -> dict[str, Any]:
        plan = self._object_plan(query, objects)
        plan.update(
            {
                "type": "composite",
                "colors": colors,
                "normalized_query": " ".join([*colors, *objects]),
                "display_min_confidence": 0.60,
            }
        )
        return plan

    @staticmethod
    def _unsupported(query: str, reason: str) -> dict[str, Any]:
        return {
            "type": "unsupported",
            "query": query,
            "colors": [],
            "objects": [],
            "text_kind": None,
            "yolo_classes": [],
            "fallback_classes": [],
            "fallback_requires_human": False,
            "unsupported": [reason],
            "required_digits": [],
            "display_min_confidence": 0.70,
        }


class PerceptionSkills:
    def __init__(self, node):
        self.node = node
        self.config = self._load_config("perception_models.yaml")
        from robot_skills.perception_capabilities import normalize_targets
        self.target_config = normalize_targets(self._load_config("perception_targets.yaml"))
        self.query_analyzer = PerceptionQueryAnalyzer(self.target_config)
        self.color_topic = self.config.get("camera", {}).get("color_topic", "/camera/camera/color/image_raw")
        self.artifact_root = str(self.config.get("camera", {}).get("artifact_root", "debug-output/perception"))
        self.capture_wait_sec = float(self.config.get("camera", {}).get("capture_wait_sec", 1.5))
        self.ocr_config = self.config.get("ocr", {}) or {}
        self._latest_image = None
        self._latest_image_time = 0.0
        self._observations_by_run: dict[str, list[dict[str, Any]]] = {}
        self._yolo_models: dict[str, Any] = {}
        self._ocr_readers: dict[str, Any] = {}
        if Image is not None and hasattr(self.node, "create_subscription"):
            self.node.create_subscription(Image, self.color_topic, self._image_cb, qos_profile_sensor_data)

    def capture_observation(
        self,
        context: SkillContext,
        min_image_time_monotonic: float | None = None,
    ) -> SkillResult:
        started = now()
        deadline = time.monotonic() + min(context.max_duration_sec, self.capture_wait_sec)
        while not self._has_usable_image(min_image_time_monotonic) and time.monotonic() < deadline:
            if getattr(context, "cancel_check", lambda: False)():
                return make_result("capture_observation", False, codes.CANCELLED, "Capture cancelled.")
            time.sleep(0.02)

        if getattr(context, "cancel_check", lambda: False)():
            return make_result("capture_observation", False, codes.CANCELLED, "Capture cancelled.")

        if not self._has_usable_image(min_image_time_monotonic):
            return make_result(
                "capture_observation",
                False,
                codes.FAILED_PERCEPTION,
                "No fresh camera image has been received yet. Check camera topic QoS and /camera/camera/color/image_raw.",
                started,
                {
                    "mode": context.mode,
                    "color_topic": self.color_topic,
                    "latest_image_time_monotonic": self._latest_image_time,
                    "min_image_time_monotonic": min_image_time_monotonic,
                },
            )

        run_dir = self._run_dir(context.run_id)
        run_dir.mkdir(parents=True, exist_ok=True)
        index = len(self._observations_by_run.get(context.run_id, []))
        image_path = run_dir / f"observation_{index:03d}.jpg"
        json_path = run_dir / f"observation_{index:03d}.json"

        ok, error = self._write_image(self._latest_image, image_path)
        observation = {
            "run_id": context.run_id,
            "index": index,
            "color_topic": self.color_topic,
            "image_path": str(image_path),
            "image_time_monotonic": self._latest_image_time,
            "min_image_time_monotonic": min_image_time_monotonic,
            "mode": context.mode,
        }
        json_path.write_text(json.dumps(observation, ensure_ascii=False, indent=2), encoding="utf-8")
        self._observations_by_run.setdefault(context.run_id, []).append(observation)

        if not ok:
            return make_result(
                "capture_observation",
                False,
                codes.FAILED_PERCEPTION,
                f"Camera image was received but could not be written: {error}",
                started,
                observation,
                artifacts=[str(json_path)],
            )

        return make_result(
            "capture_observation",
            True,
            codes.OK,
            "Observation image captured.",
            started,
            observation,
            observations=[observation],
            artifacts=[str(image_path), str(json_path)],
        )

    def detect_known_object(
        self,
        context: SkillContext,
        query: str,
        min_confidence: float = 0.5,
        provider: str = "stub",
    ) -> SkillResult:
        started = now()
        provider = provider or "stub"

        # Classify the query itself before anything that depends on captured
        # images or provider choice -- a query with no color/object/text
        # signal at all can't be helped by more pixels or a different
        # detector, so this must short-circuit regardless of provider/stub
        # state (including during dry-run, so this path is actually testable
        # without a real camera).
        plan = self._build_detection_plan(query)
        if plan["type"] in {"abstract", "unsupported"}:
            reasons = plan.get("unsupported") or [f'Unsupported perception query: "{query}".']
            return make_result(
                "detect_known_object",
                False,
                codes.NEEDS_HUMAN,
                "Perception query requires human confirmation: " + "; ".join(reasons),
                started,
                {
                    "query": query,
                    "provider": provider,
                    "perception_plan": plan,
                    "observation_count": len(self._observations_by_run.get(context.run_id, [])),
                },
            )

        if provider in {"stub", "yolo_or_stub"}:
            provider = "yolo11s_ocr" if self._observations_by_run.get(context.run_id) else "stub"
        if provider == "stub":
            return make_result(
                "detect_known_object",
                True,
                codes.PERCEPTION_STUB,
                f'Perception detector for "{query}" is not implemented yet.',
                started,
                {"query": query, "min_confidence": min_confidence, "provider": provider, "stub": True},
            )

        observations = self._observations_by_run.get(context.run_id, [])
        if not observations:
            return make_result(
                "detect_known_object",
                True,
                codes.PERCEPTION_STUB,
                "No captured observations are available for detector input.",
                started,
                {"query": query, "provider": provider, "stub": True},
            )

        if provider not in {"yolo11s", "yolo11s_ocr"}:
            return make_result(
                "detect_known_object",
                False,
                codes.FAILED_SCHEMA,
                f"Unsupported perception provider: {provider}",
                started,
                {"query": query, "provider": provider},
            )
        matching_query = self._matching_query_from_plan(query, plan)
        self._provider_errors = []
        try:
            detections = self._run_detection_plan(provider, observations, query, matching_query, plan, min_confidence)
        except (ImportError, FileNotFoundError, RuntimeError) as exc:
            return make_result(
                "detect_known_object",
                False,
                codes.FAILED_PERCEPTION,
                f"{provider} inference could not complete: {exc}",
                started,
                {"query": query, "provider": provider, "inference_completed": False,
                 "provider_errors": self._provider_errors or [{"error": str(exc)}]},
            )

        required_digits = list(plan.get("required_digits") or self._required_digit_tokens(query))
        primitive_results = self._primitive_results_from_detections(plan, detections)
        if len(required_digits) >= 1 and plan["type"] in {"text", "composite"}:
            matched_digits = sorted(
                {
                    token
                    for item in detections
                    for token in item.get("matched_tokens", [])
                    if token in required_digits
                }
            )
            found = set(required_digits).issubset(set(matched_digits))
        elif plan["type"] == "composite":
            matched_digits = []
            found = any(item.get("matched_query") and item.get("label") == "composite_match" for item in detections)
        else:
            matched_digits = []
            found = any(item.get("matched_query") for item in detections)
        approximate_found = any(item.get("approximate_match") for item in detections)
        unsupported_reasons = plan.get("unsupported") or []
        if found and unsupported_reasons:
            code = codes.NEEDS_HUMAN
            found = False
        elif approximate_found and not found:
            code = codes.NEEDS_HUMAN
        else:
            code = codes.OK
        if unsupported_reasons and any(item.get("matched_query") for item in detections):
            message = (
                f'Found partial candidates for "{query}", but human confirmation is required: '
                + "; ".join(unsupported_reasons)
            )
        elif found:
            if plan["type"] == "composite":
                message = f'Found "{query}" with matching color and object evidence.'
            else:
                message = f'Found "{query}".'
        elif approximate_found:
            message = f'Found only approximate candidates for "{query}". Please confirm.'
        elif plan["type"] == "composite" and (
            primitive_results.get("color_found") or primitive_results.get("object_found")
        ):
            message = (
                f'Found partial evidence for "{query}": '
                f"color_found={primitive_results.get('color_found')}, "
                f"object_found={primitive_results.get('object_found')}, "
                f"both_found={primitive_results.get('composite_found')}."
            )
        elif required_digits:
            message = (
                f'No confident match for "{query}" in captured observations. '
                f"Matched digits: {matched_digits or 'none'}."
            )
        else:
            message = f'No confident match for "{query}" in captured observations.'
        annotated_images = self._write_annotated_images(
            context.run_id,
            query,
            detections,
            display_min_confidence=float(plan.get("display_min_confidence", 0.5)),
        )
        return make_result(
            "detect_known_object",
            code == codes.OK,
            code,
            message,
            started,
            {
                "query": query,
                "matching_query": matching_query,
                "perception_plan": plan,
                "min_confidence": min_confidence,
                "provider": provider,
                "detections": detections,
                "observation_count": len(observations),
                "inference_completed": True,
                "goal_satisfied": found,
                "outcome": "FOUND" if found else ("NOT_FOUND" if code == codes.OK else "NEEDS_CONFIRMATION"),
                "provider_warnings": self._provider_errors,
                "required_digits": required_digits,
                "matched_digits": matched_digits,
                "primitive_results": primitive_results,
                "annotated_images": annotated_images,
            },
            artifacts=[item["annotated_image_path"] for item in annotated_images],
        )

    def _image_cb(self, msg):
        self._latest_image = msg
        self._latest_image_time = time.monotonic()

    def _has_usable_image(self, min_image_time_monotonic: float | None) -> bool:
        if self._latest_image is None:
            return False
        if min_image_time_monotonic is None:
            return True
        return self._latest_image_time > float(min_image_time_monotonic)

    def _run_dir(self, run_id: str) -> Path:
        root = Path(self.artifact_root)
        if not root.is_absolute():
            root = Path.cwd() / root
        return root / run_id

    def _write_image(self, msg, path: Path) -> tuple[bool, str | None]:
        try:
            array = self._image_to_array(msg)
            import cv2

            if array.ndim == 3 and msg.encoding.lower() == "rgb8":
                array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(path), array)
            return True, None
        except Exception as exc:
            return False, repr(exc)

    @staticmethod
    def _image_to_array(msg):
        import numpy as np

        encoding = msg.encoding.lower()
        channels = 1
        dtype = np.uint8
        if encoding in {"rgb8", "bgr8"}:
            channels = 3
        elif encoding in {"rgba8", "bgra8"}:
            channels = 4
        elif encoding in {"mono16", "16uc1"}:
            dtype = np.uint16
        array = np.frombuffer(msg.data, dtype=dtype)
        if channels == 1:
            return array.reshape((msg.height, msg.width))
        return array.reshape((msg.height, msg.width, channels))

    def _run_detection_plan(
        self,
        provider: str,
        observations: list[dict[str, Any]],
        original_query: str,
        query: str,
        plan: dict[str, Any],
        min_confidence: float,
    ) -> list[dict[str, Any]]:
        detections: list[dict[str, Any]] = []
        detection_query = str(plan.get("target_text") or original_query)
        for observation in observations:
            image_path = observation["image_path"]
            import cv2
            if cv2.imread(image_path) is None:
                raise RuntimeError(f"Observation image is missing or unreadable: {image_path}")
            if plan["type"] in {"color", "composite"}:
                for color in plan.get("colors", []):
                    detections.extend(self._detect_color_regions(image_path, color, min_confidence))
            if plan["type"] == "text" or (plan["type"] == "composite" and plan.get("text_kind")):
                detections.extend(self._ocr_text_detections(image_path, detection_query, min_confidence))
        if plan["type"] in {"object", "composite"} and plan.get("objects"):
            detections.extend(self._run_yolo_provider(provider, observations, query, plan, min_confidence))
        if plan["type"] == "composite":
            detections.extend(self._match_composite_detections(detections, original_query, min_confidence, plan))
        return detections

    def _run_yolo_provider(
        self,
        provider: str,
        observations: list[dict[str, Any]],
        query: str,
        plan: dict[str, Any],
        min_confidence: float,
    ) -> list[dict[str, Any]]:
        cfg = self.config.get("providers", {}).get(provider, {})
        model = self._load_yolo_model(provider, cfg)
        target_classes = set(plan.get("yolo_classes") or [])
        fallback_classes = set(plan.get("fallback_classes") or [])
        fallback_requires_human = bool(plan.get("fallback_requires_human"))
        detections: list[dict[str, Any]] = []
        for observation in observations:
            image_path = observation["image_path"]
            results = model.predict(
                source=image_path,
                conf=max(float(cfg.get("min_confidence", min_confidence)), min_confidence),
                imgsz=int(cfg.get("image_size", 640)),
                device=self._select_yolo_device(str(cfg.get("device", "auto"))),
                verbose=False,
            )
            for result in results:
                names = getattr(result, "names", {}) or {}
                boxes = getattr(result, "boxes", None)
                if boxes is None:
                    continue
                for box in boxes:
                    cls_id = int(box.cls[0]) if hasattr(box, "cls") else -1
                    label = str(names.get(cls_id, cls_id))
                    confidence = float(box.conf[0]) if hasattr(box, "conf") else 0.0
                    xyxy = [float(v) for v in box.xyxy[0].tolist()]
                    text = self._ocr_crop_if_needed(provider, image_path, xyxy, str(cfg.get("ocr_backend", "auto")))
                    normalized_label = self._normalize_token(label)
                    exact_match = normalized_label in target_classes or self._matches_query(query, label, text, confidence, min_confidence)
                    approximate_match = normalized_label in fallback_classes and normalized_label not in target_classes
                    detections.append(
                        {
                            "image_path": image_path,
                            "label": label,
                            "confidence": confidence,
                            "bbox": xyxy,
                            "text": text,
                            "matched_query": bool(exact_match and not (approximate_match and fallback_requires_human)),
                            "approximate_match": bool(approximate_match),
                            "fallback_requires_human": bool(approximate_match and fallback_requires_human),
                            "matched_tokens": self._matched_digit_tokens(query, text, confidence, min_confidence),
                            "primitive": "object",
                        }
                    )
        return detections

    def _write_annotated_images(
        self,
        run_id: str,
        query: str,
        detections: list[dict[str, Any]],
        display_min_confidence: float = 0.5,
    ) -> list[dict[str, Any]]:
        by_image: dict[str, list[dict[str, Any]]] = {}
        for detection in detections:
            confidence = float(detection.get("confidence") or 0.0)
            matched = bool(detection.get("matched_query") or detection.get("matched_tokens"))
            if not matched and confidence < display_min_confidence:
                continue
            image_path = detection.get("image_path")
            if image_path:
                by_image.setdefault(str(image_path), []).append(detection)

        annotated: list[dict[str, Any]] = []
        run_dir = self._run_dir(run_id)
        for image_path, image_detections in by_image.items():
            import hashlib
            query_key = hashlib.sha256(query.encode("utf-8")).hexdigest()[:12]
            output_path = run_dir / f"annotated_{Path(image_path).stem}_{query_key}.jpg"
            ok = self._draw_detections(image_path, output_path, query, image_detections)
            if not ok:
                continue
            matched = [item for item in image_detections if item.get("matched_query") or item.get("matched_tokens")]
            top = max(image_detections, key=lambda item: float(item.get("confidence") or 0.0))
            annotated.append(
                {
                    "image_path": image_path,
                    "annotated_image_path": str(output_path),
                    "matched": bool(matched),
                    "detection_count": len(image_detections),
                    "matched_count": len(matched),
                    "top_label": str(top.get("label", "")),
                    "top_confidence": float(top.get("confidence") or 0.0),
                    "query": query,
                }
            )
        annotated.sort(key=lambda item: (not item["matched"], -item["top_confidence"], item["annotated_image_path"]))
        return annotated

    @staticmethod
    def _draw_detections(
        image_path: str,
        output_path: Path,
        query: str,
        detections: list[dict[str, Any]],
    ) -> bool:
        try:
            import cv2
        except Exception:
            return False
        image = cv2.imread(image_path)
        if image is None:
            return False

        height, width = image.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        query_caption = PerceptionSkills._safe_overlay_query(query)
        cv2.putText(
            image,
            query_caption,
            (12, 24),
            font,
            0.65,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(image, query_caption, (12, 24), font, 0.65, (20, 20, 20), 1, cv2.LINE_AA)

        for detection in detections:
            bbox = detection.get("bbox")
            if not bbox or len(bbox) != 4:
                continue
            x1, y1, x2, y2 = [int(round(float(v))) for v in bbox]
            x1, x2 = sorted((max(0, min(width - 1, x1)), max(0, min(width - 1, x2))))
            y1, y2 = sorted((max(0, min(height - 1, y1)), max(0, min(height - 1, y2))))
            if x2 <= x1 or y2 <= y1:
                continue

            matched = bool(detection.get("matched_query") or detection.get("matched_tokens"))
            confidence = float(detection.get("confidence") or 0.0)
            color = (60, 190, 80) if matched else (0, 190, 255)
            if confidence < 0.5:
                color = (70, 70, 230)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            label = str(detection.get("label") or "detection")
            text = str(detection.get("text") or "").strip()
            caption = f"{label} {confidence:.2f}"
            if text:
                caption = f"{caption} OCR:{text[:16]}"
            if matched:
                caption = f"MATCH {caption}"

            (tw, th), _ = cv2.getTextSize(caption, font, 0.45, 1)
            label_y1 = max(0, y1 - th - 8)
            cv2.rectangle(image, (x1, label_y1), (min(width - 1, x1 + tw + 8), y1), color, -1)
            cv2.putText(image, caption, (x1 + 4, max(12, y1 - 5)), font, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

        output_path.parent.mkdir(parents=True, exist_ok=True)
        return bool(cv2.imwrite(str(output_path), image))

    @staticmethod
    def _safe_overlay_query(query: str) -> str:
        import re

        digits = re.findall(r"\d+", query)
        if digits:
            normalized = " ".join(token[-2:] if len(token) > 2 else token for token in digits)
            return f"query: digits {normalized}"
        try:
            query.encode("ascii")
            return f"query: {query[:48]}"
        except UnicodeEncodeError:
            return "query: object"

    def _ocr_text_detections(self, image_path: str, query: str, min_confidence: float) -> list[dict[str, Any]]:
        detections: list[dict[str, Any]] = []
        completed = 0
        errors = []
        for name, detector in (("tesseract", self._tesseract_detections), ("easyocr", self._easyocr_detections)):
            try:
                detections.extend(detector(image_path, query, min_confidence))
                completed += 1
            except Exception as exc:
                errors.append({"backend": name, "image_path": image_path,
                               "error_type": type(exc).__name__, "error": str(exc)})
        self._provider_errors = getattr(self, "_provider_errors", []) + errors
        if not completed:
            raise RuntimeError("All OCR backends failed; absence of detections is not a NOT_FOUND result")
        return detections

    @staticmethod
    def _full_image_bbox(image_path: str) -> list[float]:
        import cv2

        image = cv2.imread(image_path)
        if image is None:
            return [0.0, 0.0, 0.0, 0.0]
        height, width = image.shape[:2]
        return [0.0, 0.0, float(width), float(height)]

    def _detect_color_regions(self, image_path: str, color_name: str, min_confidence: float) -> list[dict[str, Any]]:
        import cv2
        import numpy as np

        image = cv2.imread(image_path)
        if image is None:
            return []
        color_cfg = (self.target_config.get("colors") or {}).get(color_name, {})
        ranges = color_cfg.get("hsv_ranges") or []
        if not ranges:
            return []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = None
        for hsv_range in ranges:
            lower = np.array(hsv_range.get("lower", [0, 0, 0]), dtype=np.uint8)
            upper = np.array(hsv_range.get("upper", [180, 255, 255]), dtype=np.uint8)
            current = cv2.inRange(hsv, lower, upper)
            mask = current if mask is None else (mask | current)
        if mask is None:
            return []
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections: list[dict[str, Any]] = []
        area_total = float(image.shape[0] * image.shape[1])
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < max(500.0, area_total * 0.0025):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 18 or h < 18:
                continue
            aspect = float(w) / float(max(h, 1))
            if aspect < 0.35 or aspect > 2.8:
                continue
            fill_ratio = float(area) / float(max(w * h, 1))
            if fill_ratio < 0.35:
                continue
            confidence = min(0.92, 0.50 + area / max(area_total * 0.10, 1.0) + fill_ratio * 0.15)
            if confidence < min_confidence:
                continue
            detections.append(
                {
                    "image_path": image_path,
                    "label": f"color_region_{color_name}",
                    "confidence": confidence,
                    "bbox": [float(x), float(y), float(x + w), float(y + h)],
                    "text": None,
                    "matched_query": confidence >= min_confidence,
                    "matched_tokens": [],
                    "primitive": "color",
                    "color": color_name,
                    "area_px": float(area),
                    "aspect_ratio": aspect,
                    "fill_ratio": fill_ratio,
                }
            )
        return detections

    def _match_composite_detections(
        self,
        detections: list[dict[str, Any]],
        query: str,
        min_confidence: float,
        plan: dict[str, Any] | None = None,
    ) -> list[dict[str, Any]]:
        plan = plan if plan is not None else self._build_detection_plan(query)
        colors = set(plan.get("colors") or [])
        classes = {self._normalize_token(name) for name in plan.get("yolo_classes", [])}

        def usable(item):
            try:
                confidence = float(item.get("confidence", 0))
                bbox = item.get("bbox") or []
                return (item.get("matched_query") is True and bool(item.get("image_path"))
                        and not item.get("approximate_match") and not item.get("fallback_requires_human")
                        and math.isfinite(confidence) and min_confidence <= confidence <= 1
                        and len(bbox) == 4 and all(math.isfinite(float(v)) for v in bbox)
                        and float(bbox[2]) > float(bbox[0]) and float(bbox[3]) > float(bbox[1]))
            except (TypeError, ValueError):
                return False

        color_items = [item for item in detections if item.get("primitive") == "color"
                       and item.get("color") in colors and usable(item)]
        object_items = [item for item in detections if item.get("primitive") == "object"
                        and self._normalize_token(str(item.get("label", ""))) in classes and usable(item)]
        composite: list[dict[str, Any]] = []
        for color_item in color_items:
            for object_item in object_items:
                # Pixel coordinates are comparable only within the same observation.
                if color_item["image_path"] != object_item["image_path"]:
                    continue
                overlap = self._bbox_overlap_ratio(color_item.get("bbox"), object_item.get("bbox"))
                if overlap < 0.15:
                    continue
                confidence = min(
                    float(color_item.get("confidence") or 0.0),
                    float(object_item.get("confidence") or 0.0),
                )
                if confidence < min_confidence:
                    continue
                approximate = bool(object_item.get("approximate_match") or object_item.get("fallback_requires_human"))
                composite.append(
                    {
                        "image_path": object_item.get("image_path"),
                        "label": "composite_match",
                        "confidence": confidence,
                        "bbox": object_item.get("bbox"),
                        "text": None,
                        "matched_query": not approximate,
                        "approximate_match": approximate,
                        "matched_tokens": [],
                        "primitive": "composite",
                        "query": query,
                        "color_label": color_item.get("label"),
                        "object_label": object_item.get("label"),
                        "color_image_path": color_item["image_path"],
                        "object_image_path": object_item["image_path"],
                        "color_bbox": color_item["bbox"],
                        "object_bbox": object_item["bbox"],
                        "overlap_ratio": overlap,
                    }
                )
        return composite

    @staticmethod
    def _primitive_results_from_detections(plan: dict[str, Any], detections: list[dict[str, Any]]) -> dict[str, Any]:
        colors = list(plan.get("colors") or [])
        objects = list(plan.get("objects") or [])
        matched_colors = sorted(
            {
                str(item.get("color"))
                for item in detections
                if item.get("primitive") == "color" and item.get("matched_query") and item.get("color")
            }
        )
        matched_objects = sorted(
            {
                str(item.get("object_name") or item.get("label"))
                for item in detections
                if item.get("primitive") == "object" and item.get("matched_query")
            }
        )
        composite_matches = [item for item in detections if item.get("primitive") == "composite" and item.get("matched_query")]
        return {
            "colors": colors,
            "objects": objects,
            "matched_colors": matched_colors,
            "matched_objects": matched_objects,
            "color_found": bool(matched_colors),
            "object_found": bool(matched_objects),
            "composite_found": bool(composite_matches),
            "composite_match_count": len(composite_matches),
        }

    @staticmethod
    def _bbox_overlap_ratio(a: Any, b: Any) -> float:
        if not a or not b or len(a) != 4 or len(b) != 4:
            return 0.0
        ax1, ay1, ax2, ay2 = [float(v) for v in a]
        bx1, by1, bx2, by2 = [float(v) for v in b]
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        object_area = max(1.0, (bx2 - bx1) * (by2 - by1))
        return inter / object_area

    def _load_yolo_model(self, provider: str, cfg: dict[str, Any]):
        if provider in self._yolo_models:
            return self._yolo_models[provider]
        from ultralytics import YOLO

        weights = str(cfg.get("weights", "models/perception/yolo11s.pt"))
        path = self._workspace_path(weights)
        if not path.exists():
            raise FileNotFoundError(str(path))
        model = YOLO(str(path))
        if self.target_config.get("schema_version") == 2:
            names = model.names.values() if isinstance(model.names, dict) else model.names
            if set(names) != set(self.target_config["detector_classes"]):
                raise ValueError("Loaded detector classes do not match perception_targets.yaml; review capability registration before inference.")
        self._yolo_models[provider] = model
        return model

    @staticmethod
    def _select_yolo_device(configured: str):
        if configured in {"", "none", "None"}:
            return None
        if configured == "auto":
            try:
                import torch

                return 0 if torch.cuda.is_available() else "cpu"
            except Exception:
                return "cpu"
        if configured.startswith("cuda"):
            try:
                import torch

                return configured if torch.cuda.is_available() else "cpu"
            except Exception:
                return "cpu"
        return configured

    def _ocr_crop_if_needed(
        self,
        provider: str,
        image_path: str,
        bbox: list[float],
        backend: str = "auto",
    ) -> str | None:
        if provider != "yolo11s_ocr":
            return None
        backend = backend.lower()
        if backend in {"auto", "tesseract", "pytesseract"}:
            try:
                text = self._tesseract_crop(image_path, bbox)
                if text:
                    return text
            except Exception:
                if backend in {"tesseract", "pytesseract"}:
                    return None
        if backend in {"auto", "easyocr"}:
            try:
                text = self._easyocr_crop(image_path, bbox)
                if text:
                    return text
            except Exception:
                return None
        return None

    def _tesseract_crop(self, image_path: str, bbox: list[float], query: str = "") -> str | None:
        try:
            import cv2
            import pytesseract

            self._configure_tesseract(pytesseract)
            image = cv2.imread(image_path)
            if image is None:
                return None
            x1, y1, x2, y2 = [max(0, int(v)) for v in bbox]
            crop = image[y1:y2, x1:x2]
            allowlist = "0123456789"
            if any(ch.isalpha() for ch in query) or any(token in query.lower() for token in ["letter", "text", "label"]):
                allowlist = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
            text = pytesseract.image_to_string(crop, config=f"--psm 7 -c tessedit_char_whitelist={allowlist}")
            return text.strip()
        except Exception:
            return None

    def _tesseract_detections(self, image_path: str, query: str, min_confidence: float) -> list[dict[str, Any]]:
        detections = self._tesseract_at_scale(image_path, query, min_confidence, 1.0)
        # Small shelf labels need more pixels; keep the confidence threshold unchanged.
        if self._required_digit_tokens(query) and not any(item["matched_tokens"] for item in detections):
            detections.extend(self._tesseract_at_scale(image_path, query, min_confidence, 2.0))
        if self._required_digit_tokens(query) and not any(item["matched_tokens"] for item in detections):
            detections.extend(self._tesseract_numeric_labels(image_path, query, min_confidence))
        return detections

    def _tesseract_numeric_labels(self, image_path, query, min_confidence):
        import cv2
        import numpy as np
        import pytesseract

        self._configure_tesseract(pytesseract)
        image = cv2.imread(image_path)
        if image is None:
            raise RuntimeError(f"Cannot read OCR image: {image_path}")
        # Geometry comes from the image, never from the requested digits.
        mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (0, 0, 160), (179, 60, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections, attempts = [], 0
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            rectangle = cv2.minAreaRect(contour)
            width, height = rectangle[1]
            if min(width, height) < 12 or max(width, height) > 240:
                continue
            if max(width, height) / min(width, height) > 3 or cv2.contourArea(contour) / (width * height) < .55:
                continue
            points = cv2.boxPoints(rectangle)
            sums, differences = points.sum(axis=1), points[:, 1] - points[:, 0]
            source = points[[sums.argmin(), differences.argmin(), sums.argmax(), differences.argmax()]]
            if len(np.unique(source, axis=0)) != 4:
                continue
            w = max(1, int(np.linalg.norm(source[1] - source[0]) * 3))
            h = max(1, int(np.linalg.norm(source[3] - source[0]) * 3))
            destination = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]])
            crop = cv2.warpPerspective(image, cv2.getPerspectiveTransform(source, destination), (w, h))
            crop = cv2.copyMakeBorder(crop, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=(255, 255, 255))
            data = pytesseract.image_to_data(
                crop, config="--psm 7", output_type=pytesseract.Output.DICT, timeout=2)
            for text, raw_confidence in zip(data.get("text", []), data.get("conf", [])):
                text = str(text).strip()
                confidence = float(raw_confidence) / 100
                if not text.isascii() or not text.isdigit() or confidence < min_confidence:
                    continue
                lo, hi = points.min(axis=0), points.max(axis=0)
                detections.append({
                    "image_path": image_path, "label": "ocr_text", "text": text,
                    "confidence": min(.99, confidence),
                    "bbox": [max(0., float(lo[0])), max(0., float(lo[1])),
                             min(float(image.shape[1]), float(hi[0])), min(float(image.shape[0]), float(hi[1]))],
                    "matched_query": self._matches_query(query, "ocr_text", text, confidence, min_confidence),
                    "matched_tokens": self._matched_digit_tokens(query, text, confidence, min_confidence),
                    "primitive": "text", "ocr_backend": "tesseract", "ocr_scale": 3,
                    "ocr_preprocess": "rectified_light_label", "bbox_scope": "label_region",
                })
            attempts += 1
            if attempts >= 8:
                break
        return detections

    def _tesseract_at_scale(self, image_path: str, query: str, min_confidence: float, scale: float) -> list[dict[str, Any]]:
        import cv2
        import pytesseract

        self._configure_tesseract(pytesseract)
        image = cv2.imread(image_path)
        if image is None:
            raise RuntimeError(f"Cannot read OCR image: {image_path}")
        if scale != 1.0:
            # Avoid cubic ringing on small label strokes; retain exact-token and
            # confidence checks after recognition rather than relaxing acceptance.
            image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        allowlist = PerceptionSkills._ocr_allowlist(query)
        data = pytesseract.image_to_data(
            image,
            config=f"--psm 11 -c tessedit_char_whitelist={allowlist}",
            output_type=pytesseract.Output.DICT,
        )
        detections: list[dict[str, Any]] = []
        count = len(data.get("text", []))
        for index in range(count):
            text = str(data["text"][index] or "").strip()
            if not text:
                continue
            try:
                conf_raw = float(data["conf"][index])
            except Exception:
                conf_raw = -1.0
            if conf_raw < 0:
                continue
            confidence = max(0.0, min(0.99, conf_raw / 100.0))
            if confidence < max(0.35, min_confidence - 0.2):
                continue
            x = float(data["left"][index])
            y = float(data["top"][index])
            w = float(data["width"][index])
            h = float(data["height"][index])
            if w < 3.0 or h < 3.0:
                continue
            detections.append(
                {
                    "image_path": image_path,
                    "label": "ocr_text",
                    "confidence": confidence,
                    "bbox": [x / scale, y / scale, (x + w) / scale, (y + h) / scale],
                    "text": text,
                    "matched_query": PerceptionSkills._matches_query(query, "ocr_text", text, confidence, min_confidence),
                    "matched_tokens": PerceptionSkills._matched_digit_tokens(query, text, confidence, min_confidence),
                    "primitive": "text",
                    "ocr_backend": "tesseract",
                    "ocr_scale": scale,
                }
            )
        return detections

    def _easyocr_crop(self, image_path: str, bbox: list[float], query: str = "") -> str | None:
        import cv2
        import easyocr
        import torch

        image = cv2.imread(image_path)
        if image is None:
            return None
        x1, y1, x2, y2 = [max(0, int(v)) for v in bbox]
        crop = image[y1:y2, x1:x2]
        if crop.size == 0:
            return None
        reader = self._ocr_readers.get("en")
        if reader is None:
            reader = self._create_easyocr_reader(easyocr, torch)
            self._ocr_readers["en"] = reader
        allowlist = "0123456789"
        if any(ch.isalpha() for ch in query) or any(token in query.lower() for token in ["letter", "text", "label"]):
            allowlist = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
        results = reader.readtext(crop, allowlist=allowlist, detail=0)
        text = "".join(str(item) for item in results).strip()
        return text or None

    def _easyocr_detections(self, image_path: str, query: str, min_confidence: float) -> list[dict[str, Any]]:
        import cv2
        import easyocr
        import torch

        image = cv2.imread(image_path)
        if image is None:
            return []
        reader = self._ocr_readers.get("en")
        if reader is None:
            reader = self._create_easyocr_reader(easyocr, torch)
            self._ocr_readers["en"] = reader
        detections: list[dict[str, Any]] = []
        for bbox_points, text, confidence in reader.readtext(
            image,
            allowlist=self._ocr_allowlist(query),
            detail=1,
        ):
            text = str(text or "").strip()
            confidence = float(confidence or 0.0)
            if not text or confidence < max(0.35, min_confidence - 0.2):
                continue
            xs = [float(point[0]) for point in bbox_points]
            ys = [float(point[1]) for point in bbox_points]
            bbox = [min(xs), min(ys), max(xs), max(ys)]
            if bbox[2] - bbox[0] < 3.0 or bbox[3] - bbox[1] < 3.0:
                continue
            detections.append(
                {
                    "image_path": image_path,
                    "label": "ocr_text",
                    "confidence": confidence,
                    "bbox": bbox,
                    "text": text,
                    "matched_query": self._matches_query(query, "ocr_text", text, confidence, min_confidence),
                    "matched_tokens": self._matched_digit_tokens(query, text, confidence, min_confidence),
                    "primitive": "text",
                    "ocr_backend": "easyocr",
                }
            )
        return detections

    def _configure_tesseract(self, pytesseract_module) -> None:
        command = self._workspace_path(str(self.ocr_config.get("tesseract_cmd", "tesseract")))
        if command.is_file():
            pytesseract_module.pytesseract.tesseract_cmd = str(command)
        tessdata = self._workspace_path(str(self.ocr_config.get("tessdata_prefix", "")))
        if tessdata.is_dir():
            os.environ["TESSDATA_PREFIX"] = str(tessdata)

    def _create_easyocr_reader(self, easyocr_module, torch_module):
        languages = [str(item) for item in self.ocr_config.get("easyocr_languages", ["en"])]
        model_dir = self._workspace_path(
            str(self.ocr_config.get("easyocr_model_dir", "models/perception/easyocr"))
        )
        return easyocr_module.Reader(
            languages,
            gpu=bool(torch_module.cuda.is_available()),
            model_storage_directory=str(model_dir),
            user_network_directory=str(model_dir / "user_network"),
            download_enabled=bool(self.ocr_config.get("easyocr_download_enabled", False)),
            verbose=False,
        )

    @staticmethod
    def _workspace_path(value: str) -> Path:
        path = Path(value).expanduser()
        if path.is_absolute():
            return path
        root = Path(os.environ.get("ROBOT_WS", str(Path.cwd()))).expanduser()
        return root / path

    @staticmethod
    def _ocr_allowlist(query: str) -> str:
        allowlist = "0123456789"
        if any(ch.isalpha() for ch in query) or any(token in query.lower() for token in ["letter", "text", "label"]):
            allowlist = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
        return allowlist

    def _build_detection_plan(self, query: str) -> dict[str, Any]:
        analyzed = self.query_analyzer.analyze(query)
        capable_classes = self._capable_class_map()
        if analyzed.get("type") == "legacy" and str(query).startswith("object_"):
            query = analyzed.get("query", query)

        if analyzed.get("type") in {"text", "unsupported"}:
            return analyzed
        if analyzed.get("type") != "legacy":
            # Strict-typed match (object_X / color_X / COLOR_OBJECT): the
            # analyzer already resolved exact registry names, but its own
            # yolo_classes computation predates the capability model and just
            # dumps every configured class regardless of whether the real
            # model can see it. Recompute through the same capability-aware
            # path as free text, so "red_bucket" and "red bucket" produce
            # identical results instead of two silently-diverging answers.
            return self._finalize_detection_plan(
                query,
                colors=list(analyzed.get("colors") or []),
                registered_objects=list(analyzed.get("objects") or []),
                open_objects=[],
                text_kind=analyzed.get("text_kind"),
                unsupported=list(analyzed.get("unsupported") or []),
                capable_classes=capable_classes,
            )

        normalized = self._normalize_text(query)
        colors = self._match_named_primitives(query, self.target_config.get("colors") or {})
        registered_objects = self._match_named_primitives(query, self.target_config.get("objects") or {})
        text_kind = self._text_kind(query, normalized)
        unsupported = self._unsupported_reasons(normalized)
        abstract = self._abstract_reasons(normalized)

        # Open vocabulary: a word with no registry entry at all is only worth
        # attempting if it is *literally* one of the model's real classes --
        # never a guess about which nouns "sound like" objects.
        open_objects: list[str] = []
        unknown_part = self._unknown_object_part(query, colors)
        if not registered_objects and not text_kind and unknown_part:
            open_match = self._open_vocab_match(unknown_part, capable_classes)
            if open_match:
                open_objects.append(open_match)
                # The object half was found via one word of unknown_part; any
                # other leftover word (e.g. an unregistered color like
                # "turquoise" in "turquoise chair") is a qualifier we don't
                # understand, not a second unknown object -- surface it so the
                # human knows what to confirm, without blocking the part we
                # *can* detect.
                leftover = [
                    t for t in unknown_part.split() if self._normalize_text(t) != self._normalize_text(open_match)
                ]
                if leftover:
                    unsupported.append(
                        f'"{" ".join(leftover)}" is not a recognized color or qualifier for the current model.'
                    )
            else:
                unsupported.append(
                    f'"{unknown_part}" is not one of the current model\'s detectable classes.'
                )

        if abstract:
            return {
                "type": "abstract",
                "query": query,
                "colors": colors,
                "objects": registered_objects,
                "text_kind": text_kind,
                "unsupported": abstract,
            }

        return self._finalize_detection_plan(
            query, colors, registered_objects, open_objects, text_kind, unsupported, capable_classes
        )

    def _finalize_detection_plan(
        self,
        query: str,
        colors: list[str],
        registered_objects: list[str],
        open_objects: list[str],
        text_kind: str | None,
        unsupported: list[str],
        capable_classes: dict[str, str],
    ) -> dict[str, Any]:
        # Split what the registry recognizes by name from what the currently
        # configured model can actually detect. A word can be a real, known
        # target (the registry has an entry for it) while still being
        # undetectable today because none of its classes/fallback_classes
        # overlap the model's real class list -- that's a capability gap, not
        # an unknown-word problem, and the two need different human-facing
        # messages.
        capable_objects: list[str] = []
        for name in registered_objects:
            cfg = (self.target_config.get("objects") or {}).get(name, {})
            level = self._object_capability_level(cfg, capable_classes)
            if level == "none":
                unsupported.append(
                    f'Object "{name}" is a known target, but none of its configured classes are in the '
                    "current model's detectable class list."
                )
            else:
                capable_objects.append(name)

        yolo_classes: set[str] = set()
        fallback_classes: set[str] = set()
        fallback_requires_human = False
        for object_name in capable_objects:
            cfg = (self.target_config.get("objects") or {}).get(object_name, {})
            primary = {self._normalize_text(c): c for c in cfg.get("classes") or []}
            fallback = {self._normalize_text(c): c for c in cfg.get("fallback_classes") or []}
            direct_hits = [c for key, c in primary.items() if key in capable_classes and key not in fallback]
            fallback_hits = [c for key, c in fallback.items() if key in capable_classes]
            if direct_hits:
                yolo_classes.update(self._normalize_token(item) for item in direct_hits)
            else:
                fallback_classes.update(self._normalize_token(item) for item in fallback_hits)
                fallback_requires_human = fallback_requires_human or bool(cfg.get("fallback_requires_human"))
        for object_name in open_objects:
            yolo_classes.add(self._normalize_token(object_name))

        all_objects = [*capable_objects, *open_objects]
        if text_kind and (colors or all_objects):
            plan_type = "composite"
        elif colors and all_objects:
            plan_type = "composite"
        elif text_kind:
            plan_type = "text"
        elif colors:
            plan_type = "color"
        elif all_objects:
            plan_type = "object"
        else:
            if not unsupported:
                # Only add the generic message when nothing more specific was
                # already recorded above (registered-but-incapable object,
                # open-vocab miss) -- otherwise this would just repeat the
                # same finding in vaguer words.
                unsupported.append(f'No configured perception target matches "{query}".')
            plan_type = "unsupported"

        return {
            "type": plan_type,
            "query": query,
            "colors": colors,
            "objects": all_objects,
            "configured_objects": capable_objects,
            "open_objects": open_objects,
            "text_kind": text_kind,
            "yolo_classes": sorted(item for item in yolo_classes if item),
            "fallback_classes": sorted(item for item in fallback_classes if item),
            "fallback_requires_human": fallback_requires_human,
            "unsupported": unsupported,
        }

    def _capable_class_map(self) -> dict[str, str]:
        """Normalized-key -> canonical-name map of what the configured model can
        actually detect (detector_classes, exposed internally through the legacy
        yolo_open_vocab_classes key; checked against model.names on load). This is
        the single source of truth for "can we detect this at all"; nothing
        else in this file should guess at plausibility.
        """
        return {self._normalize_text(name): name for name in (self.target_config.get("yolo_open_vocab_classes") or [])}

    @staticmethod
    def _object_capability_level(cfg: dict[str, Any], capable_classes: dict[str, str]) -> str:
        primary = {PerceptionSkills._normalize_text(c) for c in cfg.get("classes") or []}
        if primary & capable_classes.keys():
            return "direct"
        fallback = {PerceptionSkills._normalize_text(c) for c in cfg.get("fallback_classes") or []}
        if fallback & capable_classes.keys():
            return "fallback"
        return "none"

    @staticmethod
    def _open_vocab_match(unknown_part: str, capable_classes: dict[str, str]) -> str | None:
        """Try the whole remaining phrase compacted (for multi-word real classes
        like "dining table"), then each of its words independently, so a known
        object isn't missed just because it sits next to a word that didn't
        resolve to anything -- e.g. an unregistered color like "turquoise
        chair" must still find "chair". This is the symmetric counterpart of
        the color-known/object-unknown case: object-known/color-unknown (or
        object-known/text-unknown) must attempt detection too.
        """
        whole = PerceptionSkills._normalize_text(unknown_part)
        if whole and len(whole) >= 3 and whole in capable_classes:
            return capable_classes[whole]
        for token in re.split(r"[\s_\-]+", unknown_part.strip()):
            candidate = PerceptionSkills._normalize_text(token)
            if candidate and len(candidate) >= 3 and candidate in capable_classes:
                return capable_classes[candidate]
        return None

    def _unknown_object_part(self, query: str, colors: list[str]) -> str:
        """The part of the query left after removing matched color words and
        generic filler -- kept as whitespace-separated tokens (not compacted)
        so _open_vocab_match can still test each word on its own, e.g. an
        *unmatched* color word ("turquoise") next to a real object word
        ("chair") must not glue into one unmatchable string.
        """
        color_registry = self.target_config.get("colors") or {}
        matched_aliases = set()
        for color in colors:
            cfg = color_registry.get(color, {})
            for alias in [color] + list(cfg.get("aliases") or []):
                matched_aliases.add(self._normalize_text(alias))
        generic_words = {
            "find",
            "search",
            "detect",
            "lookfor",
            "lookup",
            "look",
            "locate",
            "show",
            "check",
            "thing",
            "object",
            "item",
            "color",
            "colored",
            "探して",
            "探す",
            "検出",
            "見て",
            "もの",
            "物",
            "东西",
            "色",
            "の",
            "い",
            "的",
        }
        tokens = [t for t in re.split(r"[\s_\-]+", query.strip().lower()) if t]
        remaining_tokens = [t for t in tokens if self._normalize_text(t) not in matched_aliases | generic_words]
        return " ".join(remaining_tokens)


    def _unsupported_reasons(self, normalized_query: str) -> list[str]:
        reasons: list[str] = []
        for name, cfg in (self.target_config.get("unsupported_attributes") or {}).items():
            aliases = [name] + list(cfg.get("aliases") or [])
            if any(self._normalize_text(alias) in normalized_query for alias in aliases):
                reasons.append(f'Attribute "{name}" is not reliably verifiable by current perception models.')
        return reasons

    def _abstract_reasons(self, normalized_query: str) -> list[str]:
        reasons: list[str] = []
        for name, cfg in (self.target_config.get("abstract") or {}).items():
            aliases = [name] + list(cfg.get("aliases") or [])
            if any(self._normalize_text(alias) in normalized_query for alias in aliases):
                reasons.append(f'Abstract concept "{name}" requires human or future VLM review.')
        return reasons

    def _match_named_primitives(self, query: str, registry: dict[str, Any]) -> list[str]:
        normalized_query = self._normalize_text(query)
        tokens = {
            self._normalize_text(token)
            for token in re.split(r"[\s,_/|;:\\-]+", str(query).lower())
            if token.strip()
        }
        matches: list[str] = []
        for name, cfg in registry.items():
            aliases = [name] + list(cfg.get("aliases") or [])
            if any(self._alias_matches_query(alias, normalized_query, tokens) for alias in aliases):
                matches.append(str(name))
        return matches

    @staticmethod
    def _alias_matches_query(alias: str, normalized_query: str, tokens: set[str]) -> bool:
        alias_norm = PerceptionSkills._normalize_text(alias)
        if not alias_norm:
            return False
        if alias_norm == normalized_query or alias_norm in tokens:
            return True
        ascii_alias = re.fullmatch(r"[a-z0-9]+", alias_norm) is not None
        if not ascii_alias:
            return alias_norm in normalized_query
        if len(alias_norm) <= 3:
            return normalized_query.endswith(alias_norm) and len(normalized_query) > len(alias_norm)
        return alias_norm in normalized_query

    def _text_kind(self, query: str, normalized_query: str) -> str | None:
        if any(ch.isdigit() for ch in query):
            return "digit"
        if any(self._normalize_text(alias) in normalized_query for alias in ["letter", "alphabet", "文字", "字母", "字幕"]):
            return "letter"
        text_cfg = self.target_config.get("text") or {}
        for name, cfg in text_cfg.items():
            aliases = [name] + list(cfg.get("aliases") or [])
            if any(self._normalize_text(alias) in normalized_query for alias in aliases):
                return str(name)
        if len(query.strip()) == 1 and query.strip().isascii() and query.strip().isalpha():
            return "letter"
        return None

    def _matching_query_from_plan(self, query: str, plan: dict[str, Any]) -> str:
        aliases = {str(plan.get("normalized_query") or query).lower().strip(), query.lower().strip()}
        if plan.get("target_text"):
            aliases.add(str(plan["target_text"]))
        for color in plan.get("colors", []):
            aliases.add(f"color_region_{color}")
            aliases.add(str(color))
        for object_name in plan.get("objects", []):
            cfg = (self.target_config.get("objects") or {}).get(object_name, {})
            aliases.add(str(object_name))
            aliases.update(str(item) for item in cfg.get("classes") or [])
            aliases.update(str(item) for item in cfg.get("fallback_classes") or [])
            aliases.update(str(item) for item in cfg.get("aliases") or [])
        if plan.get("text_kind"):
            aliases.add("ocr_text")
        aliases.update(self._query_aliases(self._normalize_text(query)))
        return " ".join(sorted(self._normalize_token(alias) for alias in aliases if alias))

    @staticmethod
    def _normalize_text(text: str) -> str:
        return str(text).strip().lower().replace(" ", "").replace("_", "").replace("-", "")

    @staticmethod
    def _normalize_token(text: str) -> str:
        return str(text).strip().lower().replace(" ", "_").replace("-", "_")

    @staticmethod
    def _matches_query(query: str, label: str, text: str | None, confidence: float, threshold: float) -> bool:
        if confidence < threshold:
            return False
        if PerceptionSkills._required_digit_tokens(query):
            return bool(PerceptionSkills._matched_digit_tokens(query, text, confidence, threshold))
        normalized_query = query.lower().replace(" ", "")
        normalized_label = label.lower().replace(" ", "")
        query_digits = "".join(ch for ch in normalized_query if ch.isdigit())
        if text:
            normalized_text = "".join(ch for ch in text.lower() if ch.isalnum())
            text_digits = "".join(ch for ch in normalized_text if ch.isdigit())
            text_candidates = {normalized_text, normalized_text.lstrip("0")}
            digit_runs = []
            current = ""
            for ch in normalized_text:
                if ch.isdigit():
                    current += ch
                elif current:
                    digit_runs.append(current)
                    current = ""
            if current:
                digit_runs.append(current)
            if query_digits:
                return any(run == query_digits for run in digit_runs)
            for run in digit_runs:
                text_candidates.add(run)
                text_candidates.add(run.lstrip("0"))
                if len(run) >= 2:
                    text_candidates.add(run[-2:])
                    text_candidates.add(run[-2:].lstrip("0"))
            if any(token and token in normalized_query for token in text_candidates):
                return True
            if "06" in normalized_query and any(token in {"6", "06"} for token in text_candidates):
                return True
        if query_digits:
            return False
        query_parts = {
            PerceptionSkills._normalize_text(part)
            for part in re.split(r"[\s,;/|]+", query.lower())
            if part.strip()
        }
        aliases = query_parts if len(query_parts) > 1 else PerceptionSkills._query_aliases(normalized_query)
        normalized_aliases = {PerceptionSkills._normalize_text(alias) for alias in aliases if alias}
        return normalized_label in normalized_aliases

    @staticmethod
    def _query_aliases(normalized_query: str) -> set[str]:
        aliases = {normalized_query}
        if any(token in normalized_query for token in ["車", "自動車", "vehicle", "car", "auto"]):
            aliases.update({"car", "truck", "bus", "vehicle"})
        if any(token in normalized_query for token in ["箱", "box", "carton", "package"]):
            aliases.update({"box", "package", "carton", "red_box_candidate"})
        if any(token in normalized_query for token in ["赤い箱", "赤箱", "redbox", "red_box"]):
            aliases.update({"red_box_candidate", "box"})
        if any(token in normalized_query for token in ["桶", "バケツ", "bucket", "pail", "barrel", "container"]):
            aliases.update({"bucket", "pail", "barrel", "container", "bottle", "cup", "bowl"})
        if any(token in normalized_query for token in ["フォークリフト", "forklift"]):
            aliases.update({"forklift", "truck"})
        if any(token in normalized_query for token in ["人", "person", "worker"]):
            aliases.update({"person"})
        if any(token in normalized_query for token in ["パレット", "pallet"]):
            aliases.update({"pallet"})
        return aliases

    @staticmethod
    def _normalized_detection_query(query: str) -> str:
        normalized = query.lower().replace(" ", "")
        aliases = PerceptionSkills._query_aliases(normalized)
        if normalized != query.lower().strip():
            aliases.add(query.lower().strip())
        return " ".join(sorted(alias for alias in aliases if alias))

    @staticmethod
    def _required_digit_tokens(query: str) -> list[str]:
        import re

        tokens: list[str] = []
        for match in re.findall(r"\d+", query):
            tokens.append(match)
        seen = set()
        out = []
        for token in tokens:
            if token not in seen:
                seen.add(token)
                out.append(token)
        return out

    @staticmethod
    def _matched_digit_tokens(query: str, text: str | None, confidence: float, threshold: float) -> list[str]:
        if confidence < threshold or not text:
            return []
        query_tokens = PerceptionSkills._required_digit_tokens(query)
        if not query_tokens:
            return []
        digit_runs = re.findall(r"\d+", str(text))
        shelf_label = bool(re.search(r"shelf[_\s-]*label[_\s-]*\d+", query, re.IGNORECASE))
        matched = []
        for token in query_tokens:
            if any(run == token or (shelf_label and int(run) == int(token)) for run in digit_runs):
                matched.append(token)
        return matched

    @staticmethod
    def _load_config(filename: str) -> dict[str, Any]:
        try:
            path = os.path.join(get_package_share_directory("robot_skills"), "config", filename)
        except PackageNotFoundError:
            path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config", filename))
        if not os.path.exists(path):
            return {}
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
