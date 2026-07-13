from __future__ import annotations

from robot_agent.schemas import MissionIntent, SkillCall


class PerceptionAgent:
    def __init__(self, scan_profile: str = "shelf_label"):
        self.scan_profile = scan_profile

    def build_plan(self, intent: MissionIntent) -> list[SkillCall]:
        if intent.task_type == "look_direction":
            return self.build_look_direction_plan(
                direction=str(intent.constraints.get("direction", "front")),
                band=str(intent.constraints.get("band", "middle")),
                object_query=intent.object_query,
            )

        if intent.task_type == "search_object_near_place":
            return self.build_search_plan(intent.object_query)

        if intent.task_type == "inspect_place":
            return self.build_search_plan(self._arrival_confirmation_query(intent.target))

        plan = [
            SkillCall(
                name="capture_observation",
                payload={},
                reason="Capture current scene observation for mission verification.",
                timeout_sec=3.0,
            )
        ]
        if intent.object_query:
            plan.append(
                SkillCall(
                    name="detect_known_object",
                    payload={"query": intent.object_query, "min_confidence": 0.5, "provider": "yolo11s_ocr"},
                    reason="Run non-contact known-object detection or stub.",
                    timeout_sec=5.0,
                )
            )
        return plan

    def build_look_direction_plan(
        self,
        direction: str = "front",
        band: str = "middle",
        object_query: str | None = None,
    ) -> list[SkillCall]:
        pan_deg, tilt_deg, scan_band = self._direction_camera_pose(direction, band)
        plan = [
            SkillCall(
                name="set_camera_pose",
                payload={"pan_deg": pan_deg, "tilt_deg": tilt_deg, "scan_band": scan_band},
                reason="Move camera holder to the requested view direction.",
                timeout_sec=2.0,
            ),
            SkillCall(
                name="capture_observation",
                payload={"pan_deg": pan_deg, "tilt_deg": tilt_deg, "scan_band": scan_band},
                reason="Capture image from requested view direction.",
                timeout_sec=3.0,
            ),
        ]
        if object_query:
            plan.append(
                SkillCall(
                    name="detect_known_object",
                    payload={
                        "query": object_query,
                        "min_confidence": 0.5,
                        "provider": self._provider_for_query(object_query),
                    },
                    reason="Run detector on directional observation.",
                    timeout_sec=8.0,
                )
            )
        plan.append(
            SkillCall(
                name="set_camera_pose",
                payload={"pan_deg": 0.0, "tilt_deg": 67.0, "scan_band": "return_home"},
                reason="Return camera holder to the navigation-mode neutral pose after observation.",
                timeout_sec=2.0,
            )
        )
        return plan

    def build_search_plan(self, object_query: str | None) -> list[SkillCall]:
        query = object_query or "requested object"
        if self._is_fully_unknown_query(query):
            # Nothing about this query is detectable (no known color, object,
            # or text signal at all) -- no amount of camera motion changes
            # that, so skip the 9-pose scan entirely and go straight to a
            # single detect_known_object call, which short-circuits to
            # NEEDS_HUMAN immediately without needing any captured image.
            return [
                SkillCall(
                    name="detect_known_object",
                    payload={"query": query, "min_confidence": 0.5, "provider": "yolo11s_ocr"},
                    reason="Query has no recognizable color/object/text signal; report needs_human without moving the camera.",
                    timeout_sec=2.0,
                )
            ]
        provider = self._provider_for_query(query)
        plan: list[SkillCall] = []
        for pan_deg, tilt_deg, scan_band in [
            (-35.0, 80.0, "ground_left"),
            (0.0, 80.0, "ground_center"),
            (35.0, 80.0, "ground_right"),
            (-35.0, 25.0, "middle_left"),
            (0.0, 25.0, "middle_center"),
            (35.0, 25.0, "middle_right"),
            (-35.0, -20.0, "upper_left"),
            (0.0, -20.0, "upper_center"),
            (35.0, -20.0, "upper_right"),
        ]:
            plan.append(
                SkillCall(
                    name="set_camera_pose",
                    payload={"pan_deg": pan_deg, "tilt_deg": tilt_deg, "scan_band": scan_band},
                    reason="Move camera holder through a bounded search scan.",
                    timeout_sec=2.0,
                )
            )
            plan.append(
                SkillCall(
                    name="capture_observation",
                    payload={"pan_deg": pan_deg, "tilt_deg": tilt_deg, "scan_band": scan_band},
                    reason="Capture image after camera holder reaches scan pose.",
                    timeout_sec=3.0,
                )
            )
        plan.append(
            SkillCall(
                name="set_camera_pose",
                payload={"pan_deg": 0.0, "tilt_deg": 67.0, "scan_band": "return_home"},
                reason="Return camera holder to the navigation-mode neutral pose after scanning.",
                timeout_sec=2.0,
            )
        )
        plan.append(
            SkillCall(
                name="detect_known_object",
                payload={"query": query, "min_confidence": 0.5, "provider": provider},
                reason="Run object detector over captured scan observations.",
                timeout_sec=8.0,
            )
        )
        return plan

    @staticmethod
    def _is_fully_unknown_query(query: str) -> bool:
        """True only when the query has zero recognizable signal at all (no
        known color, no known/open-vocab object, no text/OCR kind) -- never
        for a partially-known query (e.g. a known color with an unrecognized
        object still gets the camera moved, since the color half is worth
        attempting; only the object half needs a human).
        """
        try:
            from robot_skills.executors.perception import PerceptionSkills
        except Exception:
            return False
        try:
            plan = PerceptionSkills(node=None)._build_detection_plan(query)  # noqa: SLF001 - planning-time capability check
        except Exception:
            return False
        return plan.get("type") in {"abstract", "unsupported"}

    def build_arrival_confirmation_plan(self, target_name: str | None) -> list[SkillCall]:
        query = self._arrival_confirmation_query(target_name)
        return self.build_search_plan(query)

    @staticmethod
    def _arrival_confirmation_query(target_name: str | None) -> str:
        if not target_name:
            return "target label"
        normalized = target_name.lower().replace(" ", "")
        digit_tokens = PerceptionAgent._digit_tokens(target_name)
        if len(digit_tokens) >= 2:
            return " ".join(f"shelf_label_{token}" for token in digit_tokens)
        if any(token in normalized for token in ["06", "6号", "shelf06", "shelf_06"]):
            return "shelf_label_06"
        if any(token in normalized for token in ["01", "1号", "shelf01", "shelf_01"]):
            return "shelf_label_01"
        return target_name

    @staticmethod
    def _digit_tokens(text: str) -> list[str]:
        import re

        tokens: list[str] = []
        for match in re.findall(r"\d+", text):
            if len(match) >= 2:
                tokens.append(match[-2:])
            elif len(match) == 1:
                tokens.append(f"0{match}")
        seen = set()
        out = []
        for token in tokens:
            if token not in seen:
                seen.add(token)
                out.append(token)
        return out

    @staticmethod
    def _provider_for_query(query: str) -> str:
        normalized = query.lower().replace(" ", "")
        if normalized.startswith("shelf_label_") or normalized.startswith("text_") or any(
            token in normalized
            for token in ["番号", "label", "sign", "letter", "alphabet", "text", "文字", "字幕", "字母"]
        ):
            return "yolo11s_ocr"
        if any(token in normalized for token in ["赤い箱", "redbox", "red_box"]):
            return "yolo11s_ocr"
        return "yolo11s"

    @staticmethod
    def _direction_camera_pose(direction: str, band: str) -> tuple[float, float, str]:
        normalized_direction = direction.lower().replace("-", "_").replace(" ", "_")
        normalized_band = band.lower().replace("-", "_").replace(" ", "_")
        pan_by_direction = {
            "left": -35.0,
            "front_left": -35.0,
            "front": 0.0,
            "center": 0.0,
            "right": 35.0,
            "front_right": 35.0,
        }
        tilt_by_band = {
            "ground": 80.0,
            "down": 80.0,
            "middle": 25.0,
            "center": 25.0,
            "upper": -20.0,
            "up": -20.0,
        }
        pan_deg = pan_by_direction.get(normalized_direction, 0.0)
        tilt_deg = tilt_by_band.get(normalized_band, 25.0)
        return pan_deg, tilt_deg, f"{normalized_band}_{normalized_direction}"
