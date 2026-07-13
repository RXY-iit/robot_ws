from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any

from robot_agent.schemas import MissionIntent


class LlamaCppParser:
    def __init__(
        self,
        endpoint: str = "http://127.0.0.1:8080/v1/chat/completions",
        model: str = "local-qwen-parser",
        timeout_sec: float = 2.0,
    ):
        self.endpoint = endpoint
        self.model = model
        self.timeout_sec = timeout_sec
        self.last_error: str | None = None

    def parse(self, text: str) -> MissionIntent:
        self.last_error = None
        prompt = self._prompt(text)
        request = urllib.request.Request(
            self.endpoint,
            data=json.dumps(
                {
                    "model": self.model,
                    "temperature": 0,
                    "response_format": {"type": "json_object"},
                    "messages": [
                        {
                            "role": "system",
                            "content": (
                                "You are a robot command parser. Output one JSON object only. "
                                "Do not apologize. Do not chat. Do not explain. "
                                "Never output ROS topics, coordinates, velocity commands, or prose."
                            ),
                        },
                        {"role": "user", "content": prompt},
                    ],
                },
                ensure_ascii=False,
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                data = json.loads(response.read().decode("utf-8"))
        except (TimeoutError, urllib.error.URLError, json.JSONDecodeError, OSError) as exc:
            self.last_error = repr(exc)
            return MissionIntent(task_type="ask_human", raw_text=text)

        content = self._extract_content(data)
        parsed = self._parse_json_object(content)
        if not parsed:
            self.last_error = "llama_cpp_response_not_json"
            return MissionIntent(task_type="ask_human", raw_text=text)
        return self._intent_from_dict(parsed, text)

    def choose_tolerance_profile(
        self,
        command: str,
        intent: dict[str, Any],
        navigation_metrics: dict[str, Any],
        profiles: dict[str, dict[str, Any]],
    ) -> dict[str, Any] | None:
        self.last_error = None
        request = urllib.request.Request(
            self.endpoint,
            data=json.dumps(
                {
                    "model": self.model,
                    "temperature": 0,
                    "response_format": {"type": "json_object"},
                    "messages": [
                        {
                            "role": "system",
                            "content": (
                                "You select a navigation tolerance policy for a robot. "
                                "Output one JSON object only. Do not chat. Do not authorize motion."
                            ),
                        },
                        {
                            "role": "user",
                            "content": (
                                "Choose exactly one profile from fine, medium, loose. "
                                "Use fine for precise docking/manipulation, medium for ordinary approach, "
                                "loose for visual inspection where nearby arrival is enough. "
                                "Return shape: {\"profile\":\"fine|medium|loose\",\"reason\":\"short\"}.\n"
                                f"Command: {command}\n"
                                f"Intent: {json.dumps(intent, ensure_ascii=False)}\n"
                                f"Navigation metrics: {json.dumps(navigation_metrics, ensure_ascii=False)}\n"
                                f"Profiles: {json.dumps(profiles, ensure_ascii=False)}\n"
                                "JSON:"
                            ),
                        },
                    ],
                },
                ensure_ascii=False,
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                data = json.loads(response.read().decode("utf-8"))
        except (TimeoutError, urllib.error.URLError, json.JSONDecodeError, OSError) as exc:
            self.last_error = repr(exc)
            return None
        parsed = self._parse_json_object(self._extract_content(data))
        if not parsed:
            self.last_error = "llama_cpp_tolerance_response_not_json"
            return None
        profile = str(parsed.get("profile") or "").strip()
        if profile not in {"fine", "medium", "loose"}:
            self.last_error = f"invalid_tolerance_profile:{profile}"
            return None
        return {"profile": profile, "reason": str(parsed.get("reason") or "llm_selected")}

    @staticmethod
    def _prompt(text: str) -> str:
        schema = {
            "task_type": "navigate_to_object | navigate_to_place | search_object_near_place | search_object_here | inspect_place | look_direction | robot_status | list_known_targets | cancel | move_lift_bounded | move_relative | move_relative_xy | rotate_relative | ask_human",
            "target": "natural language target or null",
            "relation": "near | front | null",
            "object_query": "typed perception query such as shelf_label_06, text_BOX, object_box, color_red, red_box, or null",
            "constraints": {"direction": "UP/DOWN or forward/back/left/right", "distance_m": 0.05, "dx_m": 0.0, "dy_m": 0.0, "yaw_deg": 90, "band": "middle"},
            "requires_confirmation": False,
        }
        return (
            "Parse the command into Robot MissionIntent JSON.\n"
            "Allowed task_type values: navigate_to_object, navigate_to_place, "
            "search_object_near_place, search_object_here, inspect_place, robot_status, list_known_targets, "
            "cancel, move_lift_bounded, move_relative, move_relative_xy, rotate_relative, look_direction, ask_human.\n"
            "Rules:\n"
            "- Return JSON only.\n"
            "- Unknown target names must stay as natural-language aliases; semantic resolution happens later.\n"
            "- relation must be exactly one of: near, front, null. Use near for around / nearby.\n"
            "- If the command asks to go/check/find near a place, use search_object_near_place when an object is requested.\n"
            "- If the command asks whether an object is visible here with no place target, use search_object_here.\n"
            "- Perception object_query must be typed when possible: shelf_label_06 for shelf labels, text_BOX for exact OCR text, object_box for YOLO object, color_red for color, red_box for color+object.\n"
            "- Bare commands like check 6 or check A are ambiguous; use ask_human.\n"
            "- For lift commands, prefer distance_m over duration_sec. 10cm means 0.10, 少し means 0.05, 大きく means 0.15.\n"
            "- For relative base motion, use move_relative with direction forward/back/left/right and distance_m.\n"
            "- For diagonal relative motion, use move_relative_xy with dx_m and dy_m in the robot base frame.\n"
            "- For turning in place, use rotate_relative with yaw_deg.\n"
            "- For look left/right/front/up/down commands, use look_direction. It moves only the camera, not the base.\n"
            "- Do not say you cannot access the outside world. You are only parsing text.\n"
            "- If truly ambiguous, use task_type ask_human.\n"
            f"Required shape: {json.dumps(schema, ensure_ascii=False)}\n"
            "Examples:\n"
            '{"task_type":"search_object_near_place","target":"06棚","relation":"near","object_query":"shelf_label_06","constraints":{},"requires_confirmation":false}\n'
            '{"task_type":"search_object_here","target":null,"relation":"near","object_query":"object_car","constraints":{},"requires_confirmation":false}\n'
            '{"task_type":"move_lift_bounded","target":null,"relation":null,"object_query":null,"constraints":{"direction":"UP","distance_m":0.10},"requires_confirmation":false}\n'
            '{"task_type":"move_relative","target":null,"relation":null,"object_query":null,"constraints":{"direction":"forward","distance_m":1.0},"requires_confirmation":false}\n'
            '{"task_type":"move_relative_xy","target":null,"relation":null,"object_query":null,"constraints":{"dx_m":0.2,"dy_m":0.2},"requires_confirmation":false}\n'
            '{"task_type":"rotate_relative","target":null,"relation":null,"object_query":null,"constraints":{"yaw_deg":180},"requires_confirmation":false}\n'
            '{"task_type":"look_direction","target":null,"relation":null,"object_query":null,"constraints":{"direction":"left","band":"middle"},"requires_confirmation":false}\n'
            '{"task_type":"robot_status","target":null,"relation":null,"object_query":null,"constraints":{},"requires_confirmation":false}\n'
            f"Command: {text}\n"
            "JSON:"
        )

    @staticmethod
    def _extract_content(data: dict[str, Any]) -> str:
        choices = data.get("choices") or []
        if not choices:
            return ""
        message = choices[0].get("message") or {}
        return str(message.get("content") or choices[0].get("text") or "")

    @staticmethod
    def _parse_json_object(content: str) -> dict[str, Any] | None:
        stripped = content.strip()
        if not stripped:
            return None
        if stripped.startswith("```"):
            stripped = stripped.strip("`")
            if stripped.startswith("json"):
                stripped = stripped[4:].strip()
        start = stripped.find("{")
        end = stripped.rfind("}")
        if start < 0 or end < start:
            return None
        try:
            data = json.loads(stripped[start : end + 1])
        except json.JSONDecodeError:
            return None
        return data if isinstance(data, dict) else None

    @staticmethod
    def _intent_from_dict(data: dict[str, Any], raw_text: str) -> MissionIntent:
        allowed = {
            "navigate_to_object",
            "navigate_to_place",
            "search_object_near_place",
            "search_object_here",
            "inspect_place",
            "robot_status",
            "list_known_targets",
            "cancel",
            "move_lift_bounded",
            "move_relative",
            "move_relative_xy",
            "rotate_relative",
            "look_direction",
            "ask_human",
        }
        task_type = str(data.get("task_type") or "ask_human")
        if task_type not in allowed:
            task_type = "ask_human"
        constraints = data.get("constraints") if isinstance(data.get("constraints"), dict) else {}
        relation = data.get("relation") or "near"
        if relation not in {"near", "front", None}:
            relation = "near"
        return MissionIntent(
            task_type=task_type,
            target=data.get("target"),
            relation=relation,
            object_query=data.get("object_query"),
            constraints=constraints,
            requires_confirmation=bool(data.get("requires_confirmation", False)),
            raw_text=raw_text,
        )
