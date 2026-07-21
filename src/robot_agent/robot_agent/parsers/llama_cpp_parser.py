from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any


class LlamaCppParser:
    def __init__(
        self,
        endpoint: str = "http://127.0.0.1:8080/v1/chat/completions",
        model: str = "local-qwen-parser",
        timeout_sec: float = 2.0,
        command_reference: str = "",
    ):
        self.endpoint = endpoint
        self.model = model
        self.timeout_sec = timeout_sec
        self.command_reference = command_reference
        self.last_error: str | None = None

    def parse_candidates(self, text: str) -> tuple[list[dict[str, Any]], str | None]:
        """Last-resort layer (2026-07-13 redesign): never returns a single
        confident intent. Always returns a ranked list of up to 3 candidate
        interpretations (each a plain dict: task_type/target/object_query/
        constraints/label) for HybridParser to present as an ask_human choice
        list alongside a cancel option -- or an empty list plus an error
        string if the endpoint/response itself failed, so the caller can still
        degrade to a cancel-only prompt instead of raising.
        """
        self.last_error = None
        prompt = self._candidates_prompt(text)
        body = json.dumps(
            {
                "model": self.model,
                "temperature": 0,
                "response_format": {"type": "json_object"},
                "messages": [
                    {
                        "role": "system",
                        "content": (
                            "You are a robot command parser's last-resort fallback. Output one "
                            "JSON object only. Do not apologize. Do not chat. Do not explain. "
                            "Never output ROS topics, coordinates, velocity commands, or prose."
                        ),
                    },
                    {"role": "user", "content": prompt},
                ],
            },
            ensure_ascii=False,
        ).encode("utf-8")

        # Found 2026-07-13 against a real llama.cpp server: this prompt's huge
        # shared prefix (the command_reference block) with only a short
        # varying "Command: ..." tail occasionally hits a prompt-cache-reuse
        # glitch in llama.cpp -- despite temperature=0, a request can come back
        # as a bare "{}" instead of the real completion. Disabling cache_prompt
        # avoids the glitch but forces a full ~1100-token cold prompt eval on
        # every call (~10x slower, often exceeding timeout_sec on its own), so
        # that trade is worse than the glitch. A single retry on a JSON-shaped-
        # but-empty/invalid response is cheap and reliably recovers (the retry
        # lands on a fresh generation, not the corrupted cache continuation).
        last_exc: str | None = None
        for attempt in range(2):
            request = urllib.request.Request(
                self.endpoint,
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                    data = json.loads(response.read().decode("utf-8"))
            except (TimeoutError, urllib.error.URLError, json.JSONDecodeError, OSError) as exc:
                last_exc = repr(exc)
                continue

            content = self._extract_content(data)
            parsed = self._parse_json_object(content)
            candidates = self._candidates_from_dict(parsed) if parsed else []
            if candidates:
                return candidates, None
            last_exc = "llama_cpp_response_not_json_or_empty"

        self.last_error = last_exc
        return [], self.last_error

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
                    "cache_prompt": False,
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

    def _candidates_prompt(self, text: str) -> str:
        schema = {
            "candidates": [
                {
                    "task_type": "one of the allowed task_types below",
                    "target": "natural language place/object name, or null",
                    "object_query": "raw perception phrase (never pre-typed like red_box), or null",
                    "constraints": {"direction": "...", "distance_m": 0.0, "dx_m": 0.0, "dy_m": 0.0, "yaw_deg": 0, "band": "middle"},
                    "label": "short human-readable description of this guess, for a person to pick from",
                }
            ]
        }
        reference = self.command_reference or "(no command reference available)"
        return (
            "A regex-based deterministic parser and a semantic-similarity matcher both already "
            "failed to confidently understand the command below. You are the last-resort fallback. "
            "A human will be shown your candidates and will pick one (or cancel) next -- so you must "
            "always propose your best guesses, even under uncertainty. Never refuse, never ask a "
            "clarifying question back, never explain yourself outside the JSON.\n\n"
            "Allowed task_type values: navigate_to_object, navigate_to_place, search_object_near_place, "
            "search_object_here, inspect_place, robot_status, list_known_targets, cancel, "
            "move_lift_bounded, move_relative, move_relative_xy, rotate_relative, look_direction.\n\n"
            f"The robot's actual supported command vocabulary (grounded reference, do not invent "
            f"commands outside this set):\n{reference}\n\n"
            "Rules:\n"
            "- Return JSON only, shaped exactly like the schema below.\n"
            "- \"candidates\" is a list of at most 3 items, ranked most-likely first.\n"
            "- Do not include task_type ask_human or cancel as a candidate -- cancel is always offered "
            "separately by the system, and refusing to guess defeats the point of this fallback.\n"
            "- object_query must be the raw phrase taken from the command (e.g. \"red box\", \"赤い箱\"), "
            "never a pre-typed token like red_box or object_box -- a separate perception module "
            "classifies the raw phrase later.\n"
            "- target is a natural-language place/object name; do not resolve it to a map ID.\n"
            "- label is a short sentence describing what this candidate would actually do.\n"
            "- If you can only think of one plausible interpretation, return just one candidate; do "
            "not pad the list with unrelated guesses.\n"
            "- For lift commands, prefer distance_m over duration_sec. 10cm means 0.10, 少し/a little "
            "means 0.05, 大きく/a lot means 0.15.\n"
            "- For relative base motion, use move_relative with direction forward/back/left/right and "
            "distance_m; for diagonal motion use move_relative_xy with dx_m/dy_m; for turning in place "
            "use rotate_relative with yaw_deg; for look left/right/front/up/down (camera only, not the "
            "base) use look_direction with direction and band.\n"
            "- rotate_relative sign convention: yaw_deg is POSITIVE for a LEFT/counter-clockwise turn "
            "and NEGATIVE for a RIGHT/clockwise turn (e.g. \"turn right 90\" -> yaw_deg: -90, \"turn "
            "left 45\" -> yaw_deg: 45). Do not include a separate \"direction\" field for "
            "rotate_relative -- the sign of yaw_deg already encodes it.\n"
            f"Required shape: {json.dumps(schema, ensure_ascii=False)}\n"
            "Examples:\n"
            '{"candidates":[{"task_type":"robot_status","target":null,"object_query":null,"constraints":{},"label":"Report the robot\'s current status"},{"task_type":"list_known_targets","target":null,"object_query":null,"constraints":{},"label":"List the places the robot knows"}]}\n'
            '{"candidates":[{"task_type":"navigate_to_place","target":"home","object_query":null,"constraints":{},"label":"Return to home"}]}\n'
            '{"candidates":[{"task_type":"search_object_here","target":null,"object_query":"red box","constraints":{},"label":"Look for a red box in the current view"},{"task_type":"inspect_place","target":"front door","object_query":null,"constraints":{},"label":"Go inspect the front door"}]}\n'
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
    def _candidates_from_dict(data: dict[str, Any]) -> list[dict[str, Any]]:
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
        }
        raw_candidates = data.get("candidates")
        if not isinstance(raw_candidates, list):
            return []
        candidates: list[dict[str, Any]] = []
        for item in raw_candidates[:3]:
            if not isinstance(item, dict):
                continue
            task_type = str(item.get("task_type") or "").strip()
            if task_type not in allowed:
                continue
            constraints = item.get("constraints") if isinstance(item.get("constraints"), dict) else {}
            label = str(item.get("label") or "").strip() or task_type
            candidates.append(
                {
                    "task_type": task_type,
                    "target": item.get("target") or None,
                    "object_query": item.get("object_query") or None,
                    "constraints": constraints,
                    "label": label,
                }
            )
        return candidates
