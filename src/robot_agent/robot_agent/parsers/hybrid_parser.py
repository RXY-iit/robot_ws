from __future__ import annotations

import os
import re
from typing import Any

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from robot_agent.parsers.deterministic_parser import DeterministicParser
from robot_agent.parsers.embedding_parser import EmbeddingParser
from robot_agent.parsers.llama_cpp_parser import LlamaCppParser
from robot_agent.schemas import MissionIntent


class HybridParser:
    def __init__(self, config_dir: str | None = None):
        self.config_dir = config_dir or self._default_config_dir()
        self.model_policy = self._load_yaml("model_policy.yaml")
        self.parser_templates = self._load_yaml("parser_templates.yaml")
        self.deterministic = DeterministicParser(known_targets=self._load_known_targets())
        providers = self.model_policy.get("providers", {})
        embedding_cfg = providers.get("embedding_parser", {})
        llama_cfg = providers.get("llama_cpp_parser", {})
        self.embedding = EmbeddingParser(
            templates=self.parser_templates.get("templates", []),
            model_name=str(embedding_cfg.get("model", "intfloat/multilingual-e5-small")),
            accept_threshold=float(embedding_cfg.get("accept_threshold", 0.86)),
            clarify_threshold=float(embedding_cfg.get("clarify_threshold", 0.72)),
        )
        self.llama = LlamaCppParser(
            endpoint=str(llama_cfg.get("endpoint", "http://127.0.0.1:8080/v1/chat/completions")),
            model=str(llama_cfg.get("model", "local-qwen-parser")),
            timeout_sec=float(llama_cfg.get("timeout_sec", 2.0)),
            command_reference=self._build_command_reference(self.parser_templates.get("templates", [])),
        )
        self.last_metrics: dict[str, Any] = {}

    def parse(self, text: str) -> MissionIntent:
        route_steps: list[dict[str, Any]] = []
        deterministic = self.deterministic.parse(text)
        if deterministic.task_type != "ask_human":
            deterministic, slot_metrics = self._postprocess_slots(deterministic, text)
            route_steps.append(
                {
                    "layer": "Deterministic",
                    "status": "accepted",
                    "task_type": deterministic.task_type,
                    **slot_metrics,
                }
            )
            self.last_metrics = {
                "source": "deterministic",
                "accepted_layer": "Deterministic",
                "route_steps": route_steps,
                **slot_metrics,
            }
            return deterministic

        route_steps.append(
            {
                "layer": "Deterministic",
                "status": "rejected",
                "reason": "no_rule_match",
            }
        )
        ambiguity = self._detect_command_ambiguity(text)
        if ambiguity:
            intent = MissionIntent(
                task_type="ask_human",
                raw_text=text,
                requires_confirmation=True,
                constraints={"ambiguity": ambiguity},
            )
            route_steps.append(
                {
                    "layer": "Ambiguity Gate",
                    "status": "needs_human",
                    "reason": ambiguity.get("reason"),
                    "choices": ambiguity.get("choices", []),
                }
            )
            self.last_metrics = {
                "source": "ambiguity_gate",
                "accepted_layer": None,
                "route_steps": route_steps,
                "ambiguity": ambiguity,
            }
            return intent
        embedded, metrics = self.embedding.parse(text)
        embedded, slot_metrics = self._postprocess_slots(embedded, text)
        embedding_accepts = embedded.task_type != "ask_human" and not slot_metrics.get("needs_fallback")
        route_steps.append(
            {
                "layer": "Embedding",
                "status": "accepted" if embedding_accepts else "rejected",
                "task_type": embedded.task_type,
                "score": metrics.get("score"),
                "template_id": metrics.get("template_id"),
                "engine": metrics.get("engine"),
                "model": metrics.get("model"),
                "needs_fallback": metrics.get("needs_fallback", False),
                **slot_metrics,
            }
        )
        self.last_metrics = {
            **metrics,
            **slot_metrics,
            "accepted_layer": "Embedding" if embedding_accepts else None,
            "route_steps": route_steps,
        }
        if embedding_accepts:
            return embedded

        # Neither Deterministic nor Embedding may dead-end the pipeline at
        # ask_human on their own (2026-07-13 design change): whatever the
        # reason Embedding didn't accept -- a genuine tie (ambiguous), a score
        # that only cleared clarify_threshold, or a plain low/no match -- the
        # command always continues on to the LLM layer, which is now the sole
        # place that produces the final ask_human result (always as top-3
        # candidates + a cancel choice, never a bare unexplained ask_human).
        candidates, llm_error = self.llama.parse_candidates(text)
        choices = self._build_llm_choices(candidates)
        ambiguity = {
            "reason": "llm_unreachable" if llm_error else "llm_fallback_candidates",
            "choices": choices,
            "embedding_top_matches": metrics.get("top_matches", []),
        }
        route_steps.append(
            {
                "layer": "LLM Fallback",
                "status": "needs_human",
                "reason": ambiguity["reason"],
                "candidate_count": len(candidates),
                "model": self.llama.model,
                "endpoint": self.llama.endpoint,
                "error": llm_error,
                "choices": choices,
            }
        )
        self.last_metrics = {
            **metrics,
            "fallback": "llama_cpp",
            "llama_cpp_endpoint": self.llama.endpoint,
            "llama_cpp_error": llm_error,
            "accepted_layer": None,
            "route_steps": route_steps,
            "ambiguity": ambiguity,
        }
        return MissionIntent(
            task_type="ask_human",
            raw_text=text,
            requires_confirmation=True,
            constraints={"ambiguity": ambiguity},
        )

    @staticmethod
    def _build_llm_choices(candidates: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Always shaped like the Ambiguity Gate / Embedding-ambiguous choices
        (label/command/intent) so the dry-run matrix and viewer render them
        with the same machinery, no matter which layer produced them. Cancel
        is appended unconditionally -- even if the LLM produced zero usable
        candidates (or the endpoint errored out) -- so an ask_human result
        from this layer always gives the human at least one way out.
        """
        choices: list[dict[str, Any]] = []
        for candidate in candidates[:3]:
            task_type = candidate.get("task_type") or "ask_human"
            intent: dict[str, Any] = {"task_type": task_type}
            if candidate.get("target"):
                intent["target"] = candidate["target"]
            if candidate.get("object_query"):
                intent["object_query"] = candidate["object_query"]
            if candidate.get("constraints"):
                intent["constraints"] = candidate["constraints"]
            label = candidate.get("label") or task_type
            choices.append({"label": label, "command": label, "intent": intent})
        choices.append({"label": "キャンセル / Cancel", "command": "cancel", "intent": {"task_type": "cancel"}})
        return choices

    @staticmethod
    def _build_command_reference(templates: list[dict[str, Any]]) -> str:
        """Compact, always-in-sync summary of the Deterministic/Embedding
        command vocabulary (built straight from parser_templates.yaml), fed
        into the LLM Fallback prompt so it grounds its guesses in commands the
        rest of the system actually understands, instead of inventing shapes.
        """
        slot_hint = {
            "navigate_to_object": "target=<object name>",
            "navigate_to_place": "target=<place name>",
            "search_object_near_place": "target=<shelf/place>, object_query=<raw phrase>",
            "search_object_here": "object_query=<raw phrase>",
            "inspect_place": "target=<place name>",
            "move_relative": "constraints={direction, distance_m}",
            "move_relative_xy": "constraints={dx_m, dy_m}",
            "rotate_relative": "constraints={yaw_deg}",
            "look_direction": "constraints={direction, band}",
            "move_lift_bounded": "constraints={direction, distance_m}",
        }
        lines: list[str] = []
        for item in templates:
            task_type = str((item.get("intent") or {}).get("task_type") or "")
            if not task_type:
                continue
            examples = [str(example) for example in (item.get("examples") or [])][:2]
            hint = slot_hint.get(task_type)
            prefix = f"- {task_type} ({hint})" if hint else f"- {task_type}"
            if examples:
                example_text = "; ".join(f'"{example}"' for example in examples)
                lines.append(f"{prefix}: e.g. {example_text}")
            else:
                lines.append(prefix)
        return "\n".join(lines)

    @staticmethod
    def _postprocess_slots(intent: MissionIntent, raw_text: str) -> tuple[MissionIntent, dict[str, Any]]:
        metrics: dict[str, Any] = {}
        target_tasks = {"navigate_to_object", "navigate_to_place", "inspect_place", "search_object_near_place"}
        if intent.task_type in target_tasks:
            original = intent.target
            cleaned = HybridParser._extract_clean_target(original or raw_text)
            if cleaned:
                intent.target = cleaned
                if cleaned != original:
                    metrics["target_normalized"] = True
                    metrics["original_target"] = original
                    metrics["normalized_target"] = cleaned
            elif not intent.target:
                metrics["needs_fallback"] = True
                metrics["slot_error"] = "missing_target"
            elif HybridParser._target_looks_dirty(intent.target):
                metrics["needs_fallback"] = True
                metrics["slot_error"] = "dirty_target"
        if intent.task_type == "search_object_near_place" and not intent.object_query:
            query = HybridParser._extract_search_query(raw_text, intent.target)
            if query:
                intent.object_query = query
                metrics["object_query_normalized"] = True
            else:
                metrics["needs_fallback"] = True
                metrics["slot_error"] = "missing_object_query"
        if intent.task_type == "search_object_here" and not intent.object_query:
            metrics["needs_fallback"] = True
            metrics["slot_error"] = "missing_object_query"
        if intent.task_type in {"search_object_here", "search_object_near_place"} and intent.object_query:
            normalized_query = HybridParser._normalize_perception_query(intent.object_query)
            if normalized_query:
                if normalized_query != intent.object_query:
                    metrics["object_query_normalized"] = True
                    metrics["original_object_query"] = intent.object_query
                    metrics["normalized_object_query"] = normalized_query
                intent.object_query = normalized_query
            else:
                metrics["needs_fallback"] = True
                metrics["slot_error"] = "ambiguous_object_query"
        return intent, metrics

    @staticmethod
    def _normalize_perception_query(query: str | None) -> str | None:
        """Only guards the genuinely ambiguous case -- a bare single digit or
        single character (those get a real disambiguation prompt from
        _detect_command_ambiguity instead of silently becoming an object
        query) -- plus empty strings. The actual
        color/object/text decomposition is PerceptionSkills' job; duplicating
        it here as an allowlist is exactly what caused "red_box"/"object_car"
        to be rejected before just because a regex here didn't recognize the
        canonical form (see the deterministic-parser fix for the same bug).
        """
        if not query:
            return None
        raw = str(query).strip(" ，,。.?？")
        if not raw:
            return None
        if re.fullmatch(r"0?[1-6]", raw) or re.fullmatch(r"[A-Za-z0-9]", raw):
            return None
        return raw

    @staticmethod
    def _detect_command_ambiguity(text: str) -> dict[str, Any] | None:
        raw = text.strip()
        lowered = raw.lower()
        if re.fullmatch(r"(?:check|find|search)\s+0?[1-6]", lowered) or re.fullmatch(r"0?[1-6](?:を)?(?:確認|見て|探して)?", raw):
            digit = int(re.search(r"\d+", raw).group(0))
            return {
                "reason": "bare_shelf_digit_query",
                "choices": [
                    {
                        "label": f"この場で棚ラベル {digit:02d} を確認",
                        "command": f"check shelf_label_{digit:02d}",
                        "intent": {
                            "task_type": "search_object_here",
                            "object_query": f"shelf_label_{digit:02d}",
                        },
                    },
                    {
                        "label": f"{digit:02d}棚へ移動して確認",
                        "command": f"{digit:02d}棚を確認して",
                        "intent": {
                            "task_type": "inspect_place",
                            "target": f"{digit:02d}棚",
                        },
                    },
                    {
                        "label": "キャンセル",
                        "command": "cancel",
                        "intent": {"task_type": "cancel"},
                    },
                ],
            }
        if re.fullmatch(r"(?:check|find|search)\s+[A-Za-z0-9]", lowered):
            token = raw.split()[-1]
            return {
                "reason": "bare_single_character_query",
                "choices": [
                    {
                        "label": f"この場でテキスト {token} を確認",
                        "command": f"check text {token}",
                        "intent": {
                            "task_type": "search_object_here",
                            "object_query": f"text_{token}",
                        },
                    },
                    {
                        "label": "キャンセル",
                        "command": "cancel",
                        "intent": {"task_type": "cancel"},
                    },
                ],
            }
        return None

    @staticmethod
    def _extract_clean_target(text: str | None) -> str | None:
        if not text:
            return None
        raw = text.strip()
        lowered = raw.lower()
        region_match = re.search(
            r"(?<!\d)(0?[1-9])\s*[-_ー〜~]\s*(0?[1-9])\s*(?:棚|番棚|shelf)?",
            raw,
            re.IGNORECASE,
        )
        if region_match:
            left = int(region_match.group(1))
            right = int(region_match.group(2))
            return f"{left:02d}-{right:02d}棚"
        shelf_after_match = re.search(r"(?:shelf|棚)\s*[_\s-]*(0?[1-9])(?=\D|$)", lowered, re.IGNORECASE)
        if shelf_after_match:
            return f"{int(shelf_after_match.group(1)):02d}棚"
        shelf_before_match = re.search(r"(?<!\d)(0?[1-9])\s*(?:番棚|棚|shelf)", raw, re.IGNORECASE)
        if shelf_before_match:
            return f"{int(shelf_before_match.group(1)):02d}棚"
        for alias in [
            ("home", "home"),
            ("ホーム", "ホーム"),
            ("entrance", "entrance"),
            ("入口", "入口"),
            ("corridor", "corridor"),
            ("廊下", "廊下"),
            ("forklift", "forklift"),
            ("elevator", "elevator"),
        ]:
            if alias[0] in lowered or alias[0] in raw:
                return alias[1]
        cleaned = re.sub(
            r"^(move to|go to|inspect|check|find|search|look for)\s+",
            "",
            raw,
            flags=re.IGNORECASE,
        )
        cleaned = re.sub(r"(?:に|へ)?\s*(?:行って|行く|移動して|移動|確認して|確認|見て).*$", "", cleaned)
        cleaned = cleaned.strip(" ，,。.?？")
        return cleaned or None

    @staticmethod
    def _target_looks_dirty(target: str) -> bool:
        lowered = target.lower()
        dirty_tokens = [
            "行って",
            "移動",
            "確認",
            "見て",
            "go to",
            "move to",
            "inspect",
            "check",
        ]
        return any(token in lowered or token in target for token in dirty_tokens)

    @staticmethod
    def _extract_search_query(raw_text: str, target: str | None) -> str | None:
        text = raw_text
        if target:
            text = text.replace(target, "")
        text = re.sub(r"(?:.+?)(?:に|で)", "", text)
        text = re.sub(r"(?:がありますか|あります|ある|見える|を探して|をさがして|探して|find|search|look for|check)", "", text, flags=re.IGNORECASE)
        text = re.sub(r"(?:行って|行く|移動して|移動)", "", text)
        text = text.strip(" ，,。.?？")
        return text or None

    @staticmethod
    def _load_known_targets() -> frozenset[str]:
        # The map bundle is fixed (via ROBOT_MAP_ID) before parsing ever starts, so
        # it's safe for the deterministic parser to use it as a disambiguation
        # priority signal. Degrade to an empty set if no map is available (e.g. a
        # standalone unit test) rather than fail parsing.
        try:
            from robot_skills.semantic_map import SemanticMap

            semantic_map = SemanticMap()
        except Exception:
            return frozenset()
        names: set[str] = set()
        for item in semantic_map.list_known_targets():
            for alias in [item.get("id", ""), *(item.get("names") or [])]:
                if alias:
                    names.add(DeterministicParser._normalize_light(str(alias)))
        return frozenset(names)

    def _load_yaml(self, filename: str) -> dict[str, Any]:
        path = os.path.join(self.config_dir, filename)
        if not os.path.exists(path):
            return {}
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    @staticmethod
    def _default_config_dir() -> str:
        try:
            return os.path.join(get_package_share_directory("robot_agent"), "config")
        except PackageNotFoundError:
            return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config"))
