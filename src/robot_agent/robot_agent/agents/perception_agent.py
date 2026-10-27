from __future__ import annotations

from dataclasses import replace
import re
import yaml

from robot_agent.schemas import MissionIntent, SkillCall
from robot_agent.agents.domain import TaskRefinement, review
from robot_agent.task_catalog import TaskCatalog
from robot_agent.agents.perception_plan import target_config, parse_seed, questions_for, compile_checks
from robot_skills.perception_capabilities import question_capabilities
from robot_skills.registry import package_config_path


class PerceptionAgent:
    def refine(self, intent, client, jev, mode):
        intent = TaskCatalog().normalize(intent)
        if intent.constraints.get("unresolved_view"):
            if mode == "classic" or jev is None:
                return TaskRefinement(question="Specify a configured camera direction or observation profile.")
            intent = replace(intent, constraints=dict(intent.constraints))
            from robot_agent.command_grounding import choice
            views = self._configured_views()
            questions = {
                "profile": choice("Choose the image acquisition scope requested in this delegated observation goal.", {
                    "directional": "A particular direction", "current_view": "Current camera view", "surroundings": "Look around", "unclear": "Unclear scope"}),
                "view": choice("Choose one configured view. Ground/down without a horizontal direction means ground_center; "
                    "up means upper_center. Unspecified height uses middle. Relative changes are not fixed views.", {
                    **{name: name.replace("_", " ") for name in views},
                    "none": "No fixed view required (current view or surroundings)", "unclear": "Unsupported or ambiguous view"}),
                "analysis": choice("Does the goal explicitly request interpretation, hazards or description? "
                    "Looking toward a direction including ground/up only requests image acquisition unless interpretation is also requested.", {
                    "capture_only": "Acquire images only", "image_describe": "Describe hazards, relationships or scene meaning", "unclear": "Unclear"}),
            }
            answers, item = jev.domain_questions("perception_view_parse", intent, questions, {},
                {"stage_goal": "Resolve observation details inside Perception, not Mission.", "requested_view": intent.constraints["unresolved_view"]})
            if mode == "jev_shadow":
                return TaskRefinement(question="Specify a configured camera direction or observation profile.", traces=[item])
            if "unclear" in answers.values() or (answers["profile"] == "directional" and answers["view"] not in views):
                return TaskRefinement(question="Please clarify the camera view or unsupported relative adjustment.", traces=[item])
            intent.constraints.pop("unresolved_view")
            intent.constraints.update(profile=answers["profile"], analysis_kind=answers["analysis"])
            if answers["profile"] == "directional":
                band, direction = answers["view"].split("_", 1)
                intent.constraints.update(direction="front" if direction == "center" else direction, band=band)
            result = self.refine(intent, client, jev, mode)
            result.traces.insert(0, item)
            return result
        query = intent.object_query or intent.target
        describe = intent.constraints.get("analysis_kind") == "image_describe"
        if intent.task_type == "search_object" or query or describe:
            return self._refine_detection(intent, client, jev, mode, query or intent.raw_text)
        profile = intent.constraints.get("profile", "directional")
        if profile not in {"directional", "surroundings", "current_view"}:
            return TaskRefinement(question="Unknown observation profile.", outcome="UNSUPPORTED")
        if profile == "surroundings":
            plan = self.build_search_plan("around")
        elif profile == "current_view":
            plan = [SkillCall("capture_observation")]
        else:
            direction = intent.constraints.get("direction")
            if direction not in {"left", "right", "front", "front_left", "front_right"}:
                return TaskRefinement(question="Which camera direction should be observed?")
            plan = self.build_look_direction_plan(direction, intent.constraints.get("band", "middle"))
        if mode == "jev_primary":
            plan = [
                replace(call, payload={**call.payload, "require_feedback": True, "wait_timeout_sec": 4.0},
                        timeout_sec=max(call.timeout_sec or 0, 5.0))
                if call.name == "set_camera_pose" else call
                for call in plan
            ]
        parsed, traces = review(jev, mode, "perception_command_parse", intent,
                           {"capture_only": "Acquire images in the requested direction, without an inference claim.",
                            "ask_human": "The request also requires interpretation; clarify it."}, "capture_only",
                           {"stage_goal": "Separate image acquisition from recognition or scene interpretation.", "profile": profile})
        if parsed == "ask_human":
            return TaskRefinement(traces=traces, question="Specify whether to capture images or analyze their contents.")
        choice, items = review(jev, mode, "perception_plan_review", intent,
                                {"execute": "Execute the requested bounded observation profile.",
                                 "ask_human": "Direction or observation scope is ambiguous."}, "execute",
                                {"profile": profile, "base_motion": False})
        traces += items
        if choice == "ask_human":
            return TaskRefinement(traces=traces, question="Clarify the observation direction or scope.")
        return TaskRefinement(plan=[SkillCall("await_robot_stationary", {"settle_sec": 1.0, "timeout_sec": 180.0})] + plan,
                              traces=traces)

    def _refine_detection(self, intent, client, jev, mode, query):
        # Review this delegated goal, not the parent's navigation command.
        intent = replace(intent, raw_text=query)
        config = target_config()
        units, relation, ambiguous = parse_seed(query or "", config)
        contract = intent.constraints.get("recognition_contract")
        if contract is not None:
            tokens = contract.get("tokens") if isinstance(contract, dict) else None
            if (not isinstance(tokens, list) or not 1 <= len(tokens) <= 6
                    or contract.get("kind") != "ocr" or contract.get("aggregation") not in {"any", "all"}
                    or any(not isinstance(t, str) or not re.fullmatch(r"[A-Za-z0-9_]+", t) for t in tokens)):
                return TaskRefinement(question="The mapped recognition contract is invalid or unsupported; check the target YAML.")
            # Alternative spellings belong to one mapped identity, not N goals.
            units = [{"source": query, "kind": "ocr", "object": "none", "source_object": "none",
                      "color": "none", "literal_ocr_query": "text_" + tokens[0]}]
            relation, ambiguous = contract["aggregation"], False
        if not units or len(units) > 6 or ambiguous:
            return TaskRefinement(question="Specify up to six perception clauses with clear AND/OR grouping.")
        questions, defaults = questions_for(units, config, relation)
        if contract:
            questions["unit_count"] = {"type": "choice", "instructions": "This mapped contract defines one recognition goal. Its OCR tokens are checks (all required or alternatives as explicitly configured), not additional goals.",
                                       "criteria": {"1": "One mapped identity goal", "unclear": "Contract conflicts with the requested goal"}}
            questions["relation"] = {"type": "choice", "instructions": "Preserve the explicit token aggregation from the mapped recognition contract; do not reinterpret it as an ordering of tasks.",
                                     "criteria": {relation: "Use the configured token aggregation", "unclear": "Contract conflicts with the requested goal"}}
        traces = []
        if mode != "classic":
            answers, trace = jev.domain_questions("perception_command_parse", intent, questions, defaults,
                {"stage_goal": "Decompose every perception requirement, preserving attributes and alternatives. Camera views are not semantic steps.",
                 "clauses": [u["source"] for u in units], "recognition_contract": contract, "base_motion": False})
            traces.append(trace)
            if mode == "jev_primary":
                if answers["coverage"] != "complete" or answers["relation"] == "unclear":
                    return TaskRefinement(traces=traces, question="Perception requirement coverage or grouping is unclear.")
                relation = answers["relation"]
                if contract and relation != contract["aggregation"]:
                    return TaskRefinement(traces=traces, question="Perception grouping conflicts with the mapped recognition contract.")
                for i, unit in enumerate(units):
                    for key in ("kind", "object", "color"):
                        unit[key] = answers[f"{key}_{i}"]
                expected = sum(2 if u["kind"] == "composite" else 1 for u in units)
                if answers["unit_count"] != str(expected) or any("unclear" in u.values() or u["kind"] == "unknown" for u in units):
                    trace.adopted_decision = "ask_human_inconsistent_parse"
                    return TaskRefinement(traces=traces, question="Perception decomposition count or details are inconsistent.")
        approved = intent.constraints.get("perception_approved_objects", {})
        for i, unit in enumerate(units):
            if str(i) in approved:
                unit["object"] = approved[str(i)]
                if unit["kind"] == "unknown":
                    unit["kind"] = "object"
        checks, gaps, proposals = compile_checks(
            units, config, approved, generic_color_reference=mode == "jev_primary")
        if contract and checks:
            checks = [{"id": f"c0:token{n}", "kind": "ocr", "query": "text_" + token, "provider": "yolo11s_ocr"}
                      for n, token in enumerate(dict.fromkeys(contract["tokens"]))]
        executable, capability_reviews = [], []
        for check in checks:
            snapshot = client.capability_snapshot("perception", check) if hasattr(client, "capability_snapshot") else {}
            evidence = snapshot.get("evidence", {})
            capability_reviews.append({"id": check["id"], "kind": check["kind"],
                                       "supported": snapshot.get("supported"), "available": snapshot.get("available"),
                                       "classes": evidence.get("query_plan", {}).get("yolo_classes", []),
                                       "weights_present": evidence.get("weights_present"),
                                       "backend": evidence.get("backend")})
            if snapshot.get("supported") is False or snapshot.get("available") is False:
                gaps.append({"source": check["query"], "kind": check["kind"], "reason": "Required backend unavailable"})
                if check.get("color_query"):
                    fragment = {"id": check["id"] + ":color", "kind": "color", "query": check["color_query"], "provider": "yolo11s"}
                    fragment_capability = client.capability_snapshot("perception", fragment)
                    if fragment_capability.get("supported") is not False and fragment_capability.get("available") is not False:
                        executable.append(fragment)
            else:
                executable.append(check)
        checks = executable
        choices = [{"label": f"Replace '{p['source']}' object with '{p['object']}' (changes target)",
                    "action": f"perception_replace_{n}",
                    "constraints": {"perception_approved_objects": {**approved, p["unit"]: p["object"]}}}
                   for n, p in enumerate(proposals[:8])]
        if choices and not intent.constraints.get("perception_use_supported_only"):
            if checks:
                choices.append({"label": "Detect supported parts only (full target remains unresolved)",
                                "action": "perception_supported_only",
                                "constraints": {"perception_use_supported_only": True}})
            return TaskRefinement(traces=traces, outcome="NEEDS_HUMAN", choices=choices,
                question="Confirm the proposed object interpretation, or keep only supported parts without claiming the full target.")
        decision = "execute" if checks else "ask_human"
        # Review each concrete check once. A second generic execute/ask vote
        # contradicted supported answers in live replay without actionable facts.
        plan_questions, plan_defaults = {}, {}
        for i, check in enumerate(checks):
            key = f"check_{i}_compatible"
            plan_questions[key] = {"type": "choice", "instructions": f"Does check {i} implement the stated supported fragment? Evaluate this check independently from recorded gaps: color_red implements red only, not box; text_06 reads literal digits, not a YOLO object. The provider names identify pipelines; HSV runs inside the yolo11s pipeline without needing a YOLO class for colors. Runtime readiness is enforced separately by the skill.",
                                   "criteria": {"supported": "Semantic mapping is supported.", "unsupported": "Capability/type does not match.", "unclear": "Need clarification."}}
            plan_defaults[key] = "supported"
        if mode != "classic" and checks:
            answers, item = jev.domain_questions("perception_plan_review", intent, plan_questions, plan_defaults,
                {"stage_goal": "Check each requirement against configured detectors, HSV and OCR. Never treat substitutes as equivalent.",
                 "requirements": units, "checks": checks, "capability_reviews": capability_reviews,
                 "gaps": gaps, "proposed_substitutes": proposals, "aggregation": relation, "capture_once": True,
                 "recognition_contract": contract,
                 "capability_contract": question_capabilities(config),
                 "pipeline_contract": {"ocr": "yolo11s_ocr reads text_X literally", "color": "yolo11s pipeline runs configured HSV for color_X",
                                       "partial_policy": "Run supported fragments, but any unresolved requirement keeps goal_satisfied=false"}})
            traces.append(item)
            if mode == "jev_primary":
                reviewed = []
                for i, check in enumerate(checks):
                    compatibility = answers[f"check_{i}_compatible"]
                    if compatibility == "supported":
                        reviewed.append(check)
                    else:
                        gaps.append({"source": check["query"], "kind": check["kind"], "reason": f"Plan review: {compatibility}"})
                checks = reviewed
                decision = "execute" if checks else "ask_human"
                item.adopted_decision = "execute_supported_checks" if checks else "ask_human_no_supported_check"
        if not checks or decision != "execute":
            return TaskRefinement(traces=traces, question="Perception cannot complete this request as specified: " +
                                  ("; ".join(g["reason"] for g in gaps) or "Plan review requested clarification of the proposed checks."), outcome="UNSUPPORTED", choices=choices)
        payload = {"query": query, "provider": checks[0]["provider"], "profile": self.scan_profile,
                   "checks": checks, "unresolved_requirements": gaps, "aggregation": relation}
        if intent.constraints.get("direction"):
            if intent.constraints["direction"] not in {"left", "right", "front", "front_left", "front_right"}:
                return TaskRefinement(traces=traces, question="Unknown camera direction; choose a calibrated view.")
            pan, tilt, band = self._direction_camera_pose(intent.constraints["direction"], intent.constraints.get("band", "middle"))
            payload.update(profile="directional", pan_deg=pan, tilt_deg=tilt, scan_band=band)
        return TaskRefinement(plan=[SkillCall("search_at_place", payload, timeout_sec=180.0)], traces=traces)

    def __init__(self, scan_profile: str = "shelf_label"):
        self.scan_profile = scan_profile

    def supervise_step(self, jev, **kwargs):
        """Mission calls this only on an unhandled domain execution exception."""
        return jev.supervise_step(**kwargs)

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
        if query.strip().lower() in {"around", "surroundings", "周り", "周囲"}:
            plan: list[SkillCall] = []
            for pan_deg, scan_band in [
                (-35.0, "middle_left"),
                (0.0, "middle_center"),
                (35.0, "middle_right"),
            ]:
                plan.append(
                    SkillCall(
                        name="set_camera_pose",
                        payload={"pan_deg": pan_deg, "tilt_deg": 25.0, "scan_band": scan_band},
                        reason="Move the stopped real camera through a short surroundings scan.",
                        timeout_sec=2.0,
                    )
                )
                plan.append(
                    SkillCall(
                        name="capture_observation",
                        payload={"pan_deg": pan_deg, "tilt_deg": 25.0, "scan_band": scan_band},
                        reason="Capture a surroundings observation without requiring object detection.",
                        timeout_sec=3.0,
                    )
                )
            plan.append(
                SkillCall(
                    name="set_camera_pose",
                    payload={"pan_deg": 0.0, "tilt_deg": 67.0, "scan_band": "return_home"},
                    reason="Return camera holder to the real navigation neutral pose.",
                    timeout_sec=2.0,
                )
            )
            return plan
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
            # Legacy targets still need explicit grouping; whitespace alone is
            # not a clause boundary in the perception parser.
            return " and ".join(f"shelf_label_{token}" for token in digit_tokens)
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
        canonical_direction = {
            "left": "left",
            "front_left": "left",
            "front": "center",
            "center": "center",
            "right": "right",
            "front_right": "right",
        }.get(normalized_direction)
        canonical_band = {
            "ground": "ground",
            "down": "ground",
            "middle": "middle",
            "center": "middle",
            "upper": "upper",
            "up": "upper",
        }.get(normalized_band)
        name = f"{canonical_band}_{canonical_direction}"
        pose = PerceptionAgent._configured_views().get(name)
        if pose is None:
            raise ValueError(f"Unconfigured camera view: {direction}/{band}")
        return float(pose["pan_deg"]), float(pose["tilt_deg"]), name

    @staticmethod
    def _configured_views():
        with open(package_config_path("camera_scan_profiles.yaml"), encoding="utf-8") as stream:
            poses = yaml.safe_load(stream)["profiles"]["shelf_label"]["poses"]
        return {p["id"]: p for p in poses if re.fullmatch(r"(ground|middle|upper)_(left|center|right)", p["id"])}
