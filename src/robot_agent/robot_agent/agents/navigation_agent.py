from __future__ import annotations

from copy import deepcopy
from dataclasses import replace

from robot_agent.schemas import MissionIntent, SkillCall
from robot_agent.agents.domain import TaskRefinement
from robot_agent.agents.navigation_plan import applicable_rules, choice, path_evidence
from robot_agent.task_catalog import TaskCatalog


class NavigationAgent:
    @staticmethod
    def _questions(jev, mode, stage, intent, questions, defaults, evidence, traces):
        if mode == "classic":
            return defaults
        answers, trace = jev.domain_questions(stage, intent, questions, defaults, evidence)
        # Omitted questions are deterministic bindings, not missing API answers.
        answers = {**{k: v for k, v in defaults.items() if k not in questions}, **answers}
        if stage == "navigation_command_parse":
            valid = (answers.get("goal_binding") == "matched" and answers.get("goal_kind") == defaults["goal_kind"]
                     and answers.get("visual_confirmation") in {"required", "not_required"})
            trace.adopted_decision = "accept_navigation_goal" if valid else "ask_human"
        elif "route_strategy" in defaults and answers.get("route_strategy") != defaults["route_strategy"]:
            trace.adopted_decision = "ask_human"
        traces.append(trace)
        return defaults if mode == "jev_shadow" else answers

    def refine(self, intent, client, jev, mode):
        traces = []
        semantic = getattr(getattr(client, "semantic", None), "semantic_map", None)
        if semantic is None:
            return TaskRefinement(question="Navigation needs the active calibrated semantic map.",
                                  outcome="CAPABILITY_UNAVAILABLE")
        if intent.task_type == "follow_waypoint_list":
            from robot_skills.waypoint_routes import named_route
            try:
                route = named_route(intent.target, semantic.data.get("map_id"))
            except (OSError, ValueError) as exc:
                return TaskRefinement(question=f"Waypoint list unavailable: {exc}", outcome="UNKNOWN_ROUTE")
            # Reuse the ordinary navigation prerequisites, with no semantic pose resolution.
            plan = self.build_plan(MissionIntent("navigate_to_place", target="waypoint"))[1:-1]
            for index, pose in enumerate(route["poses"]):
                plan.append(SkillCall("goto_pose", payload={
                    "pose": {**pose, "frame_id": "map"}, "timeout_sec": 360.0,
                    "dynamic_timeout": False, "apply_navigation_annotations": False,
                    "recovery_policy": {"allow_alternate_goal": False, "allow_relaxed_tolerance": False,
                                        "allow_direct_preparation": False},
                }, timeout_sec=360.0,
                    reason=f"Waypoint list {intent.target}: {index + 1}/{len(route['poses'])} ({route['waypoints'][index].get('id', index)})",
                    catalog={"waypoint_route": {"id": route.get("route_id", intent.target),
                        "sha256": route["sha256"], "path": route["path"], "index": index,
                        "count": len(route["poses"]), "waypoint_id": route["waypoints"][index].get("id")}}))
            return TaskRefinement(plan=plan)
        selected = intent.constraints.get("navigation_selected_target")
        requested = selected or intent.target
        if not requested:
            return TaskRefinement(question="Which registered destination should the robot navigate to?")
        matches = semantic.find_matches(requested)
        if len(matches) != 1:
            if not matches:
                return TaskRefinement(question=f"Destination '{requested}' is not registered in the active map. Use List known targets and submit a registered destination; no replacement was selected.",
                                      outcome="UNKNOWN_TARGET")
            choices = [{"label": str(x.get("canonical_name") or x["id"]),
                        "action": f"navigation_target_{i}",
                        "constraints": {"navigation_selected_target": x["id"]}}
                       for i, x in enumerate(matches[:8])]
            return TaskRefinement(question=f"Choose a calibrated destination for '{requested}'.",
                                  choices=choices, outcome="NEEDS_HUMAN")
        entity = matches[0]
        target_id = str(entity["id"])
        if selected and selected != target_id:
            return TaskRefinement(question="The selected destination is no longer registered by that exact ID.",
                                  outcome="UNKNOWN_TARGET")
        relation = intent.relation or "near"
        # SemanticMap's legacy fallback-to-first-approach must not erase explicit requirements.
        if relation != "near" and (entity.get("map_region") or not any(p.get("id") == relation for p in entity.get("approach_poses", []))):
            return TaskRefinement(question=f"No calibrated '{relation}' approach for {target_id}.",
                                  outcome="UNSUPPORTED_APPROACH")
        target = semantic.resolve(target_id, relation)
        if target is None or target.needs_clarification:
            return TaskRefinement(question=f"{target_id} needs an unambiguous calibrated navigation pose.",
                                  outcome="UNCALIBRATED")
        rules, errors = applicable_rules(semantic, target_id)
        if errors or any(r["effect"] == "deny_destination" for r in rules):
            return TaskRefinement(question="Navigation rule blocks this task: " + "; ".join(
                errors or [r.get("message", r["id"]) for r in rules if r["effect"] == "deny_destination"]),
                outcome="RULE_BLOCKED")
        intent = replace(intent, target=target_id, constraints=deepcopy(intent.constraints))
        goal_kind = "region" if target.region else "point"
        arrival_spec = entity.get("visual_arrival_confirm", {})
        confirmation = intent.constraints.get("visual_confirmation")
        if confirmation is None:
            confirmation = arrival_spec.get("enabled", any(t in target_id.lower() for t in ("shelf", "棚")))
        confirmation = bool(confirmation or any(r["effect"] == "require_visual_confirmation" for r in rules))
        evidence = {"stage_goal": "Ground this navigation goal, resolve ambiguity and preserve all approach requirements.",
                    "entity": {"id": target_id, "name": target.canonical_name, "category": target.category,
                               "goal_kind": goal_kind, "pose": target.pose.to_dict(),
                               "approach_relation": relation, "approach_source": target.approach_source},
                    "rules": rules, "visual_confirmation": confirmation,
                    "target_binding": {"source": "human_selection" if selected else "semantic_map",
                                       "original_request": intent.constraints.get("requested_goal", intent.raw_text),
                                       "resolved_target_id": target_id,
                                       "destination_ambiguity_resolved": True,
                                       "registered_match_count": len(matches)}}
        questions = {
            "goal_binding": choice("Check whether the requested destination contradicts the registered binding. The map resolver already found exactly one matching entity; destination_ambiguity_resolved records that fact, not missing human approval. Do not require human confirmation solely because source is semantic_map. Approach and rule validation are separate deterministic checks.",
                                   {"matched": "The unique registered binding preserves the requested destination.",
                                    "ask_human": "The request contains an additional identity requirement not satisfied by this binding, or contradicts the bound entity."}),
            "goal_kind": choice("What is the grounded goal geometry?", {"region": "Configured region.", "point": "Configured pose."}),
            "visual_confirmation": choice("Is identity confirmation required to complete this task?",
                                          {"required": "Require visual confirmation.", "not_required": "Arrival suffices.",
                                           "ask_human": "Confirmation goal unclear."})}
        if selected:
            # The scoped human answer resolves identity only; rules, approach,
            # confirmation and the subsequent plan review remain mandatory.
            questions.pop("goal_binding")
        parsed = self._questions(jev, mode, "navigation_command_parse", intent, questions,
            {"goal_binding": "matched", "goal_kind": goal_kind,
             "visual_confirmation": "required" if confirmation else "not_required"}, evidence, traces)
        if parsed.get("goal_binding") != "matched" or parsed.get("goal_kind") != goal_kind or parsed.get("visual_confirmation") not in {"required", "not_required"}:
            return TaskRefinement(traces=traces, question="Clarify the destination, approach or identity-confirmation goal.")
        confirmation = confirmation or parsed["visual_confirmation"] == "required"
        plan = self.build_plan(TaskCatalog().legacy(intent))
        contract = {"target_id": target_id, "relation": relation, "goal_kind": goal_kind,
                    "visual_confirmation_required": confirmation, "rules": rules,
                    "confirmation_timing": "after_successful_arrival" if confirmation else "not_required"}
        for step in plan:
            if step.name == "goto_pose":
                step.catalog["navigation_contract"] = contract
                step.catalog["navigation_intent"] = intent.to_dict()
                # Explicit approach relations may not be relaxed through another side.
                if relation != "near":
                    step.payload["recovery_policy"] = {"allow_alternate_goal": False, "allow_relaxed_tolerance": False}
        if confirmation:
            from robot_agent.agents.perception_agent import PerceptionAgent
            query = arrival_spec.get("query") or PerceptionAgent._arrival_confirmation_query(target_id)
            sub = MissionIntent("search_object", object_query=query, raw_text=intent.raw_text,
                                constraints={"goal_mode": "verify", "dependency_policy": "required_condition",
                                             **({"recognition_contract": arrival_spec["recognition"]} if "recognition" in arrival_spec else {})})
            plan.append(SkillCall("search_object", payload=sub.to_dict(), kind="task", owner="perception"))
        reviewed = self._questions(jev, mode, "navigation_plan_review", intent, {
            "decision": choice("Review the ordered skills, target and rule completeness before execution. Path is not computed yet.",
                               {"execute": "Grounded task can proceed to guarded execution.",
                                "ask_human": "Unresolved goal, unsupported rule or missing prerequisite."})},
            {"decision": "execute"}, {**evidence, "phase": "semantic",
            "stage_goal": "Validate goal contract and ordered plan; do not claim that a physical path was reviewed.",
            "skills": [s.name for s in plan], "goal_contract": contract}, traces)
        if reviewed.get("decision") != "execute":
            return TaskRefinement(traces=traces, question="Navigation plan requires clarification.")
        return TaskRefinement(plan=plan, traces=traces)

    def review_path(self, intent, contract, evidence, client, jev, mode):
        semantic = getattr(getattr(client, "semantic", None), "semantic_map", None)
        if semantic is None:
            return {"decision": "ask_human", "reason": "Semantic map unavailable"}, []
        rules, errors = applicable_rules(semantic, contract["target_id"])
        if errors or any(r["effect"] == "deny_destination" for r in rules) or rules != contract["rules"]:
            return {"decision": "ask_human", "reason": "Rules changed or block navigation; refine the task again"}, []
        compact = path_evidence(evidence["path"], evidence["target_pose"], evidence.get("annotation") or {})
        traces = []
        strategy = "annotated_segments" if evidence.get("annotation") else "nav2"
        answers = self._questions(jev, mode, "navigation_plan_review", intent, {
            "decision": choice("Review goal/route-policy consistency, not physical clearance. Visual confirmation is a later task after arrival, not a prerequisite to dispatch. Safety, localization and collision checks remain mandatory in the executor and cannot be waived by this answer.",
                {"execute": "No goal or route-policy conflict is evidenced; proceed only through executor safety checks.",
                 "ask_human": "An evidenced goal/policy conflict or unsupported route requirement needs clarification."}),
            "blocking_issue": choice("Classify the specific goal/route-policy issue, if any. Missing raw safety telemetry here is handled by executor checks, not a semantic conflict. Do not infer a narrow passage from an empty annotation.",
                {"none": "No evidenced goal/policy conflict.", "goal_conflict": "Path conflicts with grounded destination or approach.",
                 "rule_conflict": "An applicable rule conflicts with this route.", "unsupported_strategy": "A required route strategy is not executable.",
                 "insufficient_route_evidence": "Goal or route-policy evidence is insufficient for this review."}),
            "route_strategy": choice("Validate the implemented strategy. Do not invent waypoints or disable narrow-passage safeguards.",
                {strategy: "Use the configured ordered annotation strategy." if strategy == "annotated_segments" else "Use guarded Nav2 execution.",
                 "ask_human": "The supported strategy cannot satisfy this route."})},
            {"decision": "execute", "route_strategy": strategy, "blocking_issue": "none"}, {"stage_goal": "Review computed path consistency with the already grounded goal and rules; do not repeat command parsing or judge unprovided sensor telemetry.",
            "phase": "computed_path", "goal_contract": contract, "path": compact,
            "execution_contract": {"collision_check": "Nav2 costmaps/controllers; not certified by JEV",
                                   "pre_dispatch_recheck": ["cancellation", "AUTO", "safety", "localization", "unchanged_start_pose"],
                                   "visual_confirmation": contract.get("confirmation_timing", "after_successful_arrival")},
            "execution_role": evidence.get("execution_role")}, traces)
        decision = answers.get("decision", "ask_human") if answers.get("route_strategy") == strategy else "ask_human"
        issue = answers.get("blocking_issue", "insufficient_route_evidence")
        if issue != "none":
            decision = "ask_human"
        reasons = {"goal_conflict": "JEV flagged a destination or approach conflict",
                   "rule_conflict": "JEV flagged an applicable route rule conflict",
                   "unsupported_strategy": "JEV flagged an unsupported route strategy",
                   "insufficient_route_evidence": "JEV needs more goal or route-policy evidence"}
        reason = reasons.get(issue, "JEV requested clarification without identifying a specific route conflict") if decision != "execute" else "Goal/route-policy review accepted; executor safety checks still required"
        if traces and mode != "jev_shadow":
            traces[-1].adopted_decision = decision
        return {"decision": decision, "path_id": compact["path_id"], "route_strategy": answers.get("route_strategy", "ask_human"),
                "implemented_route_strategy": strategy,
                "phase": "computed_path", "blocking_issue": issue, "reason": reason}, traces

    def supervise_step(self, jev, **kwargs):
        return jev.supervise_step(**kwargs)

    @staticmethod
    def arrival_feedback(result):
        metrics = result.get("metrics") or {}
        return {"reached": bool(result.get("success")), "code": result.get("code"),
                "completion_class": metrics.get("completion_class", "unclassified"),
                "final_xy_error_m": metrics.get("final_xy_error_m"),
                "final_yaw_error_rad": metrics.get("final_yaw_error_rad"),
                "result_step_id": result.get("step_id")}

    def build_plan(self, intent: MissionIntent) -> list[SkillCall]:
        if intent.target is None:
            return []
        return [
            SkillCall(
                name="resolve_semantic_target",
                payload={"target_name": intent.target, "relation": intent.relation or "near"},
                reason="Resolve natural-language target to map-frame pose.",
                timeout_sec=1.0,
            ),
            SkillCall(
                name="await_robot_stationary",
                payload={"settle_sec": 0.5, "timeout_sec": 10.0},
                reason="Stop the base before restoring the camera navigation pose.",
                timeout_sec=10.0,
            ),
            SkillCall(
                name="set_camera_pose",
                payload={
                    "pan_deg": 0.0,
                    "tilt_deg": 67.0,
                    "scan_band": "return_home",
                    "wait_timeout_sec": 5.0,
                    "tolerance_deg": 2.0,
                    "require_feedback": True,
                },
                reason="Restore and verify the camera pose required by the local costmap before navigation.",
                timeout_sec=6.0,
            ),
            SkillCall(
                name="await_robot_auto_mode",
                payload={"timeout_sec": 0.1},
                reason="Human must press A / confirm AUTO before navigation.",
                timeout_sec=0.1,
                requires_human_confirm=True,
            ),
            SkillCall(
                name="check_localization_health",
                payload={"settle_timeout_sec": 5.0, "allow_warmup_pass": True},
                reason="Verify localization before sending a Nav2 goal.",
                timeout_sec=6.0,
            ),
            SkillCall(
                name="goto_pose",
                payload={
                    "pose": "__resolved_pose__",
                    "pose_candidates": "__resolved_pose_candidates__",
                    "recovery_policy": "__resolved_recovery_policy__",
                    "timeout_sec": 60.0,
                    "dynamic_timeout": True,
                    "max_timeout_sec": 60.0,
                    "xy_goal_tolerance_m": 0.25,
                    "yaw_goal_tolerance_rad": 0.45,
                    "tolerance_profile": "semantic_near",
                },
                reason="Navigate to resolved approach pose using Nav2.",
                timeout_sec=60.0,
            ),
        ]
