"""Bounded JEV task parsing: classify, bind verbatim spans, then compile.

No connector dictionary is used to partition commands. Token offsets are code
owned; JEV selects them and code rejects malformed/overlapping task spans.
"""
import re
import math
import unicodedata

from robot_agent.schemas import MissionIntent


def choice(instructions, criteria):
    return {"type": "choice", "instructions": instructions, "criteria": criteria}


def tokens(text):
    return [{"id": f"w{i}", "text": m.group(), "start": m.start(), "end": m.end()}
            for i, m in enumerate(re.finditer(r"\d+(?:\.\d+)?|[A-Za-z_][A-Za-z_0-9-]*|[^\s]", text))]


def structure_questions(catalog):
    task_boundary = (
        "Count requested actions, not target nouns. One perception request with multiple "
        "attributes, AND requirements or OR alternatives is one search/observation task; "
        "its domain agent handles those checks. A second task requires a separately "
        "requested action, not a second noun under the same action. "
        "In a search-then-if-found-action request, count the search and the conditional "
        "action as two tasks even though execution of the action is conditional. "
        "The otherwise-stay-still branch is control flow, not another task. "
    )
    tasks = {name: f"{item['description']}; delegated to {item['owner']}. {item.get('parsing_contract', '')}"
             for name, item in catalog.tasks.items()}
    tasks.update(unknown="The task cannot be identified", absent="There is no task at this position")
    questions = {
        "command_shape": choice("Classify the user's request structure, not whether physical movement is requested. "
            "Read-only robot status, listing known destinations, observation and cancellation are robot tasks too. "
            "Short labels and questions can request these tasks without an imperative verb. "
            "Do not assess hardware/model availability or whether a destination is registered here. "
            "A compound perception target such as a colored object is one task. A hold/stay-still false branch is not an extra task.", {
                "linear": "One robot task (including a read-only query), or several ordered tasks, including unconditional cleanup after an attempt",
                "if_found_else_hold": "Search for a target; only if fully found execute one supported task (including arm, navigation or relative motion); otherwise do nothing and remain stationary",
                "unsupported_control_flow": "Other branching, loops, parallel execution or unresolved control flow",
                "not_robot_command": "Unrelated conversation or no identifiable robot task; not a read-only robot query"}),
        "task_count": choice(task_boundary + "How many goal-level tasks are explicitly requested? Count all goals, not internal robot skills or object attributes.", {
            "one": "One goal", "two": "Two goals", "three": "Three goals",
            "too_many": "More than three goals", "unclear": "Cannot determine the goals"}),
    }
    for i in range(1, 4):
        questions[f"task_{i}_type"] = choice(
            task_boundary +
            f"Identify goal {i} in the user's requested execution order from the full original command. "
            "Classify the ACTION and argument role, not a target noun in isolation. "
            "An explicit arm posture differs from base navigation; omitted actuator/frame information may require clarification. "
            "Select absent if there is no separate requested action at this position. "
            "Do not classify connecting words as tasks. Do not reject a recognized task because its detector/hardware might be unavailable.", tasks)
        questions[f"task_{i}_alternative_type"] = choice(
            f"For goal {i}, is there a second genuinely plausible, different action interpretation that the user must disambiguate? "
            "Select the alternative to the most likely reading, independently of other question answers. "
            "Missing actuator/reference frame can make a named home/pose ambiguous between arm posture and base destination. "
            "Do not invent alternatives for explicit arm or explicit navigation instructions. "
            "This is another interpretation of THIS goal, not an additional goal in the sequence. "
            "Select none when unambiguous or this goal is absent.", {
                **{name: text for name, text in tasks.items() if name in catalog.tasks},
                "none": "No other plausible action interpretation"})
    return questions


def interpretation_candidates(catalog, ordinal, primary, alternative, response):
    """Explicit semantic ambiguity or a close choice distribution, not fake scores."""
    answer = (response.get("answers") or {}).get(f"task_{ordinal}_type", {})
    probabilities = answer.get("probabilities", {}) if isinstance(answer, dict) else {}
    probabilities = {k: v for k, v in probabilities.items() if k in catalog.tasks
                     and isinstance(v, (int, float)) and not isinstance(v, bool)
                     and math.isfinite(v) and 0 <= v <= 1} if isinstance(probabilities, dict) else {}
    alternatives = [alternative] if alternative in catalog.tasks and alternative != primary else []
    if not alternatives and primary in probabilities:
        ranked = sorted(probabilities, key=probabilities.get, reverse=True)
        # These are conservative clarification heuristics, not calibrated certainty.
        if (len(ranked) >= 2 and primary in ranked[:2] and probabilities[ranked[1]] >= .25
                and probabilities[ranked[0]] - probabilities[ranked[1]] <= .20):
            alternatives = [k for k in ranked[:2] if k != primary]
    if not alternatives:
        return []
    return [{"task_type": kind, "label": f"Task {ordinal}: " + catalog.tasks[kind].get(
                "human_label", f"{catalog.tasks[kind]['owner'].title()}: {catalog.tasks[kind]['description']}"),
             **({"type_choice_probability": probabilities[kind]} if kind in probabilities else {})}
            for kind in [primary, *alternatives]]


def disambiguation_questions(catalog, task_types, ordinals):
    questions = {}
    for i in ordinals:
        primary = task_types[i - 1]
        questions[f"task_{i}_alternative_type"] = choice(
            f"Goal {i} was classified as {primary}. Its previous alternative duplicated that type. "
            "Now assess another genuinely plausible interpretation of THIS goal, not another goal in the sequence. "
            "Use the original command and the known primary interpretation; explicit arm/base instructions need no alternative. "
            "Choose none if the primary is unambiguous, unclear if unresolved. Never repeat the excluded primary type.", {
                **{name: f"{item['description']}; owner={item['owner']}"
                   for name, item in catalog.tasks.items() if name != primary},
                "none": "No other plausible interpretation", "unclear": "Cannot resolve the ambiguity"})
    return questions


def binding_questions(task_types, words):
    positions = {w["id"]: w["text"] for w in words}
    positions["unclear"] = "No unambiguous token boundary"
    questions = {}
    for i, task in enumerate(task_types, 1):
        prefix = f"task_{i}"
        for boundary in ("start", "end"):
            questions[f"{prefix}_{boundary}"] = choice(
                f"Choose the inclusive {boundary} token of goal {i} ({task}) in the numbered original text. "
                "Include its verb, complete target expression and explicit parameters. "
                "Keep AND/OR noun alternatives inside one perception request, including the final alternative. "
                "Only connectors between separately requested actions are sequence connectors. "
                "Exclude sequence connectors, conditional introducers, "
                "other tasks and branch/cleanup qualifiers. Do not rely on other question answers.", positions)
        if task not in {"get_robot_status", "list_known_targets", "cancel", "move_relative", "rotate_relative"}:
            for boundary in ("start", "end"):
                questions[f"{prefix}_goal_{boundary}"] = choice(
                    f"Choose the inclusive {boundary} token of the requested target/view/pose phrase of goal {i} ({task}). "
                    "Preserve all target attributes and the ENTIRE AND/OR expression, not just its first object. "
                    "Exclude the task verb and other goals. Preserve perception markers such as text, label, "
                    "color and all attributes: they determine the recognition contract, not just the target value. "
                    "For set_arm_pose, exclude actuator/type descriptors and bind the requested "
                    "pose name/gesture, not the words arm or pose introducing that name. "
                    "Copy the user's request, do not choose a detector, validate a preset, or substitute another target.", positions)
        dependencies = {"independent": "Only execution order is specified; no successful-result prerequisite is requested",
                        "unclear": "The success dependency is ambiguous"}
        if i > 1:
            dependencies["after_attempt"] = "The user explicitly requests THIS goal even if preceding goals fail or are unmet"
        for prior in range(1, i):
            dependencies[f"success_{prior}"] = f"Goal {i} requires the result/location produced by goal {prior}, or the user explicitly makes its success a condition"
        if i > 2:
            dependencies["success_all"] = "Requires all preceding goals to succeed"
        questions[f"{prefix}_dependency"] = choice(
            f"What success condition gates goal {i} ({task})? Temporal order alone is not a success dependency. "
            "Sequence connectors require ordering only. Do not add success edges merely because actions are sequential. "
            "A qualifier belongs to the action it modifies, not every action in the command. "
            "Inspection at a destination requires arrival. Reset/home is not automatically unconditional unless context supports it. "
            "For an explicit if-found branch choose success of the preceding search.", dependencies)
        if task == "move_relative":
            questions[f"{prefix}_direction"] = choice(f"What base translation direction is requested in goal {i}?", {
                "forward": "Forward", "back": "Backward", "left": "Left", "right": "Right", "unclear": "Unclear or compound direction"})
            questions[f"{prefix}_distance"] = choice(f"How is distance specified for goal {i}?", {
                "explicit": "A numeric distance with unit is given; code must extract exactly that value",
                "small": "Explicit vague small motion, e.g. a little; use the configured catalog small-distance default",
                "unclear": "No safe distance specified"})
        if task == "rotate_relative":
            questions[f"{prefix}_turn_direction"] = choice(f"What sign convention applies to the rotation in goal {i}?", {
                "left": "Left/counterclockwise", "right": "Right/clockwise",
                "signed": "Use the explicit signed numeric angle, or direction-neutral turn-around",
                "unclear": "Unclear or contradictory direction"})
            questions[f"{prefix}_rotation"] = choice(f"What rotation is requested in goal {i}? Do not discard explicit angles.", {
                "explicit": "An explicit numeric angle is present; code must extract it",
                "left": "Left turn with no specified angle: use catalog left-turn default",
                "right": "Right turn with no specified angle: use catalog right-turn default",
                "around": "Turn around with no specified angle: use catalog turn-around default",
                "unclear": "Unclear rotation"})
    return questions


def numeric_parameter(raw, kind):
    raw = unicodedata.normalize("NFKC", raw)
    units = r"mm|cm|meters?|metres?|m|ミリメートル|センチメートル|メートル|毫米|厘米|米" if kind == "distance" else r"degrees?|deg|度"
    matches = re.findall(rf"(?<![\w.])([+-]?\d+(?:\.\d+)?)\s*({units})(?![A-Za-z])", raw, re.I)
    if len(matches) != 1:
        raise ValueError("missing_or_ambiguous_numeric_parameter")
    value, unit = float(matches[0][0]), matches[0][1].lower()
    if not math.isfinite(value):
        raise ValueError("nonfinite_parameter")
    scale = .001 if unit in {"mm", "ミリメートル", "毫米"} else .01 if unit in {"cm", "センチメートル", "厘米"} else 1
    return value * scale


def selected_span(text, words, answers, prefix):
    positions = {w["id"]: i for i, w in enumerate(words)}
    first, last = positions.get(answers.get(prefix + "_start")), positions.get(answers.get(prefix + "_end"))
    if first is None or last is None or first > last:
        raise ValueError("invalid_source_span")
    start, end = words[first]["start"], words[last]["end"]
    return start, end, text[start:end]


def compile_tasks(agent, text, words, task_types, flow, answers):
    steps, last_end = [], -1
    for i, task in enumerate(task_types, 1):
        prefix = f"task_{i}"
        start, end, raw = selected_span(text, words, answers, prefix)
        if start < last_end:
            raise ValueError("overlapping_or_reordered_task_spans")
        last_end = end
        constraints = {"source_span": {"start": start, "end": end}, "task_ordinal": i}
        dependency = answers[prefix + "_dependency"]
        if dependency == "unclear":
            raise ValueError("unclear_task_dependency")
        refs = list(range(1, i)) if dependency == "success_all" else [int(dependency[8:])] if dependency.startswith("success_") else []
        constraints.update(depends_on=refs, dependency_policy="required_condition" if refs else
                           "cleanup_after_attempt" if dependency == "after_attempt" else "ordered_but_independent")
        if dependency == "after_attempt":
            constraints["explicit_after_attempt"] = True
        intent = MissionIntent(task, raw_text=raw, constraints=constraints)
        defaults = agent.task_catalog.tasks[task].get("defaults", {})
        if task == "move_relative":
            direction, policy = answers[prefix + "_direction"], answers[prefix + "_distance"]
            distance = numeric_parameter(raw, "distance") if policy == "explicit" else defaults.get("small_distance_m") if policy == "small" else None
            if policy != "explicit" and re.search(r"\d", unicodedata.normalize("NFKC", raw)):
                raise ValueError("explicit_number_cannot_be_defaulted")
            if direction == "unclear" or distance is None or distance <= 0:
                raise ValueError("unresolved_motion_parameters")
            constraints.update(direction=direction, distance_m=distance,
                               parameter_source="source_text" if policy == "explicit" else "catalog_small_default")
        elif task == "rotate_relative":
            policy = answers[prefix + "_rotation"]
            yaw = numeric_parameter(raw, "angle") if policy == "explicit" else defaults.get(policy + "_yaw_deg")
            direction = answers[prefix + "_turn_direction"]
            if direction == "unclear":
                raise ValueError("unclear_rotation_direction")
            if policy in {"left", "right"} and direction != policy:
                raise ValueError("contradictory_rotation_direction")
            if policy == "explicit" and direction in {"left", "right"}:
                if yaw < 0 and direction == "left":
                    raise ValueError("contradictory_rotation_sign")
                yaw = abs(yaw) * (-1 if direction == "right" else 1)
            if policy != "explicit" and re.search(r"\d", unicodedata.normalize("NFKC", raw)):
                raise ValueError("explicit_number_cannot_be_defaulted")
            if yaw is None:
                raise ValueError("unresolved_rotation_parameters")
            constraints.update(yaw_deg=yaw, parameter_source="source_text" if policy == "explicit" else "catalog_turn_default")
        elif task not in {"get_robot_status", "list_known_targets", "cancel"}:
            gs, ge, goal = selected_span(text, words, answers, prefix + "_goal")
            if gs < start or ge > end:
                raise ValueError("goal_outside_task_span")
            constraints["requested_goal"] = goal
            constraints["goal_source_span"] = {"start": gs, "end": ge}
            if task in {"search_object", "interact_with_object", "manipulate_object"}:
                intent.object_query = goal
            elif task == "observe_scene":
                constraints["unresolved_view"] = goal
            else:
                intent.target = goal
        steps.append(intent.to_dict())
    if flow == "if_found_else_hold":
        if (len(steps) != 2 or task_types[0] != "search_object"
                or task_types[1] not in agent.task_catalog.tasks or task_types[1] == "cancel"):
            raise ValueError("unsupported_conditional_graph")
        if steps[1]["constraints"]["depends_on"] != [1]:
            raise ValueError("conditional_dependency_missing")
        steps[1]["constraints"]["run_if"] = "previous_search_found"
    if len(steps) == 1:
        if flow != "linear":
            raise ValueError("conditional_graph_incomplete")
        return MissionIntent(**steps[0])
    return MissionIntent("sequence", raw_text=text, constraints={"steps": steps, "grounding_version": 2,
                         "dependency_policy": "per_task", "control_flow": flow,
                         **({"else_action": "stay_still"} if flow == "if_found_else_hold" else {})})
