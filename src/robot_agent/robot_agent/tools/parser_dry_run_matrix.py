from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from datetime import datetime
from typing import Any

from robot_agent.parsers.hybrid_parser import HybridParser

CSV_FIELDS = [
    "id",
    "category",
    "input_text",
    "expected_layer",
    "expected_task_type",
    "expected_target",
    "expected_object_query",
    "expected_constraints",
    "expected_ambiguity_reason",
    "expected_choice_count",
    "expected_choice_task_types",
    "notes",
]

RESULT_EXTRA_FIELDS = [
    "actual_layer",
    "actual_task_type",
    "actual_target",
    "actual_object_query",
    "actual_constraints",
    "actual_ambiguity_reason",
    "actual_choice_count",
    "actual_choice_task_types",
    "actual_choice_labels",
    "layer_match",
    "task_type_match",
    "target_match",
    "object_query_match",
    "constraints_match",
    "ambiguity_reason_match",
    "choice_count_match",
    "overall_pass",
    "route_trace_summary",
]

FLOAT_TOLERANCE = 1e-6


def classify_layer(last_metrics: dict[str, Any]) -> str:
    """Map HybridParser.last_metrics to one of the six pipeline states this
    harness distinguishes. accepted_layer alone collapses "ended at Embedding
    without accepting" (Ambiguous/Clarify) and "ended at LLM Fallback without
    accepting" (Rejected) into the same None, so route_steps is needed to tell
    them apart.
    """
    accepted = last_metrics.get("accepted_layer")
    if accepted in ("Deterministic", "Embedding", "LLM Fallback"):
        return accepted
    if last_metrics.get("source") == "ambiguity_gate":
        return "Ambiguity Gate"
    route_steps = last_metrics.get("route_steps", [])
    if any(step.get("layer") == "LLM Fallback" for step in route_steps):
        return "Rejected"
    return "Embedding Ambiguous"


def route_trace_summary(last_metrics: dict[str, Any]) -> str:
    parts = []
    for step in last_metrics.get("route_steps", []):
        layer = step.get("layer", "?")
        status = step.get("status", "?")
        detail = ""
        if "score" in step and step.get("score") is not None:
            detail = f"(score={step['score']:.3f})"
        elif step.get("reason"):
            detail = f"({step['reason']})"
        elif step.get("slot_error"):
            detail = f"({step['slot_error']})"
        parts.append(f"{layer}:{status}{detail}")
    return " -> ".join(parts)


def parse_expected_constraints(raw: str) -> dict[str, Any]:
    raw = (raw or "").strip()
    if not raw:
        return {}
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        return {"__parse_error__": raw}
    return parsed if isinstance(parsed, dict) else {"__parse_error__": raw}


def constraints_subset_match(expected: dict[str, Any], actual: dict[str, Any]) -> bool:
    if not expected:
        return True
    if "__parse_error__" in expected:
        return False
    for key, expected_value in expected.items():
        if key not in actual:
            return False
        actual_value = actual[key]
        if isinstance(expected_value, (int, float)) and isinstance(actual_value, (int, float)):
            if abs(float(expected_value) - float(actual_value)) > FLOAT_TOLERANCE:
                return False
        elif expected_value != actual_value:
            return False
    return True


def ambiguity_choice_task_types(choices: list[dict[str, Any]]) -> list[str]:
    """Ambiguity choices come in two shapes depending on which gate produced
    them: the command-level Ambiguity Gate nests task_type under "intent",
    while the Embedding ambiguous/needs_clarification gate (top_matches) has
    task_type at the top level. Handle both.
    """
    types: list[str] = []
    for choice in choices or []:
        if "task_type" in choice:
            types.append(str(choice.get("task_type")))
        elif isinstance(choice.get("intent"), dict):
            types.append(str(choice["intent"].get("task_type")))
    return types


def ambiguity_choice_labels(choices: list[dict[str, Any]]) -> list[str]:
    """Human-readable text for each offered choice, so a reviewer can judge
    whether the candidates actually make sense for the input (not just count
    them or check their task_type) -- e.g. "check it" offering "look right" as
    a candidate is a red flag that only shows up if you can see the label.
    The Ambiguity Gate shapes this as "label"; the Embedding-ambiguous gate
    shapes it as "text" (the matched template example).
    """
    labels: list[str] = []
    for choice in choices or []:
        label = choice.get("label") or choice.get("text")
        if label is None and isinstance(choice.get("intent"), dict):
            label = choice["intent"].get("task_type")
        labels.append(str(label) if label is not None else "?")
    return labels


def evaluate_row(row: dict[str, str], parser: HybridParser) -> dict[str, Any]:
    intent = parser.parse(row["input_text"])
    metrics = dict(parser.last_metrics or {})

    actual_layer = classify_layer(metrics)
    expected_layer = (row.get("expected_layer") or "").strip()
    expected_task_type = (row.get("expected_task_type") or "").strip()
    expected_target = (row.get("expected_target") or "").strip()
    expected_object_query = (row.get("expected_object_query") or "").strip()
    expected_constraints = parse_expected_constraints(row.get("expected_constraints", ""))
    expected_ambiguity_reason = (row.get("expected_ambiguity_reason") or "").strip()
    expected_choice_count_raw = (row.get("expected_choice_count") or "").strip()
    expected_choice_task_types_raw = (row.get("expected_choice_task_types") or "").strip()

    llm_step = next((s for s in metrics.get("route_steps", []) if s.get("layer") == "LLM Fallback"), None)
    llm_unreachable = bool(llm_step and llm_step.get("error"))

    layer_match = (not expected_layer) or (actual_layer == expected_layer)
    task_type_match = (not expected_task_type) or (intent.task_type == expected_task_type)

    # LLM Fallback wording is not template-locked, so target/object_query are
    # recorded for review but do not gate pass/fail for that layer.
    soft_fields = expected_layer == "LLM Fallback"
    target_match = True if (not expected_target or soft_fields) else (intent.target == expected_target)
    object_query_match = (
        True if (not expected_object_query or soft_fields) else (intent.object_query == expected_object_query)
    )
    constraints_match = constraints_subset_match(expected_constraints, intent.constraints or {})

    # ask_human intents (Ambiguity Gate / Embedding Ambiguous / LLM-validated
    # ambiguity) are supposed to carry a "choices" list the human can pick
    # from. Verify that mechanism actually fired, not just that task_type
    # came out as ask_human.
    actual_ambiguity = (intent.constraints or {}).get("ambiguity") or {}
    actual_choices = actual_ambiguity.get("choices") or []
    actual_ambiguity_reason = str(actual_ambiguity.get("reason") or "")
    actual_choice_count = len(actual_choices)
    actual_choice_task_types = ambiguity_choice_task_types(actual_choices)
    actual_choice_labels = ambiguity_choice_labels(actual_choices)

    ambiguity_reason_match = (not expected_ambiguity_reason) or (actual_ambiguity_reason == expected_ambiguity_reason)
    choice_count_match = (not expected_choice_count_raw) or (actual_choice_count == int(expected_choice_count_raw))
    expected_choice_task_types = {t for t in expected_choice_task_types_raw.split(";") if t}
    # Embedding-ambiguous choices are the top-3 nearest templates, which can
    # reorder/retie with float noise; only hard-check the task_type set for
    # the fully deterministic Ambiguity Gate, record-only otherwise.
    choice_task_types_hard = expected_layer == "Ambiguity Gate"
    choice_task_types_match = (
        True if not expected_choice_task_types else (expected_choice_task_types == set(actual_choice_task_types))
    )

    hard_checks = [layer_match, task_type_match, constraints_match]
    if expected_target and not soft_fields:
        hard_checks.append(target_match)
    if expected_object_query and not soft_fields:
        hard_checks.append(object_query_match)
    if expected_ambiguity_reason:
        hard_checks.append(ambiguity_reason_match)
    if expected_choice_count_raw:
        hard_checks.append(choice_count_match)
    if expected_choice_task_types and choice_task_types_hard:
        hard_checks.append(choice_task_types_match)
    overall_pass = "PASS" if all(hard_checks) else "FAIL"
    if llm_unreachable and expected_layer == "LLM Fallback":
        overall_pass = "SKIPPED"

    result = dict(row)
    result.update(
        {
            "actual_layer": actual_layer,
            "actual_task_type": intent.task_type,
            "actual_target": intent.target or "",
            "actual_object_query": intent.object_query or "",
            "actual_constraints": json.dumps(intent.constraints or {}, ensure_ascii=False),
            "actual_ambiguity_reason": actual_ambiguity_reason,
            "actual_choice_count": actual_choice_count,
            "actual_choice_task_types": ";".join(actual_choice_task_types),
            "actual_choice_labels": " | ".join(actual_choice_labels),
            "layer_match": layer_match,
            "task_type_match": task_type_match,
            "target_match": target_match,
            "object_query_match": object_query_match,
            "constraints_match": constraints_match,
            "ambiguity_reason_match": ambiguity_reason_match,
            "choice_count_match": choice_count_match,
            "overall_pass": overall_pass,
            "route_trace_summary": route_trace_summary(metrics),
        }
    )
    return result


def load_cases(path: str) -> list[dict[str, str]]:
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        return [row for row in reader]


def write_results(path: str, results: list[dict[str, Any]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fieldnames = CSV_FIELDS + RESULT_EXTRA_FIELDS
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in results:
            writer.writerow(row)


def print_summary(results: list[dict[str, Any]], require_llm: bool) -> bool:
    total = len(results)
    by_outcome: dict[str, int] = {}
    for row in results:
        outcome = row["overall_pass"]
        if outcome == "SKIPPED" and require_llm:
            outcome = "FAIL"
            row["overall_pass"] = "FAIL"
        by_outcome[outcome] = by_outcome.get(outcome, 0) + 1

    print(f"\n=== Parser Dry-Run Matrix: {total} cases ===")
    print(f"PASS={by_outcome.get('PASS', 0)} FAIL={by_outcome.get('FAIL', 0)} SKIPPED={by_outcome.get('SKIPPED', 0)}")

    print("\n-- by expected_layer --")
    layer_stats: dict[str, dict[str, int]] = {}
    for row in results:
        layer = row.get("expected_layer") or "(unspecified)"
        layer_stats.setdefault(layer, {"PASS": 0, "FAIL": 0, "SKIPPED": 0})
        layer_stats[layer][row["overall_pass"]] = layer_stats[layer].get(row["overall_pass"], 0) + 1
    for layer, stats in sorted(layer_stats.items()):
        print(f"  {layer:20s} PASS={stats.get('PASS', 0):3d} FAIL={stats.get('FAIL', 0):3d} SKIPPED={stats.get('SKIPPED', 0):3d}")

    print("\n-- by category --")
    cat_stats: dict[str, dict[str, int]] = {}
    for row in results:
        cat = row.get("category") or "(uncategorized)"
        cat_stats.setdefault(cat, {"PASS": 0, "FAIL": 0, "SKIPPED": 0})
        cat_stats[cat][row["overall_pass"]] = cat_stats[cat].get(row["overall_pass"], 0) + 1
    for cat, stats in sorted(cat_stats.items()):
        print(f"  {cat:28s} PASS={stats.get('PASS', 0):3d} FAIL={stats.get('FAIL', 0):3d} SKIPPED={stats.get('SKIPPED', 0):3d}")

    failures = [row for row in results if row["overall_pass"] == "FAIL"]
    if failures:
        print(f"\n-- {len(failures)} failing case(s) --")
        for row in failures:
            print(
                f"  [{row.get('id')}] {row.get('input_text')!r}\n"
                f"      expected: layer={row.get('expected_layer') or '-'} task_type={row.get('expected_task_type') or '-'}"
                f" target={row.get('expected_target') or '-'} object_query={row.get('expected_object_query') or '-'}"
                f" constraints={row.get('expected_constraints') or '-'}\n"
                f"      actual:   layer={row.get('actual_layer')} task_type={row.get('actual_task_type')}"
                f" target={row.get('actual_target') or '-'} object_query={row.get('actual_object_query') or '-'}"
                f" constraints={row.get('actual_constraints')}\n"
                f"      route:    {row.get('route_trace_summary')}"
            )

    return len(failures) == 0


def _repo_root() -> str:
    # .../src/robot_agent/robot_agent/tools/parser_dry_run_matrix.py -> repo root
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


def _default_cases_path() -> str:
    candidates = [
        os.path.join(os.getcwd(), "todo-my/task-agent/dry-run-test/cases-en.csv"),
        os.path.join(_repo_root(), "todo-my/task-agent/dry-run-test/cases-en.csv"),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Dry-run the Deterministic/Embedding/LLM Fallback parser matrix.")
    parser.add_argument("--cases", default=_default_cases_path(), help="Input CSV of test cases.")
    parser.add_argument("--out", default=None, help="Output CSV path (default: debug-output/parser-dry-run/<timestamp>.csv).")
    parser.add_argument("--require-llm", action="store_true", help="Treat LLM-unreachable rows as hard FAIL instead of SKIPPED.")
    parser.add_argument("--config-dir", default=None, help="Override robot_agent config dir (defaults to package share or src config).")
    args = parser.parse_args(argv)

    if not os.path.exists(args.cases):
        print(f"Cases file not found: {args.cases}", file=sys.stderr)
        return 2

    out_path = args.out or os.path.join(
        os.getcwd(), "debug-output", "parser-dry-run", f"parser_dry_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    cases = load_cases(args.cases)
    hybrid_parser = HybridParser(config_dir=args.config_dir)
    results = [evaluate_row(row, hybrid_parser) for row in cases]

    write_results(out_path, results)
    print(f"Wrote {len(results)} result rows to {out_path}")
    all_passed = print_summary(results, require_llm=args.require_llm)

    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
