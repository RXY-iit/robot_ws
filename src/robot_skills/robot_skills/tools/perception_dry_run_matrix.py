from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from datetime import datetime
from typing import Any

from robot_skills.executors.perception import PerceptionSkills

CSV_FIELDS = [
    "id",
    "category",
    "query",
    "expected_type",
    "expected_colors",
    "expected_objects",
    "expected_text_kind",
    "expected_yolo_classes",
    "expected_detectors",
    "expected_unsupported_reason_count",
    "expected_forces_needs_human",
    "notes",
]

RESULT_EXTRA_FIELDS = [
    "actual_type",
    "actual_colors",
    "actual_objects",
    "actual_text_kind",
    "actual_yolo_classes",
    "actual_fallback_classes",
    "actual_fallback_requires_human",
    "actual_detectors",
    "actual_unsupported_reasons",
    "actual_forces_needs_human",
    "type_match",
    "colors_match",
    "objects_match",
    "text_kind_match",
    "detectors_match",
    "overall_pass",
]


def detectors_for_plan(plan: dict[str, Any]) -> list[str]:
    """Mirrors PerceptionSkills._run_detection_plan's branching so this dry
    run can characterize which detector(s) a query WOULD invoke, without
    actually running YOLO/OCR/HSV against real captured images.
    """
    plan_type = plan.get("type")
    if plan_type in ("abstract", "unsupported"):
        return []
    detectors: list[str] = []
    if plan_type in ("color", "composite") and plan.get("colors"):
        detectors.append("hsv_color_region")
    if plan_type in ("text", "composite") and plan.get("text_kind"):
        detectors.append("ocr")
    if plan_type in ("object", "composite") and plan.get("objects"):
        detectors.append("yolo")
    if plan_type == "composite":
        detectors.append("composite_overlap")
    return detectors


def _semicolon(values: list[str] | None) -> str:
    return ";".join(str(v) for v in (values or []))


def evaluate_row(row: dict[str, str], analyzer: PerceptionSkills) -> dict[str, Any]:
    plan = analyzer._build_detection_plan(row["query"])  # noqa: SLF001 - intentional white-box test of the planner

    actual_type = str(plan.get("type") or "")
    actual_colors = plan.get("colors") or []
    actual_objects = plan.get("objects") or []
    actual_text_kind = str(plan.get("text_kind") or "")
    actual_yolo_classes = plan.get("yolo_classes") or []
    actual_fallback_classes = plan.get("fallback_classes") or []
    actual_fallback_requires_human = bool(plan.get("fallback_requires_human"))
    actual_unsupported = plan.get("unsupported") or []
    actual_detectors = detectors_for_plan(plan)
    actual_forces_needs_human = actual_type in ("abstract", "unsupported")

    expected_type = (row.get("expected_type") or "").strip()
    expected_colors = {c for c in (row.get("expected_colors") or "").split(";") if c}
    expected_objects = {o for o in (row.get("expected_objects") or "").split(";") if o}
    expected_text_kind = (row.get("expected_text_kind") or "").strip()
    expected_detectors = {d for d in (row.get("expected_detectors") or "").split(";") if d}
    expected_unsupported_count_raw = (row.get("expected_unsupported_reason_count") or "").strip()
    expected_forces_needs_human_raw = (row.get("expected_forces_needs_human") or "").strip().lower()

    type_match = (not expected_type) or (actual_type == expected_type)
    colors_match = (not expected_colors) or (expected_colors == set(actual_colors))
    objects_match = (not expected_objects) or (expected_objects == set(actual_objects))
    text_kind_match = (not expected_text_kind) or (actual_text_kind == expected_text_kind)
    detectors_match = (not expected_detectors) or (expected_detectors == set(actual_detectors))
    unsupported_count_match = (
        True if not expected_unsupported_count_raw else (len(actual_unsupported) == int(expected_unsupported_count_raw))
    )
    forces_needs_human_match = (
        True
        if not expected_forces_needs_human_raw
        else (actual_forces_needs_human == (expected_forces_needs_human_raw == "true"))
    )

    hard_checks = [
        type_match,
        colors_match,
        objects_match,
        text_kind_match,
        detectors_match,
        unsupported_count_match,
        forces_needs_human_match,
    ]
    overall_pass = "PASS" if all(hard_checks) else "FAIL"

    result = dict(row)
    result.update(
        {
            "actual_type": actual_type,
            "actual_colors": _semicolon(actual_colors),
            "actual_objects": _semicolon(actual_objects),
            "actual_text_kind": actual_text_kind,
            "actual_yolo_classes": _semicolon(actual_yolo_classes),
            "actual_fallback_classes": _semicolon(actual_fallback_classes),
            "actual_fallback_requires_human": actual_fallback_requires_human,
            "actual_detectors": _semicolon(actual_detectors),
            "actual_unsupported_reasons": " | ".join(actual_unsupported),
            "actual_forces_needs_human": actual_forces_needs_human,
            "type_match": type_match,
            "colors_match": colors_match,
            "objects_match": objects_match,
            "text_kind_match": text_kind_match,
            "detectors_match": detectors_match,
            "overall_pass": overall_pass,
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


def print_summary(results: list[dict[str, Any]]) -> bool:
    total = len(results)
    passed = sum(1 for r in results if r["overall_pass"] == "PASS")
    failed = total - passed
    print(f"\n=== Perception Query Planner Dry-Run: {total} cases ===")
    print(f"PASS={passed} FAIL={failed}")

    print("\n-- by expected_type --")
    type_stats: dict[str, dict[str, int]] = {}
    for row in results:
        t = row.get("expected_type") or "(unspecified)"
        type_stats.setdefault(t, {"PASS": 0, "FAIL": 0})
        type_stats[t][row["overall_pass"]] = type_stats[t].get(row["overall_pass"], 0) + 1
    for t, stats in sorted(type_stats.items()):
        print(f"  {t:15s} PASS={stats.get('PASS', 0):3d} FAIL={stats.get('FAIL', 0):3d}")

    print("\n-- by category --")
    cat_stats: dict[str, dict[str, int]] = {}
    for row in results:
        cat = row.get("category") or "(uncategorized)"
        cat_stats.setdefault(cat, {"PASS": 0, "FAIL": 0})
        cat_stats[cat][row["overall_pass"]] = cat_stats[cat].get(row["overall_pass"], 0) + 1
    for cat, stats in sorted(cat_stats.items()):
        print(f"  {cat:28s} PASS={stats.get('PASS', 0):3d} FAIL={stats.get('FAIL', 0):3d}")

    failures = [row for row in results if row["overall_pass"] == "FAIL"]
    if failures:
        print(f"\n-- {len(failures)} failing case(s) --")
        for row in failures:
            print(
                f"  [{row.get('id')}] {row.get('query')!r}\n"
                f"      expected: type={row.get('expected_type') or '-'} colors={row.get('expected_colors') or '-'}"
                f" objects={row.get('expected_objects') or '-'} text_kind={row.get('expected_text_kind') or '-'}"
                f" detectors={row.get('expected_detectors') or '-'}\n"
                f"      actual:   type={row.get('actual_type')} colors={row.get('actual_colors') or '-'}"
                f" objects={row.get('actual_objects') or '-'} text_kind={row.get('actual_text_kind') or '-'}"
                f" detectors={row.get('actual_detectors') or '-'}"
            )

    return len(failures) == 0


def _repo_root() -> str:
    # .../src/robot_skills/robot_skills/tools/perception_dry_run_matrix.py -> repo root
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


def _default_cases_path() -> str:
    candidates = [
        os.path.join(os.getcwd(), "todo-my/task-agent/dry-run-test/perception/cases-perception.csv"),
        os.path.join(_repo_root(), "todo-my/task-agent/dry-run-test/perception/cases-perception.csv"),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Dry-run the perception query-planner primitive decomposition/detector routing logic.")
    parser.add_argument("--cases", default=_default_cases_path(), help="Input CSV of test cases.")
    parser.add_argument("--out", default=None, help="Output CSV path (default: debug-output/perception-dry-run/<timestamp>.csv).")
    args = parser.parse_args(argv)

    if not os.path.exists(args.cases):
        print(f"Cases file not found: {args.cases}", file=sys.stderr)
        return 2

    out_path = args.out or os.path.join(
        os.getcwd(), "debug-output", "perception-dry-run", f"perception_dry_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    cases = load_cases(args.cases)
    analyzer = PerceptionSkills(node=None)
    results = [evaluate_row(row, analyzer) for row in cases]

    write_results(out_path, results)
    print(f"Wrote {len(results)} result rows to {out_path}")
    all_passed = print_summary(results)

    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
