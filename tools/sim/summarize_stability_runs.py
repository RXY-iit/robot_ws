#!/usr/bin/env python3
"""Build evaluation manifests referencing canonical mission reports, offline."""
import argparse
import collections
import datetime
import hashlib
import json
from pathlib import Path
import subprocess


def summarize(root, workspace):
    template = json.loads((workspace / "todo-my/jev-agent/test&pro&solve/run-template.json").read_text())
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=workspace, text=True).strip()
    source_paths = [
        "src/robot_skills/robot_skills/executors/navigation.py",
        "src/robot_skills/robot_skills/executors/perception.py",
        "src/robot_skills/robot_skills/executors/base.py",
        "src/robot_skills/robot_skills/motion_clock.py",
        "src/robot_skills/robot_skills/motion_evidence.py",
        "src/robot_agent/robot_agent/agents/mission_agent.py",
        "src/robot_agent/robot_agent/parsers/deterministic_parser.py",
        "src/robot_agent/robot_agent/parsers/hybrid_parser.py",
        "src/robot_agent/robot_agent/jev_agent.py",
        "src/robot_agent/robot_agent/human_report.py",
        "src/robot_agent/robot_agent/nodes/health_monitor.py",
        "src/robot_sim/robot_sim/pointcloud2_to_livox_custom_msg_node.py",
        "src/localization_pkg/scripts/gicp_localizer_node.py",
        "src/localization_pkg/config/gicp_localizer.yaml",
        "src/fast_lio/src/laserMapping.cpp",
        "src/robot_sim/config/sim_omni_drive.yaml",
        "src/robot_agent/config/recovery/playbooks/PB-LOC-LOST.yaml",
    ]
    fingerprint_path = root / "final-source-fingerprints.json"
    fingerprint_path.write_text(json.dumps({
        "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "git_head": revision,
        "scope": "Final dirty-worktree snapshot, NOT each earlier run's frozen revision",
        "sha256": {p: hashlib.sha256((workspace / p).read_bytes()).hexdigest() for p in source_paths},
    }, indent=2) + "\n")
    rows = []
    for path in sorted(root.glob("*.stdout.txt")):
        records = [json.loads(line[7:]) for line in path.read_text().splitlines() if line.startswith("RESULT ")]
        if not records:
            continue
        result = records[-1]
        mission_id = result["mission_id"]
        report_paths = list((workspace / "debug-output/sessions").glob(f"*/missions/{mission_id}/human-report-*.json"))
        manifest = json.loads(json.dumps(template))
        manifest.update(run_id=path.name.removesuffix(".stdout.txt"), case_id=path.name.split("-")[0],
                        execution_status="EXECUTED")
        manifest["revision"]["git_head"] = revision
        manifest["revision"]["config_revision"] = "Iterative dirty worktree; exact per-run frozen revision unavailable, excluded from A/B"
        manifest["revision"]["final_source_fingerprints"] = str(fingerprint_path.relative_to(workspace))
        manifest["evidence_refs"]["mission_id"] = mission_id
        observation_path = path.with_name(path.name.removesuffix(".stdout.txt") + ".jsonl")
        manifest["evidence_refs"]["recovery_logs"] = [str(path.relative_to(workspace)), str(observation_path.relative_to(workspace))]
        manifest["baseline"]["snapshot_path"] = str(observation_path.relative_to(workspace)) + "#before-command"
        manifest["round_id"] = root.name
        manifest["evaluation"]["time_contract"] = {"sensor_stamps": "ROS simulation time", "watchdogs": "monotonic wall time", "events": "UTC"}
        manifest["expected"]["terminal_state"] = "CANCELLED" if path.name.startswith("M14-") else "DONE"
        if path.name.startswith("M07-approach-"):
            manifest["expected"]["failure_continuation_policy"] = "Completed OCR with no matching 06 may ask human; do not count as successful visual confirmation"
        manifest["evaluation"].update(outcome=result["status"], comparison_eligible=False,
            exclusion_reason="No independent ground truth or identical-state reset; diagnostic smoke test only",
            ground_truth_source="wheel_odom feedback, not independent simulation truth")
        if report_paths:
            report_path = max(report_paths, key=lambda p: p.stat().st_mtime)
            report = json.loads(report_path.read_text())
            manifest["session_id"] = report_path.parents[2].name
            manifest["evidence_refs"]["human_report"].update(path=str(report_path.relative_to(workspace)),
                sha256=hashlib.sha256(report_path.read_bytes()).hexdigest(),
                report_id=report.get("report_id"), updated_at=report.get("updated_at"))
            manifest["input"]["user_command"] = report.get("mission", {}).get("user_text")
            manifest["input"]["agent_mode"] = report.get("mission", {}).get("agent_mode")
            jst = datetime.timezone(datetime.timedelta(hours=9))
            for field, source in (("started_at_jst", "created_at"), ("ended_at_jst", "updated_at")):
                if report.get(source):
                    manifest[field] = datetime.datetime.fromisoformat(report[source]).astimezone(jst).isoformat()
            manifest["evidence_refs"]["skill_events"]["path"] = str(report_path.parents[2].relative_to(workspace) / "agent/skill_events.jsonl")
        metrics = []
        for skill in result.get("results") or []:
            if skill.get("skill_name") not in {"move_relative", "rotate_relative", "goto_pose"}:
                continue
            m = skill.get("metrics") or {}
            metrics.append({"skill": skill["skill_name"], "success": skill.get("success"),
                            **{k: m.get(k) for k in ("completion_class", "duration_sec", "timeout_reason",
                                "position_error_m", "cross_track_m", "signed_accumulated_yaw_rad",
                                "final_yaw_error_rad", "final_xy_error_m")}})
        manifest["evaluation"]["measurements"]["motion"] = metrics
        manifest["derived_metrics"]["source_fingerprint"] = hashlib.sha256(path.read_bytes()).hexdigest()
        out = root / "evaluations"
        out.mkdir(exist_ok=True)
        (out / (manifest["run_id"] + ".json")).write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
        rows.append({"run_id": manifest["run_id"], "mission_id": mission_id, "status": result["status"], "motion": metrics})
    summary = {"runs": rows, "status_counts": dict(collections.Counter(r["status"] for r in rows)),
               "note": "Diagnostic results, not an A/B success-rate claim"}
    (root / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    summarize(args.root.resolve(), Path(__file__).resolve().parents[2])
