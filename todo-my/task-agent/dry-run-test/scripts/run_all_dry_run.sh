#!/usr/bin/env bash
# Run all dry-run matrices. Each one is independently runnable:
#   scripts/run_all_dry_run.sh                 # everything
#   scripts/run_all_dry_run.sh parser_warehouse_jp   # just one target (no .csv)
#   scripts/run_all_dry_run.sh perception_capability_real_lab
set -eo pipefail
cd "$(git rev-parse --show-toplevel)"
source install/setup.bash

TS=$(date +%Y%m%d_%H%M%S)
CASES_DIR="todo-my/task-agent/dry-run-test/cases"
RESULTS_DIR="todo-my/task-agent/dry-run-test/results"
mkdir -p "$RESULTS_DIR"

parser_cases=(parser_real_lab_en parser_real_lab_jp parser_warehouse_en parser_warehouse_jp)
# The perception planner never reads ROBOT_MAP_ID at all -- capability depends
# only on perception_targets.yaml / the loaded YOLO model, not on which map is
# active. This one case file is run under BOTH map bundles on purpose: the
# point is to prove the results are byte-identical regardless of scenario, not
# to test two different vocabularies.
perception_targets=(perception_capability_real_lab perception_capability_warehouse)

targets=("$@")
if [ ${#targets[@]} -eq 0 ]; then
  targets=("${parser_cases[@]}" "${perception_targets[@]}")
fi

status=0
for name in "${targets[@]}"; do
  is_parser=false
  for p in "${parser_cases[@]}"; do
    [ "$p" = "$name" ] && is_parser=true
  done
  echo "==== $name ===="
  if $is_parser; then
    # The deterministic parser uses the current map bundle's known-target list as
    # a disambiguation priority signal, so each scenario's matrix must run against
    # its own matching map bundle, not whatever ROBOT_MAP_ID happens to be set to.
    case "$name" in
      *real_lab*) export ROBOT_MAP_ID=sim_real_lab_001 ;;
      *warehouse*) export ROBOT_MAP_ID=warehouse ;;
    esac
    ros2 run robot_agent parser_dry_run_matrix \
      --cases "$CASES_DIR/$name.csv" \
      --out "$RESULTS_DIR/${name}_$TS.csv" || status=1
  else
    case "$name" in
      *real_lab*) export ROBOT_MAP_ID=sim_real_lab_001 ;;
      *warehouse*) export ROBOT_MAP_ID=warehouse ;;
    esac
    ros2 run robot_skills perception_dry_run_matrix \
      --cases "$CASES_DIR/perception_capability.csv" \
      --out "$RESULTS_DIR/${name}_$TS.csv" || status=1
  fi
  echo
done
exit $status
