#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
URDF_XACRO="${WS_DIR}/src/robot_description/urdf/robot.urdf.xacro"
TMP_URDF="/tmp/robot_description_check.urdf"
TMP_LAUNCH="/tmp/robot_urdf_tf_check.launch.py"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_cli_logs}"
mkdir -p "${ROS_LOG_DIR}"

DO_BUILD=1
DO_TF=1
DO_RVIZ=0
DO_BRINGUP=0

usage() {
  printf '%s\n' \
    "Usage: tools/check_robot_urdf.sh [options]" \
    "" \
    "Options:" \
    "  --no-build     Skip colcon build" \
    "  --no-tf        Skip automatic TF echo checks" \
    "  --rviz         Launch RViz robot model viewer after checks" \
    "  --bringup      Launch full robot bringup after checks" \
    "  -h, --help     Show this help" \
    "" \
    "Default checks:" \
    "  1. build robot_description and robot_bringup" \
    "  2. source install/setup.bash" \
    "  3. expand robot.urdf.xacro" \
    "  4. run check_urdf when installed" \
    "  5. start robot_state_publisher and echo sensor TFs"
}

while (($#)); do
  case "$1" in
    --no-build)
      DO_BUILD=0
      ;;
    --no-tf)
      DO_TF=0
      ;;
    --rviz)
      DO_RVIZ=1
      ;;
    --bringup)
      DO_BRINGUP=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

step() {
  printf '\n==> %s\n' "$1"
}

check_tf() {
  local parent="$1"
  local child="$2"
  local output
  local status

  set +e
  output="$(timeout 5 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1)"
  status=$?
  set -e

  printf '%s\n' "${output}"

  if grep -q 'Translation:' <<<"${output}"; then
    return 0
  fi

  printf 'Could not read %s -> %s TF. tf2_echo exit code: %s\n' "${parent}" "${child}" "${status}" >&2
  printf 'See /tmp/robot_urdf_tf_check.log\n' >&2
  return 1
}

need_file() {
  if [[ ! -f "$1" ]]; then
    printf 'Missing required file: %s\n' "$1" >&2
    exit 1
  fi
}

source_ros() {
  set +u
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  fi
  set -u
}

source_workspace() {
  set +u
  if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_DIR}/install/setup.bash"
  fi
  set -u
}

cleanup() {
  if [[ -n "${RSP_PID:-}" ]] && kill -0 "${RSP_PID}" 2>/dev/null; then
    kill "${RSP_PID}" 2>/dev/null || true
    wait "${RSP_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

stop_tf_check_publisher() {
  if [[ -n "${RSP_PID:-}" ]] && kill -0 "${RSP_PID}" 2>/dev/null; then
    kill "${RSP_PID}" 2>/dev/null || true
    wait "${RSP_PID}" 2>/dev/null || true
    unset RSP_PID
  fi
}

need_file "${URDF_XACRO}"

step "Source ROS environment"
source_ros

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'ros2 command not found. Source your ROS setup first, or install ROS 2.\n' >&2
  exit 1
fi

if [[ "${DO_BUILD}" -eq 1 ]]; then
  step "Build robot description packages"
  cd "${WS_DIR}"
  colcon build --packages-select robot_description robot_bringup
fi

step "Source workspace"
source_workspace

step "Expand xacro to URDF"
ros2 run xacro xacro "${URDF_XACRO}" > "${TMP_URDF}"
printf 'Generated: %s\n' "${TMP_URDF}"

if command -v check_urdf >/dev/null 2>&1; then
  step "Validate URDF syntax"
  check_urdf "${TMP_URDF}"
else
  step "Validate URDF syntax"
  printf 'check_urdf not found; xacro expansion succeeded, so XML generation is OK.\n'
fi

step "Show current sensor origins from xacro"
grep -E 'name="(livox|camera)_[xyz]"|name="camera_pitch"' "${URDF_XACRO}" || true

if [[ "${DO_TF}" -eq 1 ]]; then
  step "Start robot_state_publisher for TF checks"
  cat > "${TMP_LAUNCH}" <<'PY'
import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.environ["ROBOT_URDF_XACRO"]
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    }
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher_urdf_check",
            output="screen",
            parameters=[robot_description],
        )
    ])
PY

  ROBOT_URDF_XACRO="${URDF_XACRO}" ros2 launch "${TMP_LAUNCH}" >/tmp/robot_urdf_tf_check.log 2>&1 &
  RSP_PID=$!
  sleep 3

  step "TF: base_link -> livox_frame"
  check_tf base_link livox_frame

  step "TF: base_link -> camera_link"
  check_tf base_link camera_link

  stop_tf_check_publisher
fi

if [[ "${DO_RVIZ}" -eq 1 ]]; then
  step "Launch RViz robot model viewer"
  ros2 launch robot_description display.launch.py
fi

if [[ "${DO_BRINGUP}" -eq 1 ]]; then
  step "Launch full robot bringup"
  ros2 launch robot_bringup bringup.launch.py
fi

step "Done"
printf '%s\n' \
  "URDF checks completed." \
  "For visual check: tools/check_robot_urdf.sh --no-build --rviz" \
  "For real sensors: tools/check_robot_urdf.sh --no-build --bringup"
