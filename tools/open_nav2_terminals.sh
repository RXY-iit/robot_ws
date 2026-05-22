#!/usr/bin/env bash
# open_nav2_terminals.sh
# Launch Nav2 navigation in a Terminator split-pane window.
#
# Layout (5 panes):
#   +------------------+------------------+
#   | T1: robot_bringup| T3: checks       |
#   +------------------+------------------+
#   | T2: localization | T4: nav2         |
#   |    + map_server  |   + mode_switch  |
#   +------------------+------------------+
#   |   T5: RViz + PLY publisher          |
#   +-------------------------------------+
#
# Usage:
#   tools/open_nav2_terminals.sh
#   tools/open_nav2_terminals.sh --phase1-safe
#   tools/open_nav2_terminals.sh --map maps/l402_2d_map_lite_0504.yaml
#   tools/open_nav2_terminals.sh --cleanup
#
set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
MAP="${MAP:-$WS/maps/l402_2d_map_clean_0509.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/rviz/nav2_navigation.rviz}"
PLY_MAP="${PLY_MAP:-$WS/maps/saved-map/map-l402-0503/l402_points_0503}"
CLEANUP="${CLEANUP:-false}"
PHASE1_SAFE="${PHASE1_SAFE:-false}"
DEBUG_OUTPUT_ROOT="${DEBUG_OUTPUT_ROOT:-$WS/debug-output}"
DEBUG_SESSION_DIR="${DEBUG_SESSION_DIR:-$DEBUG_OUTPUT_ROOT/nav2_$(date +%Y%m%d_%H%M%S)}"
LAYOUT_NAME="robot_ws_nav2"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Opens a Terminator window with 5 panes for Nav2 navigation:
  T1 (top-left):    ros2 launch robot_bringup test_all.launch.py
  T2 (bot-left):    FAST-LIO2 + GICP localization stack
  T3 (top-right):   localization checks
  T4 (mid-right):   ros2 launch nav_pkg navigation.launch.py
  T5 (bottom):      RViz + PLY map publisher

Options:
  --map PATH       2D map yaml (default: maps/l402_2d_map_0503.yaml)
  --phase1-safe    Start sensors/localization/Nav2/RViz, but do not start
                   drive/steer/serial/lift motors. Uses static odom->base for
                   RViz/Nav2 logic checks before real robot motion.
  --cleanup        Kill old navigation processes first
  -h, --help       Show this help

Environment overrides: WS, MAP, RVIZ_CONFIG, PLY_MAP, PHASE1_SAFE, DEBUG_OUTPUT_ROOT
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)       MAP="$2";      shift 2 ;;
    --phase1-safe|--safe|--no-motors)
                 PHASE1_SAFE=true; shift ;;
    --cleanup)   CLEANUP=true;  shift   ;;
    -h|--help)   usage; exit 0          ;;
    *) echo "Unknown: $1" >&2; usage; exit 2 ;;
  esac
done

# ── sanity checks ─────────────────────────────────────────────────────────────
[[ -f "$WS/install/setup.bash" ]] || { echo "Not built: $WS/install/setup.bash" >&2; exit 1; }
[[ -f "$MAP" ]]                   || { echo "No 2D map yaml: $MAP"          >&2; exit 1; }
[[ -f "$RVIZ_CONFIG" ]]           || { echo "No RViz config: $RVIZ_CONFIG"   >&2; exit 1; }
command -v terminator >/dev/null 2>&1 || { echo "terminator not found"          >&2; exit 1; }
mkdir -p "$DEBUG_SESSION_DIR"

if [[ "$PHASE1_SAFE" == "true" ]]; then
  BRINGUP_ARGS="rviz:=false drive_motors:=false steer_motors:=false serial_motors:=false lift:=false static_odom:=false use_glim_loc:=true"
else
  BRINGUP_ARGS="rviz:=false"
fi

# ── cleanup ───────────────────────────────────────────────────────────────────
if [[ "$CLEANUP" == "true" ]]; then
  echo "[nav2] Stopping old processes..."
  pkill -f "test_all.launch.py"     || true
  pkill -f "navigation.launch.py"   || true
  pkill -f "fake_wheel_odom.py"      || true
  pkill -f "nav2_debug_recorder.py"  || true
  pkill -f "localization.launch.py" || true
  pkill -f "fast_lio_localization"  || true
  pkill -f "fastlio_mapping"        || true
  pkill -f "gicp_localizer_node"    || true
  pkill -f "rviz2"                  || true
  pkill -f "publish_ply"            || true
  pkill -f "interactive_ply_initialpose_publisher.py" || true
  ros2 daemon stop || true
  ros2 daemon start || true
  sleep 2
fi

# ── write per-pane command scripts to /tmp ────────────────────────────────────
# NOTE: These are run directly as the terminal command (no bash -c '...' wrapper)
# to avoid Terminator's ConfigObj parser mishandling single quotes.
# Each script ends with `exec bash` so the pane stays open if the main
# command exits or fails.

cat > /tmp/nav2_T1_bringup.sh << EOF
#!/bin/bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
cd $WS
echo '=== T1: robot_bringup ==='
echo 'phase1_safe: $PHASE1_SAFE'
if [[ "$PHASE1_SAFE" == "true" ]]; then
  echo 'Starting fake stationary /wheel_odom for Phase 1 safe mode...'
  tools/fake_wheel_odom.py &
fi
ros2 launch robot_bringup test_all.launch.py $BRINGUP_ARGS
exec bash
EOF

cat > /tmp/nav2_T2_localization.sh << EOF
#!/bin/bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
cd $WS
echo '=== T2: Phase 2 localization (FAST-LIO2 + GICP + map_server + pointcloud_to_laserscan) ==='
echo "2D map: $MAP"
echo "PLY map: $PLY_MAP"
echo "Waiting 8 sec for bringup..."
sleep 8
ros2 launch localization_pkg fast_lio_localization_live.launch.py \
  map:="$MAP" \
  pcd_map:="$PLY_MAP" \
  with_fast_lio:=true \
  use_fast_lio_hint:=true
exec bash
EOF

cat > /tmp/nav2_T3_checks.sh << EOF
#!/bin/bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
cd $WS
echo '=== T3: Nav2 debug monitor ==='
echo "Waiting 15 sec for localization/Nav2..."
sleep 15
tools/check_nav2_phase1.sh
echo
echo 'Starting debug recorder...'
echo 'debug output: $DEBUG_SESSION_DIR'
tools/nav2_debug_recorder.py --output-dir "$DEBUG_SESSION_DIR" &
echo \$! > "$DEBUG_SESSION_DIR/recorder.pid"
echo
echo 'Starting live monitor. Send a 2D Goal Pose in RViz and watch action/plan/cmd_vel.'
tools/watch_nav2_debug.sh
exec bash
EOF

cat > /tmp/nav2_T3_checks_legacy.sh << EOF
#!/bin/bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
cd $WS
echo '=== T3: localization checks (legacy) ==='
echo "Waiting 12 sec..."
sleep 12
while true; do
  echo
  date
  echo "--- gicp score (lower=better, thr=0.5) ---"
  timeout 2s ros2 topic echo /gicp_loc/score --once 2>/dev/null || echo "no score yet"
  echo "--- FAST-LIO2 odometry ---"
  timeout 2s ros2 topic echo /fast_lio/odometry --once --field header 2>/dev/null || echo "no /fast_lio/odometry"
  echo "--- lifecycle ---"
  ros2 lifecycle get /map_server 2>/dev/null || true
  echo "--- /scan ---"
  timeout 2s ros2 topic echo /scan --once --field header 2>/dev/null || echo "no /scan"
  echo "--- TF map→odom ---"
  timeout 3s ros2 run tf2_ros tf2_echo map odom 2>/dev/null || echo "no map→odom TF"
  timeout 2s ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null || true
  echo "--- Nav2 mode / planner / cmd_vel ---"
  timeout 2s ros2 topic echo /robot_mode --once 2>/dev/null || echo "no /robot_mode"
  timeout 2s ros2 topic echo /plan --once --field header 2>/dev/null || echo "no /plan yet"
  timeout 2s ros2 topic echo /nav2/cmd_vel --once 2>/dev/null || echo "no /nav2/cmd_vel yet"
  timeout 2s ros2 topic echo /cmd_vel --once 2>/dev/null || echo "no /cmd_vel yet"
  echo "--- /cmd_vel publishers (Nav2 behavior_server must NOT be here) ---"
  ros2 topic info /cmd_vel 2>/dev/null || true
  echo "--- costmap topics ---"
  ros2 topic list 2>/dev/null | grep -E "costmap|/plan|/local_plan" || true
  echo "--- Nav2 lifecycle ---"
  ros2 lifecycle get /controller_server 2>/dev/null || true
  ros2 lifecycle get /planner_server 2>/dev/null || true
  sleep 5
done
exec bash
EOF

cat > /tmp/nav2_T4_navigation.sh << EOF
#!/bin/bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
cd $WS
echo '=== T4: Nav2 navigation + mode switch ==='
echo "Waiting 12 sec for localization..."
sleep 12
ros2 launch nav_pkg navigation.launch.py
exec bash
EOF

cat > /tmp/nav2_T5_rviz.sh << EOF
#!/bin/bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
cd $WS
echo '=== T5: RViz + PLY publisher ==='
echo "Waiting 10 sec..."
sleep 10

if [[ -f "$PLY_MAP" ]]; then
  echo "Starting PLY publisher: $PLY_MAP"
  tools/publish_l402_overlay.sh live &
fi

echo "Opening RViz..."
rviz2 -d "$RVIZ_CONFIG"
exec bash
EOF

chmod +x /tmp/nav2_T*.sh

# ── inject layout into Terminator config ──────────────────────────────────────
# command fields use plain script paths — no quotes — to avoid ConfigObj
# misinterpreting single quotes as string delimiters.
python3 << PYEOF
import re, sys
from pathlib import Path

config_path = "$HOME/.config/terminator/config"
layout_name = "$LAYOUT_NAME"

new_layout = """
  [[{name}]]
    [[[window0]]]
      type = Window
      parent = ""
      order = 0
      size = 1680, 1000
      maximised = True
    [[[h_main]]]
      type = HPaned
      parent = window0
      order = 0
      ratio = 0.5
    [[[left_pane]]]
      type = VPaned
      parent = h_main
      order = 0
      ratio = 0.5
    [[[terminal_bringup]]]
      type = Terminal
      parent = left_pane
      order = 0
      profile = default
      command = /tmp/nav2_T1_bringup.sh
      directory = {ws}
    [[[terminal_localization]]]
      type = Terminal
      parent = left_pane
      order = 1
      profile = default
      command = /tmp/nav2_T2_localization.sh
      directory = {ws}
    [[[right_pane]]]
      type = VPaned
      parent = h_main
      order = 1
      ratio = 0.34
    [[[terminal_checks]]]
      type = Terminal
      parent = right_pane
      order = 0
      profile = default
      command = /tmp/nav2_T3_checks.sh
      directory = {ws}
    [[[right_bottom_pane]]]
      type = VPaned
      parent = right_pane
      order = 1
      ratio = 0.5
    [[[terminal_nav2]]]
      type = Terminal
      parent = right_bottom_pane
      order = 0
      profile = default
      command = /tmp/nav2_T4_navigation.sh
      directory = {ws}
    [[[terminal_rviz]]]
      type = Terminal
      parent = right_bottom_pane
      order = 1
      profile = default
      command = /tmp/nav2_T5_rviz.sh
      directory = {ws}
""".format(name=layout_name, ws="$WS")

try:
    with open(config_path, "r") as f:
        content = f.read()
except FileNotFoundError:
    content = ""

# Remove existing layout with this name if present
pattern = r"\n  \[\[" + re.escape(layout_name) + r"\]\].*?(?=\n  \[\[|\Z)"
content = re.sub(pattern, "", content, flags=re.DOTALL)

# Add new layout inside [layouts] section
if "[layouts]" in content:
    content = content.replace("[layouts]", "[layouts]" + new_layout, 1)
else:
    content += "\n[layouts]" + new_layout

Path(config_path).parent.mkdir(parents=True, exist_ok=True)
with open(config_path, "w") as f:
    f.write(content)

print(f"Layout '{layout_name}' written to {config_path}")
PYEOF

echo ""
echo "[nav2] Starting Terminator with layout '$LAYOUT_NAME'..."
echo "  phase1_safe:     $PHASE1_SAFE"
echo "  debug_output:    $DEBUG_SESSION_DIR"
echo "  T1 (top-left):    robot_bringup test_all.launch.py"
echo "  T2 (bot-left):    FAST-LIO2 + GICP localization  (starts after 8 sec)"
echo "  T3 (top-right):   GICP checks       (starts after 12 sec)"
echo "  T4 (mid-right):   Nav2              (starts after 12 sec)"
echo "  T5 (bot-right):   RViz + PLY        (starts after 10 sec)"
echo ""
echo "Nav2 operation:"
if [[ "$PHASE1_SAFE" == "true" ]]; then
  echo "  Phase1 safe mode: real movement motors are NOT launched."
  echo "  1. RViz: check TF/map/costmaps/path."
  echo "  2. Send a nearby 2D Goal Pose and confirm /nav2/cmd_vel is produced."
  echo "  3. Confirm /cmd_vel is harmless because cmd_vel_to_motor_node is not running."
else
  echo "  1. RViz: click '2D Pose Estimate' to initialize GICP if needed"
  echo "  2. Press Button A on Joy-Con -> AUTO mode"
  echo "  3. RViz: click '2D Goal Pose' -> robot navigates"
  echo "  4. Press Button B -> MANUAL, robot stops immediately"
fi
echo ""

# --no-dbus is important when another Terminator instance is already running:
# otherwise this process only asks the existing DBus master for a new window,
# and the requested layout can be ignored or opened as the default layout.
terminator --no-dbus -l "$LAYOUT_NAME" &
