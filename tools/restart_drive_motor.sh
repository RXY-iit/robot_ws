#!/usr/bin/env bash

set -euo pipefail

WS="${ROBOT_WS:-/home/matsunaga-h/robot_ws}"
DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
DRIVE_SERIAL_DEVICE="${DRIVE_SERIAL_DEVICE:-/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FT7E9M3C-if00-port0}"

if [[ "${1:-}" != "--confirmed-stationary" ]]; then
  echo "Refusing drive restart without --confirmed-stationary." >&2
  echo "Press B, verify the robot and /cmd_vel are stopped, then retry." >&2
  exit 2
fi
if [[ ! -e "$DRIVE_SERIAL_DEVICE" ]]; then
  echo "$DRIVE_SERIAL_DEVICE is missing; reconnect motor USB/power before restart." >&2
  exit 3
fi

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROBOT_WS="$WS"

RUN_DIR="$WS/debug-output/real-robot/drive-restart-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
new_processes_started=false
restart_succeeded=false

cleanup_failed_restart() {
  if [[ "$new_processes_started" == true && "$restart_succeeded" != true ]]; then
    echo "Restart failed; stopping partially started drive processes." >&2
    pkill -INT -f '/BLV_R/drive_motor.py' 2>/dev/null || true
    pkill -INT -f 'om_modbusRTU_node' 2>/dev/null || true
  fi
}
trap cleanup_failed_restart EXIT

echo "Stopping old drive_motor and Modbus processes..."
pkill -INT -f '/BLV_R/drive_motor.py' 2>/dev/null || true
pkill -INT -f 'om_modbusRTU_node' 2>/dev/null || true

for _ in {1..30}; do
  if ! pgrep -f '/BLV_R/drive_motor.py|om_modbusRTU_node' >/dev/null; then
    break
  fi
  sleep 0.1
done
if pgrep -f '/BLV_R/drive_motor.py|om_modbusRTU_node' >/dev/null; then
  echo "Old drive processes did not stop; use full --cleanup instead of forcing them." >&2
  exit 4
fi
if fuser "$DRIVE_SERIAL_DEVICE" >/dev/null 2>&1; then
  echo "$DRIVE_SERIAL_DEVICE is still occupied; refusing to start a duplicate driver." >&2
  fuser -v "$DRIVE_SERIAL_DEVICE" || true
  exit 5
fi

echo "Starting Modbus... log=$RUN_DIR/modbus.log"
nohup ros2 launch om_modbus_master om_modbus_master_launch.py \
  com:="$DRIVE_SERIAL_DEVICE" topicID:=0 baudrate:=230400 updateRate:=10 \
  secondGen:="1,2,3" globalID:=10 axisNum:=3 \
  >"$RUN_DIR/modbus.log" 2>&1 </dev/null &
echo $! >"$RUN_DIR/modbus.pid"
new_processes_started=true

modbus_ready=false
modbus_deadline=$((SECONDS + 10))
while (( SECONDS < modbus_deadline )); do
  if timeout 1 ros2 topic list 2>/dev/null | grep -qx '/om_state0'; then
    modbus_ready=true
    break
  fi
  sleep 0.2
done
if [[ "$modbus_ready" != true ]]; then
  echo "Modbus process started but /om_state0 was not advertised within 10 seconds." >&2
  tail -20 "$RUN_DIR/modbus.log" || true
  exit 6
fi

# The topic may appear slightly before the parameter services used by
# drive_motor.py are ready. Starting too early can stall its initialization.
param_service_ready=false
param_service_deadline=$((SECONDS + 10))
while (( SECONDS < param_service_deadline )); do
  if timeout 1 ros2 service list 2>/dev/null | grep -qx '/om_node/set_parameters'; then
    param_service_ready=true
    break
  fi
  sleep 0.2
done
if [[ "$param_service_ready" != true ]]; then
  echo "Modbus topic is visible, but /om_node/set_parameters was not ready within 10 seconds." >&2
  tail -30 "$RUN_DIR/modbus.log" || true
  exit 6
fi
sleep 0.5

echo "Starting drive_motor... log=$RUN_DIR/drive_motor.log"
nohup env PYTHONUNBUFFERED=1 /usr/bin/python3 \
  "$WS/src/om_modbus_master_V201/om_modbus_master/sample/BLV_R/drive_motor.py" \
  >"$RUN_DIR/drive_motor.log" 2>&1 </dev/null &
drive_pid=$!
echo "$drive_pid" >"$RUN_DIR/drive_motor.pid"

if ! timeout 25 ros2 topic echo /drive_odom --once >/dev/null 2>&1; then
  echo "drive_motor started but /drive_odom did not recover within 25 seconds." >&2
  if [[ ! -e "$DRIVE_SERIAL_DEVICE" ]]; then
    echo "CAUSE: $DRIVE_SERIAL_DEVICE disappeared during restart; check USB hub/cable/power." >&2
  elif ! kill -0 "$drive_pid" 2>/dev/null; then
    echo "CAUSE: drive_motor.py exited during initialization." >&2
  elif grep -Eq 'No response|Error checks don.t match|Exception response' "$RUN_DIR/modbus.log"; then
    echo "CAUSE: Modbus is running, but BLV-R returned a communication error." >&2
  else
    echo "CAUSE: USB serial is open, but BLV-R feedback never arrived." >&2
    echo "Check that every PWR/SYS LED is white, then check RS-485 A/B/GND, IDs 1/2/3 and 230400 baud." >&2
  fi
  echo "--- modbus.log (tail) ---" >&2
  tail -30 "$RUN_DIR/modbus.log" >&2 || true
  echo "--- drive_motor.log (tail) ---" >&2
  tail -30 "$RUN_DIR/drive_motor.log" >&2 || true
  exit 7
fi

# A setup write response also causes drive_motor.py to publish /drive_odom, so
# the first sample does not prove that all excitation and ID-share setup steps
# finished. Wait for direct health evidence that runtime queries have started
# and at least one real read response has arrived. Do not gate on a one-shot
# `ros2 topic info` publisher count: DDS discovery can transiently report zero
# while the local process and transaction stream are healthy.
runtime_ready=false
runtime_deadline=$((SECONDS + 15))
while (( SECONDS < runtime_deadline )); do
  if ! kill -0 "$(cat "$RUN_DIR/modbus.pid")" 2>/dev/null; then
    break
  fi
  health_snapshot="$(timeout 1 ros2 topic echo /drive/health_summary --once --field data 2>/dev/null || true)"
  if grep -Eq '"runtime_query_count": [1-9][0-9]*' <<<"$health_snapshot" && \
     ! grep -q '"last_read_response_age_sec": null' <<<"$health_snapshot"; then
    runtime_ready=true
    break
  fi
  sleep 0.2
done
if [[ "$runtime_ready" != true ]]; then
  echo "Drive setup produced feedback, but no verified runtime read response arrived." >&2
  tail -40 "$RUN_DIR/modbus.log" >&2 || true
  tail -40 "$RUN_DIR/drive_motor.log" >&2 || true
  exit 7
fi

if ! timeout 4 ros2 topic echo /om_response0 --once >/dev/null 2>&1; then
  echo "Runtime was reached, but no new /om_response0 arrived within 4 seconds." >&2
  tail -30 "$RUN_DIR/modbus.log" >&2 || true
  exit 7
fi

restart_succeeded=true
echo "Drive motor restart succeeded; /drive_odom feedback is active."
