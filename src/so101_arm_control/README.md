# SO101 Arm Step B baseline

This package is the Step B ROS 2/RViz/Dashboard baseline for the local 12 V
SO-ARM101 follower.

## Safety boundary

- The default launch is `MOCK` only.
- `arm_mock_dashboard.launch.py` contains no serial port, Feetech plugin, motor
  register, torque, or real-hardware branch.
- The calibration ID and SHA-256 are displayed as evidence, but no calibration
  values are written to a motor.
- Slider changes only edit a candidate. Execution requires a successful preview
  and the matching `candidate_id`.
- Real presets are operator-captured and remain subject to guarded Preview before
  execution; ObserveRegion and PressButton triggers remain `BLOCKED`.

## Start

```bash
source /home/matsunaga-h/robot_ws/install/setup.bash
ros2 launch so101_arm_control arm_mock_dashboard.launch.py
```

This starts:

- RViz with cyan actual and orange translucent candidate models;
- Dashboard at `http://127.0.0.1:8765/arm`;
- `/joint_states`;
- `/arm_controller/controller_state`;
- `/arm_controller/follow_joint_trajectory`;
- `/arm/dashboard_command` and `/arm/dashboard_state`;
- `/display_planned_path` for later MoveIt integration.

Headless launch:

```bash
ros2 launch so101_arm_control arm_mock_dashboard.launch.py rviz:=false dashboard:=false
```

## Smoke tests

With the headless graph running:

```bash
ros2 run so101_arm_control mock_smoke_test
```

With the Dashboard enabled:

```bash
ros2 run so101_arm_control dashboard_ws_smoke_test
```

Both scripts first require `mode=MOCK` and
`hardware.real_hardware_enabled=false`. They refuse to send if these facts do
not hold.

## ros2_control compatibility note

The audited upstream description includes a `mock_components/GenericSystem`
route, and the local `urdf/so_arm101_mock.urdf.xacro` plus
`config/ros2_controllers.yaml` preserve that spike. On this OS, the installed
Humble controller-manager/JTC package combination accepts the controller
parameter file but does not forward the read-only `joints`,
`command_interfaces`, and `state_interfaces` values into the controller node.
The controller consequently stays unconfigured.

The working Step B launch therefore uses a deterministic local mock server that
implements the same standard `control_msgs/action/FollowJointTrajectory`
interface. Do not upgrade/downgrade system ROS packages or switch the default
backend until the ros2_control package set is reviewed and regression-tested as
one unit.

## Step C: real hardware, read-only probe

The first real-hardware operation is deliberately not a normal robot launch.
`SOFollower.connect()` is forbidden here because it calls `configure()` and
writes motor registers. The one-shot probe uses `FeetechMotorsBus` directly,
blocks all library write methods, verifies the stable USB identity and exact
calibration SHA-256, then reads torque-enable state, present position, voltage,
temperature, current and moving state. It disconnects with
`disable_torque=False` so shutdown also performs no
register write.

Run only after physically supporting the arm, clearing the workspace, preparing
the 12 V power cutoff and confirming no process owns the port:

```bash
/home/matsunaga-h/miniconda3/envs/lerobot/bin/python \
  /home/matsunaga-h/robot_ws/src/so101_arm_control/so101_arm_control/read_only_probe.py \
  --ack-read-only
```

This command does not publish ROS commands and cannot move the arm. Do not start
the Dashboard command path or a real trajectory server until this probe reports
all six IDs and the result has been reviewed.

After reviewing the one-shot probe, start the feedback-only ROS/RViz chain:

```bash
source /home/matsunaga-h/robot_ws/install/setup.bash
ros2 launch so101_arm_control arm_real_read_only_rviz.launch.py
```

This launch has no Dashboard, action server, controller or motor command topic.
It publishes `/joint_states` at 5 Hz plus `/arm/hardware_status`. A raw position
up to five ticks outside calibration is clipped for RViz display only; a larger
error or any nonzero `Torque_Enable` stops publication.

The first write test is intentionally separate from Dashboard and trajectory
control. After the read-only launch is stopped and the workspace is physically
cleared, `supervised_micro_move` seeds the current wrist-roll goal before moving,
accounts for Feetech firmware automatically enabling torque on a goal write,
enables/monitors only motor 5, moves at most 20 ticks (about
1.3 degrees), checks current/temperature/position, and disables that motor again.
All other register and motor writes are blocked in code.

The first supervised hardware result on 2026-08-29 was wrist-roll raw position
`1767 -> 1783` for a requested `+20` ticks. Peak observed current was `1`,
temperature was `35 C`, and all six torque-enable registers read back `0` after
the test. The four-tick steady-state error established a five-tick baseline
completion tolerance; it does not change the calibration file.

## Step D: guarded real Dashboard test

`arm_real_dashboard_test.launch.py` replaces the read-only serial owner with a
guarded real `FollowJointTrajectory` server and starts the existing Dashboard.
Preview may contain one or more changed joints across the configured command
range. The gateway derives a minimum duration from per-joint velocity limits,
and the hardware server independently enforces that duration. The server
interpolates and samples at 20 Hz, checks temperature, applies an immediate
sampled hard-current stop, and detects sustained current plus following error
plus lack of progress as resistance/stall. Entering `code2real&sim` first seeds
every goal register from the measured pose, then verifies all six
`Torque_Enable` registers are ON. The monitored `HOLDING` state remains active
before, during, and between discrete goals so the arm does not fall. Per-motor
HOLD-entry tolerances account for measured servo backlash and gravity settling.
An ordinary final-position miss or operator Cancel stops at and holds the newly
measured pose, so another Preview/Execute may run without a torque drop. Hold is
released only on `real2sim`, explicit Release, a protection fault, or safe
shutdown. Both `code2real&sim` and `teleoperate` are powered modes. A real
protection fault remains latched until the operator
presses Reset HOLD fault; reset performs torque/current/temperature checks and,
when still in `code2real&sim`, safely seeds the measured pose and restores
six-axis HOLD. HOLD samples all torque/current/temperature/position at 10 Hz
and releases on hard over-current, sustained current, excessive position
disturbance, or confirmed over-temperature. Temperature uses two-stage
validation: five consecutive over-limit sync samples followed by three direct
single-motor register reads, with at least two direct reads still over the
limit. An unconfirmed sync spike is logged and HOLD continues; a failed direct
confirmation read is treated as a safety fault. The arm must be
physically supported before an intentional release because torque-off motion
can still fall under gravity. The launch shuts down as a unit if the real
server exits.

Every protection stop is persisted separately from calibration in
`todo-my/arm-baseline/so101_learned_soft_limits.json`. A current/resistance stop
near a calibrated endpoint records a backed-off directional soft boundary;
mid-range contact is logged but does not shrink the joint range. The learned
file is calibration-hash-bound and is rejected if copied to another
calibration. No protection path writes motor configuration or the LeRobot
calibration file. The 20 Hz sampled stop is software protection and does not
replace firmware/hardware current limiting.

Runtime control authority is exposed separately from the hardware environment:

- `real2sim`: torque-off manual follower motion drives `/joint_states`, RViz and Dashboard; actions are rejected.
- `code2real&sim`: Dashboard/code/VLA may submit guarded multi-joint goals; RViz shows actual and candidate.
- `teleoperate`: the calibrated 5 V leader is read through its stable USB identity
  while remaining torque-free. Switching mode first seeds the measured 12 V
  follower pose and verifies all six follower torques ON. It then moves the
  follower, under the configured per-joint velocity limits, to the calibrated
  absolute Leader pose. The runtime state remains `TELEOP_ALIGNING` until all
  six joints enter the alignment tolerance, then becomes `TELEOP_ACTIVE` and
  continuously follows Leader displacement. Initial alignment has its own
  controlled velocity profile; ACTIVE following uses a separate faster profile
  without changing the conservative `code2real&sim` trajectory limits. This
  avoids an instantaneous entry jump while still correcting the initial
  follower pose and keeping later Leader motion responsive.
  Leader feedback loss, stale data, unexpected Leader torque, follower torque
  loss, current/resistance, or confirmed temperature protection releases the
  follower and latches a fault.

The gateway publishes `/arm/control_mode`; the hardware server reports its
observed mode in `/arm/hardware_status`. Execute remains disabled until the two
agree, and mode changes are refused while a goal is active.

The leader adapter is a separate read-only process and publishes
`/arm/leader_status`. If the leader is unplugged, Arm Only and Full still start
in the configured default `real2sim` mode; the Dashboard disables only the
`teleoperate` button until fresh calibrated Leader feedback is available.

```bash
source /home/matsunaga-h/robot_ws/install/setup.bash
ros2 launch so101_arm_control arm_real_dashboard_test.launch.py
```
