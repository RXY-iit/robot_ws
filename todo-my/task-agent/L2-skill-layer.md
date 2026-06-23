# L2 Skill Layer Guidance

> Date: 2026-06-23
> Workspace: `/home/ruan-x/robot_ws`
> Goal: turn the existing ROS2 robot platform into a typed, testable Skill API.

## Decisions

Current decisions:

```text
Implementation form: ROS2 package from the start
L2 package name: src/robot_skills
Perception/model policy: hybrid
Control-critical perception: local deterministic/model output
Cloud VLM/API: allowed for non-contact confirmation, explanation, and reports only
AUTO mode: human presses A / confirms; Skills should not silently switch real robot to AUTO
```

## Position In Robot Harness

L2 is the boundary between "robot can move" and "agent can ask robot to do work".

```text
L3 Agent / planner
  -> typed Skill call
  -> L2 Skill executor
  -> ROS2 action/service/topic wrapper
  -> L1 platform packages
```

Hard rule:

```text
LLM / Agent never publishes raw /cmd_vel, raw joint trajectory, arbitrary ROS topic names,
or free-form /mission_command strings.
```

The agent may choose a Skill. The Skill owns validation, prechecks, execution, timeout, retry,
result formatting, and logging.

## Current Workspace Mapping

These packages already provide the L1 backend that L2 should wrap:

| Area | Existing module | Current role | L2 usage |
| --- | --- | --- | --- |
| Robot bringup | `src/robot_bringup` | Real robot TF, Livox MID360, RealSense D435/D455-style camera launch | Health precheck and sensor availability |
| Simulation | `src/robot_sim` | Isaac Sim bridge, omni drive command, simulated lift/camera swing, sensor helpers | Primary regression backend for Skills |
| Navigation | `src/nav_pkg` | Nav2 launch, MPPI/RPP, mode switch, camera costmap | Navigation execution backend |
| Localization | `src/localization_pkg` | map server, AMCL, GICP/FAST-LIO related localization launch files | Pose and localization health |
| Safety | `src/safety_layer` | `/cmd_vel_raw` watchdog/clamp/estop -> `/cmd_vel` | Required motion safety gate |
| Mission legacy | `src/mission_manager` | String command mission executor using BasicNavigator | Useful reference; should be wrapped or replaced by typed Skills |
| Base driver | `src/omni_base_driver` | Real omni base motor/odom interface | L1 only; L2 must not talk directly to motors |
| Manipulation-lite | `src/serial_transciever` | lift, camera pan/tilt, relay/manual motor nodes | First manipulation Skills should target these narrow controls |
| Perception inputs | `src/realsense-ros`, `src/livox_ros_driver2` | Camera and LiDAR drivers | Observation source for perception Skills |
| Debug tools | `tools/nav2_debug_recorder.py`, `tools/nav2_debug_monitor.py`, `tools/record_semantic_bag.sh` | Navigation and semantic data capture | Skill evaluation and failure replay |

## Recommended Folder Layout

Create a new package instead of mixing L2 into `nav_pkg`:

```text
src/robot_skills/
  package.xml
  setup.py
  setup.cfg
  resource/robot_skills
  robot_skills/
    __init__.py
    schemas.py
    result_codes.py
    registry.py
    health.py
    ros_utils.py
    semantic_map.py
    executors/
      __init__.py
      base.py
      navigation.py
      perception.py
      camera.py
      lift.py
      safety.py
    nodes/
      skill_server_node.py
      semantic_map_marker_node.py
  config/
    skill_registry.yaml
    semantic_map.yaml
    agent_skill_policy.yaml
  test/
    test_schemas.py
    test_semantic_map.py
    test_skill_policy.py
```

Keep prototypes that do not need ROS in:

```text
tools/agent/
  dry_run_skill_call.py
  validate_skill_registry.py
```

Logs:

```text
debug-output/agent/YYYYMMDD_HHMMSS/
  mission.json
  skill_events.jsonl
  nav2_debug/
  screenshots/
  rosbag/
```

## Data Contracts

All Skills should use Python dataclasses or Pydantic models at the boundary.
Use Pydantic when the caller can be an LLM/agent and schema validation matters.

### Core Models

```python
class Pose2D:
    frame_id: str = "map"
    x: float
    y: float
    yaw: float

class SkillContext:
    run_id: str
    mode: str              # "dry_run" | "sim" | "real"
    robot_mode: str        # "AUTO" | "MANUAL" | "UNKNOWN"
    require_human_confirm: bool
    max_duration_sec: float

class SkillResult:
    skill_name: str
    success: bool
    code: str
    message: str
    started_at: str
    ended_at: str
    metrics: dict
    observations: list
    artifacts: list
```

### Result Codes

Use stable result codes so L3 can reason without parsing prose:

```text
OK
DRY_RUN_OK
FAILED_SCHEMA
FAILED_PRECHECK
FAILED_LOCALIZATION
FAILED_NAV2_INACTIVE
FAILED_SAFETY
FAILED_TIMEOUT
FAILED_NAVIGATION
FAILED_PERCEPTION
FAILED_UNSUPPORTED
NEEDS_HUMAN
CANCELLED
ABORTED
```

## Skill Registry Schema

`config/skill_registry.yaml` should be the source of truth for agent-visible Skills:

```yaml
skills:
  - name: goto_pose
    version: 0.1.0
    description: >
      Navigate the mobile base to a validated map-frame pose using Nav2 NavigateToPose.
      Use when the target pose is already known.
    executor: robot_skills.executors.navigation:GotoPoseSkill
    input_schema: GotoPoseInput
    output_schema: SkillResult
    safety_level: navigation
    allowed_modes: [sim, real, dry_run]
    requires_human_confirm: false
    timeout_sec: 120
    retry_policy:
      max_attempts: 1
      clear_costmaps_between_attempts: false
    preconditions:
      - robot_mode_auto
      - safety_status_ok_or_watchdog
      - localization_healthy
      - nav2_active
      - target_pose_in_map
    side_effects:
      - sends_nav2_goal
    forbidden_effects:
      - publish_cmd_vel
      - publish_raw_motor_command
      - modify_map
    sim_supported: true
    real_supported: true
    metrics:
      - success
      - duration_sec
      - final_pose_error_m
      - final_yaw_error_rad
      - recoveries
```

## Skill Groups

### Group A: System And Safety Skills

These Skills do not move the robot. They are always allowed before mission execution.

#### `get_robot_status`

Purpose: collect current platform state.

Inputs:

```text
none
```

Reads:

```text
/robot_mode
/safety_status
/clock if sim
TF map->base_footprint or odom->base_footprint
Nav2 lifecycle state
camera/LiDAR topic availability
```

Output metrics:

```text
robot_mode
safety_status
localization_ok
nav2_active
camera_ok
lidar_ok
use_sim_time
```

Acceptance:

```text
returns within 1.0 sec in dry-run/mock
returns within 3.0 sec against ROS
never changes robot state
```

#### `await_robot_auto_mode`

Purpose: wait for operator-confirmed AUTO mode before motion.

Backend:

```text
publish/request a human-facing prompt if UI is available
operator presses A or otherwise confirms AUTO outside the Skill
confirm /robot_mode == AUTO
```

Preconditions:

```text
safety_status != ESTOP
nav_mode_switch_node exists
```

Forbidden:

```text
do not bypass nav_mode_switch_node
do not silently publish AUTO in real mode
```

Metrics:

```text
auto_confirmation_latency_sec
confirmed_robot_mode
```

#### `cancel_motion`

Purpose: cancel active base motion and return to a safe non-moving state.

Backend:

```text
call /navigate_to_pose/_action/cancel_goal or BasicNavigator cancelTask
publish MANUAL to /nav_mode/command when requested
verify /cmd_vel is zero or watchdog
```

This Skill must be callable from every L3 state.

### Group B: Semantic Map Skills

The current `mission_manager/config/waypoint_map.yaml` is a waypoint map, not a semantic map.
L2 should introduce a separate semantic map that can reference waypoints.

Recommended file:

```text
src/robot_skills/config/semantic_map.yaml
```

Example:

```yaml
map_id: l402_v0
frame_id: map
objects:
  - id: shelf_a
    names: ["shelf a", "A shelf", "架子A", "A棚"]
    category: shelf
    static: true
    pose: {x: 0.0, y: 0.0, yaw: 1.5708}
    approach_poses:
      - id: front
        pose: {x: 0.0, y: 0.0, yaw: 1.5708}
        clearance_mode: narrow
        speed_profile: cautious
    affordances: ["navigate_near", "inspect"]
    safety_zone_m: 0.5
    confidence: 1.0
places:
  - id: home
    names: ["home", "origin", "起点", "ホーム"]
    waypoint_id: home
```

#### `resolve_semantic_target`

Purpose: convert natural-language target name and relation into a candidate `Pose2D`.

Inputs:

```text
target_name: str
relation: "near" | "front" | "left" | "right" | "inside_area" | "at"
allowed_categories: list[str] | None
```

Output:

```text
object_id
canonical_name
category
pose
approach_pose_id
confidence
needs_clarification
alternatives
```

Acceptance:

```text
known aliases resolve deterministically
unknown names return NEEDS_HUMAN, not hallucinated coordinates
all output poses are map-frame unless explicitly marked otherwise
```

#### `list_known_targets`

Purpose: expose available semantic objects to UI or agent.

Use when the user asks "你知道哪些地方".

### Group C: Navigation Skills

Current backend:

```text
Nav2 NavigateToPose action
/nav2/cmd_vel -> nav_mode_switch_node -> /cmd_vel_raw -> cmd_vel_safety_node -> /cmd_vel
```

Do not use `/goal_pose` as the formal Skill interface.

#### `goto_pose`

Inputs:

```text
pose: Pose2D
position_tolerance_m: float = 0.30
yaw_tolerance_rad: float = 0.50
speed_profile: "normal" | "cautious" = "normal"
```

Prechecks:

```text
frame_id == "map"
robot_mode == AUTO
safety_status in ["OK", "WATCHDOG"]
Nav2 lifecycle active
TF map->base_footprint available
map server active
```

Backend:

```text
nav2_msgs/action/NavigateToPose
```

Metrics:

```text
duration_sec
path_length_m if available
final_pose_error_m
final_yaw_error_rad
recoveries
nav2_result
```

Acceptance:

```text
sim success rate >= 90% for 10 known reachable goals
does not publish /cmd_vel directly
cancel works within 1.0 sec
failure returns structured code, not exception text only
```

#### `goto_named_place`

Inputs:

```text
place_id_or_name: str
relation: "at" | "near" | "front" = "at"
```

Flow:

```text
resolve_semantic_target
goto_pose
```

Acceptance:

```text
known waypoints and semantic aliases work in dry-run
unknown target returns NEEDS_HUMAN
```

#### `goto_object_nearby`

Inputs:

```text
object_name: str
relation: "near" | "front" | "left" | "right" = "near"
```

This is the first user-facing navigation Skill for commands like:

```text
去叉车旁边
去6号架子前
到电梯门口
```

### Group D: Inspection And Perception Skills

Perception should start narrow. Do not begin with a general "understand scene" Skill.

Model choices:

```text
v0 local deterministic: color threshold, fiducial marker, known object detector
v1 local GPU: YOLO/RT-DETR style fixed detector for boxes/buttons/tools
v2 local open-vocabulary: Grounding DINO + SAM/SAM2 or equivalent
v3 cloud VLM: GPT-4.1/4o class model for image confirmation and natural-language reporting
```

Default for this workspace:

```text
Hybrid perception.
Use v0/v1 local perception for robot-control decisions.
Use cloud VLM/API only for non-contact confirmation, explanation, ambiguity handling, and reports.
Cloud VLM/API output cannot authorize contact actions or override local safety gates.
```

#### `capture_observation`

Purpose: collect synchronized camera/LiDAR/pose metadata for a mission step.

Reads:

```text
camera color image
camera depth image or /camera/depth/points
TF camera->base->map
robot pose
```

Output:

```text
artifact paths
timestamp
camera frame ids
robot pose
```

Acceptance:

```text
stores image artifact
records frame_id and timestamp
does not block Nav2 longer than timeout
```

#### `detect_known_object`

Inputs:

```text
query: str
region_hint: str | None
min_confidence: float = 0.5
```

Output:

```text
detections:
  label
  confidence
  bbox
  frame_id
  estimated_3d_position optional
```

Contact-gating:

```text
confidence >= 0.80 required before any manipulation pre-action
confidence >= 0.90 plus human confirmation required before contact
```

#### `inspect_target`

Flow:

```text
capture_observation
detect_known_object or visual confirm
return observation summary
```

Example commands:

```text
看一下电梯按钮在不在视野里
确认架子前有没有红色箱子
```

### Group E: Manipulation-Lite Skills

Current workspace has lift and camera pan/tilt control. Treat these as the first operation Skills.

Do not define general `pick_object` as executable until there is a real MoveIt/controller backend.

#### `look_at`

Purpose: move camera pan/tilt or base yaw to observe a target.

Backend options:

```text
camera pan topic: /chokudomotor/target_angle
camera tilt topic: /cameraswingmotor/target_angle
optional base rotate via Nav2 spin behavior
```

Inputs:

```text
target: semantic object id or direction
mode: "camera_only" | "base_only" | "base_then_camera"
```

Preconditions:

```text
base stopped for camera motor movement in real mode unless tested otherwise
angle limits known
```

Metrics:

```text
pan_target_deg
tilt_target_deg
angle_reached if feedback available
```

#### `set_camera_pose`

Inputs:

```text
pan_deg: float
tilt_deg: float
```

Hard limits should come from config, not prompt:

```text
pan lower/upper
tilt lower/upper
max step per command
```

#### `move_lift`

Purpose: controlled lift movement.

Backend today:

```text
/lift/command: "UP" | "DOWN" | "STOP"
sim bridge: sim_lift_command_node
```

Recommended Skill interface:

```text
move_lift(direction, duration_sec)
move_lift_to_named_pose("low" | "middle" | "high")
stop_lift()
```

Do not expose unbounded `UP`/`DOWN` to L3.

Preconditions:

```text
base stopped
robot not navigating
duration_sec <= configured max
real_supported only after limit switches/position feedback are confirmed
```

### Group F: Future Contact Skills

These are planned, not executable in the current workspace:

```text
move_arm_named_pose
move_arm_to_pregrasp
press_button
toggle_switch
grasp_object
place_object
```

Required before enabling:

```text
MoveIt 2 or chosen arm controller
joint state feedback
collision scene
workspace limits
force/travel/velocity limits
human confirmation path
sim-first validation
physical estop
```

## Model/API Policy For L2

L2 should be mostly deterministic.

Allowed:

```text
Pydantic validation
ROS2 actions/services/topics
local perception models
optional cloud VLM for observation labeling/reporting
```

Not allowed:

```text
LLM-generated ROS topic names
LLM-generated coordinates without semantic map or perception grounding
LLM-generated joint trajectory
LLM deciding safety status from prose
```

Recommended stack:

```text
Python 3.10
rclpy
Pydantic v2 if available, otherwise dataclasses + manual validation
Nav2 action client or nav2_simple_commander for first implementation
OpenCV for v0 perception
ONNX/TensorRT/PyTorch local detector later
```

## Evaluation Matrix

Each Skill should have a small measurable scorecard.

| Skill | Primary metric | Target before L3 use |
| --- | --- | --- |
| `get_robot_status` | response latency | p95 < 3 sec |
| `resolve_semantic_target` | known alias accuracy | 100% on registry aliases |
| `goto_pose` | sim navigation success | >= 90% on 10 reachable goals |
| `goto_named_place` | semantic + nav success | >= 90% for measured places |
| `cancel_motion` | cancel latency | < 1 sec to zero command |
| `capture_observation` | artifact completeness | image + pose + frame_id present |
| `detect_known_object` | precision on demo objects | >= 90% before manipulation gating |
| `set_camera_pose` | command bounds | 100% rejects out-of-range |
| `move_lift` | bounded actuation | no unbounded motion commands |

## First Implementation Order

1. Create `src/robot_skills` as a ROS2 Python package.
2. Add `schemas.py`, `result_codes.py`, `skill_registry.yaml`.
3. Add `semantic_map.yaml` and `resolve_semantic_target`.
4. Add `get_robot_status`, `await_robot_auto_mode`, `cancel_motion`, and human-confirmation-aware AUTO checks.
5. Add `goto_pose` and `goto_named_place` with dry-run mode, then sim Nav2 execution.
6. Add JSONL skill event logging.
7. Add `capture_observation`.
8. Add hybrid perception hooks: local detector/stub first, cloud VLM confirmation as non-contact report only.
9. Add `set_camera_pose` and bounded `move_lift` in sim only.

## Open Questions To Discuss

1. Should `src/mission_manager` be retired, or kept as a simple executor behind the new typed Skill API?
2. For real robot operation, do lift/camera motors have reliable feedback and limit switches, or should they remain manual/sim-only Skills for now?
3. What is the first demo scene vocabulary: L402 lab places, warehouse objects, elevator/buttons, or shelves/boxes?
4. Which concrete cloud provider/model should be used for non-contact VLM confirmation and reports?
