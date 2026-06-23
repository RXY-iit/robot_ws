# L3 Agent Layer Guidance

> Date: 2026-06-23
> Depends on: `L2-skill-layer.md`
> Goal: define the mission agent that plans, selects Skills, monitors execution, and asks humans when uncertainty or risk is high.

## Decisions

Current decisions:

```text
Implementation form: ROS2 package from the start
L3 package name: src/robot_agent
First runnable interface: ROS2 node, with optional CLI wrapper for dry-run/replay
Perception/model policy: hybrid
Human safety policy: human presses A / confirms AUTO before navigation
Contact policy: disabled until future gated Skills exist
```

## L3 Principle

L3 is not the robot controller. L3 is the task executive.

```text
User intent
  -> structured mission
  -> Skill plan
  -> Skill calls
  -> observation and verification
  -> report / retry / ask human / abort
```

L3 must never:

```text
publish /cmd_vel
publish motor or joint commands
generate arbitrary ROS topic names
write navigation maps silently
skip L2 prechecks
turn low-confidence perception into contact action
```

L3 may:

```text
parse user intent
ask L2 for robot status
resolve semantic targets through L2
choose allowed Skills from registry
sequence Skills
monitor structured SkillResult
persist mission memory
ask human for confirmation
```

## Recommended Folder Layout

Create a separate package:

```text
src/robot_agent/
  package.xml
  setup.py
  setup.cfg
  resource/robot_agent
  robot_agent/
    __init__.py
    config.py
    schemas.py
    model_clients.py
    policy.py
    prompts.py
    memory.py
    mission_log.py
    skill_client.py
    graph/
      __init__.py
      state.py
      nodes.py
      mission_graph.py
      recovery.py
    agents/
      __init__.py
      mission_agent.py
      navigation_agent.py
      perception_agent.py
    parsers/
      __init__.py
      deterministic_parser.py
      llm_parser.py
    nodes/
      agent_node.py
      text_command_node.py
  config/
    agent_runtime.yaml
    mission_policy.yaml
    model_policy.yaml
    prompt_templates.yaml
  test/
    test_parser.py
    test_policy.py
    test_mission_graph.py
    test_skill_client_mock.py
```

Keep CLI prototypes in:

```text
tools/agent/
  agent_dry_run.py
  replay_mission_log.py
  inspect_memory.py
```

## Runtime Modules

### `agent_node`

ROS2-facing node for text commands and status.

Suggested topics:

```text
Sub: /agent/text_command       std_msgs/String
Sub: /agent/cancel             std_msgs/Bool or std_msgs/String
Pub: /agent/status             std_msgs/String
Pub: /agent/mission_events     std_msgs/String JSONL line
```

Do not subscribe directly to joystick or motor topics. Those belong to L1/L2.

### `skill_client`

The only module that calls L2.

First version can call Python functions in-process. Later it can call ROS2 services/actions.

Required methods:

```python
list_skills()
validate_skill_call(skill_name, payload)
execute_skill(skill_name, payload, context) -> SkillResult
cancel_all()
```

### `mission_graph`

The mission state machine. It should be small at first.

```text
RECEIVED
  -> PARSE
  -> RESOLVE
  -> PREFLIGHT
  -> PLAN
  -> EXECUTE_STEP
  -> OBSERVE
  -> VERIFY
  -> DONE
```

Failure branches:

```text
NEEDS_HUMAN
FAILED_PRECHECK
FAILED_SKILL
RETRY
ABORTED
```

### `policy`

Hard-coded mission policy. It is more important than prompts.

Examples:

```text
navigation requires /robot_mode == AUTO; operator presses A / confirms before motion
camera/lift allowed only when base is stopped
contact actions always require explicit human confirmation
unknown target returns clarification request
cloud VLM output cannot authorize contact
```

### `memory`

Use plain inspectable storage first:

```text
debug-output/agent/.../mission.json
debug-output/agent/.../events.jsonl
src/robot_agent/config/local_memory.yaml or ~/.robot_harness/memory.sqlite
```

Memory layers:

```text
mission memory: this run's state and Skill results
episodic memory: previous mission logs
semantic memory: user-confirmed aliases and stable places
skill memory: parameters that succeeded or failed
```

Rule:

```text
Memory never silently overwrites semantic_map.yaml.
Promote memory to semantic map only after explicit user confirmation.
```

## Agent Roles

Start with three agents. Do not create a large swarm yet.

### Mission Agent

Main owner of the task.

Responsibilities:

```text
parse user task or call parser
choose mission type
build ordered Skill plan
call Navigation/Perception helper agents when useful
enforce policy
track mission state
ask human
produce final report
```

It can use an LLM after deterministic parsing fails or when the user gives a multi-step instruction.

### Navigation Agent

Narrow helper for target resolution and navigation plans.

Responsibilities:

```text
turn "去叉车旁边" into resolve_semantic_target + goto_pose
select relation/approach pose
interpret navigation SkillResult
choose retry or ask human
```

It does not call Nav2 directly.

### Perception Agent

Narrow helper for inspection/search/visual confirmation.

Responsibilities:

```text
choose capture_observation / detect_known_object / inspect_target
summarize detections
decide if confidence is enough for reporting
mark low confidence as NEEDS_HUMAN
```

It does not authorize contact.

## Model And API Policy

### Default v0: Deterministic First

Use no LLM for the first running demo:

```text
regex/parser for commands
semantic_map alias lookup
Skill registry validation
mission graph with mock Skill client
```

This should support:

```text
去 home
去 shelf_a
去架子A前
取消
查看机器人状态
```

### v1: Hybrid Cloud LLM/API For Parsing, Planning, And Confirmation

Use cloud API for language and non-contact visual confirmation only, not low-level control.

Recommended use:

```text
parse natural language into MissionIntent JSON
decompose multi-step tasks
generate user-facing clarification questions
summarize mission result
describe captured images for operator understanding
second-opinion visual confirmation for non-contact tasks
```

Suggested model class:

```text
fast/cheap model for command parsing
stronger model for multi-step planning and report generation
```

Keep exact model names configurable in `model_policy.yaml`, for example:

```yaml
providers:
  command_parser:
    provider: openai
    model: configurable-fast-model
    temperature: 0
  mission_planner:
    provider: openai
    model: configurable-reasoning-model
    temperature: 0
  visual_confirm:
    provider: cloud
    model: configurable-vlm
    allowed_for_contact: false
```

Reason: model availability changes. The code should not hard-code a specific cloud model into safety logic.

Hybrid rule:

```text
Local semantic map + local detector/stub make control-critical decisions.
Cloud LLM/VLM may help parse, explain, summarize, or ask a better question.
Cloud output cannot authorize robot motion, contact, or safety override by itself.
```

### v2: Local Models

Local model use cases:

```text
speech-to-text if voice UI is needed
small local LLM for command parsing when offline
local object detector for boxes/buttons/tools
local VLM only for non-contact explanation unless verified
```

Hybrid recommendation:

```text
Use deterministic parser + semantic map + local detector for control-critical paths.
Use API LLM/VLM for interpretation, non-contact visual confirmation, reporting, and operator UX.
```

## Mission Schemas

### `MissionIntent`

```python
class MissionIntent:
    task_type: str
    target: str | None
    relation: str | None
    object_query: str | None
    constraints: dict
    requires_confirmation: bool
    raw_text: str
```

Allowed `task_type` in v0/v1:

```text
robot_status
navigate_to_place
navigate_to_object
inspect_place
search_object_near_place
cancel
ask_human
```

Future, gated:

```text
prepare_manipulation
contact_action
pick_and_place
```

### `MissionState`

```python
class MissionState:
    mission_id: str
    user_text: str
    mode: str                    # dry_run | sim | real
    intent: MissionIntent | None
    current_step_index: int
    plan: list[SkillCall]
    skill_results: list[SkillResult]
    observations: list[Observation]
    status: str
    question_for_human: str | None
    final_message: str | None
```

### `SkillCall`

```python
class SkillCall:
    name: str
    payload: dict
    reason: str
    timeout_sec: float
    requires_human_confirm: bool
```

## Mission Graph Detail

### Node: `parse_instruction`

Input:

```text
raw user text
```

Output:

```text
MissionIntent
```

Policy:

```text
try deterministic parser first
if not matched, call LLM parser
if schema invalid, ask human
```

### Node: `resolve_targets`

Calls:

```text
resolve_semantic_target
list_known_targets if ambiguous
```

Rules:

```text
target confidence < 0.70 -> NEEDS_HUMAN
multiple close aliases -> ask human to choose
do not invent coordinates
```

### Node: `safety_precheck`

Calls:

```text
get_robot_status
await_robot_auto_mode if navigation is planned and robot is not AUTO
```

Rules:

```text
real mode + ESTOP -> fail precheck
real mode + unknown localization -> fail precheck
contact requested -> ask human and require contact skill support
```

### Node: `plan_steps`

Examples:

```text
"去架子A前"
  -> resolve_semantic_target(target="架子A", relation="front")
  -> goto_pose(pose=resolved_pose, speed_profile="cautious")

"去架子A前找红色箱子"
  -> resolve_semantic_target("架子A", "front")
  -> goto_pose(...)
  -> capture_observation()
  -> detect_known_object(query="red box")

"查看机器人状态"
  -> get_robot_status()
```

### Node: `execute_step`

Rules:

```text
execute only one Skill at a time in v0
write event before and after every Skill
on cancel, call cancel_motion immediately
respect timeout from registry, not from prompt
```

### Node: `observe`

After navigation:

```text
get_robot_status
optionally capture_observation if task requires visual verification
```

### Node: `verify`

Examples:

```text
navigation task: final pose error within tolerance or Nav2 success
inspection task: detection confidence above reporting threshold
search task: at least one candidate or clear "not found"
```

### Node: `recover`

Allowed v0 recovery:

```text
clear costmaps via Skill if available
retry goto_pose once for navigation timeout/failure
ask human after retry fails
cancel_motion for unsafe state
```

Avoid automatic repeated retries in real mode until reliability is measured.

## Agent Runtime Config

`config/agent_runtime.yaml`:

```yaml
mode: sim       # dry_run | sim | real
ros_domain_id: 20
default_language: zh
log_root: debug-output/agent
skill_registry_path: src/robot_skills/config/skill_registry.yaml
semantic_map_path: src/robot_skills/config/semantic_map.yaml
max_mission_duration_sec: 300
auto_mode_policy: human_confirm_required
implementation_form: ros2_package
model_policy: hybrid
human_confirmation:
  switch_to_auto: always
  contact_actions: always
  real_navigation_to_unknown: always
  low_confidence_perception: always
```

`config/mission_policy.yaml`:

```yaml
allowed_task_types:
  dry_run:
    - robot_status
    - navigate_to_place
    - navigate_to_object
    - inspect_place
    - search_object_near_place
    - cancel
  sim:
    - robot_status
    - navigate_to_place
    - navigate_to_object
    - inspect_place
    - search_object_near_place
    - set_camera_pose
    - move_lift_bounded
    - cancel
  real:
    - robot_status
    - navigate_to_place
    - navigate_to_object
    - inspect_place
    - cancel
blocked_until_implemented:
  - press_button
  - grasp_object
  - place_object
```

## Prompt Boundary

Prompts should describe how to produce typed JSON only. They should not include map coordinates,
ROS topic names, or safety override instructions.

LLM parser output example:

```json
{
  "task_type": "navigate_to_object",
  "target": "叉车",
  "relation": "near",
  "object_query": null,
  "constraints": {
    "speed_profile": "cautious"
  },
  "requires_confirmation": false
}
```

Bad output that must be rejected:

```json
{
  "task_type": "raw_ros_publish",
  "topic": "/cmd_vel",
  "linear_x": 1.0
}
```

## Metrics

### Mission Metrics

```text
parse_success_rate
schema_validation_success_rate
semantic_resolution_success_rate
precheck_success_rate
mission_success_rate
mean_mission_duration_sec
human_intervention_rate
abort_rate
unsafe_command_rejection_count
```

### Navigation Agent Metrics

```text
target_resolution_accuracy
navigation_success_rate
navigation_timeout_rate
average_final_pose_error_m
cancel_latency_sec
retry_success_rate
```

### Perception Agent Metrics

```text
observation_capture_success_rate
detection_precision
detection_recall on demo dataset
low_confidence_handoff_rate
false_positive_contact_gate_count
```

### Model Metrics

```text
LLM parser valid_json_rate
LLM parser schema_valid_rate
LLM ambiguity_detection_rate
LLM latency_p95
LLM cost_per_mission if using cloud API
local_model_fps if using local detector/VLM
```

## Demo Roadmap

### Demo 0: Dry-Run Agent

Command:

```text
agent_dry_run.py "去架子A前"
```

Expected:

```text
parsed MissionIntent
resolved semantic target
planned goto_pose SkillCall
no ROS motion
JSONL mission log written
```

Done when:

```text
10 common Chinese/Japanese/English aliases parse correctly
unknown targets ask for clarification
invalid command is rejected
```

### Demo 1: Sim Navigation Agent

Expected:

```text
starts after existing sim/nav stack is running
checks robot status
waits for operator-confirmed AUTO
sends NavigateToPose through L2
records SkillResult
```

Use existing launcher:

```text
tools/sim/open_sim_nav2_windows.sh --cleanup
```

Done when:

```text
>= 90% success over 10 reachable semantic targets
cancel works
blocked goals become NEEDS_HUMAN or FAILED_NAVIGATION
```

### Demo 2: Navigate Then Inspect

Command:

```text
去架子A前找红色箱子
```

Expected:

```text
navigation to shelf approach pose
observation captured
red-box detector/stub result reported
low confidence asks human
```

Done when:

```text
image artifact + pose + detection JSON are logged for every run
```

### Demo 3: Manipulation-Lite In Sim

Command:

```text
到架子A前，把相机转向架子
```

Expected:

```text
navigation complete
base stopped
set_camera_pose or look_at Skill called
observation captured
```

Do not include contact.

## Current Workspace-Specific Guidance

1. Treat `src/mission_manager` as a useful Phase 4 prototype, but do not expose its string command interface directly to LLM.
2. Keep `nav_pkg`, `localization_pkg`, `safety_layer`, and `robot_sim` stable while building L2/L3.
3. Use `ROS_DOMAIN_ID=20` for sim unless you intentionally change the stack.
4. Require `/robot_mode == AUTO` before navigation Skill execution; if not AUTO, return `NEEDS_HUMAN` and ask the operator to press A / confirm.
5. Respect the existing command chain: `/nav2/cmd_vel -> /cmd_vel_raw -> /cmd_vel`.
6. Use `tools/nav2_debug_recorder.py` or equivalent logs for failed navigation missions.
7. Keep semantic map separate from `mission_manager/config/waypoint_map.yaml` until the new format is validated.
8. Start camera/lift Skills in sim mode first because current files show command topics but safety feedback/limits need confirmation for real mode.

## Open Questions To Discuss

1. What is your preferred cloud provider/model family for language parsing and non-contact VLM confirmation?
2. Should voice input be in scope now, or stay as text-only until Skills are stable?
3. Which first user-facing demo matters most: "navigate to semantic place", "navigate then inspect", or "navigate then camera/lift operation"?
4. Should cloud API access be allowed during real robot navigation, or only in sim/offline review?
