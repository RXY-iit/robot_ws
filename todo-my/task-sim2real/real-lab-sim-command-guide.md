# Real Lab Sim Command Guide

Short command guide for:

1. create GLIM map
2. test navigation and localization
3. create segment map for agents
4. test agent tasks

Use existing tested tools only.

## 0. Common Setup

```bash
cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
```

Open this USD in Isaac Sim:

```text
/home/ruan-x/robot_ws/src/robot_sim/usd/scenes/robot_sim_real_lab.usd
```

Isaac Sim launch:

```bash
cd /home/ruan-x/isaac-sim
./isaac-sim.sh
```

## 1. Create GLIM Map

Start the mapping helper:

```bash
cd /home/ruan-x/robot_ws
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_real_lab_001 \
  --gpu \
  --direct-cmd-vel \
  --record-d455 \
  --cleanup
```

Note: this existing helper may still print the old warehouse scene in its
terminal reminder. For this run, keep Isaac opened on `robot_sim_real_lab.usd`.

In Isaac Script Editor, before Play:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/disable_isaac_joint_state_publisher.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/prepare_d455_camera_only.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())
```

Press Play, then run:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
```

Drive slowly. When finished, stop GLIM with Ctrl-C in the GLIM pane and wait for `MAP SAVE OK`.

## 2. Export Maps

Save GLIM dump from `/tmp/dump` if the helper did not copy it automatically:

```bash
cd /home/ruan-x/robot_ws
test -s /tmp/dump/graph.txt
mkdir -p maps/sim_real_lab_001
cp -a /tmp/dump maps/sim_real_lab_001/glim_dump
```

Export 3D PLY from the latest saved dump:

```bash
cd /home/ruan-x/robot_ws
export DUMP=maps/sim_real_lab_001/glim_dump
export PLY=maps/sim_real_lab_001/map_3d.ply

/usr/bin/python3 tools/sim/export_glim_dump_to_ply.py "$DUMP" \
  --output "$PLY" \
  --voxel 0.05
```

Create a first-pass 2D map only if needed:

```bash
cd /home/ruan-x/robot_ws
export PLY=maps/sim_real_lab_001/map_3d.ply
export MAP_PREFIX=maps/sim_real_lab_001/nav2_draft

/usr/bin/python3 tools/ply_to_nav2_map.py "$PLY" \
  --output "$MAP_PREFIX" \
  --resolution 0.05 \
  --z-min 0.90 \
  --z-max 1.35 \
  --min-hits 3 \
  --inflate-radius 0.06
```

Current tested 2D map for navigation:

```text
maps/sim_real_lab_001/nav2.yaml
maps/sim_real_lab_001/nav2.pgm
```

Useful outputs:

```text
maps/sim_real_lab_001/glim_dump/
maps/sim_real_lab_001/map_3d.ply
maps/sim_real_lab_001/nav2.yaml
maps/sim_real_lab_001/nav2.pgm
maps/sim_real_lab_001/semantic_map.yaml
maps/sim_real_lab_001/semantic_segments.yaml
```

## 3. Test Localization And Nav2

Keep Isaac Sim open and in Play. Then:

```bash
cd /home/ruan-x/robot_ws
tools/sim/open_sim_nav2_windows.sh \
  --map maps/sim_real_lab_001/nav2.yaml \
  --ply maps/sim_real_lab_001/map_3d.ply \
  --initial-x 0.0 \
  --initial-y 0.0 \
  --initial-yaw 0.0 \
  --no-motion-policy \
  --cleanup
```

For A/B testing the old front-first relay behavior, replace `--no-motion-policy`
with `--motion-policy`.

Quick checks:

```bash
ros2 topic echo /gicp_loc/score --once
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true
WS=/home/ruan-x/robot_ws USE_SIM_TIME=true tools/check_nav2_phase1.sh
```

In RViz:

```text
1. Check /map, /sim_glim_points, TF, global/local costmaps.
2. Use 2D Pose Estimate if localization needs correction.
3. Use 2D Goal Pose for a short navigation test.
```

## 4. Create Segment Map For Agents

Start segmentation UI:

```bash
cd /home/ruan-x/robot_ws
export ROBOT_MAP_ID=sim_real_lab_001
ros2 run robot_skills segmentation_tool --port 8771
```

Equivalent explicit bundle paths:

```bash
cd /home/ruan-x/robot_ws
ros2 run robot_skills segmentation_tool \
  --map-yaml maps/sim_real_lab_001/nav2.yaml \
  --segments-yaml maps/sim_real_lab_001/semantic_segments.yaml \
  --semantic-map-yaml maps/sim_real_lab_001/semantic_map.yaml \
  --port 8771
```

If the browser still shows the old map, stop the tool and restart with the
explicit `--map-yaml`, `--segments-yaml`, and `--semantic-map-yaml` arguments
above.

Open:

```text
http://127.0.0.1:8771
```

Annotate important regions and objects, then save:

```text
front_door / back_door
trash_bin / meeting_table / sofa / tool_box
desk_rxy / desk_ire / desk_tkd / desk_tcb
desk_min / desk_hyk / desk_mkr
desk_tks / desk_ysd / desk_ikr / desk_myj
desk_isz / desk_sek / desk_tym
free space / approach regions
lab equipment targets
```

Real-lab base template currently includes:

```text
front_door, back_door, trash_bin, meeting_table, sofa, tool_box
desk_rxy, desk_ire, desk_tkd, desk_tcb
desk_min, desk_hyk, desk_mkr
desk_tks, desk_ysd, desk_ikr, desk_myj
desk_isz, desk_sek, desk_tym
wall_cabinet_left_upper, wall_cabinet_left_lower
```

Visualize semantic markers:

```bash
cd /home/ruan-x/robot_ws
export ROBOT_MAP_ID=sim_real_lab_001
ros2 run robot_skills semantic_map_marker_node
```

## 4.1. Calibrate Navigation Alignment Zones

Use this only for map-level navigation helper regions such as narrow-passage
entry/exit alignment zones. These are not semantic objects and should not be
saved through `segmentation_tool`.

```bash
cd /home/ruan-x/robot_ws
export ROBOT_MAP_ID=sim_real_lab_001
ros2 run robot_skills navigation_annotation_tool --port 8772
```

Equivalent explicit bundle paths:

```bash
cd /home/ruan-x/robot_ws
ros2 run robot_skills navigation_annotation_tool \
  --map-yaml maps/sim_real_lab_001/nav2.yaml \
  --annotation-yaml maps/sim_real_lab_001/navigation_annotations.yaml \
  --port 8772
```

Open:

```text
http://127.0.0.1:8772
```

For each `narrow_passage`:

```text
1. Select passage.
2. Select or create entry/exit alignment zone.
3. Polygon mode: mark an open area where robot can safely rotate.
4. Pose mode: click the recommended staging pose inside the polygon.
5. Click Face passage, or manually set yaw_deg.
6. Set enabled=true after calibration.
7. Save alignment zone.
```

Use explicit path if you want to bypass map id lookup:

```bash
cd /home/ruan-x/robot_ws
export ROBOT_SEMANTIC_MAP=/home/ruan-x/robot_ws/maps/sim_real_lab_001/semantic_map.yaml
ros2 run robot_skills semantic_map_marker_node
```

Do not copy real-lab data over `semantic_map.yaml` for normal use. Keep
warehouse and lab as separate fixtures, then select one at runtime:

```bash
cd /home/ruan-x/robot_ws
export ROBOT_MAP_ID=sim_real_lab_001
tools/run_agent_dashboard_sim_clean.sh
```

Warehouse fixture check:

```bash
cd /home/ruan-x/robot_ws
export ROBOT_MAP_ID=warehouse
tools/run_agent_dashboard_sim_clean.sh
```

## 5. Test Agent Tasks

Start dashboard:

```bash
cd /home/ruan-x/robot_ws
export ROBOT_MAP_ID=sim_real_lab_001
tools/run_agent_dashboard_sim_clean.sh
```

Open dashboard URL printed by the launch, then test simple commands first:

```text
go to the sofa
go to meeting table
go to rxy table
look left
inspect tool box
list known targets
```

Lightweight real-lab parser / semantic regression:

```bash
cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=/home/ruan-x/robot_ws/src/robot_agent:/home/ruan-x/robot_ws/src/robot_skills:$PYTHONPATH \
  python3 -m pytest -q src/robot_agent/test/test_real_lab_parser_semantic.py
```

Dry-run parser regression:

```bash
cd /home/ruan-x/robot_ws
for lang in en ja zh; do
  python3 src/robot_agent/robot_agent/tools/parser_dry_run_matrix.py \
    --config-dir src/robot_agent/config \
    --cases todo-my/task-agent/dry-run-test/cases-$lang.csv \
    --out todo-my/task-agent/dry-run-test/results-$lang.csv
done
```

Dry-run perception planner:

```bash
cd /home/ruan-x/robot_ws
python3 src/robot_skills/robot_skills/tools/perception_dry_run_matrix.py \
  --cases todo-my/task-agent/dry-run-test/perception/cases-perception.csv \
  --out todo-my/task-agent/dry-run-test/perception/results-perception.csv
```

## 6. Useful Status Commands

```bash
ros2 topic list | sort
ros2 topic hz /point_cloud --window 10
ros2 topic hz /livox/imu --window 20
ros2 topic hz /livox/lidar --window 10
ros2 topic echo /gicp_loc/score --once
ros2 topic info -v /cmd_vel
ros2 topic info -v /sim/target_joint_states
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true
```
