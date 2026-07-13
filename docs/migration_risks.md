# Migration Risks

Generated: 2026-07-13

## Overall Risk

The workspace is **整理后适合** for sim2real. The main risk is not architectural; the layered design is sound. The risk is that simulation data, hardcoded paths, and sim-tuned parameters are mixed with real-ready packages.

## Safety Risks

| Risk | Severity | Evidence | Mitigation |
|---|---|---|---|
| Real robot can move if `/cmd_vel` is bypassed | High | Direct sim mode uses `/sim/cmd_vel`; real must not copy this pattern. | Enforce `/cmd_vel_raw -> safety_layer -> /cmd_vel`; audit publishers. |
| Depth camera is front-biased | High | MPPI comments prefer forward motion due to front depth camera. | Limit reverse/lateral motion until rear/side safety coverage exists. |
| Estop is partly ROS-topic based | High | `/emergency_stop` and Joy button are software-level. | Add hardware estop and motor-driver disable path. |
| Safety limits are not real-calibrated | Medium | Current values are launch literals. | Start with lower real limits and increment after stop-distance tests. |
| Motion policy may override Nav2 behavior near goals | Medium | `nav_mode_switch_node` applies close-range policies. | Test with policy disabled/enabled in real before autonomous missions. |

## Interface Risks

| Risk | Severity | Evidence | Mitigation |
|---|---|---|---|
| `PointCloud2` vs Livox `CustomMsg` mismatch | High | Launch supports both via `with_fast_lio`; `/livox/lidar` type changes by xfer format. | Make LiDAR mode explicit per launch and document topic type. |
| QoS mismatch on sensor topics | Medium | Custom nodes use default QoS; sensor drivers may use sensor-data QoS. | Run `ros2 topic info -v` and adapt subscriber QoS. |
| `base_link` vs `base_footprint` ambiguity | Medium | Some docs mention both; Nav2 uses `base_footprint`. | Make `base_footprint` primary 2D contract and publish consistent TF. |
| `/clock` leaks into real launch | Medium | Sim guides use `use_sim_time:=true`; launch defaults are mixed. | Real launch defaults must set `use_sim_time:=false`. |
| Hardcoded absolute paths | Medium | Map/BT/model paths include `/home/ruan-x/robot_ws`. | Use launch args/package share/env profile. |

## Parameter Risks

| Risk | Severity | Evidence | Mitigation |
|---|---|---|---|
| Sim map is used on real robot | High | Localization defaults point to sim map bundle. | Create real map bundle and make real launch require it. |
| GICP threshold too loose | High | Config says sim uses wider `fitness_score_threshold: 0.50`. | Calibrate real score threshold from real logs. |
| Footprint mismatch | High | Footprint is manually encoded in Nav2 config. | Measure robot and verify clearance with RViz footprint. |
| Camera obstacle parameters do not match real camera | Medium | Costmap uses `/camera/depth/points` with height/range gates. | Tune with real depth data and representative obstacles. |
| MPPI lateral/reverse behavior overfits sim | Medium | Several comments document sim/real-lab rounds. | Use staged real tests and conservative limits. |

## Deployment Risks

| Risk | Severity | Evidence | Mitigation |
|---|---|---|---|
| Repository is very large | High | Workspace is about 30 GB; large bags/assets/models exist. | Split runtime repo, sim assets, bags, and models. |
| Dirty worktree | Medium | Many modified/untracked files. | Commit or branch current sim checkpoint before real migration. |
| Third-party packages are vendored without clear pinning | Medium | `fast_lio`, `realsense-ros`, `DynamixelSDK`, `livox_ros_driver2`, `om_modbus_master`. | Pin versions/submodules and document patches. |
| Build/install/log directories in workspace | Low/Medium | Large generated directories present. | Ensure `.gitignore` excludes generated outputs. |
| Model file in repo | Medium | `models/perception/yolo11s.pt` is ~19 MB. | Use model artifact storage or Git LFS. |

## Agent/Skill Risks

| Risk | Severity | Evidence | Mitigation |
|---|---|---|---|
| Some skills are not real-supported | Medium | `detect_known_object`, `set_camera_pose`, `move_lift_bounded`, `recover_localization_spin` marked real unsupported. | Enforce registry at runtime in `real` mode. |
| Agent cancels on health BAD but health model is still evolving | Medium | Agent monitors GICP, cmd_vel, Nav2 lifecycle. | Add real hardware and sensor freshness health components. |
| Semantic target map is sim-derived | Medium | `maps/sim_real_lab_001` and skill semantic configs are active. | Build real semantic map after real mapping. |

