# Git Repository Audit

Generated: 2026-07-13

## Current Git Status

Branch:

- `main`
- ahead of `origin/main` by 3 commits

Recent commits:

- `4cd67ae` checkpoint: save workspace before agent process
- `ef976f3` checkpoint sim navigation workspace
- `5840056` tag `sim-mapping-true-imu-20260605` checkpoint sim mapping true imu
- `e019b24` `origin/main` add todo md
- `f594ec3` Add Isaac Sim workspace structure and clone guide

Worktree:

- Many modified files in navigation, localization, safety, sim, RViz, tools, and notes.
- Many untracked files/directories, including:
  - `models/`
  - `src/robot_agent/`
  - `src/robot_skills/`
  - `src/robot_sim/usd/scenes/real_lab/`
  - `todo-my/task-agent/dry-run-test/`
  - `todo-my/task-sim2real/`
  - `tools/local_llm/`
  - `tools/run_agent_dashboard_sim_clean.sh`

## Large Files Observed

Largest categories:

- Whole workspace size: about `30G`.
- Git object packs include one pack around `3.4G`.
- Simulation bags:
  - `sim_bag/sim_factory_mid360_trueimu_d455_004/sim_factory_mid360_trueimu_d455_004_0.db3`: about `8.5G`
  - `sim_bag/sim_factory_mid360_trueimu_d455_004/*.zstd`: about `3.6G`
  - `sim_bag/sim_factory_mid360_trueimu_d455_003/*.zstd`: about `5.2G`
  - `sim_bag/sim_real_lab_001/*.zstd`: about `4.0G`
- USD/asset/model files:
  - `src/robot_sim/usd/robot_sim_flattened.usda`: about `31M`
  - many `models/omniverse_assets/**` USD/textures above `10M`
  - `models/perception/yolo11s.pt`: about `19M`
- Third-party docs/meshes:
  - `src/fast_lio/doc/*.gif`: about `35-52M`
  - `src/realsense-ros/realsense2_description/meshes/*.dae`: about `15-33M`

## Repository Split Recommendation

Recommended structure:

1. Runtime robot repo
   - `src/robot_agent`
   - `src/robot_skills`
   - `src/nav_pkg`
   - `src/localization_pkg`
   - `src/safety_layer`
   - `src/robot_bringup`
   - `src/robot_description`
   - `src/omni_base_driver`
   - `src/my_messages`
   - minimal real hardware drivers or submodules

2. Simulation repo or artifact bundle
   - `src/robot_sim`
   - `src/robot_sim/usd/**`
   - `models/omniverse_assets/**`
   - sim launch tooling

3. Data/artifact storage
   - `sim_bag/**`
   - large map dumps
   - `.db3`, `.zstd`, `.ply` if large
   - perception model weights

4. Third-party source management
   - `fast_lio`
   - `livox_ros_driver2`
   - `realsense-ros`
   - `DynamixelSDK`
   - `om_modbus_master_V201`

Use Git submodules, vcstool `.repos`, or a vendor lockfile. Avoid silently modifying third-party packages without patch notes.

## Immediate Git Actions

Recommended before real migration:

1. Create a migration branch from current state, for example `sim2real-prep-20260713`.
2. Commit the generated `docs/*.md` audit files.
3. Decide whether untracked `robot_agent` and `robot_skills` should be part of main runtime repo.
4. Add or verify `.gitignore` entries for:
   - `build/`
   - `install/`
   - `log/`
   - `sim_bag/`
   - `*.db3`
   - `*.db3.zstd`
   - large generated map dumps
5. Move `models/omniverse_assets` and large sim bags to artifact storage or Git LFS if they must stay discoverable.
6. Tag the current sim-success state after docs are committed.

## Risk

Current repo status is usable for development, but not clean enough for deterministic real robot deployment. The most important cleanup is separating source code from large generated simulation data.

