# Real Robot Calibration Bag Summary — 2026-06-10

## Context

本文件汇总当前较有用的真机 calibration bag。方向约定：

- `forward` / `reverse`: 机器人 `+X` / `-X`
- `left` / `right`: 相对机器人 `+X` 朝向的左/右
- odom 数据来自 `tools/analyze_calibration_bag.py`
- 实测数据来自现场尺量/目测记录

当前 steer home raw ticks：

```cpp
home_pos = {444, 2951, 3213};
```

当前 drive wheel 有效半径：

```text
WHEEL_EFFECTIVE_RADIUS_M = 0.107
```

## Good / Usable Bags

| Bag | Run command | Nominal script command | Wheel odom result | Manual measurement | Assessment |
|---|---|---:|---:|---:|---|
| `calib_straight-forward_real_01_20260610_204430` | `tools/open_test_f_real_windows.sh --cleanup --profile straight-forward --trial 01` | `+X`, `0.15 m/s * 20 s = 3.0 m` | `+2.108 m`, lateral `-0.015 m`, yaw `-0.74 deg` | `+2.17 m` | Good for coarse linear scale. Command chain continuous; real motion about `0.70-0.72` of nominal. |
| `calib_straight-reverse_real_04_20260610_204715` | `tools/open_test_f_real_windows.sh --cleanup --profile straight-reverse --trial 04` | `-X`, `0.15 m/s * 20 s = 3.0 m` | `-2.082 m`, lateral `-0.004 m`, yaw `+0.16 deg` | `-2.15 m` | Good for coarse reverse scale. Similar to forward, so gross direction/sign is consistent. |
| `calib_spin-ccw_real_04_20260610_205327` | `tools/open_test_f_real_windows.sh --cleanup --profile spin-ccw --trial 04` | `+720 deg`, `0.5 rad/s * 25.13 s` | `+521.35 deg`, translation about `5 mm` | not recorded | Good for coarse yaw scale. Translation leakage is small; yaw response is about `0.724` of nominal. |

## Lower Confidence / Diagnostic Bags

| Bag | Run command | Nominal script command | Wheel odom result | Manual measurement | Assessment |
|---|---|---:|---:|---:|---|
| `calib_straight-forward_real_04_20260610_204812` | `tools/open_test_f_real_windows.sh --cleanup --profile straight-forward --trial 04` | `+X`, `0.15 m/s * 20 s = 3.0 m` | `+1.733 m`, lateral `-0.005 m`, yaw `-0.45 deg` | `+1.69 m` | Lower confidence. Command and ROS feedback were continuous, but drive response was much lower than the other straight runs. Useful as evidence of non-smooth / under-speed behavior, not for fitting final scale. |

## Earlier Useful Reference Bags

These were collected before the latest steer home update and before the final coarse drive scale interpretation. They are still useful as debugging references, but should not be mixed directly into final parameter fitting.

| Bag | Run command | Wheel odom result | Notes |
|---|---|---:|---|
| `calib_straight-forward_real_01_20260610_154721` | `tools/open_test_f_real_windows.sh --cleanup --profile straight-forward --trial 01` | `+2.962 m`, yaw `-0.18 deg` | Old home / previous setup. Odom looked clean but physical scale was not yet reconciled. |
| `calib_straight-forward_real_02_20260610_155211` | `tools/open_test_f_real_windows.sh --cleanup --profile straight-forward --trial 02` | `+2.959 m`, yaw `+0.02 deg` | Old setup. |
| `calib_straight-forward_real_03_20260610_155613` | `tools/open_test_f_real_windows.sh --cleanup --profile straight-forward --trial 03` | `+2.958 m`, yaw `-0.09 deg` | Old setup. User observed left bias about `15 cm`. |
| `calib_spin-cw_real_01_20260610_160218` | `tools/open_test_f_real_windows.sh --cleanup --profile spin-cw --trial 01` | `-720.00 deg`, translation near `0` | Useful proof that drive/steer sign mapping was fixed at that time. |
| `calib_spin-ccw_real_02_20260610_160734` | `tools/open_test_f_real_windows.sh --cleanup --profile spin-ccw --trial 02` | `+718.44 deg`, translation near `0` | Old setup. |
| `calib_lateral-left_real_02_20260610_161556` | `tools/open_test_f_real_windows.sh --cleanup --profile lateral-left --trial 02` | `+1.995 m`, forward leak `+0.015 m` | Old setup. Physical observation was better than lateral-left/01. |
| `calib_lateral-right_real_03_20260610_162156` | `tools/open_test_f_real_windows.sh --cleanup --profile lateral-right --trial 03` | `-1.995 m`, forward leak `+0.003 m` | Old setup. |

## Observed Current Behavior

For the latest useful bags:

| Quantity | Observation |
|---|---:|
| Forward manual / nominal | `2.17 / 3.0 = 0.723` |
| Forward odom / nominal | `2.108 / 3.0 = 0.703` |
| Reverse manual / nominal | `2.15 / 3.0 = 0.717` |
| Reverse odom / nominal | `2.082 / 3.0 = 0.694` |
| CCW yaw odom / nominal | `521.35 / 720 = 0.724` |
| Recommended coarse global motion scale | `0.71` |

The command chain appears continuous in the latest bags:

- `/cmd_vel` reached the expected command values.
- `/steer_odom` followed `/steer_angle`.
- `/drive_odom` published around `12.6-13 Hz`.
- `/om_state0.state_error` stayed `0`.

The non-smoothness is therefore more likely in the drive execution layer than in ROS topic continuity:

- BLV-R drive response lower than command.
- Possible power / driver limit / mechanical friction / floor-load interaction.
- One latest forward run (`204812`) had much lower distance and should be treated as diagnostic rather than clean calibration data.

## Coarse Sim-Real Recommendation

For the next step, do not wait for final precision calibration. Use this coarse model first:

```yaml
motion_scale: 0.71
wheel_effective_radius_m: 0.107
steer_home_raw_ticks: [1671, 2951, 3213]
```

Use this only to make sim and real roughly comparable. Final calibration should be redone after drive smoothness / speed tracking is improved.
