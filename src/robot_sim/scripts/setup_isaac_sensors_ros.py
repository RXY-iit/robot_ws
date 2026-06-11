"""Start mapping Isaac Sim sensor ROS2 publishers for robot_sim.

Run from Isaac Sim Script Editor before Play. This performs only the safe
pre-Play cleanup and true IMU setup. Press Play afterward, then run the MID360
setup exactly once.

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_sensors_ros.py").read())
"""

import os
import runpy
import omni.timeline


SCRIPT_DIR = "/home/ruan-x/robot_ws/src/robot_sim/scripts"
SIM_SCRIPT_DIR = f"{SCRIPT_DIR}/sim"

SUPPORTED_LIDAR_MODES = {
    "mid360_like",
    "os1_128",
    "os1_32",
    "hesai_xt32",
    "sick_multiscan136",
}

# Supported values:
#   mid360_like       Experimental Livox/MID360-like custom profile. This is
#                     the current localization mainline because its raw
#                     /point_cloud already has useful vertical spread.
#   os1_128           Official Ouster OS1 128ch profile, kept as stable
#                     fallback/baseline.
#   os1_32, hesai_xt32, sick_multiscan136
#                     Additional official Isaac profiles for comparison.
#
# Isaac Script Editor keeps globals between executions. If an old run left
# LIDAR_MODE="mid360" or another removed mode in the Python console, ignore it
# and fall back to mid360_like instead of failing the combined setup.
LIDAR_MODE = globals().get("LIDAR_MODE", "mid360_like")
if LIDAR_MODE not in SUPPORTED_LIDAR_MODES:
    print(
        f"Warning: unsupported LIDAR_MODE={LIDAR_MODE!r}; "
        "falling back to 'mid360_like'."
    )
    LIDAR_MODE = "mid360_like"


def _run_script(name, extra_globals=None):
    path = os.path.join(SCRIPT_DIR, name)
    globals_for_script = {"__name__": "__main__"}
    if extra_globals:
        globals_for_script.update(extra_globals)
    runpy.run_path(path, init_globals=globals_for_script)


def _run_sim_script(name, extra_globals=None):
    path = os.path.join(SIM_SCRIPT_DIR, name)
    globals_for_script = {"__name__": "__main__"}
    if extra_globals:
        globals_for_script.update(extra_globals)
    runpy.run_path(path, init_globals=globals_for_script)


def main():
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError(
            "Run setup_isaac_sensors_ros.py BEFORE Play. Runtime RTX cleanup is "
            "unsafe and can trigger CUDA error 700."
        )

    print("Starting robot_sim safe pre-Play mapping sensor setup...")
    _run_sim_script("cleanup_foreign_articulations.py")
    _run_sim_script("cleanup_isaac_livox_imu_ros.py")
    _run_sim_script("cleanup_isaac_mid360_ros.py")
    _run_sim_script("setup_isaac_livox_imu_ros.py")
    print("Pre-Play setup ready.")
    print("Next: press Play, then run setup_isaac_mid360_like_ros.py exactly once.")


main()
