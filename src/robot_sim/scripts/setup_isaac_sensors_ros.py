"""Start mapping Isaac Sim sensor ROS2 publishers for robot_sim.

Run from Isaac Sim Script Editor after opening full_warehouse_r.usd. Run the
pre-Play cleanup scripts first, press Play, then execute this file:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_sensors_ros.py").read())
"""

import os
import runpy


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
    print("Starting robot_sim Isaac mapping ROS2 sensors...")
    _run_sim_script("cleanup_foreign_articulations.py")
    _run_sim_script("cleanup_isaac_livox_imu_ros.py")
    _run_sim_script("cleanup_isaac_mid360_ros.py")
    _run_sim_script("setup_isaac_livox_imu_ros.py")
    if LIDAR_MODE == "mid360_like":
        _run_script("setup_isaac_mid360_like_ros.py")
    else:
        _run_script("setup_isaac_mid360_ros.py", {"LIDAR_MODE": LIDAR_MODE})
    print("robot_sim Isaac mapping ROS2 sensors ready.")
    print(f"LiDAR mode: {LIDAR_MODE}")
    print("Expected topics:")
    print("  /livox/imu")
    print("  /point_cloud")
    print("  /livox/lidar_pc2_accumulated")
    print("  /livox/lidar")


main()
