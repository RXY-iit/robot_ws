#!/usr/bin/env python3
"""Read-only wall/simulation-rate and RGB-depth stamp audit; sends no goals."""
import argparse
from bisect import bisect_left
import json
import math
import time


def summarize(samples):
    if not samples:
        return {"count": 0}
    wall_span = samples[-1][0] - samples[0][0]
    sim_span = samples[-1][1] - samples[0][1]
    backwards = sum(b[1] < a[1] for a, b in zip(samples, samples[1:]))
    return {"count": len(samples),
            "wall_hz": (len(samples) - 1) / wall_span if wall_span > 0 else None,
            "source_sim_hz": (len(samples) - 1) / sim_span if sim_span > 0 and not backwards else None,
            "duplicate_stamps": sum(b[1] == a[1] for a, b in zip(samples, samples[1:])),
            "backward_stamps": backwards}


def nearest_skews(left, right):
    stamps = sorted(s[1] for s in right)
    errors = []
    if not stamps:
        return errors
    for _, stamp in left:
        if not stamps[0] <= stamp <= stamps[-1]:
            continue  # Ignore collection boundaries.
        i = bisect_left(stamps, stamp)
        candidates = stamps[max(0, i - 1):i + 1]
        errors.append(min(abs(stamp - value) for value in candidates))
    return sorted(errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30, help="Wall-clock seconds")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be positive and finite")
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, Imu, PointCloud2
    from nav_msgs.msg import Odometry
    rclpy.init(args=[])
    node = rclpy.create_node("b1_sensor_timing_audit")
    samples = {}
    specs = [("clock", "/clock", Clock), ("rgb", "/camera/camera/color/image_raw", Image),
             ("depth", "/camera/camera/aligned_depth_to_color/image_raw", Image),
             ("arm", "/arm_camera/color/image_raw", Image), ("lidar", "/point_cloud", PointCloud2),
             ("imu", "/livox/imu", Imu), ("fast_lio", "/fast_lio/odometry", Odometry)]
    for name, topic, msg_type in specs:
        samples[name] = []
        def receive(msg, key=name):
            stamp = msg.clock if key == "clock" else msg.header.stamp
            samples[key].append((time.monotonic(), stamp.sec + stamp.nanosec / 1e9))
        node.create_subscription(msg_type, topic, receive, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        clock = samples["clock"]
        rtf = ((clock[-1][1] - clock[0][1]) / (clock[-1][0] - clock[0][0])
               if len(clock) > 1 and clock[-1][0] > clock[0][0] else None)
        errors = nearest_skews(samples["rgb"], samples["depth"])
        print(json.dumps({"rtf": rtf, "streams": {k: summarize(v) for k, v in samples.items()},
                          "rgb_depth_nearest_stamp_skew_sec": {
                              "pairs": len(errors), "max": max(errors) if errors else None,
                              "p95": errors[min(len(errors)-1, math.ceil(.95*len(errors))-1)] if errors else None}},
                         indent=2, allow_nan=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
