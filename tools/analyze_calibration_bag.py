#!/usr/bin/env python3
"""Summarize event-marked real-robot calibration bags."""

import argparse
import math
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(msg):
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Bag directory or sqlite3 file")
    args = parser.parse_args()

    bag = Path(args.bag).expanduser()
    db = bag if bag.is_file() else next(bag.glob("*.db3"))
    conn = sqlite3.connect(db)
    topics = {
        name: (topic_id, msg_type)
        for topic_id, name, msg_type in conn.execute(
            "select id, name, type from topics"
        )
    }
    first_stamp = conn.execute("select min(timestamp) from messages").fetchone()[0]

    def messages(name):
        if name not in topics:
            return []
        topic_id, msg_type = topics[name]
        cls = get_message(msg_type)
        return [
            ((stamp - first_stamp) / 1e9, deserialize_message(data, cls))
            for stamp, data in conn.execute(
                "select timestamp, data from messages "
                "where topic_id=? order by timestamp",
                (topic_id,),
            )
        ]

    odom = messages("/wheel_odom")
    drive = messages("/drive_odom")
    events = messages("/calibration/event")
    if not odom:
        raise SystemExit("Bag has no /wheel_odom")
    if not events:
        raise SystemExit("Bag has no /calibration/event; use the updated experiment script")

    def nearest(rows, stamp):
        return min(rows, key=lambda row: abs(row[0] - stamp))

    starts = {}
    print(f"bag: {db}")
    for stamp, event in events:
        parts = event.data.split(":")
        if len(parts) != 3:
            continue
        profile, state, label = parts
        if state == "active_start":
            starts[label] = stamp
            continue
        if state != "active_end" or label not in starts:
            continue

        start = starts.pop(label)
        start_odom = nearest(odom, start)
        end_odom = nearest(odom, stamp)
        p0 = start_odom[1].pose.pose.position
        p1 = end_odom[1].pose.pose.position
        yaw0 = yaw(start_odom[1])
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)

        segment_odom = [row for row in odom if start_odom[0] <= row[0] <= end_odom[0]]
        delta_yaw = 0.0
        for (_, previous), (_, current) in zip(segment_odom, segment_odom[1:]):
            step = yaw(current) - yaw(previous)
            delta_yaw += math.atan2(math.sin(step), math.cos(step))

        drive_times = [t for t, _ in drive if start <= t <= stamp]
        gaps = [b - a for a, b in zip(drive_times, drive_times[1:])]
        max_gap = max(gaps, default=0.0)
        print(
            f"{profile:18s} {label:18s} duration={stamp-start:6.2f}s "
            f"forward={forward:+7.3f}m lateral={lateral:+7.3f}m "
            f"yaw={math.degrees(delta_yaw):+8.2f}deg "
            f"drive_hz={len(drive_times)/max(stamp-start, 1e-9):5.2f} "
            f"max_gap={max_gap:5.3f}s"
        )


if __name__ == "__main__":
    main()
