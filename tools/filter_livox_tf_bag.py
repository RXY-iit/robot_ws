#!/usr/bin/env python3
"""Rewrite a rosbag2 bag while removing conflicting TF edges.

Default behavior removes dynamic /tf transforms whose child frame is
``livox_frame``. For AMCL bag replay, also remove ``map -> odom`` so AMCL is
the only publisher of that localization transform.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def storage_options(uri: Path) -> rosbag2_py.StorageOptions:
    return rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3")


def converter_options() -> rosbag2_py.ConverterOptions:
    return rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )


def parse_edge(text: str) -> tuple[str, str]:
    if ":" not in text:
        raise argparse.ArgumentTypeError("edge must be formatted as parent:child, e.g. map:odom")
    parent, child = text.split(":", 1)
    if not parent or not child:
        raise argparse.ArgumentTypeError("edge must include both parent and child")
    return parent, child


def should_drop_transform(tf, topic: str, child_frames: set[str], drop_edges: set[tuple[str, str]]) -> bool:
    if topic == "/tf" and tf.child_frame_id in child_frames:
        return True
    return (tf.header.frame_id, tf.child_frame_id) in drop_edges


def rewrite_bag(
    input_uri: Path,
    output_uri: Path,
    child_frames: set[str],
    drop_edges: set[tuple[str, str]],
    force: bool,
) -> None:
    if output_uri.exists():
        if not force:
            raise SystemExit(f"Output already exists: {output_uri} (use --force to replace it)")
        shutil.rmtree(output_uri)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options(input_uri), converter_options())

    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options(output_uri), converter_options())

    topics = reader.get_all_topics_and_types()
    type_by_topic = {topic.name: topic.type for topic in topics}
    for topic in topics:
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic.name,
                type=topic.type,
                serialization_format=topic.serialization_format,
                offered_qos_profiles=topic.offered_qos_profiles,
            )
        )

    tf_msg_type = get_message("tf2_msgs/msg/TFMessage")
    removed_by_child = 0
    removed_by_edge = 0
    written = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic in {"/tf", "/tf_static"}:
            msg = deserialize_message(data, tf_msg_type)
            kept = []
            for tf in msg.transforms:
                if should_drop_transform(tf, topic, child_frames, drop_edges):
                    if (tf.header.frame_id, tf.child_frame_id) in drop_edges:
                        removed_by_edge += 1
                    else:
                        removed_by_child += 1
                    continue
                kept.append(tf)
            if not kept:
                continue
            msg.transforms = kept
            data = serialize_message(msg)

        writer.write(topic, data, timestamp)
        written += 1

    writer.close()

    print(f"Input:   {input_uri}")
    print(f"Output:  {output_uri}")
    print(f"Topics:  {len(type_by_topic)}")
    print(f"Written messages: {written}")
    print(f"Removed dynamic /tf transforms with child_frame_id in {sorted(child_frames)}: {removed_by_child}")
    print(f"Removed exact TF edges {sorted(drop_edges)} from /tf and /tf_static: {removed_by_edge}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Copy a rosbag2 sqlite bag and remove dynamic /tf transforms for LiDAR "
            "frames that conflict with the robot's static base_link->LiDAR transform."
        )
    )
    parser.add_argument("input_bag", type=Path, help="Input rosbag2 directory")
    parser.add_argument("output_bag", type=Path, help="Output rosbag2 directory to create")
    parser.add_argument(
        "--child-frame",
        action="append",
        default=["livox_frame"],
        help="TF child_frame_id to remove from /tf. Repeat for more frames.",
    )
    parser.add_argument(
        "--drop-edge",
        action="append",
        type=parse_edge,
        default=[],
        help="Exact TF edge parent:child to remove from /tf and /tf_static. Repeat for more edges.",
    )
    parser.add_argument("--force", action="store_true", help="Replace output_bag if it exists")
    args = parser.parse_args()

    rewrite_bag(args.input_bag, args.output_bag, set(args.child_frame), set(args.drop_edge), args.force)


if __name__ == "__main__":
    main()
