#!/usr/bin/env python3
"""Export a GLIM dump directory to a binary little-endian PLY point cloud."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np


def read_t_world_origin(data_txt: Path) -> np.ndarray:
    lines = data_txt.read_text(errors="replace").splitlines()
    for i, line in enumerate(lines):
        if line.strip() != "T_world_origin:":
            continue
        rows = []
        for row in lines[i + 1 : i + 5]:
            values = [float(x) for x in row.split()]
            if len(values) != 4:
                raise ValueError(f"Bad transform row in {data_txt}: {row}")
            rows.append(values)
        return np.asarray(rows, dtype=np.float32)
    raise ValueError(f"T_world_origin not found in {data_txt}")


def iter_submaps(dump_dir: Path):
    for path in sorted(dump_dir.iterdir()):
        if path.is_dir() and re.fullmatch(r"\d{6}", path.name):
            points = path / "points_compact.bin"
            data = path / "data.txt"
            if points.exists() and data.exists():
                yield path, points, data


def load_points(points_bin: Path) -> np.ndarray:
    raw = np.fromfile(points_bin, dtype="<f4")
    if raw.size % 3 != 0:
        raise ValueError(f"{points_bin} float count is not divisible by 3")
    return raw.reshape((-1, 3))


def write_binary_ply(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")
    with path.open("wb") as f:
        f.write(header)
        points.astype("<f4", copy=False).tofile(f)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump", help="GLIM dump directory containing 000000/ points_compact.bin submaps")
    parser.add_argument("--output", required=True, help="Output binary little-endian PLY")
    parser.add_argument(
        "--voxel",
        type=float,
        default=0.05,
        help="Voxel size for output downsampling in meters; <=0 disables downsampling",
    )
    parser.add_argument(
        "--max-submaps",
        type=int,
        default=0,
        help="Debug limit; 0 exports all submaps",
    )
    return parser.parse_args()


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0.0 or len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(keep)]


def main() -> None:
    args = parse_args()
    dump_dir = Path(args.dump)
    output = Path(args.output)
    if not dump_dir.is_dir():
        raise SystemExit(f"Missing dump directory: {dump_dir}")

    chunks = []
    submaps = list(iter_submaps(dump_dir))
    if args.max_submaps > 0:
        submaps = submaps[: args.max_submaps]
    if not submaps:
        raise SystemExit(f"No GLIM submaps found in {dump_dir}")

    total_in = 0
    for submap, points_bin, data_txt in submaps:
        transform = read_t_world_origin(data_txt)
        points = load_points(points_bin)
        total_in += len(points)
        rot = transform[:3, :3]
        trans = transform[:3, 3]
        world = points @ rot.T + trans
        chunks.append(world.astype(np.float32, copy=False))
        print(f"{submap.name}: {len(points)} points")

    all_points = np.vstack(chunks)
    downsampled = voxel_downsample(all_points, args.voxel)
    write_binary_ply(output, downsampled)
    print(f"input_points={total_in}")
    print(f"output_points={len(downsampled)}")
    print(f"output={output}")


if __name__ == "__main__":
    main()
