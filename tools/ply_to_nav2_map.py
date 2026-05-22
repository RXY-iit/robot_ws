#!/usr/bin/env python3
"""Convert a simple PLY point cloud into a Nav2 occupancy map.

用途:
  GLIM viewer 导出的 3D PLY 点云不能直接给 Nav2 / AMCL 使用。
  这个脚本会读取 PLY 里的 x/y/z 点，把指定高度范围内的点投影到 XY 平面，
  生成 Nav2 map_server 可以读取的 2D occupancy map:

    maps/l402_2d_map.pgm
    maps/l402_2d_map.yaml

常用命令:
  /usr/bin/python3 tools/ply_to_nav2_map.py \
    /home/matsunaga-h/robot_ws/maps/saved-map/l402-points/l402_points \
    --output maps/l402_2d_map \
    --resolution 0.05 \
    --z-min -0.3 \
    --z-max 1.5 \
    --min-hits 1 \
    --inflate-radius 0.12

参数说明:
  input:
    输入 PLY 文件。这里是 GLIM viewer export 出来的 l402_points。

  --output:
    输出文件名前缀，不写扩展名。比如 maps/l402_2d_map 会生成:
      maps/l402_2d_map.pgm
      maps/l402_2d_map.yaml

  --resolution:
    2D 地图分辨率，单位 m/pixel。0.05 表示每个像素 5 cm。
    数值越小地图越细，但文件更大，也更容易保留噪声。

  --z-min / --z-max:
    高度过滤范围，只把这个 z 范围内的点当作 2D 地图障碍物候选。
    如果地板被画成障碍物，尝试提高 --z-min，例如 0.05 或 0.1。
    如果天花板/高处架子被画进地图，尝试降低 --z-max，例如 1.0 或 1.2。

  --padding:
    在点云外侧给地图增加的空白边界，单位 m。默认 1.0。

  --min-hits:
    同一个 grid cell 内至少有多少点才标记为 occupied。
    1 最敏感，容易保留细小障碍物，也容易保留噪声。
    2 或 3 更干净，但可能漏掉稀疏墙面。

  --inflate-radius:
    生成地图时把障碍物向外膨胀的半径，单位 m。
    0.12 表示障碍物会变粗一点，给机器人留安全距离。

注意:
  这个脚本生成的是 first-pass 2D map。它没有做 ray tracing free-space 推断，
  非障碍物区域会被直接标成 free，适合先让 map_server / AMCL / Nav2 跑起来。
  最终导航用地图还需要在 RViz 中检查，并根据效果调整 z filter 和 inflation。
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path

import numpy as np


PLY_TYPES = {
    "char": ("i1", 1),
    "uchar": ("u1", 1),
    "int8": ("i1", 1),
    "uint8": ("u1", 1),
    "short": ("<i2", 2),
    "ushort": ("<u2", 2),
    "int16": ("<i2", 2),
    "uint16": ("<u2", 2),
    "int": ("<i4", 4),
    "uint": ("<u4", 4),
    "int32": ("<i4", 4),
    "uint32": ("<u4", 4),
    "float": ("<f4", 4),
    "float32": ("<f4", 4),
    "double": ("<f8", 8),
    "float64": ("<f8", 8),
}


def read_ply_vertices(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        header_lines: list[str] = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PLY header ended unexpectedly")
            decoded = line.decode("ascii").strip()
            header_lines.append(decoded)
            if decoded == "end_header":
                break

        if header_lines[0] != "ply":
            raise ValueError("Not a PLY file")
        if "format binary_little_endian 1.0" not in header_lines:
            raise ValueError("Only binary_little_endian PLY is supported")

        vertex_count = None
        properties: list[tuple[str, str]] = []
        in_vertex = False
        for line in header_lines:
            parts = line.split()
            if not parts:
                continue
            if parts[:2] == ["element", "vertex"]:
                vertex_count = int(parts[2])
                in_vertex = True
                continue
            if parts[0] == "element" and parts[1] != "vertex":
                in_vertex = False
            if in_vertex and parts[0] == "property":
                if parts[1] == "list":
                    raise ValueError("List properties are not supported for vertex data")
                properties.append((parts[2], parts[1]))

        if vertex_count is None:
            raise ValueError("PLY has no vertex element")

        dtype = np.dtype([(name, PLY_TYPES[prop_type][0]) for name, prop_type in properties])
        vertices = np.frombuffer(f.read(vertex_count * dtype.itemsize), dtype=dtype, count=vertex_count)

    for required in ("x", "y", "z"):
        if required not in vertices.dtype.names:
            raise ValueError(f"PLY vertex is missing '{required}'")

    return np.column_stack([vertices["x"], vertices["y"], vertices["z"]]).astype(np.float32)


def inflate_obstacles(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return occupied
    inflated = occupied.copy()
    ys, xs = np.nonzero(occupied)
    height, width = occupied.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            ny = ys + dy
            nx = xs + dx
            ok = (0 <= nx) & (nx < width) & (0 <= ny) & (ny < height)
            inflated[ny[ok], nx[ok]] = True
    return inflated


def write_nav2_map(points: np.ndarray, args: argparse.Namespace) -> None:
    z = points[:, 2]
    points = points[(z >= args.z_min) & (z <= args.z_max)]
    if len(points) == 0:
        raise SystemExit("No points remain after z filtering; adjust --z-min/--z-max")

    min_x = float(np.min(points[:, 0]) - args.padding)
    max_x = float(np.max(points[:, 0]) + args.padding)
    min_y = float(np.min(points[:, 1]) - args.padding)
    max_y = float(np.max(points[:, 1]) + args.padding)

    width = max(1, int(math.ceil((max_x - min_x) / args.resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / args.resolution)))

    ix = np.floor((points[:, 0] - min_x) / args.resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - min_y) / args.resolution).astype(np.int64)
    ok = (0 <= ix) & (ix < width) & (0 <= iy) & (iy < height)

    hits = np.zeros((height, width), dtype=np.uint16)
    np.add.at(hits, (iy[ok], ix[ok]), 1)
    occupied = hits >= args.min_hits
    occupied = inflate_obstacles(occupied, int(round(args.inflate_radius / args.resolution)))

    image = np.full((height, width), 254, dtype=np.uint8)
    image[occupied] = 0
    image = np.flipud(image)

    output = Path(args.output)
    pgm_path = output.with_suffix(".pgm")
    yaml_path = output.with_suffix(".yaml")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)

    with pgm_path.open("wb") as f:
        f.write(f"P5\n# generated from {Path(args.input).name}\n{width} {height}\n255\n".encode())
        f.write(image.tobytes())

    yaml_path.write_text(
        f"image: {pgm_path.name}\n"
        f"mode: trinary\n"
        f"resolution: {args.resolution:.6f}\n"
        f"origin: [{min_x:.6f}, {min_y:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )

    print(f"Input points after filter: {len(points)}")
    print(f"Map size: {width} x {height}")
    print(f"Origin: {min_x:.3f}, {min_y:.3f}, 0.0")
    print(f"Occupied cells: {int(np.count_nonzero(occupied))}")
    print(f"Wrote: {pgm_path}")
    print(f"Wrote: {yaml_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="Input binary little-endian PLY file")
    parser.add_argument("--output", default="maps/l402_2d_map")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--z-min", type=float, default=-0.3)
    parser.add_argument("--z-max", type=float, default=1.5)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--min-hits", type=int, default=1)
    parser.add_argument("--inflate-radius", type=float, default=0.12)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    points = read_ply_vertices(Path(args.input))
    write_nav2_map(points, args)


if __name__ == "__main__":
    main()
