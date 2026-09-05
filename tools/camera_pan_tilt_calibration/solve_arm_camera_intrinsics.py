#!/usr/bin/env python3
"""Solve SO101 wrist-camera intrinsics from one captured session."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='output/session_* directory')
    parser.add_argument('--minimum-views', type=int, default=10)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--force', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session.expanduser().resolve()
    if not session.is_dir():
        raise SystemExit(f'Not a session directory: {session}')
    if args.minimum_views < 3:
        raise SystemExit('--minimum-views must be at least 3')

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    sample_names: list[str] = []
    image_size: tuple[int, int] | None = None

    for sample in sorted(session.glob('sample_*')):
        corner_path = sample / 'arm_charuco.json'
        metadata_path = sample / 'metadata.json'
        if not corner_path.is_file() or not metadata_path.is_file():
            continue
        corners = json.loads(corner_path.read_text(encoding='utf-8'))['corners']
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        if len(corners) < 4:
            continue
        size = (
            int(metadata['images']['arm_color']['width']),
            int(metadata['images']['arm_color']['height']),
        )
        if image_size is None:
            image_size = size
        elif image_size != size:
            raise SystemExit(
                f'Mixed arm image sizes are not accepted: {image_size} vs {size} in {sample.name}'
            )
        object_points.append(
            np.asarray([entry['board_xyz_m'] for entry in corners], dtype=np.float32)
        )
        image_points.append(
            np.asarray([entry['pixel_uv'] for entry in corners], dtype=np.float32)
        )
        sample_names.append(sample.name)

    if len(object_points) < args.minimum_views or image_size is None:
        raise SystemExit(
            f'Need at least {args.minimum_views} valid views; found {len(object_points)}. '
            'Use varied board positions, distances and tilts across the image.'
        )

    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    if not np.all(np.isfinite(camera_matrix)) or not np.all(np.isfinite(distortion)):
        raise SystemExit('Calibration produced non-finite values; dataset is degenerate')

    per_view_error = []
    for name, obj, img, rvec, tvec in zip(
        sample_names, object_points, image_points, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        error = np.linalg.norm(img - projected.reshape(-1, 2), axis=1)
        per_view_error.append({
            'sample': name,
            'mean_reprojection_error_px': float(np.mean(error)),
            'max_reprojection_error_px': float(np.max(error)),
            'corner_count': int(len(obj)),
        })

    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    distortion_values = [float(value) for value in distortion.reshape(-1)]
    camera_yaml = {
        'image_width': image_size[0],
        'image_height': image_size[1],
        'camera_name': 'arm_camera',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [float(value) for value in camera_matrix.reshape(-1)],
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': len(distortion_values),
            'data': distortion_values,
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        },
    }

    output = (args.output or session / 'arm_camera_calibrated.yaml').resolve()
    report = session / 'arm_camera_intrinsics_report.json'
    for path in (output, report):
        if path.exists() and not args.force:
            raise SystemExit(f'Refusing to overwrite {path}; pass --force after review')
    output.write_text(yaml.safe_dump(camera_yaml, sort_keys=False), encoding='utf-8')
    report.write_text(
        json.dumps({
            'solver': 'cv2.calibrateCamera using detected ChArUco board corners',
            'session': str(session),
            'view_count': len(object_points),
            'image_size': list(image_size),
            'opencv_rms_reprojection_error_px': float(rms),
            'per_view': per_view_error,
            'output_camera_info': str(output),
            'scope': (
                'Camera intrinsics only. This does not solve wrist-camera hand-eye '
                'or base_link-to-arm/world transforms.'
            ),
        }, indent=2) + '\n',
        encoding='utf-8',
    )
    print(f'Wrote {output}')
    print(f'Wrote {report}')
    print(f'views={len(object_points)} OpenCV_RMS={rms:.6f}px')
    return 0


if __name__ == '__main__':
    sys.exit(main())
