#!/usr/bin/env python3
"""Fast, stateful wrist-camera target tracking for Touch Anything.

SAM supplies the operator-confirmed initial mask.  This module deliberately
contains no ROS or motion code: pyramidal KLT propagates that mask frame to
frame, while ORB/RANSAC is an event-triggered geometric recovery path.  A
failed KLT and failed ORB recovery is reported to the caller; only the caller
may request semantic recovery and human confirmation.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class TrackerConfig:
    max_features: int = 160
    grid_rows: int = 4
    grid_cols: int = 4
    quality_level: float = 0.01
    min_distance_px: float = 5.0
    mask_erode_px: int = 3
    tip_exclusion_radius_px: int = 22
    lk_window_px: int = 21
    lk_levels: int = 3
    lk_fb_error_px: float = 1.5
    # A small, low-texture elevator-button mask can legitimately expose only
    # 6--9 Shi-Tomasi corners. Six correspondences still leave redundancy for
    # a 4-DOF partial-affine fit and RANSAC; the inlier-ratio and spatial-
    # coverage gates below continue to reject geometrically weak motion.
    minimum_tracks: int = 6
    minimum_inlier_ratio: float = 0.50
    minimum_coverage_ratio: float = 0.10
    frame_area_ratio_min: float = 0.35
    frame_area_ratio_max: float = 3.00
    orb_features: int = 700
    orb_ratio_test: float = 0.75
    orb_minimum_matches: int = 10
    rolling_keyframe_interval_frames: int = 15


def _as_mask(mask: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    result = np.asarray(mask, dtype=np.uint8)
    if result.shape != shape:
        raise ValueError(f"mask shape {result.shape} != image shape {shape}")
    result = (result > 0).astype(np.uint8)
    if int(np.count_nonzero(result)) < 20:
        raise ValueError("tracking mask has fewer than 20 pixels")
    return result


def _centroid(mask: np.ndarray) -> np.ndarray:
    moments = cv2.moments(mask, binaryImage=True)
    if moments["m00"] <= 0.0:
        raise ValueError("tracking mask is empty")
    return np.array(
        [moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]],
        dtype=np.float64,
    )


class ContinuousTargetTracker:
    """Human-initialized target tracker with immutable and rolling references."""

    def __init__(self, config: TrackerConfig | None = None) -> None:
        self.config = config or TrackerConfig()
        self._orb = cv2.ORB_create(nfeatures=self.config.orb_features)
        self.reset()

    def reset(self) -> None:
        self.reference_rgb: np.ndarray | None = None
        self.reference_gray: np.ndarray | None = None
        self.reference_mask: np.ndarray | None = None
        self.reference_keypoints = []
        self.reference_descriptors: np.ndarray | None = None
        self.rolling_gray: np.ndarray | None = None
        self.rolling_mask: np.ndarray | None = None
        self.rolling_keypoints = []
        self.rolling_descriptors: np.ndarray | None = None
        self.rolling_frame_index = 0
        self.previous_gray: np.ndarray | None = None
        self.current_mask: np.ndarray | None = None
        self.points: np.ndarray | None = None
        self.frame_index = 0
        self.last_result: dict[str, Any] | None = None

    @property
    def active(self) -> bool:
        return self.previous_gray is not None and self.current_mask is not None

    @staticmethod
    def _gray(rgb: np.ndarray) -> np.ndarray:
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("tracker requires an RGB image")
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    def _feature_mask(
        self,
        mask: np.ndarray,
        excluded_tip_uv: tuple[float, float] | None,
    ) -> np.ndarray:
        valid = (mask > 0).astype(np.uint8) * 255
        erode = max(0, int(self.config.mask_erode_px))
        if erode:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (erode * 2 + 1, erode * 2 + 1)
            )
            eroded = cv2.erode(valid, kernel)
            if int(np.count_nonzero(eroded)) >= self.config.minimum_tracks:
                valid = eroded
        if excluded_tip_uv is not None:
            cv2.circle(
                valid,
                (int(round(excluded_tip_uv[0])), int(round(excluded_tip_uv[1]))),
                max(0, int(self.config.tip_exclusion_radius_px)),
                0,
                thickness=-1,
            )
        return valid

    def _detect_points(
        self,
        gray: np.ndarray,
        mask: np.ndarray,
        excluded_tip_uv: tuple[float, float] | None,
    ) -> np.ndarray | None:
        valid = self._feature_mask(mask, excluded_tip_uv)
        height, width = gray.shape
        rows = max(1, int(self.config.grid_rows))
        cols = max(1, int(self.config.grid_cols))
        per_cell = max(2, int(math.ceil(self.config.max_features / (rows * cols))))
        collected: list[np.ndarray] = []
        for row in range(rows):
            y0, y1 = height * row // rows, height * (row + 1) // rows
            for col in range(cols):
                x0, x1 = width * col // cols, width * (col + 1) // cols
                cell_mask = valid[y0:y1, x0:x1]
                if not np.any(cell_mask):
                    continue
                points = cv2.goodFeaturesToTrack(
                    gray[y0:y1, x0:x1],
                    maxCorners=per_cell,
                    qualityLevel=float(self.config.quality_level),
                    minDistance=float(self.config.min_distance_px),
                    mask=cell_mask,
                    blockSize=5,
                    useHarrisDetector=False,
                )
                if points is None:
                    continue
                points[:, 0, 0] += float(x0)
                points[:, 0, 1] += float(y0)
                collected.append(points)
        if not collected:
            return None
        merged = np.concatenate(collected, axis=0)
        return merged[: self.config.max_features].astype(np.float32)

    def initialize(
        self,
        rgb: np.ndarray,
        mask: np.ndarray,
        *,
        excluded_tip_uv: tuple[float, float] | None = None,
        preserve_identity_reference: bool = False,
    ) -> dict[str, Any]:
        image = np.asarray(rgb, dtype=np.uint8).copy()
        gray = self._gray(image)
        valid_mask = _as_mask(mask, gray.shape)
        if not preserve_identity_reference or self.reference_gray is None:
            self.reference_rgb = image.copy()
            self.reference_gray = gray.copy()
            self.reference_mask = valid_mask.copy()
            orb_mask = self._feature_mask(valid_mask, excluded_tip_uv)
            self.reference_keypoints, self.reference_descriptors = (
                self._orb.detectAndCompute(gray, orb_mask)
            )
        self.previous_gray = gray
        self.current_mask = valid_mask
        self.points = self._detect_points(gray, valid_mask, excluded_tip_uv)
        self.frame_index = 0
        self._set_rolling_reference(gray, valid_mask, excluded_tip_uv)
        point_count = 0 if self.points is None else int(len(self.points))
        if point_count < self.config.minimum_tracks:
            raise ValueError(
                f"confirmed target has only {point_count} usable KLT points; "
                f"need {self.config.minimum_tracks}"
            )
        result = self._result(
            "INITIALIZED", "SAM_HUMAN_CONFIRMED", valid_mask,
            tracked_points=point_count, inlier_ratio=1.0,
            processing_ms=0.0,
            feature_points=self.points,
        )
        self.last_result = result
        return result

    def _set_rolling_reference(
        self,
        gray: np.ndarray,
        mask: np.ndarray,
        excluded_tip_uv: tuple[float, float] | None,
    ) -> None:
        feature_mask = self._feature_mask(mask, excluded_tip_uv)
        keypoints, descriptors = self._orb.detectAndCompute(gray, feature_mask)
        if descriptors is None or len(keypoints) < self.config.orb_minimum_matches:
            return
        self.rolling_gray = gray.copy()
        self.rolling_mask = mask.copy()
        self.rolling_keypoints = keypoints
        self.rolling_descriptors = descriptors
        self.rolling_frame_index = self.frame_index

    def _coverage_ratio(self, points: np.ndarray, mask: np.ndarray) -> float:
        if len(points) < 2:
            return 0.0
        _, _, w, h = cv2.boundingRect((mask > 0).astype(np.uint8))
        mask_area = max(float(w * h), 1.0)
        _, _, point_width, point_height = cv2.boundingRect(
            points.reshape(-1, 1, 2).astype(np.float32)
        )
        return float(point_width * point_height) / mask_area

    def _result(
        self,
        status: str,
        method: str,
        mask: np.ndarray | None,
        *,
        tracked_points: int,
        inlier_ratio: float,
        processing_ms: float,
        reason: str = "",
        affine: np.ndarray | None = None,
        feature_points: np.ndarray | None = None,
        previous_feature_points: np.ndarray | None = None,
    ) -> dict[str, Any]:
        if mask is None:
            centroid = None
            mask_pixels = 0
            mask_fraction = 0.0
        else:
            centroid = _centroid(mask).tolist()
            mask_pixels = int(np.count_nonzero(mask))
            mask_fraction = float(mask_pixels) / float(mask.size)
        def bounded_points(points: np.ndarray | None) -> list[list[float]]:
            if points is None:
                return []
            values = np.asarray(points, dtype=np.float32).reshape(-1, 2)
            values = values[: self.config.max_features]
            return [[float(point[0]), float(point[1])] for point in values]

        return {
            "status": status,
            "method": method,
            "reason": reason,
            "frame_index": self.frame_index,
            "centroid_uv": centroid,
            "mask_pixels": mask_pixels,
            "mask_fraction": mask_fraction,
            "tracked_points": int(tracked_points),
            "inlier_ratio": float(inlier_ratio),
            "processing_ms": float(processing_ms),
            "feature_points_uv": bounded_points(feature_points),
            "previous_feature_points_uv": bounded_points(
                previous_feature_points
            ),
            "affine_previous_to_current": (
                None if affine is None else np.asarray(affine).tolist()
            ),
        }

    def _accept_warp(
        self, affine: np.ndarray, source_mask: np.ndarray, shape: tuple[int, int]
    ) -> tuple[np.ndarray | None, str]:
        linear = np.asarray(affine[:, :2], dtype=np.float64)
        determinant = float(np.linalg.det(linear))
        if not math.isfinite(determinant) or determinant <= 0.0:
            return None, "non-positive affine determinant"
        scale = math.sqrt(determinant)
        if not 0.70 <= scale <= 1.45:
            return None, f"implausible frame scale {scale:.3f}"
        height, width = shape
        warped = cv2.warpAffine(
            source_mask.astype(np.uint8), affine.astype(np.float32),
            (width, height), flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0,
        )
        new_area = int(np.count_nonzero(warped))
        old_area = max(int(np.count_nonzero(source_mask)), 1)
        ratio = float(new_area) / float(old_area)
        if not self.config.frame_area_ratio_min <= ratio <= self.config.frame_area_ratio_max:
            return None, f"implausible frame mask area ratio {ratio:.3f}"
        if new_area < 20:
            return None, "warped mask is too small"
        centre = _centroid(warped)
        if not (0.0 <= centre[0] < width and 0.0 <= centre[1] < height):
            return None, "warped centroid is outside image"
        return warped, ""

    def _orb_recover(
        self,
        gray: np.ndarray,
        excluded_tip_uv: tuple[float, float] | None,
    ) -> tuple[
        np.ndarray | None, np.ndarray | None, np.ndarray | None, int, float, str, str
    ]:
        keypoints, descriptors = self._orb.detectAndCompute(gray, None)
        if descriptors is None or len(keypoints) < self.config.orb_minimum_matches:
            return (
                None, None, None, 0, 0.0,
                "current frame has too few ORB descriptors", "NONE",
            )
        references = [
            (
                "ROLLING_KEYFRAME", self.rolling_mask,
                self.rolling_keypoints, self.rolling_descriptors,
            ),
            (
                "IMMUTABLE_HUMAN_KEYFRAME", self.reference_mask,
                self.reference_keypoints, self.reference_descriptors,
            ),
        ]
        failures: list[str] = []
        for name, source_mask, source_keypoints, source_descriptors in references:
            if (
                source_mask is None or source_descriptors is None
                or len(source_keypoints) < self.config.orb_minimum_matches
            ):
                failures.append(f"{name}: unavailable")
                continue
            matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(
                source_descriptors, descriptors, k=2
            )
            good = [
                pair[0] for pair in matches
                if len(pair) == 2
                and pair[0].distance < self.config.orb_ratio_test * pair[1].distance
            ]
            if len(good) < self.config.orb_minimum_matches:
                failures.append(f"{name}: {len(good)} ratio-test matches")
                continue
            source = np.float32(
                [source_keypoints[item.queryIdx].pt for item in good]
            )
            destination = np.float32([keypoints[item.trainIdx].pt for item in good])
            affine, inliers = cv2.estimateAffinePartial2D(
                source, destination, method=cv2.RANSAC,
                ransacReprojThreshold=3.0, maxIters=2000, confidence=0.995,
            )
            if affine is None or inliers is None:
                failures.append(f"{name}: RANSAC failed")
                continue
            inlier_selection = inliers.reshape(-1) > 0
            ratio = float(np.mean(inlier_selection))
            verified_points = destination[inlier_selection]
            if (
                ratio < self.config.minimum_inlier_ratio
                or len(verified_points) < self.config.minimum_tracks
            ):
                failures.append(
                    f"{name}: inliers={ratio:.2f}/{len(verified_points)}"
                )
                continue
            mask, reason = self._accept_warp(affine, source_mask, gray.shape)
            if mask is None:
                failures.append(f"{name}: {reason}")
                continue
            return (
                mask, affine, verified_points, len(good), ratio, "", name,
            )
        return None, None, None, 0, 0.0, "; ".join(failures), "NONE"

    def update(
        self,
        rgb: np.ndarray,
        *,
        excluded_tip_uv: tuple[float, float] | None = None,
    ) -> dict[str, Any]:
        started = time.perf_counter()
        if not self.active or self.points is None:
            raise RuntimeError("tracker is not initialized")
        gray = self._gray(np.asarray(rgb, dtype=np.uint8))
        self.frame_index += 1
        old_points = self.points.reshape(-1, 1, 2).astype(np.float32)
        new_points, forward_status, _ = cv2.calcOpticalFlowPyrLK(
            self.previous_gray, gray, old_points, None,
            winSize=(self.config.lk_window_px, self.config.lk_window_px),
            maxLevel=self.config.lk_levels,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        klt_reason = "pyramidal KLT failed"
        if new_points is not None and forward_status is not None:
            back_points, backward_status, _ = cv2.calcOpticalFlowPyrLK(
                gray, self.previous_gray, new_points, None,
                winSize=(self.config.lk_window_px, self.config.lk_window_px),
                maxLevel=self.config.lk_levels,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
            )
            if back_points is not None and backward_status is not None:
                fb_error = np.linalg.norm(
                    back_points.reshape(-1, 2) - old_points.reshape(-1, 2), axis=1
                )
                good = (
                    (forward_status.reshape(-1) > 0)
                    & (backward_status.reshape(-1) > 0)
                    & np.isfinite(fb_error)
                    & (fb_error <= self.config.lk_fb_error_px)
                )
                source = old_points.reshape(-1, 2)[good]
                destination = new_points.reshape(-1, 2)[good]
                if len(source) >= self.config.minimum_tracks:
                    affine, inliers = cv2.estimateAffinePartial2D(
                        source, destination, method=cv2.RANSAC,
                        ransacReprojThreshold=2.5, maxIters=1000, confidence=0.995,
                    )
                    if affine is not None and inliers is not None:
                        inlier_ratio = float(np.mean(inliers.reshape(-1) > 0))
                        inlier_points = destination[inliers.reshape(-1) > 0]
                        coverage = self._coverage_ratio(inlier_points, self.current_mask)
                        warped, reason = self._accept_warp(
                            affine, self.current_mask, gray.shape
                        )
                        if (
                            warped is not None
                            and inlier_ratio >= self.config.minimum_inlier_ratio
                            and coverage >= self.config.minimum_coverage_ratio
                        ):
                            self.previous_gray = gray
                            self.current_mask = warped
                            refreshed = self._detect_points(
                                gray, warped, excluded_tip_uv
                            )
                            # A close/partially occluded elevator button can
                            # temporarily yield too few fresh Shi-Tomasi
                            # corners.  Keep the geometrically verified KLT
                            # inliers for the next frame instead of returning
                            # TRACKING with a dead tracker.
                            self.points = (
                                refreshed
                                if refreshed is not None
                                and len(refreshed) >= self.config.minimum_tracks
                                else inlier_points.reshape(-1, 1, 2).astype(np.float32)
                            )
                            elapsed = (time.perf_counter() - started) * 1000.0
                            result = self._result(
                                "TRACKING", "PYRAMID_KLT_RANSAC", warped,
                                tracked_points=int(len(inlier_points)),
                                inlier_ratio=inlier_ratio,
                                processing_ms=elapsed, affine=affine,
                                feature_points=inlier_points,
                                previous_feature_points=(
                                    source[inliers.reshape(-1) > 0]
                                ),
                            )
                            result["feature_coverage_ratio"] = coverage
                            interval = max(
                                1, int(self.config.rolling_keyframe_interval_frames)
                            )
                            if self.frame_index - self.rolling_frame_index >= interval:
                                self._set_rolling_reference(
                                    gray, warped, excluded_tip_uv
                                )
                            self.last_result = result
                            return result
                        klt_reason = reason or (
                            f"KLT geometry weak: inliers={inlier_ratio:.2f}, "
                            f"coverage={coverage:.2f}"
                        )
                    else:
                        klt_reason = "KLT RANSAC failed"
                else:
                    klt_reason = (
                        f"KLT retained {len(source)} tracks; need "
                        f"{self.config.minimum_tracks}"
                    )

        (
            recovered, affine, orb_points, matches, inlier_ratio,
            orb_reason, orb_reference,
        ) = self._orb_recover(gray, excluded_tip_uv)
        elapsed = (time.perf_counter() - started) * 1000.0
        if recovered is not None:
            self.previous_gray = gray
            self.current_mask = recovered
            refreshed = self._detect_points(gray, recovered, excluded_tip_uv)
            # ORB supplied enough RANSAC-consistent correspondences to recover
            # identity.  If the recovered mask itself is low-texture, retain
            # the verified current ORB locations as the rolling seed.
            if refreshed is not None and len(refreshed) >= self.config.minimum_tracks:
                self.points = refreshed
            else:
                self.points = (
                    None if orb_points is None else
                    np.asarray(orb_points, dtype=np.float32).reshape(-1, 1, 2)[
                        : self.config.max_features
                    ]
                )
            result = self._result(
                "RECOVERED", f"ORB_RANSAC_{orb_reference}", recovered,
                tracked_points=matches, inlier_ratio=inlier_ratio,
                processing_ms=elapsed, affine=affine,
                reason=f"KLT rejected: {klt_reason}",
                feature_points=orb_points,
            )
            self._set_rolling_reference(gray, recovered, excluded_tip_uv)
            self.last_result = result
            return result

        result = self._result(
            "LOST", "SEMANTIC_RECOVERY_REQUIRED", None,
            tracked_points=matches, inlier_ratio=inlier_ratio,
            processing_ms=elapsed,
            reason=f"KLT: {klt_reason}; ORB: {orb_reason}",
        )
        self.last_result = result
        return result
