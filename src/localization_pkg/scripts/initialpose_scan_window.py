"""Bounded fresh scans; azimuth coverage is measured, never inferred from Hz."""

import math

import numpy as np


class InitialPoseScanWindow:
    def __init__(self, started, min_sec, min_scans, max_scans, timeout_sec, coverage, min_stamp=0, source_time=False):
        self.started, self.min_sec, self.min_scans = started, min_sec, min_scans
        self.max_scans, self.timeout_sec, self.coverage = max_scans, timeout_sec, coverage
        self.chunks = []
        self.sectors = []
        self.last_stamp = min_stamp
        self.failed = False
        self.failure_reason = ""
        self.coverage_observed = 0.0
        self.times = []
        self.continuous_started = None
        self.last_observation_at = None
        self.source_time = source_time
        self.source_started = None
        self.source_span_sec = 0.0

    def add(self, points, stamp, now, coverage_points=None):
        if self.failed and self.failure_reason != "fresh_scan_coverage_timeout":
            return None
        if now - self.started > self.timeout_sec:
            self.failed = True
            self.failure_reason = self.failure_reason or "fresh_scan_coverage_timeout"
        if stamp <= self.last_stamp:
            return None
        finite = points[np.isfinite(points).all(axis=1)]
        if not len(finite):
            return None
        self.last_stamp = stamp
        sample_time = stamp / 1e9 if self.source_time else now
        if self.source_started is None:
            self.source_started = sample_time
        self.source_span_sec = sample_time - self.source_started
        if self.last_observation_at is None or sample_time - self.last_observation_at > max(3 * self.min_sec, 5):
            self.continuous_started = sample_time
        self.last_observation_at = sample_time
        coverage_points = finite if coverage_points is None else coverage_points
        valid = coverage_points[np.isfinite(coverage_points).all(axis=1)]
        valid = valid[np.linalg.norm(valid[:, :2], axis=1) > 0.3]
        bins = np.unique(np.floor((np.arctan2(valid[:, 1], valid[:, 0]) + math.pi) / (2 * math.pi) * 36).astype(int) % 36)
        # Bound matching memory while retaining coverage measured before sampling.
        self.chunks.append(finite[::max(1, math.ceil(len(finite) / 20000))])
        self.sectors.append(bins)
        self.times.append(sample_time)
        # Expired data cannot combine with a new burst to satisfy recovery.
        keep = [i for i, t in enumerate(self.times) if sample_time - t <= max(3 * self.min_sec, 5)][-self.max_scans:]
        self.chunks = [self.chunks[i] for i in keep]
        self.sectors = [self.sectors[i] for i in keep]
        self.times = [self.times[i] for i in keep]
        coverage = len(np.unique(np.concatenate(self.sectors))) / 36
        self.coverage_observed = coverage
        if sample_time - self.continuous_started < self.min_sec or len(self.chunks) < self.min_scans or coverage < self.coverage:
            return None
        self.failed = False
        self.failure_reason = ""
        return np.concatenate(self.chunks, axis=0)
