import importlib.util
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location("initialpose_scan_window", Path(__file__).resolve().parents[1] / "scripts" / "initialpose_scan_window.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
InitialPoseScanWindow = module.InitialPoseScanWindow


def ring(start=0, end=36):
    angles = (np.arange(start, end) + .5) * 2 * np.pi / 36 - np.pi
    return np.column_stack([2 * np.cos(angles), 2 * np.sin(angles), np.zeros(len(angles))])


def test_empty_return_sectors_do_not_require_endless_accumulation():
    window = InitialPoseScanWindow(0, 10, 2, 30, 75, .90, source_time=True)
    assert window.add(ring(0, 34), 1_000_000_000, 1) is None
    assert window.add(ring(0, 34), 9_000_000_000, 20) is None
    assert window.add(ring(0, 34), 11_000_000_000, 25) is not None
    assert window.coverage_observed == 34 / 36


def test_waits_for_time_unique_scans_and_full_azimuth():
    window = InitialPoseScanWindow(0, 10, 3, 30, 75, 1, min_stamp=10)
    assert window.add(ring(), 9, 2) is None
    assert window.add(ring(0, 18), 11, 3) is None
    assert window.add(ring(0, 18), 11, 4) is None
    assert len(window.chunks) == 1
    assert window.add(ring(0, 18), 12, 11) is None
    assert window.add(ring(18), 13, 20) is not None


def test_timeout_stays_visible_but_fresh_coverage_can_recover():
    window = InitialPoseScanWindow(0, 10, 2, 30, 75, 1)
    for stamp, now in enumerate([1, 10, 30, 60, 76], 1):
        assert window.add(ring(0, 18), stamp, now) is None
    assert window.failed
    assert window.add(ring(), 6, 77) is not None
    assert not window.failed


def test_long_gap_requires_new_accumulation_not_old_coverage():
    window = InitialPoseScanWindow(0, 10, 2, 30, 75, 1)
    assert window.add(ring(), 1, 0) is None
    assert window.add(ring(), 2, 80) is None
    assert window.failed
    assert window.add(ring(), 3, 91) is not None


def test_coverage_measured_before_matching_height_filter():
    window = InitialPoseScanWindow(0, 10, 2, 30, 75, 1)
    assert window.add(ring(0, 20), 1, 0, coverage_points=ring()) is None
    assert window.add(ring(0, 20), 2, 11, coverage_points=ring()) is not None
    assert window.coverage_observed == 1


def test_non_timeout_failure_does_not_silently_resume():
    window = InitialPoseScanWindow(0, 0, 1, 30, 75, 1)
    window.failed = True
    window.failure_reason = "lidar_frame_mismatch"
    assert window.add(ring(), 1, 1) is None


def test_old_coverage_expires_and_memory_is_bounded():
    window = InitialPoseScanWindow(0, 10, 2, 2, 75, 1)
    assert window.add(ring(18), 1, 1) is None
    assert window.add(ring(0, 18), 2, 2) is None
    assert window.add(ring(0, 18), 3, 20) is None
    assert len(window.chunks) == 2


def test_invalid_points_do_not_count_as_scans():
    window = InitialPoseScanWindow(0, 0, 1, 2, 75, 1)
    assert window.add(np.array([[np.nan, 0, 0]]), 1, 1) is None
    assert window.chunks == []


def test_high_rate_stream_completes_with_bounded_memory():
    window = InitialPoseScanWindow(0, 10, 10, 30, 75, 1)
    for i in range(101):
        result = window.add(ring(), i + 1, i / 10)
    assert result is not None
    assert len(window.chunks) == 30
    assert window.times[-1] - window.times[0] < window.min_sec


def test_sim_window_requires_source_time_not_wall_elapsed():
    window = InitialPoseScanWindow(0, 10, 2, 30, 75, 1, source_time=True)
    assert window.add(ring(), 100000000000, 0) is None
    assert window.add(ring(), 105000000000, 30) is None
    assert window.add(ring(), 110000000000, 60) is not None


def test_repeated_stamp_or_clock_rewind_never_completes_old_window():
    window = InitialPoseScanWindow(0, 10, 2, 30, 75, 1, source_time=True)
    assert window.add(ring(), 100000000000, 0) is None
    assert window.add(ring(), 100000000000, 50) is None
    assert window.add(ring(), 1000000000, 60) is None
