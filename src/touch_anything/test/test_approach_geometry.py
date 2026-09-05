import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "approach_geometry.py"
SPEC = importlib.util.spec_from_file_location("approach_geometry", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
adaptive_close_mask_fraction = MODULE.adaptive_close_mask_fraction
bounded_retreat_step = MODULE.bounded_retreat_step
monocular_lateral_target = MODULE.monocular_lateral_target
next_approach_waypoint = MODULE.next_approach_waypoint
project_camera_point = MODULE.project_camera_point
validate_tracking_continuity = MODULE.validate_tracking_continuity
visual_tip_alignment_delta = MODULE.visual_tip_alignment_delta
base_yz_pixel_alignment_delta = MODULE.base_yz_pixel_alignment_delta
empirical_image_center_alignment_delta = MODULE.empirical_image_center_alignment_delta
pixel_inside_mask = MODULE.pixel_inside_mask


def test_adaptive_close_fraction_uses_38mm_square_as_reference():
    assert adaptive_close_mask_fraction(0.038, 0.038) == pytest.approx(0.25)


def test_adaptive_close_fraction_scales_and_is_bounded():
    assert adaptive_close_mask_fraction(0.025, 0.019) == pytest.approx(
        0.25 * 0.025 * 0.019 / (0.038 ** 2)
    )
    assert adaptive_close_mask_fraction(0.0038, 0.0038) == pytest.approx(0.06)
    assert adaptive_close_mask_fraction(0.10, 0.10) == pytest.approx(0.35)


K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])


def test_projection_uses_calibrated_pinhole_geometry():
    assert project_camera_point(np.array([0.1, -0.05, 1.0]), K) == pytest.approx((370.0, 215.0))


def test_projection_rejects_point_behind_camera():
    with pytest.raises(ValueError):
        project_camera_point(np.array([0.0, 0.0, -1.0]), K)


def test_monocular_ray_changes_lateral_coordinates_but_keeps_d435_x_plane():
    # REP-103 optical axes (right, down, forward) mapped to a level robot
    # camera looking along base +X.
    base_from_camera = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    result = monocular_lateral_target(
        (370.0, 240.0), K, None, base_from_camera, np.zeros(3), np.array([1.0, 0.0, 0.0])
    )
    assert result["point_base_m"][0] == pytest.approx(1.0)
    assert result["point_base_m"][1] < 0.0
    assert result["monocular_depth_observed"] is False


def test_next_waypoint_advances_in_x_and_uses_visual_y_z():
    result = next_approach_waypoint(
        np.array([0.50, 0.0, 0.4]), np.array([0.58, 0.02, 0.41]), 0.01, 0.015
    )
    assert result["remaining_before_m"] == pytest.approx(0.08)
    assert result["remaining_after_m"] == pytest.approx(0.07)
    assert result["waypoint_base_m"] == pytest.approx([0.51, 0.02, 0.41])
    assert result["complete"] is False


@pytest.mark.parametrize(
    ("target", "expected_signs"),
    [
        ((320.0, 260.0), (-1, 0)),  # image down -> robot right (-Y)
        ((320.0, 220.0), (1, 0)),   # image up -> robot left (+Y)
        ((300.0, 240.0), (0, -1)),  # image left -> arm down (-Z)
        ((340.0, 240.0), (0, 1)),   # image right -> arm up (+Z)
    ],
)
def test_empirical_image_center_mapping_matches_real_robot(target, expected_signs):
    result = empirical_image_center_alignment_delta(target, (320.0, 240.0), 0.0002)
    delta_y, delta_z = result["delta_base_yz_m"]
    actual_signs = (int(np.sign(delta_y)), int(np.sign(delta_z)))
    assert actual_signs == expected_signs


def test_empirical_image_center_mapping_is_zero_at_centre():
    result = empirical_image_center_alignment_delta(
        (319.5, 239.5), (319.5, 239.5), 0.0002
    )
    assert result["pixel_error_norm"] == pytest.approx(0.0)
    assert result["delta_base_yz_m"] == pytest.approx([0.0, 0.0])


def test_pixel_inside_mask_requires_the_actual_tip_pixel_to_be_segmented():
    mask = np.zeros((20, 30), dtype=bool)
    mask[8:13, 12:18] = True
    assert pixel_inside_mask(mask, (15.2, 10.1)) is True
    assert pixel_inside_mask(mask, (11.0, 10.0)) is False
    assert pixel_inside_mask(mask, (-1.0, 10.0)) is False


def test_retreat_is_capped_by_far_x_plane():
    assert bounded_retreat_step(0.50, 0.40, 0.025) == pytest.approx(-0.025)
    assert bounded_retreat_step(0.41, 0.40, 0.025) == pytest.approx(-0.010)
    assert bounded_retreat_step(0.40, 0.40, 0.025) == pytest.approx(0.0)
    assert bounded_retreat_step(0.3998, 0.40, 0.025) == pytest.approx(0.0)


def test_approach_stops_before_contact():
    result = next_approach_waypoint(
        np.array([0.566, 0.0, 0.4]), np.array([0.58, 0.0, 0.4]), 0.005, 0.015
    )
    assert result["complete"] is True
    assert result["waypoint_base_m"][0] == pytest.approx(0.565)


def test_tracking_continuity_accepts_small_stopped_frame_change():
    result = validate_tracking_continuity(
        (300, 200), (312, 205), 10000, 11200, 80.0, 0.35, 2.85
    )
    assert result["valid"] is True
    assert result["centroid_shift_px"] == pytest.approx(13.0)
    assert result["mask_area_ratio"] == pytest.approx(1.12)


def test_tracking_continuity_rejects_large_jump_or_area_change():
    assert validate_tracking_continuity(
        (100, 100), (200, 100), 10000, 10000, 80.0, 0.35, 2.85
    )["valid"] is False


def test_visual_tip_alignment_uses_pixel_error_for_lateral_delta():
    base_from_camera = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    result = visual_tip_alignment_delta(
        (330.0, 230.0), (320.0, 240.0), K, None,
        base_from_camera, 0.5,
    )
    assert result["pixel_error_uv"] == pytest.approx([10.0, -10.0])
    assert result["pixel_error_norm"] == pytest.approx(np.sqrt(200.0))
    assert result["delta_base_m"][1] < 0.0
    assert result["delta_base_m"][2] > 0.0
    assert result["depth_observed_by_arm_camera"] is False
    assert validate_tracking_continuity(
        (100, 100), (102, 100), 10000, 30000, 80.0, 0.35, 2.85
    )["valid"] is False


def test_base_yz_image_jacobian_moves_target_toward_fixed_tip():
    # Level forward camera: base +Y is image-left and base +Z is image-up.
    base_from_camera = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    result = base_yz_pixel_alignment_delta(
        (370.0, 210.0), (320.0, 240.0), K, None,
        base_from_camera, 0.5,
    )
    # Target is image-right/up; move camera base-right/up to shift its image
    # left/down onto the rigid tip pixel.
    assert result["delta_base_yz_m"] == pytest.approx([-0.05, 0.03], abs=5e-4)
    assert result["pixel_error_uv"] == pytest.approx([50.0, -30.0])


def test_base_yz_image_jacobian_handles_tilted_axis_coupling():
    angle = np.deg2rad(25.0)
    level = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    base_roll = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(angle), -np.sin(angle)],
        [0.0, np.sin(angle), np.cos(angle)],
    ])
    result = base_yz_pixel_alignment_delta(
        (360.0, 220.0), (320.0, 240.0), K, None,
        base_roll @ level, 0.6,
    )
    assert np.isfinite(result["delta_base_yz_m"]).all()
    assert min(result["jacobian_singular_values"]) > 1.0
