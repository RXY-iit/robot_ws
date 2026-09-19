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
mild_single_motor_hold_entry_overrun = MODULE.mild_single_motor_hold_entry_overrun
next_approach_waypoint = MODULE.next_approach_waypoint
project_camera_point = MODULE.project_camera_point
validate_tracking_continuity = MODULE.validate_tracking_continuity
visual_tip_alignment_delta = MODULE.visual_tip_alignment_delta
base_yz_pixel_alignment_delta = MODULE.base_yz_pixel_alignment_delta
compensated_cartesian_delta = MODULE.compensated_cartesian_delta
diagnostic_visualization_should_defer = (
    MODULE.diagnostic_visualization_should_defer
)
empirical_image_center_alignment_delta = MODULE.empirical_image_center_alignment_delta
pixel_inside_mask = MODULE.pixel_inside_mask
phase_sam_minimum_score = MODULE.phase_sam_minimum_score
phase_tracking_area_ratio_bounds = MODULE.phase_tracking_area_ratio_bounds
should_retreat_for_too_close = MODULE.should_retreat_for_too_close
rolling_approach_phase = MODULE.rolling_approach_phase
rolling_forward_target_x = MODULE.rolling_forward_target_x
rolling_tracker_freshness_gate = MODULE.rolling_tracker_freshness_gate
rolling_z_motion_compensation = MODULE.rolling_z_motion_compensation
receding_joint_action_horizon = MODULE.receding_joint_action_horizon


@pytest.mark.parametrize(
    "automatic,rolling,request_active,expected",
    [
        (False, False, False, False),
        (True, False, False, True),
        (False, True, False, True),
        (False, False, True, True),
    ],
)
def test_diagnostic_visualization_is_deferred_for_every_motion_owner(
    automatic, rolling, request_active, expected,
):
    assert diagnostic_visualization_should_defer(
        automatic_approach=automatic,
        rolling_session_active=rolling,
        execute_request_active=request_active,
    ) is expected


def test_tracker_freshness_waits_for_fresh_inflight_frame():
    result = rolling_tracker_freshness_gate(
        status="TRACKING",
        completed_age_sec=0.279,
        maximum_completed_age_sec=0.200,
        inflight_processing_age_sec=0.037,
        inflight_input_age_sec=0.071,
        maximum_inflight_processing_sec=0.150,
    )
    assert result["decision"] == "WAIT_INFLIGHT"


def test_tracker_freshness_stops_after_inflight_deadline():
    result = rolling_tracker_freshness_gate(
        status="TRACKING",
        completed_age_sec=0.351,
        maximum_completed_age_sec=0.200,
        inflight_processing_age_sec=0.151,
        inflight_input_age_sec=0.190,
        maximum_inflight_processing_sec=0.150,
    )
    assert result["decision"] == "STOP"


def test_tracker_freshness_never_hides_lost_status():
    result = rolling_tracker_freshness_gate(
        status="LOST",
        completed_age_sec=0.050,
        maximum_completed_age_sec=0.200,
        inflight_processing_age_sec=0.010,
        inflight_input_age_sec=0.020,
        maximum_inflight_processing_sec=0.150,
    )
    assert result["decision"] == "STOP"


def test_receding_horizon_predicts_delay_and_builds_ten_smooth_steps():
    result = receding_joint_action_horizon(
        current_positions={"a": 0.0, "b": 0.5},
        target_positions={"a": 1.0, "b": 0.0},
        current_velocities={"a": 0.4, "b": -0.2},
        joint_bounds={"a": (-1.0, 1.0), "b": (-1.0, 1.0)},
        steps=10,
        step_period_sec=0.05,
        prediction_lead_sec=0.10,
        maximum_prediction_velocity_rad_s=1.0,
    )
    assert result["predicted_start_positions"] == pytest.approx(
        {"a": 0.04, "b": 0.48}
    )
    assert len(result["positions"]) == 10
    assert result["positions"][-1] == pytest.approx({"a": 1.0, "b": 0.0})
    assert all(
        left["a"] < right["a"]
        for left, right in zip(result["positions"], result["positions"][1:])
    )


def test_rolling_approach_phase_transitions_from_far_to_near_and_press():
    assert rolling_approach_phase(
        advanced_from_far_m=0.020, near_distance_m=0.035,
        press_authorized=False,
    ) == "FAR_TRACK_AND_ADVANCE"
    assert rolling_approach_phase(
        advanced_from_far_m=0.034, near_distance_m=0.035,
        press_authorized=False,
    ) == "NEAR_ALIGN"
    assert rolling_approach_phase(
        advanced_from_far_m=0.034, near_distance_m=0.035,
        press_authorized=True,
    ) == "PRESS"


def test_rolling_forward_target_uses_short_horizon_and_hard_caps():
    far_target, complete = rolling_forward_target_x(
        phase="FAR_TRACK_AND_ADVANCE", actual_x_m=0.120, far_x_m=0.100,
        near_distance_m=0.035, horizon_m=0.006,
    )
    assert far_target == pytest.approx(0.126)
    assert complete is False
    capped_near, _ = rolling_forward_target_x(
        phase="FAR_TRACK_AND_ADVANCE", actual_x_m=0.133, far_x_m=0.100,
        near_distance_m=0.035, horizon_m=0.006,
    )
    assert capped_near == pytest.approx(0.135)
    press_target, complete = rolling_forward_target_x(
        phase="PRESS", actual_x_m=0.150, far_x_m=0.100,
        near_distance_m=0.035, horizon_m=0.006,
        target_plane_x_m=0.155, maximum_press_past_target_m=0.020,
    )
    assert press_target == pytest.approx(0.156)
    assert complete is False
    capped_press, complete = rolling_forward_target_x(
        phase="PRESS", actual_x_m=0.175, far_x_m=0.100,
        near_distance_m=0.035, horizon_m=0.006,
        target_plane_x_m=0.155, maximum_press_past_target_m=0.020,
    )
    assert capped_press == pytest.approx(0.175)
    assert complete is True


def test_rolling_z_compensation_carries_unachieved_target_and_pauses_x():
    result = rolling_z_motion_compensation(
        visual_delta_z_m=-0.0014,
        previous_target_error_z_m=0.0398,
        nominal_x_horizon_m=0.006,
        phase="FAR_TRACK_AND_ADVANCE",
        residual_gain=0.80,
        far_z_per_x_gain=0.88,
        maximum_compensation_m=0.025,
        slowdown_start_error_m=0.006,
        slowdown_full_error_m=0.020,
    )
    assert result["x_horizon_scale"] == pytest.approx(0.0)
    assert result["scaled_x_horizon_m"] == pytest.approx(0.0)
    assert result["compensation_z_m"] == pytest.approx(0.025)
    assert result["command_delta_z_m"] == pytest.approx(0.0236)


def test_rolling_z_compensation_adds_x_coupling_when_z_is_caught_up():
    result = rolling_z_motion_compensation(
        visual_delta_z_m=0.001,
        previous_target_error_z_m=0.002,
        nominal_x_horizon_m=0.006,
        phase="FAR_TRACK_AND_ADVANCE",
        residual_gain=0.80,
        far_z_per_x_gain=0.88,
        maximum_compensation_m=0.025,
        slowdown_start_error_m=0.006,
        slowdown_full_error_m=0.020,
    )
    assert result["x_horizon_scale"] == pytest.approx(1.0)
    assert result["model_feedforward_z_m"] == pytest.approx(0.00528)
    assert result["command_delta_z_m"] == pytest.approx(0.00788)


def test_real_bag_rightward_drift_commands_up_and_preserves_z_residual():
    """Regression for touch_regression_20260918_155846."""
    alignment = empirical_image_center_alignment_delta(
        (495.53, 244.80), (319.5, 239.5), 0.00020
    )
    assert alignment["delta_base_yz_m"][1] > 0.0
    delta_yz = np.asarray(alignment["delta_base_yz_m"], dtype=np.float64)
    delta_yz *= min(1.0, 0.003 / float(np.linalg.norm(delta_yz)))
    result = rolling_z_motion_compensation(
        visual_delta_z_m=float(delta_yz[1]),
        previous_target_error_z_m=0.0398,
        nominal_x_horizon_m=0.006,
        phase="FAR_TRACK_AND_ADVANCE",
        residual_gain=0.80,
        far_z_per_x_gain=0.88,
        maximum_compensation_m=0.025,
        slowdown_start_error_m=0.006,
        slowdown_full_error_m=0.020,
    )
    assert result["scaled_x_horizon_m"] == pytest.approx(0.0)
    assert result["command_delta_z_m"] > 0.025


def test_mild_isolated_elbow_hold_entry_overrun_is_recoverable():
    message = (
        "trajectory failed: -4 Final error exceeds per-motor HOLD-entry "
        "tolerance: {'elbow_flex': {'error': -46, 'tolerance': 45}}; "
        "all errors={'elbow_flex': -46, 'wrist_flex': 2}"
    )
    result = mild_single_motor_hold_entry_overrun(
        message, motor="elbow_flex", maximum_excess_ticks=5
    )
    assert result["eligible"] is True
    assert result["excess_ticks"] == 1


def test_hold_entry_overrun_rejects_multiple_motors_or_large_elbow_miss():
    multiple = (
        "Final error exceeds per-motor HOLD-entry tolerance: "
        "{'elbow_flex': {'error': -46, 'tolerance': 45}, "
        "'shoulder_lift': {'error': 21, 'tolerance': 20}}; all errors={}"
    )
    large = (
        "Final error exceeds per-motor HOLD-entry tolerance: "
        "{'elbow_flex': {'error': -55, 'tolerance': 45}}; all errors={}"
    )
    assert not mild_single_motor_hold_entry_overrun(
        multiple, motor="elbow_flex", maximum_excess_ticks=5
    )["eligible"]
    assert not mild_single_motor_hold_entry_overrun(
        large, motor="elbow_flex", maximum_excess_ticks=5
    )["eligible"]


@pytest.mark.parametrize(
    ("handoff", "near", "press", "expected"),
    [
        (False, False, False, 0.40),
        (True, False, False, 0.20),
        (True, True, False, 0.10),
        (True, True, True, 0.05),
    ],
)
def test_sam_score_gate_follows_touch_phase_not_mask_size(
    handoff, near, press, expected
):
    assert phase_sam_minimum_score(
        operator_handoff_complete=handoff,
        near_phase_latched=near,
        press_phase_active=press,
        initial_score=0.40,
        far_tracking_score=0.20,
        near_tracking_score=0.10,
        press_score=0.05,
    ) == pytest.approx(expected)


def test_tracking_area_ratio_relaxes_only_after_near_is_latched():
    assert phase_tracking_area_ratio_bounds(
        near_phase_latched=False,
        far_minimum_ratio=0.20,
        near_minimum_ratio=0.05,
        maximum_ratio=3.0,
    ) == pytest.approx((0.20, 3.0))
    assert phase_tracking_area_ratio_bounds(
        near_phase_latched=True,
        far_minimum_ratio=0.20,
        near_minimum_ratio=0.05,
        maximum_ratio=3.0,
        near_maximum_ratio=12.0,
    ) == pytest.approx((0.05, 12.0))


@pytest.mark.parametrize(
    ("too_close", "aligned", "expected"),
    [
        (False, False, False),
        (False, True, False),
        (True, False, True),
        (True, True, False),
    ],
)
def test_too_close_retreat_requires_missing_press_alignment(
    too_close, aligned, expected
):
    assert should_retreat_for_too_close(
        too_close=too_close, press_alignment_ready=aligned
    ) is expected


def test_near_cartesian_compensation_preserves_goal_and_biases_loaded_axes():
    desired = np.array([0.0, 0.010, 0.010])
    result = compensated_cartesian_delta(
        desired, x_hold_bias_m=0.004, y_gain=1.10, z_gain=1.20,
        z_up_bias_m=0.015, maximum_norm_m=0.040,
    )
    assert result["desired_delta_base_m"] == pytest.approx(desired)
    assert result["command_delta_base_m"] == pytest.approx(
        [0.004, 0.011, 0.027]
    )
    assert result["bounded"] is False


def test_near_cartesian_compensation_bounds_large_command_without_flipping_direction():
    result = compensated_cartesian_delta(
        np.array([0.0, 0.020, 0.030]),
        x_hold_bias_m=0.004, y_gain=1.10, z_gain=1.20,
        z_up_bias_m=0.015, maximum_norm_m=0.040,
    )
    command = np.asarray(result["command_delta_base_m"])
    assert np.linalg.norm(command) == pytest.approx(0.040)
    assert result["bounded"] is True
    assert command[0] > 0.0 and command[1] > 0.0 and command[2] > 0.0


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
