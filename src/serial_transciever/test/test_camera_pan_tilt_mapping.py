import math

from serial_transciever.camera_pan_tilt_joint_state_node import calibrated_angle


def test_reference_maps_to_zero():
    assert calibrated_angle(267.1, 267.1, 0.017, -0.000006) == 0.0


def test_measured_feedback_maps_to_finite_nonzero_angles():
    pan = calibrated_angle(266.92, 267.1, 0.0170137853, -0.00000617291927)
    tilt = calibrated_angle(46.41, 101.8, 0.0160077655, -0.00000578161438)
    assert math.isfinite(pan) and math.isfinite(tilt)
    assert pan < 0.0
    assert tilt < 0.0
