import pytest

from so101_arm_control.arm_mount_tf_publisher import (
    map_lift_to_mount_z,
    quaternion_from_euler,
)


def test_lift_estimate_maps_and_clamps_to_measured_mount_range():
    args = (0.0, 200.0, 0.8925, 1.1225)
    assert map_lift_to_mount_z(0.0, *args) == pytest.approx(0.8925)
    assert map_lift_to_mount_z(100.0, *args) == pytest.approx(1.0075)
    assert map_lift_to_mount_z(200.0, *args) == pytest.approx(1.1225)
    assert map_lift_to_mount_z(-10.0, *args) == pytest.approx(0.8925)
    assert map_lift_to_mount_z(250.0, *args) == pytest.approx(1.1225)


def test_remounted_base_is_level_and_front_back_is_corrected():
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
    assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_invalid_lift_range_is_rejected():
    with pytest.raises(ValueError):
        map_lift_to_mount_z(0.0, 1.0, 1.0, 0.8, 1.1)
