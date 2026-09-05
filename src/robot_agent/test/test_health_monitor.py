import math
from types import SimpleNamespace

from robot_agent.nodes.health_monitor import (
    HealthMonitor,
    SignalObservation,
    aggregate_status,
    diagnostic_level_number,
    json_safe,
)


def test_aggregate_does_not_treat_not_instrumented_as_available():
    status, reason = aggregate_status(
        [
            {
                "id": "physical_estop",
                "status": "NOT_INSTRUMENTED",
                "critical": True,
            },
            {"id": "drive_feedback", "status": "OBSERVED", "critical": True},
        ]
    )

    assert status == "UNKNOWN"
    assert reason is not None
    assert reason["id"] == "physical_estop"


def test_aggregate_prioritizes_explicit_block_over_unknown():
    status, reason = aggregate_status(
        [
            {"id": "physical_estop", "status": "NOT_INSTRUMENTED", "critical": True},
            {"id": "software_safety", "status": "BLOCKED", "critical": True},
        ]
    )

    assert status == "BLOCKED"
    assert reason is not None
    assert reason["id"] == "software_safety"


def test_aggregate_prioritizes_observed_degradation_over_unknown():
    status, reason = aggregate_status(
        [
            {"id": "physical_estop", "status": "NOT_INSTRUMENTED", "critical": True},
            {"id": "drive_health", "status": "DEGRADED", "critical": True},
        ]
    )

    assert status == "DEGRADED"
    assert reason is not None
    assert reason["id"] == "drive_health"


def test_optional_stale_item_does_not_degrade_domain():
    status, reason = aggregate_status(
        [
            {"id": "required", "status": "OK", "critical": True},
            {"id": "event_driven", "status": "STALE", "critical": False},
        ]
    )

    assert status == "AVAILABLE"
    assert reason is None


def test_recovered_history_does_not_degrade_current_domain():
    status, reason = aggregate_status(
        [
            {"id": "drive_transaction", "status": "RECOVERED", "critical": True},
            {"id": "drive_alarm", "status": "OBSERVED", "critical": True},
        ]
    )

    assert status == "AVAILABLE"
    assert reason is None


def test_signal_snapshot_uses_receiver_clock_and_window_rate():
    signal = SignalObservation(source="/test", evidence="DIRECT_TOPIC")
    signal.update({"value": 1}, now_monotonic=10.0, source_stamp_sec=99.0)
    signal.update({"value": 2}, now_monotonic=11.0, source_stamp_sec=100.0)

    snapshot = signal.snapshot(now_monotonic=12.0, ros_now_sec=101.5)

    assert snapshot["receive_age_sec"] == 1.0
    assert snapshot["source_age_sec"] == 1.5
    assert snapshot["receive_rate_hz_5s"] == 1.0
    assert snapshot["max_receive_gap_sec_5s"] == 1.0
    assert snapshot["message_count"] == 2


def test_json_safe_replaces_non_finite_numbers():
    value = json_safe({"ok": 1.0, "nan": math.nan, "items": [math.inf, -math.inf]})

    assert value == {"ok": 1.0, "nan": None, "items": [None, None]}


def test_software_estop_is_an_explicit_block():
    status, summary = HealthMonitor._safety_validator("ESTOP")

    assert status == "BLOCKED"
    assert "ESTOP" in summary


def test_idle_watchdog_is_a_safe_zero_output_state():
    status, summary = HealthMonitor._safety_validator("WATCHDOG")

    assert status == "OK"
    assert "zero output" in summary


def test_manual_stop_remains_visible_as_degraded():
    status, summary = HealthMonitor._safety_validator("MANUAL_STOP")

    assert status == "DEGRADED"
    assert "MANUAL_STOP" in summary


def test_joy_source_requires_consistent_explicit_authority():
    status, summary = HealthMonitor._joy_source_validator(
        {
            "source": "DASHBOARD",
            "dashboard_acquired": True,
            "reason": "dashboard control explicitly acquired",
        }
    )

    assert status == "OK"
    assert "explicitly owned" in summary


def test_joy_source_rejects_inconsistent_dashboard_state():
    status, summary = HealthMonitor._joy_source_validator(
        {"source": "DASHBOARD", "dashboard_acquired": False}
    )

    assert status == "ERROR"
    assert "Inconsistent" in summary


def test_modbus_summary_requires_completed_response():
    status, summary = HealthMonitor._modbus_validator(
        {
            "state": {"state_driver": 1, "state_message": 1, "state_error": 0},
            "last_response_age_sec": None,
            "query_timeouts_5s": 0,
        }
    )

    assert status == "UNKNOWN"
    assert "completed" in summary


def test_modbus_startup_without_state_snapshot_is_unknown_not_error():
    status, summary = HealthMonitor._modbus_validator(
        {
            "state": None,
            "last_response_age_sec": None,
            "completion_deadline_misses_5s": 0,
        }
    )

    assert status == "UNKNOWN"
    assert "no /om_state0 state snapshot yet" in summary


def test_modbus_summary_reports_recent_callback_deadline_miss_as_degraded():
    status, summary = HealthMonitor._modbus_validator(
        {
            "state": {"state_driver": 0, "state_message": 0, "state_error": 0},
            "last_response_age_sec": 0.1,
            "completion_deadline_misses_5s": 2,
        }
    )

    assert status == "DEGRADED"
    assert "2 completion callback deadline miss" in summary
    assert "not proven" in summary


def test_modbus_summary_recovers_old_callback_deadline_miss_with_fresh_reads():
    status, summary = HealthMonitor._modbus_validator(
        {
            "state": {"state_driver": 0, "state_message": 0, "state_error": 0},
            "last_response_age_sec": 0.1,
            "completion_deadline_miss_count": 1,
            "completion_deadline_misses_5s": 0,
            "last_completion_deadline_miss_age_sec": 42.0,
            "last_read_response_age_sec": 0.2,
        }
    )

    assert status == "RECOVERED"
    assert "Fresh read responses" in summary
    assert "42.0s ago" in summary


def test_modbus_summary_includes_deadline_miss_query_context():
    status, summary = HealthMonitor._modbus_validator(
        {
            "state": {"state_driver": 0, "state_message": 0, "state_error": 0},
            "last_response_age_sec": 0.1,
            "completion_deadline_miss_count": 1,
            "completion_deadline_misses_5s": 0,
            "last_completion_deadline_miss_age_sec": 12.0,
            "last_completion_deadline_miss_query_kind": "drive_diagnostics",
            "last_completion_deadline_miss_elapsed_sec": 0.263,
            "last_read_response_age_sec": 0.2,
        }
    )

    assert status == "RECOVERED"
    assert "during drive_diagnostics after 0.263s" in summary


def test_modbus_summary_prioritizes_direct_driver_error_event():
    status, summary = HealthMonitor._modbus_validator(
        {
            "state": {"state_driver": 0, "state_message": 0, "state_error": 0},
            "last_response_age_sec": 0.1,
            "driver_error_event_count": 1,
            "driver_no_response_event_count": 1,
            "last_driver_error_age_sec": 7.5,
            "completion_deadline_miss_count": 2,
        }
    )

    assert status == "DEGRADED"
    assert "direct Modbus driver error" in summary
    assert "7.5s ago" in summary


def test_modbus_summary_recovers_driver_error_only_after_complete_direct_evidence():
    status, summary = HealthMonitor._modbus_validator(
        {
            "state": {"state_driver": 0, "state_message": 0, "state_error": 0},
            "last_response_age_sec": 0.1,
            "last_read_response_age_sec": 0.2,
            "last_direct_io_age_sec": 0.3,
            "last_electrical_age_sec": 0.4,
            "driver_error_event_count": 1,
            "last_driver_error_age_sec": 12.0,
            "alarm_status": _alarm_samples(),
        }
    )

    assert status == "RECOVERED"
    assert "IDs 1/2/3" in summary
    assert "session error" in summary


def test_modbus_summary_does_not_recover_without_all_driver_alarm_samples():
    status, summary = HealthMonitor._modbus_validator(
        {
            "state": {"state_driver": 0, "state_message": 0, "state_error": 0},
            "last_response_age_sec": 0.1,
            "last_read_response_age_sec": 0.2,
            "last_direct_io_age_sec": 0.3,
            "last_electrical_age_sec": 0.4,
            "driver_error_event_count": 1,
            "last_driver_error_age_sec": 12.0,
            "alarm_status": _alarm_samples()[:2],
        }
    )

    assert status == "DEGRADED"
    assert "waiting for 10s" in summary


def test_dynamixel_summary_uses_direct_read_result():
    status, summary = HealthMonitor._dynamixel_validator(
        {
            "motors": [
                {
                    "id": dxl_id,
                    "observed": True,
                    "comm_success": True,
                    "device_error": 0,
                    "last_success_age_sec": 0.1,
                }
                for dxl_id in (11, 12, 13)
            ]
        }
    )

    assert status == "OBSERVED"
    assert "all configured" in summary


def test_dynamixel_identity_uses_direct_ping_model_numbers():
    status, summary = HealthMonitor._dynamixel_identity_validator(
        {
            "motors": [
                {"id": dxl_id, "identity_observed": True, "model_number": 1234}
                for dxl_id in (11, 12, 13)
            ]
        }
    )

    assert status == "OBSERVED"
    assert "ID 11=1234" in summary


def test_dynamixel_hardware_status_uses_verified_direct_registers():
    status, summary = HealthMonitor._dynamixel_hardware_validator(
        {
            "motors": [
                {
                    "id": dxl_id,
                    "hardware_status_supported": True,
                    "hardware_observed": True,
                    "hardware_comm_success": True,
                    "hardware_device_error": 0,
                    "hardware_error_status": 0,
                    "last_hardware_success_age_sec": 0.2,
                    "input_voltage_v": 12.1,
                    "present_current_a": 0.15,
                    "temperature_c": 35,
                }
                for dxl_id in (11, 12, 13)
            ]
        }
    )

    assert status == "OK"
    assert "12.1V" in summary


def test_dynamixel_hardware_error_register_is_not_reported_healthy():
    motors = [
        {
            "id": dxl_id,
            "hardware_status_supported": True,
            "hardware_observed": True,
            "hardware_comm_success": True,
            "hardware_device_error": 0,
            "hardware_error_status": 0,
            "last_hardware_success_age_sec": 0.2,
            "input_voltage_v": 12.1,
            "present_current_a": 0.15,
            "temperature_c": 35,
        }
        for dxl_id in (11, 12, 13)
    ]
    motors[1]["hardware_error_status"] = 4

    status, summary = HealthMonitor._dynamixel_hardware_validator({"motors": motors})

    assert status == "ERROR"
    assert "ID 12=0x04" in summary


def _so101_hardware_status() -> dict:
    motors = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )
    return {
        "healthy": True,
        "control_mode": "real2sim",
        "voltage_deci_v": {name: 125 for name in motors},
        "temperature_c": {name: 35 for name in motors},
        "current_raw": {name: 0 for name in motors},
        "torque_enable": {name: 0 for name in motors},
        "unexpected_torque": {},
        "hold": {"state": "RELEASED", "reason": "startup torque-off"},
    }


def test_so101_arm_hardware_status_uses_all_six_direct_register_sets():
    status, summary = HealthMonitor._so101_arm_hardware_validator(
        _so101_hardware_status()
    )

    assert status == "OK"
    assert "12.5-12.5V" in summary
    assert "torque 0/6" in summary


def test_so101_arm_hold_fault_is_not_reported_healthy():
    value = _so101_hardware_status()
    value["hold"] = {"state": "FAULT", "reason": "HOLD_OVERTEMPERATURE"}

    status, summary = HealthMonitor._so101_arm_hardware_validator(value)

    assert status == "ERROR"
    assert "HOLD_OVERTEMPERATURE" in summary


def test_so101_arm_unexpected_torque_is_error():
    value = _so101_hardware_status()
    value["unexpected_torque"] = {"elbow_flex": 1}

    status, summary = HealthMonitor._so101_arm_hardware_validator(value)

    assert status == "ERROR"
    assert "unexpected torque" in summary


def test_so101_gateway_without_feedback_is_not_healthy():
    status, summary = HealthMonitor._so101_arm_gateway_validator(
        {
            "hardware": {"real_hardware_enabled": True},
            "health": "MODE_SYNCING",
            "control_mode_synced": False,
            "feedback_age_sec": None,
            "controller": {"action_ready": False},
        }
    )
    assert status == "UNKNOWN"
    assert "no usable real joint feedback" in summary


def test_so101_gateway_ready_requires_fresh_feedback():
    status, _summary = HealthMonitor._so101_arm_gateway_validator(
        {
            "hardware": {"real_hardware_enabled": True},
            "health": "REAL2SIM_READY",
            "control_mode_synced": True,
            "feedback_age_sec": 0.05,
            "controller": {"action_ready": True},
        }
    )
    assert status == "OK"


def test_so101_visual_joint_state_requires_all_six_joints():
    status, summary = HealthMonitor._so101_visual_joint_state_validator(
        {
            "names": ["shoulder_pan_joint"],
            "positions": [0.0],
        }
    )
    assert status == "ERROR"
    assert "missing joints" in summary


def test_blvd_direct_io_reports_qualified_raw_din3_mapping():
    status, summary = HealthMonitor._blvd_direct_io_validator(
        {
            "direct_io": [
                {"slave_id": slave_id, "raw": 8, "din3": True}
                for slave_id in (1, 2, 3)
            ],
            "last_direct_io_age_sec": 0.2,
        }
    )

    assert status == "OBSERVED"
    assert "released=1" in summary


def test_physical_estop_released_requires_all_three_din3_high():
    status, summary = HealthMonitor._physical_estop_validator(
        {
            "direct_io": [
                {"slave_id": slave_id, "raw": 196616, "din3": True}
                for slave_id in (1, 2, 3)
            ],
            "last_direct_io_age_sec": 0.2,
        }
    )

    assert status == "OK"
    assert "released" in summary


def test_physical_estop_pressed_requires_all_three_din3_low():
    status, summary = HealthMonitor._physical_estop_validator(
        {
            "direct_io": [
                {"slave_id": slave_id, "raw": 196608, "din3": False}
                for slave_id in (1, 2, 3)
            ],
            "last_direct_io_age_sec": 0.2,
        }
    )

    assert status == "BLOCKED"
    assert "pressed/locked" in summary


def test_physical_estop_disagreement_is_error():
    status, summary = HealthMonitor._physical_estop_validator(
        {
            "direct_io": [
                {"slave_id": 1, "raw": 196616, "din3": True},
                {"slave_id": 2, "raw": 196608, "din3": False},
                {"slave_id": 3, "raw": 196608, "din3": False},
            ],
            "last_direct_io_age_sec": 0.2,
        }
    )

    assert status == "ERROR"
    assert "disagree" in summary


def test_blvd_electrical_summary_uses_three_axis_official_monitor_values():
    status, summary = HealthMonitor._blvd_electrical_validator(
        {
            "last_electrical_age_sec": 0.4,
            "electrical_status": [
                {
                    "slave_id": slave_id,
                    "wheel_position": position,
                    "main_power_supply_current_a": 0.125 + slave_id * 0.001,
                    "inverter_voltage_v": 24.1,
                    "main_power_supply_voltage_v": 24.3,
                }
                for slave_id, position in (
                    (1, "back"),
                    (2, "front_left"),
                    (3, "front_right"),
                )
            ],
        }
    )

    assert status == "OBSERVED"
    assert "ID 1 back" in summary
    assert "ID 3 front_right" in summary


def test_blvd_electrical_summary_is_unknown_before_first_direct_sample():
    status, summary = HealthMonitor._blvd_electrical_validator(
        {"last_electrical_age_sec": None, "electrical_status": None}
    )

    assert status == "UNKNOWN"
    assert "No three-axis" in summary


def _alarm_samples(overrides=None):
    overrides = overrides or {}
    samples = []
    for slave_id, position in ((1, "back"), (2, "front_left"), (3, "front_right")):
        item = {
            "slave_id": slave_id,
            "wheel_position": position,
            "present_alarm": 0,
            "alarm_history_1": 0,
            "alarm_history_changed_in_session": False,
            "sample_age_sec": 0.4,
        }
        item.update(overrides.get(slave_id, {}))
        samples.append(item)
    return samples


def test_blvd_alarm_status_requires_three_fresh_zero_present_alarms():
    status, summary = HealthMonitor._blvd_alarm_validator(
        {"alarm_status": _alarm_samples()}
    )

    assert status == "OBSERVED"
    assert "IDs 1/2/3" in summary


def test_blvd_present_alarm_is_error_and_identifies_physical_driver():
    status, summary = HealthMonitor._blvd_alarm_validator(
        {"alarm_status": _alarm_samples({1: {"present_alarm": 0x30}})}
    )

    assert status == "ERROR"
    assert "ID 1=0x30" in summary


def test_blvd_alarm_history_change_is_recovered_after_current_alarm_clears():
    status, summary = HealthMonitor._blvd_alarm_validator(
        {
            "alarm_status": _alarm_samples(
                {1: {"alarm_history_1": 0x41, "alarm_history_changed_in_session": True}}
            )
        }
    )

    assert status == "RECOVERED"
    assert "history changed" in summary


def test_drive_restart_success_is_visible_as_recovered_not_current_ok():
    status, summary = HealthMonitor._drive_restart_validator(
        {
            "drive_restart": {
                "state": "SUCCEEDED",
                "message": "Drive motor feedback recovered.",
            }
        }
    )

    assert status == "RECOVERED"
    assert "recovery succeeded" in summary


def test_lift_health_remains_estimated_with_open_serial_and_no_hardware_position():
    status, summary = HealthMonitor._lift_health_validator(
        {
            "serial": {"open": True, "write_error_count": 0},
            "motion": {
                "estimated_position_mm": 200.0,
                "command": "STOP",
                "position_reference": "UNREFERENCED_SOFTWARE_ESTIMATE",
            },
        }
    )

    assert status == "ESTIMATED"
    assert "200.0 mm" in summary
    assert "UNREFERENCED" in summary


def test_lift_closed_serial_is_error_even_when_position_number_exists():
    status, summary = HealthMonitor._lift_health_validator(
        {
            "serial": {"open": False, "last_error": "device missing"},
            "motion": {"estimated_position_mm": 200.0},
        }
    )

    assert status == "ERROR"
    assert "device missing" in summary


def test_lift_wrong_usb_identity_is_error():
    status, summary = HealthMonitor._lift_health_validator(
        {
            "serial": {
                "open": True,
                "write_error_count": 0,
                "usb_identity_verified": False,
            },
            "motion": {"estimated_position_mm": 0.0},
        }
    )

    assert status == "ERROR"
    assert "verified Arduino UNO" in summary


def test_lift_v2_protocol_requires_current_ack_verification():
    status, summary = HealthMonitor._lift_health_validator(
        {
            "serial": {
                "open": True,
                "write_error_count": 0,
                "usb_identity_verified": True,
            },
            "protocol": {
                "ready": False,
                "ack_supported": False,
                "consecutive_ack_timeouts": 0,
            },
            "motion": {"estimated_position_mm": 0.0},
        }
    )

    assert status == "DEGRADED"
    assert "ACK" in summary


def test_process_display_name_maps_major_robot_workloads():
    assert HealthMonitor._process_display_name("/opt/ros/lib/controller_server", "server") == "Nav2 controller"
    assert HealthMonitor._process_display_name("python3 health_monitor", "python3") == "Health Monitor"


def test_realsense_diagnostic_names_match_observed_d435_statuses():
    assert (
        HealthMonitor._realsense_diagnostic_key("camera: color")
        == "camera_color_diagnostics"
    )
    assert (
        HealthMonitor._realsense_diagnostic_key("camera: depth")
        == "camera_depth_diagnostics"
    )
    assert (
        HealthMonitor._realsense_diagnostic_key("camera: Temperatures")
        == "camera_temperatures"
    )


def test_realsense_diagnostic_filter_rejects_unrelated_statuses():
    assert HealthMonitor._realsense_diagnostic_key("health_dashboard/H4/laser_scan") is None
    assert HealthMonitor._realsense_diagnostic_key("lifecycle_manager_navigation: Nav2 Health") is None


def test_diagnostic_level_normalizes_humble_byte_representation():
    assert diagnostic_level_number(b"\x00") == 0
    assert diagnostic_level_number(b"\x01") == 1
    assert diagnostic_level_number(2) == 2


def test_realsense_ok_diagnostic_is_accepted_after_level_normalization():
    status, summary = HealthMonitor._diagnostic_status_validator(
        {"level": diagnostic_level_number(b"\x00"), "message": "Desired frequency met"}
    )

    assert status == "OK"
    assert "Desired frequency met" in summary


def _livox_health_state(**state_overrides):
    state = {
        "cur_work_state": 1,
        "core_temp": 4234,
        "time_sync_type": 0,
        "status_code": " ".join(["0"] * 32),
        "lidar_diag_status": 0,
        "hms_code": [0] * 8,
    }
    state.update(state_overrides)
    return {
        "schema_version": 1,
        "devices": [
            {
                "handle": 1,
                "ip": "192.168.1.147",
                "push_age_sec": 0.2,
                "state_json_valid": True,
                "state": state,
            }
        ],
    }


def test_livox_native_state_accepts_sampling_without_external_time_sync():
    status, summary = HealthMonitor._livox_device_health_validator(_livox_health_state())

    assert status == "OK"
    assert "192.168.1.147" in summary


def test_livox_mid360_missing_optional_status_code_is_not_a_fault():
    value = _livox_health_state()
    del value["devices"][0]["state"]["status_code"]

    status, summary = HealthMonitor._livox_device_health_validator(value)

    assert status == "OK"
    assert "status_code not exposed" in summary


def test_livox_diagnostic_warning_degrades_without_inventing_temperature_limit():
    status, summary = HealthMonitor._livox_device_health_validator(
        _livox_health_state(core_temp=9876, lidar_diag_status=0x0001)
    )

    assert status == "DEGRADED"
    assert "warning" in summary


def test_livox_diagnostic_error_is_error():
    status, summary = HealthMonitor._livox_device_health_validator(
        _livox_health_state(lidar_diag_status=0x0020)
    )

    assert status == "ERROR"
    assert "error/safety_error" in summary


def test_livox_nonzero_hms_code_is_visible_as_degraded():
    status, summary = HealthMonitor._livox_device_health_validator(
        _livox_health_state(hms_code=[0x01020002, 0, 0, 0, 0, 0, 0, 0])
    )

    assert status == "DEGRADED"
    assert "0x01020002" in summary


def test_livox_hms_fatal_code_is_error():
    status, summary = HealthMonitor._livox_device_health_validator(
        _livox_health_state(hms_code=[0x01150004, 0, 0, 0, 0, 0, 0, 0])
    )

    assert status == "ERROR"
    assert "level=4" in summary


def test_livox_native_push_age_is_checked_inside_low_rate_ros_summary():
    value = _livox_health_state()
    value["devices"][0]["push_age_sec"] = 3.0

    status, summary = HealthMonitor._livox_device_health_validator(value)

    assert status == "DEGRADED"
    assert "3.00s old" in summary


def test_invalid_livox_diagnostic_payload_does_not_claim_pointcloud_failure():
    status, summary = HealthMonitor._livox_device_health_validator("invalid")

    assert status == "DEGRADED"
    assert "point-cloud state is not inferred" in summary


class _LifecycleFuture:
    def __init__(self, state_id=3, label="active", cancelled=False):
        self._cancelled = cancelled
        self._response = SimpleNamespace(
            current_state=SimpleNamespace(id=state_id, label=label)
        )

    def cancelled(self):
        return self._cancelled

    def result(self):
        return self._response


class _LifecycleMonitorDouble:
    def __init__(self):
        self._lifecycle_pending = {}
        self.marks = []

    def _mark(self, *args, **kwargs):
        self.marks.append((args, kwargs))


def test_lifecycle_result_ignores_late_future_without_removing_retry():
    monitor = _LifecycleMonitorDouble()
    late = _LifecycleFuture()
    retry = _LifecycleFuture()
    monitor._lifecycle_pending["planner"] = (retry, 20.0)

    HealthMonitor._lifecycle_result(
        monitor, "planner", "/planner_server/get_state", late
    )

    assert monitor._lifecycle_pending["planner"][0] is retry
    assert monitor.marks == []


def test_lifecycle_result_accepts_only_current_pending_future():
    monitor = _LifecycleMonitorDouble()
    current = _LifecycleFuture(state_id=3, label="active")
    monitor._lifecycle_pending["planner"] = (current, 20.0)

    HealthMonitor._lifecycle_result(
        monitor, "planner", "/planner_server/get_state", current
    )

    assert "planner" not in monitor._lifecycle_pending
    args, kwargs = monitor.marks[0]
    assert args[:3] == (
        "lifecycle_planner",
        "/planner_server/get_state",
        {"id": 3, "label": "active"},
    )
    assert kwargs["evidence"] == "DIRECT_SERVICE_RESPONSE"
