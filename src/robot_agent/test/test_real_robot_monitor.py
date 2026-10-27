from robot_agent.nodes.real_robot_monitor import drive_restart_guard_rejection


def valid_guard_inputs():
    return {
        "robot_mode": "MANUAL",
        "cmd_age": 0.1,
        "cmd_vel": (0.0, 0.0, 0.0),
        "drive_age": 0.1,
        "drive_values": [0.0, 0.0, 0.0],
        "feedback_stale_sec": 1.0,
        "wheel_speed_limit": 0.01,
        "joy_age": 0.1,
        "joy_source": {
            "source": "PHYSICAL",
            "dashboard_acquired": False,
            "physical_active_age_sec": None,
        },
    }


def test_drive_restart_guard_accepts_direct_stationary_evidence():
    assert drive_restart_guard_rejection(**valid_guard_inputs()) is None


def test_drive_restart_guard_rejects_measured_motion():
    inputs = valid_guard_inputs()
    inputs["drive_values"] = [0.0, 0.02, 0.0]
    assert "measured wheel speed" in drive_restart_guard_rejection(**inputs)


def test_drive_restart_guard_rejects_dashboard_authority():
    inputs = valid_guard_inputs()
    inputs["joy_source"] = {
        "source": "DASHBOARD",
        "dashboard_acquired": True,
        "physical_active_age_sec": None,
    }
    assert "Release Dashboard control" in drive_restart_guard_rejection(**inputs)


def test_drive_restart_guard_rejects_recent_physical_input():
    inputs = valid_guard_inputs()
    inputs["joy_source"]["physical_active_age_sec"] = 0.2
    assert "physical Joy neutral" in drive_restart_guard_rejection(**inputs)
