from robot_agent.lidar_thermal import LidarThermalEvidence
from tools.sim.inject_lidar_thermal import ThermalScenario


def test_sim_temperature_warns_on_rising_trend_before_bad_and_resets_after_clock_reset():
    evidence = LidarThermalEvidence()
    for t in range(0, 21, 2):
        evidence.observe({"core_temp_c": 40 + t * .5}, source_sec=t, sim=True)
    status, _, details = evidence.assess(sim=True)
    assert status == "WARN"
    assert details["core_temp_c"] == 50
    assert details["rise_c_per_sec_30s"] == .5
    evidence.observe({"core_temp_c": 80}, source_sec=80, sim=True)
    assert evidence.assess(sim=True)[0] == "ERROR"
    evidence.observe({"core_temp_c": 40}, source_sec=0, sim=True)
    assert evidence.assess(sim=True)[0] == "OK"


def test_real_temperature_is_observed_without_inventing_device_threshold():
    evidence = LidarThermalEvidence()
    payload = {"devices": [{"ip": "lidar", "state": {"core_temp": 9876}}]}
    evidence.observe(payload, source_sec=10, sim=False)
    status, _, details = evidence.assess(sim=False)
    assert status == "OBSERVED"
    assert details["core_temp_c"] == 98.76
    assert details["threshold_basis"] == "native_diagnostic_only"


def test_real_warning_requires_explicit_calibrated_threshold():
    evidence = LidarThermalEvidence(real_warn_c=70)
    payload = {"devices": [{"ip": "lidar", "state": {"core_temp": 7100}}]}
    evidence.observe(payload, source_sec=10, sim=False)
    assert evidence.assess(sim=False)[0] == "WARN"


def test_sim_thermal_model_uses_sim_time_and_cooling_is_explicit():
    model = ThermalScenario(warm_c_per_sec=1, cool_c_per_sec=2)
    model.advance(10)
    assert model.command("start")
    model.advance(20)
    assert model.temp_c == 50
    assert model.command("cool")
    model.advance(25)
    assert model.temp_c == 40
    assert model.phase == "idle"
    model.command("start")
    assert model.advance(0) == "clock_reset"
    assert model.phase == "idle"
