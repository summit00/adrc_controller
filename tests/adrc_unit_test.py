import pytest
import sys
from pathlib import Path
import numpy as np

project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root))

from adrc.adrcController import ADRCController

@pytest.mark.parametrize("dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected", [
    (0.001, 90, 10, 2.0, 1.0, 0.0, 50.0),  # First-order: u = (kp*(r-z1) - z2)/b0, initial z1=measurement, z2=0
])
def test_adrc_first_order_basic(dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected):
    adrc = ADRCController(order=1, dt=dt, bandwidth_eso=bandwidth_eso, bandwidth_controller=bandwidth_controller, b0=b0)
    adrc.set_output_limits(-100, 100)
    adrc.reset()

    output = adrc.update(setpoint, measurement)
    assert abs(output - expected) < 1e-6, f"Expected {expected}, got {output}"


@pytest.mark.parametrize("dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected", [
    (0.001, 90, 10, 2.0, 10.0, 0.0, 100.0),  # Should saturate at max
])
def test_adrc_first_order_saturation_max(dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected):
    adrc = ADRCController(order=1, dt=dt, bandwidth_eso=bandwidth_eso, bandwidth_controller=bandwidth_controller, b0=b0)
    adrc.set_output_limits(-100, 100)
    adrc.reset()

    output = adrc.update(setpoint, measurement)
    assert output == expected, f"Expected saturation at {expected}, got {output}"


@pytest.mark.parametrize("dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected", [
    (0.001, 90, 10, 2.0, -10.0, 0.0, -100.0),  # Should saturate at min
])
def test_adrc_first_order_saturation_min(dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected):
    adrc = ADRCController(order=1, dt=dt, bandwidth_eso=bandwidth_eso, bandwidth_controller=bandwidth_controller, b0=b0)
    adrc.set_output_limits(-100, 100)
    adrc.reset()

    output = adrc.update(setpoint, measurement)
    assert output == expected, f"Expected saturation at {expected}, got {output}"


@pytest.mark.parametrize("dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected", [
    (0.001, 90, 10, 2.0, 1.0, 0.0, 50.0),  # Second-order: u = (kp*(r-z1) - kd*z2 - z3)/b0, initial z1=measurement, z2=0, z3=0
])
def test_adrc_second_order_basic(dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected):
    adrc = ADRCController(order=2, dt=dt, bandwidth_eso=bandwidth_eso, bandwidth_controller=bandwidth_controller, b0=b0)
    adrc.set_output_limits(-100, 100)
    adrc.reset()

    output = adrc.update(setpoint, measurement)
    assert abs(output - expected) < 1e-6, f"Expected {expected}, got {output}"


@pytest.mark.parametrize("dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected", [
    (0.001, 90, 10, 2.0, 10.0, 0.0, 100.0),  # Should saturate at max
])
def test_adrc_second_order_saturation_max(dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected):
    adrc = ADRCController(order=2, dt=dt, bandwidth_eso=bandwidth_eso, bandwidth_controller=bandwidth_controller, b0=b0)
    adrc.set_output_limits(-100, 100)
    adrc.reset()

    output = adrc.update(setpoint, measurement)
    assert output == expected, f"Expected saturation at {expected}, got {output}"


@pytest.mark.parametrize("dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected", [
    (0.001, 90, 10, 2.0, -10.0, 0.0, -100.0),  # Should saturate at min
])
def test_adrc_second_order_saturation_min(dt, bandwidth_eso, bandwidth_controller, b0, setpoint, measurement, expected):
    adrc = ADRCController(order=2, dt=dt, bandwidth_eso=bandwidth_eso, bandwidth_controller=bandwidth_controller, b0=b0)
    adrc.set_output_limits(-100, 100)
    adrc.reset()

    output = adrc.update(setpoint, measurement)
    assert output == expected, f"Expected saturation at {expected}, got {output}"


# Test that an AssertionError is raised if ESO bandwidth is too high for the sample time
def test_adrc_bandwidth_assertion():
    dt = 0.001
    bandwidth_eso = 200  # 200 * 0.001 = 0.2 > 0.1, should fail
    bandwidth_controller = 10
    b0 = 2.0
    with pytest.raises(AssertionError, match=r"ESO bandwidth too high for sample time"):
        ADRCController(order=1, dt=dt, bandwidth_eso=bandwidth_eso, bandwidth_controller=bandwidth_controller, b0=b0)
