import numpy as np
import pytest

from flight_dynamics import AircraftParameters, air_density, simulate_flight, summarise_flight


def test_density_decreases_with_altitude():
    assert air_density(5000) < air_density(0)


def test_baseline_reaches_target_region():
    data = simulate_flight(duration=180)
    assert data["Altitude (m)"].iloc[-1] == pytest.approx(1000, abs=25)
    assert np.isfinite(summarise_flight(data)["Time to 98% target altitude (s)"])


def test_headwind_reduces_ground_range_not_airspeed():
    calm = simulate_flight(duration=60, wind_mps=0)
    headwind = simulate_flight(duration=60, wind_mps=-15)
    assert headwind["Range (m)"].iloc[-1] < calm["Range (m)"].iloc[-1]
    assert headwind["Airspeed (m/s)"].iloc[-1] == pytest.approx(
        calm["Airspeed (m/s)"].iloc[-1], rel=1e-6
    )


def test_physical_limits_are_respected():
    p = AircraftParameters()
    data = simulate_flight(parameters=p, duration=30)
    assert data["Lift coefficient"].max() <= p.maximum_lift_coefficient
    assert data["Thrust (N)"].max() <= p.maximum_thrust

