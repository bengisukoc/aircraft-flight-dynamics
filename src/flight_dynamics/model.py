"""Two-dimensional point-mass aircraft model and simple autopilot."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

G = 9.80665


@dataclass(frozen=True)
class AircraftParameters:
    mass: float = 1_200.0          # kg
    wing_area: float = 16.2        # m^2
    zero_lift_drag: float = 0.025
    induced_drag_factor: float = 0.045
    maximum_lift_coefficient: float = 1.40
    maximum_thrust: float = 5_000.0  # N


def air_density(altitude_m: float) -> float:
    """Return a simple ISA-troposphere density approximation."""
    altitude = max(0.0, float(altitude_m))
    temperature = 288.15 - 0.0065 * min(altitude, 11_000.0)
    pressure = 101_325.0 * (temperature / 288.15) ** 5.25588
    return pressure / (287.05 * temperature)


def _controls(state: np.ndarray, p: AircraftParameters, target_altitude: float,
              target_speed: float) -> tuple[float, float, float, float]:
    _, altitude, speed, gamma = state
    speed = max(speed, 20.0)
    rho = air_density(altitude)
    q = 0.5 * rho * speed ** 2
    gamma_command = np.clip(0.00035 * (target_altitude - altitude),
                            np.deg2rad(-8), np.deg2rad(8))
    trim_cl = p.mass * G * np.cos(gamma) / (q * p.wing_area)
    cl = float(np.clip(trim_cl + 0.85 * (gamma_command - gamma),
                       0.05, p.maximum_lift_coefficient))
    cd = p.zero_lift_drag + p.induced_drag_factor * cl ** 2
    drag = q * p.wing_area * cd
    thrust = float(np.clip(drag + p.mass * G * np.sin(gamma)
                           + p.mass * 0.45 * (target_speed - speed),
                           0.0, p.maximum_thrust))
    return cl, thrust, q, drag


def _derivatives(state: np.ndarray, p: AircraftParameters, target_altitude: float,
                 target_speed: float, wind_mps: float) -> np.ndarray:
    _, altitude, speed, gamma = state
    speed = max(speed, 20.0)
    cl, thrust, q, drag = _controls(state, p, target_altitude, target_speed)
    lift = q * p.wing_area * cl
    dx = speed * np.cos(gamma) + wind_mps
    dh = speed * np.sin(gamma)
    dv = (thrust - drag) / p.mass - G * np.sin(gamma)
    dgamma = (lift - p.mass * G * np.cos(gamma)) / (p.mass * speed)
    return np.array([dx, dh, dv, dgamma])


def _rk4(state: np.ndarray, dt: float, p: AircraftParameters, target_altitude: float,
         target_speed: float, wind_mps: float) -> np.ndarray:
    args = (p, target_altitude, target_speed, wind_mps)
    k1 = _derivatives(state, *args)
    k2 = _derivatives(state + 0.5 * dt * k1, *args)
    k3 = _derivatives(state + 0.5 * dt * k2, *args)
    k4 = _derivatives(state + dt * k3, *args)
    return state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def simulate_flight(parameters: AircraftParameters = AircraftParameters(),
                    duration: float = 180.0, dt: float = 0.05,
                    target_altitude: float = 1_000.0, target_speed: float = 70.0,
                    wind_mps: float = 0.0) -> pd.DataFrame:
    """Simulate a controlled climb from sea level."""
    if duration <= 0 or dt <= 0 or target_speed <= 20:
        raise ValueError("duration, dt and target speed must be physically valid")
    time = np.arange(0, duration + dt, dt)
    state = np.array([0.0, 0.0, 65.0, 0.0])
    rows = []
    for t in time:
        cl, thrust, q, drag = _controls(state, parameters, target_altitude, target_speed)
        rows.append({
            "Time (s)": t,
            "Range (m)": state[0],
            "Altitude (m)": state[1],
            "Airspeed (m/s)": state[2],
            "Flight-path angle (deg)": np.rad2deg(state[3]),
            "Lift coefficient": cl,
            "Thrust (N)": thrust,
            "Drag (N)": drag,
            "Dynamic pressure (Pa)": q,
        })
        state = _rk4(state, dt, parameters, target_altitude, target_speed, wind_mps)
        state[1] = max(0.0, state[1])
    return pd.DataFrame(rows)


def summarise_flight(data: pd.DataFrame, target_altitude: float = 1_000.0) -> dict[str, float]:
    """Calculate climb and steady-flight performance metrics."""
    reached = data[data["Altitude (m)"] >= 0.98 * target_altitude]
    time_to_altitude = float(reached["Time (s)"].iloc[0]) if not reached.empty else np.nan
    return {
        "Time to 98% target altitude (s)": time_to_altitude,
        "Final altitude (m)": float(data["Altitude (m)"].iloc[-1]),
        "Ground distance (km)": float(data["Range (m)"].iloc[-1] / 1000),
        "Maximum flight-path angle (deg)": float(data["Flight-path angle (deg)"].max()),
        "Maximum lift coefficient": float(data["Lift coefficient"].max()),
        "Maximum thrust (N)": float(data["Thrust (N)"].max()),
    }

