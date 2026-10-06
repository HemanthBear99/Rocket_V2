"""Navigation sensor and state-estimation helpers.

The physics integrator keeps using the exact simulation truth state. This
module provides an optional estimated state for guidance, beginning with a
GPS/GNSS-style position and velocity measurement model.

References and fidelity limits:
    docs/MATHEMATICAL_REFERENCES.md#10-gpsins-navigation

This is a noisy-measurement/dead-reckoning demonstration, not a complete
strapdown INS or covariance-based GPS/INS Kalman filter.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import constants as C
from .config_definition import SimulationConfig
from .utils import vec_norm


@dataclass(frozen=True)
class GPSMeasurement:
    """One GPS/GNSS-like ECI position and velocity measurement."""

    position_eci_m: np.ndarray
    velocity_eci_mps: np.ndarray
    time_s: float
    available: bool
    position_error_m: float
    velocity_error_mps: float


@dataclass(frozen=True)
class AltimeterMeasurement:
    """Landing-altimeter altitude and vertical-velocity measurement."""

    altitude_m: float
    vertical_velocity_mps: float
    time_s: float
    available: bool
    altitude_error_m: float
    velocity_error_mps: float


@dataclass(frozen=True)
class NavigationEstimate:
    """Estimated vehicle state used by guidance when enabled."""

    position_eci_m: np.ndarray
    velocity_eci_mps: np.ndarray
    time_s: float
    gps_available: bool
    uses_truth_fallback: bool
    position_error_m: float
    velocity_error_mps: float
    imu_available: bool = False
    altimeter_available: bool = False
    altimeter_altitude_m: float | None = None


class GPSReceiver:
    """Simple GPS/GNSS receiver model with configurable white noise."""

    def __init__(self, seed: int | None = None):
        self.rng = np.random.default_rng(seed)

    def measure(self, state, config: SimulationConfig) -> GPSMeasurement:
        """Measure ECI position and velocity with Gaussian receiver noise."""
        if not bool(config.enable_gps):
            return GPSMeasurement(
                position_eci_m=np.asarray(state.r, dtype=float).copy(),
                velocity_eci_mps=np.asarray(state.v, dtype=float).copy(),
                time_s=float(state.t),
                available=False,
                position_error_m=0.0,
                velocity_error_mps=0.0,
            )

        position_noise = self.rng.normal(
            0.0,
            float(config.gps_position_sigma_m),
            size=3,
        )
        velocity_noise = self.rng.normal(
            0.0,
            float(config.gps_velocity_sigma_mps),
            size=3,
        )
        position = np.asarray(state.r, dtype=float) + position_noise
        velocity = np.asarray(state.v, dtype=float) + velocity_noise
        return GPSMeasurement(
            position_eci_m=position,
            velocity_eci_mps=velocity,
            time_s=float(state.t),
            available=True,
            position_error_m=float(vec_norm(position_noise)),
            velocity_error_mps=float(vec_norm(velocity_noise)),
        )


class IMUPropagator:
    """Simple IMU-style acceleration propagation model."""

    def __init__(self, seed: int | None = None):
        self.rng = np.random.default_rng(seed)
        self.accel_bias = None

    def measure_acceleration(
        self,
        true_acceleration_eci_mps2: np.ndarray,
        config: SimulationConfig,
        dt: float,
    ) -> np.ndarray:
        """Measure inertial acceleration with bias and white noise."""
        if self.accel_bias is None:
            self.accel_bias = self.rng.normal(
                0.0,
                float(config.imu_accel_bias),
                size=3,
            )
        noise_sigma = float(config.imu_accel_noise) / np.sqrt(max(float(dt), 1e-9))
        noise = self.rng.normal(0.0, noise_sigma, size=3)
        return np.asarray(true_acceleration_eci_mps2, dtype=float) + self.accel_bias + noise


class LandingAltimeter:
    """Low-altitude radar/laser altimeter model for terminal landing."""

    def __init__(self, seed: int | None = None):
        self.rng = np.random.default_rng(seed)

    def measure(self, state, config: SimulationConfig) -> AltimeterMeasurement:
        """Measure altitude above the local spherical surface and vertical speed."""
        altitude = float(vec_norm(state.r) - C.R_EARTH)
        if (
            not bool(config.enable_landing_altimeter)
            or altitude > float(config.landing_altimeter_max_altitude_m)
        ):
            return AltimeterMeasurement(
                altitude_m=altitude,
                vertical_velocity_mps=0.0,
                time_s=float(state.t),
                available=False,
                altitude_error_m=0.0,
                velocity_error_mps=0.0,
            )

        vertical = _unit_or_default(state.r)
        true_vertical_velocity = float(np.dot(state.v, vertical))
        altitude_noise = float(
            self.rng.normal(0.0, float(config.landing_altimeter_altitude_sigma_m))
        )
        velocity_noise = float(
            self.rng.normal(0.0, float(config.landing_altimeter_velocity_sigma_mps))
        )
        return AltimeterMeasurement(
            altitude_m=altitude + altitude_noise,
            vertical_velocity_mps=true_vertical_velocity + velocity_noise,
            time_s=float(state.t),
            available=True,
            altitude_error_m=abs(altitude_noise),
            velocity_error_mps=abs(velocity_noise),
        )


@dataclass
class NavigationState:
    """Mutable per-vehicle navigation state."""

    estimate: NavigationEstimate | None = None
    last_gps_time_s: float | None = None
    gps_receiver: GPSReceiver | None = None
    imu_propagator: IMUPropagator | None = None
    landing_altimeter: LandingAltimeter | None = None
    previous_truth_velocity_eci_mps: np.ndarray | None = None
    previous_truth_time_s: float | None = None


def _unit_or_default(vector: np.ndarray) -> np.ndarray:
    norm = float(vec_norm(vector))
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0])
    return np.asarray(vector, dtype=float) / norm


def _truth_estimate(state, gps_available: bool = False) -> NavigationEstimate:
    return NavigationEstimate(
        position_eci_m=np.asarray(state.r, dtype=float).copy(),
        velocity_eci_mps=np.asarray(state.v, dtype=float).copy(),
        time_s=float(state.t),
        gps_available=gps_available,
        uses_truth_fallback=not gps_available,
        position_error_m=0.0,
        velocity_error_mps=0.0,
        imu_available=False,
        altimeter_available=False,
        altimeter_altitude_m=None,
    )


def _with_error_metrics(estimate: NavigationEstimate, state) -> NavigationEstimate:
    return NavigationEstimate(
        position_eci_m=estimate.position_eci_m,
        velocity_eci_mps=estimate.velocity_eci_mps,
        time_s=estimate.time_s,
        gps_available=estimate.gps_available,
        uses_truth_fallback=estimate.uses_truth_fallback,
        position_error_m=float(vec_norm(estimate.position_eci_m - state.r)),
        velocity_error_mps=float(vec_norm(estimate.velocity_eci_mps - state.v)),
        imu_available=estimate.imu_available,
        altimeter_available=estimate.altimeter_available,
        altimeter_altitude_m=estimate.altimeter_altitude_m,
    )


def _apply_altimeter_correction(
    estimate: NavigationEstimate,
    measurement: AltimeterMeasurement,
    state,
) -> NavigationEstimate:
    if not measurement.available:
        return estimate

    vertical = _unit_or_default(state.r)
    estimated_position = np.asarray(estimate.position_eci_m, dtype=float)
    estimated_velocity = np.asarray(estimate.velocity_eci_mps, dtype=float)

    horizontal_position = estimated_position - float(np.dot(estimated_position, vertical)) * vertical
    corrected_position = horizontal_position + (C.R_EARTH + measurement.altitude_m) * vertical

    horizontal_velocity = estimated_velocity - float(np.dot(estimated_velocity, vertical)) * vertical
    corrected_velocity = horizontal_velocity + measurement.vertical_velocity_mps * vertical

    return NavigationEstimate(
        position_eci_m=corrected_position,
        velocity_eci_mps=corrected_velocity,
        time_s=measurement.time_s,
        gps_available=estimate.gps_available,
        uses_truth_fallback=estimate.uses_truth_fallback,
        position_error_m=estimate.position_error_m,
        velocity_error_mps=estimate.velocity_error_mps,
        imu_available=estimate.imu_available,
        altimeter_available=True,
        altimeter_altitude_m=measurement.altitude_m,
    )


def update_navigation_estimate(
    nav_state: NavigationState | None,
    state,
    config: SimulationConfig,
    dt: float,
) -> NavigationState:
    """Update the per-vehicle navigation estimate from truth and sensors."""
    nav_state = nav_state or NavigationState()

    if nav_state.gps_receiver is None:
        nav_state.gps_receiver = GPSReceiver(seed=config.gps_seed)
    if nav_state.imu_propagator is None:
        imu_seed = None if config.gps_seed is None else config.gps_seed + 1
        nav_state.imu_propagator = IMUPropagator(seed=imu_seed)
    if nav_state.landing_altimeter is None:
        alt_seed = None if config.gps_seed is None else config.gps_seed + 2
        nav_state.landing_altimeter = LandingAltimeter(seed=alt_seed)

    sensors_enabled = (
        bool(config.enable_gps)
        or bool(config.enable_imu)
        or bool(config.enable_landing_altimeter)
    )
    if not sensors_enabled:
        nav_state.estimate = _truth_estimate(state, gps_available=False)
        nav_state.previous_truth_velocity_eci_mps = np.asarray(state.v, dtype=float).copy()
        nav_state.previous_truth_time_s = float(state.t)
        return nav_state

    gps_due = bool(config.enable_gps) and (
        nav_state.last_gps_time_s is None
        or float(state.t) - nav_state.last_gps_time_s >= float(config.gps_update_period_s)
    )
    if gps_due:
        measurement = nav_state.gps_receiver.measure(state, config)
        nav_state.last_gps_time_s = float(state.t)
        nav_state.estimate = NavigationEstimate(
            position_eci_m=measurement.position_eci_m,
            velocity_eci_mps=measurement.velocity_eci_mps,
            time_s=measurement.time_s,
            gps_available=measurement.available,
            uses_truth_fallback=False,
            position_error_m=measurement.position_error_m,
            velocity_error_mps=measurement.velocity_error_mps,
            imu_available=False,
            altimeter_available=False,
            altimeter_altitude_m=None,
        )
    elif (
        bool(config.enable_imu)
        and nav_state.estimate is not None
        and nav_state.previous_truth_velocity_eci_mps is not None
        and nav_state.previous_truth_time_s is not None
    ):
        elapsed = max(float(state.t) - float(nav_state.estimate.time_s), float(dt), 1e-9)
        truth_elapsed = max(float(state.t) - float(nav_state.previous_truth_time_s), elapsed, 1e-9)
                                                                          
        true_accel_eci = (
            np.asarray(state.v, dtype=float) - nav_state.previous_truth_velocity_eci_mps
        ) / truth_elapsed
                                                                               
                                                                       
        r_current = np.asarray(state.r, dtype=float)
        r_norm = float(vec_norm(r_current))
        g_accel = -C.MU_EARTH * r_current / max(r_norm ** 3, 1.0)
        specific_force = true_accel_eci - g_accel
        measured_specific_force = nav_state.imu_propagator.measure_acceleration(
            specific_force,
            config,
            elapsed,
        )
                                                                        
        a_total_est = measured_specific_force + g_accel
        previous_position = np.asarray(nav_state.estimate.position_eci_m, dtype=float)
        previous_velocity = np.asarray(nav_state.estimate.velocity_eci_mps, dtype=float)
        propagated_velocity = previous_velocity + a_total_est * elapsed
        propagated_position = (
            previous_position
            + previous_velocity * elapsed
            + 0.5 * a_total_est * elapsed ** 2
        )
        nav_state.estimate = NavigationEstimate(
            position_eci_m=propagated_position,
            velocity_eci_mps=propagated_velocity,
            time_s=float(state.t),
            gps_available=False,
            uses_truth_fallback=False,
            position_error_m=0.0,
            velocity_error_mps=0.0,
            imu_available=True,
            altimeter_available=False,
            altimeter_altitude_m=None,
        )
    elif nav_state.estimate is None:
        nav_state.estimate = _truth_estimate(state, gps_available=False)

    if nav_state.estimate is not None and bool(config.enable_landing_altimeter):
        altimeter_measurement = nav_state.landing_altimeter.measure(state, config)
        nav_state.estimate = _apply_altimeter_correction(
            nav_state.estimate,
            altimeter_measurement,
            state,
        )

    if nav_state.estimate is not None:
        nav_state.estimate = _with_error_metrics(nav_state.estimate, state)

    nav_state.previous_truth_velocity_eci_mps = np.asarray(state.v, dtype=float).copy()
    nav_state.previous_truth_time_s = float(state.t)
    return nav_state
