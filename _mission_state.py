"""Separation and launch-state helpers for mission orchestration."""


import numpy as np

from . import constants as C
from ._guidance_common import compute_local_vertical
from .config_definition import SimulationConfig
from .frames import rotate_vector_by_quaternion
from .state import State, create_initial_state


def _compute_separation_axis(state: State) -> np.ndarray:
    """Preferred inertial split axis for optional separation dynamics."""
    axis = rotate_vector_by_quaternion(C.BODY_Z_AXIS, state.q)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < C.ZERO_TOLERANCE:
        return compute_local_vertical(state.r)
    return axis / axis_norm


def _compute_separation_spin_axis(state: State, separation_axis: np.ndarray) -> np.ndarray:
    """Stable axis perpendicular to the split direction for residual tumble."""
    reference = state.v if state.speed > C.ZERO_TOLERANCE else state.r
    spin_axis = np.cross(separation_axis, reference)
    spin_norm = float(np.linalg.norm(spin_axis))
    if spin_norm < C.ZERO_TOLERANCE:
        spin_axis = np.cross(separation_axis, np.array([0.0, 0.0, 1.0], dtype=float))
        spin_norm = float(np.linalg.norm(spin_axis))
    if spin_norm < C.ZERO_TOLERANCE:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return spin_axis / spin_norm


def _compute_separation_adjustments(
    state: State,
    orbiter_mass: float,
    booster_mass: float,
    config: SimulationConfig | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return momentum-balanced daughter-state perturbations at separation."""
    zero = np.zeros(3, dtype=float)
    if config is None or not config.enable_separation_dynamics:
        return zero, zero, zero, zero

    total_mass = max(float(orbiter_mass + booster_mass), 1.0)
    separation_axis = _compute_separation_axis(state)
    spin_axis = _compute_separation_spin_axis(state, separation_axis)

    relative_delta_v = max(float(config.separation_delta_v), 0.0)
    tumble_rate = max(float(config.separation_tumble_rate), 0.0)

    orbiter_dv = separation_axis * relative_delta_v * (booster_mass / total_mass)
    booster_dv = -separation_axis * relative_delta_v * (orbiter_mass / total_mass)
    orbiter_domega = spin_axis * tumble_rate * (booster_mass / total_mass)
    booster_domega = -spin_axis * tumble_rate * (orbiter_mass / total_mass)
    return orbiter_dv, booster_dv, orbiter_domega, booster_domega


def _inherit_separation_state(
    state: State,
    mass: float,
    delta_v: np.ndarray | None = None,
    delta_omega: np.ndarray | None = None,
    dry_mass_kg: float | None = None,
) -> State:
    """Clone the current rigid-body kinematics and apply daughter adjustments."""
    inherited = State(
        r=state.r.copy(),
        v=state.v.copy(),
        q=state.q.copy(),
        omega=state.omega.copy(),
        m=mass,
        t=state.t,
        sim_config=state.sim_config,
        dry_mass_kg=dry_mass_kg if dry_mass_kg is not None else state.dry_mass_kg,
    )
    if delta_v is not None:
        inherited.v = inherited.v + np.asarray(delta_v, dtype=float)
    if delta_omega is not None:
        inherited.omega = inherited.omega + np.asarray(delta_omega, dtype=float)
    return inherited


def _create_configured_initial_state(config: SimulationConfig) -> State:
    """Create the nominal launch state with configured mass dispersion applied."""
    state = create_initial_state(config)
    if config is not None:
        state.m += float(config.runtime_initial_mass_offset_kg)
    return state


def _configured_max_mass(config: SimulationConfig) -> float:
    """Upper validation bound for a possibly mass-dispersed launch vehicle."""
    if config is None:
        return C.INITIAL_MASS * 1.01
    initial_mass = (
        float(config.stage1_dry_mass)
        + float(config.stage1_prop_mass)
        + float(config.stage2_dry_mass)
        + float(config.stage2_prop_mass)
        + float(config.payload_mass)
        + max(float(config.runtime_initial_mass_offset_kg), 0.0)
    )
    return initial_mass * 1.01


def _remaining_step_dt(state: State, dt: float, max_time: float) -> float:
    """Return a timestep that cannot integrate past the configured horizon."""
    return max(0.0, min(float(dt), float(max_time) - float(state.t)))


