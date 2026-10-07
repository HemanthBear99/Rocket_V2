"""
RLV Phase-I Ascent Simulation - Global State Vector

This module defines the single global state dataclass that contains all
simulation state variables. No duplicated state is allowed anywhere.
"""

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

import numpy as np

from . import constants as C
from .utils import cross3, vec_norm

if TYPE_CHECKING:
    from .config_definition import SimulationConfig


@dataclass
class State:
    """
    Global state vector for the RLV simulation.

    All state variables are stored in a single location to ensure consistency
    and prevent duplication.

    Attributes:
        r: Position vector in inertial frame (m) [3]
        v: Velocity vector in inertial frame (m/s) [3]
        q: Orientation quaternion [w, x, y, z] (unit quaternion)
        omega: Angular velocity in body frame (rad/s) [3]
        m: Total vehicle mass (kg)
        t: Simulation time (s)
    """


    r: np.ndarray = field(default_factory=lambda: np.zeros(3))


    v: np.ndarray = field(default_factory=lambda: np.zeros(3))


    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))


    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))


    m: float = 0.0


    t: float = 0.0


    sim_config: Optional['SimulationConfig'] = field(default=None, repr=False)
    dry_mass_kg: float | None = field(default=None, repr=False)

    def __post_init__(self):
        """Ensure arrays are numpy arrays with correct dtype and q is unit-length."""
        for attr in ['r', 'v', 'q', 'omega']:
            setattr(self, attr, np.asarray(getattr(self, attr), dtype=np.float64))


        q_norm = float(vec_norm(self.q))
        if np.isfinite(q_norm) and abs(q_norm - 1.0) > C.QUATERNION_NORM_TOL:
            raise ValueError(
                f"State quaternion must be unit length (norm={q_norm:.8f}); "
                "normalize the quaternion before constructing State."
            )

    def copy(self) -> 'State':
        """Create a deep copy of the state."""
        return State(
            r=self.r.copy(),
            v=self.v.copy(),
            q=self.q.copy(),
            omega=self.omega.copy(),
            m=self.m,
            t=self.t,
            sim_config=self.sim_config,
            dry_mass_kg=self.dry_mass_kg,
        )

    def to_vector(self) -> np.ndarray:
        """Convert state to a flat numpy array check [r, v, q, omega, m]."""
        return np.concatenate([
            self.r, self.v, self.q, self.omega, [self.m]
        ])

    @classmethod
    def from_vector(
        cls,
        vec: np.ndarray,
        t: float,
        *,
        sim_config: Optional['SimulationConfig'] = None,
        dry_mass_kg: float | None = None,
    ) -> 'State':
        """
        Create a State from a flat numpy array.

        Args:
            vec: State vector [r(3), v(3), q(4), omega(3), m(1)]
            t: Current simulation time
            sim_config: Optional runtime configuration carried outside the
                dynamics vector.
            dry_mass_kg: Optional dry mass carried outside the dynamics vector.
        """
        return cls(
            r=vec[0:3].copy(),
            v=vec[3:6].copy(),
            q=vec[6:10].copy(),
            omega=vec[10:13].copy(),
            m=vec[13],
            t=t,
            sim_config=sim_config,
            dry_mass_kg=dry_mass_kg,
        )

    @property
    def altitude(self) -> float:
        """Altitude above the configured Earth surface (m)."""
        from .forces import _altitude_from_r
        return float(_altitude_from_r(self.r, self.sim_config))

    @property
    def speed(self) -> float:
        """Magnitude of velocity (m/s)."""
        return vec_norm(self.v)

    @property
    def propellant_remaining(self) -> float:
        """Remaining propellant mass (kg)."""
        if self.dry_mass_kg is not None:
            dry_mass = float(self.dry_mass_kg)
        elif self.sim_config is not None:
            dry_mass = (
                float(self.sim_config.stage1_dry_mass)
                + float(self.sim_config.stage2_dry_mass)
                + float(self.sim_config.payload_mass)
            )
        else:
            dry_mass = C.DRY_MASS
        return max(0.0, self.m - dry_mass)

    def __str__(self) -> str:
        """Human-readable state summary."""
        return (
            f"State(t={self.t:.2f}s, "
            f"alt={self.altitude/1000:.2f}km, "
            f"v={self.speed:.1f}m/s, "
            f"m={self.m:.1f}kg)"
        )


def _configured_launch_position(config=None) -> np.ndarray:
    """Return the launch-site ECI position at t=0 for the selected Earth model."""
    if config is None:
        return C.INITIAL_POSITION.copy()

    return C.surface_position_from_lat_lon_deg(
        float(config.launch_site_lat_deg),
        float(config.launch_site_lon_deg),
        altitude_m=float(config.launch_site_altitude_m),
    )


def create_initial_state(config=None) -> State:
    """
    Create the initial state for the simulation.

    Returns:
        State object initialized with launch conditions.
    """
    r0 = _configured_launch_position(config)
    v0 = cross3(np.array([0.0, 0.0, C.EARTH_ROTATION_RATE], dtype=float), r0)
    q0 = C._quaternion_align_body_z(r0)

    if config is not None:
        dry_mass = (
            float(config.stage1_dry_mass)
            + float(config.stage2_dry_mass)
            + float(config.payload_mass)
        )
        initial_mass = (
            dry_mass
            + float(config.stage1_prop_mass)
            + float(config.stage2_prop_mass)
        )
    else:
        dry_mass = C.STAGE1_DRY_MASS + C.STAGE2_DRY_MASS
        initial_mass = C.INITIAL_MASS

    return State(
        r=r0,
        v=v0,
        q=q0,
        omega=C.INITIAL_OMEGA.copy(),
        m=initial_mass,
        t=0.0,
        sim_config=config,
        dry_mass_kg=dry_mass,
    )
