"""
RLV Phase-I Ascent Simulation - Physical Constants and Vehicle Parameters

This module defines all physical constants, Earth parameters, vehicle specifications,
and control system parameters used throughout the simulation.

VALUES FROM: Developer_Implementation.pdf - Appendix A
"""

import numpy as np

MU_EARTH = 3.986004418e14


R_EARTH = 6.371e6

# Equatorial (not mean) Earth radius, WGS84 - required by the J2 and EGM96
# gravity terms, which are normalized to the equatorial radius by definition.
R_EARTH_EQUATORIAL_WGS84 = 6378137.0


G0 = 9.80665


RHO_0 = 1.225
H_SCALE = 8500.0


STAGE1_DRY_MASS = 22200.0
STAGE1_PROPELLANT_MASS = 410900.0
STAGE1_WET_MASS = 433100.0


STAGE2_MASS = 111500.0
STAGE2_DRY_MASS = 4000.0
STAGE2_PROPELLANT_MASS = 107500.0
STAGE2_WET_MASS = STAGE2_DRY_MASS + STAGE2_PROPELLANT_MASS


DRY_MASS = STAGE1_DRY_MASS + STAGE2_MASS
PROPELLANT_MASS = STAGE1_PROPELLANT_MASS
INITIAL_MASS = STAGE1_WET_MASS + STAGE2_MASS


STAGE1_LANDING_FUEL_RESERVE = 35000.0
STAGE1_ASCENT_PROPELLANT = STAGE1_PROPELLANT_MASS - STAGE1_LANDING_FUEL_RESERVE


THRUST_MAGNITUDE = 7.607e6
ISP = 282.0


# Stage-1 engine cluster; recovery burns use a subset of identical engines.
STAGE1_ENGINE_COUNT = 9
STAGE1_ENGINE_THRUST = THRUST_MAGNITUDE / STAGE1_ENGINE_COUNT
BOOSTBACK_ENGINE_COUNT = 3
ENTRY_ENGINE_COUNT = 3
LANDING_ENGINE_COUNT = 1
BOOSTBACK_THRUST = BOOSTBACK_ENGINE_COUNT * STAGE1_ENGINE_THRUST
ENTRY_THRUST = ENTRY_ENGINE_COUNT * STAGE1_ENGINE_THRUST
LANDING_THRUST = LANDING_ENGINE_COUNT * STAGE1_ENGINE_THRUST


MASS_FLOW_RATE = THRUST_MAGNITUDE / (ISP * G0)


DRAG_COEFFICIENT = 0.42
REFERENCE_AREA = 10.75
REFERENCE_DIAMETER = 3.7


C_N_ALPHA = 4.0


C_M_DAMPING = -8.0


H_STAGE1 = 47.0
H_STACK = 70.0


H_CP = 33.0


IXX_FULL = 7.2e7
IZZ_FULL = 2.0e6


IXX_EMPTY = 1.55e7
IZZ_EMPTY = 1.0e6


INERTIA_TENSOR_FULL = np.diag([IXX_FULL, IXX_FULL, IZZ_FULL])
INERTIA_TENSOR_EMPTY = np.diag([IXX_EMPTY, IXX_EMPTY, IZZ_EMPTY])


STAGE1_RECOVERY_CG_FULL = 21.0
STAGE1_RECOVERY_CG_EMPTY = 26.0
STAGE1_RECOVERY_CP = 31.0
STAGE1_RECOVERY_IXX_FULL = 1.1e7
STAGE1_RECOVERY_IXX_EMPTY = 5.5e6
STAGE1_RECOVERY_IZZ_FULL = 1.03e5
STAGE1_RECOVERY_IZZ_EMPTY = 5.15e4


MAX_DYNAMIC_PRESSURE = 35000.0


KP_ATTITUDE = 1.6e8
KD_ATTITUDE = 1.07e8


KI_ATTITUDE = 8.0e6

ATTITUDE_INTEGRAL_WINDUP_FRACTION = 0.35


MAX_TORQUE = 3.2e7


BOOSTER_MAX_TORQUE_NM = 1.2e7

# Controller gain-scheduling tuning knob for the recovered booster's attitude
# control loop ONLY (see _simulation_step.py's control_inertia calculation).
# It scales the inertia value fed to control._schedule_gains, not the actual
# rigid-body inertia used in the Euler rotational equation of motion (that
# physical inertia comes from mass.compute_inertia_tensor and is unaffected
# by this constant). In effect it detunes the booster's gain-scheduled gains
# below what the vehicle's true empty-stage inertia would otherwise produce.
# This is an empirical controller detune with no underlying physics equation
# to check it against - not a vehicle property.
BOOSTER_CONTROL_INERTIA_SCALE = 0.8


MAX_GIMBAL_ANGLE = np.radians(15.0)
S1_TVC_LEVER_ARM = 32.0
S2_TVC_LEVER_ARM = 12.0
BOOSTER_TVC_LEVER_ARM = 21.0


RCS_LEVER_ARM = 20.0


RCS_PROPELLANT_MASS = 300.0
RCS_ISP = 220.0
RCS_THRUST_PER_THRUSTER = 500.0

# Full-authority RCS torque at the DEFAULT thruster count (8). rcs.py and
# control.py both need this as the real upper bound of RCS authority: using a
# smaller ad-hoc literal made the RCS-only attitude deadband unreachable,
# because available torque in that regime is 80 kN.m, not <50 kN.m.
RCS_DEFAULT_NUM_THRUSTERS = 8
RCS_FULL_AUTHORITY_NM = (
    RCS_THRUST_PER_THRUSTER * RCS_DEFAULT_NUM_THRUSTERS * RCS_LEVER_ARM
)


PITCHOVER_START_ALTITUDE = 0.0
PITCHOVER_END_ALTITUDE = 2000.0
PITCHOVER_ANGLE = 9.0 * np.pi / 180.0
PITCHOVER_AZIMUTH = 90.0 * np.pi / 180.0
PITCHOVER_RAMP_DISTANCE = 100.0


TARGET_PITCH_ANGLE = np.radians(45.0)


GRAVITY_TURN_START_ALTITUDE = 500.0
GRAVITY_TURN_TRANSITION_RANGE = 20000.0
MIN_VELOCITY_FOR_TURN = 50.0


DT = 0.05


MAX_TIME = 6000.0


S2_COAST_TO_INSERTION_S = 2500.0
S2_RECOVERY_DESCENT_S = 2500.0


S2_RECOVERY_MIN_MAX_TIME_S = 9000.0


TARGET_ALTITUDE = 110000.0
TARGET_SPEED = 2500.0


DEFAULT_LAUNCH_SITE_LAT_DEG = 28.608402
DEFAULT_LAUNCH_SITE_LON_DEG = -80.604201

DEFAULT_LANDING_SITE_LAT_DEG = DEFAULT_LAUNCH_SITE_LAT_DEG
DEFAULT_LANDING_SITE_LON_DEG = DEFAULT_LAUNCH_SITE_LON_DEG


def surface_position_from_lat_lon_deg(
    lat_deg: float,
    lon_deg: float,
    radius_m: float = R_EARTH,
    altitude_m: float = 0.0,
) -> np.ndarray:
    """Return a spherical-Earth surface position from geodetic-style lat/lon."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    radius = radius_m + altitude_m
    cos_lat = np.cos(lat)
    return np.array([
        radius * cos_lat * np.cos(lon),
        radius * cos_lat * np.sin(lon),
        radius * np.sin(lat),
    ], dtype=float)


def _quaternion_align_body_z(direction: np.ndarray) -> np.ndarray:
    """Quaternion rotating body +Z onto the requested inertial direction."""
    reference = np.array([0.0, 0.0, 1.0], dtype=float)
    target = np.asarray(direction, dtype=float)
    target_norm = np.linalg.norm(target)
    if target_norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    target = target / target_norm
    dot = float(np.clip(np.dot(reference, target), -1.0, 1.0))
    if dot > 0.999999:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    if dot < -0.999999:
        axis = np.array([1.0, 0.0, 0.0], dtype=float)
        return np.array([0.0, axis[0], axis[1], axis[2]], dtype=float)

    axis = np.cross(reference, target)
    quat = np.array([1.0 + dot, axis[0], axis[1], axis[2]], dtype=float)
    return quat / np.linalg.norm(quat)


INITIAL_POSITION = surface_position_from_lat_lon_deg(
    DEFAULT_LAUNCH_SITE_LAT_DEG,
    DEFAULT_LAUNCH_SITE_LON_DEG,
)


EARTH_ROTATION_RATE = 7.2921159e-5


INITIAL_VELOCITY = np.cross(
    np.array([0.0, 0.0, EARTH_ROTATION_RATE], dtype=float),
    INITIAL_POSITION,
)


INITIAL_QUATERNION = _quaternion_align_body_z(INITIAL_POSITION)


INITIAL_OMEGA = np.array([0.0, 0.0, 0.0])


BODY_Z_AXIS = np.array([0.0, 0.0, 1.0])


ATM_T0 = 288.15
ATM_P0 = 101325.0
ATM_RHO0 = RHO_0


ATM_LAPSE_RATE = 0.0065
ATM_TROPOPAUSE = 11000.0
ATM_T_STRATOSPHERE = 216.65


ATM_SPEED_OF_SOUND_FALLBACK = 340.0


GAMMA = 1.4
R_GAS = 287.05


MACH_BREAKPOINTS = np.array([0.0, 0.8, 1.05, 1.3, 2.0, 5.0, 10.0, 25.0])
CD_VALUES = np.array([0.42, 0.42, 0.75, 0.65, 0.50, 0.35, 0.25, 0.20])


CL_ALPHA_VALUES = np.array([2.0, 2.0, 1.8, 1.6, 1.2, 0.8, 0.6, 0.4])


ISP_VAC = 311.0


STAGE2_THRUST = 950000.0
STAGE2_ISP_VAC = 348.0
STAGE2_MASS_FLOW_RATE = STAGE2_THRUST / (STAGE2_ISP_VAC * G0)
STAGE2_BURN_TIME = STAGE2_PROPELLANT_MASS / STAGE2_MASS_FLOW_RATE


TARGET_ORBIT_ALTITUDE = 400000.0


QUATERNION_NORM_TOL = 1e-6
ENERGY_TOLERANCE = 1e-3


ZERO_TOLERANCE = 1e-10


BOOSTER_RECOVERY_MAX_DT = 0.05


STACKED_ASCENT_MAX_DT = 0.05
SMALL_VELOCITY_TOL = 1e-5
DENSITY_FLOOR = 1e-12
CRASH_ALTITUDE_TOLERANCE = -1000.0


LAUNCH_PAD_CLEAR_ALTITUDE = 50.0


AERO_DISABLE_ALTITUDE = 120000.0


WIND_REF_ALT = 10000.0
WIND_REF_SPEED = 30.0
WIND_EXPONENT = 0.2
WIND_DIRECTION_AZIMUTH = np.radians(90.0)


GRID_FIN_COUNT = 4
GRID_FIN_AREA_M2 = 4.0
GRID_FIN_CD = 1.2
LANDING_LEG_COUNT = 4
LANDING_LEG_AREA_M2 = 0.3
LANDING_LEG_CD = 1.0


GUIDANCE_MIX_START_ALT = 2000.0
GUIDANCE_MIX_END_ALT = 40000.0
GUIDANCE_MAX_Q_OVERRIDE = 20000.0


SEPARATION_ANGULAR_KICK_RADS = 0.005


MAX_PITCH_ANGLE = np.radians(75.0)
MAX_GIMBAL_RATE = np.radians(25.0)
