"""Shared plotting utilities: data extraction, time-series helpers, mission markers."""


import os
from dataclasses import dataclass
from typing import NamedTuple

from matplotlib.axes import Axes

try:

    import matplotlib

    matplotlib.use("Agg")

    import matplotlib.pyplot as plt

except ModuleNotFoundError:

    matplotlib = None

    plt = None

import numpy as np

from . import constants as C
from .utils import compute_relative_velocity

try:

    import matplotlib

    matplotlib.use("Agg")

    import matplotlib.pyplot as plt

except ModuleNotFoundError:

    matplotlib = None

    plt = None


@dataclass

class TrajectoryData:

    """Container for processed trajectory data used in plotting.


    Attributes:

        time: Time array in seconds

        altitude: Altitude array in kilometers

        velocity: Inertial velocity magnitude in m/s

        velocity_rel: Relative (airspeed) velocity magnitude in m/s

        mass: Vehicle mass in kg

        pitch_angle: Commanded pitch angle from vertical in degrees

        actual_pitch: Actual body pitch angle from vertical in degrees

        attitude_error: Attitude tracking error in degrees

        torque: Control torque magnitude in NÂ·m

        gamma_cmd: Commanded flight path angle from horizontal in degrees

        gamma_actual: Actual flight path angle from horizontal in degrees

        gamma_rel: Computed relative flight path angle from horizontal in degrees

        position: Position vector array [n x 3] in meters (ECI)

        velocity_vec: Velocity vector array [n x 3] in m/s (ECI)

        velocity_rel_vec: Relative velocity vector array [n x 3] in m/s

        downrange: Downrange distance in kilometers

        dynamic_pressure: Dynamic pressure in Pascals

        thrust_force: Actual thrust magnitude in N

        # Extended telemetry for research-grade plots

        omega_x: Angular velocity body X component (rad/s)

        omega_y: Angular velocity body Y component (rad/s)

        omega_z: Angular velocity body Z component (rad/s)

        quaternion_norm: Quaternion norm history (should be 1.0)

        velocity_horizontal: Horizontal velocity component (m/s)

        velocity_vertical: Vertical velocity component (m/s)

        velocity_x: ECI velocity X (m/s)

        velocity_y: ECI velocity Y (m/s)

        velocity_z: ECI velocity Z (m/s)

        throttle: Throttle setting (0.0 to 1.0)

        thrust_on: Thrust active flag (0 or 1)

        commanded_quat: Commanded quaternion [n x 4]

        actual_quat: Actual quaternion [n x 4]

        mach_number: Mach number

        temperature: Atmospheric temperature (K)

        pressure: Atmospheric pressure (Pa)

        density: Atmospheric density (kg/m^3)

        speed_of_sound: Speed of sound (m/s)

        downrange_ground: Ground-track downrange (km)

    """

    time: np.ndarray

    altitude: np.ndarray

    velocity: np.ndarray

    velocity_rel: np.ndarray

    mass: np.ndarray

    pitch_angle: np.ndarray

    actual_pitch: np.ndarray

    attitude_error: np.ndarray

    torque: np.ndarray

    gamma_cmd: np.ndarray

    gamma_actual: np.ndarray

    gamma_rel: np.ndarray

    position: np.ndarray

    velocity_vec: np.ndarray

    velocity_rel_vec: np.ndarray

    downrange: np.ndarray

    dynamic_pressure: np.ndarray

    thrust_force: np.ndarray


    omega_x: np.ndarray = None

    omega_y: np.ndarray = None

    omega_z: np.ndarray = None

    quaternion_norm: np.ndarray = None

    velocity_horizontal: np.ndarray = None

    velocity_vertical: np.ndarray = None

    velocity_x: np.ndarray = None

    velocity_y: np.ndarray = None

    velocity_z: np.ndarray = None

    throttle: np.ndarray = None

    thrust_on: np.ndarray = None

    commanded_quat: np.ndarray = None

    actual_quat: np.ndarray = None

    mach_number: np.ndarray = None

    temperature: np.ndarray = None

    pressure: np.ndarray = None

    density: np.ndarray = None

    speed_of_sound: np.ndarray = None

    downrange_ground: np.ndarray = None


class PlotEvent(NamedTuple):

    """Lightweight timeline marker for annotated mission plots."""

    time: float

    label: str

    color: str = '0.35'

    linestyle: str = '--'


def configure_plot_style() -> None:

    """Configure matplotlib defaults for batch trajectory plots."""

    if matplotlib is None or plt is None:

        raise ModuleNotFoundError("matplotlib is required for plot rendering")

    plt.rcParams.update({

        'figure.figsize': (10, 6),

        'figure.dpi': 300,

        'savefig.dpi': 300,

        'axes.grid': True,

        'axes.axisbelow': True,

        'grid.alpha': 0.3,

        'grid.linestyle': '-',

        'grid.linewidth': 0.5,

        'font.size': 11,

        'axes.titlesize': 13,

        'axes.labelsize': 12,

        'legend.fontsize': 10,

        'legend.framealpha': 0.95,

        'legend.edgecolor': 'gray',

        'lines.linewidth': 1.8,

        'lines.markersize': 6,

        'axes.linewidth': 0.8,

        'xtick.direction': 'in',

        'ytick.direction': 'in',

        'xtick.major.width': 0.8,

        'ytick.major.width': 0.8,

    })


def extract_log_data(log) -> TrajectoryData:

    """Extract and process simulation log data for plotting.


    Args:

        log: Simulation log object with trajectory data


    Returns:

        TrajectoryData object with processed arrays


    Raises:

        AttributeError: If required log attributes are missing

    """

    def required_array(name: str) -> np.ndarray:

        if not hasattr(log, name):

            raise AttributeError(f"Missing required telemetry field: {name}")

        arr = np.array(getattr(log, name))

        if len(arr) != len(time):

            raise ValueError(

                f"Telemetry field {name} has {len(arr)} sample(s), expected {len(time)}"

            )

        return arr


    time = np.array(log.time)

    altitude = np.array(log.altitude)

    velocity = np.array(log.velocity)

    mass = np.array(log.mass)

    pitch_angle = np.array(log.pitch_angle)

    attitude_error = np.array(log.attitude_error)

    torque = np.array(log.torque_magnitude)


    pos_x = np.array(log.position_x)

    pos_y = np.array(log.position_y)

    pos_z = np.array(log.position_z)

    vel_x = np.array(log.velocity_x)

    vel_y = np.array(log.velocity_y)

    vel_z = np.array(log.velocity_z)


    th_x = np.array(log.inertial_thrust_x)

    th_y = np.array(log.inertial_thrust_y)

    th_z = np.array(log.inertial_thrust_z)

    thrust_vec = np.column_stack((th_x, th_y, th_z))

    thrust_mag = np.linalg.norm(thrust_vec, axis=1)


    position = np.column_stack((pos_x, pos_y, pos_z))

    velocity_vec = np.column_stack((vel_x, vel_y, vel_z))


    velocity_rel_vec = np.array([

        compute_relative_velocity(p, v)

        for p, v in zip(position, velocity_vec)

    ])

    velocity_rel = np.linalg.norm(velocity_rel_vec, axis=1)


    r_mag = np.linalg.norm(position, axis=1)

    r_hat = position / r_mag[:, np.newaxis]


    v_rel_radial = np.sum(velocity_rel_vec * r_hat, axis=1)

    sin_gamma_rel = np.clip(v_rel_radial / np.maximum(velocity_rel, 1.0), -1.0, 1.0)

    gamma_rel = np.degrees(np.arcsin(sin_gamma_rel))


    gamma_cmd = required_array('gamma_command_deg')

    gamma_actual = required_array('gamma_actual_deg')


    downrange = np.sqrt((pos_x - pos_x[0])**2 + (pos_y - pos_y[0])**2) / 1000.0


    actual_pitch = required_array('actual_pitch_angle')


    omega_x = required_array('omega_x')

    omega_y = required_array('omega_y')

    omega_z = required_array('omega_z')

    quaternion_norm = required_array('quaternion_norm')


    velocity_horizontal = required_array('velocity_horizontal')

    velocity_vertical = required_array('velocity_vertical')


    throttle_arr = required_array('throttle')

    thrust_on_arr = required_array('thrust_on')


    cq_w = required_array('commanded_quat_w')

    cq_x = required_array('commanded_quat_x')

    cq_y = required_array('commanded_quat_y')

    cq_z = required_array('commanded_quat_z')

    aq_w = required_array('actual_quat_w')

    aq_x = required_array('actual_quat_x')

    aq_y = required_array('actual_quat_y')

    aq_z = required_array('actual_quat_z')

    commanded_quat = np.column_stack((cq_w, cq_x, cq_y, cq_z))

    actual_quat = np.column_stack((aq_w, aq_x, aq_y, aq_z))


    from .forces import compute_atmosphere_properties


    atm_temp = np.zeros(len(time))

    atm_pressure = np.zeros(len(time))

    atm_density = np.zeros(len(time))

    atm_sos = np.zeros(len(time))

    mach_arr = np.zeros(len(time))

    for i, (h, v_r) in enumerate(zip(altitude, velocity_rel)):

        T_atm, P_atm, rho_atm, a_atm = compute_atmosphere_properties(h * 1000.0)

        atm_temp[i] = T_atm

        atm_pressure[i] = P_atm

        atm_density[i] = rho_atm

        atm_sos[i] = a_atm

        mach_arr[i] = v_r / a_atm if a_atm > 0 else 0.0

    dynamic_pressure = 0.5 * atm_density * velocity_rel**2


    downrange_ground = required_array('downrange_ground')


    return TrajectoryData(

        time=time,

        altitude=altitude,

        velocity=velocity,

        velocity_rel=velocity_rel,

        mass=mass,

        pitch_angle=pitch_angle,

        actual_pitch=actual_pitch,

        attitude_error=attitude_error,

        torque=torque,

        gamma_cmd=gamma_cmd,

        gamma_actual=gamma_actual,

        gamma_rel=gamma_rel,

        position=position,

        velocity_vec=velocity_vec,

        velocity_rel_vec=velocity_rel_vec,

        downrange=downrange,

        dynamic_pressure=dynamic_pressure,

        thrust_force=thrust_mag,


        omega_x=omega_x,

        omega_y=omega_y,

        omega_z=omega_z,

        quaternion_norm=quaternion_norm,

        velocity_horizontal=velocity_horizontal,

        velocity_vertical=velocity_vertical,

        velocity_x=vel_x,

        velocity_y=vel_y,

        velocity_z=vel_z,

        throttle=throttle_arr,

        thrust_on=thrust_on_arr,

        commanded_quat=commanded_quat,

        actual_quat=actual_quat,

        mach_number=mach_arr,

        temperature=atm_temp,

        pressure=atm_pressure,

        density=atm_density,

        speed_of_sound=atm_sos,

        downrange_ground=downrange_ground,

    )


def compute_gravity_turn_start(data: TrajectoryData, threshold: float = 0.1) -> float:

    """Determine the start time of the gravity turn maneuver.


    Args:

        data: TrajectoryData object

        threshold: Pitch rate threshold in deg/s to detect gravity turn


    Returns:

        Time in seconds when gravity turn begins (as Python float)

    """

    if len(data.time) < 2:

        return 0.0


    dt = np.diff(data.time)


    pitch_rate = np.abs(np.diff(data.pitch_angle) / np.maximum(dt, 1e-9))

    indices = np.where(pitch_rate > threshold)[0]


    if len(indices) > 0:

        return float(data.time[indices[0]])

    return float(data.time[min(20, len(data.time) - 1)])


def _compute_engine_on_mask(data: TrajectoryData) -> np.ndarray:

    """Return boolean mask of samples with meaningful thrust."""

    if data.thrust_force is not None:

        thrust = np.asarray(data.thrust_force)

        if thrust.size > 0 and np.nanmax(thrust) > 0:

            threshold = max(1e3, 0.01 * float(np.nanmax(thrust)))

            return thrust > threshold


    if data.thrust_on is not None:

        return np.asarray(data.thrust_on) > 0.5


    return np.ones(len(data.time), dtype=bool)


def _find_stage_separation_index(

    data: TrajectoryData,

    min_mass_drop_kg: float = 20000.0

) -> int | None:

    """Detect stage-separation index from a large discrete mass drop."""

    if len(data.mass) < 2:

        return None


    dm = np.diff(np.asarray(data.mass))

    idx = int(np.argmin(dm))

    if dm[idx] < -min_mass_drop_kg:

        return idx + 1

    return None


def _find_stage1_meco_index(

    data: TrajectoryData,

    min_off_duration_s: float = 0.5

) -> int:

    """Find S1 MECO as first sustained thrust-off segment after powered ascent."""

    n = len(data.time)

    if n == 0:

        return 0

    if n == 1:

        return 0


    t = np.asarray(data.time)

    on_mask = _compute_engine_on_mask(data)

    off_mask = ~on_mask


    i = 0

    while i < n:

        if off_mask[i]:

            seg_start = i

            while i + 1 < n and off_mask[i + 1]:

                i += 1

            seg_end = i


            off_duration = float(t[seg_end] - t[seg_start]) if seg_end > seg_start else 0.0

            if seg_start > 0 and np.any(on_mask[:seg_start]) and off_duration >= min_off_duration_s:

                return seg_start

        i += 1


    edges = np.where(on_mask[:-1] & (~on_mask[1:]))[0]

    if len(edges) > 0:

        return int(edges[0] + 1)


    sep_idx = _find_stage_separation_index(data)

    if sep_idx is not None and sep_idx > 0:

        return sep_idx - 1


    return n - 1


def _find_stage2_ignition_index(data: TrajectoryData, meco_idx: int) -> int | None:

    """Find first thrust-on sample after S1 MECO (S2 reignition)."""

    on_mask = _compute_engine_on_mask(data)

    if meco_idx >= len(on_mask) - 1:

        return None


    candidates = np.where(on_mask[meco_idx + 1:])[0]

    if len(candidates) == 0:

        return None


    return int(meco_idx + 1 + candidates[0])


def _compute_ground_track_enu(data: TrajectoryData) -> tuple[np.ndarray, np.ndarray]:

    """Project trajectory onto local East/North plane in Earth-fixed coordinates."""

    t = np.asarray(data.time)

    x_eci = data.position[:, 0]

    y_eci = data.position[:, 1]

    z_eci = data.position[:, 2]


    theta = C.EARTH_ROTATION_RATE * t

    cos_t = np.cos(theta)

    sin_t = np.sin(theta)


    x_ecef = x_eci * cos_t + y_eci * sin_t

    y_ecef = -x_eci * sin_t + y_eci * cos_t

    z_ecef = z_eci


    r_ecef = np.column_stack((x_ecef, y_ecef, z_ecef))

    r_norm = np.linalg.norm(r_ecef, axis=1)

    r_hat = r_ecef / np.maximum(r_norm[:, np.newaxis], 1.0)

    r_ground = C.R_EARTH * r_hat


    launch_up = C.INITIAL_POSITION / np.linalg.norm(C.INITIAL_POSITION)

    k_axis = np.array([0.0, 0.0, 1.0])

    launch_east = np.cross(k_axis, launch_up)

    east_norm = np.linalg.norm(launch_east)

    if east_norm < C.ZERO_TOLERANCE:

        launch_east = np.array([0.0, 1.0, 0.0])

        east_norm = 1.0

    launch_east /= east_norm

    launch_north = np.cross(launch_up, launch_east)

    launch_north /= np.linalg.norm(launch_north)


    r0 = C.INITIAL_POSITION

    delta = r_ground - r0

    east_km = delta @ launch_east / 1000.0

    north_km = delta @ launch_north / 1000.0

    return east_km, north_km


@dataclass(frozen=True)
class TimeSeriesLine:
    y: np.ndarray
    label: str = ""
    color: str | None = "#1f77b4"
    linestyle: str = "-"
    linewidth: float = 1.8
    fmt: str | None = None


@dataclass(frozen=True)
class HLine:
    y: float
    label: str = ""
    color: str = "gray"
    linestyle: str = ":"
    linewidth: float = 1.0
    alpha: float = 1.0


ZERO_HLINE = HLine(0.0, alpha=0.5)


@dataclass(frozen=True)
class TimeSeriesPlot:
    filename: str
    title: str
    ylabel: str
    lines: tuple[TimeSeriesLine, ...]
    hlines: tuple[HLine, ...] = ()
    xlabel: str = "Time (s)"
    show_legend: bool = False
    legend_loc: str = "best"
    xlim: tuple[float | None, float | None] | None = None
    ylim: tuple[float | None, float | None] | None = None
    use_time_xlim: bool = True


def _save_figure(fig, output_dir: str, filename: str) -> str:
    plt.tight_layout()
    path = os.path.join(output_dir, filename)
    fig.savefig(path, bbox_inches="tight", dpi=300)
    plt.close(fig)
    return path


def plot_time_series(
    data: TrajectoryData,
    output_dir: str,
    spec: TimeSeriesPlot,
    *,
    x: np.ndarray | None = None,
) -> str:
    fig, ax = plt.subplots()
    x_arr = np.asarray(data.time if x is None else x, dtype=float)
    for line in spec.lines:
        if line.fmt:
            kw: dict = {"linewidth": line.linewidth}
            if line.label:
                kw["label"] = line.label
            ax.plot(x_arr, line.y, line.fmt, **kw)
        else:
            kw = {"color": line.color, "linewidth": line.linewidth}
            if line.linestyle != "-":
                kw["linestyle"] = line.linestyle
            if line.label:
                kw["label"] = line.label
            ax.plot(x_arr, line.y, **kw)
    for hl in spec.hlines:

        hl_kw: dict = {
            "y": hl.y,
            "color": hl.color,
            "linestyle": hl.linestyle,
            "alpha": hl.alpha,
        }
        if hl.linewidth != 1.0:
            hl_kw["linewidth"] = hl.linewidth
        if hl.label:
            hl_kw["label"] = hl.label
        ax.axhline(**hl_kw)
    ax.set_xlabel(spec.xlabel)
    ax.set_ylabel(spec.ylabel)
    ax.set_title(spec.title, fontweight="bold")
    if spec.xlim is not None:
        ax.set_xlim(*spec.xlim)
    elif spec.use_time_xlim and x is None:
        ax.set_xlim(0, data.time[-1])
    if spec.ylim is not None:
        ax.set_ylim(*spec.ylim)
    if spec.show_legend:
        ax.legend(loc=spec.legend_loc, framealpha=0.95)
    return _save_figure(fig, output_dir, spec.filename)


def plot_cmd_vs_actual(
    data: TrajectoryData,
    output_dir: str,
    *,
    filename: str,
    title: str,
    ylabel: str,
    cmd: np.ndarray,
    actual: np.ndarray,
    cmd_label: str,
    actual_label: str,
) -> str:
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename=filename,
            title=title,
            ylabel=ylabel,
            lines=(
                TimeSeriesLine(cmd, label=cmd_label, color="#1f77b4", linewidth=2.0),
                TimeSeriesLine(
                    actual,
                    label=actual_label,
                    color="#d62728",
                    linestyle="--",
                    linewidth=1.5,
                ),
            ),
            show_legend=True,
            legend_loc="best",
        ),
    )


@dataclass(frozen=True)
class VehicleSeries:
    time: np.ndarray
    y: np.ndarray
    label: str
    color: str
    linewidth: float = 2.0


@dataclass(frozen=True)
class MultiVehicleSeriesPlot:
    filename: str
    title: str
    ylabel: str
    xlabel: str = "Time (s)"
    figsize: tuple[float, float] = (11, 6)


def _add_timeline_event_markers(ax: Axes, events: list[PlotEvent]) -> None:
    """Draw vertical mission-event markers with compact labels near the top axis edge."""
    if not events:
        return

    seen = set()
    deduped: list[PlotEvent] = []
    for event in events:
        key = (round(float(event.time), 3), event.label)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(event)

    label_offsets = (-16, -32, -48)
    for idx, event in enumerate(deduped):
        ax.axvline(event.time, color=event.color, linestyle=event.linestyle, linewidth=1.1, alpha=0.75)
        ax.annotate(
            event.label,
            xy=(event.time, 1.0),
            xycoords=("data", "axes fraction"),
            xytext=(4, label_offsets[idx % len(label_offsets)]),
            textcoords="offset points",
            rotation=90,
            va="top",
            ha="left",
            fontsize=8,
            color=event.color,
            bbox={"boxstyle": "round,pad=0.18", "facecolor": "white", "edgecolor": event.color, "alpha": 0.8},
        )


def plot_multi_vehicle_series(
    output_dir: str,
    spec: MultiVehicleSeriesPlot,
    series: tuple[VehicleSeries, ...],
    *,
    events: list[PlotEvent] | None = None,
) -> str:
    """Plot ascent/orbiter/booster telemetry on shared time axes (mission split plots)."""
    fig, ax = plt.subplots(figsize=spec.figsize)
    for vehicle in series:
        if len(vehicle.time) > 0:
            ax.plot(
                vehicle.time,
                vehicle.y,
                color=vehicle.color,
                linewidth=vehicle.linewidth,
                label=vehicle.label,
            )
    if events:
        _add_timeline_event_markers(ax, events)
    ax.set_xlabel(spec.xlabel)
    ax.set_ylabel(spec.ylabel)
    ax.set_title(spec.title, fontweight="bold")
    ax.legend(loc="best", framealpha=0.95)
    ax.grid(True, alpha=0.3)
    return _save_figure(fig, output_dir, spec.filename)


