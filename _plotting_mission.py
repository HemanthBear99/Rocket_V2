"""Full-mission, booster recovery, and plot orchestration functions."""

import json
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes

from ._plotting_common import (
    MultiVehicleSeriesPlot,
    PlotEvent,
    VehicleSeries,
    _add_timeline_event_markers,
    configure_plot_style,
    extract_log_data,
    plot_multi_vehicle_series,
)
from ._plotting_general import (
    plot_acceleration_profile,
    plot_altitude_profile,
    plot_altitude_rate,
    plot_altitude_vs_velocity,
    plot_angular_velocity_x,
    plot_angular_velocity_y,
    plot_angular_velocity_z,
    plot_ascent_profile,
    plot_atmospheric_density,
    plot_atmospheric_pressure,
    plot_atmospheric_temperature,
    plot_attitude_error,
    plot_commanded_vs_actual_quat_w,
    plot_commanded_vs_actual_quat_x,
    plot_commanded_vs_actual_quat_y,
    plot_commanded_vs_actual_quat_z,
    plot_comprehensive_dashboard,
    plot_control_dynamics,
    plot_control_torque,
    plot_delta_v_budget,
    plot_downrange_distance,
    plot_drag_coefficient_vs_mach,
    plot_drag_loss,
    plot_dynamic_pressure,
    plot_dynamic_pressure_vs_altitude,
    plot_effective_isp,
    plot_energy_budget,
    plot_flight_path_readme,
    plot_geocentric_radius,
    plot_gravity_loss,
    plot_ground_track,
    plot_horizontal_vs_vertical_velocity,
    plot_inertia_variation,
    plot_mach_number,
    plot_mass_flow_rate,
    plot_mass_profile,
    plot_natural_frequency,
    plot_physics_check,
    plot_pitch_angle,
    plot_pitch_gamma_diagnostic,
    plot_position_eci_x,
    plot_position_eci_y,
    plot_position_eci_z,
    plot_propellant_fraction,
    plot_quaternion_norm,
    plot_specific_angular_momentum,
    plot_specific_orbital_energy,
    plot_speed_of_sound,
    plot_throttle_history,
    plot_thrust_vs_gravity,
    plot_trajectory_3d,
    plot_trajectory_local,
    plot_twr,
    plot_velocity_eci_x,
    plot_velocity_eci_y,
    plot_velocity_eci_z,
    plot_velocity_profile,
)
from .frames import quaternion_to_euler_zyx
from .recovery import great_circle_distance_m, target_landing_site_eci
from .utils import compute_ground_relative_velocity


def _extract_series(log, field: str) -> np.ndarray:

    """Return telemetry field as numpy array, or empty array if unavailable."""

    values = getattr(log, field, None)

    if values is None:

        return np.array([])

    return np.array(values)





def _compute_engine_on_mask_from_log(log) -> np.ndarray:

    """Infer engine-on samples from raw log telemetry."""

    t = _extract_series(log, "time")

    n = len(t)

    if n == 0:

        return np.array([], dtype=bool)



    thrust_on = _extract_series(log, "thrust_on")

    if len(thrust_on) == n:

        return thrust_on > 0.5



    thrust_x = _extract_series(log, "inertial_thrust_x")

    thrust_y = _extract_series(log, "inertial_thrust_y")

    thrust_z = _extract_series(log, "inertial_thrust_z")

    if len(thrust_x) == len(thrust_y) == len(thrust_z) == n:

        thrust_mag = np.sqrt(thrust_x**2 + thrust_y**2 + thrust_z**2)

        if np.nanmax(thrust_mag) > 0.0:

            threshold = max(1e3, 0.01 * float(np.nanmax(thrust_mag)))

            return thrust_mag > threshold



    return np.ones(n, dtype=bool)





def _find_stage1_meco_time(log, min_off_duration_s: float = 0.5) -> float | None:

    """Infer the first sustained stage-1 engine cutoff from the ascent log."""

    t = _extract_series(log, "time")

    n = len(t)

    if n == 0:

        return None

    if n == 1:

        return float(t[0])



    on_mask = _compute_engine_on_mask_from_log(log)

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

                return float(t[seg_start])

        i += 1



    edges = np.where(on_mask[:-1] & (~on_mask[1:]))[0]

    if len(edges) > 0:

        return float(t[int(edges[0] + 1)])



    return None





def _find_phase_time(log, phase: str) -> float | None:

    """Return first logged time for a named mission phase, if present."""

    t = _extract_series(log, "time")

    phases = list(getattr(log, "phase_name", []))

    if len(t) == 0 or len(phases) != len(t):

        return None



    for i, name in enumerate(phases):

        if name == phase:

            return float(t[i])

    return None





def _collect_mission_timeline_events(

    ascent_log,

    orbiter_log,

    booster_log,

    separation_time: float

) -> list[PlotEvent]:

    """Assemble the key mission events for combined altitude/velocity timelines."""

    events: list[PlotEvent] = []



    meco_time = _find_stage1_meco_time(ascent_log)

    if meco_time is not None:

        events.append(PlotEvent(meco_time, "MECO", '0.25', ':'))



    events.append(PlotEvent(float(separation_time), "Stage Separation", 'black', '--'))



    boostback_time = _find_phase_time(booster_log, "BOOSTER_BOOSTBACK")

    if boostback_time is not None:

        events.append(PlotEvent(boostback_time, "Boostback Burn", '#cc2222', '--'))



    orbit_insertion_time = _find_phase_time(orbiter_log, "ORBIT_INSERTION")

    if orbit_insertion_time is not None:

        events.append(PlotEvent(orbit_insertion_time, "Orbit Insertion", '#228833', '--'))



    orbit_achieved_time = _find_phase_time(orbiter_log, "ORBIT_ACHIEVED")

    if orbit_achieved_time is not None:

        events.append(PlotEvent(orbit_achieved_time, "Orbit Achieved", '#228833', ':'))



                                                                    

    events.extend(_collect_s2_recovery_events(orbiter_log))



    return sorted(events, key=lambda event: event.time)





def _collect_booster_timeline_events(booster_log, separation_time: float) -> list[PlotEvent]:

    """Assemble key booster events for post-separation timeline plots."""

    events: list[PlotEvent] = [PlotEvent(float(separation_time), "Stage Separation", 'black', '--')]



    phase_markers = [

        ("BOOSTER_FLIP", "Flip", '#7f7f7f', ':'),

        ("BOOSTER_BOOSTBACK", "Boostback Burn", '#cc2222', '--'),

        ("BOOSTER_ENTRY", "Entry Burn", '#ff7f0e', '--'),

        ("BOOSTER_LANDING", "Landing Burn", '#1f77b4', '--'),

    ]

    for phase_name, label, color, linestyle in phase_markers:

        phase_time = _find_phase_time(booster_log, phase_name)

        if phase_time is not None:

            events.append(PlotEvent(phase_time, label, color, linestyle))



    t_b = _extract_series(booster_log, "time")

    h_b = _extract_series(booster_log, "altitude")

    if len(t_b) > 0 and len(h_b) == len(t_b) and float(h_b[-1]) <= 0.1:

        events.append(PlotEvent(float(t_b[-1]), "Touchdown", '0.15', ':'))



    return sorted(events, key=lambda event: event.time)





def _mission_events_with_liftoff_maxq(

    ascent_log,

    orbiter_log,

    booster_log,

    separation_time: float,

) -> list[PlotEvent]:

    """Full-mission event set used by reviewer-response attitude/load plots."""

    events = [PlotEvent(0.0, "Liftoff", 'black', '-')]

    q = _extract_series(ascent_log, "dynamic_pressure")

    t = _extract_series(ascent_log, "time")

    if len(t) > 0 and len(q) == len(t):

        events.append(PlotEvent(float(t[int(np.argmax(q))]), "Max-Q", '#9467bd', ':'))

    events.extend(_collect_mission_timeline_events(ascent_log, orbiter_log, booster_log, separation_time))

    events.extend(_collect_booster_timeline_events(booster_log, separation_time))

    events.append(PlotEvent(float(separation_time), "Stage 2 Ignition", '#228833', ':'))

    events.append(PlotEvent(float(separation_time), "Fairing Sep. Not Modeled", '0.45', ':'))

    return sorted(events, key=lambda event: event.time)





def _plot_log_branch(ax: Axes, log, field: str, label: str, color: str, *, scale: float = 1.0,

                     linestyle: str = '-') -> None:

    t = _extract_series(log, "time")

    y = _extract_series(log, field)

    if len(t) > 0 and len(y) == len(t):

        ax.plot(t, y * scale, color=color, linewidth=1.8, linestyle=linestyle, label=label)





def _quat_matrix(log, prefix: str) -> np.ndarray:

    parts = [

        _extract_series(log, f"{prefix}_quat_w"),

        _extract_series(log, f"{prefix}_quat_x"),

        _extract_series(log, f"{prefix}_quat_y"),

        _extract_series(log, f"{prefix}_quat_z"),

    ]

    if not parts or any(len(part) == 0 for part in parts):

        return np.empty((0, 4))

    n = min(len(part) for part in parts)

    return np.column_stack([part[:n] for part in parts])





def _euler_from_log(log, prefix: str) -> tuple[np.ndarray, np.ndarray]:

    t = _extract_series(log, "time")

    q = _quat_matrix(log, prefix)

    n = min(len(t), len(q))

    if n == 0:

        return np.array([]), np.empty((0, 3))

    angles = np.array([quaternion_to_euler_zyx(row, degrees=True) for row in q[:n]])

    return t[:n], angles





def _booster_touchdown_speed(booster_log) -> float:

    """Compute surface-relative touchdown speed from the final logged state vectors.



    The logged 'velocity_rel' field is populated from the guidance dict's 'v_rel'

    key, which is computed from the *pre-integration* state (one RK4 step earlier

    than the logged position/velocity).  This means velocity_rel[-1] is one

    timestep stale at the moment of landing detection.



    This function recomputes v_rel from the *post-integration* position_x/y/z and

    velocity_x/y/z arrays â€” the same vectors that check_termination uses â€” giving

    a result consistent with the terminal's "Touchdown at X m/s" line.

    """


    px = getattr(booster_log, 'position_x', None)

    py = getattr(booster_log, 'position_y', None)

    pz = getattr(booster_log, 'position_z', None)

    vx = getattr(booster_log, 'velocity_x', None)

    vy = getattr(booster_log, 'velocity_y', None)

    vz = getattr(booster_log, 'velocity_z', None)

    if px and vx and len(px) > 0:

        pos = np.array([float(px[-1]), float(py[-1]), float(pz[-1])])

        vel = np.array([float(vx[-1]), float(vy[-1]), float(vz[-1])])

        return float(np.linalg.norm(compute_ground_relative_velocity(pos, vel)))

                                                                      

    vr = getattr(booster_log, 'velocity_rel', None)

    return float(vr[-1]) if vr and len(vr) > 0 else 0.0





def plot_mission_altitude_split(ascent_log, orbiter_log, booster_log,
                                separation_time: float, output_dir: str) -> str:
    """Altitude timeline for stacked ascent, orbiter, and booster segments."""
    return plot_multi_vehicle_series(
        output_dir,
        MultiVehicleSeriesPlot(
            filename="59_mission_altitude_split.png",
            title="Mission Altitude Timeline: Ascent, Orbiter, and Booster",
            ylabel="Altitude (km)",
        ),
        (
            VehicleSeries(
                _extract_series(ascent_log, "time"),
                _extract_series(ascent_log, "altitude"),
                "Stacked Ascent (S1+S2)",
                "tab:blue",
            ),
            VehicleSeries(
                _extract_series(orbiter_log, "time"),
                _extract_series(orbiter_log, "altitude"),
                "Orbiter (S2)",
                "tab:green",
            ),
            VehicleSeries(
                _extract_series(booster_log, "time"),
                _extract_series(booster_log, "altitude"),
                "Booster (S1)",
                "tab:red",
            ),
        ),
        events=_collect_mission_timeline_events(
            ascent_log, orbiter_log, booster_log, separation_time
        ),
    )





def plot_mission_velocity_split(ascent_log, orbiter_log, booster_log,
                                separation_time: float, output_dir: str) -> str:
    """Velocity timeline for stacked ascent, orbiter, and booster segments."""
    return plot_multi_vehicle_series(
        output_dir,
        MultiVehicleSeriesPlot(
            filename="60_mission_velocity_split.png",
            title="Mission Velocity Timeline: Ascent, Orbiter, and Booster",
            ylabel="Velocity (m/s)",
        ),
        (
            VehicleSeries(
                _extract_series(ascent_log, "time"),
                _extract_series(ascent_log, "velocity"),
                "Stacked Ascent (S1+S2)",
                "tab:blue",
            ),
            VehicleSeries(
                _extract_series(orbiter_log, "time"),
                _extract_series(orbiter_log, "velocity"),
                "Orbiter (S2)",
                "tab:green",
            ),
            VehicleSeries(
                _extract_series(booster_log, "time"),
                _extract_series(booster_log, "velocity"),
                "Booster (S1)",
                "tab:red",
            ),
        ),
        events=_collect_mission_timeline_events(
            ascent_log, orbiter_log, booster_log, separation_time
        ),
    )





def plot_booster_altitude_profile(booster_log, separation_time: float, output_dir: str) -> str:

    """Booster altitude from separation to touchdown/termination."""

    fig, ax = plt.subplots(figsize=(10, 6))



    t_b = _extract_series(booster_log, "time")

    h_b = _extract_series(booster_log, "altitude")

    v_b_rel = _extract_series(booster_log, "velocity_rel")                          



    if len(t_b) == 0:

        ax.text(0.5, 0.5, 'No booster telemetry available', transform=ax.transAxes,

                ha='center', va='center', fontsize=12)

    else:

        ax.plot(t_b, h_b, color='tab:red', linewidth=2.0, label='Booster altitude')

        peak_idx = int(np.argmax(h_b))

        ax.scatter([t_b[peak_idx]], [h_b[peak_idx]], color='darkred', s=45,

                   label=f'Apogee: {h_b[peak_idx]:.1f} km')

        ax.scatter([t_b[-1]], [h_b[-1]], color='black', s=45,

                   label=f'Final: {h_b[-1]:.1f} km')

        if len(v_b_rel) > 0 and h_b[-1] <= 0.1:

            td_speed = _booster_touchdown_speed(booster_log)

            ax.annotate(f'Touchdown speed: {td_speed:.1f} m/s',

                        xy=(t_b[-1], h_b[-1]), xytext=(-160, 25),

                        textcoords='offset points',

                        arrowprops=dict(arrowstyle='->', alpha=0.7))



    _add_timeline_event_markers(ax, _collect_booster_timeline_events(booster_log, separation_time))

    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Altitude (km)')

    ax.set_title('Booster Altitude Profile (Post-Separation)', fontweight='bold')

    ax.legend(loc='best', framealpha=0.95)

    ax.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '61_booster_altitude_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_booster_velocity_profile(booster_log, separation_time: float, output_dir: str) -> str:

    """Booster speed and vertical velocity versus time.



    Both inertial speed and surface-relative speed are shown.  The inertial

    speed at touchdown (~465 m/s at equator) reflects Earth rotation, NOT a

    crash â€” the surface-relative touchdown speed is ~3-5 m/s.

    """

    fig, ax = plt.subplots(figsize=(10, 6))



    t_b   = _extract_series(booster_log, "time")

    v_b   = _extract_series(booster_log, "velocity")

    v_rel = _extract_series(booster_log, "velocity_rel")

    v_vert = _extract_series(booster_log, "velocity_vertical")



    if len(t_b) == 0:

        ax.text(0.5, 0.5, 'No booster telemetry available', transform=ax.transAxes,

                ha='center', va='center', fontsize=12)

    else:

        ax.plot(t_b, v_b, color='tab:orange', linewidth=1.9,

                label='Inertial speed (incl. Earth rotation)')

        if len(v_rel) == len(t_b):

            ax.plot(t_b, v_rel, color='tab:red', linewidth=1.9, linestyle='--',

                    label='Surface-relative speed (airspeed)')

        if len(v_vert) == len(t_b):

            ax.plot(t_b, v_vert, color='tab:purple', linewidth=1.7, alpha=0.9,

                    label='Vertical velocity')

                                                                                           

        if len(v_rel) == len(t_b) and len(t_b) > 0:

            td_speed = _booster_touchdown_speed(booster_log)

            ax.annotate(f'Touchdown: {td_speed:.1f} m/s (surface-rel.)',

                        xy=(t_b[-1], v_rel[-1]),

                        xytext=(-120, 30), textcoords='offset points',

                        arrowprops=dict(arrowstyle='->', lw=1.1), fontsize=9)



    ax.axhline(0.0, color='gray', linestyle=':', alpha=0.5)

    _add_timeline_event_markers(ax, _collect_booster_timeline_events(booster_log, separation_time))

    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Velocity (m/s)')

    ax.set_title('Booster Velocity Profile (Post-Separation)', fontweight='bold')

    ax.legend(loc='best', framealpha=0.95)

    ax.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '62_booster_velocity_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_booster_landing_zoom(booster_log, output_dir: str, window_s: float = 140.0) -> str:

    """Zoom into the final landing segment of booster descent.



    Speed axis shows SURFACE-RELATIVE speed (not inertial), so the touchdown

    value correctly reads ~3-5 m/s rather than ~465 m/s (Earth rotation offset).

    """

    fig, ax1 = plt.subplots(figsize=(10, 6))



    t_b   = _extract_series(booster_log, "time")

    h_b   = _extract_series(booster_log, "altitude")

    v_rel = _extract_series(booster_log, "velocity_rel")                    



    if len(t_b) == 0:

        ax1.text(0.5, 0.5, 'No booster telemetry available', transform=ax1.transAxes,

                 ha='center', va='center', fontsize=12)

    else:

        t_end = t_b[-1]

        t_min = max(t_b[0], t_end - window_s)

        mask = t_b >= t_min

        tz = t_b[mask]

        hz = h_b[mask]

        vz_rel = v_rel[mask] if len(v_rel) == len(t_b) else np.array([])



        ax1.plot(tz, hz, color='tab:red', linewidth=2.0, label='Altitude')

        ax1.set_ylabel('Altitude (km)', color='tab:red')

        ax1.tick_params(axis='y', labelcolor='tab:red')



        ax2 = ax1.twinx()

        if len(vz_rel) == len(tz):

            ax2.plot(tz, vz_rel, color='tab:blue', linewidth=1.8,

                     label='Surface-relative speed')

        ax2.set_ylabel('Surface-Relative Speed (m/s)', color='tab:blue')

        ax2.tick_params(axis='y', labelcolor='tab:blue')



        lines = ax1.get_lines() + ax2.get_lines()

        labels = [l.get_label() for l in lines]

        ax1.legend(lines, labels, loc='upper right', framealpha=0.95)



        ax1.scatter([tz[-1]], [hz[-1]], color='black', s=55, zorder=5, marker='*')

        td_spd = _booster_touchdown_speed(booster_log)

        ax1.annotate(f'Touchdown\n{td_spd:.1f} m/s (surface-rel.)',

                     xy=(tz[-1], hz[-1]), xytext=(-110, 30),

                     textcoords='offset points',

                     arrowprops=dict(arrowstyle='->', alpha=0.8), fontsize=9)



    ax1.set_xlabel('Time (s)')

    ax1.set_title('Booster Landing Segment â€” Surface-Relative Speed (Zoomed)',

                  fontweight='bold')

    ax1.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '63_booster_landing_zoom.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_booster_velocity_components(booster_log, output_dir: str) -> str:

    """Booster radial and inertial-horizontal velocity history."""

    fig, ax = plt.subplots(figsize=(10, 6))

    t = _extract_series(booster_log, "time")

    v_rad = _extract_series(booster_log, "radial_velocity")

    v_horiz = _extract_series(booster_log, "horizontal_velocity_inertial")



    if len(t) == 0:

        ax.text(0.5, 0.5, 'No booster telemetry available', transform=ax.transAxes,

                ha='center', va='center', fontsize=12)

    else:

        if len(v_rad) == len(t):

            ax.plot(t, v_rad, color='tab:red', linewidth=1.9, label='Radial velocity')

        if len(v_horiz) == len(t):

            ax.plot(t, v_horiz, color='tab:blue', linewidth=1.9, label='Horizontal velocity (inertial)')

        ax.axhline(0.0, color='gray', linestyle=':', alpha=0.5)

        ax.legend(loc='best', framealpha=0.95)



    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Velocity (m/s)')

    ax.set_title('Booster Velocity Components', fontweight='bold')

    ax.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '64_booster_velocity_components.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_booster_ignition_prediction(booster_log, output_dir: str) -> str:

    """Actual altitude vs predicted ignition altitude â€” zoomed to descent phase.



    The predicted ignition altitude is the energy-based estimate of how much

    altitude is needed to decelerate to zero from the current speed.  Values

    range from ~1â€“12 km, so this plot clips to the post-apogee descent where

    the comparison is meaningful (apogee to touchdown).

    """

    fig, ax = plt.subplots(figsize=(10, 6))

    t = _extract_series(booster_log, "time")

    alt_km = _extract_series(booster_log, "altitude")

    ignite_pred_m = _extract_series(booster_log, "ignition_altitude_prediction_m")



    if len(t) == 0:

        ax.text(0.5, 0.5, 'No booster telemetry available', transform=ax.transAxes,

                ha='center', va='center', fontsize=12)

    else:

                                                          

        apogee_idx = int(np.argmax(alt_km))

        t_desc = t[apogee_idx:]

        alt_desc = alt_km[apogee_idx:]



        ax.plot(t_desc, alt_desc, color='tab:red', linewidth=1.9, label='Actual altitude')



        if len(ignite_pred_m) == len(t):

            pred_desc = ignite_pred_m[apogee_idx:] / 1000.0            

            ax.plot(t_desc, pred_desc, color='tab:green', linewidth=1.8,

                    linestyle='--', label='Predicted ignition altitude')



                                                                                

            trigger = np.where(alt_desc <= pred_desc * 1.5)[0]

            if len(trigger) > 0:

                ti = trigger[0]

                ax.axvline(t_desc[ti], color='gray', linestyle=':', alpha=0.7,

                           label=f'~Ignition trigger (t={t_desc[ti]:.0f}s)')



        ax.legend(loc='upper right', framealpha=0.95)

        ax.set_title('Booster Landing Ignition Prediction (Descent Phase)', fontweight='bold')



    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Altitude (km)')

    ax.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '65_booster_ignition_prediction.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_booster_phase_fuel_budget(booster_log, output_dir: str) -> str:

    """Propellant consumed in each booster phase."""

    fig, ax = plt.subplots(figsize=(10, 6))

    t = _extract_series(booster_log, "time")

    m = _extract_series(booster_log, "mass")

    phases = getattr(booster_log, "phase_name", [])



    usage = {}

    if len(t) > 1 and len(m) == len(t) and len(phases) == len(t):

        start = 0

        for i in range(1, len(t)):

            if phases[i] != phases[start]:

                phase = phases[start]

                usage[phase] = usage.get(phase, 0.0) + max(0.0, m[start] - m[i - 1])

                start = i

        phase = phases[start]

        usage[phase] = usage.get(phase, 0.0) + max(0.0, m[start] - m[-1])



    if len(usage) == 0:

        ax.text(0.5, 0.5, 'No phase fuel usage data available', transform=ax.transAxes,

                ha='center', va='center', fontsize=12)

    else:

        labels = list(usage.keys())

        values = [usage[k] for k in labels]

        bars = ax.bar(labels, values, color='#4c78a8', alpha=0.9)

        for b, v in zip(bars, values):

            ax.text(b.get_x() + b.get_width() / 2.0, b.get_height(), f'{v:.0f}',

                    ha='center', va='bottom', fontsize=9)



    ax.set_ylabel('Propellant Used (kg)')

    ax.set_title('Booster Phase Fuel Budget', fontweight='bold')

    ax.grid(True, axis='y', alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '66_booster_phase_fuel_budget.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_rtls_trajectory_profile(ascent_log, orbiter_log, booster_log,

                                  separation_time: float, output_dir: str) -> str:

    """RTLS Trajectory Profile â€” Altitude vs Downrange Distance.



    Replicates the classic RTLS altitude-vs-downrange diagram showing the

    characteristic hairpin loop of the booster returning to the launch site.

    """

    fig, ax = plt.subplots(figsize=(12, 7))



    dr_a = _extract_series(ascent_log,  "downrange_ground")                        

    h_a  = _extract_series(ascent_log,  "altitude")

                                                                           

                                                                               

    dr_b = _extract_series(booster_log, "downrange_ground")

    h_b  = _extract_series(booster_log, "altitude")

    ph_b = list(getattr(booster_log, "phase_name", []))



                                                          

    if len(dr_a) == 0:

        dr_a = _extract_series(ascent_log, "downrange")

    if len(dr_b) == 0:

        dr_b = _extract_series(booster_log, "downrange")



                                                                 

    if len(dr_a) > 0 and len(dr_b) > 0:

        dr_full = np.concatenate([dr_a, dr_b])

        h_full  = np.concatenate([h_a,  h_b])

        ax.plot(dr_full, h_full, color='#CC2222', linewidth=2.2, zorder=3)

    elif len(dr_a) > 0:

        ax.plot(dr_a, h_a, color='#CC2222', linewidth=2.2, zorder=3)



    def _first_phase_idx(target):

        for i, p in enumerate(ph_b):

            if p == target:

                return i

        return None



    entry_idx   = _first_phase_idx("BOOSTER_ENTRY")

    landing_idx = _first_phase_idx("BOOSTER_LANDING")



                   

    if len(dr_a) > 0:

        ax.scatter([dr_a[0]], [h_a[0]], color='black', s=70, marker='^', zorder=6)

        ax.annotate("Launch",

                    xy=(dr_a[0], h_a[0]), xytext=(8, 8),

                    textcoords='offset points', fontsize=9)



                                           

    if len(dr_b) > 0:

        ax.scatter([dr_b[0]], [h_b[0]], color='#CC2222', s=70, zorder=6)

        ax.annotate("MECO / Stage Sep\n& Boostback Burn",

                    xy=(dr_b[0], h_b[0]), xytext=(8, -35),

                    textcoords='offset points', fontsize=9,

                    arrowprops=dict(arrowstyle='->', color='black', lw=1.1))



                      

        peak_idx = int(np.argmax(h_b))

        ax.scatter([dr_b[peak_idx]], [h_b[peak_idx]], color='#CC2222', s=70, zorder=6)

        ax.annotate("Boostback Burn\n(Hairpin Turn)",

                    xy=(dr_b[peak_idx], h_b[peak_idx]), xytext=(12, 4),

                    textcoords='offset points', fontsize=9,

                    arrowprops=dict(arrowstyle='->', color='black', lw=1.1))



                

    if entry_idx is not None:

        ax.scatter([dr_b[entry_idx]], [h_b[entry_idx]], color='#CC2222', s=60, zorder=6)

        ax.annotate("Entry Burn",

                    xy=(dr_b[entry_idx], h_b[entry_idx]), xytext=(-85, 8),

                    textcoords='offset points', fontsize=9,

                    arrowprops=dict(arrowstyle='->', color='black', lw=1.1))



                                     

    if landing_idx is not None:

        ax.scatter([dr_b[landing_idx]], [h_b[landing_idx]], color='#CC2222', s=60, zorder=6)

        ax.annotate("Terminal\nDescent",

                    xy=(dr_b[landing_idx], h_b[landing_idx]), xytext=(-95, 18),

                    textcoords='offset points', fontsize=9,

                    arrowprops=dict(arrowstyle='->', color='black', lw=1.1))



               

    if len(dr_b) > 0:

        ax.scatter([dr_b[-1]], [max(h_b[-1], 0.0)], color='black', s=90,

                   marker='*', zorder=7)

        ax.annotate("RTLS\nLanding",

                    xy=(dr_b[-1], max(h_b[-1], 0.0)), xytext=(-65, 20),

                    textcoords='offset points', fontsize=9,

                    arrowprops=dict(arrowstyle='->', color='black', lw=1.1))



                        

    if len(dr_a) > 1:

        mid = len(dr_a) // 2

        ax.text(dr_a[mid] + 4, h_a[mid], "Ascent Phase",

                fontsize=10, style='italic', color='#444444', ha='left')



    if len(dr_b) > 0:

        peak_idx = int(np.argmax(h_b))

        if peak_idx < len(dr_b) - 10:

            mid_r = (peak_idx + len(dr_b)) // 2

            ax.text(dr_b[mid_r] + 3, h_b[mid_r] + 2, "Return Flight",

                    fontsize=10, style='italic', color='#444444', ha='left')



    ax.set_xlabel("Downrange Distance from Launch Site (km)", fontsize=12)

    ax.set_ylabel("Altitude (km)", fontsize=12)

    ax.set_title("Return to Launch Site (RTLS) Trajectory Profile (Altitude vs. Downrange)",

                 fontweight='bold', fontsize=13)

    ax.set_ylim(bottom=-2)

    ax.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '67_rtls_trajectory_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_full_mission_attitude_error(ascent_log, orbiter_log, booster_log,

                                     separation_time: float, output_dir: str) -> str:

    """Full-mission attitude tracking error with mission phase markers."""

    fig, ax = plt.subplots(figsize=(12, 6))

    _plot_log_branch(ax, ascent_log, "attitude_error", "Stacked ascent", 'tab:blue')

    _plot_log_branch(ax, orbiter_log, "attitude_error", "Stage 2 / orbiter", 'tab:green')

    _plot_log_branch(ax, booster_log, "attitude_error", "Stage 1 / booster", 'tab:red')

    _add_timeline_event_markers(

        ax,

        _mission_events_with_liftoff_maxq(ascent_log, orbiter_log, booster_log, separation_time),

    )

    ax.set_xlabel("Time (s)")

    ax.set_ylabel("Attitude Error (deg)")

    ax.set_title("Full-Mission 6DoF Attitude Tracking Error", fontweight='bold')

    ax.legend(loc='best', framealpha=0.95)

    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    path = os.path.join(output_dir, '68_full_mission_attitude_error.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_full_mission_angular_rates(ascent_log, orbiter_log, booster_log,

                                    separation_time: float, output_dir: str) -> str:

    """Body angular-rate evolution for ascent, orbiter, and RTLS booster."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    components = [

        ("omega_x", "omega_x (rad/s)"),

        ("omega_y", "omega_y (rad/s)"),

        ("omega_z", "omega_z (rad/s)"),

    ]

    for ax, (field, ylabel) in zip(axes, components):

        _plot_log_branch(ax, ascent_log, field, "Stacked ascent", 'tab:blue')

        _plot_log_branch(ax, orbiter_log, field, "Stage 2 / orbiter", 'tab:green')

        _plot_log_branch(ax, booster_log, field, "Stage 1 / booster", 'tab:red')

        _add_timeline_event_markers(

            ax,

            _mission_events_with_liftoff_maxq(ascent_log, orbiter_log, booster_log, separation_time),

        )

        ax.set_ylabel(ylabel)

        ax.grid(True, alpha=0.3)

    axes[0].set_title("Full-Mission Body Angular Rate Evolution", fontweight='bold')

    axes[0].legend(loc='best', framealpha=0.95)

    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()

    path = os.path.join(output_dir, '69_full_mission_body_rates.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_full_mission_euler_angles(ascent_log, orbiter_log, booster_log,

                                   separation_time: float, output_dir: str) -> str:

    """Euler-angle view derived from logged quaternions for presentation only."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    labels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]

    branches = [

        (ascent_log, "Stacked ascent", 'tab:blue'),

        (orbiter_log, "Stage 2 / orbiter", 'tab:green'),

        (booster_log, "Stage 1 / booster", 'tab:red'),

    ]

    for log, branch_label, color in branches:

        t, angles = _euler_from_log(log, "actual")

        if len(t) == 0:

            continue

        for idx, ax in enumerate(axes):

            ax.plot(t, angles[:, idx], color=color, linewidth=1.7, label=branch_label)

    for ax, ylabel in zip(axes, labels):

        _add_timeline_event_markers(

            ax,

            _mission_events_with_liftoff_maxq(ascent_log, orbiter_log, booster_log, separation_time),

        )

        ax.set_ylabel(ylabel)

        ax.grid(True, alpha=0.3)

    axes[0].set_title("Full-Mission Euler Angles Derived From Quaternion State", fontweight='bold')

    axes[0].legend(loc='best', framealpha=0.95)

    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()

    path = os.path.join(output_dir, '70_full_mission_euler_angles.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_full_mission_quaternion_tracking(ascent_log, orbiter_log, booster_log,

                                          separation_time: float, output_dir: str) -> str:

    """Commanded vs actual quaternion components for all mission branches."""

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    suffixes = ["w", "x", "y", "z"]

    branches = [

        (ascent_log, "Ascent", 'tab:blue'),

        (orbiter_log, "S2", 'tab:green'),

        (booster_log, "Booster", 'tab:red'),

    ]

    for log, label, color in branches:

        t = _extract_series(log, "time")

        for ax, suffix in zip(axes, suffixes):

            actual = _extract_series(log, f"actual_quat_{suffix}")

            cmd = _extract_series(log, f"commanded_quat_{suffix}")

            if len(t) > 0 and len(actual) == len(t):

                ax.plot(t, actual, color=color, linewidth=1.5, label=f"{label} actual")

            if len(t) > 0 and len(cmd) == len(t):

                ax.plot(t, cmd, color=color, linewidth=1.2, linestyle='--', alpha=0.75,

                        label=f"{label} cmd")

    for ax, suffix in zip(axes, suffixes):

        ax.set_ylabel(f"q_{suffix}")

        ax.grid(True, alpha=0.3)

    axes[0].set_title("Full-Mission Commanded vs Actual Quaternion Components", fontweight='bold')

    axes[0].legend(loc='best', ncol=3, fontsize=8, framealpha=0.95)

    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()

    path = os.path.join(output_dir, '71_full_mission_quaternion_tracking.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_full_mission_control_response(ascent_log, orbiter_log, booster_log,

                                       separation_time: float, output_dir: str) -> str:

    """Control response summary: attitude error, torque, rates, throttle."""

    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

    branches = [

        (ascent_log, "Ascent", 'tab:blue'),

        (orbiter_log, "S2", 'tab:green'),

        (booster_log, "Booster", 'tab:red'),

    ]

    for log, label, color in branches:

        _plot_log_branch(axes[0], log, "attitude_error", label, color)

        _plot_log_branch(axes[1], log, "torque_magnitude", label, color, scale=1.0e-6)

        t = _extract_series(log, "time")

        wx = _extract_series(log, "omega_x")

        wy = _extract_series(log, "omega_y")

        wz = _extract_series(log, "omega_z")

        if len(t) > 0 and len(wx) == len(wy) == len(wz) == len(t):

            axes[2].plot(t, np.sqrt(wx**2 + wy**2 + wz**2), color=color, linewidth=1.6, label=label)

        _plot_log_branch(axes[3], log, "throttle", label, color)

    axes[0].set_ylabel("Attitude error (deg)")

    axes[1].set_ylabel("Torque (MN m)")

    axes[2].set_ylabel("|omega| (rad/s)")

    axes[3].set_ylabel("Throttle")

    axes[3].set_xlabel("Time (s)")

    for ax in axes:

        _add_timeline_event_markers(

            ax,

            _mission_events_with_liftoff_maxq(ascent_log, orbiter_log, booster_log, separation_time),

        )

        ax.grid(True, alpha=0.3)

    axes[0].set_title("Control System Evolution Across Full Mission", fontweight='bold')

    axes[0].legend(loc='best', framealpha=0.95)

    plt.tight_layout()

    path = os.path.join(output_dir, '72_full_mission_control_response.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_full_mission_torque_components(ascent_log, orbiter_log, booster_log,

                                        separation_time: float, output_dir: str) -> str:

    """Control torque components in the body frame for the full mission."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    components = [

        ("torque_x", "tau_x (MN m)"),

        ("torque_y", "tau_y (MN m)"),

        ("torque_z", "tau_z (MN m)"),

    ]

    branches = [

        (ascent_log, "Ascent", 'tab:blue'),

        (orbiter_log, "S2", 'tab:green'),

        (booster_log, "Booster", 'tab:red'),

    ]

    for ax, (field, ylabel) in zip(axes, components):

        for log, label, color in branches:

            _plot_log_branch(ax, log, field, label, color, scale=1.0e-6)

        _add_timeline_event_markers(

            ax,

            _mission_events_with_liftoff_maxq(ascent_log, orbiter_log, booster_log, separation_time),

        )

        ax.set_ylabel(ylabel)

        ax.grid(True, alpha=0.3)

    axes[0].set_title("Full-Mission Body-Frame Control Torque Components", fontweight='bold')

    axes[0].legend(loc='best', framealpha=0.95)

    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()

    path = os.path.join(output_dir, '75_full_mission_torque_components.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_gravity_turn_pitch_fpa_tracking(ascent_log, separation_time: float, output_dir: str) -> str:

    """Commanded vs actual pitch and flight-path angle through the gravity turn."""

    t = _extract_series(ascent_log, "time")

    if len(t) == 0:

        raise ValueError("ascent log has no time samples")



    pitch_cmd = _extract_series(ascent_log, "pitch_angle")

    pitch_actual = _extract_series(ascent_log, "actual_pitch_angle")

    gamma_cmd = _extract_series(ascent_log, "gamma_command_deg")

    gamma_actual = _extract_series(ascent_log, "gamma_actual_deg")

    q = _extract_series(ascent_log, "dynamic_pressure")

    h = _extract_series(ascent_log, "altitude")



    n = min(len(t), len(pitch_cmd), len(pitch_actual), len(gamma_cmd), len(gamma_actual))

    if n == 0:

        raise ValueError("ascent log has no pitch/gamma tracking samples")



    t = t[:n]

    pitch_cmd = pitch_cmd[:n]

    pitch_actual = pitch_actual[:n]

    gamma_cmd = gamma_cmd[:n]

    gamma_actual = gamma_actual[:n]

    pitch_from_gamma_cmd = 90.0 - gamma_cmd

    velocity_tilt_actual = 90.0 - gamma_actual

    pitch_error = pitch_actual - pitch_cmd

    gamma_error = gamma_actual - gamma_cmd



    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    fig.suptitle("Gravity Turn Pitch and Flight-Path-Angle Tracking", fontweight="bold", fontsize=14)



    axes[0].plot(t, pitch_cmd, color="#005f73", linewidth=2.2, label="Commanded pitch from vertical")

    axes[0].plot(t, pitch_actual, color="#ca6702", linewidth=2.0, linestyle="--", label="Actual body pitch from vertical")

    axes[0].plot(t, pitch_from_gamma_cmd, color="#6a4c93", linewidth=1.7, linestyle=":", label="90 deg - commanded gamma")

    axes[0].set_ylabel("Pitch / Tilt (deg)")

    axes[0].set_ylim(-5, 100)

    axes[0].legend(loc="best", fontsize=9)



    axes[1].plot(t, gamma_cmd, color="#0a9396", linewidth=2.2, label="Commanded flight-path angle gamma")

    axes[1].plot(t, gamma_actual, color="#bb3e03", linewidth=2.0, linestyle="--", label="Actual flight-path angle gamma")

    axes[1].plot(t, velocity_tilt_actual, color="#7f7f7f", linewidth=1.5, linestyle=":", label="Actual velocity tilt from vertical")

    axes[1].axhline(90.0, color="0.6", linestyle=":", linewidth=1.0)

    axes[1].axhline(0.0, color="0.6", linestyle=":", linewidth=1.0)

    axes[1].set_ylabel("Gamma (deg from horizontal)")

    axes[1].set_ylim(-15, 100)

    axes[1].legend(loc="best", fontsize=9)



    axes[2].plot(t, pitch_error, color="#ae2012", linewidth=2.0, label="Pitch tracking error")

    axes[2].plot(t, gamma_error, color="#1d3557", linewidth=2.0, linestyle="--", label="Gamma tracking error")

    axes[2].axhline(0.0, color="0.35", linestyle=":", linewidth=1.0)

    axes[2].set_ylabel("Error (deg)")

    axes[2].set_xlabel("Mission Time (s)")

    axes[2].legend(loc="best", fontsize=9)



    events = [PlotEvent(0.0, "Liftoff", "black", "-")]

    if len(q) == len(_extract_series(ascent_log, "time")) and len(q) > 0:

        t_all = _extract_series(ascent_log, "time")

        events.append(PlotEvent(float(t_all[int(np.argmax(q))]), "Max-Q", "#9467bd", ":"))

    meco_time = _find_stage1_meco_time(ascent_log)

    if meco_time is not None:

        events.append(PlotEvent(meco_time, "MECO", "0.25", ":"))

    events.append(PlotEvent(float(separation_time), "Stage Separation", "black", "--"))



    for ax in axes:

        _add_timeline_event_markers(ax, events)

        ax.grid(True, alpha=0.25)

        ax.set_xlim(0.0, max(float(separation_time), float(t[-1])))



    if len(h) == len(_extract_series(ascent_log, "time")):

        gravity_turn_idx = np.where(h > 1000.0)[0]

        if len(gravity_turn_idx) > 0:

            gt_time = float(_extract_series(ascent_log, "time")[int(gravity_turn_idx[0])])

            axes[0].axvspan(gt_time, float(separation_time), color="#e9d8a6", alpha=0.18, label="Gravity-turn region")



    fig.tight_layout()

    path = os.path.join(output_dir, '76_gravity_turn_pitch_fpa_tracking.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_full_mission_aero_loads(ascent_log, orbiter_log, booster_log,

                                 separation_time: float, output_dir: str) -> str:

    """Max-Q and re-entry load evolution for ascent and booster return."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)

    events = _mission_events_with_liftoff_maxq(ascent_log, orbiter_log, booster_log, separation_time)

    for log, label, color in (

        (ascent_log, "Ascent", 'tab:blue'),

        (booster_log, "Booster RTLS", 'tab:red'),

    ):

        t = _extract_series(log, "time")

        q = _extract_series(log, "dynamic_pressure")

        alt = _extract_series(log, "altitude")

        aoa = _extract_series(log, "angle_of_attack_deg")

        q_alpha = _extract_series(log, "q_alpha")

        if len(t) > 0 and len(q) == len(t):

            axes[0].plot(t, q / 1000.0, color=color, linewidth=1.8, label=label)

            idx = int(np.argmax(q))

            axes[0].scatter([t[idx]], [q[idx] / 1000.0], color=color, s=45)

            axes[0].annotate(f"{label} peak q\n{q[idx]/1000.0:.1f} kPa",

                             xy=(t[idx], q[idx] / 1000.0), xytext=(8, 12),

                             textcoords='offset points', fontsize=8,

                             arrowprops=dict(arrowstyle='->', lw=0.8))

        if len(alt) == len(q) and len(q) > 0:

            axes[1].plot(alt, q / 1000.0, color=color, linewidth=1.8, label=label)

        if len(aoa) == len(q) and len(q) > 0:

            axes[2].plot(q / 1000.0, aoa, color=color, linewidth=1.8, label=label)

        if len(t) > 0 and len(q_alpha) == len(t):

            ax2 = axes[0].twinx()

            ax2.plot(t, q_alpha / 1000.0, color=color, linestyle=':', alpha=0.35)

            ax2.set_ylabel("Q-alpha (kPa rad)")

    _add_timeline_event_markers(axes[0], events)

    axes[0].set_ylabel("Dynamic Pressure (kPa)")

    axes[1].set_xlabel("Altitude (km)")

    axes[1].set_ylabel("Dynamic Pressure (kPa)")

    axes[2].set_xlabel("Dynamic Pressure (kPa)")

    axes[2].set_ylabel("Angle of Attack (deg)")

    axes[0].set_title("Max-Q and Booster Re-entry Aerodynamic Load Evolution", fontweight='bold')

    for ax in axes:

        ax.grid(True, alpha=0.3)

        ax.legend(loc='best', framealpha=0.95)

    plt.tight_layout()

    path = os.path.join(output_dir, '73_full_mission_aero_loads.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_rtls_landing_error(booster_log, separation_time: float, output_dir: str,

                            config=None) -> str:

    """Precise RTLS landing error in metres, not rounded kilometre wording."""

    from .config_factory import create_default_config




    cfg = config or create_default_config()

    fig, ax = plt.subplots(figsize=(10, 6))

    t = _extract_series(booster_log, "time")

    dr = _extract_series(booster_log, "downrange_ground")

    h = _extract_series(booster_log, "altitude")

    if len(t) == 0:

        ax.text(0.5, 0.5, "No booster telemetry available", transform=ax.transAxes,

                ha='center', va='center')

    else:

        ax.plot(dr, h, color='tab:red', linewidth=2.0, label="Booster ground track")

        target_site = target_landing_site_eci(float(t[-1]), cfg.booster_landing_target_downrange_km, config=cfg)

        final_pos = np.array([

            booster_log.position_x[-1],

            booster_log.position_y[-1],

            booster_log.position_z[-1],

        ], dtype=float)

        final_vel = np.array([

            booster_log.velocity_x[-1],

            booster_log.velocity_y[-1],

            booster_log.velocity_z[-1],

        ], dtype=float)

        site_error_m = great_circle_distance_m(final_pos, target_site)

        vertical = final_pos / max(np.linalg.norm(final_pos), 1.0)

        from .utils import compute_ground_relative_velocity
        v_rel = compute_ground_relative_velocity(final_pos, final_vel)

        v_vertical = float(np.dot(v_rel, vertical))

        v_horizontal = float(np.linalg.norm(v_rel - v_vertical * vertical))

        ax.scatter([dr[-1]], [h[-1]], color='black', s=80, marker='*', label="Touchdown")

        ax.annotate(

            "\n".join([

                f"Landing error: {site_error_m:.1f} m",

                f"Horizontal speed: {v_horizontal:.2f} m/s",

                f"Vertical speed: {v_vertical:.2f} m/s",

                f"Tolerance: {cfg.booster_pad_tolerance_m:.0f} m",

            ]),

            xy=(dr[-1], h[-1]),

            xytext=(-190, 50),

            textcoords='offset points',

            bbox=dict(boxstyle='round', facecolor='white', alpha=0.92),

            arrowprops=dict(arrowstyle='->', lw=1.0),

            fontsize=9,

        )

    ax.set_xlabel("Ground-Track Downrange from Launch Site (km)")

    ax.set_ylabel("Altitude (km)")

    ax.set_title("RTLS Landing Error and Terminal Conditions", fontweight='bold')

    ax.legend(loc='best', framealpha=0.95)

    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    path = os.path.join(output_dir, '74_rtls_landing_error_metres.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_full_mission_arc(ascent_log, orbiter_log, booster_log,

                           separation_time: float, output_dir: str) -> str:

    """Full Mission Arc schematic â€” S1 ascent, RTLS flyback, and S2 orbital trajectory.



    Replicates the three-arc mission overview diagram showing stage separation,

    flyback burn (â‘ ), reentry burn (â‘¡), landing burn (â‘¢), and the KÃ¡rmÃ¡n line.

    """

    fig, ax = plt.subplots(figsize=(14, 7))



                                                                                 

    dr_a = _extract_series(ascent_log,  "downrange_ground")

    h_a  = _extract_series(ascent_log,  "altitude")

    dr_b = _extract_series(booster_log, "downrange_ground")

    h_b  = _extract_series(booster_log, "altitude")

    ph_b = list(getattr(booster_log, "phase_name", []))

    dr_o = _extract_series(orbiter_log, "downrange_ground")

    h_o  = _extract_series(orbiter_log, "altitude")

    ph_o = list(getattr(orbiter_log,  "phase_name", []))



                                                                    

    if len(dr_a) == 0:

        dr_a = _extract_series(ascent_log, "downrange")

    if len(dr_b) == 0:

        dr_b = _extract_series(booster_log, "downrange")

    if len(dr_o) == 0:

        dr_o = _extract_series(orbiter_log, "downrange")



                                                                                     

    MAX_S2_DOWNRANGE = 800.0      

    if len(dr_o) > 0:

        clip_mask = dr_o <= MAX_S2_DOWNRANGE

        if len(ph_o) == len(dr_o):

            for i, p in enumerate(ph_o):

                if p == "ORBIT_ACHIEVED":

                    clip_mask[i + 1:] = False

                    break

        last = int(np.argmax(~clip_mask)) if not np.all(clip_mask) else len(dr_o) - 1

        dr_o = dr_o[:last + 1]

        h_o  = h_o[:last + 1]



    sep_dr = float(dr_a[-1]) if len(dr_a) > 0 else 0.0

    sep_h  = float(h_a[-1])  if len(dr_a) > 0 else 0.0



                          

    if len(dr_a) > 0:

        ax.plot(dr_a, h_a, color='#CC2222', linewidth=2.0,

                label='Stage 1 Ascent (stacked)', zorder=4)



                                                                         

    if len(dr_o) > 0:

        dr_o2 = np.concatenate([[sep_dr], dr_o])

        h_o2  = np.concatenate([[sep_h],  h_o])

        ax.plot(dr_o2, h_o2, color='#228833', linewidth=2.0,

                label='Stage 2 trajectory (to orbit)', zorder=4)



                                                                  

    if len(dr_b) > 0:

        dr_b2 = np.concatenate([[sep_dr], dr_b])

        h_b2  = np.concatenate([[sep_h],  h_b])

        ax.plot(dr_b2, h_b2, color='#2244CC', linewidth=2.0,

                label='Booster RTLS flyback', zorder=4)



                                          

    all_x = []

    for arr in [dr_a, dr_b, dr_o]:

        if len(arr) > 0:

            all_x.extend([float(np.min(arr)), float(np.max(arr))])

    x_min = (min(all_x) - 15) if all_x else -20

    x_max = (max(all_x) + 15) if all_x else 200



                   

    ax.axhline(100, color='gray', linestyle='--', linewidth=1.2, alpha=0.7, zorder=2)

    ax.text(x_max - 5, 101.5, "100 km KÃ¡rmÃ¡n line",

            fontsize=9, ha='right', va='bottom', color='gray')



               

    ax.axhline(0, color='saddlebrown', linewidth=1.5, alpha=0.6, zorder=2)

    ax.fill_between([x_min, x_max], [-5, -5], [0, 0],

                    color='saddlebrown', alpha=0.10, zorder=1)



                 

    if len(dr_a) > 0:

        ax.scatter([dr_a[0]], [0.0], color='black', s=80, marker='^', zorder=7)

        ax.text(float(dr_a[0]) + 1.5, 2.5, "launch site", fontsize=9)



                      

    ax.scatter([sep_dr], [sep_h], color='#CC2222', s=100, marker='D', zorder=7)

    ax.text(sep_dr + 1.5, sep_h - 5, "stage sep", fontsize=9, color='#CC2222')



    def _first_idx(phases, target):

        for i, p in enumerate(phases):

            if p == target:

                return i

        return None



    boostback_idx = _first_idx(ph_b, "BOOSTER_BOOSTBACK")

    entry_idx     = _first_idx(ph_b, "BOOSTER_ENTRY")

    landing_idx   = _first_idx(ph_b, "BOOSTER_LANDING")



                                  

    if boostback_idx is not None:

        ax.scatter([dr_b[boostback_idx]], [h_b[boostback_idx]],

                   color='#2244CC', s=90, zorder=7)

        ax.annotate("â‘  flyback burn",

                    xy=(dr_b[boostback_idx], h_b[boostback_idx]),

                    xytext=(10, 8), textcoords='offset points', fontsize=9,

                    color='#2244CC')



                      

    if entry_idx is not None:

        ax.scatter([dr_b[entry_idx]], [h_b[entry_idx]],

                   color='#2244CC', s=90, zorder=7)

        ax.annotate("â‘¡ reentry burn",

                    xy=(dr_b[entry_idx], h_b[entry_idx]),

                    xytext=(-105, 10), textcoords='offset points', fontsize=9,

                    color='#2244CC',

                    arrowprops=dict(arrowstyle='->', color='#2244CC', lw=1.0))



                      

    if landing_idx is not None:

        ax.scatter([dr_b[landing_idx]], [h_b[landing_idx]],

                   color='#2244CC', s=90, zorder=7)

        ax.annotate("â‘¢ landing burn",

                    xy=(dr_b[landing_idx], h_b[landing_idx]),

                    xytext=(-95, 20), textcoords='offset points', fontsize=9,

                    color='#2244CC',

                    arrowprops=dict(arrowstyle='->', color='#2244CC', lw=1.0))



                    

    if len(dr_b) > 0:

        ax.scatter([dr_b[-1]], [max(float(h_b[-1]), 0.0)],

                   color='black', s=110, marker='*', zorder=8)



                              

    if len(dr_b) > 0:

        peak_idx = int(np.argmax(h_b))

        if peak_idx < len(dr_b) - 5:

            mid_ret = (peak_idx + len(dr_b)) // 2

            ax.text(float(dr_b[mid_ret]) - 3, float(h_b[mid_ret]) + 3,

                    "flyback\ntrajectory", fontsize=9, ha='right',

                    style='italic', color='#2244CC')

        ax.text(float(dr_b[peak_idx]) + 3, float(h_b[peak_idx]) + 1,

                "reentry\ntrajectory", fontsize=9, ha='left',

                style='italic', color='#2244CC')



    if len(dr_o) > 0:

        mid_o = len(dr_o) // 2

        ax.text(float(dr_o[mid_o]) + 3, float(h_o[mid_o]) + 2,

                "stage 2 trajectory", fontsize=9, style='italic', color='#228833')



    all_h = []

    for arr in [h_b, h_o, h_a]:

        if len(arr) > 0:

            all_h.append(float(np.max(arr)))

    h_top = max(all_h) * 1.12 if all_h else 200



    ax.set_xlim(x_min, x_max)

    ax.set_ylim(-5, h_top)

    ax.set_xlabel("Downrange Distance from Launch Site (km)", fontsize=12)

    ax.set_ylabel("Altitude (km)", fontsize=12)

    ax.set_title("Full Mission Arc: Stage 1 Ascent + RTLS Flyback + Stage 2 Orbital Trajectory",

                 fontweight='bold', fontsize=13)

    ax.legend(loc='upper right', framealpha=0.95, fontsize=10)

    ax.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '77_full_mission_arc.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





                                                                               

                                               

                                                                               



def _has_s2_recovery(orbiter_log) -> bool:

    """True when the orbiter telemetry includes the Stage-2 recovery phases."""

    return _find_phase_time(orbiter_log, "S2_LANDING") is not None





def _collect_s2_recovery_events(orbiter_log) -> list[PlotEvent]:

    """Phase markers for the Stage-2 deorbit / entry / landing sequence."""

    events: list[PlotEvent] = []

    markers = [

        ("S2_ORBIT_HOLD", "S2 Orbit Hold", '#2ca02c', ':'),

        ("S2_DEORBIT", "S2 Deorbit", '#9467bd', '--'),

        ("S2_ENTRY", "S2 Entry", '#ff7f0e', '--'),

        ("S2_LANDING", "S2 Landing Burn", '#1f77b4', '--'),

    ]

    for phase_name, label, color, linestyle in markers:

        phase_time = _find_phase_time(orbiter_log, phase_name)

        if phase_time is not None:

            events.append(PlotEvent(phase_time, label, color, linestyle))

    if _find_phase_time(orbiter_log, "S2_LANDING") is not None:

        t_o = _extract_series(orbiter_log, "time")

        h_o = _extract_series(orbiter_log, "altitude")

        if len(t_o) > 0 and len(h_o) == len(t_o) and float(h_o[-1]) <= 0.1:

            events.append(PlotEvent(float(t_o[-1]), "S2 Touchdown", '0.15', ':'))

    return sorted(events, key=lambda event: event.time)





def plot_s2_recovery_profile(orbiter_log, output_dir: str) -> str:

    """Stage-2 altitude and surface-relative speed from orbit to drone-ship touchdown."""

    fig, ax1 = plt.subplots(figsize=(11, 6))

    t = _extract_series(orbiter_log, "time")

    h = _extract_series(orbiter_log, "altitude")

    v_rel = _extract_series(orbiter_log, "velocity_rel")



    if len(t) == 0:

        ax1.text(0.5, 0.5, 'No Stage-2 recovery telemetry', transform=ax1.transAxes,

                 ha='center', va='center', fontsize=12)

    else:

        ax1.plot(t, h, color='tab:green', linewidth=2.0, label='Altitude')

        ax1.set_ylabel('Altitude (km)', color='tab:green')

        ax1.tick_params(axis='y', labelcolor='tab:green')



        ax2 = ax1.twinx()

        if len(v_rel) == len(t):

            ax2.plot(t, v_rel, color='tab:blue', linewidth=1.5, linestyle='--',

                     label='Surface-relative speed')

        ax2.set_ylabel('Surface-Relative Speed (m/s)', color='tab:blue')

        ax2.tick_params(axis='y', labelcolor='tab:blue')



        td = _booster_touchdown_speed(orbiter_log)

        ax1.scatter([t[-1]], [max(float(h[-1]), 0.0)], marker='*', color='black', s=70, zorder=6)

        ax1.annotate(f'Drone-ship touchdown\n{td:.1f} m/s (surface-rel.)',

                     xy=(t[-1], max(float(h[-1]), 0.0)), xytext=(-150, 40),

                     textcoords='offset points', fontsize=9,

                     arrowprops=dict(arrowstyle='->', alpha=0.8))



        lines = ax1.get_lines() + ax2.get_lines()

        ax1.legend(lines, [l.get_label() for l in lines], loc='center right', framealpha=0.95)



    _add_timeline_event_markers(ax1, _collect_s2_recovery_events(orbiter_log))

    ax1.set_xlabel('Time (s)')

    ax1.set_title('Stage 2 Recovery: Orbit -> Deorbit -> Entry -> Drone-Ship Landing',

                  fontweight='bold')

    ax1.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '78_s2_recovery_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_s2_entry_profile(orbiter_log, output_dir: str) -> str:

    """Stage-2 entry: surface-relative speed vs altitude (belly-flop aero-braking)."""

    fig, ax = plt.subplots(figsize=(10, 6))

    h = _extract_series(orbiter_log, "altitude")

    v_rel = _extract_series(orbiter_log, "velocity_rel")

    phases = list(getattr(orbiter_log, "phase_name", []))



    descent_idx = [i for i, p in enumerate(phases) if p in ("S2_DEORBIT", "S2_ENTRY", "S2_LANDING")]

    if len(h) == 0 or len(v_rel) != len(h) or not descent_idx:

        ax.text(0.5, 0.5, 'No Stage-2 entry telemetry', transform=ax.transAxes,

                ha='center', va='center', fontsize=12)

    else:

        sl = slice(descent_idx[0], descent_idx[-1] + 1)

        ax.plot(v_rel[sl], h[sl], color='tab:green', linewidth=2.0)

        entry_t = _find_phase_time(orbiter_log, "S2_ENTRY")

        land_t = _find_phase_time(orbiter_log, "S2_LANDING")

        t = _extract_series(orbiter_log, "time")

        for marker_t, label, color in (

            (entry_t, "Entry interface (70 km)", '#ff7f0e'),

            (land_t, "Landing burn", '#1f77b4'),

        ):

            if marker_t is not None and len(t) == len(h):

                j = int(np.argmin(np.abs(t - marker_t)))

                ax.scatter([v_rel[j]], [h[j]], color=color, s=55, zorder=5)

                ax.annotate(label, xy=(v_rel[j], h[j]), xytext=(10, 6),

                            textcoords='offset points', fontsize=9, color=color)

        ax.scatter([v_rel[sl][-1]], [max(float(h[sl][-1]), 0.0)], marker='*',

                   color='black', s=70, zorder=6)



    ax.set_xlabel('Surface-Relative Speed (m/s)')

    ax.set_ylabel('Altitude (km)')

    ax.set_title('Stage 2 Entry Profile: Speed vs Altitude (Belly-Flop Aero-Braking)',

                 fontweight='bold')

    ax.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '79_s2_entry_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_s2_landing_zoom(orbiter_log, output_dir: str, window_s: float = 320.0) -> str:

    """Zoom into the Stage-2 terminal landing burn: altitude + descent rate."""

    fig, ax1 = plt.subplots(figsize=(10, 6))

    t = _extract_series(orbiter_log, "time")

    h = _extract_series(orbiter_log, "altitude")

    v_rel = _extract_series(orbiter_log, "velocity_rel")



    if len(t) == 0:

        ax1.text(0.5, 0.5, 'No Stage-2 landing telemetry', transform=ax1.transAxes,

                 ha='center', va='center', fontsize=12)

    else:

        t_min = max(float(t[0]), float(t[-1]) - window_s)

        mask = t >= t_min

        tz, hz = t[mask], h[mask]

        vz = v_rel[mask] if len(v_rel) == len(t) else np.array([])



        ax1.plot(tz, hz, color='tab:green', linewidth=2.0, label='Altitude')

        ax1.set_ylabel('Altitude (km)', color='tab:green')

        ax1.tick_params(axis='y', labelcolor='tab:green')



        ax2 = ax1.twinx()

        if len(vz) == len(tz):

            ax2.plot(tz, vz, color='tab:blue', linewidth=1.7, label='Surface-relative speed')

        ax2.set_ylabel('Surface-Relative Speed (m/s)', color='tab:blue')

        ax2.tick_params(axis='y', labelcolor='tab:blue')



        td = _booster_touchdown_speed(orbiter_log)

        ax1.scatter([tz[-1]], [max(float(hz[-1]), 0.0)], marker='*', color='black', s=60, zorder=5)

        ax1.annotate(f'Touchdown\n{td:.1f} m/s', xy=(tz[-1], max(float(hz[-1]), 0.0)),

                     xytext=(-100, 30), textcoords='offset points', fontsize=9,

                     arrowprops=dict(arrowstyle='->', alpha=0.8))



        lines = ax1.get_lines() + ax2.get_lines()

        ax1.legend(lines, [l.get_label() for l in lines], loc='upper right', framealpha=0.95)



    ax1.set_xlabel('Time (s)')

    ax1.set_title('Stage 2 Landing Burn Segment (Zoomed)', fontweight='bold')

    ax1.grid(True, alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '80_s2_landing_zoom.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_s2_phase_fuel_budget(orbiter_log, output_dir: str) -> str:

    """Propellant consumed in each Stage-2 phase (insertion, deorbit, entry, landing)."""

    fig, ax = plt.subplots(figsize=(10, 6))

    t = _extract_series(orbiter_log, "time")

    m = _extract_series(orbiter_log, "mass")

    phases = list(getattr(orbiter_log, "phase_name", []))



    usage: dict[str, float] = {}

    if len(t) > 1 and len(m) == len(t) and len(phases) == len(t):

        start = 0

        for i in range(1, len(t)):

            if phases[i] != phases[start]:

                usage[phases[start]] = usage.get(phases[start], 0.0) + max(0.0, m[start] - m[i - 1])

                start = i

        usage[phases[start]] = usage.get(phases[start], 0.0) + max(0.0, m[start] - m[-1])



    usage = {k: v for k, v in usage.items() if v > 1.0}

    if not usage:

        ax.text(0.5, 0.5, 'No Stage-2 phase fuel data', transform=ax.transAxes,

                ha='center', va='center', fontsize=12)

    else:

        labels = list(usage.keys())

        values = [usage[k] for k in labels]

        bars = ax.bar(labels, values, color='#2ca02c', alpha=0.9)

        for b, v in zip(bars, values):

            ax.text(b.get_x() + b.get_width() / 2.0, b.get_height(), f'{v:.0f}',

                    ha='center', va='bottom', fontsize=9)

        plt.setp(ax.get_xticklabels(), rotation=20, ha='right')



    ax.set_ylabel('Propellant Used (kg)')

    ax.set_title('Stage 2 Phase Fuel Budget', fontweight='bold')

    ax.grid(True, axis='y', alpha=0.3)



    plt.tight_layout()

    path = os.path.join(output_dir, '81_s2_phase_fuel_budget.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path





def plot_s2_trajectory_path(orbiter_log, output_dir: str) -> str:

    """Stage-2 spatial trajectory in its orbital plane, drawn around Earth.



    Projects the ECI position history onto the orbital plane (true scale) so the

    full path is visible: separation -> climb to orbit -> near-circular orbit ->

    deorbit -> descent -> landing back at the surface.

    """

    from .constants import R_EARTH



    fig, ax = plt.subplots(figsize=(9, 9))

    px = _extract_series(orbiter_log, "position_x")

    py = _extract_series(orbiter_log, "position_y")

    pz = _extract_series(orbiter_log, "position_z")

    vx = _extract_series(orbiter_log, "velocity_x")

    vy = _extract_series(orbiter_log, "velocity_y")

    vz = _extract_series(orbiter_log, "velocity_z")

    t = _extract_series(orbiter_log, "time")



    n = len(px)

    if n == 0 or len(py) != n or len(pz) != n:

        ax.text(0.5, 0.5, 'No Stage-2 trajectory telemetry', transform=ax.transAxes,

                ha='center', va='center', fontsize=12)

    else:

        pos = np.column_stack([px, py, pz])

        mid = n // 2

        r0 = pos[mid]

        v0 = (np.array([vx[mid], vy[mid], vz[mid]])

              if len(vx) == n else np.array([0.0, 0.0, 1.0]))

        normal = np.cross(r0, v0)

        if np.linalg.norm(normal) < 1e-6:

            normal = np.array([0.0, 0.0, 1.0])

        normal = normal / np.linalg.norm(normal)

        e1 = r0 / max(np.linalg.norm(r0), 1.0)

        e2 = np.cross(normal, e1)

        e2 = e2 / max(np.linalg.norm(e2), 1.0)



        x_km = (pos @ e1) / 1000.0

        y_km = (pos @ e2) / 1000.0

        r_e_km = R_EARTH / 1000.0



        ax.add_patch(plt.Circle((0, 0), r_e_km, facecolor='#aac4e6', edgecolor='#5577aa',

                                alpha=0.45, zorder=1, label='Earth'))

        ax.plot(x_km, y_km, color='tab:green', linewidth=1.6, zorder=3, label='Stage 2 path')



        def _mark(phase: str, label: str, color: str) -> None:

            tp = _find_phase_time(orbiter_log, phase)

            if tp is not None and len(t) == n:

                j = int(np.argmin(np.abs(t - tp)))

                ax.scatter([x_km[j]], [y_km[j]], color=color, s=55, zorder=5)

                ax.annotate(label, (x_km[j], y_km[j]), textcoords='offset points',

                            xytext=(7, 7), fontsize=9, color=color, fontweight='bold')



        ax.scatter([x_km[0]], [y_km[0]], color='black', s=70, marker='D', zorder=6)

        ax.annotate('Separation', (x_km[0], y_km[0]), textcoords='offset points',

                    xytext=(8, -12), fontsize=9, fontweight='bold')

        _mark("ORBIT_ACHIEVED", "Orbit achieved", '#228833')

        _mark("S2_DEORBIT", "Deorbit burn", '#9467bd')

        _mark("S2_ENTRY", "Entry", '#ff7f0e')

        ax.scatter([x_km[-1]], [y_km[-1]], color='red', s=110, marker='*', zorder=7)

        ax.annotate('Landing', (x_km[-1], y_km[-1]), textcoords='offset points',

                    xytext=(8, 8), fontsize=9, color='red', fontweight='bold')



        ax.set_aspect('equal', adjustable='datalim')

        ax.set_xlabel('Orbital-plane X (km)')

        ax.set_ylabel('Orbital-plane Y (km)')

        ax.legend(loc='upper right', framealpha=0.95)



    ax.set_title('Stage 2 Trajectory Path: Separation -> Orbit -> Deorbit -> Landing',

                 fontweight='bold')

    ax.grid(True, alpha=0.25)



    plt.tight_layout()

    path = os.path.join(output_dir, '82_s2_trajectory_path.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def build_plot_manifest(saved_files):
    entries = [{"path": str(p)} for p in saved_files]
    return {
        "plots": entries,
        "summary_files": ["mission_summary.json", "mission_summary.md"],
    }


def write_plot_manifest(saved_files: list[str], output_dir: str | os.PathLike = "plots") -> Path:

    """Write plot_manifest.json beside generated plots."""

    output_path = Path(output_dir)

    output_path.mkdir(parents=True, exist_ok=True)

    manifest_path = output_path / "plot_manifest.json"

    manifest_path.write_text(

        json.dumps(build_plot_manifest(saved_files), indent=2, sort_keys=True),

        encoding="utf-8",

    )

    return manifest_path





def generate_mission_segment_plots(

    ascent_log, orbiter_log, booster_log, separation_time: float, output_dir: str = "plots", config=None

) -> list[str]:

    """Generate mission-level split tracking plots (orbiter/booster visibility)."""

    os.makedirs(output_dir, exist_ok=True)

    deprecated_outputs = [

        os.path.join(output_dir, '68_full_mission_arc.png'),

    ]

    for deprecated in deprecated_outputs:

        if os.path.exists(deprecated):

            os.remove(deprecated)

    configure_plot_style()



    plots = [

        plot_mission_altitude_split(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_mission_velocity_split(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_booster_altitude_profile(booster_log, separation_time, output_dir),

        plot_booster_velocity_profile(booster_log, separation_time, output_dir),

        plot_booster_landing_zoom(booster_log, output_dir),

        plot_booster_velocity_components(booster_log, output_dir),

        plot_booster_ignition_prediction(booster_log, output_dir),

        plot_booster_phase_fuel_budget(booster_log, output_dir),

        plot_rtls_trajectory_profile(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_full_mission_arc(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_full_mission_attitude_error(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_full_mission_angular_rates(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_full_mission_euler_angles(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_full_mission_quaternion_tracking(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_full_mission_control_response(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_full_mission_torque_components(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_gravity_turn_pitch_fpa_tracking(ascent_log, separation_time, output_dir),

        plot_full_mission_aero_loads(ascent_log, orbiter_log, booster_log, separation_time, output_dir),

        plot_rtls_landing_error(booster_log, separation_time, output_dir, config=config),

    ]



                                                                               

    if _has_s2_recovery(orbiter_log):

        plots += [

            plot_s2_recovery_profile(orbiter_log, output_dir),

            plot_s2_entry_profile(orbiter_log, output_dir),

            plot_s2_landing_zoom(orbiter_log, output_dir),

            plot_s2_phase_fuel_budget(orbiter_log, output_dir),

            plot_s2_trajectory_path(orbiter_log, output_dir),

        ]



    return plots





def generate_all_plots(log, output_dir: str = "plots") -> list[str]:

    """Generate all trajectory and telemetry plots."""

    os.makedirs(output_dir, exist_ok=True)



    deprecated_outputs = [

        os.path.join(output_dir, '10_flight_path_angle.png'),

    ]

    for deprecated in deprecated_outputs:

        if os.path.exists(deprecated):

            os.remove(deprecated)



    configure_plot_style()

    data = extract_log_data(log)



    saved_files = []

    plot_functions = [

        plot_altitude_profile,

        plot_velocity_profile,

        plot_mass_profile,

        plot_pitch_angle,

        plot_attitude_error,

        plot_control_torque,

        plot_trajectory_local,

        plot_trajectory_3d,

        plot_thrust_vs_gravity,

        plot_dynamic_pressure,

        plot_physics_check,

        plot_pitch_gamma_diagnostic,

        plot_comprehensive_dashboard,

        plot_ascent_profile,

        plot_control_dynamics,

        plot_flight_path_readme,

        plot_quaternion_norm,

        plot_angular_velocity_x,

        plot_angular_velocity_y,

        plot_angular_velocity_z,

        plot_acceleration_profile,

        plot_twr,

        plot_specific_orbital_energy,

        plot_velocity_eci_x,

        plot_velocity_eci_y,

        plot_velocity_eci_z,

        plot_horizontal_vs_vertical_velocity,

        plot_mach_number,

        plot_atmospheric_temperature,

        plot_atmospheric_density,

        plot_atmospheric_pressure,

        plot_speed_of_sound,

        plot_throttle_history,

        plot_inertia_variation,

        plot_commanded_vs_actual_quat_w,

        plot_commanded_vs_actual_quat_x,

        plot_commanded_vs_actual_quat_y,

        plot_commanded_vs_actual_quat_z,

        plot_downrange_distance,

        plot_mass_flow_rate,

        plot_propellant_fraction,

        plot_drag_coefficient_vs_mach,

        plot_energy_budget,

        plot_altitude_vs_velocity,

        plot_gravity_loss,

        plot_drag_loss,

        plot_delta_v_budget,

        plot_position_eci_x,

        plot_position_eci_y,

        plot_position_eci_z,

        plot_natural_frequency,

        plot_geocentric_radius,

        plot_specific_angular_momentum,

        plot_dynamic_pressure_vs_altitude,

        plot_effective_isp,

        plot_altitude_rate,

        plot_ground_track,

    ]



    failures = []

    for plot_func in plot_functions:

        try:

            filepath = plot_func(data, output_dir)

            saved_files.append(filepath)

        except Exception as e:

            failures.append(f"{plot_func.__name__}: {e}")



    if failures:

        raise RuntimeError(

            "Plot generation failed for "

            f"{len(failures)} plot(s): " + "; ".join(failures)

        )



    return saved_files





def main() -> None:

    """Run simulation and generate all plots."""

    from ._run_full_mission import run_full_mission
    from .config_factory import create_default_config
    from .mission_summary import write_mission_summary

    print("=" * 70)

    print("  RLV Phase-I Trajectory Visualization - Research Publication Suite")

    print("=" * 70)



    print("\nRunning full mission simulation (ascent + orbiter + booster)...")

    mission = run_full_mission(dt=0.05, verbose=True)



    print(f"\nAscent complete: {mission.ascent_reason}")

    separation_text = (

        f"{mission.separation_time:.2f} s"

        if mission.separation_time is not None

        else "not reached"

    )

    print(f"Separation time: {separation_text}")

    print(f"Orbiter result:  {mission.orbiter_reason}")

    print(f"Booster result:  {mission.booster_reason}")

    print(f"Orbiter final:   {mission.orbiter_final_state.altitude/1000:.1f} km | "

          f"{mission.orbiter_final_state.speed:.1f} m/s")

    from .utils import compute_relative_velocity as _crv

    _bst = mission.booster_final_state

    _bst_v_rel = float(np.linalg.norm(_crv(_bst.r, _bst.v)))

    print(f"Booster final:   {_bst.altitude/1000:.1f} km | "

          f"{_bst_v_rel:.1f} m/s (surface-relative)")



    print("\nGenerating publication-quality plots...")

    output_dir = "plots"

    saved_files = generate_all_plots(mission.ascent_log, output_dir)

    if mission.separation_time is not None:

        saved_files.extend(

            generate_mission_segment_plots(

                mission.ascent_log,

                mission.orbiter_log,

                mission.booster_log,

                mission.separation_time,

                output_dir,

                config=create_default_config(),

            )

        )

    else:

        print("Skipping mission segment plots because stage separation was not reached.")

    manifest_path = write_plot_manifest(saved_files, output_dir)

    summary_files = write_mission_summary(mission, create_default_config(), output_dir)



    print(f"\n{'=' * 70}")

    print(f"  Generated {len(saved_files)} publication-quality plots in '{output_dir}/'")

    print(f"  Plot manifest: {manifest_path}")

    print(f"  Mission summary: {', '.join(summary_files)}")

    print(f"{'=' * 70}")

    for i, path in enumerate(saved_files, 1):

        print(f"  {i:2d}. {os.path.basename(path)}")

    print(f"{'=' * 70}")

    print("  All plots saved at 300 DPI - ready for research paper submission.")

    print(f"{'=' * 70}")





if __name__ == "__main__":

    main()


