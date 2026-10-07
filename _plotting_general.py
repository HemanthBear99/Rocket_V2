"""General ascent and trajectory plot functions."""

import os

import matplotlib.pyplot as plt
import numpy as np

from . import constants as C
from ._plotting_common import (
    ZERO_HLINE,
    HLine,
    TimeSeriesLine,
    TimeSeriesPlot,
    TrajectoryData,
    _compute_engine_on_mask,
    _compute_ground_track_enu,
    _find_stage1_meco_index,
    _find_stage2_ignition_index,
    _find_stage_separation_index,
    compute_gravity_turn_start,
    plot_cmd_vs_actual,
    plot_time_series,
)


def plot_altitude_profile(data: TrajectoryData, output_dir: str) -> str:

    """Generate altitude vs time profile plot.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, ax = plt.subplots()

    meco_idx = _find_stage1_meco_index(data)


    ax.fill_between(data.time, 0, data.altitude, alpha=0.25, color='#1f77b4')

    ax.plot(data.time, data.altitude, 'b-', linewidth=2, label='Altitude')


    ax.scatter([data.time[0]], [data.altitude[0]],

               c='green', s=80, marker='o', zorder=5, label='Liftoff')

    ax.scatter([data.time[meco_idx]], [data.altitude[meco_idx]],

               c='red', s=80, marker='x', zorder=5,

               label=f'S1 MECO ({data.altitude[meco_idx]:.1f} km)')

    if meco_idx != len(data.time) - 1:

        ax.scatter([data.time[-1]], [data.altitude[-1]],

                   c='darkorange', s=90, marker='*', zorder=5,

                   label=f'Final ({data.altitude[-1]:.1f} km)')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Altitude (km)')

    ax.set_title('Altitude Profile', fontweight='bold')

    ax.legend(loc='lower right', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '01_altitude_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_velocity_profile(data: TrajectoryData, output_dir: str) -> str:

    """Generate velocity profile comparing inertial and relative velocities.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, ax = plt.subplots()


    ax.plot(data.time, data.velocity, 'r-', linewidth=2,

            label='Inertial Velocity')

    ax.plot(data.time, data.velocity_rel, 'g--', linewidth=2,

            label='Relative Velocity (Airspeed)')


    ax.scatter([data.time[0]], [data.velocity[0]],

               c='red', s=50, marker='o', zorder=5)

    ax.scatter([data.time[0]], [data.velocity_rel[0]],

               c='green', s=50, marker='o', zorder=5)


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Velocity (m/s)')

    ax.set_title('Velocity Profile: Inertial vs Relative', fontweight='bold')

    ax.legend(loc='lower right', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, None)


    ax.text(0.02, 0.98,

            f'Difference due to Earth rotation\n(~{data.velocity[0]:.0f} m/s at equator)',

            transform=ax.transAxes, fontsize=9,

            verticalalignment='top',

            bbox={'boxstyle': 'round', 'facecolor': 'white', 'alpha': 0.9})


    plt.tight_layout()

    path = os.path.join(output_dir, '02_velocity_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_mass_profile(data: TrajectoryData, output_dir: str) -> str:

    """Generate vehicle mass vs time profile.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, ax = plt.subplots()


    mass_tonnes = data.mass / 1000.0


    ax.fill_between(data.time, 0, mass_tonnes, alpha=0.25, color='#2ca02c')

    ax.plot(data.time, mass_tonnes, 'g-', linewidth=2, label='Vehicle Mass')

    s1_meco_mass_t = (C.DRY_MASS + C.STAGE1_LANDING_FUEL_RESERVE) / 1000.0

    ax.axhline(y=s1_meco_mass_t, color='orange', linestyle='--',

               linewidth=1.5, label=f'S1 MECO Mass ({s1_meco_mass_t:.0f} t)')

    ax.axhline(y=C.STAGE2_DRY_MASS / 1000.0, color='red', linestyle=':',

               linewidth=1.2, label=f'S2 Dry Mass ({C.STAGE2_DRY_MASS/1000:.0f} t)')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Mass (tonnes)')

    ax.set_title('Vehicle Mass Profile', fontweight='bold')

    ax.legend(loc='upper right', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '03_mass_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_pitch_angle(data: TrajectoryData, output_dir: str) -> str:

    """Generate pitch angle evolution plot.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, ax = plt.subplots()


    gravity_turn_time = compute_gravity_turn_start(data)


    ax.fill_between(data.time, 0, data.pitch_angle, alpha=0.25, color='#9467bd')

    ax.plot(data.time, data.pitch_angle, color='purple', linewidth=2,

            label='Pitch Angle (from Vertical)')

    ax.axvline(x=gravity_turn_time, color='gray', linestyle=':',

               linewidth=1.5, label='Gravity Turn Start')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Pitch Angle (Â° from Vertical)')

    ax.set_title('Pitch Angle Evolution', fontweight='bold')

    ax.legend(loc='upper left', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, 95)


    plt.tight_layout()

    path = os.path.join(output_dir, '04_pitch_angle.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_attitude_error(data: TrajectoryData, output_dir: str) -> str:
    """Generate attitude control error plot."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="05_attitude_error.png",
            title="Attitude Control Error",
            ylabel="Error (°)",
            lines=(TimeSeriesLine(data.attitude_error, label="Attitude Error", fmt="c-", linewidth=1.5),),
            hlines=(HLine(1.0, label="1° Threshold", color="orange", linestyle="--", linewidth=1.5),),
            show_legend=True,
            legend_loc="upper right",
        ),
    )


def plot_control_torque(data: TrajectoryData, output_dir: str) -> str:
    """Generate control torque magnitude plot."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="06_control_torque.png",
            title="Control Torque Magnitude",
            ylabel="Torque (MN·m)",
            lines=(
                TimeSeriesLine(
                    data.torque / 1e6,
                    color="#ff7f0e",
                    linewidth=1.5,
                ),
            ),
        ),
    )


def plot_trajectory_local(data: TrajectoryData, output_dir: str) -> str:
    """Generate 2D trajectory plot (downrange vs altitude)."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="07_trajectory_local.png",
            title="Ground Track (Local Frame)",
            ylabel="Altitude (km)",
            xlabel="Downrange (km)",
            lines=(
                TimeSeriesLine(
                    data.altitude,
                    label="Ascent Trajectory",
                    fmt="b-",
                    linewidth=2.0,
                ),
            ),
            show_legend=True,
            legend_loc="lower right",
            use_time_xlim=False,
            xlim=(0, None),
            ylim=(0, None),
        ),
        x=data.downrange,
    )


def plot_trajectory_3d(data: TrajectoryData, output_dir: str) -> str:

    """Generate 3D trajectory plot.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig = plt.figure(figsize=(10, 8))

    ax = fig.add_subplot(111, projection='3d')


    pos_x = data.position[:, 0]

    pos_y = data.position[:, 1]


    east = (pos_y - pos_y[0]) / 1000

    north = (pos_x - pos_x[0]) / 1000


    ax.plot(east, north, data.altitude, linewidth=2, color='#1f77b4')


    ax.set_xlabel('East (km)')

    ax.set_ylabel('North (km)')

    ax.set_zlabel('Altitude (km)')

    ax.set_title('3D Trajectory', fontweight='bold')


    plt.tight_layout()

    path = os.path.join(output_dir, '08_trajectory_3d.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_thrust_vs_gravity(data: TrajectoryData, output_dir: str) -> str:
    """Generate thrust vs gravity force comparison plot."""
    thrust_force = data.thrust_force / 1e6
    r_center = data.altitude * 1000.0 + C.R_EARTH
    gravity_force = (C.MU_EARTH * data.mass / (r_center**2)) / 1e6
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="09_thrust_vs_gravity.png",
            title="Thrust vs Weight",
            ylabel="Force (MN)",
            lines=(
                TimeSeriesLine(thrust_force, label="Thrust", fmt="r-", linewidth=2.0),
                TimeSeriesLine(gravity_force, label="Weight", fmt="b-", linewidth=2.0),
            ),
            show_legend=True,
            legend_loc="upper right",
        ),
    )


def plot_flight_path_angle(data: TrajectoryData, output_dir: str) -> str:
    """Generate flight path angle evolution plot."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="10_flight_path_angle.png",
            title="Flight Path Angle Evolution",
            ylabel=r"Flight Path Angle $\gamma$ (° from Horizontal)",
            lines=(
                TimeSeriesLine(
                    data.gamma_rel,
                    label=r"$\gamma_{relative}$ (Primary)",
                    color="#1f77b4",
                    linewidth=2.5,
                ),
                TimeSeriesLine(
                    data.gamma_cmd,
                    label=r"$\gamma_{command}$",
                    color="green",
                    linestyle="--",
                    linewidth=1.5,
                ),
                TimeSeriesLine(
                    data.gamma_actual,
                    label=r"$\gamma_{actual}$",
                    color="red",
                    linestyle=":",
                    linewidth=1.5,
                ),
            ),
            hlines=(
                HLine(90.0, color="gray", linestyle=":", alpha=0.5),
                HLine(0.0, color="gray", linestyle=":", alpha=0.5),
            ),
            show_legend=True,
            legend_loc="center right",
            ylim=(-5, 100),
        ),
    )


def plot_dynamic_pressure(data: TrajectoryData, output_dir: str) -> str:
    """Generate dynamic pressure plot with structural limit."""
    limit_kpa = C.MAX_DYNAMIC_PRESSURE / 1000.0
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="10_dynamic_pressure.png",
            title="Dynamic Pressure (q)",
            ylabel="Dynamic Pressure (kPa)",
            lines=(
                TimeSeriesLine(
                    data.dynamic_pressure / 1000.0,
                    label="Dynamic Pressure",
                    color="#9467bd",
                    linewidth=2.0,
                ),
            ),
            hlines=(
                HLine(
                    limit_kpa,
                    label=f"Structural Limit ({limit_kpa:.0f} kPa)",
                    color="red",
                    linestyle="--",
                    linewidth=1.5,
                ),
            ),
            show_legend=True,
            legend_loc="upper right",
            ylim=(0, None),
        ),
    )


def plot_physics_check(data: TrajectoryData, output_dir: str) -> str:

    """Generate physics validation plot (angle of attack analysis).


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)


    v_rel_mag = np.linalg.norm(data.velocity_rel_vec, axis=1)

    r_mag = np.linalg.norm(data.position, axis=1)

    r_hat = data.position / r_mag[:, np.newaxis]

    v_rel_radial = np.sum(data.velocity_rel_vec * r_hat, axis=1)

    cos_theta = np.clip(v_rel_radial / np.maximum(v_rel_mag, 1.0), -1.0, 1.0)

    velocity_angle = np.degrees(np.arccos(cos_theta))


    ax1.plot(data.time, data.pitch_angle, 'purple', linewidth=2,

             label='Pitch Angle $\\theta$ (Body Z)')

    ax1.plot(data.time, velocity_angle, 'g--', linewidth=2,

             label='Velocity Angle (Wind-Relative)')

    ax1.set_ylabel('Angle from Vertical (Â°)')

    ax1.set_title('Pitch vs Wind-Relative Velocity Vector', fontweight='bold')

    ax1.legend(loc='lower right', framealpha=0.95)

    ax1.set_ylim(0, 95)

    ax1.grid(True, which='both', alpha=0.3)


    alpha = np.abs(data.pitch_angle - velocity_angle)


    q_threshold = 100.0

    alpha_masked = np.where(data.dynamic_pressure > q_threshold, alpha, np.nan)


    line1 = ax2.plot(data.time, alpha_masked, 'r-', linewidth=2,

             label='Angle of Attack (Wind-Relative)')

    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.8)

    line2 = ax2.axhline(y=10, color='orange', linestyle='--',

                label='10Â° Safety Threshold')


    ax2.set_ylabel('Angle of Attack (Â°)')


    ax2.set_ylim(0, min(15, max(12, np.nanmax(alpha_masked)*1.2 if not np.all(np.isnan(alpha_masked)) else 12)))


    ax2b = ax2.twinx()

    q_kpa = data.dynamic_pressure / 1000.0


    ax2b.fill_between(data.time, 0, q_kpa, color='gray', alpha=0.15, label='Dynamic Pressure (q)')

    line3 = ax2b.plot(data.time, q_kpa, color='gray', linestyle=':', linewidth=1, label='Dynamic Pressure')


    ax2b.set_ylabel('Dynamic Pressure (kPa)', color='gray')

    ax2b.tick_params(axis='y', labelcolor='gray')

    ax2b.set_ylim(0, None)


    lines = line1 + [line2] + line3

    lbls = [l.get_label() for l in lines]

    ax2.legend(lines, lbls, loc='upper right', framealpha=0.95)


    ax2.set_xlabel('Time (s)')

    ax2.set_title('Angle of Attack & Dynamic Pressure', fontweight='bold')


    high_q_idx = np.where(q_kpa > 10.0)[0]

    if len(high_q_idx) > 0:

        t_lock = data.time[high_q_idx[0]]

        ax2.axvline(x=t_lock, color='blue', linestyle='-.', alpha=0.5)

        ax2.text(t_lock + 2, ax2.get_ylim()[1]*0.8, "High-Q / Prograde Lock",

                 color='blue', fontsize=9, rotation=0)


    plt.tight_layout()

    path = os.path.join(output_dir, '12_physics_validation.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_pitch_gamma_diagnostic(data: TrajectoryData, output_dir: str) -> str:

    """Generate diagnostic plot comparing thrust direction vs velocity direction.


    This is the key diagnostic for verifying pitch/trajectory coupling:

    - Thrust pitch (from vertical): Actual body Z vs local vertical

    - Velocity tilt (from vertical): atan(v_horiz/v_vert)

    - Gamma command (from horizontal): Guidance target


    If these don't align properly, there's a frame or coupling issue.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

    fig.suptitle('Pitch/Gamma Diagnostic: Thrust Direction vs Velocity Direction',

                 fontsize=14, fontweight='bold')


    r_mag = np.linalg.norm(data.position, axis=1)

    r_hat = data.position / r_mag[:, np.newaxis]


    v_rel_radial = np.sum(data.velocity_rel_vec * r_hat, axis=1)

    v_rel_tangent = np.sqrt(np.maximum(data.velocity_rel**2 - v_rel_radial**2, 0))

    velocity_tilt_from_vertical = np.degrees(np.arctan2(v_rel_tangent, np.maximum(v_rel_radial, 1.0)))


    pitch_cmd = data.pitch_angle

    pitch_actual = data.actual_pitch


    gamma_implies_pitch = 90.0 - data.gamma_cmd


    ax1.plot(data.time, pitch_cmd, 'b-', linewidth=2, label='Pitch Command (from vertical)')

    ax1.plot(data.time, pitch_actual, 'b--', linewidth=1.5, alpha=0.7, label='Pitch Actual (from vertical)')

    ax1.plot(data.time, velocity_tilt_from_vertical, 'g-', linewidth=2, label='Velocity Tilt (from vertical)')

    ax1.plot(data.time, gamma_implies_pitch, 'r:', linewidth=2, label='90Â° - Î³_cmd')


    ax1.set_ylabel('Angle from Vertical (Â°)')

    ax1.set_title('Comparison: Thrust Pitch vs Velocity Tilt')

    ax1.legend(loc='upper left', fontsize=9, framealpha=0.95)

    ax1.set_ylim(0, 95)

    ax1.axhline(45, color='gray', linestyle=':', alpha=0.3)


    gamma_measured = 90.0 - velocity_tilt_from_vertical

    ax2.plot(data.time, data.gamma_cmd, 'purple', linewidth=2, label='Î³ Command')

    ax2.plot(data.time, gamma_measured, 'orange', linewidth=2, label='Î³ Measured (from velocity)')

    ax2.plot(data.time, data.gamma_rel, 'k--', linewidth=1.5, alpha=0.7, label='Î³ Relative (logged)')


    ax2.set_ylabel('Flight Path Angle Î³ (Â° from horizontal)')

    ax2.set_title('Flight Path Angle: Command vs Actual')

    ax2.legend(loc='upper right', fontsize=9, framealpha=0.95)

    ax2.set_ylim(-5, 100)

    ax2.axhline(90, color='gray', linestyle=':', alpha=0.3, label='Vertical')

    ax2.axhline(0, color='gray', linestyle=':', alpha=0.3, label='Horizontal')


    pitch_velocity_diff = np.abs(pitch_actual - velocity_tilt_from_vertical)

    gamma_tracking_error = np.abs(data.gamma_cmd - gamma_measured)


    ax3.plot(data.time, pitch_velocity_diff, 'r-', linewidth=2,

             label='|Thrust Pitch - Velocity Tilt|')

    ax3.plot(data.time, gamma_tracking_error, 'b--', linewidth=1.5,

             label='|Î³_cmd - Î³_meas|')

    ax3.axhline(10, color='orange', linestyle='--', linewidth=1.5, label='10Â° Warning')


    ax3.set_xlabel('Time (s)')

    ax3.set_ylabel('Angle Difference (Â°)')

    ax3.set_title('Alignment & Tracking Error')

    ax3.legend(loc='upper right', fontsize=9, framealpha=0.95)

    ax3.set_ylim(0, None)


    meco_idx = _find_stage1_meco_index(data)

    final_pitch = pitch_actual[meco_idx] if len(pitch_actual) > 0 else 0

    final_v_tilt = velocity_tilt_from_vertical[meco_idx] if len(velocity_tilt_from_vertical) > 0 else 0

    final_gamma = data.gamma_cmd[meco_idx] if len(data.gamma_cmd) > 0 else 0

    max_downrange = data.downrange[meco_idx] if len(data.downrange) > 0 else 0


    summary = (f"At S1 MECO:\n"

               f"  Pitch (from vert): {final_pitch:.1f}Â°\n"

               f"  Vel tilt (from vert): {final_v_tilt:.1f}Â°\n"

               f"  Î³ command: {final_gamma:.1f}Â° (from horiz)\n"

               f"  Downrange: {max_downrange:.1f} km\n\n"

               f"Expected: pitch â‰ˆ vel_tilt â‰ˆ (90-Î³)")


    ax3.text(0.98, 0.97, summary, transform=ax3.transAxes, fontsize=9,

             verticalalignment='top', horizontalalignment='right',

             bbox={'boxstyle': 'round', 'facecolor': 'lightyellow', 'alpha': 0.95})


    plt.tight_layout()

    path = os.path.join(output_dir, '14_pitch_gamma_diagnostic.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_comprehensive_dashboard(data: TrajectoryData, output_dir: str) -> str:

    """Generate comprehensive 6-panel dashboard summary.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, axes = plt.subplots(2, 3, figsize=(16, 10))

    fig.suptitle('RLV Phase-I Ascent Summary', fontsize=16, fontweight='bold')


    ax = axes[0, 0]

    ax.fill_between(data.time, 0, data.altitude, alpha=0.25, color='#1f77b4')

    ax.plot(data.time, data.altitude, 'b-', linewidth=2)

    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Altitude (km)')

    ax.set_title('Altitude Profile')

    ax.set_xlim(0, data.time[-1])


    ax = axes[0, 1]

    ax.plot(data.time, data.velocity, 'r-', linewidth=2, label='Inertial')

    ax.plot(data.time, data.velocity_rel, 'g--', linewidth=2, label='Relative')

    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Velocity (m/s)')

    ax.set_title('Velocity Profile')

    ax.legend(fontsize=8, framealpha=0.95)

    ax.set_xlim(0, data.time[-1])


    ax = axes[0, 2]

    mass_tonnes = data.mass / 1000.0

    ax.fill_between(data.time, 0, mass_tonnes, alpha=0.25, color='#2ca02c')

    ax.plot(data.time, mass_tonnes, 'g-', linewidth=2)

    ax.axhline(y=(C.DRY_MASS + C.STAGE1_LANDING_FUEL_RESERVE)/1000,

               color='orange', linestyle='--', linewidth=1)

    ax.axhline(y=C.STAGE2_DRY_MASS/1000, color='red', linestyle=':', linewidth=1)

    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Mass (tonnes)')

    ax.set_title('Vehicle Mass')

    ax.set_xlim(0, data.time[-1])


    ax = axes[1, 0]

    ax.plot(data.time, data.pitch_angle, 'purple', linewidth=2)

    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Pitch (Â° from Vertical)')

    ax.set_title('Pitch Angle')

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, 95)


    ax = axes[1, 1]

    ax.plot(data.time, data.gamma_rel, 'b-', linewidth=2)

    ax.axhline(y=90, color='gray', linestyle=':', alpha=0.5)

    ax.axhline(y=0, color='gray', linestyle=':', alpha=0.5)

    ax.set_xlabel('Time (s)')

    ax.set_ylabel(r'$\gamma$ (Â° from Horizontal)')

    ax.set_title('Flight Path Angle')

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(-5, 100)


    ax = axes[1, 2]

    ax.plot(data.downrange, data.altitude, 'b-', linewidth=2)

    ax.set_xlabel('Downrange (km)')

    ax.set_ylabel('Altitude (km)')

    ax.set_title('Ground Track')

    ax.set_xlim(0, None)

    ax.set_ylim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '13_comprehensive_summary.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_ascent_profile(data: TrajectoryData, output_dir: str) -> str:

    """Generate the ascent_profile.png dashboard referenced in README.


    Combined 4-panel view of Altitude, Velocity, Mass, and Pitch.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    fig.suptitle('RLV Phase-I Ascent Profile Dashboard', fontsize=16, fontweight='bold')

    meco_idx = _find_stage1_meco_index(data)


    ax = axes[0, 0]

    ax.fill_between(data.time, 0, data.altitude, alpha=0.3, color='#1f77b4')

    ax.plot(data.time, data.altitude, 'b-', linewidth=2.5)

    ax.scatter([data.time[meco_idx]], [data.altitude[meco_idx]], c='red', s=90, marker='x',

               zorder=5, label=f'S1 MECO: {data.altitude[meco_idx]:.1f} km')

    if meco_idx != len(data.time) - 1:

        ax.scatter([data.time[-1]], [data.altitude[-1]], c='darkorange', s=100, marker='*',

                   zorder=5, label=f'Final: {data.altitude[-1]:.1f} km')

    ax.set_xlabel('Time (s)', fontsize=11)

    ax.set_ylabel('Altitude (km)', fontsize=11)

    ax.set_title('Altitude Profile', fontweight='bold', fontsize=12)

    ax.legend(loc='lower right', fontsize=10)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, None)

    ax.grid(True, alpha=0.3)


    ax = axes[0, 1]

    ax.plot(data.time, data.velocity, 'r-', linewidth=2.5, label='Inertial')

    ax.plot(data.time, data.velocity_rel, 'g--', linewidth=2, label='Relative')

    ax.scatter([data.time[meco_idx]], [data.velocity[meco_idx]], c='red', s=90, marker='x',

               zorder=5, label=f'S1 MECO: {data.velocity[meco_idx]:.0f} m/s')

    if meco_idx != len(data.time) - 1:

        ax.scatter([data.time[-1]], [data.velocity[-1]], c='darkorange', s=100, marker='*',

                   zorder=5, label=f'Final: {data.velocity[-1]:.0f} m/s')

    ax.set_xlabel('Time (s)', fontsize=11)

    ax.set_ylabel('Velocity (m/s)', fontsize=11)

    ax.set_title('Velocity Profile', fontweight='bold', fontsize=12)

    ax.legend(loc='lower right', fontsize=10)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, None)

    ax.grid(True, alpha=0.3)


    ax = axes[1, 0]

    mass_tonnes = data.mass / 1000.0

    ax.fill_between(data.time, 0, mass_tonnes, alpha=0.3, color='#2ca02c')

    ax.plot(data.time, mass_tonnes, 'g-', linewidth=2.5)

    s1_meco_mass_t = (C.DRY_MASS + C.STAGE1_LANDING_FUEL_RESERVE) / 1000.0

    ax.axhline(y=s1_meco_mass_t, color='orange', linestyle='--', linewidth=1.8,

               label=f'S1 MECO Mass: {s1_meco_mass_t:.0f} t')

    ax.axhline(y=C.STAGE2_DRY_MASS / 1000.0, color='red', linestyle=':', linewidth=1.3,

               label=f'S2 Dry Mass: {C.STAGE2_DRY_MASS/1000:.0f} t')

    ax.set_xlabel('Time (s)', fontsize=11)

    ax.set_ylabel('Mass (tonnes)', fontsize=11)

    ax.set_title('Vehicle Mass', fontweight='bold', fontsize=12)

    ax.legend(loc='upper right', fontsize=10)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, None)

    ax.grid(True, alpha=0.3)


    ax = axes[1, 1]

    ax.fill_between(data.time, 0, data.pitch_angle, alpha=0.3, color='#9467bd')

    ax.plot(data.time, data.pitch_angle, color='purple', linewidth=2.5)

    ax.set_xlabel('Time (s)', fontsize=11)

    ax.set_ylabel('Pitch Angle (Â° from Vertical)', fontsize=11)

    ax.set_title('Pitch Angle Evolution', fontweight='bold', fontsize=12)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, 95)

    ax.grid(True, alpha=0.3)


    plt.tight_layout()

    path = os.path.join(output_dir, 'ascent_profile.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_control_dynamics(data: TrajectoryData, output_dir: str) -> str:

    """Generate the control_dynamics.png referenced in README.


    Combined view of Attitude Error, Control Torque, and Guidance Commands.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

    fig.suptitle('Control System Performance', fontsize=16, fontweight='bold')


    ax = axes[0]

    ax.plot(data.time, data.attitude_error, 'c-', linewidth=2, label='Attitude Error')

    ax.axhline(y=1.0, color='orange', linestyle='--', linewidth=2, label='1Â° Threshold')

    max_error = np.max(data.attitude_error)

    ax.axhline(y=max_error, color='red', linestyle=':', linewidth=1.5, alpha=0.7,

               label=f'Max: {max_error:.2f}Â°')

    ax.set_ylabel('Attitude Error (Â°)', fontsize=11)

    ax.set_title('Attitude Tracking Error', fontweight='bold', fontsize=12)

    ax.legend(loc='upper right', fontsize=10)

    ax.grid(True, alpha=0.3)

    ax.set_ylim(0, None)


    ax = axes[1]

    torque_mn = data.torque / 1e6

    ax.plot(data.time, torque_mn, color='#ff7f0e', linewidth=2, label='Control Torque')

    ax.axhline(y=C.MAX_TORQUE/1e6, color='red', linestyle='--', linewidth=1.5,

               label=f'Saturation: {C.MAX_TORQUE/1e6:.1f} MNÂ·m')

    ax.set_ylabel('Torque (MNÂ·m)', fontsize=11)

    ax.set_title('Control Torque Magnitude', fontweight='bold', fontsize=12)

    ax.legend(loc='upper right', fontsize=10)

    ax.grid(True, alpha=0.3)

    ax.set_ylim(0, None)


    ax = axes[2]

    ax.plot(data.time, data.gamma_cmd, 'b-', linewidth=2.5, label='Î³ Command')

    ax.plot(data.time, data.gamma_rel, 'g--', linewidth=2, label='Î³ Actual')

    ax.axhline(y=90, color='gray', linestyle=':', alpha=0.5)

    ax.axhline(y=0, color='gray', linestyle=':', alpha=0.5)

    ax.set_xlabel('Time (s)', fontsize=11)

    ax.set_ylabel('Flight Path Angle (Â°)', fontsize=11)

    ax.set_title('Guidance Commands', fontweight='bold', fontsize=12)

    ax.legend(loc='center right', fontsize=10)

    ax.grid(True, alpha=0.3)

    ax.set_ylim(-5, 100)


    plt.tight_layout()

    path = os.path.join(output_dir, 'control_dynamics.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_flight_path_readme(data: TrajectoryData, output_dir: str) -> str:

    """Generate the 11_flight_path_angle.png referenced in README.


    Flight path angle evolution with correct filename for README.


    Args:

        data: TrajectoryData object

        output_dir: Directory to save the plot


    Returns:

        Path to saved plot file

    """

    fig, ax = plt.subplots(figsize=(10, 7))


    ax.plot(data.time, data.gamma_rel, 'b-', linewidth=3,

            label=r'$\gamma_{relative}$ (Primary)')

    ax.plot(data.time, data.gamma_cmd, 'g--', linewidth=2, alpha=0.8,

            label=r'$\gamma_{command}$')

    ax.plot(data.time, data.gamma_actual, 'r:', linewidth=2, alpha=0.8,

            label=r'$\gamma_{actual}$')


    ax.axhline(y=90, color='gray', linestyle=':', alpha=0.5, linewidth=1.5)

    ax.axhline(y=0, color='gray', linestyle=':', alpha=0.5, linewidth=1.5)


    ax.set_xlabel('Time (s)', fontsize=12)

    ax.set_ylabel(r'Flight Path Angle $\gamma$ (Â° from Horizontal)', fontsize=12)

    ax.set_title('Flight Path Angle Evolution', fontweight='bold', fontsize=14)

    ax.legend(loc='center right', fontsize=11, framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(-5, 100)

    ax.grid(True, alpha=0.3)


    textstr = (r'$\gamma$ Definition:' + '\n'

               r'$\gamma = 90Â°$: Vertical climb' + '\n'

               r'$\gamma = 0Â°$: Horizontal flight' + '\n'

               r'$\gamma < 0Â°$: Descent')

    ax.text(0.02, 0.35, textstr, transform=ax.transAxes, fontsize=10,

            verticalalignment='top',

            bbox={'boxstyle': 'round', 'facecolor': 'lightyellow', 'alpha': 0.95})


    plt.tight_layout()

    path = os.path.join(output_dir, '16_flight_path_angle.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)


    return path


def plot_quaternion_norm(data: TrajectoryData, output_dir: str) -> str:

    """Quaternion norm history â€” validates numerical integration integrity.


    A deviation from ||q|| = 1 indicates numerical drift in the attitude

    integrator. This is a key validation metric for any quaternion-based

    6-DOF simulation.

    """

    fig, ax = plt.subplots()


    norm_err = np.maximum(np.abs(data.quaternion_norm - 1.0), 1e-16)

    ax.semilogy(data.time, norm_err, color='#d62728', linewidth=1.8,

                label='|  ||q|| - 1  |')

    ax.axhline(y=C.QUATERNION_NORM_TOL, color='orange', linestyle='--',

               linewidth=1.5, label=f'Tolerance ({C.QUATERNION_NORM_TOL:.0e})')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Quaternion Norm Error  |  ||q|| - 1  |')

    ax.set_title('Quaternion Norm Integrity Check', fontweight='bold')

    ax.legend(loc='upper right', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])


    textstr = (f'Max deviation: {np.max(norm_err):.2e}\n'

               f'Mean deviation: {np.mean(norm_err):.2e}\n'

               f'RK4 + renormalization')

    ax.text(0.02, 0.97, textstr, transform=ax.transAxes, fontsize=9,

            verticalalignment='top',

            bbox={'boxstyle': 'round', 'facecolor': 'lightyellow', 'alpha': 0.95})


    plt.tight_layout()

    path = os.path.join(output_dir, '18_quaternion_norm.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_angular_velocity_x(data: TrajectoryData, output_dir: str) -> str:
    """Body-frame angular velocity X component (pitch/yaw transverse rate)."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="19_omega_x_pitch_yaw.png",
            title=r"Angular Velocity — Pitch/Yaw Transverse Rate ($\omega_x$)",
            ylabel=r"$\omega_x$ (°/s)",
            lines=(TimeSeriesLine(np.degrees(data.omega_x), color="#1f77b4"),),
            hlines=(ZERO_HLINE,),
        ),
    )


def plot_angular_velocity_y(data: TrajectoryData, output_dir: str) -> str:
    """Body-frame angular velocity Y component (pitch rate)."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="20_omega_y_pitch.png",
            title=r"Angular Velocity — Pitch Rate ($\omega_y$)",
            ylabel=r"$\omega_y$ (°/s)",
            lines=(TimeSeriesLine(np.degrees(data.omega_y), color="#ff7f0e"),),
            hlines=(ZERO_HLINE,),
        ),
    )


def plot_angular_velocity_z(data: TrajectoryData, output_dir: str) -> str:
    """Body-frame angular velocity Z component (roll rate about longitudinal axis)."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="21_omega_z_roll.png",
            title=r"Angular Velocity — Roll Rate ($\omega_z$, about longitudinal axis)",
            ylabel=r"$\omega_z$ (°/s)",
            lines=(TimeSeriesLine(np.degrees(data.omega_z), color="#2ca02c"),),
            hlines=(ZERO_HLINE,),
        ),
    )


def plot_acceleration_profile(data: TrajectoryData, output_dir: str) -> str:
    """Total acceleration magnitude in g-units vs time."""
    dt = np.diff(data.time)
    dv = np.diff(data.velocity)
    accel = np.zeros(len(data.time))
    accel[1:] = dv / dt
    accel[0] = accel[1]
    accel_g = accel / C.G0
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="22_acceleration_profile.png",
            title="Vehicle Acceleration Profile",
            ylabel="Acceleration (g)",
            lines=(
                TimeSeriesLine(accel_g, label="Net Acceleration", color="#d62728", linewidth=1.8),
            ),
            hlines=(ZERO_HLINE,),
            show_legend=True,
            legend_loc="upper left",
        ),
    )


def plot_twr(data: TrajectoryData, output_dir: str) -> str:

    """Thrust-to-Weight Ratio vs time.


    TWR > 1 is required for liftoff; TWR increases as propellant is consumed.

    This is a fundamental figure of merit for launch vehicle performance.

    """

    fig, ax = plt.subplots()

    meco_idx = _find_stage1_meco_index(data)


    r_center = data.altitude * 1000.0 + C.R_EARTH

    weight = C.MU_EARTH * data.mass / (r_center ** 2)

    twr = data.thrust_force / np.maximum(weight, 1.0)


    ax.plot(data.time, twr, color='#e377c2', linewidth=2, label='TWR')

    ax.axhline(y=1.0, color='red', linestyle='--', linewidth=1.5, label='TWR = 1 (hover)')

    ax.fill_between(data.time, 1.0, twr, where=(twr > 1.0),

                    alpha=0.15, color='green', label='Excess thrust')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Thrust-to-Weight Ratio')

    ax.set_title('Thrust-to-Weight Ratio (TWR)', fontweight='bold')

    ax.legend(loc='upper left', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, None)


    textstr = (f'Liftoff TWR: {twr[0]:.2f}\n'

               f'S1 MECO TWR: {twr[meco_idx]:.2f}\n'

               f'Final TWR: {twr[-1]:.2f}')

    ax.text(0.98, 0.50, textstr, transform=ax.transAxes, fontsize=10,

            ha='right', va='center',

            bbox={'boxstyle': 'round', 'facecolor': 'lightyellow', 'alpha': 0.95})


    plt.tight_layout()

    path = os.path.join(output_dir, '23_thrust_to_weight.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_specific_orbital_energy(data: TrajectoryData, output_dir: str) -> str:
    """Specific orbital energy gain vs time (relative to liftoff)."""
    r_mag = np.linalg.norm(data.position, axis=1)
    specific_energy = 0.5 * data.velocity ** 2 - C.MU_EARTH / r_mag
    delta_energy_mj = (specific_energy - specific_energy[0]) / 1e6
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="24_specific_orbital_energy.png",
            title=r"Specific Orbital Energy Gain  $\Delta\varepsilon = \varepsilon - \varepsilon_0$",
            ylabel=r"$\Delta\varepsilon$ (MJ/kg)",
            lines=(
                TimeSeriesLine(
                    delta_energy_mj,
                    label=r"$\Delta\varepsilon$ (gain from liftoff)",
                    color="#17becf",
                    linewidth=2.0,
                ),
            ),
            hlines=(ZERO_HLINE,),
            show_legend=True,
            legend_loc="upper left",
        ),
    )


def plot_velocity_eci_x(data: TrajectoryData, output_dir: str) -> str:
    """ECI velocity X-component vs time."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="25_velocity_eci_x.png",
            title="ECI Velocity X-Component (Radial at Launch)",
            ylabel="$V_X$ (m/s)",
            lines=(TimeSeriesLine(data.velocity_x, color="#1f77b4"),),
            hlines=(ZERO_HLINE,),
        ),
    )


def plot_velocity_eci_y(data: TrajectoryData, output_dir: str) -> str:
    """ECI velocity Y-component vs time."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="26_velocity_eci_y.png",
            title="ECI Velocity Y-Component (East at Launch)",
            ylabel="$V_Y$ (m/s)",
            lines=(TimeSeriesLine(data.velocity_y, color="#ff7f0e"),),
            hlines=(ZERO_HLINE,),
        ),
    )


def plot_velocity_eci_z(data: TrajectoryData, output_dir: str) -> str:
    """ECI velocity Z-component vs time."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="27_velocity_eci_z.png",
            title="ECI Velocity Z-Component (North at Launch)",
            ylabel="$V_Z$ (m/s)",
            lines=(TimeSeriesLine(data.velocity_z, color="#2ca02c"),),
            hlines=(ZERO_HLINE,),
        ),
    )


def plot_horizontal_vs_vertical_velocity(data: TrajectoryData, output_dir: str) -> str:

    """Horizontal vs vertical velocity decomposition.


    This shows the transition from vertical ascent to horizontal flight,

    which is the hallmark of a gravity turn trajectory.

    """

    fig, ax = plt.subplots()


    ax.plot(data.time, data.velocity_vertical, color='#1f77b4', linewidth=2,

            label='Vertical (radial)')

    ax.plot(data.time, data.velocity_horizontal, color='#ff7f0e', linewidth=2,

            label='Horizontal (tangential)')

    ax.axhline(y=0, color='gray', linestyle=':', alpha=0.5)


    diff = np.abs(data.velocity_horizontal - data.velocity_vertical)

    if len(diff) > 5:

        crossover_idx = np.argmin(diff[5:]) + 5

        ax.axvline(x=data.time[crossover_idx], color='purple', linestyle='-.',

                   alpha=0.6, linewidth=1.5,

                   label=f'Crossover t={data.time[crossover_idx]:.0f}s')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Velocity Component (m/s)')

    ax.set_title('Horizontal vs Vertical Velocity Decomposition', fontweight='bold')

    ax.legend(loc='upper left', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])


    plt.tight_layout()

    path = os.path.join(output_dir, '28_horiz_vs_vert_velocity.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_mach_number(data: TrajectoryData, output_dir: str) -> str:

    """Mach number vs time.


    Key regimes: subsonic (M<0.8), transonic (0.8<M<1.3), supersonic (M>1.3),

    hypersonic (M>5). Annotated with regime boundaries.

    """

    fig, ax = plt.subplots()


    ax.plot(data.time, data.mach_number, color='#d62728', linewidth=2, label='Mach Number')


    ax.axhline(y=1.0, color='orange', linestyle='--', linewidth=1.5, label='M = 1 (sonic)')

    ax.axhline(y=5.0, color='blue', linestyle='-.', linewidth=1.2, label='M = 5 (hypersonic)')


    ax.axhspan(0, 0.8, alpha=0.04, color='green')

    ax.axhspan(0.8, 1.3, alpha=0.06, color='orange')

    ax.axhspan(1.3, 5.0, alpha=0.04, color='red')

    ax.axhspan(5.0, max(15, np.max(data.mach_number)*1.1), alpha=0.04, color='blue')


    ax.text(data.time[-1]*0.02, 0.4, 'Subsonic', fontsize=8, color='green')

    ax.text(data.time[-1]*0.02, 1.05, 'Transonic', fontsize=8, color='orange')

    ax.text(data.time[-1]*0.02, 3.0, 'Supersonic', fontsize=8, color='red')

    if np.max(data.mach_number) > 5.5:

        ax.text(data.time[-1]*0.02, 6.0, 'Hypersonic', fontsize=8, color='blue')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Mach Number')

    ax.set_title('Mach Number Profile', fontweight='bold')

    ax.legend(loc='upper left', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '29_mach_number.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_atmospheric_temperature(data: TrajectoryData, output_dir: str) -> str:
    """Atmospheric temperature vs altitude (US Standard Atmosphere 1976)."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="30_atm_temperature.png",
            title="Atmospheric Temperature vs Altitude",
            ylabel="Altitude (km)",
            xlabel="Temperature (K)",
            lines=(TimeSeriesLine(data.altitude, color="#d62728", linewidth=2.0),),
            hlines=(
                HLine(
                    11.0,
                    label="Tropopause (11 km)",
                    linestyle="--",
                    linewidth=1.2,
                ),
            ),
            show_legend=True,
            legend_loc="upper right",
            use_time_xlim=False,
            ylim=(0, None),
        ),
        x=data.temperature,
    )


def plot_atmospheric_density(data: TrajectoryData, output_dir: str) -> str:

    """Atmospheric density (log scale) vs altitude."""

    fig, ax = plt.subplots()


    valid = data.density > 0

    ax.semilogy(data.altitude[valid], data.density[valid],

                color='#9467bd', linewidth=2)


    ax.set_xlabel('Altitude (km)')

    ax.set_ylabel(r'Density $\rho$ (kg/m$^3$)')

    ax.set_title('Atmospheric Density vs Altitude (Log Scale)', fontweight='bold')

    ax.set_xlim(0, None)


    textstr = (f'Sea level: {data.density[0]:.3f} kg/m$^3$\n'

               f'Final sample: {data.density[-1]:.2e} kg/m$^3$')

    ax.text(0.50, 0.97, textstr, transform=ax.transAxes, fontsize=10,

            ha='center', va='top',

            bbox={'boxstyle': 'round', 'facecolor': 'lightyellow', 'alpha': 0.95})


    plt.tight_layout()

    path = os.path.join(output_dir, '31_atm_density.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_atmospheric_pressure(data: TrajectoryData, output_dir: str) -> str:

    """Atmospheric pressure (log scale) vs altitude."""

    fig, ax = plt.subplots()


    valid = data.pressure > 0

    ax.semilogy(data.altitude[valid], data.pressure[valid] / 1000.0,

                color='#17becf', linewidth=2)


    ax.set_xlabel('Altitude (km)')

    ax.set_ylabel('Pressure (kPa)')

    ax.set_title('Atmospheric Pressure vs Altitude (Log Scale)', fontweight='bold')

    ax.set_xlim(0, None)


    textstr = f'Sea level: {data.pressure[0]/1000:.1f} kPa'

    ax.text(0.50, 0.97, textstr, transform=ax.transAxes, fontsize=10,

            ha='center', va='top',

            bbox={'boxstyle': 'round', 'facecolor': 'lightyellow', 'alpha': 0.95})


    plt.tight_layout()

    path = os.path.join(output_dir, '32_atm_pressure.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_speed_of_sound(data: TrajectoryData, output_dir: str) -> str:

    """Speed of sound vs altitude along trajectory."""

    fig, ax = plt.subplots()


    alt_limit_km = 120.0

    valid = data.altitude <= alt_limit_km

    if np.any(valid):

        ax.plot(data.altitude[valid], data.speed_of_sound[valid], color='#8c564b', linewidth=2)

    else:

        ax.plot(data.altitude, data.speed_of_sound, color='#8c564b', linewidth=2)

    ax.axvline(x=11.0, color='gray', linestyle='--', linewidth=1.2, alpha=0.5,

               label='Tropopause (11 km)')


    ax.set_xlabel('Altitude (km)')

    ax.set_ylabel('Speed of Sound (m/s)')

    ax.set_title('Speed of Sound vs Altitude', fontweight='bold')

    ax.set_xlim(0, alt_limit_km)

    ax.legend(loc='best', framealpha=0.95)


    plt.tight_layout()

    path = os.path.join(output_dir, '33_speed_of_sound.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_throttle_history(data: TrajectoryData, output_dir: str) -> str:

    """Engine throttle setting vs time."""

    fig, ax = plt.subplots()


    throttle_pct = data.throttle * 100.0

    ax.plot(data.time, throttle_pct, color='#e377c2', linewidth=2, label='Throttle')

    ax.fill_between(data.time, 0, throttle_pct, alpha=0.2, color='#e377c2')


    ax.plot(data.time, data.thrust_on * 100.0, 'k--', linewidth=1.0,

            alpha=0.4, label='Engine ON flag')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Throttle (%)')

    ax.set_title('Engine Throttle History', fontweight='bold')

    ax.legend(loc='upper right', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(-5, 110)


    plt.tight_layout()

    path = os.path.join(output_dir, '34_throttle_history.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_inertia_variation(data: TrajectoryData, output_dir: str) -> str:
    """Moment of inertia variation with mass (linear interpolation model)."""
    from rlv_sim.mass import compute_inertia_tensor

    inertia_hist = np.array([compute_inertia_tensor(m) for m in data.mass])
    ixx = inertia_hist[:, 0, 0]
    izz = inertia_hist[:, 2, 2]
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="35_inertia_variation.png",
            title="Principal Moments of Inertia vs Time",
            ylabel=r"Moment of Inertia ($\times 10^6$ kg·m²)",
            lines=(
                TimeSeriesLine(
                    ixx / 1e6,
                    label=r"$I_{xx} = I_{yy}$ (pitch/yaw)",
                    color="#1f77b4",
                    linewidth=2.0,
                ),
                TimeSeriesLine(
                    izz / 1e6,
                    label=r"$I_{zz}$ (roll)",
                    color="#ff7f0e",
                    linewidth=2.0,
                ),
            ),
            show_legend=True,
            legend_loc="upper right",
            ylim=(0, None),
        ),
    )


def plot_commanded_vs_actual_quat_w(data: TrajectoryData, output_dir: str) -> str:
    """Commanded vs Actual quaternion scalar component q_w."""
    return plot_cmd_vs_actual(
        data,
        output_dir,
        filename="36_quat_w_cmd_vs_actual.png",
        title=r"Commanded vs Actual Quaternion — Scalar $q_w$",
        ylabel=r"Quaternion $q_w$ (scalar)",
        cmd=data.commanded_quat[:, 0],
        actual=data.actual_quat[:, 0],
        cmd_label=r"$q_w^{cmd}$",
        actual_label=r"$q_w^{actual}$",
    )


def plot_commanded_vs_actual_quat_x(data: TrajectoryData, output_dir: str) -> str:
    """Commanded vs Actual quaternion q_x component."""
    return plot_cmd_vs_actual(
        data,
        output_dir,
        filename="37_quat_x_cmd_vs_actual.png",
        title=r"Commanded vs Actual Quaternion — $q_x$",
        ylabel=r"Quaternion $q_x$",
        cmd=data.commanded_quat[:, 1],
        actual=data.actual_quat[:, 1],
        cmd_label=r"$q_x^{cmd}$",
        actual_label=r"$q_x^{actual}$",
    )


def plot_commanded_vs_actual_quat_y(data: TrajectoryData, output_dir: str) -> str:
    """Commanded vs Actual quaternion q_y component."""
    return plot_cmd_vs_actual(
        data,
        output_dir,
        filename="38_quat_y_cmd_vs_actual.png",
        title=r"Commanded vs Actual Quaternion — $q_y$",
        ylabel=r"Quaternion $q_y$",
        cmd=data.commanded_quat[:, 2],
        actual=data.actual_quat[:, 2],
        cmd_label=r"$q_y^{cmd}$",
        actual_label=r"$q_y^{actual}$",
    )


def plot_commanded_vs_actual_quat_z(data: TrajectoryData, output_dir: str) -> str:
    """Commanded vs Actual quaternion q_z component."""
    return plot_cmd_vs_actual(
        data,
        output_dir,
        filename="39_quat_z_cmd_vs_actual.png",
        title=r"Commanded vs Actual Quaternion — $q_z$",
        ylabel=r"Quaternion $q_z$",
        cmd=data.commanded_quat[:, 3],
        actual=data.actual_quat[:, 3],
        cmd_label=r"$q_z^{cmd}$",
        actual_label=r"$q_z^{actual}$",
    )


def plot_downrange_distance(data: TrajectoryData, output_dir: str) -> str:
    """Downrange distance vs time."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="40_downrange_distance.png",
            title="Downrange Distance vs Time",
            ylabel="Downrange Distance (km)",
            lines=(
                TimeSeriesLine(
                    data.downrange,
                    label="Downrange (ECI projection)",
                    color="#8c564b",
                    linewidth=2.0,
                ),
            ),
            show_legend=True,
            legend_loc="upper left",
            ylim=(0, None),
        ),
    )


def plot_mass_flow_rate(data: TrajectoryData, output_dir: str) -> str:

    """Propellant mass flow rate vs time.


    Computed from mass derivative. Shows constant flow rate during powered

    flight and zero during coast.

    """

    fig, ax = plt.subplots()

    sep_idx = _find_stage_separation_index(data)

    meco_idx = _find_stage1_meco_index(data)

    s2_ign_idx = _find_stage2_ignition_index(data, meco_idx)


    dt = np.diff(data.time)

    dm = np.diff(data.mass)

    mdot_fd = np.full(len(data.time), np.nan)

    valid_dt = dt > 0

    valid_idx = np.where(valid_dt)[0] + 1

    mdot_fd[valid_idx] = -dm[valid_dt] / dt[valid_dt]

    if len(mdot_fd) > 1:

        mdot_fd[0] = mdot_fd[1]


    if sep_idx is not None and sep_idx < len(mdot_fd):

        mdot_fd[sep_idx] = np.nan


    spike_threshold = 10.0 * max(C.MASS_FLOW_RATE, C.STAGE2_MASS_FLOW_RATE)

    mdot_fd = np.where(mdot_fd > spike_threshold, np.nan, mdot_fd)

    mdot_fd = np.where(mdot_fd < -1e-6, np.nan, np.maximum(mdot_fd, 0.0))


    mdot = mdot_fd.copy()

    if data.throttle is not None:

        throttle = np.asarray(data.throttle)

        on_mask = _compute_engine_on_mask(data)

        s1_mask = on_mask.copy()

        s1_mask[meco_idx + 1:] = False

        s2_mask = on_mask.copy()

        if s2_ign_idx is not None:

            s2_mask[:s2_ign_idx] = False

        else:

            s2_mask[:] = False


        mdot[:] = np.nan

        mdot[s1_mask] = np.clip(throttle[s1_mask], 0.0, 1.0) * C.MASS_FLOW_RATE

        mdot[s2_mask] = np.clip(throttle[s2_mask], 0.0, 1.0) * C.STAGE2_MASS_FLOW_RATE


    on_mask = _compute_engine_on_mask(data)

    s1_mask = on_mask.copy()

    s1_mask[meco_idx + 1:] = False

    s2_mask = on_mask.copy()

    if s2_ign_idx is not None:

        s2_mask[:s2_ign_idx] = False

    else:

        s2_mask[:] = False


    mdot_s1 = np.where(s1_mask, mdot, np.nan)

    mdot_s2 = np.where(s2_mask, mdot, np.nan)

    ax.plot(data.time, mdot_s1, color='#bcbd22', linewidth=2, label='Mass Flow Rate')

    ax.plot(data.time, mdot_s2, color='#bcbd22', linewidth=2)

    ax.axhline(y=C.MASS_FLOW_RATE, color='red', linestyle='--', linewidth=1.5,

               label=f'Stage 1 nominal: {C.MASS_FLOW_RATE:.1f} kg/s')

    ax.axhline(y=C.STAGE2_MASS_FLOW_RATE, color='blue', linestyle=':', linewidth=1.3,

               label=f'Stage 2 nominal: {C.STAGE2_MASS_FLOW_RATE:.1f} kg/s')

    if sep_idx is not None:

        ax.axvline(x=data.time[sep_idx], color='gray', linestyle='-.', linewidth=1.0,

                   alpha=0.6, label='Stage separation')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel(r'$\dot{m}$ (kg/s)')

    ax.set_title('Propellant Mass Flow Rate', fontweight='bold')

    ax.legend(loc='upper right', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    finite_mdot = mdot[np.isfinite(mdot)]

    if finite_mdot.size > 0:

        ax.set_ylim(0, max(1.0, 1.2 * float(np.nanmax(finite_mdot))))

    else:

        ax.set_ylim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '41_mass_flow_rate.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_propellant_fraction(data: TrajectoryData, output_dir: str) -> str:

    """Propellant fraction remaining vs time.


    Shows the fraction of total propellant still on board.

    """

    fig, ax = plt.subplots()

    meco_idx = _find_stage1_meco_index(data)

    sep_idx = _find_stage_separation_index(data)

    n = len(data.time)


    s1_remaining = np.full(n, np.nan)

    m0 = float(data.mass[0])

    m_meco = float(data.mass[meco_idx])

    s1_burnable = max(m0 - m_meco, 1.0)

    s1_remaining[:meco_idx + 1] = np.clip(

        (data.mass[:meco_idx + 1] - m_meco) / s1_burnable * 100.0,

        0.0,

        100.0,

    )

    s1_end_idx = sep_idx if sep_idx is not None else (n - 1)

    if meco_idx + 1 <= s1_end_idx:

        s1_remaining[meco_idx + 1:s1_end_idx + 1] = 0.0

    if s1_end_idx + 1 < n:

        s1_remaining[s1_end_idx + 1:] = np.nan


    s2_remaining = np.full(n, np.nan)

    s2_start_idx = sep_idx if sep_idx is not None else _find_stage2_ignition_index(data, meco_idx)

    if s2_start_idx is not None and s2_start_idx < n:

        m_s2_start = float(data.mass[s2_start_idx])

        s2_burnable = max(m_s2_start - C.STAGE2_DRY_MASS, 1.0)

        s2_remaining[s2_start_idx:] = np.clip(

            (data.mass[s2_start_idx:] - C.STAGE2_DRY_MASS) / s2_burnable * 100.0,

            0.0,

            100.0,

        )


    ax.plot(data.time, s1_remaining, color='#7f7f7f', linewidth=2,

            label='Stage 1 Propellant Remaining')

    ax.fill_between(data.time, 0, s1_remaining, where=np.isfinite(s1_remaining),

                    alpha=0.15, color='#7f7f7f')

    if np.any(np.isfinite(s2_remaining)):

        ax.plot(data.time, s2_remaining, color='#1f77b4', linewidth=2,

                label='Stage 2 Propellant Remaining')

        ax.fill_between(data.time, 0, s2_remaining, where=np.isfinite(s2_remaining),

                        alpha=0.10, color='#1f77b4')

    if sep_idx is not None:

        ax.axvline(x=data.time[sep_idx], color='gray', linestyle='-.', linewidth=1.0,

                   alpha=0.6, label='Stage separation')

    ax.axhline(y=10, color='red', linestyle='--', linewidth=1.5,

                label='10% Reserve Warning')


    ax.set_xlabel('Time (s)')

    ax.set_ylabel('Propellant Remaining (%)')

    ax.set_title('Propellant Fraction Remaining', fontweight='bold')

    ax.legend(loc='upper right', framealpha=0.95)

    ax.set_xlim(0, data.time[-1])

    ax.set_ylim(0, 105)


    reserve_kg = C.STAGE1_LANDING_FUEL_RESERVE

    if np.any(np.isfinite(s2_remaining)):

        textstr = f'Final Stage 2 remaining: {s2_remaining[-1]:.1f}%'

    else:

        textstr = (f'At S1 MECO: {s1_remaining[meco_idx]:.1f}% ascent propellant\n'

                   f'(RTLS reserve {reserve_kg/1000:.0f} t retained â€” not counted above)')

    ax.text(0.98, 0.50, textstr, transform=ax.transAxes, fontsize=10,

            ha='right', va='center',

            bbox={'boxstyle': 'round', 'facecolor': 'lightyellow', 'alpha': 0.95})


    plt.tight_layout()

    path = os.path.join(output_dir, '42_propellant_fraction.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_drag_coefficient_vs_mach(data: TrajectoryData, output_dir: str) -> str:

    """Drag coefficient Cd vs Mach number (as experienced along trajectory).


    Shows the Mach-dependent Cd look-up table values, including the transonic

    drag rise â€” a critical phenomenon for launch vehicles.

    """

    fig, ax = plt.subplots()


    cd_trajectory = np.interp(data.mach_number, C.MACH_BREAKPOINTS, C.CD_VALUES)


    ax.plot(data.mach_number, cd_trajectory, 'o', markersize=1.5, alpha=0.3,

            color='#1f77b4', label='Trajectory Cd(M)')


    mach_fine = np.linspace(0, max(np.max(data.mach_number), 10), 500)

    cd_fine = np.interp(mach_fine, C.MACH_BREAKPOINTS, C.CD_VALUES)

    ax.plot(mach_fine, cd_fine, 'r-', linewidth=2, label='Cd(M) Look-up Table')


    ax.axvline(x=1.0, color='gray', linestyle=':', linewidth=1.2, alpha=0.6,

               label='M = 1')


    ax.set_xlabel('Mach Number')

    ax.set_ylabel(r'Drag Coefficient $C_D$')

    ax.set_title('Drag Coefficient vs Mach Number', fontweight='bold')

    ax.legend(loc='upper right', framealpha=0.95)

    ax.set_xlim(0, None)

    ax.set_ylim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '43_drag_coeff_vs_mach.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_energy_budget(data: TrajectoryData, output_dir: str) -> str:
    """Energy budget: gain in KE, PE, and total mechanical energy from liftoff."""
    r_mag = np.linalg.norm(data.position, axis=1)
    ke = 0.5 * data.velocity ** 2
    pe = -C.MU_EARTH / r_mag
    te = ke + pe
    dke = (ke - ke[0]) / 1e6
    dpe = (pe - pe[0]) / 1e6
    dte = (te - te[0]) / 1e6
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="44_energy_budget.png",
            title="Specific Mechanical Energy Budget (Gain from Liftoff)",
            ylabel="Specific Energy Gain (MJ/kg)",
            lines=(
                TimeSeriesLine(dke, label="ΔKinetic Energy", color="#d62728", linewidth=2.0),
                TimeSeriesLine(dpe, label="ΔPotential Energy", color="#1f77b4", linewidth=2.0),
                TimeSeriesLine(dte, label="ΔTotal Mech. Energy", color="#2ca02c", linewidth=2.5),
            ),
            hlines=(ZERO_HLINE,),
            show_legend=True,
            legend_loc="upper left",
        ),
    )


def plot_altitude_vs_velocity(data: TrajectoryData, output_dir: str) -> str:

    """Altitude vs velocity (state-space trajectory).


    This phase portrait shows the trajectory in altitude-velocity space,

    a common representation in astrodynamics for performance analysis.

    """

    fig, ax = plt.subplots()

    meco_idx = _find_stage1_meco_index(data)


    sc = ax.scatter(data.velocity, data.altitude, c=data.time, cmap='viridis',

                    s=3, alpha=0.8)

    plt.colorbar(sc, ax=ax, label='Time (s)')


    ax.scatter([data.velocity[0]], [data.altitude[0]], c='green', s=80,

               marker='o', zorder=5, label='Liftoff')

    ax.scatter([data.velocity[meco_idx]], [data.altitude[meco_idx]], c='red', s=80,

               marker='x', zorder=5,

               label=f'S1 MECO ({data.velocity[meco_idx]:.0f} m/s, {data.altitude[meco_idx]:.0f} km)')

    if meco_idx != len(data.time) - 1:

        ax.scatter([data.velocity[-1]], [data.altitude[-1]], c='darkorange', s=85,

                   marker='*', zorder=5,

                   label=f'Final ({data.velocity[-1]:.0f} m/s, {data.altitude[-1]:.0f} km)')


    ax.set_xlabel('Inertial Velocity (m/s)')

    ax.set_ylabel('Altitude (km)')

    ax.set_title('Altitude-Velocity State Space', fontweight='bold')

    ax.legend(loc='upper left', framealpha=0.95)

    ax.set_xlim(0, None)

    ax.set_ylim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '45_altitude_vs_velocity.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_gravity_loss(data: TrajectoryData, output_dir: str) -> str:
    """Cumulative gravity loss vs time."""
    r_mag = np.linalg.norm(data.position, axis=1)
    g_local = C.MU_EARTH / (r_mag ** 2)
    gamma_rad = np.radians(data.gamma_rel)
    gravity_loss_rate = g_local * np.sin(gamma_rad)
    cumulative_loss = np.zeros(len(data.time))
    for i in range(1, len(data.time)):
        dt = data.time[i] - data.time[i - 1]
        cumulative_loss[i] = cumulative_loss[i - 1] + 0.5 * (
            gravity_loss_rate[i] + gravity_loss_rate[i - 1]
        ) * dt
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="46_gravity_loss.png",
            title="Cumulative Gravity Loss",
            ylabel=r"Gravity Loss $\Delta V_{grav}$ (m/s)",
            lines=(
                TimeSeriesLine(
                    cumulative_loss,
                    label="Cumulative Gravity Loss",
                    color="#d62728",
                    linewidth=2.0,
                ),
            ),
            show_legend=True,
            legend_loc="upper left",
            ylim=(0, None),
        ),
    )


def plot_drag_loss(data: TrajectoryData, output_dir: str) -> str:
    """Cumulative drag loss vs time."""
    cd_traj = np.interp(data.mach_number, C.MACH_BREAKPOINTS, C.CD_VALUES)
    drag_force = 0.5 * data.density * cd_traj * C.REFERENCE_AREA * data.velocity_rel ** 2
    drag_decel = drag_force / data.mass
    cumulative_drag = np.zeros(len(data.time))
    for i in range(1, len(data.time)):
        dt = data.time[i] - data.time[i - 1]
        cumulative_drag[i] = cumulative_drag[i - 1] + 0.5 * (
            drag_decel[i] + drag_decel[i - 1]
        ) * dt
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="47_drag_loss.png",
            title="Cumulative Drag Loss",
            ylabel=r"Drag Loss $\Delta V_{drag}$ (m/s)",
            lines=(
                TimeSeriesLine(
                    cumulative_drag,
                    label="Cumulative Drag Loss",
                    color="#ff7f0e",
                    linewidth=2.0,
                ),
            ),
            show_legend=True,
            legend_loc="upper left",
            ylim=(0, None),
        ),
    )


def plot_delta_v_budget(data: TrajectoryData, output_dir: str) -> str:
    """Delta-V budget breakdown: ideal, gravity loss, drag loss, achieved."""
    meco_idx = _find_stage1_meco_index(data)
    sep_idx = _find_stage_separation_index(data)
    mass = np.maximum(np.asarray(data.mass), 1.0)
    ideal_dv = np.zeros(len(data.time))
    m0 = mass[0]
    m_meco = mass[meco_idx]
    ideal_dv[:meco_idx + 1] = C.ISP * C.G0 * np.log(m0 / mass[:meco_idx + 1])
    ideal_stage1_total = C.ISP * C.G0 * np.log(m0 / m_meco)
    if sep_idx is not None and sep_idx < len(data.time):
        if sep_idx > meco_idx + 1:
            ideal_dv[meco_idx + 1:sep_idx] = ideal_stage1_total
        m_s2_start = mass[sep_idx]
        ideal_dv[sep_idx:] = ideal_stage1_total + (
            C.STAGE2_ISP_VAC * C.G0 * np.log(m_s2_start / mass[sep_idx:])
        )
    elif meco_idx + 1 < len(data.time):
        ideal_dv[meco_idx + 1:] = ideal_stage1_total
    achieved_dv = data.velocity - data.velocity[0]
    r_mag = np.linalg.norm(data.position, axis=1)
    g_local = C.MU_EARTH / (r_mag ** 2)
    gamma_rad = np.radians(data.gamma_rel)
    grav_rate = g_local * np.sin(gamma_rad)
    grav_loss = np.zeros(len(data.time))
    for i in range(1, len(data.time)):
        dt = data.time[i] - data.time[i - 1]
        grav_loss[i] = grav_loss[i - 1] + 0.5 * (grav_rate[i] + grav_rate[i - 1]) * dt
    cd_traj = np.interp(data.mach_number, C.MACH_BREAKPOINTS, C.CD_VALUES)
    drag_force = 0.5 * data.density * cd_traj * C.REFERENCE_AREA * data.velocity_rel ** 2
    drag_decel = drag_force / data.mass
    drag_loss = np.zeros(len(data.time))
    for i in range(1, len(data.time)):
        dt = data.time[i] - data.time[i - 1]
        drag_loss[i] = drag_loss[i - 1] + 0.5 * (drag_decel[i] + drag_decel[i - 1]) * dt
    steering_loss = np.maximum(ideal_dv - achieved_dv - grav_loss - drag_loss, 0.0)
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="48_delta_v_budget.png",
            title=r"$\Delta V$ Budget Breakdown",
            ylabel=r"$\Delta V$ (m/s)",
            lines=(
                TimeSeriesLine(ideal_dv, label="Ideal (Tsiolkovsky)", fmt="k-", linewidth=2.5),
                TimeSeriesLine(achieved_dv, label="Achieved", fmt="b-", linewidth=2.0),
                TimeSeriesLine(grav_loss, label="Gravity Loss", color="red", linestyle="--", linewidth=1.5),
                TimeSeriesLine(drag_loss, label="Drag Loss", color="green", linestyle="--", linewidth=1.5),
                TimeSeriesLine(
                    steering_loss,
                    label="Steering Loss",
                    color="#ff7f0e",
                    linestyle="--",
                    linewidth=1.5,
                ),
            ),
            show_legend=True,
            legend_loc="upper left",
            ylim=(0, None),
        ),
    )


def plot_position_eci_x(data: TrajectoryData, output_dir: str) -> str:
    """ECI position X-component vs time."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="49_position_eci_x.png",
            title="ECI Position X-Component (Radial at Launch)",
            ylabel="$r_X$ (km)",
            lines=(TimeSeriesLine(data.position[:, 0] / 1000.0, color="#1f77b4"),),
        ),
    )


def plot_position_eci_y(data: TrajectoryData, output_dir: str) -> str:
    """ECI position Y-component vs time."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="50_position_eci_y.png",
            title="ECI Position Y-Component (East at Launch)",
            ylabel="$r_Y$ (km)",
            lines=(TimeSeriesLine(data.position[:, 1] / 1000.0, color="#ff7f0e"),),
        ),
    )


def plot_position_eci_z(data: TrajectoryData, output_dir: str) -> str:
    """ECI position Z-component vs time."""
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="51_position_eci_z.png",
            title="ECI Position Z-Component (North at Launch)",
            ylabel="$r_Z$ (km)",
            lines=(TimeSeriesLine(data.position[:, 2] / 1000.0, color="#2ca02c"),),
        ),
    )


def plot_natural_frequency(data: TrajectoryData, output_dir: str) -> str:

    """Control system natural frequency and damping ratio vs time.


    Uses the *scheduled* gains (Kp, Kd scaled proportionally to inertia)

    so that Ï‰n and Î¶ correctly show constant values â€” confirming that

    gain scheduling is working as designed.

    """

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    from rlv_sim.mass import compute_inertia_tensor


    inertia_hist = np.array([compute_inertia_tensor(m) for m in data.mass])

    Ixx = inertia_hist[:, 0, 0]


    gain_ratio = Ixx / C.IXX_FULL

    kp_eff = C.KP_ATTITUDE * gain_ratio

    kd_eff = C.KD_ATTITUDE * gain_ratio


    omega_n = np.sqrt(kp_eff / (2.0 * Ixx))

    zeta    = kd_eff / (2.0 * np.sqrt(kp_eff * Ixx / 2.0))


    omega_n_ref = np.sqrt(C.KP_ATTITUDE / (2.0 * Ixx))

    zeta_ref    = C.KD_ATTITUDE / (2.0 * np.sqrt(C.KP_ATTITUDE * Ixx / 2.0))


    ax1.plot(data.time, omega_n, color='#1f77b4', linewidth=2,

             label='Scheduled (actual)')

    ax1.plot(data.time, omega_n_ref, color='#aec7e8', linewidth=1.5,

             linestyle='--', label='Unscheduled (reference gains)')

    ax1.set_ylabel(r'$\omega_n$ (rad/s)')

    ax1.set_title('Control System Natural Frequency', fontweight='bold')

    ax1.legend(loc='best', fontsize=9, framealpha=0.95)


    ax2.plot(data.time, zeta, color='#d62728', linewidth=2,

             label='Scheduled (actual)')

    ax2.plot(data.time, zeta_ref, color='#f4a582', linewidth=1.5,

             linestyle='--', label='Unscheduled (reference gains)')

    ax2.axhline(y=1.0, color='gray', linestyle='--', linewidth=1.2,

                label='Critical damping')

    ax2.axhline(y=0.7, color='green', linestyle=':', linewidth=1.2,

                label='Design target (0.7)')

    ax2.set_xlabel('Time (s)')

    ax2.set_ylabel(r'Damping Ratio $\zeta$')

    ax2.set_title('Control System Damping Ratio', fontweight='bold')

    ax2.legend(loc='upper left', framealpha=0.95, fontsize=9)


    ax1.set_xlim(0, data.time[-1])

    ax2.set_xlim(0, data.time[-1])


    plt.tight_layout()

    path = os.path.join(output_dir, '52_control_bandwidth.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_geocentric_radius(data: TrajectoryData, output_dir: str) -> str:
    """Geocentric radius vs time."""
    r_mag = np.linalg.norm(data.position, axis=1) / 1000.0
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="53_geocentric_radius.png",
            title="Geocentric Radius vs Time",
            ylabel="Geocentric Radius (km)",
            lines=(TimeSeriesLine(r_mag, label="Geocentric Radius", color="#17becf", linewidth=2.0),),
            hlines=(
                HLine(
                    C.R_EARTH / 1000.0,
                    label=f"Earth Surface ({C.R_EARTH/1000:.0f} km)",
                    color="orange",
                    linestyle="--",
                    linewidth=1.5,
                ),
            ),
            show_legend=True,
            legend_loc="lower right",
        ),
    )


def plot_specific_angular_momentum(data: TrajectoryData, output_dir: str) -> str:
    """Specific angular momentum magnitude vs time."""
    h_vec = np.cross(data.position, data.velocity_vec)
    h_mag = np.linalg.norm(h_vec, axis=1) / 1e9
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="54_angular_momentum.png",
            title="Specific Angular Momentum",
            ylabel=r"Specific Angular Momentum ($\times 10^9$ m$^2$/s)",
            lines=(TimeSeriesLine(h_mag, label=r"$h = ||r \times v||$", color="#9467bd", linewidth=2.0),),
            show_legend=True,
            legend_loc="lower right",
        ),
    )


def plot_dynamic_pressure_vs_altitude(data: TrajectoryData, output_dir: str) -> str:

    """Dynamic pressure vs altitude (instead of time).


    Shows the Max-Q region in altitude space.

    """

    fig, ax = plt.subplots()


    q_kpa = data.dynamic_pressure / 1000.0

    ax.plot(data.altitude, q_kpa, color='#9467bd', linewidth=2,

            label='Dynamic Pressure')

    ax.axhline(y=C.MAX_DYNAMIC_PRESSURE / 1000.0, color='red', linestyle='--',

               linewidth=1.5, label=f'Limit ({C.MAX_DYNAMIC_PRESSURE/1000:.0f} kPa)')


    maxq_idx = np.argmax(q_kpa)

    ax.scatter([data.altitude[maxq_idx]], [q_kpa[maxq_idx]], c='red', s=100,

               marker='*', zorder=5,

               label=f'Max-Q: {q_kpa[maxq_idx]:.1f} kPa @ {data.altitude[maxq_idx]:.1f} km')


    ax.set_xlabel('Altitude (km)')

    ax.set_ylabel('Dynamic Pressure (kPa)')

    ax.set_title('Dynamic Pressure vs Altitude', fontweight='bold')

    ax.legend(loc='upper right', framealpha=0.95)

    ax.set_xlim(0, None)

    ax.set_ylim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '55_q_vs_altitude.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_effective_isp(data: TrajectoryData, output_dir: str) -> str:

    """Effective specific impulse vs altitude.


    ISP varies with ambient pressure: Isp_eff = T / (mdot * g0)

    where T = T_vac - (T_vac - T_sl) * (P_amb / P_sl).

    """

    fig, ax = plt.subplots()

    meco_idx = _find_stage1_meco_index(data)

    s2_start_idx = _find_stage2_ignition_index(data, meco_idx)

    on_mask = _compute_engine_on_mask(data)

    sample_idx = np.arange(len(data.time))


    T_sl = C.THRUST_MAGNITUDE

    T_vac = C.ISP_VAC * C.G0 * C.MASS_FLOW_RATE

    P_sl = C.ATM_P0


    T_eff_s1 = T_vac - (T_vac - T_sl) * np.clip(data.pressure / P_sl, 0, 1)

    Isp_stage1 = np.full(len(data.time), np.nan)

    Isp_stage2 = np.full(len(data.time), np.nan)


    if s2_start_idx is None:

        stage1_mask = on_mask

    else:

        stage1_mask = on_mask & (sample_idx < s2_start_idx)

    Isp_stage1[stage1_mask] = T_eff_s1[stage1_mask] / (C.MASS_FLOW_RATE * C.G0)


    if s2_start_idx is not None:

        stage2_mask = on_mask & (sample_idx >= s2_start_idx)

        Isp_stage2[stage2_mask] = C.STAGE2_ISP_VAC


    ax.plot(data.altitude, Isp_stage1, color='#e377c2', linewidth=2, label='Stage 1 effective Isp')

    if np.any(np.isfinite(Isp_stage2)):

        ax.plot(data.altitude, Isp_stage2, color='#8c564b', linewidth=2, label='Stage 2 Isp')

    ax.axhline(y=C.ISP, color='blue', linestyle='--', linewidth=1.2,

               label=f'Sea Level Isp = {C.ISP:.0f} s')

    ax.axhline(y=C.ISP_VAC, color='red', linestyle='--', linewidth=1.2,

               label=f'Stage 1 Vacuum Isp = {C.ISP_VAC:.0f} s')

    ax.axhline(y=C.STAGE2_ISP_VAC, color='purple', linestyle='-.', linewidth=1.2,

               label=f'Stage 2 Vacuum Isp = {C.STAGE2_ISP_VAC:.0f} s')


    ax.set_xlabel('Altitude (km)')

    ax.set_ylabel('Effective Isp (s)')

    ax.set_title('Effective Specific Impulse vs Altitude', fontweight='bold')

    ax.legend(loc='center right', framealpha=0.95)

    ax.set_xlim(0, None)


    plt.tight_layout()

    path = os.path.join(output_dir, '56_effective_isp.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


def plot_altitude_rate(data: TrajectoryData, output_dir: str) -> str:
    """Rate of altitude change (vertical velocity) vs time."""
    dt = np.diff(data.time)
    dh = np.diff(data.altitude) * 1000.0
    hdot = np.zeros(len(data.time))
    hdot[1:] = dh / dt
    hdot[0] = hdot[1]
    return plot_time_series(
        data,
        output_dir,
        TimeSeriesPlot(
            filename="57_altitude_rate.png",
            title="Altitude Rate of Change",
            ylabel=r"$\dot{h}$ (m/s)",
            lines=(TimeSeriesLine(hdot, label="Altitude Rate", color="#8c564b", linewidth=1.8),),
            hlines=(ZERO_HLINE,),
            show_legend=True,
            legend_loc="upper right",
        ),
    )


def plot_ground_track(data: TrajectoryData, output_dir: str) -> str:

    """Ground track projection (East vs North from launch site).


    Shows the 2D footprint of the trajectory projected onto Earth's surface.

    """

    fig, ax = plt.subplots()

    meco_idx = _find_stage1_meco_index(data)


    east, north = _compute_ground_track_enu(data)


    sc = ax.scatter(east, north, c=data.time, cmap='plasma', s=4, alpha=0.8)

    plt.colorbar(sc, ax=ax, label='Time (s)')


    ax.scatter([0], [0], c='green', s=100, marker='^', zorder=5, label='Launch Site')

    ax.scatter([east[meco_idx]], [north[meco_idx]], c='red', s=90, marker='x',

               zorder=5, label='S1 MECO projection')

    if meco_idx != len(data.time) - 1:

        ax.scatter([east[-1]], [north[-1]], c='darkorange', s=100, marker='*',

                   zorder=5, label='Final projection')


    ax.set_xlabel('East (km)')

    ax.set_ylabel('North (km)')

    ax.set_title('Ground Track Projection', fontweight='bold')

    ax.legend(loc='upper left', framealpha=0.95)


    north_center = (north.max() + north.min()) / 2.0

    north_half = max((north.max() - north.min()) / 2.0, 5.0)

    ax.set_ylim(north_center - north_half, north_center + north_half)


    plt.tight_layout()

    path = os.path.join(output_dir, '58_ground_track.png')

    fig.savefig(path, bbox_inches='tight', dpi=300)

    plt.close(fig)

    return path


