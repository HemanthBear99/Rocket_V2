"""Example landing-guidance plugin: textbook ZEM/ZEV with a constant-deceleration
vertical profile. Copy this file into your plugin folder, rename it, and edit.

ZEM/ZEV (Guo, Hawkins & Wie, 2013) with zero final velocity gives
a = 6 r / t_go^2 - 4 v / t_go for the horizontal channel. The vertical channel
stops the descent at the target touchdown speed by the ground:
a_v = g + (v_down^2 - v_td^2) / (2 h).
"""

import math

import numpy as np

NAME = "Example: ZEM/ZEV"
DESCRIPTION = (
    "Textbook zero-effort-miss / zero-effort-velocity divert with a "
    "constant-deceleration vertical stop. A starting point for your own law."
)


def landing_command(state):
    up = state.up
    h = max(state.altitude_m, 0.5)
    v_down = max(-state.vertical_speed_mps, 0.0)
    v_td = state.target_descent_rate_mps

    a_vertical = state.gravity_mps2 + (v_down ** 2 - v_td ** 2) / (2.0 * h)
    a_vertical = float(np.clip(a_vertical, 0.0, state.max_accel_mps2))

    t_go = max(state.time_to_go_s, 2.0)
    a_horizontal = (6.0 * state.position_error_m / t_go ** 2
                    - 4.0 * state.horizontal_velocity_mps / t_go)
    # Stay inside the tilt cone the sim enforces, so the vertical channel
    # keeps priority instead of being clipped by it.
    limit = math.tan(math.radians(state.tilt_limit_deg)) * max(a_vertical, state.gravity_mps2)
    norm = float(np.linalg.norm(a_horizontal))
    if norm > limit > 0.0:
        a_horizontal = a_horizontal * (limit / norm)
    return a_vertical * up + a_horizontal
