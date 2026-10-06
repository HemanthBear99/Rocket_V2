"""
Engine gimbal actuator dynamics (second-order rate-limited slewing).
"""

from dataclasses import dataclass

import numpy as np

from . import constants as C
from .utils import cross3, vec_norm


@dataclass
class ActuatorState:
    """Tracks current thrust direction for gimbal dynamics."""
    thrust_dir: np.ndarray = None
    throttle: float = 0.0

    def __post_init__(self):
        if self.thrust_dir is None:
            self.thrust_dir = np.array([1.0, 0.0, 0.0], dtype=float)                       
        else:
            self.thrust_dir = np.asarray(self.thrust_dir, dtype=float)
        self.throttle = float(np.clip(self.throttle, 0.0, 1.0))


def _limit_rotation(current: np.ndarray, desired: np.ndarray, max_rate: float, dt: float) -> np.ndarray:
    """Limit change in direction to respect max gimbal rate (rad/s)."""
    cur_n = current / (vec_norm(current) + 1e-12)
    des_n = desired / (vec_norm(desired) + 1e-12)
    dot = np.clip(np.dot(cur_n, des_n), -1.0, 1.0)
    angle = np.arccos(dot)
    if angle <= max_rate * dt:
        return des_n
                                               
    axis = cross3(cur_n, des_n)
    axis_norm = vec_norm(axis)
    if axis_norm < 1e-9:
        if dot >= 0.0:
                                                                               
                                       
            return des_n
                                                                              
                                                 
        perp = np.array([1.0, 0.0, 0.0])
        if abs(float(cur_n[0])) > 0.9:
            perp = np.array([0.0, 1.0, 0.0])
        axis = cross3(cur_n, perp)
        axis_norm = vec_norm(axis)
    axis /= axis_norm
    theta = max_rate * dt
                        
    return (
        cur_n * np.cos(theta)
        + cross3(axis, cur_n) * np.sin(theta)
        + axis * np.dot(axis, cur_n) * (1 - np.cos(theta))
    )


def update_actuator(state: ActuatorState, desired_dir: np.ndarray, dt: float) -> ActuatorState:
    """
    Advance gimbal actuator toward desired thrust direction with rate limit.
    """
    limited = _limit_rotation(state.thrust_dir, desired_dir, C.MAX_GIMBAL_RATE, dt)
    return ActuatorState(thrust_dir=limited, throttle=state.throttle)
