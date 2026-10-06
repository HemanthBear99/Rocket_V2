"""Structured return types for forces and control."""

from typing import Any, TypedDict

import numpy as np
from numpy.typing import NDArray

GuidanceOutput = dict[str, Any]


class ControlOutput(TypedDict, total=False):
    q_commanded: NDArray[np.float64]
    error_axis: NDArray[np.float64]
    error_angle: float
    error_degrees: float
    torque: NDArray[np.float64]
    torque_magnitude: float
    saturated: bool
    attitude_controller: str
    integral_torque: NDArray[np.float64]
    integral_torque_magnitude: float


class ForceBreakdown(TypedDict):
    gravity: NDArray[np.float64]
    thrust: NDArray[np.float64]
    drag: NDArray[np.float64]
    lift: NDArray[np.float64]
    grid_fin: NDArray[np.float64]
    total: NDArray[np.float64]
    gravity_magnitude: float
    thrust_magnitude: float
    drag_magnitude: float
    lift_magnitude: float
    grid_fin_magnitude: float
