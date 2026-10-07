"""
RLV Phase-I Ascent Simulation - Reference Frame Transformations

This module implements quaternion operations and reference frame transformations.
All orientation representation uses quaternions exclusively (no Euler angles).

Quaternion Convention: [w, x, y, z] where w is the scalar component.
"""

import numpy as np

from . import constants as C
from .utils import cross3, vec_norm


def quaternion_normalize(q: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion to unit length.

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        Normalized quaternion
    """
    norm = vec_norm(q)
    if norm < C.ZERO_TOLERANCE:

        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions: q1 * q2

    Args:
        q1: First quaternion [w, x, y, z]
        q2: Second quaternion [w, x, y, z]

    Returns:
        Product quaternion [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """
    Compute the conjugate of a quaternion.

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        Conjugate quaternion [w, -x, -y, -z]
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quaternion_inverse(q: np.ndarray) -> np.ndarray:
    """
    Compute the inverse of a quaternion.
    For unit quaternions, inverse equals conjugate.

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        Inverse quaternion
    """
    conj = quaternion_conjugate(q)
    norm_sq = np.dot(q, q)
    if norm_sq < C.ZERO_TOLERANCE:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return conj / norm_sq


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a rotation matrix R(q).

    The rotation matrix transforms vectors from body frame to inertial frame:
    v_inertial = R(q) @ v_body

    Args:
        q: Unit quaternion [w, x, y, z]

    Returns:
        3x3 rotation matrix
    """
    w, x, y, z = quaternion_normalize(q)

    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)]
    ])


def omega_matrix(omega: np.ndarray) -> np.ndarray:
    """
    Construct the Omega matrix for quaternion kinematics.

    The quaternion derivative is: q_dot = 0.5 * Omega(omega) @ q

    Args:
        omega: Angular velocity in body frame [wx, wy, wz] (rad/s)

    Returns:
        4x4 Omega matrix
    """
    wx, wy, wz = omega

    return np.array([
        [0.0, -wx, -wy, -wz],
        [wx,  0.0,  wz, -wy],
        [wy, -wz,  0.0,  wx],
        [wz,  wy, -wx,  0.0]
    ])


def quaternion_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """
    Compute the quaternion time derivative.

    q_dot = 0.5 * Omega(omega) @ q

    Args:
        q: Current quaternion [w, x, y, z]
        omega: Angular velocity in body frame (rad/s)

    Returns:
        Quaternion derivative [w_dot, x_dot, y_dot, z_dot]
    """
    Omega = omega_matrix(omega)
    return 0.5 * Omega @ q


def rotate_vector_by_quaternion(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """
    Rotate a vector by a quaternion.

    Transforms v from body frame to inertial frame.

    Args:
        v: Vector to rotate [3]
        q: Quaternion [w, x, y, z]

    Returns:
        Rotated vector [3]
    """
    R = quaternion_to_rotation_matrix(q)
    return R @ v


def direction_to_quaternion(direction: np.ndarray,
                            reference: np.ndarray = None) -> np.ndarray:
    """
    Compute a quaternion that rotates the reference direction to the target direction.

    Args:
        direction: Target direction in inertial frame (will be normalized)
        reference: Reference direction in body frame (default: +Z = [0, 0, 1])

    Returns:
        Quaternion [w, x, y, z] that transforms body reference to inertial target
    """

    if reference is None:
        reference = np.array([0.0, 0.0, 1.0])


    d_norm = vec_norm(direction)
    if d_norm < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])


    d = direction / d_norm
    r_norm = vec_norm(reference)
    if r_norm < C.ZERO_TOLERANCE:
        raise ValueError("reference direction must be non-zero")
    r = reference / r_norm


    dot = np.clip(np.dot(r, d), -1.0, 1.0)

    if 1.0 + dot < 1e-12:  # exactly opposite: any perpendicular axis works

        perp = np.array([1, 0, 0]) if abs(r[0]) < 0.9 else np.array([0, 1, 0])
        axis = cross3(r, perp)
        axis = axis / vec_norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])


    # Half-way quaternion: [1 + r.d, r x d] normalised is the shortest-arc
    # rotation and stays accurate for arbitrarily small angles (the old
    # explicit-angle form snapped anything within ~0.8 deg to identity).
    q = np.empty(4)
    q[0] = 1.0 + dot
    q[1:] = cross3(r, d)
    return q / vec_norm(q)


def quaternion_error(q_current: np.ndarray, q_desired: np.ndarray) -> np.ndarray:
    """
    Compute the body-frame quaternion error between current and desired
    orientations.

    q_error = q_current^(-1) * q_desired

    This is the rotation, expressed in the body frame, needed to go from the
    current orientation to the desired one.

    Args:
        q_current: Current quaternion [w, x, y, z]
        q_desired: Desired quaternion [w, x, y, z]

    Returns:
        Error quaternion [w, x, y, z]
    """
    q_inv = quaternion_inverse(q_current)
    q_err = quaternion_multiply(q_inv, q_desired)


    if q_err[0] < 0:
        q_err = -q_err

    return q_err


def quaternion_to_euler_zyx(q: np.ndarray, degrees: bool = False) -> np.ndarray:
    """
    Convert a body-to-inertial quaternion to ZYX Euler angles.

    Returns roll, pitch, yaw using the aerospace-friendly intrinsic ZYX
    convention.  The simulation still uses quaternions internally; this helper
    exists only for plotting and thesis reporting.
    """
    w, x, y, z = quaternion_normalize(q)

    roll = np.arctan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sin_pitch, -1.0, 1.0))
    yaw = np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )

    angles = np.array([roll, pitch, yaw], dtype=float)
    return np.degrees(angles) if degrees else angles
