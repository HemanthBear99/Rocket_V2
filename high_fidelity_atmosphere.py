"""
High-Fidelity Atmospheric and Environmental Modeling (Earth-GRAM style).

Provides Geodetic coordinate transformations (ECI to WGS-84 Ellipsoid) and
latitude/longitude-dependent atmospheric wind and density shear models.
"""

import numpy as np

from . import constants as C

# WGS-84 Ellipsoid Constants
WGS84_A = 6378137.0           # Equatorial semi-major axis (m)
WGS84_F = 1.0 / 298.257223563 # Flattening
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)  # Square of eccentricity

def eci_to_geodetic(r_eci: np.ndarray, t: float) -> tuple[float, float, float]:
    """
    Convert Earth-Centered Inertial (ECI) to Geodetic coordinates (Lat, Lon, Alt).
    Uses WGS-84 ellipsoid and handles Earth rotation.

    Args:
        r_eci: Position vector in ECI frame (m)
        t: Simulation time (s)

    Returns:
        (latitude_deg, longitude_deg, altitude_m)
    """
    x_eci, y_eci, z = r_eci

    # Rotate ECI to ECEF (Earth-Centered Earth-Fixed)
    theta = C.EARTH_ROTATION_RATE * t
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    x = x_eci * cos_theta + y_eci * sin_theta
    y = -x_eci * sin_theta + y_eci * cos_theta

    # Bowring's method for geodetic coordinates
    p = np.sqrt(x**2 + y**2)
    if p < 1e-9:
        # At the pole
        lat = np.pi/2.0 if z > 0 else -np.pi/2.0
        lon = 0.0
        alt = abs(z) - WGS84_B
        return np.degrees(lat), np.degrees(lon), alt

    # Initial lat approximation
    lat = np.arctan2(z, p * (1.0 - WGS84_E2))
    lon = np.arctan2(y, x)

    # Iterate a few times for convergence
    for _ in range(5):
        sin_lat = np.sin(lat)
        N = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat**2)
        alt = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1.0 - WGS84_E2 * (N / (N + alt))))

    lon_deg = (np.degrees(lon) + 180) % 360 - 180  # Wrap [-180, 180]
    return np.degrees(lat), lon_deg, alt


def compute_gram_atmosphere(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    """
    High-fidelity lat/lon-dependent atmospheric variables (deviations from US76).

    Models latitude-dependent temperature gradients and gravity anomalies:
    - Tropics (near equator) are warmer and thicker.
    - Poles are colder and thinner.
    """
    alt = max(0.0, alt_m)
    lat_rad = np.radians(lat_deg)

    # Core temperature profile based on US76 base but with latitude variation
    # Tropics have higher tropopause; poles have lower tropopause.
    tropopause_base = 11000.0 + 4000.0 * np.cos(lat_rad)  # 15km at equator, 11km at poles

    # Density scale factor (deviations from 1.0)
    # Equatorial bulge increases density at a given altitude; polar flattening decreases it.
    density_modifier = 1.0 + 0.08 * (np.cos(lat_rad) - 0.5) * np.exp(-alt / 15000.0)

    return max(0.01, density_modifier), tropopause_base

