"""
Tabular Aerodynamic Database (Aero-Deck) loader and interpolator.

Provides multidimensional linear interpolation for aerodynamic coefficients
(Cd, Cl, Cm) as a function of Mach, alpha (AoA), beta (sideslip), and fin deflection.
"""

import json
import logging
import os

import numpy as np

logger = logging.getLogger(__name__)


class AeroDatabase:
    """
    Aero-Deck multi-dimensional aerodynamic coefficient lookup.
    Uses numpy for custom n-linear interpolation.
    """
    def __init__(self, filepath=None):
        self.axes = {}
        self.data = {}
        self.is_loaded = False
        if filepath and os.path.exists(filepath):
            self.load_database(filepath)

    def load_database(self, filepath):
        """Load tabular database from JSON."""
        try:
            with open(filepath, 'r') as f:
                db = json.load(f)

            # The JSON should define axes and matching grids
            # e.g., axes: {"mach": [...], "alpha": [...], "beta": [...], "fin": [...]}
            self.axes = {k: np.array(v, dtype=float) for k, v in db.get("axes", {}).items()}
            self.data = {k: np.array(v, dtype=float) for k, v in db.get("coefficients", {}).items()}
            self.is_loaded = True
        except Exception:
            logger.exception(
                "Failed to load Aero-Deck database %r; falling back to analytic aero model.",
                filepath,
            )
            self.is_loaded = False

    def interpolate_4d(self, mach, alpha_deg, beta_deg, fin_deg, coef_name):
        """
        Custom 4D linear interpolation.
        Returns the interpolated coefficient value.
        """
        if not self.is_loaded or coef_name not in self.data:
            return None

        # Axes names
        ax_names = ["mach", "alpha", "beta", "fin"]
        coords = [mach, alpha_deg, beta_deg, fin_deg]

        # We need to find the bounding indices and weights for each dimension
        indices = []
        weights = []
        for name, val in zip(ax_names, coords):
            ax = self.axes[name]
            if len(ax) < 2:
                # Singleton axis (e.g. no fin-deflection data): nothing to
                # interpolate between, so both hypercube corners on this
                # axis are the same single point.
                indices.append((0, 0))
                weights.append((1.0, 0.0))
                continue

            # Clip coordinate to axis limits
            val_clipped = np.clip(val, ax[0], ax[-1])
            idx = np.searchsorted(ax, val_clipped) - 1
            idx = max(0, min(idx, len(ax) - 2))

            x0 = ax[idx]
            x1 = ax[idx + 1]
            w1 = (val_clipped - x0) / (x1 - x0) if x1 > x0 else 0.0
            w0 = 1.0 - w1

            indices.append((idx, idx + 1))
            weights.append((w0, w1))

        # Perform 4D tensor interpolation over the hypercube of 16 points
        grid = self.data[coef_name]
        val_accum = 0.0
        for i in range(16):
            # Binary expansion to index coordinates
            idx_mach = indices[0][(i >> 3) & 1]
            idx_alpha = indices[1][(i >> 2) & 1]
            idx_beta = indices[2][(i >> 1) & 1]
            idx_fin = indices[3][i & 1]

            w_mach = weights[0][(i >> 3) & 1]
            w_alpha = weights[1][(i >> 2) & 1]
            w_beta = weights[2][(i >> 1) & 1]
            w_fin = weights[3][i & 1]

            point_val = grid[idx_mach, idx_alpha, idx_beta, idx_fin]
            val_accum += w_mach * w_alpha * w_beta * w_fin * point_val

        return val_accum


