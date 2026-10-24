"""Interpret external turning radii for the existing waypoint arrival test."""

import math


DEFAULT_ARRIVAL_RADIUS_M = 20.0


def waypoint_arrival_radius(point):
    """Allow zero turning radius without requiring an exact position match."""
    radius = point.get("routeRadius", DEFAULT_ARRIVAL_RADIUS_M)
    if (isinstance(radius, bool) or not isinstance(radius, (int, float))
            or not math.isfinite(radius) or radius < 0):
        raise ValueError("routeRadius must be a finite nonnegative number")
    return DEFAULT_ARRIVAL_RADIUS_M if radius == 0 else float(radius)
