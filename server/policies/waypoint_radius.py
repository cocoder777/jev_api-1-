"""Waypoint arrival tolerance, independent of the external turning radius."""


DEFAULT_ARRIVAL_RADIUS_M = 100.0


def waypoint_arrival_radius(point):
    """Use 100 m for every waypoint; routeRadius is an unused turning radius."""
    return DEFAULT_ARRIVAL_RADIUS_M
