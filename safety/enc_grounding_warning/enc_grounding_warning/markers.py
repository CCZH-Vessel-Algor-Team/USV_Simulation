"""Marker helpers for grounding warnings and user-defined depth regions."""

import math

from .ukc_model import (
    RISK_CAUTION,
    RISK_DANGER,
    RISK_GROUNDED,
    RISK_SAFE,
    RISK_WARNING,
)


def risk_rgba(risk: int):
    """Return (r, g, b, a) for a risk level."""
    if risk == RISK_SAFE:
        return (0.0, 1.0, 0.0, 0.8)
    if risk == RISK_CAUTION:
        return (1.0, 1.0, 0.0, 0.8)
    if risk == RISK_WARNING:
        return (1.0, 0.6, 0.0, 0.9)
    if risk == RISK_DANGER:
        return (1.0, 0.0, 0.0, 0.9)
    if risk == RISK_GROUNDED:
        return (0.6, 0.0, 0.0, 1.0)
    return (0.5, 0.5, 0.5, 0.6)


def depth_region_markers(settings: dict, frame_id: str, stamp):
    """Draw only applied user regions as unfilled dark-red rings.

    One replaceable line batch avoids stale per-region IDs after deletion.
    An empty configuration explicitly deletes the batch, including for RViz
    subscribers that connect after the last region was removed.
    """
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "user_depth_regions"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.pose.orientation.w = 1.0
    marker.pose.position.z = 0.05
    marker.scale.x = 0.3
    marker.color.r = 0.55
    marker.color.g = 0.08
    marker.color.b = 0.08
    marker.color.a = 0.9
    regions = settings["shoals"] if settings["enabled"] else []
    marker.action = Marker.ADD if regions else Marker.DELETE
    segments = 96
    for region in regions:
        ring = [
            Point(
                x=region["x"] + region["radius"] * math.cos(2 * math.pi * i / segments),
                y=region["y"] + region["radius"] * math.sin(2 * math.pi * i / segments),
            )
            for i in range(segments)
        ]
        for i in range(segments):
            marker.points.extend((ring[i], ring[(i + 1) % segments]))
    return MarkerArray(markers=[marker])
