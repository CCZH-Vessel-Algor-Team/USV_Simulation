"""Shared marker color helpers for grounding warning visualization."""

from .ukc_model import (
    RISK_CAUTION,
    RISK_DANGER,
    RISK_GROUNDED,
    RISK_SAFE,
    RISK_UNKNOWN,
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
