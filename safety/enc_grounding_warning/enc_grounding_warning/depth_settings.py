"""Validate user-defined depth regions for the live editor."""

from __future__ import annotations

import math

import yaml

from .bathymetry import SimGridProvider


MAX_SHOALS = 128


def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_settings(data: dict) -> dict:
    """Return normalized settings, rejecting unusable water depths/regions."""
    if not isinstance(data, dict) or not isinstance(data.get("enabled"), bool):
        raise ValueError("enabled must be a boolean")
    base = _number(data.get("base_depth_m"), "base_depth_m")
    if not 0.0 <= base <= 11000.0:
        raise ValueError("base_depth_m must be between 0 and 11000 m")
    regions = data.get("shoals", [])
    if not isinstance(regions, list) or len(regions) > MAX_SHOALS:
        raise ValueError(f"shoals must be a list with at most {MAX_SHOALS} regions")
    shoals = []
    for index, region in enumerate(regions):
        if not isinstance(region, dict):
            raise ValueError(f"shoals[{index}] must be an object")
        shoal = {
            name: _number(region.get(name), f"shoals[{index}].{name}")
            for name in ("x", "y", "radius", "depth_m")
        }
        if shoal["radius"] <= 0.0:
            raise ValueError(f"shoals[{index}].radius must be greater than 0")
        if not 0.0 <= shoal["depth_m"] <= 11000.0:
            raise ValueError(f"shoals[{index}].depth_m must be between 0 and 11000 m")
        shoals.append(shoal)
    enabled = data["enabled"] and bool(shoals)
    return {"enabled": enabled, "base_depth_m": base, "shoals": shoals if enabled else []}


def initial_settings(config_path: str) -> dict:
    """The original map is a background, never a list of editable user regions."""
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    config = config.get("depth_grid", config)
    flat = str(config.get("mode", "flat")).lower() != "file"
    return validate_settings({
        "enabled": False,
        "base_depth_m": config.get("depth_m", 3.0) if flat else 3.0,
        "shoals": [],
    })


def build_provider(config_path: str, settings: dict) -> SimGridProvider:
    settings = validate_settings(settings)
    provider = SimGridProvider(config_path)
    if settings["enabled"]:
        provider.apply_regions(settings["shoals"])
    return provider
