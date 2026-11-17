"""Validate and atomically persist user-defined depth regions."""

from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile

import yaml

from .bathymetry import SimGridProvider


MAX_SHOALS = 128


def default_settings_path(namespace: str) -> Path:
    """Keep saved settings separate for each boat namespace."""
    ros_home = Path(os.environ.get("ROS_HOME", "~/.ros")).expanduser()
    namespace_parts = [part for part in namespace.split("/") if part]
    return ros_home.joinpath(
        "enc_grounding_warning", *(namespace_parts or ["_root"]), "depth_settings.yaml"
    )


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


class DepthSettingsStore:
    """Save before applying, so a failed write cannot masquerade as a saved edit."""

    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser()

    def load(self, fallback: dict, config_path: str | None = None) -> dict:
        if not self.path.exists():
            return validate_settings(fallback)
        with self.path.open(encoding="utf-8") as stream:
            saved = yaml.safe_load(stream)
        if not isinstance(saved, dict) or saved.get("version") not in (1, 2):
            raise ValueError(f"Unsupported depth settings file: {self.path}")
        settings = validate_settings(saved.get("settings"))
        if saved["version"] == 1 and config_path is not None:
            # The old editor copied presets into its table. They belong to the
            # original map, so exclude them when migrating the editable list.
            with open(config_path, encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            config = config.get("depth_grid", config)
            presets = config.get("shoals", []) or []
            settings["shoals"] = [region for region in settings["shoals"] if region not in presets]
        settings["base_depth_m"] = fallback["base_depth_m"]
        return validate_settings(settings)

    def save(self, settings: dict) -> None:
        normalized = validate_settings(settings)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=self.path.parent,
                prefix=f".{self.path.name}.", delete=False,
            ) as stream:
                temporary_path = Path(stream.name)
                yaml.safe_dump(
                    {"version": 2, "settings": normalized}, stream, sort_keys=False
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_path, self.path)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()
