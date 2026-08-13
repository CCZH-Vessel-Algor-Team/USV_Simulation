import os
import tempfile

import yaml

from enc_grounding_warning.bathymetry import SimGridProvider
from enc_grounding_warning.bathymetry import UNKNOWN_DEPTH


def _write_config(tmpdir, overrides=None):
    cfg = {
        "depth_grid": {
            "frame_id": "map",
            "origin_x": 0.0,
            "origin_y": 0.0,
            "resolution": 2.0,
            "width": 100,
            "height": 100,
            "mode": "flat",
            "depth_m": 3.0,
            "uncertainty_m": 0.1,
            "quality": 3,
            "shoals": [
                {"x": 50.0, "y": 50.0, "radius": 10.0, "depth_m": 0.5},
            ],
        }
    }
    if overrides:
        cfg["depth_grid"].update(overrides)
    path = os.path.join(tmpdir, "depth.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f)
    return path


def test_flat_depth_and_shoal():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = _write_config(tmpdir)
        provider = SimGridProvider(path)
        hit = provider.query_point(10.0, 10.0)
        assert hit is not None
        assert abs(hit["depth"] - 3.0) < 1e-6

        hit = provider.query_point(50.0, 50.0)
        assert hit is not None
        assert hit["depth"] < 1.0


def test_out_of_bounds_unknown():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = _write_config(tmpdir)
        provider = SimGridProvider(path)
        assert provider.query_point(1000.0, 1000.0) is None


def test_near_grid_shape():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = _write_config(tmpdir)
        provider = SimGridProvider(path)
        grid = provider.near_grid(50.0, 50.0, 100.0, 2.0)
        assert grid["width"] == 50
        assert grid["height"] == 50
        assert grid["depth"].shape == (50, 50)


def test_route_profile():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = _write_config(tmpdir)
        provider = SimGridProvider(path)
        samples = provider.route_profile(
            [(0.0, 0.0), (100.0, 100.0)], 5.0, 4.9, 2.0, 1.0
        )
        assert len(samples) > 20
        assert any(s["depth"] is not None and s["depth"] < 1.0 for s in samples)


def test_corridor_grid_only_publishes_route_vicinity():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = _write_config(tmpdir)
        provider = SimGridProvider(path)
        data = provider.corridor_grid(
            [(0.0, 0.0), (100.0, 0.0)],
            half_width=10.0,
            resolution=2.0,
        )
        assert data is not None
        depth = data["depth"]
        # Corridor center should be populated with the flat depth.
        assert depth[depth.shape[0] // 2, depth.shape[1] // 2] > 0.0
        # Cells far from the corridor remain unknown.
        assert (depth == UNKNOWN_DEPTH).any()
        # All known cells must be within half_width of the route.
        known = depth != UNKNOWN_DEPTH
        assert known.any()


def test_undulation_creates_spatial_depth_variation():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = _write_config(
            tmpdir,
            {
                "depth_m": 3.0,
                "undulation": {
                    "enabled": True,
                    "amplitude_m": 0.8,
                    "wavelength_m": 60.0,
                    "direction_deg": 30.0,
                },
                "shoals": [],
            },
        )
        provider = SimGridProvider(path)
        sample = provider.depth[::10, ::10]
        assert sample.min() < sample.max()
        assert sample.min() >= 2.0 - 1e-6
        assert sample.max() <= 4.0 + 1e-6
