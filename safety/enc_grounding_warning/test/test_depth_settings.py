"""Persistent regional depths must survive restarts and failed saves."""

import copy

import numpy as np
import pytest
import yaml

from enc_grounding_warning.depth_settings import (
    DepthSettingsStore,
    build_provider,
    default_settings_path,
    initial_settings,
    validate_settings,
)
from enc_grounding_warning.ukc_model import RISK_DANGER, RISK_SAFE, compute_ukc


@pytest.fixture
def grid_file(tmp_path):
    path = tmp_path / "grid.yaml"
    path.write_text(yaml.safe_dump({"depth_grid": {
        "origin_x": 0.0, "origin_y": 0.0, "resolution": 1.0,
        "width": 40, "height": 40, "depth_m": 3.0,
        "undulation": {"enabled": True, "amplitude_m": 0.4},
        "shoals": [{"x": 25.0, "y": 25.0, "radius": 4.0, "depth_m": 0.9}],
    }}))
    return str(path)


@pytest.fixture
def settings():
    return {"enabled": True, "base_depth_m": 3.0, "shoals": [
        {"x": 10.0, "y": 10.0, "radius": 6.0, "depth_m": 1.0},
        {"x": 12.0, "y": 10.0, "radius": 4.0, "depth_m": 0.7},
    ]}


def test_regions_preserve_original_background_and_overlap_at_shallower_depth(grid_file, settings):
    original = build_provider(grid_file, initial_settings(grid_file))
    provider = build_provider(grid_file, settings)
    assert provider.query_point(35.0, 35.0)["depth"] == original.query_point(35.0, 35.0)["depth"]
    assert provider.query_point(10.0, 10.0)["depth"] == pytest.approx(0.7)
    assert provider.query_point(5.0, 10.0)["depth"] == 1.0
    assert provider.query_point(25.0, 25.0)["depth"] == pytest.approx(0.9)
    params = {"static_draft_m": 0.5, "ukc_required_abs_m": 0.3}
    assert compute_ukc(provider.query_point(10.0, 10.0)["depth"], 0, params, 0)["risk"] == RISK_DANGER
    assert compute_ukc(provider.query_point(35.0, 35.0)["depth"], 0, params, 0)["risk"] == RISK_SAFE


def test_new_store_and_provider_restore_saved_regions(tmp_path, grid_file, settings):
    saved = tmp_path / "nested" / "settings.yaml"
    DepthSettingsStore(saved).save(settings)
    loaded = DepthSettingsStore(saved).load(initial_settings(grid_file))
    assert loaded == settings
    assert build_provider(grid_file, loaded).query_point(10.0, 10.0)["depth"] == pytest.approx(0.7)


def test_near_display_uses_region_map_coordinates_at_different_resolution(grid_file, settings):
    provider = build_provider(grid_file, settings)
    output = provider.near_grid(20.0, 20.0, 40.0, 2.0)
    for row, column in [(4, 4), (5, 5), (17, 17)]:
        x = output["origin_x"] + (column + 0.5) * output["resolution"]
        y = output["origin_y"] + (row + 0.5) * output["resolution"]
        assert output["depth"][row, column] == provider.query_point(x, y)["depth"]


def test_restore_original_grid_is_also_persistent(tmp_path, grid_file, settings):
    original = build_provider(grid_file, initial_settings(grid_file))
    settings["enabled"] = False
    store = DepthSettingsStore(tmp_path / "settings.yaml")
    store.save(settings)
    restored = build_provider(grid_file, store.load(initial_settings(grid_file)))
    np.testing.assert_array_equal(restored.depth, original.depth)


def test_restore_csv_after_custom_depths(tmp_path, settings):
    (tmp_path / "depth.csv").write_text("2,3\n4,5\n")
    path = tmp_path / "grid.yaml"
    path.write_text(yaml.safe_dump({"depth_grid": {
        "width": 2, "height": 2, "resolution": 1.0,
        "mode": "file", "data_file": "depth.csv",
    }}))
    settings["shoals"] = [{"x": 0.5, "y": 0.5, "radius": 0.6, "depth_m": 6.0}]
    np.testing.assert_array_equal(build_provider(str(path), settings).depth, [[6, 3], [4, 5]])
    settings["enabled"] = False
    np.testing.assert_array_equal(build_provider(str(path), settings).depth, [[2, 3], [4, 5]])


@pytest.mark.parametrize("field,value", [
    ("base_depth_m", -1), ("base_depth_m", float("nan")),
    ("base_depth_m", True), ("enabled", "true"),
    ("shoals", [{"x": 0, "y": 0, "radius": 0, "depth_m": 1}]),
    ("shoals", [{"x": float("inf"), "y": 0, "radius": 1, "depth_m": 1}]),
    ("shoals", [{"x": 0, "y": 0, "radius": 1, "depth_m": 11001}]),
    ("shoals", [{}] * 129),
])
def test_invalid_settings_rejected(settings, field, value):
    settings[field] = value
    with pytest.raises(ValueError):
        validate_settings(settings)


def test_failed_atomic_replace_preserves_last_saved_file(tmp_path, settings, monkeypatch):
    store = DepthSettingsStore(tmp_path / "settings.yaml")
    store.save(settings)
    before = store.path.read_bytes()
    replacement = copy.deepcopy(settings)
    replacement["base_depth_m"] = 6.0

    def reject(*args):
        raise OSError("disk write failed")

    monkeypatch.setattr("enc_grounding_warning.depth_settings.os.replace", reject)
    with pytest.raises(OSError, match="disk write failed"):
        store.save(replacement)
    assert store.path.read_bytes() == before
    assert list(tmp_path.iterdir()) == [store.path]


def test_namespace_paths_are_separate(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_HOME", str(tmp_path))
    assert default_settings_path("/usv_1") != default_settings_path("/usv_2")
    assert default_settings_path("/fleet/usv_1") == (
        tmp_path / "enc_grounding_warning/fleet/usv_1/depth_settings.yaml"
    )


def test_initial_list_excludes_presets_and_empty_list_restores_every_cell(grid_file, settings):
    initial = initial_settings(grid_file)
    assert initial["shoals"] == []
    original = build_provider(grid_file, initial)
    settings["shoals"] = [{"x": 25.0, "y": 25.0, "radius": 6.0, "depth_m": 6.0}]
    assert build_provider(grid_file, settings).query_point(25, 25)["depth"] == 6.0
    settings["shoals"] = []
    restored = build_provider(grid_file, settings)
    np.testing.assert_array_equal(restored.depth, original.depth)
    np.testing.assert_array_equal(restored.flags, original.flags)
    assert not validate_settings(settings)["enabled"]


def test_deleting_one_region_preserves_other_regions_and_restores_original(grid_file, settings):
    original = build_provider(grid_file, initial_settings(grid_file))
    settings["shoals"] = [
        {"x": 10.0, "y": 10.0, "radius": 6.0, "depth_m": 0.4},
        {"x": 12.0, "y": 10.0, "radius": 4.0, "depth_m": 0.7},
    ]
    assert build_provider(grid_file, settings).query_point(10, 10)["depth"] == pytest.approx(0.4)
    settings["shoals"].pop(0)
    remaining = build_provider(grid_file, settings)
    assert remaining.query_point(10, 10)["depth"] == pytest.approx(0.7)
    assert remaining.query_point(5, 10)["depth"] == original.query_point(5, 10)["depth"]
    settings["shoals"].clear()
    np.testing.assert_array_equal(build_provider(grid_file, settings).depth, original.depth)


def test_migrate_legacy_file_hides_original_presets(tmp_path, grid_file):
    original_preset = {"x": 25.0, "y": 25.0, "radius": 4.0, "depth_m": 0.9}
    user_region = {"x": 10.0, "y": 10.0, "radius": 3.0, "depth_m": 0.4}
    path = tmp_path / "legacy.yaml"
    path.write_text(yaml.safe_dump({"version": 1, "settings": {
        "enabled": True, "base_depth_m": 6.0, "shoals": [original_preset, user_region],
    }}))
    store = DepthSettingsStore(path)
    migrated = store.load(initial_settings(grid_file), grid_file)
    assert migrated == {"enabled": True, "base_depth_m": 3.0, "shoals": [user_region]}
    store.save(migrated)
    assert yaml.safe_load(path.read_text())["version"] == 2
    assert store.load(initial_settings(grid_file), grid_file) == migrated


def test_user_depth_overrides_unknown_csv_and_deletion_restores_unknown_flags(tmp_path):
    from enc_grounding_warning.bathymetry import FLAG_NO_DATA, UNKNOWN_DEPTH
    (tmp_path / "depth.csv").write_text(f"{UNKNOWN_DEPTH},3\n4,5\n")
    path = tmp_path / "grid.yaml"
    path.write_text(yaml.safe_dump({"depth_grid": {
        "width": 2, "height": 2, "resolution": 1.0, "mode": "file", "data_file": "depth.csv",
    }}))
    settings = initial_settings(str(path))
    original = build_provider(str(path), settings)
    settings.update(enabled=True, shoals=[{"x": 0.5, "y": 0.5, "radius": 0.6, "depth_m": 2.0}])
    changed = build_provider(str(path), settings)
    assert changed.query_point(0.5, 0.5)["depth"] == 2.0
    assert not (int(changed.flags[0, 0]) & FLAG_NO_DATA)
    settings["shoals"].clear()
    restored = build_provider(str(path), settings)
    np.testing.assert_array_equal(restored.depth, original.depth)
    np.testing.assert_array_equal(restored.flags, original.flags)
