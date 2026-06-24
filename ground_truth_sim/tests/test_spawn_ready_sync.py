"""Unit tests for spawn-ready motion sync helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from ground_truth_sim.spawn_ready_sync import (  # noqa: E402
    all_tracks_spawn_ready,
    normalize_motion_ready_topic,
    pending_spawn_track_ids,
)


class TestSpawnReadySync(unittest.TestCase):
    def test_normalize_motion_ready_topic(self) -> None:
        self.assertEqual(normalize_motion_ready_topic("/sim/gt_motion_ready"), "sim/gt_motion_ready")
        self.assertEqual(normalize_motion_ready_topic(""), "sim/gt_motion_ready")

    def test_all_tracks_spawn_ready(self) -> None:
        expected = {1, 2, 3}
        spawned = {1, 2, 3}
        drivers = {1: object(), 2: object(), 3: object()}
        self.assertTrue(all_tracks_spawn_ready(expected, spawned, drivers, set()))

    def test_pending_spawn_blocks_ready(self) -> None:
        expected = {1}
        spawned = {1}
        drivers = {1: object()}
        self.assertFalse(all_tracks_spawn_ready(expected, spawned, drivers, {1}))

    def test_missing_driver_blocks_ready(self) -> None:
        expected = {1, 2}
        spawned = {1, 2}
        drivers = {1: object()}
        self.assertFalse(all_tracks_spawn_ready(expected, spawned, drivers, set()))

    def test_empty_expected_not_ready(self) -> None:
        self.assertFalse(all_tracks_spawn_ready(set(), {1}, {1: object()}, set()))

    def test_pending_spawn_track_ids(self) -> None:
        futures = {1: object(), 2: object()}
        self.assertEqual(pending_spawn_track_ids(futures), {1, 2})


if __name__ == "__main__":
    unittest.main()
