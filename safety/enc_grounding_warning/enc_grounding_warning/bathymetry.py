"""Depth grid provider abstraction for the grounding warning stack.

The logical interface supports an ENC-backed provider in the future; the
simulation implementation reads a depth grid matrix from YAML + CSV or builds
one from a flat depth plus shoals.
"""

from __future__ import annotations

import math
import os

import numpy as np
import yaml


UNKNOWN_DEPTH = -9999.0

FLAG_LAND = 1
FLAG_DRYING = 2
FLAG_NO_DATA = 4
FLAG_HAZARD_UNKNOWN_DEPTH = 8

QUALITY_UNKNOWN = 0
QUALITY_A1 = 1
QUALITY_A2 = 2
QUALITY_B = 3
QUALITY_C = 4
QUALITY_D = 5
QUALITY_U = 6


class BathymetryProvider:
    """Interface consumed by UKC / grounding warning logic."""

    def query_point(self, x: float, y: float):
        raise NotImplementedError

    def query_footprint(
        self,
        x: float,
        y: float,
        yaw: float,
        half_length: float,
        half_width: float,
        margin: float = 0.0,
    ):
        raise NotImplementedError

    def near_grid(
        self,
        center_x: float,
        center_y: float,
        extent_m: float,
        resolution: float,
    ):
        raise NotImplementedError

    def corridor_grid(
        self,
        points,
        half_width: float,
        resolution: float,
    ):
        raise NotImplementedError

    def route_profile(
        self,
        points,
        sample_ds: float,
        half_length: float,
        half_width: float,
        margin: float = 0.0,
    ):
        raise NotImplementedError


class SimGridProvider(BathymetryProvider):
    """Provider backed by an in-memory depth matrix (simulation)."""

    def __init__(self, config_path: str):
        self.config_path = os.path.abspath(config_path)
        with open(self.config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        cfg = cfg.get("depth_grid", cfg)

        self.frame_id = str(cfg.get("frame_id", "map"))
        self.origin_x = float(cfg.get("origin_x", 0.0))
        self.origin_y = float(cfg.get("origin_y", 0.0))
        self.resolution = float(cfg.get("resolution", 5.0))
        self.width = int(cfg.get("width", 1))
        self.height = int(cfg.get("height", 1))
        self.default_uncertainty = float(cfg.get("uncertainty_m", 0.1))
        self.default_quality = int(cfg.get("quality", QUALITY_B))

        mode = str(cfg.get("mode", "flat")).lower()
        if mode == "file":
            data_file = str(cfg.get("data_file", "")).strip()
            if not data_file:
                raise ValueError("sim_depth_grid mode=file requires data_file")
            if not os.path.isabs(data_file):
                data_file = os.path.join(os.path.dirname(self.config_path), data_file)
            self.depth = self._load_csv(data_file)
        else:
            self.depth = self._build_flat(cfg)

        self.depth = self.depth.astype(np.float32)
        self.uncertainty = np.full(
            self.depth.shape, self.default_uncertainty, dtype=np.float32
        )
        self.quality = np.full(self.depth.shape, self.default_quality, dtype=np.uint8)
        self.flags = np.zeros(self.depth.shape, dtype=np.uint8)

        self.flags[self.depth < 0.0] |= FLAG_DRYING
        self.flags[self.depth <= UNKNOWN_DEPTH / 2.0] |= FLAG_NO_DATA

    def _load_csv(self, path: str) -> np.ndarray:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"depth grid CSV not found: {path}")
        raw = np.genfromtxt(
            path, delimiter=",", dtype=np.float64, filling_values=UNKNOWN_DEPTH
        )
        if raw.shape != (self.height, self.width):
            raise ValueError(
                f"depth CSV shape {raw.shape} != configured ({self.height}, {self.width})"
            )
        raw[np.isnan(raw)] = UNKNOWN_DEPTH
        return raw.astype(np.float32)

    def _build_flat(self, cfg: dict) -> np.ndarray:
        depth = np.full(
            (self.height, self.width),
            float(cfg.get("depth_m", 0.0)),
            dtype=np.float64,
        )

        undulation = cfg.get("undulation") or {}
        if undulation.get("enabled", False):
            amplitude = float(undulation.get("amplitude_m", 0.0))
            wavelength = float(undulation.get("wavelength_m", 100.0))
            direction_deg = float(undulation.get("direction_deg", 0.0))
            theta = math.radians(direction_deg)
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)

            ix = np.arange(self.width, dtype=np.float64)
            iy = np.arange(self.height, dtype=np.float64)
            gx = self.origin_x + (ix + 0.5) * self.resolution
            gy = self.origin_y + (iy + 0.5) * self.resolution
            gx2, gy2 = np.meshgrid(gx, gy)
            phase = 2.0 * math.pi * (gx2 * cos_t + gy2 * sin_t) / wavelength
            depth += amplitude * np.sin(phase)
            np.maximum(depth, 0.1, out=depth)

        for shoal in cfg.get("shoals", []) or []:
            sx = float(shoal["x"])
            sy = float(shoal["y"])
            radius = float(shoal.get("radius", 10.0))
            sdepth = float(shoal["depth_m"])
            ix0 = int(math.floor((sx - radius - self.origin_x) / self.resolution))
            iy0 = int(math.floor((sy - radius - self.origin_y) / self.resolution))
            ix1 = int(math.ceil((sx + radius - self.origin_x) / self.resolution))
            iy1 = int(math.ceil((sy + radius - self.origin_y) / self.resolution))
            for iy in range(max(0, iy0), min(self.height, iy1 + 1)):
                gy = self.origin_y + (iy + 0.5) * self.resolution
                for ix in range(max(0, ix0), min(self.width, ix1 + 1)):
                    gx = self.origin_x + (ix + 0.5) * self.resolution
                    if (gx - sx) ** 2 + (gy - sy) ** 2 <= radius * radius:
                        depth[iy, ix] = min(depth[iy, ix], sdepth)
        return depth

    def _to_index(self, x: float, y: float):
        ix = int(math.floor((x - self.origin_x) / self.resolution))
        iy = int(math.floor((y - self.origin_y) / self.resolution))
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return None
        return ix, iy

    def query_point(self, x: float, y: float):
        idx = self._to_index(x, y)
        if idx is None:
            return None
        ix, iy = idx
        depth = float(self.depth[iy, ix])
        if depth <= UNKNOWN_DEPTH / 2.0:
            return None
        return {
            "depth": depth,
            "uncertainty": float(self.uncertainty[iy, ix]),
            "quality": int(self.quality[iy, ix]),
            "flags": int(self.flags[iy, ix]),
        }

    def query_footprint(
        self,
        x: float,
        y: float,
        yaw: float,
        half_length: float,
        half_width: float,
        margin: float = 0.0,
    ):
        if yaw is None:
            yaw = 0.0
        hx = half_length / 2.0 + margin
        hy = half_width / 2.0 + margin
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)

        corners = [
            (x + hx * math.cos(yaw) - hy * math.sin(yaw),
             y + hx * math.sin(yaw) + hy * math.cos(yaw)),
            (x + hx * math.cos(yaw) + hy * math.sin(yaw),
             y + hx * math.sin(yaw) - hy * math.cos(yaw)),
            (x - hx * math.cos(yaw) + hy * math.sin(yaw),
             y - hx * math.sin(yaw) - hy * math.cos(yaw)),
            (x - hx * math.cos(yaw) - hy * math.sin(yaw),
             y - hx * math.sin(yaw) + hy * math.cos(yaw)),
        ]
        min_x = min(p[0] for p in corners)
        max_x = max(p[0] for p in corners)
        min_y = min(p[1] for p in corners)
        max_y = max(p[1] for p in corners)

        ix0 = int(math.floor((min_x - self.origin_x) / self.resolution))
        iy0 = int(math.floor((min_y - self.origin_y) / self.resolution))
        ix1 = int(math.ceil((max_x - self.origin_x) / self.resolution))
        iy1 = int(math.ceil((max_y - self.origin_y) / self.resolution))

        cell_diag = math.hypot(self.resolution, self.resolution) / 2.0
        best = None
        unknown_inside = False
        flags = 0
        worst_quality = 0
        worst_uncertainty = 0.0

        for iy in range(max(0, iy0), min(self.height, iy1 + 1)):
            gy = self.origin_y + (iy + 0.5) * self.resolution
            for ix in range(max(0, ix0), min(self.width, ix1 + 1)):
                gx = self.origin_x + (ix + 0.5) * self.resolution
                dx = gx - x
                dy = gy - y
                u = dx * cos_y - dy * sin_y
                v = dx * sin_y + dy * cos_y
                if abs(u) > hx + cell_diag or abs(v) > hy + cell_diag:
                    continue
                depth = float(self.depth[iy, ix])
                cell_flags = int(self.flags[iy, ix])
                flags |= cell_flags
                worst_quality = max(worst_quality, int(self.quality[iy, ix]))
                worst_uncertainty = max(
                    worst_uncertainty, float(self.uncertainty[iy, ix])
                )
                if depth <= UNKNOWN_DEPTH / 2.0:
                    unknown_inside = True
                    continue
                if best is None or depth < best:
                    best = depth

        if best is None:
            return None
        return {
            "depth": best,
            "uncertainty": worst_uncertainty,
            "quality": worst_quality,
            "flags": flags | (FLAG_NO_DATA if unknown_inside else 0),
        }

    def near_grid(
        self,
        center_x: float,
        center_y: float,
        extent_m: float,
        resolution: float,
    ):
        half = extent_m / 2.0
        start_x = center_x - half
        start_y = center_y - half
        width = max(1, int(round(extent_m / resolution)))
        height = max(1, int(round(extent_m / resolution)))

        gx0 = int(math.floor((start_x - self.origin_x) / self.resolution))
        gy0 = int(math.floor((start_y - self.origin_y) / self.resolution))
        gxs = gx0 + np.arange(width, dtype=np.int64)
        gys = gy0 + np.arange(height, dtype=np.int64)
        valid_x = (gxs >= 0) & (gxs < self.width)
        valid_y = (gys >= 0) & (gys < self.height)

        depth = np.full((height, width), UNKNOWN_DEPTH, dtype=np.float32)
        uncertainty = np.zeros((height, width), dtype=np.float32)
        quality = np.zeros((height, width), dtype=np.uint8)
        flags = np.full((height, width), FLAG_NO_DATA, dtype=np.uint8)

        ix_global = gxs[valid_x]
        iy_global = gys[valid_y]
        local_x = np.nonzero(valid_x)[0]
        local_y = np.nonzero(valid_y)[0]
        sub_depth = self.depth[np.ix_(iy_global, ix_global)]
        depth[np.ix_(local_y, local_x)] = sub_depth
        uncertainty[np.ix_(local_y, local_x)] = self.uncertainty[
            np.ix_(iy_global, ix_global)
        ]
        quality[np.ix_(local_y, local_x)] = self.quality[np.ix_(iy_global, ix_global)]
        flags[np.ix_(local_y, local_x)] = self.flags[np.ix_(iy_global, ix_global)]

        return {
            "origin_x": start_x,
            "origin_y": start_y,
            "resolution": resolution,
            "width": width,
            "height": height,
            "depth": depth,
            "uncertainty": uncertainty,
            "quality": quality,
            "flags": flags,
        }

    @staticmethod
    def _dist_point_to_segment(px, py, ax, ay, bx, by):
        dx = bx - ax
        dy = by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq <= 1e-12:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
        t = min(1.0, max(0.0, t))
        cx = ax + t * dx
        cy = ay + t * dy
        return math.hypot(px - cx, py - cy)

    def corridor_grid(
        self,
        points,
        half_width: float,
        resolution: float,
    ):
        """Rasterize only cells within ``half_width`` of the route polyline."""
        pts = [(float(p[0]), float(p[1])) for p in points]
        if len(pts) < 2:
            return None

        min_x = min(p[0] for p in pts) - half_width
        max_x = max(p[0] for p in pts) + half_width
        min_y = min(p[1] for p in pts) - half_width
        max_y = max(p[1] for p in pts) + half_width

        width = max(1, int(math.ceil((max_x - min_x) / resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / resolution)))

        depth = np.full((height, width), UNKNOWN_DEPTH, dtype=np.float32)
        uncertainty = np.zeros((height, width), dtype=np.float32)
        quality = np.zeros((height, width), dtype=np.uint8)
        flags = np.full((height, width), FLAG_NO_DATA, dtype=np.uint8)

        segments = [
            (pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
            for i in range(len(pts) - 1)
        ]

        for iy in range(height):
            gy = min_y + (iy + 0.5) * resolution
            for ix in range(width):
                gx = min_x + (ix + 0.5) * resolution
                dist = min(
                    self._dist_point_to_segment(gx, gy, ax, ay, bx, by)
                    for ax, ay, bx, by in segments
                )
                if dist > half_width:
                    continue
                idx = self._to_index(gx, gy)
                if idx is None:
                    continue
                ix_src, iy_src = idx
                depth[iy, ix] = self.depth[iy_src, ix_src]
                uncertainty[iy, ix] = self.uncertainty[iy_src, ix_src]
                quality[iy, ix] = self.quality[iy_src, ix_src]
                flags[iy, ix] = self.flags[iy_src, ix_src]

        return {
            "origin_x": min_x,
            "origin_y": min_y,
            "resolution": resolution,
            "width": width,
            "height": height,
            "depth": depth,
            "uncertainty": uncertainty,
            "quality": quality,
            "flags": flags,
        }

    def route_profile(
        self,
        points,
        sample_ds: float,
        half_length: float,
        half_width: float,
        margin: float = 0.0,
    ):
        pts = [(float(p[0]), float(p[1])) for p in points]
        if len(pts) < 2:
            return []

        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                            pts[i][1] - pts[i - 1][1]))
        total = cum[-1]
        if total < 1e-6:
            return []

        n = max(2, int(math.ceil(total / sample_ds)) + 1)
        samples = []
        ptr = 0
        for k in range(n):
            s = min(float(k) * sample_ds, total)
            while ptr + 1 < len(pts) and cum[ptr + 1] < s:
                ptr += 1
            seg_start = cum[ptr]
            seg_len = cum[ptr + 1] - cum[ptr]
            frac = 0.0 if seg_len <= 1e-9 else (s - seg_start) / seg_len
            x = pts[ptr][0] + frac * (pts[ptr + 1][0] - pts[ptr][0])
            y = pts[ptr][1] + frac * (pts[ptr + 1][1] - pts[ptr][1])
            yaw = math.atan2(pts[ptr + 1][1] - pts[ptr][1],
                             pts[ptr + 1][0] - pts[ptr][0])
            hit = self.query_footprint(x, y, yaw, half_length, half_width, margin)
            samples.append(
                {
                    "distance": s,
                    "x": x,
                    "y": y,
                    "route_index": ptr,
                    "depth": hit["depth"] if hit else None,
                    "uncertainty": hit["uncertainty"] if hit else 0.0,
                    "quality": hit["quality"] if hit else QUALITY_UNKNOWN,
                    "flags": hit["flags"] if hit else FLAG_NO_DATA,
                }
            )
        return samples
