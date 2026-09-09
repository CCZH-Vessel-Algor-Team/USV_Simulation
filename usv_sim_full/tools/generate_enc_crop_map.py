#!/usr/bin/env python3
"""Rasterize a square S-57 ENC crop into the ENU frame used by Gazebo and ROS."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from osgeo import ogr
from pyproj import Transformer
from shapely.geometry import GeometryCollection, box, mapping, shape
from shapely.ops import transform as transform_geometry
from shapely.ops import unary_union


AREA_LAYERS = ("LNDARE",)
LINE_LAYERS = ("COALNE",)
POINT_LAYERS = ("OBSTRN", "UWTROC", "WRECKS", "BOYLAT", "BOYCAR", "BCNCAR", "BCNSPP")


def parse_args() -> argparse.Namespace:
    package_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("enc", type=Path, help="Path to the S-57 .000 cell")
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=package_dir / "maps" / "CN441122_enc_5km",
        help="Output path without .pgm/.yaml suffix",
    )
    parser.add_argument("--longitude", type=float, default=119.481403)
    parser.add_argument("--latitude", type=float, default=34.692120)
    parser.add_argument("--resolution", type=float, default=2.0)
    parser.add_argument(
        "--pixels",
        type=int,
        default=2501,
        help="Odd square image dimension so its center pixel is the ENU origin",
    )
    parser.add_argument("--coast-buffer", type=float, default=2.0)
    parser.add_argument("--hazard-radius", type=float, default=4.0)
    return parser.parse_args()


def enu_transformer(longitude: float, latitude: float) -> Transformer:
    return Transformer.from_pipeline(
        "+proj=pipeline "
        "+step +proj=cart +ellps=WGS84 "
        "+step +proj=topocentric +ellps=WGS84 "
        f"+lat_0={latitude} +lon_0={longitude} +h_0=0"
    )


def project_geometry(geometry: ogr.Geometry, transformer: Transformer):
    geographic = shape(json.loads(geometry.ExportToJson()))
    return transform_geometry(lambda x, y, z=None: transformer.transform(x, y), geographic)


def crop_layer(
    dataset: ogr.DataSource,
    layer_name: str,
    transformer: Transformer,
    crop: object,
    buffer_m: float,
) -> list[object]:
    layer = dataset.GetLayerByName(layer_name)
    if layer is None:
        return []

    features = []
    for feature in layer:
        geometry = feature.GetGeometryRef()
        if geometry is None:
            continue
        projected = project_geometry(geometry, transformer)
        if projected.is_empty:
            continue
        if buffer_m > 0:
            projected = projected.buffer(buffer_m)
        clipped = projected.intersection(crop)
        if not clipped.is_empty:
            features.append(clipped)
    return features


def geometry_to_pixels(geometry: object, origin: float, resolution: float, pixels: int):
    def convert(x, y, z=None):
        return ((x - origin) / resolution - 0.5, pixels - (y - origin) / resolution - 0.5)

    return transform_geometry(convert, geometry)


def fill_geometry(image: np.ndarray, geometry: object, origin: float, resolution: float) -> None:
    pixel_geometry = geometry_to_pixels(geometry, origin, resolution, image.shape[0])
    geometries = pixel_geometry.geoms if hasattr(pixel_geometry, "geoms") else [pixel_geometry]
    for item in geometries:
        if item.is_empty:
            continue
        if item.geom_type == "Polygon":
            exterior = np.rint(item.exterior.coords).astype(np.int32)
            cv2.fillPoly(image, [exterior], 0)
            for interior in item.interiors:
                cv2.fillPoly(image, [np.rint(interior.coords).astype(np.int32)], 254)


def main() -> None:
    args = parse_args()
    if args.pixels < 3 or args.pixels % 2 == 0:
        raise ValueError("--pixels must be an odd integer of at least 3")
    if args.resolution <= 0 or args.coast_buffer < 0 or args.hazard_radius < 0:
        raise ValueError("resolution and buffer radii must be non-negative")

    enc_path = args.enc.resolve()
    dataset = ogr.Open(str(enc_path), 0)
    if dataset is None:
        raise RuntimeError(f"Cannot open ENC: {enc_path}")

    transformer = enu_transformer(args.longitude, args.latitude)
    half_extent = args.pixels * args.resolution / 2.0
    origin = -half_extent
    crop = box(origin, origin, half_extent, half_extent)

    occupied = []
    layer_counts: dict[str, int] = {}
    for layer_name in AREA_LAYERS:
        features = crop_layer(dataset, layer_name, transformer, crop, 0.0)
        occupied.extend(features)
        layer_counts[layer_name] = len(features)
    for layer_name in LINE_LAYERS:
        features = crop_layer(dataset, layer_name, transformer, crop, args.coast_buffer)
        occupied.extend(features)
        layer_counts[layer_name] = len(features)
    for layer_name in POINT_LAYERS:
        features = crop_layer(dataset, layer_name, transformer, crop, args.hazard_radius)
        occupied.extend(features)
        layer_counts[layer_name] = len(features)

    obstacle_geometry = unary_union(occupied) if occupied else GeometryCollection()
    image = np.full((args.pixels, args.pixels), 254, dtype=np.uint8)
    if not obstacle_geometry.is_empty:
        fill_geometry(image, obstacle_geometry, origin, args.resolution)

    output_prefix = args.output_prefix.resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    metadata_path = output_prefix.with_suffix(".geo.json")
    if not cv2.imwrite(str(pgm_path), image):
        raise RuntimeError(f"Cannot write {pgm_path}")

    map_config = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": args.resolution,
        "origin": [origin, origin, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with yaml_path.open("w", encoding="ascii") as stream:
        yaml.safe_dump(map_config, stream, sort_keys=False)

    metadata = {
        "type": "FeatureCollection",
        "name": output_prefix.name,
        "crs": "EPSG:4326",
        "features": [
            {
                "type": "Feature",
                "properties": {
                    "description": "Gazebo and ROS ENU origin; center pixel is (0, 0).",
                    "enu_origin_longitude": args.longitude,
                    "enu_origin_latitude": args.latitude,
                    "pixel": [args.pixels // 2, args.pixels // 2],
                    "resolution_m": args.resolution,
                    "pixels": args.pixels,
                    "source_enc": enc_path.name,
                    "layer_counts": layer_counts,
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [args.longitude, args.latitude],
                },
            }
        ],
    }
    with metadata_path.open("w", encoding="ascii") as stream:
        json.dump(metadata, stream, indent=2)
        stream.write("\n")

    print(f"Wrote {pgm_path}")
    print(f"Wrote {yaml_path}")
    print(f"Wrote {metadata_path}")
    print(f"Obstacle features after crop: {layer_counts}")


if __name__ == "__main__":
    main()
