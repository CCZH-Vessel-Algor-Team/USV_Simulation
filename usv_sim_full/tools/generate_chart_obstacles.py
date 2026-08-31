#!/usr/bin/env python3
"""Generate an extruded Gazebo obstacle mesh from a ROS occupancy map."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import triangulate


MB_MARKER_BUOY_RED_VISUAL = {
    "mesh_uri": "model://mb_marker_buoy_red/meshes/mb_marker_buoy.dae",
    "ambient": "0.8 0.0 0.0 1",
    "diffuse": "1.0 0.0 0.0 1",
    "pose": "0 0 0 1.57079 0 0",
    "scale": "2 2 2",
}


VISUAL_REPLACEMENTS = {
    f"chart_obstacle_{index:03d}": MB_MARKER_BUOY_RED_VISUAL
    for index in (*range(5, 17), *range(42, 122))
}


def parse_args():
    package_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "map_yaml",
        nargs="?",
        type=Path,
        default=package_dir / "maps" / "CN441122_enc_5km.yaml",
    )
    parser.add_argument(
        "model_dir",
        nargs="?",
        type=Path,
        default=(
            package_dir
            / "description"
            / "world"
            / "models"
            / "ccs_chart_obstacles_enc_5km"
        ),
        help="Gazebo model directory to generate",
    )
    parser.add_argument(
        "--model-name",
        default="ccs_chart_obstacles_enc_5km",
        help="Name written into the generated model.sdf",
    )
    parser.add_argument("--bottom", type=float, default=-5.0)
    parser.add_argument("--top", type=float, default=3.0)
    parser.add_argument(
        "--simplify-pixels",
        type=float,
        default=1.0,
        help="Contour simplification tolerance in map pixels",
    )
    return parser.parse_args()


def load_map(map_yaml):
    with map_yaml.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    image_path = Path(config["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Cannot read occupancy image: {image_path}")

    gray = image.astype(np.float32) / 255.0
    occupancy = gray if int(config.get("negate", 0)) else 1.0 - gray
    mask = (occupancy > float(config["occupied_thresh"])).astype(np.uint8)
    return config, mask


def contour_shape(contour):
    points = [(float(point[0][0]), float(point[0][1])) for point in contour]
    if len(points) == 1:
        return Point(points[0]).buffer(0.5, cap_style="square")
    if len(points) == 2:
        return LineString(points).buffer(0.5, cap_style="square", join_style="mitre")
    return Polygon(points)


def extract_polygons(mask, simplify_pixels):
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
    )
    if hierarchy is None:
        return []

    polygons = []
    hierarchy = hierarchy[0]
    for index, contour in enumerate(contours):
        if hierarchy[index][3] != -1:
            continue

        outer = contour_shape(contour)
        holes = []
        child = hierarchy[index][2]
        while child != -1:
            hole = contour_shape(contours[child])
            if isinstance(hole, Polygon):
                holes.append(list(hole.exterior.coords))
            child = hierarchy[child][0]

        if isinstance(outer, Polygon) and holes:
            outer = Polygon(outer.exterior.coords, holes)

        # Expanding half a pixel moves centerline contours onto pixel boundaries.
        polygon = outer.buffer(0.5, join_style="mitre")
        if simplify_pixels > 0:
            polygon = polygon.simplify(simplify_pixels, preserve_topology=True)
        polygon = polygon.buffer(0)
        if not polygon.is_empty:
            polygons.extend(polygon.geoms if isinstance(polygon, MultiPolygon) else [polygon])
    return polygons


class ObjMesh:
    def __init__(self):
        self.vertices = []
        self.faces = []
        self.vertex_ids = {}

    def vertex(self, xyz):
        key = tuple(round(value, 6) for value in xyz)
        if key not in self.vertex_ids:
            self.vertex_ids[key] = len(self.vertices) + 1
            self.vertices.append(key)
        return self.vertex_ids[key]

    def face(self, *xyz):
        self.faces.append(tuple(self.vertex(point) for point in xyz))


def map_point(point, image_height, resolution, origin):
    local_x = (point[0] + 0.5) * resolution
    local_y = (image_height - point[1] - 0.5) * resolution
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    return (
        origin[0] + cosine * local_x - sine * local_y,
        origin[1] + sine * local_x + cosine * local_y,
    )


def add_polygon(
    mesh, polygon, image_height, resolution, origin, offset, bottom, top
):
    def world_xyz(point, z):
        x, y = map_point(point, image_height, resolution, origin)
        return x - offset[0], y - offset[1], z

    for ring in [polygon.exterior, *polygon.interiors]:
        points = list(ring.coords)
        for first, second in zip(points, points[1:]):
            mesh.face(
                world_xyz(first, bottom),
                world_xyz(second, bottom),
                world_xyz(second, top),
                world_xyz(first, top),
            )

    for triangle in triangulate(polygon):
        if triangle.area == 0 or polygon.intersection(triangle).area < triangle.area * 0.999999:
            continue
        points = list(triangle.exterior.coords)[:3]
        # map_point flips the image Y axis into world coordinates, which reverses
        # triangle winding. Flip the cap faces back so top normals point upward.
        mesh.face(*(world_xyz(point, top) for point in reversed(points)))
        mesh.face(*(world_xyz(point, bottom) for point in points))


def write_obj(output, mesh, source_map, object_name):
    output.parent.mkdir(parents=True, exist_ok=True)
    material_path = output.with_suffix(".mtl")
    with material_path.open("w", encoding="ascii") as stream:
        stream.write("newmtl chart_land\n")
        stream.write("Ka 0.32 0.28 0.18\n")
        stream.write("Kd 0.42 0.36 0.22\n")
        stream.write("Ks 0.02 0.02 0.02\n")
        stream.write("Ns 5.0\n")
        stream.write("d 1.0\n")

    normals = []
    for face in mesh.faces:
        first, second, third = (mesh.vertices[index - 1] for index in face[:3])
        edge_a = [second[i] - first[i] for i in range(3)]
        edge_b = [third[i] - first[i] for i in range(3)]
        normal = (
            edge_a[1] * edge_b[2] - edge_a[2] * edge_b[1],
            edge_a[2] * edge_b[0] - edge_a[0] * edge_b[2],
            edge_a[0] * edge_b[1] - edge_a[1] * edge_b[0],
        )
        length = math.sqrt(sum(value * value for value in normal))
        if length == 0:
            raise ValueError(f"Degenerate face in generated mesh: {face}")
        normals.append(tuple(value / length for value in normal))

    with output.open("w", encoding="ascii") as stream:
        stream.write(f"# Generated from {source_map.name}\n")
        stream.write(f"mtllib {material_path.name}\n")
        stream.write(f"o {object_name}\n")
        stream.write("usemtl chart_land\n")
        for vertex in mesh.vertices:
            stream.write("v {:.6f} {:.6f} {:.6f}\n".format(*vertex))
        for normal in normals:
            stream.write("vn {:.9f} {:.9f} {:.9f}\n".format(*normal))
        for normal_index, face in enumerate(mesh.faces, start=1):
            stream.write(
                "f {}\n".format(
                    " ".join(f"{index}//{normal_index}" for index in face)
                )
            )


def write_model_sdf(model_dir, model_name, components):
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.9">',
        f'  <model name="{model_name}">',
        '    <static>true</static>',
    ]
    for name, center in components:
        visual_replacement = VISUAL_REPLACEMENTS.get(name)
        if visual_replacement:
            visual_lines = [
                '      <visual name="visual">',
                f'        <pose>{visual_replacement["pose"]}</pose>',
                '        <geometry>',
                '          <mesh>',
                f'            <uri>{visual_replacement["mesh_uri"]}</uri>',
                f'            <scale>{visual_replacement["scale"]}</scale>',
                '          </mesh>',
                '        </geometry>',
                '        <material>',
                f'          <ambient>{visual_replacement["ambient"]}</ambient>',
                f'          <diffuse>{visual_replacement["diffuse"]}</diffuse>',
                '        </material>',
                '      </visual>',
            ]
        else:
            visual_lines = [
                '      <visual name="visual">',
                '        <geometry>',
                '          <mesh>',
                f'            <uri>meshes/{name}.obj</uri>',
                '          </mesh>',
                '        </geometry>',
                '        <material>',
                '          <ambient>0.32 0.28 0.18 1</ambient>',
                '          <diffuse>0.42 0.36 0.22 1</diffuse>',
                '        </material>',
                '      </visual>',
            ]

        lines.extend(
            [
                f'    <link name="{name}">',
                f'      <pose>{center[0]:.6f} {center[1]:.6f} 0 0 0 0</pose>',
                *visual_lines,
                '      <collision name="collision">',
                '        <geometry>',
                '          <mesh>',
                f'            <uri>meshes/{name}.obj</uri>',
                '          </mesh>',
                '        </geometry>',
                '      </collision>',
                '    </link>',
            ]
        )
    lines.extend(['  </model>', '</sdf>', ''])
    with (model_dir / "model.sdf").open("w", encoding="ascii") as stream:
        stream.write("\n".join(lines))


def main():
    args = parse_args()
    if args.bottom >= args.top:
        raise ValueError("--bottom must be lower than --top")

    map_yaml = args.map_yaml.resolve()
    config, mask = load_map(map_yaml)
    polygons = extract_polygons(mask, args.simplify_pixels)
    polygons.sort(key=lambda polygon: polygon.area, reverse=True)
    origin = [float(value) for value in config["origin"]]
    resolution = float(config["resolution"])

    model_dir = args.model_dir.resolve()
    mesh_dir = model_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for pattern in ("chart_obstacle*.obj", "chart_obstacle*.mtl"):
        for stale_file in mesh_dir.glob(pattern):
            stale_file.unlink()

    components = []
    total_vertices = 0
    total_faces = 0
    for index, polygon in enumerate(polygons, start=1):
        name = f"chart_obstacle_{index:03d}"
        center = map_point(
            polygon.centroid.coords[0], mask.shape[0], resolution, origin
        )
        mesh = ObjMesh()
        add_polygon(
            mesh,
            polygon,
            mask.shape[0],
            resolution,
            origin,
            center,
            args.bottom,
            args.top,
        )
        write_obj(mesh_dir / f"{name}.obj", mesh, map_yaml, name)
        components.append((name, center))
        total_vertices += len(mesh.vertices)
        total_faces += len(mesh.faces)

    write_model_sdf(model_dir, args.model_name, components)
    print(
        f"Generated {model_dir}: {len(components)} connected components, "
        f"{total_vertices} vertices, {total_faces} faces"
    )


if __name__ == "__main__":
    main()
