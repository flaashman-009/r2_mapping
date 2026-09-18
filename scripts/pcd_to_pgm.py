#!/usr/bin/env python3
"""Convert a PCD point cloud to a ROS-style occupancy grid PGM/YAML.

The filter chain is intentionally configurable:
  1. timestamp freshness filter (when the PCD has a time/timestamp field)
  2. Z pass-through
  3. voxel-grid downsampling
  4. radius outlier removal
  5. 2D projection and obstacle inflation
  6. PGM + map YAML output

This script only uses numpy so it can run on a small development machine.
"""

import argparse
import math
import os
import struct
import time

import numpy as np


PCD_DTYPE = {
    ("I", 1): "i1",
    ("I", 2): "i2",
    ("I", 4): "i4",
    ("I", 8): "i8",
    ("U", 1): "u1",
    ("U", 2): "u2",
    ("U", 4): "u4",
    ("U", 8): "u8",
    ("F", 4): "f4",
    ("F", 8): "f8",
}


def _read_pcd_header(fh):
    header = {}
    while True:
        raw = fh.readline()
        if not raw:
            raise ValueError("PCD header ended before DATA")
        line = raw.decode("ascii", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        key, _, value = line.partition(" ")
        key = key.upper()
        header[key] = value.strip()
        if key == "DATA":
            return header


def _pcd_dtype(header):
    fields = header["FIELDS"].split()
    sizes = [int(v) for v in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(v) for v in header.get("COUNT", " ".join(["1"] * len(fields))).split()]
    names = []
    formats = []
    offsets = []
    offset = 0
    for field, size, ptype, count in zip(fields, sizes, types, counts):
        base = PCD_DTYPE[(ptype, size)]
        for i in range(count):
            name = field if count == 1 else "%s_%d" % (field, i)
            names.append(name)
            formats.append(base)
            offsets.append(offset)
            offset += size
    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": offset,
    })
    return header, dtype


def read_pcd(path):
    with open(path, "rb") as fh:
        header = _read_pcd_header(fh)
        data_kind = header["DATA"].lower()
        header, dtype = _pcd_dtype(header)

        if data_kind == "ascii":
            rows = np.loadtxt(fh, dtype=dtype, ndmin=1)
            return rows

        if data_kind == "binary":
            points = int(header.get("POINTS", "0"))
            if points <= 0:
                width = int(header.get("WIDTH", "0"))
                height = int(header.get("HEIGHT", "1"))
                points = width * height
            raw = fh.read(points * dtype.itemsize)
            return np.frombuffer(raw, dtype=dtype, count=points)

        raise ValueError("PCD DATA=%s is not supported; use ascii or binary" %
                         header["DATA"])


def timestamp_scale(values):
    """Infer common PCD timestamp units and return a seconds multiplier."""
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return 1.0
    magnitude = float(np.nanmedian(np.abs(finite)))
    if magnitude > 1e17:
        return 1e-9
    if magnitude > 1e14:
        return 1e-6
    if magnitude > 1e11:
        return 1e-3
    if magnitude > 1e8:
        return 1.0
    return 1.0


def filter_timestamps(points, field, max_age_ms):
    if not field:
        return points, {"enabled": False}
    if field not in points.dtype.names:
        raise ValueError("timestamp field %r is not in PCD fields %s" %
                         (field, points.dtype.names))
    stamps = points[field].astype(np.float64)
    scale = timestamp_scale(stamps)
    seconds = stamps * scale
    newest = float(np.nanmax(seconds))
    age_s = newest - seconds
    mask = age_s <= max_age_ms / 1000.0
    kept = points[mask]
    return kept, {
        "enabled": True,
        "field": field,
        "newest_latency_ms": float(np.nanmax(age_s) * 1000.0),
        "kept_points": int(kept.size),
        "removed_points": int(points.size - kept.size),
    }


def xyz_from_pcd(points):
    names = points.dtype.names or ()
    if not all(name in names for name in ("x", "y", "z")):
        raise ValueError("PCD must contain x, y and z fields; got %s" % (names,))
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    return xyz.astype(np.float64, copy=False)


def filter_z(points, z_min, z_max):
    mask = np.isfinite(points).all(axis=1)
    mask &= points[:, 2] >= z_min
    mask &= points[:, 2] <= z_max
    return points[mask]


def voxel_downsample(points, voxel_size):
    if voxel_size <= 0.0 or points.size == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((unique_keys.shape[0], 3), dtype=np.float64)
    counts = np.zeros(unique_keys.shape[0], dtype=np.int64)
    np.add.at(sums, inverse, points)
    np.add.at(counts, inverse, 1)
    return sums / counts[:, None]


def radius_outlier_remove(points, radius, min_neighbors):
    if radius <= 0.0 or min_neighbors <= 1 or points.size == 0:
        return points
    cells = np.floor(points / radius).astype(np.int64)
    buckets = {}
    for index, cell in enumerate(cells):
        buckets.setdefault(tuple(cell), []).append(index)

    neighbors = 3
    radius_sq = radius * radius
    keep = np.zeros(points.shape[0], dtype=bool)
    offsets = [(dx, dy, dz)
               for dx in range(-neighbors, neighbors + 1)
               for dy in range(-neighbors, neighbors + 1)
               for dz in range(-neighbors, neighbors + 1)]
    for index, cell in enumerate(cells):
        cx, cy, cz = map(int, cell)
        count = 0
        for dx, dy, dz in offsets:
            for other in buckets.get((cx + dx, cy + dy, cz + dz), ()):
                diff = points[index] - points[other]
                if float(np.dot(diff, diff)) <= radius_sq:
                    count += 1
                    if count >= min_neighbors:
                        keep[index] = True
                        break
            if keep[index]:
                break
    return points[keep]


def project_to_grid(points, resolution, min_points, inflate_cells):
    if points.size == 0:
        raise ValueError("no points left after filtering")
    xy = points[:, :2]
    min_xy = np.floor(xy.min(axis=0) / resolution) * resolution
    max_xy = np.ceil(xy.max(axis=0) / resolution) * resolution
    width = max(1, int(math.ceil((max_xy[0] - min_xy[0]) / resolution)) + 1)
    height = max(1, int(math.ceil((max_xy[1] - min_xy[1]) / resolution)) + 1)

    cols = np.floor((xy[:, 0] - min_xy[0]) / resolution).astype(np.int64)
    rows = np.floor((xy[:, 1] - min_xy[1]) / resolution).astype(np.int64)
    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (rows, cols), 1)
    occupied = counts >= min_points

    if inflate_cells > 0 and occupied.any():
        expanded = occupied.copy()
        for dy in range(-inflate_cells, inflate_cells + 1):
            for dx in range(-inflate_cells, inflate_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                src = occupied
                dst = np.zeros_like(expanded)
                ys0 = max(0, dy)
                ys1 = min(height, height + dy)
                xs0 = max(0, dx)
                xs1 = min(width, width + dx)
                dst[ys0:ys1, xs0:xs1] = src[
                    max(0, -dy):min(height, height - dy),
                    max(0, -dx):min(width, width - dx),
                ]
                expanded |= dst
        occupied = expanded

    image = np.full((height, width), 254, dtype=np.uint8)
    image[occupied] = 0
    # PGM row 0 is the top row, while ROS map origin is the lower-left.
    image = np.flipud(image)
    return image, min_xy


def save_map(image, origin, resolution, output_prefix):
    out_dir = os.path.dirname(os.path.abspath(output_prefix))
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = output_prefix + ".pgm"
    yaml_path = output_prefix + ".yaml"
    base = os.path.basename(output_prefix)

    header = "P5\n%d %d\n255\n" % (image.shape[1], image.shape[0])
    with open(pgm_path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(image.tobytes())

    with open(yaml_path, "w", encoding="ascii") as fh:
        fh.write("image: %s.pgm\n" % base)
        fh.write("resolution: %.8f\n" % resolution)
        fh.write("origin: [%.8f, %.8f, 0.0]\n" % (origin[0], origin[1]))
        fh.write("negate: 0\n")
        fh.write("occupied_thresh: 0.65\n")
        fh.write("free_thresh: 0.196\n")
    return pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser(
        description="Filter a PCD point cloud and save a ROS-style PGM map.")
    parser.add_argument("input", help="input .pcd file")
    parser.add_argument("output_prefix", help="output prefix, e.g. ~/maps/room")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="map resolution in m/cell")
    parser.add_argument("--z-min", type=float, default=-0.1)
    parser.add_argument("--z-max", type=float, default=1.5)
    parser.add_argument("--voxel-size", type=float, default=0.03)
    parser.add_argument("--outlier-radius", type=float, default=0.08)
    parser.add_argument("--min-neighbors", type=int, default=3)
    parser.add_argument("--min-points", type=int, default=1,
                        help="minimum points in a cell to mark occupied")
    parser.add_argument("--inflate-cells", type=int, default=2,
                        help="obstacle inflation radius in grid cells")
    parser.add_argument("--time-field", default="",
                        help="optional PCD timestamp field, e.g. t/time")
    parser.add_argument("--max-age-ms", type=float, default=200.0,
                        help="drop points older than this relative window")
    args = parser.parse_args()

    started = time.perf_counter()
    points = read_pcd(args.input)
    load_ms = (time.perf_counter() - started) * 1000.0

    points, timestamp_stats = filter_timestamps(
        points, args.time_field, args.max_age_ms)
    xyz = xyz_from_pcd(points)
    xyz = filter_z(xyz, args.z_min, args.z_max)
    after_z = int(xyz.shape[0])
    xyz = voxel_downsample(xyz, args.voxel_size)
    after_voxel = int(xyz.shape[0])
    xyz = radius_outlier_remove(
        xyz, args.outlier_radius, args.min_neighbors)
    after_outlier = int(xyz.shape[0])
    image, origin = project_to_grid(
        xyz, args.resolution, args.min_points, args.inflate_cells)
    pgm_path, yaml_path = save_map(
        image, origin, args.resolution, args.output_prefix)
    total_ms = (time.perf_counter() - started) * 1000.0

    print("load_ms=%.1f" % load_ms)
    print("timestamp=%s" % timestamp_stats)
    print("points_after_z=%d" % after_z)
    print("points_after_voxel=%d" % after_voxel)
    print("points_after_outlier=%d" % after_outlier)
    print("map_size=%dx%d" % (image.shape[1], image.shape[0]))
    print("pgm=%s" % pgm_path)
    print("yaml=%s" % yaml_path)
    print("total_ms=%.1f" % total_ms)


if __name__ == "__main__":
    main()
