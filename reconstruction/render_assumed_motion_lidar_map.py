#!/usr/bin/env python3
"""Render a top-down lidar map using a simple assumed straight-line trajectory.

This is intentionally not a SLAM implementation. It is a first reconstruction
diagnostic: if the rig is pushed along a measured straight path with little
rotation, place each 2D lidar scan along that path and inspect whether walls and
large objects form coherent metric structure.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


Point = tuple[float, float, float]
ColoredPoint = tuple[float, float, float, float]


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def load_scans(path: Path) -> list[dict[str, Any]]:
    scans: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            item = json.loads(line)
            if item.get("type") == "scan":
                item["_line_number"] = line_number
                scans.append(item)
    if not scans:
        raise ValueError(f"no lidar scans found in {path}")
    return scans


def raw_lidar_point_to_rig_xy(
    angle_deg: float,
    distance_m: float,
    angle_offset_deg: float,
) -> Point:
    """Convert a raw RPLIDAR polar point into the provisional rig frame.

    Rig frame convention:
      +x forward, approximately camera-facing direction
      +y left
      +z up

    The angle offset is the same software parameter used by the camera overlay
    tools.
    """

    angle_rad = math.radians(angle_deg + angle_offset_deg)
    return (
        distance_m * math.cos(angle_rad),
        distance_m * math.sin(angle_rad),
        0.0,
    )


def pose_progress(elapsed_s: float, motion_start_s: float, motion_end_s: float) -> float:
    if elapsed_s <= motion_start_s:
        return 0.0
    if elapsed_s >= motion_end_s:
        return 1.0
    return (elapsed_s - motion_start_s) / (motion_end_s - motion_start_s)


def transform_point(
    local: Point,
    progress: float,
    path_length_m: float,
    path_yaw_deg: float,
) -> tuple[float, float]:
    yaw = math.radians(path_yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    pose_x = progress * path_length_m * cos_yaw
    pose_y = progress * path_length_m * sin_yaw
    local_x, local_y, _local_z = local
    return (
        pose_x + cos_yaw * local_x - sin_yaw * local_y,
        pose_y + sin_yaw * local_x + cos_yaw * local_y,
    )


def time_color(fraction: float) -> str:
    """Blue-to-yellow-to-red gradient for elapsed capture time."""

    fraction = max(0.0, min(1.0, fraction))
    if fraction < 0.5:
        t = fraction / 0.5
        r = round(56 + (250 - 56) * t)
        g = round(189 + (204 - 189) * t)
        b = round(248 + (21 - 248) * t)
    else:
        t = (fraction - 0.5) / 0.5
        r = round(250 + (251 - 250) * t)
        g = round(204 + (113 - 204) * t)
        b = round(21 + (133 - 21) * t)
    return f"#{r:02x}{g:02x}{b:02x}"


def collect_points(
    scans: list[dict[str, Any]],
    angle_offset_deg: float,
    path_length_m: float,
    path_yaw_deg: float,
    motion_start_s: float,
    motion_end_s: float,
    min_distance_m: float,
    max_distance_m: float,
    min_quality: int,
    scan_stride: int,
    point_stride: int,
) -> tuple[list[ColoredPoint], dict[str, Any]]:
    first_timestamp_us = int(scans[0]["timestamp_us"])
    last_timestamp_us = int(scans[-1]["timestamp_us"])
    duration_s = (last_timestamp_us - first_timestamp_us) / 1e6
    if duration_s <= 0:
        raise ValueError("lidar scan duration must be positive")
    if motion_end_s <= motion_start_s:
        raise ValueError("--motion-end-s must be greater than --motion-start-s")

    points: list[ColoredPoint] = []
    considered_scans = 0
    considered_returns = 0
    filtered_returns = 0
    for scan_index, scan in enumerate(scans):
        if scan_index % scan_stride != 0:
            continue
        considered_scans += 1
        elapsed_s = (int(scan["timestamp_us"]) - first_timestamp_us) / 1e6
        progress = pose_progress(elapsed_s, motion_start_s, motion_end_s)
        time_fraction = elapsed_s / duration_s
        for point_index, point in enumerate(scan["points"]):
            if point_index % point_stride != 0:
                continue
            angle_deg, distance_m, quality = float(point[0]), float(point[1]), int(point[2])
            considered_returns += 1
            if (
                distance_m < min_distance_m
                or distance_m > max_distance_m
                or quality < min_quality
            ):
                filtered_returns += 1
                continue
            local = raw_lidar_point_to_rig_xy(angle_deg, distance_m, angle_offset_deg)
            x_m, y_m = transform_point(local, progress, path_length_m, path_yaw_deg)
            points.append((x_m, y_m, 0.0, time_fraction))

    summary = {
        "input_scan_count": len(scans),
        "used_scan_count": considered_scans,
        "considered_returns": considered_returns,
        "filtered_returns": filtered_returns,
        "output_points": len(points),
        "lidar_duration_s": duration_s,
    }
    return points, summary


def point_bounds(points: list[ColoredPoint], padding_m: float) -> tuple[float, float, float, float]:
    if not points:
        raise ValueError("no output points survived filtering")
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (
        min(xs) - padding_m,
        max(xs) + padding_m,
        min(ys) - padding_m,
        max(ys) + padding_m,
    )


def render_svg(
    output: Path,
    points: list[ColoredPoint],
    session_id: str,
    path_length_m: float,
    path_yaw_deg: float,
    motion_start_s: float,
    motion_end_s: float,
    angle_offset_deg: float,
    summary: dict[str, Any],
    max_points_in_svg: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    min_x, max_x, min_y, max_y = point_bounds(points, padding_m=0.4)
    world_width = max_x - min_x
    world_height = max_y - min_y

    width, height = 1500, 1050
    margin_left, margin_right, margin_top, margin_bottom = 88, 42, 130, 132
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom
    scale = min(plot_width / world_width, plot_height / world_height)

    def px(point_x: float) -> float:
        return margin_left + (point_x - min_x) * scale

    def py(point_y: float) -> float:
        return margin_top + (max_y - point_y) * scale

    yaw = math.radians(path_yaw_deg)
    start_x, start_y = 0.0, 0.0
    end_x = path_length_m * math.cos(yaw)
    end_y = path_length_m * math.sin(yaw)

    point_step = max(1, math.ceil(len(points) / max_points_in_svg))
    displayed_points = points[::point_step]

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        f'<text x="48" y="52" fill="#f8fafc" font-size="30" '
        f'font-weight="700">Session {session_id} assumed-motion lidar map</text>',
        (
            f'<text x="48" y="82" fill="#94a3b8" font-size="16">'
            f'Straight path assumption: {path_length_m:.3f} m, yaw {path_yaw_deg:.1f}°, '
            f'motion window {motion_start_s:.1f}-{motion_end_s:.1f}s, '
            f'lidar angle offset {angle_offset_deg:.1f}°</text>'
        ),
        (
            f'<text x="48" y="106" fill="#fbbf24" font-size="14">'
            'Diagnostic only: this is not SLAM; rotation or wheel slip will smear the map.</text>'
        ),
        (
            f'<rect x="{margin_left}" y="{margin_top}" width="{plot_width}" '
            f'height="{plot_height}" rx="18" fill="#0d1726" stroke="#334155"/>'
        ),
    ]

    grid_start_x = math.floor(min_x)
    grid_end_x = math.ceil(max_x)
    grid_start_y = math.floor(min_y)
    grid_end_y = math.ceil(max_y)
    for x_m in range(grid_start_x, grid_end_x + 1):
        x = px(float(x_m))
        svg.append(
            f'<line x1="{x:.1f}" y1="{margin_top}" x2="{x:.1f}" '
            f'y2="{margin_top + plot_height}" stroke="#223246" stroke-width="1"/>'
        )
        svg.append(
            f'<text x="{x + 4:.1f}" y="{margin_top + plot_height + 24}" '
            f'fill="#64748b" font-size="12">{x_m}m</text>'
        )
    for y_m in range(grid_start_y, grid_end_y + 1):
        y = py(float(y_m))
        svg.append(
            f'<line x1="{margin_left}" y1="{y:.1f}" '
            f'x2="{margin_left + plot_width}" y2="{y:.1f}" '
            'stroke="#223246" stroke-width="1"/>'
        )
        svg.append(
            f'<text x="32" y="{y + 4:.1f}" fill="#64748b" '
            f'font-size="12">{y_m}m</text>'
        )

    for x_m, y_m, _z_m, time_fraction in displayed_points:
        svg.append(
            f'<circle cx="{px(x_m):.1f}" cy="{py(y_m):.1f}" r="1.4" '
            f'fill="{time_color(time_fraction)}" fill-opacity="0.62"/>'
        )

    svg.extend(
        [
            f'<line x1="{px(start_x):.1f}" y1="{py(start_y):.1f}" '
            f'x2="{px(end_x):.1f}" y2="{py(end_y):.1f}" '
            'stroke="#f8fafc" stroke-width="5" stroke-linecap="round"/>',
            f'<circle cx="{px(start_x):.1f}" cy="{py(start_y):.1f}" r="8" fill="#22c55e"/>',
            f'<circle cx="{px(end_x):.1f}" cy="{py(end_y):.1f}" r="8" fill="#ef4444"/>',
            f'<text x="{px(start_x) + 12:.1f}" y="{py(start_y) - 10:.1f}" '
            'fill="#bbf7d0" font-size="14" font-weight="700">start</text>',
            f'<text x="{px(end_x) + 12:.1f}" y="{py(end_y) - 10:.1f}" '
            'fill="#fecaca" font-size="14" font-weight="700">end</text>',
        ]
    )

    legend_y = height - 92
    svg.extend(
        [
            f'<rect x="48" y="{legend_y - 36}" width="{width - 96}" height="86" '
            'rx="16" fill="#0d1726" stroke="#334155"/>',
            f'<text x="70" y="{legend_y - 8}" fill="#f8fafc" font-size="17" '
            f'font-weight="700">Output points: {summary["output_points"]:,} '
            f'({len(displayed_points):,} displayed in SVG)</text>',
            f'<text x="70" y="{legend_y + 18}" fill="#94a3b8" font-size="14">'
            f'Used scans: {summary["used_scan_count"]}/{summary["input_scan_count"]}; '
            f'filtered returns: {summary["filtered_returns"]:,}/'
            f'{summary["considered_returns"]:,}; lidar duration: '
            f'{summary["lidar_duration_s"]:.2f}s</text>',
            f'<circle cx="{width - 376}" cy="{legend_y + 14}" r="7" fill="#38bdf8"/>',
            f'<text x="{width - 360}" y="{legend_y + 19}" fill="#cbd5e1" '
            'font-size="14">early</text>',
            f'<circle cx="{width - 286}" cy="{legend_y + 14}" r="7" fill="#facc15"/>',
            f'<text x="{width - 270}" y="{legend_y + 19}" fill="#cbd5e1" '
            'font-size="14">middle</text>',
            f'<circle cx="{width - 176}" cy="{legend_y + 14}" r="7" fill="#fb7185"/>',
            f'<text x="{width - 160}" y="{legend_y + 19}" fill="#cbd5e1" '
            'font-size="14">late</text>',
            "</svg>",
        ]
    )
    output.write_text("\n".join(svg), encoding="utf-8")


def write_ply(output: Path, points: list[ColoredPoint]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(points)}",
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "end_header",
    ]
    for x_m, y_m, z_m, time_fraction in points:
        color = time_color(time_fraction).lstrip("#")
        red = int(color[0:2], 16)
        green = int(color[2:4], 16)
        blue = int(color[4:6], 16)
        lines.append(f"{x_m:.4f} {y_m:.4f} {z_m:.4f} {red} {green} {blue}")
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render a lidar map from an assumed straight-line rig motion"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--output", type=Path, required=True, help="Output SVG path")
    parser.add_argument("--ply-output", type=Path, help="Optional output PLY path")
    parser.add_argument(
        "--path-length-m",
        type=float,
        required=True,
        help="Measured straight-line rig travel distance in meters",
    )
    parser.add_argument("--path-yaw-deg", type=float, default=0.0)
    parser.add_argument("--motion-start-s", type=float, default=5.0)
    parser.add_argument("--motion-end-s", type=float, default=25.0)
    parser.add_argument("--lidar-angle-offset-deg", type=float, default=125.0)
    parser.add_argument("--min-distance-m", type=float, default=0.15)
    parser.add_argument("--max-distance-m", type=float, default=5.0)
    parser.add_argument("--min-quality", type=int, default=1)
    parser.add_argument("--scan-stride", type=int, default=2)
    parser.add_argument("--point-stride", type=int, default=1)
    parser.add_argument("--max-points-in-svg", type=int, default=80000)
    args = parser.parse_args()

    if args.path_length_m <= 0:
        parser.error("--path-length-m must be positive")
    if args.scan_stride <= 0:
        parser.error("--scan-stride must be positive")
    if args.point_stride <= 0:
        parser.error("--point-stride must be positive")
    if args.max_points_in_svg <= 0:
        parser.error("--max-points-in-svg must be positive")

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    scans = load_scans(session / manifest["lidar"]["scans"])
    points, summary = collect_points(
        scans,
        args.lidar_angle_offset_deg,
        args.path_length_m,
        args.path_yaw_deg,
        args.motion_start_s,
        args.motion_end_s,
        args.min_distance_m,
        args.max_distance_m,
        args.min_quality,
        args.scan_stride,
        args.point_stride,
    )
    render_svg(
        args.output,
        points,
        str(manifest["session_id"]),
        args.path_length_m,
        args.path_yaw_deg,
        args.motion_start_s,
        args.motion_end_s,
        args.lidar_angle_offset_deg,
        summary,
        args.max_points_in_svg,
    )
    if args.ply_output:
        write_ply(args.ply_output, points)

    print(f"Wrote {args.output}")
    if args.ply_output:
        print(f"Wrote {args.ply_output}")
    print(f"Input scans: {summary['input_scan_count']}")
    print(f"Used scans: {summary['used_scan_count']}")
    print(f"Output points: {summary['output_points']}")
    print(f"Filtered returns: {summary['filtered_returns']}")
    print(f"Lidar duration: {summary['lidar_duration_s']:.3f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
