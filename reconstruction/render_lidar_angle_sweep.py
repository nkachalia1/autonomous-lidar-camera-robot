#!/usr/bin/env python3
"""Render a contact-sheet SVG of lidar-to-camera overlay angle candidates."""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
from typing import Any

from project_lidar_overlay import (
    camera_intrinsics_from_fov,
    choose_frame_timestamp,
    choose_scan,
    image_data_uri,
    lidar_point_to_lidar_xyz,
    load_json,
    load_lidar_scans,
    nominal_lidar_to_camera,
    point_color,
    project_camera_point,
    rotate_camera_adjustment,
)


def parse_angles(text: str) -> list[float]:
    angles = [float(item.strip()) for item in text.split(",") if item.strip()]
    if not angles:
        raise argparse.ArgumentTypeError("at least one angle is required")
    return angles


def format_angle(angle: float) -> str:
    if angle.is_integer():
        return f"{int(angle):+d}"
    return f"{angle:+.1f}"


def project_scan(
    scan: dict[str, Any],
    angle_offset_deg: float,
    camera_forward_m: float,
    camera_left_m: float,
    camera_up_m: float,
    yaw_deg: float,
    pitch_deg: float,
    roll_deg: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: int,
    height: int,
    min_distance_m: float,
    max_distance_m: float,
) -> tuple[list[tuple[float, float, float, int]], int, int, int]:
    projected_points: list[tuple[float, float, float, int]] = []
    clipped_count = 0
    behind_count = 0
    filtered_count = 0

    for point in scan["points"]:
        point_angle, distance_m, quality = (
            float(point[0]),
            float(point[1]),
            int(point[2]),
        )
        if distance_m < min_distance_m or distance_m > max_distance_m:
            filtered_count += 1
            continue

        point_lidar = lidar_point_to_lidar_xyz(
            point_angle,
            distance_m,
            angle_offset_deg,
        )
        point_camera = nominal_lidar_to_camera(
            point_lidar,
            camera_forward_m,
            camera_left_m,
            camera_up_m,
        )
        point_camera = rotate_camera_adjustment(
            point_camera,
            yaw_deg,
            pitch_deg,
            roll_deg,
        )
        projected = project_camera_point(point_camera, fx, fy, cx, cy)
        if projected is None:
            behind_count += 1
            continue
        u, v = projected
        if not (0 <= u < width and 0 <= v < height):
            clipped_count += 1
            continue
        projected_points.append((u, v, distance_m, quality))

    return projected_points, clipped_count, behind_count, filtered_count


def render_sweep_svg(
    output: Path,
    background: Path,
    session_id: str,
    frame_index: int,
    scan_index: int,
    delta_ms: float,
    angles: list[float],
    panel_results: list[tuple[float, list[tuple[float, float, float, int]], int, int]],
    width: int,
    height: int,
    columns: int,
    thumb_width: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    thumb_height = round(thumb_width * height / width)
    rows = math.ceil(len(angles) / columns)
    margin = 28
    title_height = 94
    gap = 14
    canvas_width = margin * 2 + columns * thumb_width + (columns - 1) * gap
    canvas_height = (
        margin * 2
        + title_height
        + rows * thumb_height
        + max(rows - 1, 0) * gap
        + 54
    )
    scale_x = thumb_width / width
    scale_y = thumb_height / height
    image_href = image_data_uri(background)

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_width}" '
            f'height="{canvas_height}" viewBox="0 0 {canvas_width} {canvas_height}">'
        ),
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        f'<text x="{margin}" y="38" fill="#f8fafc" font-size="26" '
        f'font-weight="700">Session {html.escape(session_id)} angle sweep</text>',
        f'<text x="{margin}" y="66" fill="#94a3b8" font-size="15">'
        f'camera frame {frame_index}, lidar scan {scan_index}, '
        f'timestamp delta {delta_ms:+.1f} ms</text>',
        f'<text x="{margin}" y="88" fill="#94a3b8" font-size="13">'
        'Use opaque surfaces first; glass/window returns are less reliable.</text>',
    ]

    top = margin + title_height
    for index, (angle, points, clipped_count, behind_count) in enumerate(panel_results):
        col = index % columns
        row = index // columns
        x0 = margin + col * (thumb_width + gap)
        y0 = top + row * (thumb_height + gap)
        svg.extend(
            [
                f'<rect x="{x0}" y="{y0}" width="{thumb_width}" '
                f'height="{thumb_height}" fill="#020617" stroke="#334155"/>',
                f'<image x="{x0}" y="{y0}" width="{thumb_width}" '
                f'height="{thumb_height}" preserveAspectRatio="xMidYMid slice" '
                f'href="{image_href}"/>',
                f'<rect x="{x0}" y="{y0}" width="188" height="30" '
                'fill="#020617" fill-opacity="0.88"/>',
                f'<text x="{x0 + 8}" y="{y0 + 21}" fill="#f8fafc" '
                f'font-size="15">{format_angle(angle)} deg | in {len(points)}</text>',
            ]
        )
        for u, v, distance_m, _quality in points:
            px = x0 + u * scale_x
            py = y0 + v * scale_y
            svg.append(
                f'<circle cx="{px:.1f}" cy="{py:.1f}" r="2.2" '
                f'fill="{point_color(distance_m)}" fill-opacity="0.9" '
                'stroke="#020617" stroke-width="0.6"/>'
            )
        svg.append(
            f'<text x="{x0 + 8}" y="{y0 + thumb_height - 9}" '
            f'fill="#e2e8f0" font-size="11">clipped {clipped_count}; '
            f'behind {behind_count}</text>'
        )

    svg.append("</svg>")
    output.write_text("\n".join(svg), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render lidar overlay angle sweep")
    parser.add_argument("session", type=Path)
    parser.add_argument("--background", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--angles", type=parse_angles, required=True)
    parser.add_argument("--frame-index", type=int, default=None)
    parser.add_argument("--scan-index", type=int, default=None)
    parser.add_argument("--camera-forward-m", type=float, required=True)
    parser.add_argument("--camera-left-m", type=float, required=True)
    parser.add_argument("--camera-up-m", type=float, required=True)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--min-distance-m", type=float, default=0.15)
    parser.add_argument("--max-distance-m", type=float, default=8.0)
    parser.add_argument("--fov-x-deg", type=float, default=62.2)
    parser.add_argument("--fov-y-deg", type=float, default=37.2)
    parser.add_argument("--columns", type=int, default=3)
    parser.add_argument("--thumb-width", type=int, default=480)
    args = parser.parse_args()

    if args.columns <= 0:
        parser.error("--columns must be positive")
    if args.thumb_width <= 0:
        parser.error("--thumb-width must be positive")

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    camera_metadata = load_json(session / manifest["camera"]["metadata"])
    scans = load_lidar_scans(session / manifest["lidar"]["scans"])
    width = int(manifest["camera"]["width"])
    height = int(manifest["camera"]["height"])
    fx, fy, cx, cy = camera_intrinsics_from_fov(
        width,
        height,
        args.fov_x_deg,
        args.fov_y_deg,
    )

    frame_index, frame_timestamp_ns = choose_frame_timestamp(
        camera_metadata,
        args.frame_index,
    )
    scan_index, scan = choose_scan(scans, args.scan_index, frame_timestamp_ns)
    delta_ms = (int(scan["timestamp_us"]) * 1000 - frame_timestamp_ns) / 1e6

    panel_results = []
    for angle in args.angles:
        points, clipped_count, behind_count, _filtered_count = project_scan(
            scan,
            angle,
            args.camera_forward_m,
            args.camera_left_m,
            args.camera_up_m,
            args.yaw_deg,
            args.pitch_deg,
            args.roll_deg,
            fx,
            fy,
            cx,
            cy,
            width,
            height,
            args.min_distance_m,
            args.max_distance_m,
        )
        panel_results.append((angle, points, clipped_count, behind_count))

    render_sweep_svg(
        args.output,
        args.background.resolve(),
        str(manifest["session_id"]),
        frame_index,
        scan_index,
        delta_ms,
        args.angles,
        panel_results,
        width,
        height,
        args.columns,
        args.thumb_width,
    )

    print(f"Wrote {args.output}")
    print(f"Angles: {', '.join(format_angle(angle) for angle in args.angles)}")
    print(f"Frame/scan timestamp delta: {delta_ms:+.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
