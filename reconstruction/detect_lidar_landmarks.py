#!/usr/bin/env python3
"""Detect straight landmark candidates in an ICP-rendered lidar map.

The intended use is a no-new-hardware scale check: place two flat opaque boards
or box faces at a measured separation, capture a motion session, run ICP, then
use this script to label straight map features and measure the distance between
the two reference-board candidates.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import random
from pathlib import Path
from typing import Any

import render_icp_lidar_map as icp


Point2 = tuple[float, float]


COLORS = [
    "#fb7185",
    "#38bdf8",
    "#facc15",
    "#a78bfa",
    "#34d399",
    "#f97316",
    "#e879f9",
    "#60a5fa",
    "#bef264",
    "#f472b6",
    "#22d3ee",
    "#fde68a",
]


def load_trajectory(path: Path) -> list[tuple[int, icp.Pose2]]:
    payload = icp.load_json(path)
    poses = []
    for item in payload["poses"]:
        poses.append(
            (
                int(item["scan_index"]),
                (float(item["x_m"]), float(item["y_m"]), float(item["theta_rad"])),
            )
        )
    if not poses:
        raise ValueError(f"no poses found in {path}")
    return poses


def downsample_points(points: list[Point2], max_points: int, seed: int) -> list[Point2]:
    if len(points) <= max_points:
        return points
    rng = random.Random(seed)
    indices = sorted(rng.sample(range(len(points)), max_points))
    return [points[index] for index in indices]


def fit_line_from_points(first: Point2, second: Point2) -> tuple[float, float, float] | None:
    x1, y1 = first
    x2, y2 = second
    dx = x2 - x1
    dy = y2 - y1
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None
    a = dy / norm
    b = -dx / norm
    c = -(a * x1 + b * y1)
    return a, b, c


def line_distance(line: tuple[float, float, float], point: Point2) -> float:
    a, b, c = line
    return abs(a * point[0] + b * point[1] + c)


def line_projection(line: tuple[float, float, float], point: Point2) -> float:
    a, b, _c = line
    # Direction vector along line.
    direction_x = -b
    direction_y = a
    return point[0] * direction_x + point[1] * direction_y


def line_segment_from_inliers(
    line: tuple[float, float, float],
    inliers: list[Point2],
) -> tuple[Point2, Point2, Point2, float, float]:
    a, b, c = line
    direction_x = -b
    direction_y = a
    projections = [line_projection(line, point) for point in inliers]
    min_t = min(projections)
    max_t = max(projections)
    # Closest point on the infinite line to the origin is -c * normal.
    origin_x = -c * a
    origin_y = -c * b
    start = origin_x + min_t * direction_x, origin_y + min_t * direction_y
    end = origin_x + max_t * direction_x, origin_y + max_t * direction_y
    center = (start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    angle_deg = math.degrees(math.atan2(direction_y, direction_x))
    return start, end, center, length, angle_deg


def detect_lines(
    points: list[Point2],
    max_candidates: int,
    ransac_iterations: int,
    line_threshold_m: float,
    min_inliers: int,
    min_segment_length_m: float,
    max_points_for_ransac: int,
    seed: int,
) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    remaining = downsample_points(points, max_points_for_ransac, seed)
    candidates: list[dict[str, Any]] = []

    for candidate_index in range(max_candidates):
        if len(remaining) < min_inliers:
            break

        best_line: tuple[float, float, float] | None = None
        best_inliers: list[Point2] = []
        best_score = -1.0

        for _iteration in range(ransac_iterations):
            first, second = rng.sample(remaining, 2)
            line = fit_line_from_points(first, second)
            if line is None:
                continue

            inliers = [
                point
                for point in remaining
                if line_distance(line, point) <= line_threshold_m
            ]
            if len(inliers) < min_inliers:
                continue

            start, end, _center, length, _angle_deg = line_segment_from_inliers(line, inliers)
            if length < min_segment_length_m:
                continue

            # Prefer long, dense line segments.
            score = len(inliers) * min(length, 2.0)
            if score > best_score:
                best_score = score
                best_line = line
                best_inliers = inliers

        if best_line is None:
            break

        start, end, center, length, angle_deg = line_segment_from_inliers(
            best_line,
            best_inliers,
        )
        candidates.append(
            {
                "id": candidate_index + 1,
                "inlier_count": len(best_inliers),
                "start": {"x_m": start[0], "y_m": start[1]},
                "end": {"x_m": end[0], "y_m": end[1]},
                "center": {"x_m": center[0], "y_m": center[1]},
                "length_m": length,
                "angle_deg": angle_deg,
            }
        )

        # Remove inliers for this candidate so later candidates find different
        # boards/walls instead of re-detecting the same one.
        inlier_ids = {id(point) for point in best_inliers}
        remaining = [point for point in remaining if id(point) not in inlier_ids]

    return candidates


def point_bounds(points: list[Point2], padding_m: float) -> tuple[float, float, float, float]:
    if not points:
        raise ValueError("no map points available")
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs) - padding_m, max(xs) + padding_m, min(ys) - padding_m, max(ys) + padding_m


def render_svg(
    output: Path,
    points: list[Point2],
    candidates: list[dict[str, Any]],
    session_id: str,
    max_points_in_svg: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    min_x, max_x, min_y, max_y = point_bounds(points, padding_m=0.4)
    world_width = max_x - min_x
    world_height = max_y - min_y

    width, height = 1500, 1050
    margin_left, margin_right, margin_top, margin_bottom = 88, 42, 118, 132
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom
    scale = min(plot_width / world_width, plot_height / world_height)

    def px(point_x: float) -> float:
        return margin_left + (point_x - min_x) * scale

    def py(point_y: float) -> float:
        return margin_top + (max_y - point_y) * scale

    point_step = max(1, math.ceil(len(points) / max_points_in_svg))
    displayed_points = points[::point_step]

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        (
            f'<text x="48" y="52" fill="#f8fafc" font-size="30" font-weight="700">'
            f'Session {html.escape(session_id)} lidar landmark candidates</text>'
        ),
        (
            f'<text x="48" y="82" fill="#94a3b8" font-size="16">'
            'Pick the two numbered line segments corresponding to the measured reference boards.</text>'
        ),
        (
            f'<rect x="{margin_left}" y="{margin_top}" width="{plot_width}" '
            f'height="{plot_height}" rx="18" fill="#0d1726" stroke="#334155"/>'
        ),
    ]

    for x_m in range(math.floor(min_x), math.ceil(max_x) + 1):
        x = px(float(x_m))
        svg.append(
            f'<line x1="{x:.1f}" y1="{margin_top}" x2="{x:.1f}" '
            f'y2="{margin_top + plot_height}" stroke="#223246" stroke-width="1"/>'
        )
        svg.append(
            f'<text x="{x + 4:.1f}" y="{margin_top + plot_height + 24}" '
            f'fill="#64748b" font-size="12">{x_m}m</text>'
        )
    for y_m in range(math.floor(min_y), math.ceil(max_y) + 1):
        y = py(float(y_m))
        svg.append(
            f'<line x1="{margin_left}" y1="{y:.1f}" '
            f'x2="{margin_left + plot_width}" y2="{y:.1f}" '
            'stroke="#223246" stroke-width="1"/>'
        )
        svg.append(
            f'<text x="32" y="{y + 4:.1f}" fill="#64748b" font-size="12">{y_m}m</text>'
        )

    for x_m, y_m in displayed_points:
        svg.append(
            f'<circle cx="{px(x_m):.1f}" cy="{py(y_m):.1f}" r="1.2" '
            'fill="#94a3b8" fill-opacity="0.30"/>'
        )

    for candidate in candidates:
        color = COLORS[(candidate["id"] - 1) % len(COLORS)]
        start = candidate["start"]
        end = candidate["end"]
        center = candidate["center"]
        svg.append(
            f'<line x1="{px(start["x_m"]):.1f}" y1="{py(start["y_m"]):.1f}" '
            f'x2="{px(end["x_m"]):.1f}" y2="{py(end["y_m"]):.1f}" '
            f'stroke="{color}" stroke-width="5" stroke-linecap="round"/>'
        )
        label_x = px(center["x_m"])
        label_y = py(center["y_m"])
        svg.append(
            f'<circle cx="{label_x:.1f}" cy="{label_y:.1f}" r="16" '
            f'fill="{color}" stroke="#020617" stroke-width="2"/>'
        )
        svg.append(
            f'<text x="{label_x:.1f}" y="{label_y + 5:.1f}" text-anchor="middle" '
            'fill="#020617" font-size="16" font-weight="800">'
            f'{candidate["id"]}</text>'
        )

    legend_y = height - 96
    svg.extend(
        [
            f'<rect x="48" y="{legend_y - 42}" width="{width - 96}" height="94" '
            'rx="16" fill="#0d1726" stroke="#334155"/>',
            (
                f'<text x="70" y="{legend_y - 14}" fill="#f8fafc" font-size="17" '
                f'font-weight="700">Detected line candidates: {len(candidates)}</text>'
            ),
            (
                f'<text x="70" y="{legend_y + 12}" fill="#94a3b8" font-size="14">'
                f'Map points: {len(points):,}; displayed points: {len(displayed_points):,}</text>'
            ),
            (
                f'<text x="70" y="{legend_y + 36}" fill="#94a3b8" font-size="14">'
                'Use --measure-ids A B --expected-distance-m DIST after identifying the two reference boards.</text>'
            ),
            "</svg>",
        ]
    )
    output.write_text("\n".join(svg), encoding="utf-8")


def write_json(output: Path, candidates: list[dict[str, Any]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"candidates": candidates}, indent=2) + "\n", encoding="utf-8")


def measure_candidates(
    candidates: list[dict[str, Any]],
    first_id: int,
    second_id: int,
    expected_distance_m: float | None,
) -> str:
    by_id = {candidate["id"]: candidate for candidate in candidates}
    if first_id not in by_id or second_id not in by_id:
        raise ValueError(f"candidate IDs must be in {sorted(by_id)}")

    first = by_id[first_id]["center"]
    second = by_id[second_id]["center"]
    dx = second["x_m"] - first["x_m"]
    dy = second["y_m"] - first["y_m"]
    distance = math.hypot(dx, dy)
    lines = [
        f"Candidate {first_id} center: ({first['x_m']:.3f}, {first['y_m']:.3f}) m",
        f"Candidate {second_id} center: ({second['x_m']:.3f}, {second['y_m']:.3f}) m",
        f"Measured center distance: {distance:.3f} m",
        f"Delta vector: dx={dx:.3f} m, dy={dy:.3f} m",
    ]
    if expected_distance_m is not None:
        error = distance - expected_distance_m
        percent = 100.0 * error / expected_distance_m if expected_distance_m else 0.0
        lines.extend(
            [
                f"Expected distance: {expected_distance_m:.3f} m",
                f"Error: {error:+.3f} m ({percent:+.1f}%)",
            ]
        )
    return "\n".join(lines)


def parse_measure_ids(text: str) -> tuple[int, int]:
    parts = [part.strip() for part in text.replace(",", " ").split() if part.strip()]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected exactly two candidate IDs")
    return int(parts[0]), int(parts[1])


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Detect and label straight reference landmarks in an ICP lidar map"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--trajectory", type=Path, required=True, help="ICP trajectory JSON")
    parser.add_argument("--output", type=Path, required=True, help="Output labeled SVG")
    parser.add_argument("--json-output", type=Path, help="Optional candidate JSON")
    parser.add_argument("--lidar-angle-offset-deg", type=float, default=125.0)
    parser.add_argument("--min-distance-m", type=float, default=0.20)
    parser.add_argument("--max-distance-m", type=float, default=5.0)
    parser.add_argument("--min-quality", type=int, default=1)
    parser.add_argument("--render-scan-stride", type=int, default=2)
    parser.add_argument("--render-point-stride", type=int, default=1)
    parser.add_argument("--max-valid-points-ratio", type=float, default=1.5)
    parser.add_argument("--max-candidates", type=int, default=12)
    parser.add_argument("--ransac-iterations", type=int, default=1600)
    parser.add_argument("--line-threshold-m", type=float, default=0.035)
    parser.add_argument("--min-inliers", type=int, default=220)
    parser.add_argument("--min-segment-length-m", type=float, default=0.25)
    parser.add_argument("--max-points-for-ransac", type=int, default=18000)
    parser.add_argument("--max-points-in-svg", type=int, default=70000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--measure-ids", type=parse_measure_ids)
    parser.add_argument("--expected-distance-m", type=float)
    args = parser.parse_args()

    session = args.session.resolve()
    manifest = icp.load_json(session / "manifest.json")
    scans = icp.load_scans(session / manifest["lidar"]["scans"])
    trajectory = load_trajectory(args.trajectory)
    skipped_scan_indices = icp.oversized_scan_indices(scans, args.max_valid_points_ratio)

    map_points, _summary = icp.collect_map_points(
        scans,
        trajectory,
        args.lidar_angle_offset_deg,
        args.min_distance_m,
        args.max_distance_m,
        args.min_quality,
        args.render_scan_stride,
        args.render_point_stride,
        skipped_scan_indices,
    )
    points_xy = [(point[0], point[1]) for point in map_points]
    candidates = detect_lines(
        points_xy,
        args.max_candidates,
        args.ransac_iterations,
        args.line_threshold_m,
        args.min_inliers,
        args.min_segment_length_m,
        args.max_points_for_ransac,
        args.seed,
    )
    render_svg(args.output, points_xy, candidates, str(manifest["session_id"]), args.max_points_in_svg)
    if args.json_output:
        write_json(args.json_output, candidates)

    print(f"Wrote {args.output}")
    if args.json_output:
        print(f"Wrote {args.json_output}")
    print(f"Map points: {len(points_xy)}")
    print(f"Detected candidates: {len(candidates)}")
    for candidate in candidates:
        center = candidate["center"]
        print(
            f"{candidate['id']:2d}: center=({center['x_m']:+.3f}, {center['y_m']:+.3f}) m "
            f"length={candidate['length_m']:.3f} m "
            f"angle={candidate['angle_deg']:+.1f} deg "
            f"inliers={candidate['inlier_count']}"
        )

    if args.measure_ids:
        print()
        print(
            measure_candidates(
                candidates,
                args.measure_ids[0],
                args.measure_ids[1],
                args.expected_distance_m,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
