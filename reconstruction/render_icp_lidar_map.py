#!/usr/bin/env python3
"""Estimate a simple 2D lidar trajectory with ICP and render a top-down map.

This is deliberately a first odometry diagnostic, not a full SLAM back-end.  It
matches nearby lidar scans during the user-marked motion window, accumulates the
relative transforms, and projects all lidar returns into the estimated map
frame.  The intended use is to learn whether lidar scan matching reduces the
smearing seen with a hand-assumed straight-line trajectory.
"""

from __future__ import annotations

import argparse
import binascii
import html
import json
import math
import struct
import zlib
from bisect import bisect_right
from pathlib import Path
from typing import Any


Point2 = tuple[float, float]
Pose2 = tuple[float, float, float]
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


def normalize_angle(angle_rad: float) -> float:
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    return angle_rad


def transform_point(pose: Pose2, point: Point2) -> Point2:
    x, y, theta = pose
    px, py = point
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return x + cos_t * px - sin_t * py, y + sin_t * px + cos_t * py


def compose_pose(first: Pose2, second: Pose2) -> Pose2:
    """Return first ∘ second, meaning apply second, then first."""

    fx, fy, ftheta = first
    sx, sy, stheta = second
    cos_f = math.cos(ftheta)
    sin_f = math.sin(ftheta)
    return (
        fx + cos_f * sx - sin_f * sy,
        fy + sin_f * sx + cos_f * sy,
        normalize_angle(ftheta + stheta),
    )


def inverse_pose(pose: Pose2) -> Pose2:
    x, y, theta = pose
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return (
        -cos_t * x - sin_t * y,
        sin_t * x - cos_t * y,
        normalize_angle(-theta),
    )


def raw_lidar_point_to_xy(
    angle_deg: float,
    distance_m: float,
    angle_offset_deg: float,
) -> Point2:
    angle_rad = math.radians(angle_deg + angle_offset_deg)
    return distance_m * math.cos(angle_rad), distance_m * math.sin(angle_rad)


def scan_to_points(
    scan: dict[str, Any],
    angle_offset_deg: float,
    min_distance_m: float,
    max_distance_m: float,
    min_quality: int,
    point_stride: int,
) -> list[Point2]:
    points: list[Point2] = []
    for point_index, point in enumerate(scan["points"]):
        if point_index % point_stride != 0:
            continue
        angle_deg, distance_m, quality = float(point[0]), float(point[1]), int(point[2])
        if (
            distance_m < min_distance_m
            or distance_m > max_distance_m
            or quality < min_quality
        ):
            continue
        points.append(raw_lidar_point_to_xy(angle_deg, distance_m, angle_offset_deg))
    return points


def build_grid(points: list[Point2], cell_size_m: float) -> dict[tuple[int, int], list[Point2]]:
    grid: dict[tuple[int, int], list[Point2]] = {}
    for x, y in points:
        cell = math.floor(x / cell_size_m), math.floor(y / cell_size_m)
        grid.setdefault(cell, []).append((x, y))
    return grid


def nearest_point(
    point: Point2,
    grid: dict[tuple[int, int], list[Point2]],
    cell_size_m: float,
    max_distance_m: float,
) -> tuple[Point2, float] | None:
    x, y = point
    cell_x = math.floor(x / cell_size_m)
    cell_y = math.floor(y / cell_size_m)
    max_distance_sq = max_distance_m * max_distance_m
    best_point: Point2 | None = None
    best_distance_sq = max_distance_sq
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for candidate in grid.get((cell_x + dx, cell_y + dy), []):
                candidate_x, candidate_y = candidate
                distance_sq = (candidate_x - x) ** 2 + (candidate_y - y) ** 2
                if distance_sq < best_distance_sq:
                    best_distance_sq = distance_sq
                    best_point = candidate
    if best_point is None:
        return None
    return best_point, best_distance_sq


def best_fit_transform(source_points: list[Point2], target_points: list[Point2]) -> Pose2:
    if len(source_points) != len(target_points):
        raise ValueError("source and target correspondence counts differ")
    if len(source_points) < 2:
        return 0.0, 0.0, 0.0

    source_cx = sum(point[0] for point in source_points) / len(source_points)
    source_cy = sum(point[1] for point in source_points) / len(source_points)
    target_cx = sum(point[0] for point in target_points) / len(target_points)
    target_cy = sum(point[1] for point in target_points) / len(target_points)

    cross = 0.0
    dot = 0.0
    for source, target in zip(source_points, target_points):
        sx = source[0] - source_cx
        sy = source[1] - source_cy
        tx = target[0] - target_cx
        ty = target[1] - target_cy
        cross += sx * ty - sy * tx
        dot += sx * tx + sy * ty

    theta = math.atan2(cross, dot)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    translation_x = target_cx - (cos_t * source_cx - sin_t * source_cy)
    translation_y = target_cy - (sin_t * source_cx + cos_t * source_cy)
    return translation_x, translation_y, theta


def icp_current_to_previous(
    current_points: list[Point2],
    previous_points: list[Point2],
    max_pair_distance_m: float,
    trim_fraction: float,
    iterations: int,
    min_pairs: int,
) -> tuple[Pose2, dict[str, float]]:
    """Estimate transform that maps current-frame points into previous frame."""

    if len(current_points) < min_pairs or len(previous_points) < min_pairs:
        return (0.0, 0.0, 0.0), {"pairs": 0, "rmse_m": float("inf"), "iterations": 0}

    grid = build_grid(previous_points, max_pair_distance_m)
    estimate: Pose2 = (0.0, 0.0, 0.0)
    best_rmse = float("inf")
    used_pairs = 0
    completed_iterations = 0

    for iteration in range(iterations):
        matches: list[tuple[float, Point2, Point2]] = []
        for point in current_points:
            transformed = transform_point(estimate, point)
            nearest = nearest_point(
                transformed,
                grid,
                max_pair_distance_m,
                max_pair_distance_m,
            )
            if nearest is None:
                continue
            target, distance_sq = nearest
            matches.append((distance_sq, transformed, target))

        if len(matches) < min_pairs:
            break

        matches.sort(key=lambda item: item[0])
        keep_count = max(min_pairs, int(len(matches) * trim_fraction))
        trimmed = matches[:keep_count]
        source_for_increment = [item[1] for item in trimmed]
        target_for_increment = [item[2] for item in trimmed]
        increment = best_fit_transform(source_for_increment, target_for_increment)
        estimate = compose_pose(increment, estimate)

        rmse = math.sqrt(sum(item[0] for item in trimmed) / len(trimmed))
        best_rmse = rmse
        used_pairs = len(trimmed)
        completed_iterations = iteration + 1

        if math.hypot(increment[0], increment[1]) < 1e-4 and abs(increment[2]) < 1e-4:
            break

    return estimate, {
        "pairs": float(used_pairs),
        "rmse_m": best_rmse,
        "iterations": float(completed_iterations),
    }


def elapsed_seconds(scans: list[dict[str, Any]], scan_index: int) -> float:
    first_timestamp_us = int(scans[0]["timestamp_us"])
    return (int(scans[scan_index]["timestamp_us"]) - first_timestamp_us) / 1e6


def motion_scan_indices(
    scans: list[dict[str, Any]],
    motion_start_s: float,
    motion_end_s: float,
    match_stride: int,
) -> list[int]:
    indices = [
        scan_index
        for scan_index in range(len(scans))
        if motion_start_s <= elapsed_seconds(scans, scan_index) <= motion_end_s
    ]
    if not indices:
        raise ValueError("no scans fall inside the requested motion window")

    selected = indices[::match_stride]
    if selected[-1] != indices[-1]:
        selected.append(indices[-1])
    return selected


def estimate_trajectory(
    scans: list[dict[str, Any]],
    angle_offset_deg: float,
    motion_start_s: float,
    motion_end_s: float,
    match_stride: int,
    match_point_stride: int,
    min_distance_m: float,
    max_distance_m: float,
    min_quality: int,
    max_pair_distance_m: float,
    trim_fraction: float,
    icp_iterations: int,
    min_pairs: int,
    max_step_translation_m: float,
    max_step_rotation_deg: float,
) -> tuple[list[tuple[int, Pose2]], dict[str, Any]]:
    selected_indices = motion_scan_indices(scans, motion_start_s, motion_end_s, match_stride)
    selected_points = {
        scan_index: scan_to_points(
            scans[scan_index],
            angle_offset_deg,
            min_distance_m,
            max_distance_m,
            min_quality,
            match_point_stride,
        )
        for scan_index in selected_indices
    }

    trajectory: list[tuple[int, Pose2]] = [(selected_indices[0], (0.0, 0.0, 0.0))]
    icp_records: list[dict[str, Any]] = []
    rejected_steps = 0

    max_step_rotation_rad = math.radians(max_step_rotation_deg)
    for previous_index, current_index in zip(selected_indices, selected_indices[1:]):
        previous_pose = trajectory[-1][1]
        relative, stats = icp_current_to_previous(
            selected_points[current_index],
            selected_points[previous_index],
            max_pair_distance_m,
            trim_fraction,
            icp_iterations,
            min_pairs,
        )
        step_translation = math.hypot(relative[0], relative[1])
        step_rotation = abs(relative[2])
        accepted = (
            stats["pairs"] >= min_pairs
            and step_translation <= max_step_translation_m
            and step_rotation <= max_step_rotation_rad
            and math.isfinite(stats["rmse_m"])
        )
        if not accepted:
            relative = (0.0, 0.0, 0.0)
            rejected_steps += 1

        current_pose = compose_pose(previous_pose, relative)
        rmse_m = stats["rmse_m"] if math.isfinite(stats["rmse_m"]) else None
        trajectory.append((current_index, current_pose))
        icp_records.append(
            {
                "previous_scan": previous_index,
                "current_scan": current_index,
                "accepted": accepted,
                "dx_m": relative[0],
                "dy_m": relative[1],
                "dtheta_deg": math.degrees(relative[2]),
                "step_translation_m": step_translation,
                "step_rotation_deg": math.degrees(step_rotation),
                "pairs": int(stats["pairs"]),
                "rmse_m": rmse_m,
                "iterations": int(stats["iterations"]),
            }
        )

    final_pose = trajectory[-1][1]
    step_lengths = [
        math.hypot(record["dx_m"], record["dy_m"]) for record in icp_records if record["accepted"]
    ]
    summary = {
        "selected_scan_count": len(selected_indices),
        "icp_step_count": len(icp_records),
        "rejected_steps": rejected_steps,
        "estimated_path_length_m": sum(step_lengths),
        "estimated_net_displacement_m": math.hypot(final_pose[0], final_pose[1]),
        "estimated_net_rotation_deg": math.degrees(final_pose[2]),
        "mean_accepted_step_m": sum(step_lengths) / len(step_lengths) if step_lengths else 0.0,
        "icp_records": icp_records,
    }
    return trajectory, summary


def interpolate_angle(start: float, end: float, fraction: float) -> float:
    return normalize_angle(start + normalize_angle(end - start) * fraction)


def pose_at_scan(scan_index: int, trajectory: list[tuple[int, Pose2]]) -> Pose2:
    indices = [item[0] for item in trajectory]
    if scan_index <= indices[0]:
        return trajectory[0][1]
    if scan_index >= indices[-1]:
        return trajectory[-1][1]

    right_position = bisect_right(indices, scan_index)
    left_index, left_pose = trajectory[right_position - 1]
    right_index, right_pose = trajectory[right_position]
    fraction = (scan_index - left_index) / (right_index - left_index)
    return (
        left_pose[0] + (right_pose[0] - left_pose[0]) * fraction,
        left_pose[1] + (right_pose[1] - left_pose[1]) * fraction,
        interpolate_angle(left_pose[2], right_pose[2], fraction),
    )


def collect_map_points(
    scans: list[dict[str, Any]],
    trajectory: list[tuple[int, Pose2]],
    angle_offset_deg: float,
    min_distance_m: float,
    max_distance_m: float,
    min_quality: int,
    render_scan_stride: int,
    render_point_stride: int,
) -> tuple[list[ColoredPoint], dict[str, Any]]:
    first_timestamp_us = int(scans[0]["timestamp_us"])
    last_timestamp_us = int(scans[-1]["timestamp_us"])
    duration_s = (last_timestamp_us - first_timestamp_us) / 1e6
    points: list[ColoredPoint] = []
    considered_scans = 0
    considered_returns = 0
    filtered_returns = 0

    for scan_index, scan in enumerate(scans):
        if scan_index % render_scan_stride != 0:
            continue
        considered_scans += 1
        pose = pose_at_scan(scan_index, trajectory)
        elapsed_s = (int(scan["timestamp_us"]) - first_timestamp_us) / 1e6
        time_fraction = elapsed_s / duration_s if duration_s > 0 else 0.0
        for point_index, point in enumerate(scan["points"]):
            if point_index % render_point_stride != 0:
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
            local = raw_lidar_point_to_xy(angle_deg, distance_m, angle_offset_deg)
            world_x, world_y = transform_point(pose, local)
            points.append((world_x, world_y, 0.0, time_fraction))

    return points, {
        "input_scan_count": len(scans),
        "used_scan_count": considered_scans,
        "considered_returns": considered_returns,
        "filtered_returns": filtered_returns,
        "output_points": len(points),
        "lidar_duration_s": duration_s,
    }


def point_bounds(points: list[ColoredPoint], padding_m: float) -> tuple[float, float, float, float]:
    if not points:
        raise ValueError("no output points survived filtering")
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs) - padding_m, max(xs) + padding_m, min(ys) - padding_m, max(ys) + padding_m


def time_color(fraction: float) -> str:
    fraction = max(0.0, min(1.0, fraction))
    if fraction < 0.5:
        t = fraction / 0.5
        red = round(56 + (250 - 56) * t)
        green = round(189 + (204 - 189) * t)
        blue = round(248 + (21 - 248) * t)
    else:
        t = (fraction - 0.5) / 0.5
        red = round(250 + (251 - 250) * t)
        green = round(204 + (113 - 204) * t)
        blue = round(21 + (133 - 21) * t)
    return f"#{red:02x}{green:02x}{blue:02x}"


def hex_to_rgb(color: str) -> tuple[int, int, int]:
    color = color.lstrip("#")
    return int(color[0:2], 16), int(color[2:4], 16), int(color[4:6], 16)


def render_svg(
    output: Path,
    points: list[ColoredPoint],
    trajectory: list[tuple[int, Pose2]],
    session_id: str,
    angle_offset_deg: float,
    motion_start_s: float,
    motion_end_s: float,
    map_summary: dict[str, Any],
    trajectory_summary: dict[str, Any],
    max_points_in_svg: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    min_x, max_x, min_y, max_y = point_bounds(points, padding_m=0.4)
    world_width = max_x - min_x
    world_height = max_y - min_y

    width, height = 1500, 1050
    margin_left, margin_right, margin_top, margin_bottom = 88, 42, 132, 134
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
            f'Session {html.escape(session_id)} ICP-estimated lidar map</text>'
        ),
        (
            f'<text x="48" y="82" fill="#94a3b8" font-size="16">'
            f'ICP odometry, motion window {motion_start_s:.1f}-{motion_end_s:.1f}s, '
            f'lidar angle offset {angle_offset_deg:.1f}°</text>'
        ),
        (
            f'<text x="48" y="106" fill="#fbbf24" font-size="14">'
            'Diagnostic only: no loop closure, no occupancy grid, no global SLAM.</text>'
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

    for x_m, y_m, _z_m, fraction in displayed_points:
        svg.append(
            f'<circle cx="{px(x_m):.1f}" cy="{py(y_m):.1f}" r="1.4" '
            f'fill="{time_color(fraction)}" fill-opacity="0.58"/>'
        )

    trajectory_points = [pose for _scan_index, pose in trajectory]
    if len(trajectory_points) >= 2:
        path_data = " ".join(
            f'{"M" if index == 0 else "L"} {px(pose[0]):.1f} {py(pose[1]):.1f}'
            for index, pose in enumerate(trajectory_points)
        )
        svg.append(
            f'<path d="{path_data}" fill="none" stroke="#f8fafc" '
            'stroke-width="4" stroke-linejoin="round" stroke-linecap="round"/>'
        )

    start_pose = trajectory_points[0]
    end_pose = trajectory_points[-1]
    svg.extend(
        [
            f'<circle cx="{px(start_pose[0]):.1f}" cy="{py(start_pose[1]):.1f}" '
            'r="8" fill="#22c55e"/>',
            f'<circle cx="{px(end_pose[0]):.1f}" cy="{py(end_pose[1]):.1f}" '
            'r="8" fill="#ef4444"/>',
            f'<text x="{px(start_pose[0]) + 12:.1f}" y="{py(start_pose[1]) - 10:.1f}" '
            'fill="#bbf7d0" font-size="14" font-weight="700">start</text>',
            f'<text x="{px(end_pose[0]) + 12:.1f}" y="{py(end_pose[1]) - 10:.1f}" '
            'fill="#fecaca" font-size="14" font-weight="700">end</text>',
        ]
    )

    legend_y = height - 96
    svg.extend(
        [
            f'<rect x="48" y="{legend_y - 42}" width="{width - 96}" height="94" '
            'rx="16" fill="#0d1726" stroke="#334155"/>',
            (
                f'<text x="70" y="{legend_y - 14}" fill="#f8fafc" font-size="17" '
                f'font-weight="700">Estimated path: '
                f'{trajectory_summary["estimated_path_length_m"]:.3f} m; net '
                f'{trajectory_summary["estimated_net_displacement_m"]:.3f} m; '
                f'rotation {trajectory_summary["estimated_net_rotation_deg"]:.1f}°</text>'
            ),
            (
                f'<text x="70" y="{legend_y + 12}" fill="#94a3b8" font-size="14">'
                f'ICP steps: {trajectory_summary["icp_step_count"]}; rejected: '
                f'{trajectory_summary["rejected_steps"]}; selected scans: '
                f'{trajectory_summary["selected_scan_count"]}; map points: '
                f'{map_summary["output_points"]:,} ({len(displayed_points):,} displayed)</text>'
            ),
            (
                f'<text x="70" y="{legend_y + 36}" fill="#94a3b8" font-size="14">'
                f'Used scans: {map_summary["used_scan_count"]}/{map_summary["input_scan_count"]}; '
                f'filtered returns: {map_summary["filtered_returns"]:,}/'
                f'{map_summary["considered_returns"]:,}; duration: '
                f'{map_summary["lidar_duration_s"]:.2f}s</text>'
            ),
            "</svg>",
        ]
    )
    output.write_text("\n".join(svg), encoding="utf-8")


def blend_pixel(
    pixels: bytearray,
    width: int,
    height: int,
    x: int,
    y: int,
    color: tuple[int, int, int],
    alpha: float,
) -> None:
    if x < 0 or x >= width or y < 0 or y >= height:
        return
    offset = (y * width + x) * 3
    inv_alpha = 1.0 - alpha
    pixels[offset] = round(pixels[offset] * inv_alpha + color[0] * alpha)
    pixels[offset + 1] = round(pixels[offset + 1] * inv_alpha + color[1] * alpha)
    pixels[offset + 2] = round(pixels[offset + 2] * inv_alpha + color[2] * alpha)


def draw_circle(
    pixels: bytearray,
    width: int,
    height: int,
    cx: int,
    cy: int,
    radius: int,
    color: tuple[int, int, int],
    alpha: float,
) -> None:
    radius_sq = radius * radius
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius_sq:
                blend_pixel(pixels, width, height, x, y, color, alpha)


def draw_line(
    pixels: bytearray,
    width: int,
    height: int,
    start: tuple[int, int],
    end: tuple[int, int],
    color: tuple[int, int, int],
    alpha: float,
    thickness: int,
) -> None:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x, y = x0, y0
    while True:
        draw_circle(pixels, width, height, x, y, thickness, color, alpha)
        if x == x1 and y == y1:
            break
        error2 = 2 * error
        if error2 >= dy:
            error += dy
            x += sx
        if error2 <= dx:
            error += dx
            y += sy


def png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + chunk_type
        + data
        + struct.pack(">I", binascii.crc32(chunk_type + data) & 0xFFFFFFFF)
    )


def write_png_rgb(output: Path, width: int, height: int, pixels: bytearray) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = [
        b"\x00" + bytes(pixels[row * width * 3 : (row + 1) * width * 3])
        for row in range(height)
    ]
    payload = b"".join(rows)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + png_chunk(b"IDAT", zlib.compress(payload, level=9))
        + png_chunk(b"IEND", b"")
    )
    output.write_bytes(png)


def render_png(
    output: Path,
    points: list[ColoredPoint],
    trajectory: list[tuple[int, Pose2]],
    max_points_in_png: int,
) -> None:
    """Render a lightweight raster preview without external dependencies."""

    min_x, max_x, min_y, max_y = point_bounds(points, padding_m=0.4)
    world_width = max_x - min_x
    world_height = max_y - min_y
    width, height = 1500, 1050
    margin_left, margin_right, margin_top, margin_bottom = 88, 42, 80, 86
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom
    scale = min(plot_width / world_width, plot_height / world_height)

    def px(point_x: float) -> int:
        return round(margin_left + (point_x - min_x) * scale)

    def py(point_y: float) -> int:
        return round(margin_top + (max_y - point_y) * scale)

    background = (7, 17, 31)
    plot_background = (13, 23, 38)
    grid_color = (34, 50, 70)
    pixels = bytearray(background * (width * height))

    for y in range(margin_top, margin_top + plot_height):
        for x in range(margin_left, margin_left + plot_width):
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = bytes(plot_background)

    for x_m in range(math.floor(min_x), math.ceil(max_x) + 1):
        x = px(float(x_m))
        draw_line(
            pixels,
            width,
            height,
            (x, margin_top),
            (x, margin_top + plot_height),
            grid_color,
            0.55,
            0,
        )
    for y_m in range(math.floor(min_y), math.ceil(max_y) + 1):
        y = py(float(y_m))
        draw_line(
            pixels,
            width,
            height,
            (margin_left, y),
            (margin_left + plot_width, y),
            grid_color,
            0.55,
            0,
        )

    point_step = max(1, math.ceil(len(points) / max_points_in_png))
    for x_m, y_m, _z_m, fraction in points[::point_step]:
        draw_circle(
            pixels,
            width,
            height,
            px(x_m),
            py(y_m),
            1,
            hex_to_rgb(time_color(fraction)),
            0.62,
        )

    trajectory_pixels = [(px(pose[0]), py(pose[1])) for _scan_index, pose in trajectory]
    for start, end in zip(trajectory_pixels, trajectory_pixels[1:]):
        draw_line(pixels, width, height, start, end, (248, 250, 252), 0.95, 2)

    if trajectory_pixels:
        draw_circle(pixels, width, height, *trajectory_pixels[0], 7, (34, 197, 94), 1.0)
        draw_circle(pixels, width, height, *trajectory_pixels[-1], 7, (239, 68, 68), 1.0)

    write_png_rgb(output, width, height, pixels)


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
    for x_m, y_m, z_m, fraction in points:
        color = time_color(fraction).lstrip("#")
        lines.append(
            f"{x_m:.4f} {y_m:.4f} {z_m:.4f} "
            f"{int(color[0:2], 16)} {int(color[2:4], 16)} {int(color[4:6], 16)}"
        )
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_trajectory_json(
    output: Path,
    trajectory: list[tuple[int, Pose2]],
    summary: dict[str, Any],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "coordinate_frame": "2D map frame; x/y are meters; theta is radians",
        "summary": summary,
        "poses": [
            {"scan_index": scan_index, "x_m": pose[0], "y_m": pose[1], "theta_rad": pose[2]}
            for scan_index, pose in trajectory
        ],
    }
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Estimate a rough 2D lidar trajectory with ICP and render a map"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--output", type=Path, required=True, help="Output SVG path")
    parser.add_argument("--ply-output", type=Path, help="Optional output PLY path")
    parser.add_argument("--png-output", type=Path, help="Optional raster preview PNG path")
    parser.add_argument("--trajectory-output", type=Path, help="Optional trajectory JSON path")
    parser.add_argument("--motion-start-s", type=float, default=5.0)
    parser.add_argument("--motion-end-s", type=float, default=25.0)
    parser.add_argument("--lidar-angle-offset-deg", type=float, default=125.0)
    parser.add_argument("--min-distance-m", type=float, default=0.20)
    parser.add_argument("--max-distance-m", type=float, default=5.0)
    parser.add_argument("--min-quality", type=int, default=1)
    parser.add_argument("--match-stride", type=int, default=4)
    parser.add_argument("--match-point-stride", type=int, default=2)
    parser.add_argument("--render-scan-stride", type=int, default=2)
    parser.add_argument("--render-point-stride", type=int, default=1)
    parser.add_argument("--max-pair-distance-m", type=float, default=0.15)
    parser.add_argument("--trim-fraction", type=float, default=0.70)
    parser.add_argument("--icp-iterations", type=int, default=20)
    parser.add_argument("--min-pairs", type=int, default=80)
    parser.add_argument("--max-step-translation-m", type=float, default=0.12)
    parser.add_argument("--max-step-rotation-deg", type=float, default=8.0)
    parser.add_argument("--max-points-in-svg", type=int, default=80000)
    parser.add_argument("--max-points-in-png", type=int, default=120000)
    args = parser.parse_args()

    if args.motion_end_s <= args.motion_start_s:
        parser.error("--motion-end-s must be greater than --motion-start-s")
    if args.match_stride <= 0 or args.match_point_stride <= 0:
        parser.error("--match-stride and --match-point-stride must be positive")
    if args.render_scan_stride <= 0 or args.render_point_stride <= 0:
        parser.error("--render-scan-stride and --render-point-stride must be positive")
    if not 0.0 < args.trim_fraction <= 1.0:
        parser.error("--trim-fraction must be in the range (0, 1]")
    if args.max_points_in_svg <= 0 or args.max_points_in_png <= 0:
        parser.error("--max-points-in-svg and --max-points-in-png must be positive")

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    scans = load_scans(session / manifest["lidar"]["scans"])

    trajectory, trajectory_summary = estimate_trajectory(
        scans,
        args.lidar_angle_offset_deg,
        args.motion_start_s,
        args.motion_end_s,
        args.match_stride,
        args.match_point_stride,
        args.min_distance_m,
        args.max_distance_m,
        args.min_quality,
        args.max_pair_distance_m,
        args.trim_fraction,
        args.icp_iterations,
        args.min_pairs,
        args.max_step_translation_m,
        args.max_step_rotation_deg,
    )
    points, map_summary = collect_map_points(
        scans,
        trajectory,
        args.lidar_angle_offset_deg,
        args.min_distance_m,
        args.max_distance_m,
        args.min_quality,
        args.render_scan_stride,
        args.render_point_stride,
    )

    render_svg(
        args.output,
        points,
        trajectory,
        str(manifest["session_id"]),
        args.lidar_angle_offset_deg,
        args.motion_start_s,
        args.motion_end_s,
        map_summary,
        trajectory_summary,
        args.max_points_in_svg,
    )
    if args.ply_output:
        write_ply(args.ply_output, points)
    if args.png_output:
        render_png(args.png_output, points, trajectory, args.max_points_in_png)
    if args.trajectory_output:
        write_trajectory_json(args.trajectory_output, trajectory, trajectory_summary)

    print(f"Wrote {args.output}")
    if args.ply_output:
        print(f"Wrote {args.ply_output}")
    if args.png_output:
        print(f"Wrote {args.png_output}")
    if args.trajectory_output:
        print(f"Wrote {args.trajectory_output}")
    print(f"Input scans: {map_summary['input_scan_count']}")
    print(f"Selected scans for ICP: {trajectory_summary['selected_scan_count']}")
    print(f"ICP steps: {trajectory_summary['icp_step_count']}")
    print(f"Rejected ICP steps: {trajectory_summary['rejected_steps']}")
    print(f"Estimated path length: {trajectory_summary['estimated_path_length_m']:.3f} m")
    print(f"Estimated net displacement: {trajectory_summary['estimated_net_displacement_m']:.3f} m")
    print(f"Estimated net rotation: {trajectory_summary['estimated_net_rotation_deg']:.2f} deg")
    print(f"Map output points: {map_summary['output_points']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
