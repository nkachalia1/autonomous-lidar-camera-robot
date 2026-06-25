#!/usr/bin/env python3
"""Render sampled camera frames on an ICP-derived lidar trajectory.

This is the first camera/lidar fusion artifact: it does not reconstruct 3D
geometry yet.  It proves that camera frame timestamps can be synchronized to the
metric lidar trajectory, producing a pose for each sampled video frame.
"""

from __future__ import annotations

import argparse
import html
import json
import math
from bisect import bisect_right
from pathlib import Path
from typing import Any

import render_icp_lidar_map as icp
from detect_lidar_landmarks import load_trajectory


Pose2 = tuple[float, float, float]
ColoredPoint = tuple[float, float, float, float]


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def normalize_angle(angle_rad: float) -> float:
    return icp.normalize_angle(angle_rad)


def interpolate_angle(start: float, end: float, fraction: float) -> float:
    return normalize_angle(start + normalize_angle(end - start) * fraction)


def pose_at_timestamp_ns(
    timestamp_ns: int,
    trajectory: list[tuple[int, Pose2]],
    scans: list[dict[str, Any]],
) -> Pose2:
    scan_timestamps_ns = [int(scans[scan_index]["timestamp_us"]) * 1000 for scan_index, _ in trajectory]
    if timestamp_ns <= scan_timestamps_ns[0]:
        return trajectory[0][1]
    if timestamp_ns >= scan_timestamps_ns[-1]:
        return trajectory[-1][1]

    right = bisect_right(scan_timestamps_ns, timestamp_ns)
    left_timestamp = scan_timestamps_ns[right - 1]
    right_timestamp = scan_timestamps_ns[right]
    left_pose = trajectory[right - 1][1]
    right_pose = trajectory[right][1]
    fraction = (timestamp_ns - left_timestamp) / (right_timestamp - left_timestamp)
    return (
        left_pose[0] + (right_pose[0] - left_pose[0]) * fraction,
        left_pose[1] + (right_pose[1] - left_pose[1]) * fraction,
        interpolate_angle(left_pose[2], right_pose[2], fraction),
    )


def transform_offset(pose: Pose2, forward_m: float, left_m: float) -> tuple[float, float]:
    x, y, theta = pose
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return (
        x + cos_t * forward_m - sin_t * left_m,
        y + sin_t * forward_m + cos_t * left_m,
    )


def sample_frame_indices(
    camera_metadata: list[dict[str, Any]],
    sample_count: int,
    sample_start_s: float | None,
    sample_end_s: float | None,
) -> list[int]:
    if sample_count <= 0:
        raise ValueError("sample_count must be positive")
    timestamps = [
        frame["SensorTimestamp"]
        for frame in camera_metadata
        if isinstance(frame, dict) and isinstance(frame.get("SensorTimestamp"), int)
    ]
    if len(timestamps) != len(camera_metadata):
        raise ValueError("camera metadata must contain SensorTimestamp for every frame")
    if not timestamps:
        raise ValueError("camera metadata is empty")

    first_timestamp = timestamps[0]
    start_ns = (
        first_timestamp + round(sample_start_s * 1e9)
        if sample_start_s is not None
        else timestamps[0]
    )
    end_ns = (
        first_timestamp + round(sample_end_s * 1e9)
        if sample_end_s is not None
        else timestamps[-1]
    )
    start_ns = max(start_ns, timestamps[0])
    end_ns = min(end_ns, timestamps[-1])
    if end_ns < start_ns:
        raise ValueError("sample end precedes sample start")

    if sample_count == 1:
        targets = [(start_ns + end_ns) // 2]
    else:
        targets = [
            round(start_ns + (end_ns - start_ns) * index / (sample_count - 1))
            for index in range(sample_count)
        ]

    sampled: list[int] = []
    for target in targets:
        right = bisect_right(timestamps, target)
        candidates = []
        if right > 0:
            candidates.append(right - 1)
        if right < len(timestamps):
            candidates.append(right)
        best = min(candidates, key=lambda index: abs(timestamps[index] - target))
        if not sampled or best != sampled[-1]:
            sampled.append(best)
    return sampled


def build_frame_samples(
    camera_metadata: list[dict[str, Any]],
    trajectory: list[tuple[int, Pose2]],
    scans: list[dict[str, Any]],
    sample_count: int,
    sample_start_s: float | None,
    sample_end_s: float | None,
    camera_forward_m: float,
    camera_left_m: float,
) -> list[dict[str, Any]]:
    indices = sample_frame_indices(
        camera_metadata,
        sample_count,
        sample_start_s,
        sample_end_s,
    )
    first_timestamp = int(camera_metadata[0]["SensorTimestamp"])
    samples: list[dict[str, Any]] = []
    for sample_number, frame_index in enumerate(indices, start=1):
        timestamp_ns = int(camera_metadata[frame_index]["SensorTimestamp"])
        rig_pose = pose_at_timestamp_ns(timestamp_ns, trajectory, scans)
        camera_x, camera_y = transform_offset(rig_pose, camera_forward_m, camera_left_m)
        samples.append(
            {
                "sample_number": sample_number,
                "frame_index": frame_index,
                "video_time_s": (timestamp_ns - first_timestamp) / 1e9,
                "sensor_timestamp_ns": timestamp_ns,
                "rig_pose": {
                    "x_m": rig_pose[0],
                    "y_m": rig_pose[1],
                    "theta_rad": rig_pose[2],
                    "theta_deg": math.degrees(rig_pose[2]),
                },
                "camera_pose": {
                    "x_m": camera_x,
                    "y_m": camera_y,
                    "theta_rad": rig_pose[2],
                    "theta_deg": math.degrees(rig_pose[2]),
                },
            }
        )
    return samples


def point_bounds(
    points: list[ColoredPoint],
    camera_samples: list[dict[str, Any]],
    padding_m: float,
) -> tuple[float, float, float, float]:
    if not points and not camera_samples:
        raise ValueError("no map points or camera samples available")
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    xs.extend(sample["camera_pose"]["x_m"] for sample in camera_samples)
    ys.extend(sample["camera_pose"]["y_m"] for sample in camera_samples)
    return min(xs) - padding_m, max(xs) + padding_m, min(ys) - padding_m, max(ys) + padding_m


def color_for_sample(index: int, count: int) -> str:
    if count <= 1:
        fraction = 0.0
    else:
        fraction = index / (count - 1)
    return icp.time_color(fraction)


def render_svg(
    output: Path,
    session_id: str,
    map_points: list[ColoredPoint],
    trajectory: list[tuple[int, Pose2]],
    camera_samples: list[dict[str, Any]],
    max_points_in_svg: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    min_x, max_x, min_y, max_y = point_bounds(map_points, camera_samples, padding_m=0.4)
    world_width = max_x - min_x
    world_height = max_y - min_y

    width, height = 1600, 1120
    margin_left, margin_right, margin_top, margin_bottom = 88, 42, 126, 210
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom
    scale = min(plot_width / world_width, plot_height / world_height)

    def px(point_x: float) -> float:
        return margin_left + (point_x - min_x) * scale

    def py(point_y: float) -> float:
        return margin_top + (max_y - point_y) * scale

    point_step = max(1, math.ceil(len(map_points) / max_points_in_svg))
    displayed_points = map_points[::point_step]

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        (
            f'<text x="48" y="52" fill="#f8fafc" font-size="30" font-weight="700">'
            f'Session {html.escape(session_id)} camera/lidar pose timeline</text>'
        ),
        (
            f'<text x="48" y="82" fill="#94a3b8" font-size="16">'
            'Camera frames are sampled from camera metadata and placed on the ICP lidar trajectory by monotonic timestamp.</text>'
        ),
        (
            f'<text x="48" y="106" fill="#fbbf24" font-size="14">'
            'Fusion milestone: timestamp/pose association only; no 3D visual reconstruction yet.</text>'
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

    for x_m, y_m, _z_m, time_fraction in displayed_points:
        svg.append(
            f'<circle cx="{px(x_m):.1f}" cy="{py(y_m):.1f}" r="1.3" '
            f'fill="{icp.time_color(time_fraction)}" fill-opacity="0.22"/>'
        )

    trajectory_pixels = [(px(pose[0]), py(pose[1])) for _scan_index, pose in trajectory]
    if len(trajectory_pixels) >= 2:
        path_data = " ".join(
            f'{"M" if index == 0 else "L"} {x:.1f} {y:.1f}'
            for index, (x, y) in enumerate(trajectory_pixels)
        )
        svg.append(
            f'<path d="{path_data}" fill="none" stroke="#f8fafc" '
            'stroke-width="4" stroke-linejoin="round" stroke-linecap="round"/>'
        )

    for index, sample in enumerate(camera_samples):
        pose = sample["camera_pose"]
        x = px(pose["x_m"])
        y = py(pose["y_m"])
        theta = pose["theta_rad"]
        color = color_for_sample(index, len(camera_samples))
        arrow_len = 28.0
        arrow_x = x + math.cos(theta) * arrow_len
        arrow_y = y - math.sin(theta) * arrow_len
        svg.append(
            f'<line x1="{x:.1f}" y1="{y:.1f}" x2="{arrow_x:.1f}" y2="{arrow_y:.1f}" '
            f'stroke="{color}" stroke-width="4" stroke-linecap="round"/>'
        )
        svg.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="15" fill="{color}" '
            'stroke="#020617" stroke-width="2"/>'
        )
        svg.append(
            f'<text x="{x:.1f}" y="{y + 5:.1f}" text-anchor="middle" '
            'fill="#020617" font-size="14" font-weight="800">'
            f'{sample["sample_number"]}</text>'
        )

    legend_y = height - 176
    svg.extend(
        [
            f'<rect x="48" y="{legend_y - 38}" width="{width - 96}" height="148" '
            'rx="16" fill="#0d1726" stroke="#334155"/>',
            (
                f'<text x="70" y="{legend_y - 10}" fill="#f8fafc" font-size="17" '
                f'font-weight="700">Sampled camera frames: {len(camera_samples)}; '
                f'map points displayed: {len(displayed_points):,}/{len(map_points):,}</text>'
            ),
            (
                f'<text x="70" y="{legend_y + 18}" fill="#94a3b8" font-size="14">'
                'Each numbered marker is a camera frame pose. The short line shows estimated rig/camera forward direction.</text>'
            ),
        ]
    )

    row_y = legend_y + 48
    x_cursor = 70
    for sample in camera_samples[:16]:
        color = color_for_sample(sample["sample_number"] - 1, len(camera_samples))
        svg.append(
            f'<circle cx="{x_cursor}" cy="{row_y}" r="8" fill="{color}"/>'
        )
        svg.append(
            f'<text x="{x_cursor + 14}" y="{row_y + 5}" fill="#cbd5e1" font-size="13">'
            f'{sample["sample_number"]}: frame {sample["frame_index"]}, '
            f't={sample["video_time_s"]:.1f}s</text>'
        )
        x_cursor += 186
        if x_cursor > width - 220:
            x_cursor = 70
            row_y += 24

    svg.append("</svg>")
    output.write_text("\n".join(svg), encoding="utf-8")


def write_json(output: Path, payload: dict[str, Any]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render camera frame samples associated with an ICP lidar trajectory"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--trajectory", type=Path, required=True, help="ICP trajectory JSON")
    parser.add_argument("--output", type=Path, required=True, help="Output SVG path")
    parser.add_argument("--json-output", type=Path, help="Optional frame-pose JSON")
    parser.add_argument("--sample-count", type=int, default=12)
    parser.add_argument("--sample-start-s", type=float)
    parser.add_argument("--sample-end-s", type=float)
    parser.add_argument("--lidar-angle-offset-deg", type=float, default=125.0)
    parser.add_argument("--camera-forward-m", type=float, default=-0.0737)
    parser.add_argument("--camera-left-m", type=float, default=0.0051)
    parser.add_argument("--min-distance-m", type=float, default=0.20)
    parser.add_argument("--max-distance-m", type=float, default=5.0)
    parser.add_argument("--min-quality", type=int, default=1)
    parser.add_argument("--render-scan-stride", type=int, default=2)
    parser.add_argument("--render-point-stride", type=int, default=1)
    parser.add_argument("--max-valid-points-ratio", type=float, default=1.5)
    parser.add_argument("--max-points-in-svg", type=int, default=75000)
    args = parser.parse_args()

    if args.sample_count <= 0:
        parser.error("--sample-count must be positive")

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    camera_metadata = load_json(session / manifest["camera"]["metadata"])
    scans = icp.load_scans(session / manifest["lidar"]["scans"])
    trajectory = load_trajectory(args.trajectory)
    skipped_scan_indices = icp.oversized_scan_indices(scans, args.max_valid_points_ratio)

    map_points, map_summary = icp.collect_map_points(
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
    camera_samples = build_frame_samples(
        camera_metadata,
        trajectory,
        scans,
        args.sample_count,
        args.sample_start_s,
        args.sample_end_s,
        args.camera_forward_m,
        args.camera_left_m,
    )

    render_svg(
        args.output,
        str(manifest["session_id"]),
        map_points,
        trajectory,
        camera_samples,
        args.max_points_in_svg,
    )
    payload = {
        "session_id": manifest["session_id"],
        "camera_video": manifest["camera"]["video"],
        "camera_metadata": manifest["camera"]["metadata"],
        "trajectory": str(args.trajectory),
        "coordinate_frame": "ICP 2D map frame, meters, radians",
        "camera_offset_used": {
            "forward_m": args.camera_forward_m,
            "left_m": args.camera_left_m,
        },
        "map_summary": map_summary,
        "sampled_frames": camera_samples,
    }
    if args.json_output:
        write_json(args.json_output, payload)

    print(f"Wrote {args.output}")
    if args.json_output:
        print(f"Wrote {args.json_output}")
    print(f"Camera frames available: {len(camera_metadata)}")
    print(f"Sampled camera frames: {len(camera_samples)}")
    for sample in camera_samples:
        pose = sample["camera_pose"]
        print(
            f"{sample['sample_number']:2d}: frame={sample['frame_index']:3d} "
            f"t={sample['video_time_s']:6.3f}s "
            f"pose=({pose['x_m']:+.3f}, {pose['y_m']:+.3f}, "
            f"{pose['theta_deg']:+.1f}deg)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
