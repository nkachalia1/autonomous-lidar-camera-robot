#!/usr/bin/env python3
"""Render camera, lidar, and timing SVGs for a recorded sensor session."""

from __future__ import annotations

import argparse
import base64
import json
import math
import statistics
from pathlib import Path
from typing import Any


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def svg_header(width: int, height: int, title: str, subtitle: str) -> list[str]:
    return [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'xmlns:xlink="http://www.w3.org/1999/xlink" '
            f'width="{width}" height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        f'<text x="48" y="52" fill="#f8fafc" font-size="30" font-weight="700">{title}</text>',
        f'<text x="48" y="82" fill="#94a3b8" font-size="17">{subtitle}</text>',
    ]


def render_camera_contact_sheet(
    frame_dir: Path, output: Path, session_id: str
) -> None:
    frames = [
        (5, frame_dir / "camera-5.png"),
        (60, frame_dir / "camera-60.png"),
        (120, frame_dir / "camera-120.png"),
        (178, frame_dir / "camera-178.png"),
    ]
    missing = [path for _, path in frames if not path.exists()]
    if missing:
        raise FileNotFoundError(f"Missing camera preview frames: {missing}")

    width, height = 1500, 960
    panel_width, panel_height = 680, 382
    positions = [(48, 130), (772, 130), (48, 560), (772, 560)]
    svg = svg_header(
        width,
        height,
        f"Session {session_id} — camera samples",
        "Four decoded frames from the stationary three-minute capture.",
    )

    for (second, path), (x, y) in zip(frames, positions):
        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
        svg.extend(
            [
                f'<rect x="{x}" y="{y}" width="{panel_width}" height="{panel_height}" '
                'rx="16" fill="#111e30" stroke="#334155"/>',
                f'<image x="{x}" y="{y}" width="{panel_width}" height="{panel_height}" '
                'preserveAspectRatio="xMidYMid slice" '
                f'href="data:image/png;base64,{encoded}"/>',
                f'<rect x="{x + 16}" y="{y + 16}" width="104" height="38" '
                'rx="19" fill="#07111f" fill-opacity="0.85"/>',
                f'<text x="{x + 34}" y="{y + 42}" fill="#f8fafc" '
                f'font-size="18" font-weight="700">t = {second}s</text>',
            ]
        )

    svg.append("</svg>")
    output.write_text("\n".join(svg), encoding="utf-8")


def lidar_point(point: list[float]) -> tuple[float, float]:
    angle_deg, distance_m = point[0], point[1]
    angle_rad = math.radians(angle_deg)
    return distance_m * math.cos(angle_rad), distance_m * math.sin(angle_rad)


def render_lidar_scans(scans: list[dict[str, Any]], output: Path, session_id: str) -> None:
    selected = [
        ("first", scans[0]),
        ("middle", scans[len(scans) // 2]),
        ("last", scans[-1]),
    ]
    width, height = 1500, 660
    centers = [(260, 360), (750, 360), (1240, 360)]
    radius_px = 205
    max_range_m = 4.0
    scale = radius_px / max_range_m
    colors = ["#38bdf8", "#a78bfa", "#fb7185"]

    svg = svg_header(
        width,
        height,
        f"Session {session_id} — lidar snapshots",
        "First, middle, and last complete scans in the lidar sensor frame (4 m radius).",
    )

    for (label, scan), (cx, cy), color in zip(selected, centers, colors):
        svg.append(
            f'<rect x="{cx - 225}" y="{cy - 245}" width="450" height="480" '
            'rx="18" fill="#0d1726" stroke="#334155"/>'
        )
        # The panel boundary represents 4 m. Square range guides avoid visual
        # ambiguity when this SVG is rasterized by different renderers.
        for meter in range(1, 4):
            radius = meter * scale
            svg.append(
                f'<rect x="{cx - radius:.1f}" y="{cy - radius:.1f}" '
                f'width="{radius * 2:.1f}" height="{radius * 2:.1f}" '
                'fill="none" stroke="#304158" stroke-width="1"/>'
            )
        svg.extend(
            [
                f'<line x1="{cx - radius_px}" y1="{cy}" x2="{cx + radius_px}" '
                f'y2="{cy}" stroke="#50657e"/>',
                f'<line x1="{cx}" y1="{cy - radius_px}" x2="{cx}" '
                f'y2="{cy + radius_px}" stroke="#50657e"/>',
            ]
        )
        visible = 0
        for point in scan["points"]:
            x_m, y_m = lidar_point(point)
            if point[1] > max_range_m:
                continue
            visible += 1
            x = cx + x_m * scale
            y = cy - y_m * scale
            svg.append(
                f'<circle cx="{x:.1f}" cy="{y:.1f}" r="1.8" fill="{color}"/>'
            )
        svg.extend(
            [
                f'<circle cx="{cx}" cy="{cy}" r="7" fill="#f8fafc"/>',
                f'<line x1="{cx}" y1="{cy}" x2="{cx + 40}" y2="{cy}" '
                'stroke="#f8fafc" stroke-width="4"/>',
                f'<text x="{cx - 205}" y="{cy - 205}" fill="#f8fafc" '
                f'font-size="23" font-weight="700">{label.title()} scan</text>',
                f'<text x="{cx - 205}" y="{cy + 263}" fill="#94a3b8" '
                f'font-size="15">index {scan["scan_index"]}; '
                f'{scan["valid_point_count"]} valid returns; {visible} within 4 m</text>',
                f'<text x="{cx + 157}" y="{cy + 216}" fill="#64748b" '
                f'font-size="13">edge = 4 m</text>',
            ]
        )
    svg.append("</svg>")
    output.write_text("\n".join(svg), encoding="utf-8")


def render_timing(
    camera_timestamps_ns: list[int],
    lidar_timestamps_us: list[int],
    output: Path,
    session_id: str,
) -> None:
    camera_gaps = [
        (current - previous) / 1e6
        for previous, current in zip(camera_timestamps_ns, camera_timestamps_ns[1:])
    ]
    lidar_gaps = [
        (current - previous) / 1e3
        for previous, current in zip(lidar_timestamps_us, lidar_timestamps_us[1:])
    ]
    camera_elapsed = [
        (timestamp - camera_timestamps_ns[0]) / 1e9
        for timestamp in camera_timestamps_ns[1:]
    ]
    lidar_elapsed = [
        (timestamp - lidar_timestamps_us[0]) / 1e6
        for timestamp in lidar_timestamps_us[1:]
    ]

    width, height = 1500, 760
    left, right, top, bottom = 100, 1440, 130, 650
    plot_width, plot_height = right - left, bottom - top
    duration = max(camera_elapsed[-1], lidar_elapsed[-1])
    max_gap = max(max(camera_gaps), max(lidar_gaps), 350.0)

    def px_x(seconds: float) -> float:
        return left + seconds / duration * plot_width

    def px_y(milliseconds: float) -> float:
        return bottom - milliseconds / max_gap * plot_height

    svg = svg_header(
        width,
        height,
        f"Session {session_id} — timestamp cadence",
        "Regular cadence dominates; highlighted spikes are the two detected anomalies.",
    )
    svg.append(
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" '
        'rx="14" fill="#0d1726" stroke="#334155"/>'
    )

    for second in range(0, 181, 30):
        x = px_x(second)
        svg.extend(
            [
                f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{bottom}" '
                'stroke="#26384d"/>',
                f'<text x="{x - 12:.1f}" y="{bottom + 28}" fill="#94a3b8" '
                f'font-size="14">{second}s</text>',
            ]
        )
    for gap in range(0, 351, 50):
        y = px_y(gap)
        svg.extend(
            [
                f'<line x1="{left}" y1="{y:.1f}" x2="{right}" y2="{y:.1f}" '
                'stroke="#26384d"/>',
                f'<text x="{left - 52}" y="{y + 5:.1f}" fill="#94a3b8" '
                f'font-size="14">{gap}ms</text>',
            ]
        )

    for elapsed, gap in zip(camera_elapsed, camera_gaps):
        color = "#fbbf24" if gap > statistics.median(camera_gaps) * 1.5 else "#38bdf8"
        svg.append(
            f'<circle cx="{px_x(elapsed):.1f}" cy="{px_y(gap):.1f}" '
            f'r="2.1" fill="{color}"/>'
        )
    for elapsed, gap in zip(lidar_elapsed, lidar_gaps):
        color = "#fb7185" if gap > statistics.median(lidar_gaps) * 1.5 else "#a78bfa"
        svg.append(
            f'<circle cx="{px_x(elapsed):.1f}" cy="{px_y(gap):.1f}" '
            f'r="2.1" fill="{color}"/>'
        )

    svg.extend(
        [
            '<circle cx="110" cy="710" r="6" fill="#38bdf8"/>',
            '<text x="125" y="716" fill="#cbd5e1" font-size="16">camera gaps</text>',
            '<circle cx="270" cy="710" r="6" fill="#a78bfa"/>',
            '<text x="285" y="716" fill="#cbd5e1" font-size="16">lidar gaps</text>',
            '<circle cx="420" cy="710" r="6" fill="#fbbf24"/>',
            '<text x="435" y="716" fill="#cbd5e1" font-size="16">camera anomaly</text>',
            '<circle cx="610" cy="710" r="6" fill="#fb7185"/>',
            '<text x="625" y="716" fill="#cbd5e1" font-size="16">lidar anomaly</text>',
            "</svg>",
        ]
    )
    output.write_text("\n".join(svg), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Render visual session evidence")
    parser.add_argument("session", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    session = args.session.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = load_json(session / "manifest.json")
    session_id = manifest["session_id"]

    camera_metadata = load_json(session / manifest["camera"]["metadata"])
    camera_timestamps_ns = [frame["SensorTimestamp"] for frame in camera_metadata]

    scans: list[dict[str, Any]] = []
    lidar_timestamps_us: list[int] = []
    with (session / manifest["lidar"]["scans"]).open(encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            if item.get("type") == "scan":
                scans.append(item)
                lidar_timestamps_us.append(item["timestamp_us"])

    render_camera_contact_sheet(
        session / "preview-frames",
        output_dir / f"{session_id}-camera.svg",
        session_id,
    )
    render_lidar_scans(
        scans,
        output_dir / f"{session_id}-lidar.svg",
        session_id,
    )
    render_timing(
        camera_timestamps_ns,
        lidar_timestamps_us,
        output_dir / f"{session_id}-timing.svg",
        session_id,
    )
    print(f"Wrote session visuals to {output_dir}")


if __name__ == "__main__":
    main()
