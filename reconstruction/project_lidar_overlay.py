#!/usr/bin/env python3
"""Project one 2D lidar scan onto a camera image as an SVG overlay.

This is an early calibration/debugging tool. It intentionally exposes the
rough rig parameters as command-line arguments because the first useful goal is
visual feedback: do projected lidar points land on plausible image features?
"""

from __future__ import annotations

import argparse
import base64
import html
import json
import math
import mimetypes
from pathlib import Path
from typing import Any


DistortionCoefficients = tuple[float, float, float, float, float]


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def load_lidar_scans(path: Path) -> list[dict[str, Any]]:
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
        raise ValueError(f"no lidar scan records found in {path}")
    return scans


def camera_intrinsics_from_fov(
    width: int,
    height: int,
    fov_x_deg: float,
    fov_y_deg: float,
) -> tuple[float, float, float, float]:
    fx = width / (2.0 * math.tan(math.radians(fov_x_deg) / 2.0))
    fy = height / (2.0 * math.tan(math.radians(fov_y_deg) / 2.0))
    return fx, fy, width / 2.0, height / 2.0


def yaml_section_data(path: Path, section: str) -> list[float]:
    """Extract a `data: [newline list]` block from our generated YAML files."""

    lines = path.read_text(encoding="utf-8").splitlines()
    in_section = False
    in_data = False
    values: list[float] = []
    for line in lines:
        if not line.startswith(" ") and line.endswith(":"):
            in_section = line[:-1] == section
            in_data = False
            continue
        if not in_section:
            continue
        stripped = line.strip()
        if stripped == "data:":
            in_data = True
            continue
        if in_data:
            if stripped.startswith("- "):
                values.append(float(stripped[2:]))
                continue
            if stripped and not stripped.startswith("#"):
                break
    if not values:
        raise ValueError(f"could not find {section}.data in {path}")
    return values


def camera_intrinsics_from_yaml(
    path: Path,
) -> tuple[float, float, float, float, DistortionCoefficients | None]:
    matrix = yaml_section_data(path, "camera_matrix")
    if len(matrix) != 9:
        raise ValueError(f"camera_matrix.data in {path} must contain 9 values")

    distortion_values = yaml_section_data(path, "distortion_coefficients")
    distortion: DistortionCoefficients | None = None
    if len(distortion_values) >= 5:
        distortion = (
            distortion_values[0],
            distortion_values[1],
            distortion_values[2],
            distortion_values[3],
            distortion_values[4],
        )

    return matrix[0], matrix[4], matrix[2], matrix[5], distortion


def lidar_point_to_lidar_xyz(
    angle_deg: float,
    distance_m: float,
    angle_offset_deg: float,
) -> tuple[float, float, float]:
    angle = math.radians(angle_deg + angle_offset_deg)
    return distance_m * math.cos(angle), distance_m * math.sin(angle), 0.0


def nominal_lidar_to_camera(
    point_lidar: tuple[float, float, float],
    camera_forward_m: float,
    camera_left_m: float,
    camera_up_m: float,
) -> tuple[float, float, float]:
    """Map lidar coordinates to an OpenCV-style camera frame.

    Coordinate convention:
      lidar:  +x forward, +y left, +z up
      camera: +x right,   +y down, +z forward

    The translation arguments describe the camera optical center in the lidar
    frame. The nominal orientation assumes camera forward is lidar +x.
    """

    x_l, y_l, z_l = point_lidar
    rel_x = x_l - camera_forward_m
    rel_y = y_l - camera_left_m
    rel_z = z_l - camera_up_m
    return -rel_y, -rel_z, rel_x


def rotate_camera_adjustment(
    point_camera: tuple[float, float, float],
    yaw_deg: float,
    pitch_deg: float,
    roll_deg: float,
) -> tuple[float, float, float]:
    """Apply empirical camera-frame yaw, pitch, and roll adjustments.

    These are deliberately simple tuning knobs for first-pass visual alignment,
    not a replacement for a calibrated extrinsic transform.
    """

    x, y, z = point_camera

    yaw = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x, z = cos_yaw * x + sin_yaw * z, -sin_yaw * x + cos_yaw * z

    pitch = math.radians(pitch_deg)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    y, z = cos_pitch * y - sin_pitch * z, sin_pitch * y + cos_pitch * z

    roll = math.radians(roll_deg)
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    x, y = cos_roll * x - sin_roll * y, sin_roll * x + cos_roll * y

    return x, y, z


def project_camera_point(
    point_camera: tuple[float, float, float],
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    distortion: DistortionCoefficients | None = None,
) -> tuple[float, float] | None:
    x, y, z = point_camera
    if z <= 0.01:
        return None
    x_n = x / z
    y_n = y / z
    if distortion is not None:
        k1, k2, p1, p2, k3 = distortion
        r2 = x_n * x_n + y_n * y_n
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        x_tangential = 2.0 * p1 * x_n * y_n + p2 * (r2 + 2.0 * x_n * x_n)
        y_tangential = p1 * (r2 + 2.0 * y_n * y_n) + 2.0 * p2 * x_n * y_n
        x_n = x_n * radial + x_tangential
        y_n = y_n * radial + y_tangential
    return fx * x_n + cx, fy * y_n + cy


def distortion_label(distortion: DistortionCoefficients | None) -> str:
    if distortion is None:
        return "none"
    return ", ".join(f"{value:.3g}" for value in distortion)


def point_color(distance_m: float) -> str:
    if distance_m < 1.0:
        return "#fb7185"
    if distance_m < 2.0:
        return "#facc15"
    if distance_m < 4.0:
        return "#38bdf8"
    return "#a78bfa"


def image_data_uri(path: Path) -> str:
    mime_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{encoded}"


def choose_frame_timestamp(
    camera_metadata: list[dict[str, Any]],
    frame_index: int | None,
) -> tuple[int, int]:
    if not camera_metadata:
        raise ValueError("camera metadata is empty")
    if frame_index is None:
        frame_index = len(camera_metadata) // 2
    if frame_index < 0 or frame_index >= len(camera_metadata):
        raise ValueError(
            f"frame index {frame_index} outside 0..{len(camera_metadata) - 1}"
        )
    timestamp_ns = camera_metadata[frame_index].get("SensorTimestamp")
    if not isinstance(timestamp_ns, int):
        raise ValueError(f"frame {frame_index} has no integer SensorTimestamp")
    return frame_index, timestamp_ns


def choose_scan(
    scans: list[dict[str, Any]],
    scan_index: int | None,
    target_timestamp_ns: int,
) -> tuple[int, dict[str, Any]]:
    if scan_index is not None:
        if scan_index < 0 or scan_index >= len(scans):
            raise ValueError(f"scan index {scan_index} outside 0..{len(scans) - 1}")
        return scan_index, scans[scan_index]

    target_us = target_timestamp_ns / 1000.0
    best_index, best_scan = min(
        enumerate(scans),
        key=lambda item: abs(float(item[1]["timestamp_us"]) - target_us),
    )
    return best_index, best_scan


def render_overlay_svg(
    output: Path,
    background: Path,
    width: int,
    height: int,
    projected_points: list[tuple[float, float, float, int]],
    clipped_count: int,
    behind_count: int,
    title: str,
    subtitle: str,
    details: list[str],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    margin = 32
    legend_height = 172
    canvas_width = width + margin * 2
    canvas_height = height + margin * 2 + legend_height
    image_href = image_data_uri(background)

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_width}" '
            f'height="{canvas_height}" viewBox="0 0 {canvas_width} {canvas_height}">'
        ),
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        f'<text x="{margin}" y="34" fill="#f8fafc" font-size="24" '
        f'font-weight="700">{html.escape(title)}</text>',
        f'<text x="{margin}" y="62" fill="#94a3b8" font-size="15">'
        f'{html.escape(subtitle)}</text>',
        f'<image x="{margin}" y="{margin + 48}" width="{width}" height="{height}" '
        f'preserveAspectRatio="xMidYMid slice" href="{image_href}"/>',
        f'<rect x="{margin}" y="{margin + 48}" width="{width}" height="{height}" '
        'fill="none" stroke="#e2e8f0" stroke-width="2"/>',
    ]

    y_offset = margin + 48
    for u, v, distance_m, quality in projected_points:
        radius = 4.0 if quality > 20 else 3.0
        svg.append(
            f'<circle cx="{margin + u:.1f}" cy="{y_offset + v:.1f}" r="{radius:.1f}" '
            f'fill="{point_color(distance_m)}" fill-opacity="0.82" '
            'stroke="#020617" stroke-width="1"/>'
        )

    legend_y = y_offset + height + 46
    svg.extend(
        [
            f'<rect x="{margin}" y="{legend_y - 28}" width="{width}" '
            f'height="{legend_height - 28}" rx="16" fill="#0d1726" '
            'stroke="#334155"/>',
            f'<text x="{margin + 22}" y="{legend_y}" fill="#f8fafc" '
            f'font-size="18" font-weight="700">Projected lidar returns: '
            f'{len(projected_points)}</text>',
            f'<text x="{margin + 22}" y="{legend_y + 28}" fill="#94a3b8" '
            f'font-size="14">Clipped outside image: {clipped_count}; '
            f'behind camera/too close: {behind_count}</text>',
        ]
    )

    color_labels = [
        ("#fb7185", "<1 m"),
        ("#facc15", "1-2 m"),
        ("#38bdf8", "2-4 m"),
        ("#a78bfa", ">4 m"),
    ]
    label_x = margin + 22
    for color, label in color_labels:
        svg.append(
            f'<circle cx="{label_x}" cy="{legend_y + 62}" r="6" fill="{color}"/>'
        )
        svg.append(
            f'<text x="{label_x + 12}" y="{legend_y + 67}" fill="#cbd5e1" '
            f'font-size="14">{html.escape(label)}</text>'
        )
        label_x += 86

    for index, detail in enumerate(details):
        svg.append(
            f'<text x="{margin + 22}" y="{legend_y + 96 + index * 22}" '
            f'fill="#94a3b8" font-size="13">{html.escape(detail)}</text>'
        )

    svg.append("</svg>")
    output.write_text("\n".join(svg), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render a first-pass camera/lidar projection overlay"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument(
        "--background",
        type=Path,
        required=True,
        help="Camera still/frame to use behind the projected lidar points",
    )
    parser.add_argument("--output", type=Path, required=True, help="Output SVG path")
    parser.add_argument("--frame-index", type=int, default=None)
    parser.add_argument("--scan-index", type=int, default=None)
    parser.add_argument("--camera-forward-m", type=float, required=True)
    parser.add_argument("--camera-left-m", type=float, required=True)
    parser.add_argument("--camera-up-m", type=float, required=True)
    parser.add_argument("--lidar-angle-offset-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--min-distance-m", type=float, default=0.15)
    parser.add_argument("--max-distance-m", type=float, default=8.0)
    parser.add_argument("--fov-x-deg", type=float, default=62.2)
    parser.add_argument("--fov-y-deg", type=float, default=37.2)
    parser.add_argument(
        "--camera-intrinsics",
        type=Path,
        help="YAML camera intrinsics file; overrides FOV-derived intrinsics",
    )
    parser.add_argument("--fx", type=float, default=None)
    parser.add_argument("--fy", type=float, default=None)
    parser.add_argument("--cx", type=float, default=None)
    parser.add_argument("--cy", type=float, default=None)
    args = parser.parse_args()

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    camera_metadata = load_json(session / manifest["camera"]["metadata"])
    scans = load_lidar_scans(session / manifest["lidar"]["scans"])

    width = int(manifest["camera"]["width"])
    height = int(manifest["camera"]["height"])

    distortion: DistortionCoefficients | None = None
    if args.camera_intrinsics:
        fx, fy, cx, cy, distortion = camera_intrinsics_from_yaml(
            args.camera_intrinsics.resolve()
        )
        intrinsics_source = str(args.camera_intrinsics)
    elif args.fx is None or args.fy is None or args.cx is None or args.cy is None:
        fx, fy, cx, cy = camera_intrinsics_from_fov(
            width,
            height,
            args.fov_x_deg,
            args.fov_y_deg,
        )
        intrinsics_source = "fov"
    else:
        fx, fy, cx, cy = args.fx, args.fy, args.cx, args.cy
        intrinsics_source = "manual"

    frame_index, frame_timestamp_ns = choose_frame_timestamp(
        camera_metadata,
        args.frame_index,
    )
    scan_index, scan = choose_scan(scans, args.scan_index, frame_timestamp_ns)
    scan_timestamp_ns = int(scan["timestamp_us"]) * 1000
    delta_ms = (scan_timestamp_ns - frame_timestamp_ns) / 1e6

    projected_points: list[tuple[float, float, float, int]] = []
    clipped_count = 0
    behind_count = 0
    filtered_count = 0

    for point in scan["points"]:
        angle_deg, distance_m, quality = float(point[0]), float(point[1]), int(point[2])
        if distance_m < args.min_distance_m or distance_m > args.max_distance_m:
            filtered_count += 1
            continue
        point_lidar = lidar_point_to_lidar_xyz(
            angle_deg,
            distance_m,
            args.lidar_angle_offset_deg,
        )
        point_camera = nominal_lidar_to_camera(
            point_lidar,
            args.camera_forward_m,
            args.camera_left_m,
            args.camera_up_m,
        )
        point_camera = rotate_camera_adjustment(
            point_camera,
            args.yaw_deg,
            args.pitch_deg,
            args.roll_deg,
        )
        projected = project_camera_point(point_camera, fx, fy, cx, cy, distortion)
        if projected is None:
            behind_count += 1
            continue
        u, v = projected
        if not (0 <= u < width and 0 <= v < height):
            clipped_count += 1
            continue
        projected_points.append((u, v, distance_m, quality))

    title = f"Session {manifest['session_id']} lidar-camera overlay"
    subtitle = (
        f"camera frame {frame_index}, lidar scan {scan_index}, "
        f"timestamp delta {delta_ms:+.1f} ms"
    )
    details = [
        (
            "translation lidar->camera: "
            f"forward={args.camera_forward_m:.3f} m, "
            f"left={args.camera_left_m:.3f} m, up={args.camera_up_m:.3f} m"
        ),
        (
            "adjustments: "
            f"lidar angle offset={args.lidar_angle_offset_deg:.1f} deg, "
            f"yaw={args.yaw_deg:.1f}, pitch={args.pitch_deg:.1f}, "
            f"roll={args.roll_deg:.1f} deg"
        ),
        (
            f"intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}; "
            f"source={intrinsics_source}; distance-filtered returns={filtered_count}"
        ),
        f"distortion coefficients: {distortion_label(distortion)}",
    ]
    render_overlay_svg(
        args.output,
        args.background.resolve(),
        width,
        height,
        projected_points,
        clipped_count,
        behind_count,
        title,
        subtitle,
        details,
    )

    print(f"Wrote {args.output}")
    print(f"Projected points inside image: {len(projected_points)}")
    print(f"Clipped outside image: {clipped_count}")
    print(f"Behind camera/too close: {behind_count}")
    print(f"Distance-filtered returns: {filtered_count}")
    print(f"Frame/scan timestamp delta: {delta_ms:+.1f} ms")
    if not projected_points:
        print("WARNING: no projected points landed inside the image.")
        print("Adjust --lidar-angle-offset-deg, translation, yaw, or pitch.")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
