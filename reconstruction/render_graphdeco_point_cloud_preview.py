#!/usr/bin/env python3
"""Render a lightweight SVG preview of GraphDECO Gaussian-splatting PLY files.

The official GraphDECO output PLY is usually binary and stores Gaussian
parameters such as ``f_dc_*``, opacity, scale, and rotation.  This script reads
only the vertex positions and enough color information for a diagnostic preview.
It is not a full Gaussian renderer.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import struct
from pathlib import Path
from typing import Any

import cv2
import numpy as np


PLY_SCALAR_FORMATS: dict[tuple[str, str], str] = {
    ("ascii", "float"): "f",
    ("binary_little_endian", "float"): "f",
    ("binary_little_endian", "float32"): "f",
    ("binary_little_endian", "double"): "d",
    ("binary_little_endian", "uchar"): "B",
    ("binary_little_endian", "uint8"): "B",
    ("binary_little_endian", "char"): "b",
    ("binary_little_endian", "int8"): "b",
    ("binary_little_endian", "ushort"): "H",
    ("binary_little_endian", "uint16"): "H",
    ("binary_little_endian", "short"): "h",
    ("binary_little_endian", "int16"): "h",
    ("binary_little_endian", "uint"): "I",
    ("binary_little_endian", "uint32"): "I",
    ("binary_little_endian", "int"): "i",
    ("binary_little_endian", "int32"): "i",
}


def sigmoid(value: float) -> float:
    if value >= 0.0:
        z = math.exp(-value)
        return 1.0 / (1.0 + z)
    z = math.exp(value)
    return z / (1.0 + z)


def sh_dc_to_rgb(value: float) -> int:
    # GraphDECO stores RGB DC spherical-harmonic coefficients.  Their helper
    # converts with rgb = 0.28209479177387814 * dc + 0.5.
    rgb = 0.28209479177387814 * value + 0.5
    return max(0, min(255, round(rgb * 255)))


def parse_ply(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    with path.open("rb") as stream:
        header_lines: list[str] = []
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError(f"unexpected EOF while reading PLY header: {path}")
            line = raw.decode("ascii", errors="replace").strip()
            header_lines.append(line)
            if line == "end_header":
                break
        data_offset = stream.tell()

        fmt = ""
        vertex_count = 0
        properties: list[tuple[str, str]] = []
        in_vertex = False
        for line in header_lines:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "format":
                fmt = parts[1]
            elif parts[:2] == ["element", "vertex"]:
                vertex_count = int(parts[2])
                in_vertex = True
            elif parts[0] == "element":
                in_vertex = False
            elif in_vertex and parts[0] == "property":
                if parts[1] == "list":
                    raise ValueError("list vertex properties are not supported")
                properties.append((parts[2], parts[1]))

        if fmt not in {"ascii", "binary_little_endian"}:
            raise ValueError(f"unsupported PLY format: {fmt}")
        if not vertex_count:
            raise ValueError("PLY has no vertex records")

        stream.seek(data_offset)
        points: list[dict[str, Any]] = []
        if fmt == "ascii":
            for _ in range(vertex_count):
                line = stream.readline().decode("utf-8", errors="replace").strip()
                values = line.split()
                if len(values) < len(properties):
                    raise ValueError(f"short ASCII PLY vertex row: {line}")
                payload = {
                    name: float(value) if type_name not in {"uchar", "uint8"} else int(value)
                    for (name, type_name), value in zip(properties, values)
                }
                points.append(payload)
        else:
            struct_format = "<" + "".join(
                PLY_SCALAR_FORMATS[(fmt, type_name)] for _name, type_name in properties
            )
            record_size = struct.calcsize(struct_format)
            for _ in range(vertex_count):
                raw = stream.read(record_size)
                if len(raw) != record_size:
                    raise ValueError("short binary PLY vertex row")
                unpacked = struct.unpack(struct_format, raw)
                points.append({name: value for (name, _type_name), value in zip(properties, unpacked)})

    metadata = {
        "path": str(path),
        "format": fmt,
        "vertex_count": vertex_count,
        "properties": [name for name, _type_name in properties],
    }
    return points, metadata


def point_rgb(point: dict[str, Any]) -> tuple[int, int, int]:
    if {"red", "green", "blue"}.issubset(point):
        return int(point["red"]), int(point["green"]), int(point["blue"])
    if {"f_dc_0", "f_dc_1", "f_dc_2"}.issubset(point):
        return (
            sh_dc_to_rgb(float(point["f_dc_0"])),
            sh_dc_to_rgb(float(point["f_dc_1"])),
            sh_dc_to_rgb(float(point["f_dc_2"])),
        )
    return 148, 163, 184


def point_opacity(point: dict[str, Any]) -> float:
    if "opacity" in point:
        return max(0.12, min(0.95, sigmoid(float(point["opacity"]))))
    return 0.82


def bounds(points: list[dict[str, Any]]) -> dict[str, float]:
    xs = [float(point["x"]) for point in points]
    ys = [float(point["y"]) for point in points]
    zs = [float(point["z"]) for point in points]
    return {
        "x_min_m": min(xs),
        "x_max_m": max(xs),
        "y_min_m": min(ys),
        "y_max_m": max(ys),
        "z_min_m": min(zs),
        "z_max_m": max(zs),
    }


def median(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def summarize(points: list[dict[str, Any]], metadata: dict[str, Any]) -> dict[str, Any]:
    opacities = [point_opacity(point) for point in points]
    colors = [point_rgb(point) for point in points]
    return {
        **metadata,
        **bounds(points),
        "opacity_min": min(opacities),
        "opacity_median": median(opacities),
        "opacity_max": max(opacities),
        "mean_rgb": [
            round(sum(color[channel] for color in colors) / len(colors), 2)
            for channel in range(3)
        ],
    }


def render_svg(
    output: Path,
    points: list[dict[str, Any]],
    summary: dict[str, Any],
    *,
    title: str,
    max_points: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    width = 1500
    height = 920
    panels = [
        ("top x/y", 48, 112, 660, 360, "x", "y"),
        ("front x/z", 792, 112, 660, 360, "x", "z"),
        ("side y/z", 48, 532, 660, 320, "y", "z"),
    ]
    b = bounds(points)
    samples = points
    if len(points) > max_points:
        stride = math.ceil(len(points) / max_points)
        samples = points[::stride]

    def scale(value: float, lo: float, hi: float, pixel_lo: float, pixel_hi: float) -> float:
        if abs(hi - lo) < 1e-9:
            return 0.5 * (pixel_lo + pixel_hi)
        return pixel_lo + (value - lo) * (pixel_hi - pixel_lo) / (hi - lo)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#0f172a"/>',
        f'<text x="48" y="52" fill="#f8fafc" font-size="30" font-weight="700">{html.escape(title)}</text>',
        (
            f'<text x="48" y="82" fill="#94a3b8" font-size="16">'
            f'vertices={summary["vertex_count"]}; '
            f'x={summary["x_min_m"]:+.2f}..{summary["x_max_m"]:+.2f} m; '
            f'y={summary["y_min_m"]:+.2f}..{summary["y_max_m"]:+.2f} m; '
            f'z={summary["z_min_m"]:+.2f}..{summary["z_max_m"]:+.2f} m; '
            f'opacity median={summary["opacity_median"]:.2f}</text>'
        ),
    ]
    axis_bounds = {
        "x": (b["x_min_m"], b["x_max_m"]),
        "y": (b["y_min_m"], b["y_max_m"]),
        "z": (b["z_min_m"], b["z_max_m"]),
    }
    for label, left, top, panel_w, panel_h, axis_x, axis_y in panels:
        svg.append(
            f'<rect x="{left}" y="{top}" width="{panel_w}" height="{panel_h}" rx="18" fill="#111827" stroke="#334155"/>'
        )
        svg.append(
            f'<text x="{left + 18}" y="{top + 30}" fill="#e2e8f0" font-size="18" font-weight="700">{label}</text>'
        )
        min_x, max_x = axis_bounds[axis_x]
        min_y, max_y = axis_bounds[axis_y]
        for point in samples:
            px = scale(float(point[axis_x]), min_x, max_x, left + 36, left + panel_w - 36)
            py = scale(float(point[axis_y]), min_y, max_y, top + panel_h - 36, top + 48)
            red, green, blue = point_rgb(point)
            opacity = point_opacity(point)
            svg.append(
                f'<circle cx="{px:.2f}" cy="{py:.2f}" r="2.1" fill="rgb({red},{green},{blue})" fill-opacity="{opacity:.3f}"/>'
            )
        svg.append(
            f'<text x="{left + 18}" y="{top + panel_h - 14}" fill="#64748b" font-size="12">'
            f'{axis_x}: {min_x:+.2f}..{max_x:+.2f} m; {axis_y}: {min_y:+.2f}..{max_y:+.2f} m</text>'
        )
    svg.append("</svg>")
    output.write_text("\n".join(svg) + "\n", encoding="utf-8")


def render_png(
    output: Path,
    points: list[dict[str, Any]],
    summary: dict[str, Any],
    *,
    title: str,
    max_points: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    width = 1500
    height = 920
    image = np.full((height, width, 3), (42, 23, 15), dtype=np.uint8)  # BGR #0f172a
    panels = [
        ("top x/y", 48, 112, 660, 360, "x", "y"),
        ("front x/z", 792, 112, 660, 360, "x", "z"),
        ("side y/z", 48, 532, 660, 320, "y", "z"),
    ]
    b = bounds(points)
    samples = points
    if len(points) > max_points:
        stride = math.ceil(len(points) / max_points)
        samples = points[::stride]

    def scale(value: float, lo: float, hi: float, pixel_lo: float, pixel_hi: float) -> float:
        if abs(hi - lo) < 1e-9:
            return 0.5 * (pixel_lo + pixel_hi)
        return pixel_lo + (value - lo) * (pixel_hi - pixel_lo) / (hi - lo)

    axis_bounds = {
        "x": (b["x_min_m"], b["x_max_m"]),
        "y": (b["y_min_m"], b["y_max_m"]),
        "z": (b["z_min_m"], b["z_max_m"]),
    }
    cv2.putText(image, title, (48, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 250, 248), 2, cv2.LINE_AA)
    subtitle = (
        f'vertices={summary["vertex_count"]}; '
        f'x={summary["x_min_m"]:+.2f}..{summary["x_max_m"]:+.2f}m; '
        f'y={summary["y_min_m"]:+.2f}..{summary["y_max_m"]:+.2f}m; '
        f'z={summary["z_min_m"]:+.2f}..{summary["z_max_m"]:+.2f}m; '
        f'op med={summary["opacity_median"]:.2f}'
    )
    cv2.putText(image, subtitle, (48, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (184, 163, 148), 1, cv2.LINE_AA)

    for label, left, top, panel_w, panel_h, axis_x, axis_y in panels:
        cv2.rectangle(
            image,
            (left, top),
            (left + panel_w, top + panel_h),
            (39, 24, 17),
            thickness=-1,
        )
        cv2.rectangle(
            image,
            (left, top),
            (left + panel_w, top + panel_h),
            (85, 65, 51),
            thickness=1,
        )
        cv2.putText(image, label, (left + 18, top + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (240, 232, 226), 1, cv2.LINE_AA)
        min_x, max_x = axis_bounds[axis_x]
        min_y, max_y = axis_bounds[axis_y]
        for point in samples:
            px = int(round(scale(float(point[axis_x]), min_x, max_x, left + 36, left + panel_w - 36)))
            py = int(round(scale(float(point[axis_y]), min_y, max_y, top + panel_h - 36, top + 48)))
            red, green, blue = point_rgb(point)
            opacity = point_opacity(point)
            bgr = (int(blue * opacity), int(green * opacity), int(red * opacity))
            cv2.circle(image, (px, py), 2, bgr, thickness=-1, lineType=cv2.LINE_AA)
        footer = f"{axis_x}: {min_x:+.2f}..{max_x:+.2f}m; {axis_y}: {min_y:+.2f}..{max_y:+.2f}m"
        cv2.putText(image, footer, (left + 18, top + panel_h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (139, 116, 100), 1, cv2.LINE_AA)
    ok = cv2.imwrite(str(output), image)
    if not ok:
        raise RuntimeError(f"failed to write PNG: {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render an SVG preview of a GraphDECO/input PLY")
    parser.add_argument("ply", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--png-output", type=Path)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--title", default="GraphDECO point cloud preview")
    parser.add_argument("--max-points", type=int, default=12000)
    args = parser.parse_args()

    points, metadata = parse_ply(args.ply)
    summary = summarize(points, metadata)
    render_svg(args.output, points, summary, title=args.title, max_points=args.max_points)
    if args.png_output:
        render_png(args.png_output, points, summary, title=args.title, max_points=args.max_points)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {args.output}")
    if args.png_output:
        print(f"Wrote {args.png_output}")
    if args.json_output:
        print(f"Wrote {args.json_output}")
    print(f"Vertices: {summary['vertex_count']}")
    print(
        "Bounds m: "
        f"x {summary['x_min_m']:+.3f}..{summary['x_max_m']:+.3f}, "
        f"y {summary['y_min_m']:+.3f}..{summary['y_max_m']:+.3f}, "
        f"z {summary['z_min_m']:+.3f}..{summary['z_max_m']:+.3f}"
    )
    print(
        "Opacity min/median/max: "
        f"{summary['opacity_min']:.3f}/{summary['opacity_median']:.3f}/{summary['opacity_max']:.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
