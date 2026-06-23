#!/usr/bin/env python3
"""Render SLAMTEC simple_grabber text output as a dependency-free SVG."""

from __future__ import annotations

import argparse
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path


SAMPLE_PATTERN = re.compile(
    r"theta:\s*(?P<angle>[0-9.]+)\s+Dist:\s*(?P<distance>[0-9.]+)"
)


@dataclass(frozen=True)
class Sample:
    angle_deg: float
    distance_m: float

    @property
    def x_m(self) -> float:
        return self.distance_m * math.cos(math.radians(self.angle_deg))

    @property
    def y_m(self) -> float:
        return self.distance_m * math.sin(math.radians(self.angle_deg))


def parse_samples(path: Path) -> tuple[list[Sample], int]:
    matches = SAMPLE_PATTERN.finditer(path.read_text(encoding="utf-8", errors="replace"))
    all_samples = [
        Sample(
            angle_deg=float(match.group("angle")),
            distance_m=float(match.group("distance")) / 1000.0,
        )
        for match in matches
    ]
    valid = [sample for sample in all_samples if sample.distance_m > 0.0]
    if not valid:
        raise ValueError(f"No positive lidar ranges found in {path}")
    return valid, len(all_samples)


def point_color(distance_m: float, max_range_m: float) -> str:
    fraction = max(0.0, min(distance_m / max_range_m, 1.0))
    red = round(30 + 210 * fraction)
    green = round(170 - 95 * fraction)
    blue = round(230 - 80 * fraction)
    return f"rgb({red},{green},{blue})"


def panel_svg(
    samples: list[Sample],
    *,
    center_x: float,
    center_y: float,
    size_px: float,
    max_range_m: float,
    title: str,
) -> str:
    scale = (size_px / 2.0) / max_range_m
    parts: list[str] = []
    parts.append(
        f'<rect x="{center_x - size_px / 2:.1f}" '
        f'y="{center_y - size_px / 2:.1f}" width="{size_px:.1f}" '
        f'height="{size_px:.1f}" rx="18" fill="#0d1726" stroke="#334155"/>'
    )

    for radius_m in range(1, math.ceil(max_range_m) + 1):
        radius_px = radius_m * scale
        parts.append(
            f'<circle cx="{center_x:.1f}" cy="{center_y:.1f}" '
            f'r="{radius_px:.1f}" fill="none" stroke="#304158" '
            f'stroke-width="1"/>'
        )
        if radius_m < max_range_m + 0.01:
            parts.append(
                f'<text x="{center_x + radius_px + 5:.1f}" '
                f'y="{center_y - 5:.1f}" fill="#8fa5bd" '
                f'font-size="14">{radius_m} m</text>'
            )

    half = size_px / 2.0
    parts.extend(
        [
            f'<line x1="{center_x - half:.1f}" y1="{center_y:.1f}" '
            f'x2="{center_x + half:.1f}" y2="{center_y:.1f}" '
            'stroke="#50657e" stroke-width="1"/>',
            f'<line x1="{center_x:.1f}" y1="{center_y - half:.1f}" '
            f'x2="{center_x:.1f}" y2="{center_y + half:.1f}" '
            'stroke="#50657e" stroke-width="1"/>',
        ]
    )

    visible = [sample for sample in samples if sample.distance_m <= max_range_m]
    for sample in visible:
        x = center_x + sample.x_m * scale
        y = center_y - sample.y_m * scale
        parts.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.4" '
            f'fill="{point_color(sample.distance_m, max_range_m)}"/>'
        )

    parts.extend(
        [
            f'<circle cx="{center_x:.1f}" cy="{center_y:.1f}" r="8" '
            'fill="#f8fafc" stroke="#111827" stroke-width="3"/>',
            f'<line x1="{center_x:.1f}" y1="{center_y:.1f}" '
            f'x2="{center_x + 48:.1f}" y2="{center_y:.1f}" '
            'stroke="#f8fafc" stroke-width="4"/>',
            f'<polygon points="{center_x + 58:.1f},{center_y:.1f} '
            f'{center_x + 44:.1f},{center_y - 8:.1f} '
            f'{center_x + 44:.1f},{center_y + 8:.1f}" fill="#f8fafc"/>',
            f'<text x="{center_x - half:.1f}" y="{center_y - half - 20:.1f}" '
            f'fill="#e2e8f0" font-size="25" font-weight="700">{title}</text>',
            f'<text x="{center_x - half:.1f}" y="{center_y + half + 28:.1f}" '
            f'fill="#94a3b8" font-size="15">{len(visible)} visible returns; '
            'arrow marks lidar 0° direction</text>',
        ]
    )
    return "\n".join(parts)


def render_svg(samples: list[Sample], total_count: int, output_path: Path) -> None:
    width = 1540
    height = 950
    panel_size = 660
    maximum = max(sample.distance_m for sample in samples)
    full_range = max(4.0, math.ceil(maximum))
    median = statistics.median(sample.distance_m for sample in samples)
    zero_count = total_count - len(samples)

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}"
 viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#07111f"/>
<style>
  text {{ font-family: Inter, "Segoe UI", Arial, sans-serif; }}
</style>
<text x="60" y="54" fill="#f8fafc" font-size="32" font-weight="700">
  RPLIDAR A1M8 — single 360° scan
</text>
<text x="60" y="83" fill="#94a3b8" font-size="17">
  Top-down sensor-frame view; this is one horizontal slice, not yet a map.
</text>
{panel_svg(samples, center_x=390, center_y=480, size_px=panel_size,
           max_range_m=4.0, title="Room-scale detail (4 m radius)")}
{panel_svg(samples, center_x=1150, center_y=480, size_px=panel_size,
           max_range_m=full_range, title=f"Full scan ({full_range:.0f} m radius)")}
<rect x="60" y="885" width="1420" height="48" rx="12" fill="#111e30"/>
<text x="82" y="916" fill="#cbd5e1" font-size="17">
  Samples: {total_count}  •  valid returns: {len(samples)}  •  zero returns: {zero_count}
  •  min: {min(sample.distance_m for sample in samples):.3f} m
  •  median: {median:.3f} m  •  max: {maximum:.3f} m
</text>
</svg>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(svg, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot SLAMTEC simple_grabber text output as an SVG."
    )
    parser.add_argument("input", type=Path, help="Path to lidar-scan.txt")
    parser.add_argument("output", type=Path, help="Destination SVG path")
    args = parser.parse_args()

    samples, total_count = parse_samples(args.input)
    render_svg(samples, total_count, args.output)
    print(
        f"Wrote {args.output} with {len(samples)} valid returns "
        f"from {total_count} samples."
    )


if __name__ == "__main__":
    main()
