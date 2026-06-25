#!/usr/bin/env python3
"""Render an interactive HTML picker for measuring distances in a lidar map.

This is a manual fallback for small landmarks that automatic line detection can
miss.  Open the generated HTML in a browser, click the center of landmark A,
then click the center of landmark B.  The page reports the map-coordinate
distance between those two picked points.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import random
from pathlib import Path
from typing import TypeVar

import render_icp_lidar_map as icp
from detect_lidar_landmarks import load_trajectory


T = TypeVar("T")


def downsample(items: list[T], max_items: int, seed: int) -> list[T]:
    if len(items) <= max_items:
        return items
    rng = random.Random(seed)
    indices = sorted(rng.sample(range(len(items)), max_items))
    return [items[index] for index in indices]


def point_bounds(
    points: list[tuple[float, float, float]],
    padding_m: float,
) -> tuple[float, float, float, float]:
    if not points:
        raise ValueError("no map points available")
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs) - padding_m, max(xs) + padding_m, min(ys) - padding_m, max(ys) + padding_m


def render_html(
    output: Path,
    session_id: str,
    points: list[tuple[float, float, float]],
    bounds: tuple[float, float, float, float],
    expected_distance_m: float | None,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    min_x, max_x, min_y, max_y = bounds
    point_payload = [
        [round(point[0], 4), round(point[1], 4), round(point[2], 4)]
        for point in points
    ]
    expected_literal = "null" if expected_distance_m is None else f"{expected_distance_m:.9f}"

    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(session_id)} lidar measurement picker</title>
  <style>
    :root {{
      color-scheme: dark;
      font-family: Inter, Segoe UI, Arial, sans-serif;
      background: #07111f;
      color: #e2e8f0;
    }}
    body {{
      margin: 0;
      padding: 24px;
      background: #07111f;
    }}
    main {{
      max-width: 1560px;
      margin: 0 auto;
    }}
    h1 {{
      margin: 0 0 6px;
      font-size: 26px;
    }}
    p {{
      color: #94a3b8;
      margin: 6px 0;
      line-height: 1.45;
    }}
    canvas {{
      display: block;
      width: 1500px;
      max-width: 100%;
      height: auto;
      margin-top: 18px;
      border-radius: 18px;
      background: #0d1726;
      border: 1px solid #334155;
      cursor: crosshair;
    }}
    .panel {{
      margin-top: 16px;
      padding: 16px 18px;
      border: 1px solid #334155;
      border-radius: 16px;
      background: #0d1726;
    }}
    button {{
      background: #2563eb;
      color: white;
      border: 0;
      border-radius: 10px;
      padding: 9px 12px;
      font-weight: 700;
      margin-right: 8px;
      cursor: pointer;
    }}
    button.secondary {{
      background: #334155;
    }}
    pre {{
      white-space: pre-wrap;
      background: #020617;
      border: 1px solid #1e293b;
      border-radius: 12px;
      padding: 12px;
      color: #cbd5e1;
      min-height: 92px;
    }}
    .hint {{
      color: #fbbf24;
    }}
  </style>
</head>
<body>
<main>
  <h1>Session {html.escape(session_id)} lidar measurement picker</h1>
  <p class="hint">Click the center of Board A's grey lidar-return cluster, then click the center of Board B's cluster.</p>
  <p>If you mis-click, press Reset. The measured distance is in map meters.</p>
  <canvas id="map" width="1500" height="1050"></canvas>
  <div class="panel">
    <button id="reset">Reset picks</button>
    <button id="copy" class="secondary">Copy measurement</button>
    <pre id="readout">Click Board A.</pre>
  </div>
</main>
<script>
const points = {json.dumps(point_payload)};
const bounds = {{ minX: {min_x:.9f}, maxX: {max_x:.9f}, minY: {min_y:.9f}, maxY: {max_y:.9f} }};
const expectedDistanceM = {expected_literal};
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const readout = document.getElementById('readout');
const plot = {{ left: 70, top: 58, right: 32, bottom: 74 }};
const plotWidth = canvas.width - plot.left - plot.right;
const plotHeight = canvas.height - plot.top - plot.bottom;
const worldWidth = bounds.maxX - bounds.minX;
const worldHeight = bounds.maxY - bounds.minY;
const scale = Math.min(plotWidth / worldWidth, plotHeight / worldHeight);
const usedWidth = worldWidth * scale;
const usedHeight = worldHeight * scale;
const offsetX = plot.left + (plotWidth - usedWidth) / 2;
const offsetY = plot.top + (plotHeight - usedHeight) / 2;
let picks = [];

function px(x) {{ return offsetX + (x - bounds.minX) * scale; }}
function py(y) {{ return offsetY + (bounds.maxY - y) * scale; }}
function mx(x) {{ return bounds.minX + (x - offsetX) / scale; }}
function my(y) {{ return bounds.maxY - (y - offsetY) / scale; }}

function timeColor(t) {{
  t = Math.max(0, Math.min(1, t));
  let r, g, b;
  if (t < 0.5) {{
    const u = t / 0.5;
    r = Math.round(56 + (250 - 56) * u);
    g = Math.round(189 + (204 - 189) * u);
    b = Math.round(248 + (21 - 248) * u);
  }} else {{
    const u = (t - 0.5) / 0.5;
    r = Math.round(250 + (251 - 250) * u);
    g = Math.round(204 + (113 - 204) * u);
    b = Math.round(21 + (133 - 21) * u);
  }}
  return `rgb(${{r}}, ${{g}}, ${{b}})`;
}}

function drawGrid() {{
  ctx.fillStyle = '#07111f';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#0d1726';
  ctx.fillRect(offsetX, offsetY, usedWidth, usedHeight);
  ctx.strokeStyle = '#223246';
  ctx.lineWidth = 1;
  ctx.fillStyle = '#64748b';
  ctx.font = '12px Segoe UI, Arial';
  for (let x = Math.floor(bounds.minX); x <= Math.ceil(bounds.maxX); x++) {{
    const sx = px(x);
    ctx.beginPath();
    ctx.moveTo(sx, offsetY);
    ctx.lineTo(sx, offsetY + usedHeight);
    ctx.stroke();
    ctx.fillText(`${{x}}m`, sx + 4, offsetY + usedHeight + 22);
  }}
  for (let y = Math.floor(bounds.minY); y <= Math.ceil(bounds.maxY); y++) {{
    const sy = py(y);
    ctx.beginPath();
    ctx.moveTo(offsetX, sy);
    ctx.lineTo(offsetX + usedWidth, sy);
    ctx.stroke();
    ctx.fillText(`${{y}}m`, 18, sy + 4);
  }}
}}

function drawPoints() {{
  for (const point of points) {{
    ctx.fillStyle = timeColor(point[2]);
    ctx.globalAlpha = 0.58;
    ctx.fillRect(px(point[0]), py(point[1]), 2, 2);
  }}
  ctx.globalAlpha = 1;
}}

function drawPicks() {{
  const colors = ['#22c55e', '#ef4444'];
  picks.forEach((pick, index) => {{
    const x = px(pick.x);
    const y = py(pick.y);
    ctx.fillStyle = colors[index];
    ctx.strokeStyle = '#020617';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.arc(x, y, 12, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#020617';
    ctx.font = 'bold 16px Segoe UI, Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(index === 0 ? 'A' : 'B', x, y);
  }});
  if (picks.length === 2) {{
    ctx.strokeStyle = '#f8fafc';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(px(picks[0].x), py(picks[0].y));
    ctx.lineTo(px(picks[1].x), py(picks[1].y));
    ctx.stroke();
  }}
}}

function measurementText() {{
  if (picks.length === 0) return 'Click Board A.';
  const a = picks[0];
  if (picks.length === 1) {{
    return `A: (${{a.x.toFixed(3)}}, ${{a.y.toFixed(3)}}) m\\nClick Board B.`;
  }}
  const b = picks[1];
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const distance = Math.hypot(dx, dy);
  let text = '';
  text += `A: (${{a.x.toFixed(3)}}, ${{a.y.toFixed(3)}}) m\\n`;
  text += `B: (${{b.x.toFixed(3)}}, ${{b.y.toFixed(3)}}) m\\n`;
  text += `Measured distance: ${{distance.toFixed(3)}} m\\n`;
  text += `Delta vector: dx=${{dx.toFixed(3)}} m, dy=${{dy.toFixed(3)}} m`;
  if (expectedDistanceM !== null) {{
    const error = distance - expectedDistanceM;
    const percent = 100 * error / expectedDistanceM;
    text += `\\nExpected distance: ${{expectedDistanceM.toFixed(3)}} m`;
    text += `\\nError: ${{error >= 0 ? '+' : ''}}${{error.toFixed(3)}} m (${{percent >= 0 ? '+' : ''}}${{percent.toFixed(1)}}%)`;
  }}
  return text;
}}

function redraw() {{
  drawGrid();
  drawPoints();
  drawPicks();
  readout.textContent = measurementText();
}}

canvas.addEventListener('click', (event) => {{
  const rect = canvas.getBoundingClientRect();
  const x = (event.clientX - rect.left) * canvas.width / rect.width;
  const y = (event.clientY - rect.top) * canvas.height / rect.height;
  if (x < offsetX || x > offsetX + usedWidth || y < offsetY || y > offsetY + usedHeight) return;
  const pick = {{ x: mx(x), y: my(y) }};
  if (picks.length >= 2) picks = [];
  picks.push(pick);
  redraw();
}});

document.getElementById('reset').addEventListener('click', () => {{
  picks = [];
  redraw();
}});
document.getElementById('copy').addEventListener('click', async () => {{
  await navigator.clipboard.writeText(readout.textContent);
}});

redraw();
</script>
</body>
</html>
"""
    output.write_text(document, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render an interactive picker for measuring lidar map distances"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--trajectory", type=Path, required=True, help="ICP trajectory JSON")
    parser.add_argument("--output", type=Path, required=True, help="Output HTML file")
    parser.add_argument("--lidar-angle-offset-deg", type=float, default=125.0)
    parser.add_argument("--min-distance-m", type=float, default=0.20)
    parser.add_argument("--max-distance-m", type=float, default=5.0)
    parser.add_argument("--min-quality", type=int, default=1)
    parser.add_argument("--render-scan-stride", type=int, default=2)
    parser.add_argument("--render-point-stride", type=int, default=1)
    parser.add_argument("--max-valid-points-ratio", type=float, default=1.5)
    parser.add_argument("--max-points", type=int, default=90000)
    parser.add_argument("--expected-distance-m", type=float)
    parser.add_argument("--seed", type=int, default=7)
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
    points = [(point[0], point[1], point[3]) for point in map_points]
    points = downsample(points, args.max_points, args.seed)
    bounds = point_bounds(points, padding_m=0.4)
    render_html(args.output, str(manifest["session_id"]), points, bounds, args.expected_distance_m)
    print(f"Wrote {args.output}")
    print(f"Displayed points: {len(points)}")
    print(
        "Open the HTML, click Board A and Board B, then copy the measurement text."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
