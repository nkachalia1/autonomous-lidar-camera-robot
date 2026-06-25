#!/usr/bin/env python3
"""Extract sampled camera frames and render them beside estimated poses.

This script consumes the JSON from ``render_camera_pose_timeline.py``.  The
timeline JSON already contains the frame indices and timestamp-associated poses;
this script adds the missing human visual: decoded camera thumbnails.
"""

from __future__ import annotations

import argparse
import base64
import html
import json
import shutil
import subprocess
from pathlib import Path
from typing import Any


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def find_ffmpeg(explicit_path: Path | None) -> str:
    if explicit_path is not None:
        if not explicit_path.exists():
            raise FileNotFoundError(f"--ffmpeg does not exist: {explicit_path}")
        return str(explicit_path)

    system_ffmpeg = shutil.which("ffmpeg")
    if system_ffmpeg:
        return system_ffmpeg

    try:
        import imageio_ffmpeg  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(
            "ffmpeg was not found on PATH and Python package imageio-ffmpeg is "
            "not installed. Install one lightweight option with: "
            "python -m pip install imageio-ffmpeg"
        ) from exc

    return imageio_ffmpeg.get_ffmpeg_exe()


def extract_frame(
    ffmpeg: str,
    video: Path,
    frame_index: int,
    output: Path,
    thumbnail_width: int,
    jpeg_quality: int,
    force: bool,
) -> None:
    if output.exists() and output.stat().st_size > 0 and not force:
        return

    output.parent.mkdir(parents=True, exist_ok=True)
    video_filter = f"select=eq(n\\,{frame_index}),scale={thumbnail_width}:-2"
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        str(video),
        "-vf",
        video_filter,
        "-frames:v",
        "1",
        "-q:v",
        str(jpeg_quality),
        str(output),
    ]
    result = subprocess.run(command, text=True, capture_output=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"ffmpeg failed while extracting frame {frame_index}:\n"
            f"{result.stderr.strip()}"
        )
    if not output.exists() or output.stat().st_size == 0:
        raise RuntimeError(f"ffmpeg produced no output for frame {frame_index}")


def image_data_uri(path: Path) -> str:
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:image/jpeg;base64,{encoded}"


def render_svg(
    output: Path,
    session_id: str,
    pose_json_path: Path,
    samples: list[dict[str, Any]],
    frame_paths: list[Path],
    thumbnail_width: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    columns = 3
    rows = (len(samples) + columns - 1) // columns
    tile_width = 560
    image_width = 520
    image_height = round(image_width * 9 / 16)
    tile_height = image_height + 128
    margin_x = 48
    margin_top = 136
    gap_x = 28
    gap_y = 30
    width = margin_x * 2 + columns * tile_width + (columns - 1) * gap_x
    height = margin_top + rows * tile_height + (rows - 1) * gap_y + 58

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        (
            f'<text x="48" y="52" fill="#f8fafc" font-size="30" font-weight="700">'
            f'Session {html.escape(session_id)} camera pose contact sheet</text>'
        ),
        (
            f'<text x="48" y="82" fill="#94a3b8" font-size="16">'
            'Decoded H.264 frames are paired with timestamp-matched ICP lidar poses.</text>'
        ),
        (
            f'<text x="48" y="106" fill="#fbbf24" font-size="14">'
            f'Source poses: {html.escape(str(pose_json_path))}; '
            f'thumbnails decoded at width {thumbnail_width}px.</text>'
        ),
    ]

    for index, (sample, frame_path) in enumerate(zip(samples, frame_paths)):
        column = index % columns
        row = index // columns
        x = margin_x + column * (tile_width + gap_x)
        y = margin_top + row * (tile_height + gap_y)
        image_x = x + 20
        image_y = y + 54
        camera_pose = sample["camera_pose"]

        svg.extend(
            [
                (
                    f'<rect x="{x}" y="{y}" width="{tile_width}" height="{tile_height}" '
                    'rx="18" fill="#0d1726" stroke="#334155"/>'
                ),
                (
                    f'<text x="{x + 20}" y="{y + 34}" fill="#f8fafc" '
                    'font-size="21" font-weight="800">'
                    f'Sample {sample["sample_number"]}: frame {sample["frame_index"]}</text>'
                ),
                (
                    f'<image x="{image_x}" y="{image_y}" width="{image_width}" '
                    f'height="{image_height}" preserveAspectRatio="xMidYMid meet" '
                    f'href="{image_data_uri(frame_path)}"/>'
                ),
                (
                    f'<rect x="{image_x}" y="{image_y}" width="156" height="34" '
                    'rx="17" fill="#020617" fill-opacity="0.78"/>'
                ),
                (
                    f'<text x="{image_x + 16}" y="{image_y + 23}" fill="#f8fafc" '
                    f'font-size="15" font-weight="700">t = {sample["video_time_s"]:.3f}s</text>'
                ),
                (
                    f'<text x="{x + 20}" y="{image_y + image_height + 32}" '
                    f'fill="#cbd5e1" font-size="15">'
                    f'camera pose: x={camera_pose["x_m"]:+.3f} m, '
                    f'y={camera_pose["y_m"]:+.3f} m, '
                    f'yaw={camera_pose["theta_deg"]:+.1f} deg</text>'
                ),
                (
                    f'<text x="{x + 20}" y="{image_y + image_height + 56}" '
                    f'fill="#64748b" font-size="13">'
                    f'{html.escape(str(frame_path))}</text>'
                ),
            ]
        )

    svg.append("</svg>")
    output.write_text("\n".join(svg), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract sampled H.264 camera frames and render a pose contact sheet"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--pose-json", type=Path, required=True, help="Camera pose JSON")
    parser.add_argument("--output", type=Path, required=True, help="Output SVG path")
    parser.add_argument(
        "--frame-dir",
        type=Path,
        help="Directory for extracted JPEG thumbnails",
    )
    parser.add_argument("--ffmpeg", type=Path, help="Optional ffmpeg executable path")
    parser.add_argument("--thumbnail-width", type=int, default=640)
    parser.add_argument("--jpeg-quality", type=int, default=4)
    parser.add_argument("--force", action="store_true", help="Re-extract existing frames")
    args = parser.parse_args()

    if args.thumbnail_width <= 0:
        parser.error("--thumbnail-width must be positive")
    if not 1 <= args.jpeg_quality <= 31:
        parser.error("--jpeg-quality must be between 1 and 31")

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    pose_payload = load_json(args.pose_json)
    samples = pose_payload["sampled_frames"]
    if not samples:
        raise ValueError("pose JSON does not contain any sampled frames")

    video = session / manifest["camera"]["video"]
    if not video.exists():
        raise FileNotFoundError(f"Camera video not found: {video}")

    frame_dir = args.frame_dir
    if frame_dir is None:
        frame_dir = args.output.parent / f"{manifest['session_id']}-camera-samples"
    frame_dir.mkdir(parents=True, exist_ok=True)

    ffmpeg = find_ffmpeg(args.ffmpeg)
    frame_paths: list[Path] = []
    for sample in samples:
        frame_index = int(sample["frame_index"])
        sample_number = int(sample["sample_number"])
        output_frame = frame_dir / f"sample-{sample_number:02d}-frame-{frame_index:04d}.jpg"
        extract_frame(
            ffmpeg,
            video,
            frame_index,
            output_frame,
            args.thumbnail_width,
            args.jpeg_quality,
            args.force,
        )
        frame_paths.append(output_frame)

    render_svg(
        args.output,
        str(manifest["session_id"]),
        args.pose_json,
        samples,
        frame_paths,
        args.thumbnail_width,
    )

    print(f"Wrote {args.output}")
    print(f"Extracted/reused frames in {frame_dir}")
    print(f"Samples: {len(samples)}")
    print(f"ffmpeg: {ffmpeg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
