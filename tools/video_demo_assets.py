#!/usr/bin/env python3
"""Create a compact GIF and contact sheet for the project README.

Requires ``imageio`` and ``imageio-ffmpeg``. They are intentionally not a
project runtime dependency; this is a workstation documentation utility.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v3 as iio
from PIL import Image, ImageDraw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--gif", type=Path, required=True)
    parser.add_argument("--contact-sheet", type=Path)
    parser.add_argument("--start-s", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, default=12.0)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier. Default: 1.0.",
    )
    parser.add_argument("--width", type=int, default=480)
    parser.add_argument("--sheet-frames", type=int, default=8)
    return parser.parse_args()


def fit(frame: Image.Image, width: int) -> Image.Image:
    height = round(frame.height * width / frame.width)
    return frame.resize((width, height), Image.Resampling.LANCZOS)


def main() -> int:
    args = parse_args()
    if not args.video.exists():
        raise SystemExit(f"Video does not exist: {args.video}")
    if args.duration_s <= 0 or args.fps <= 0 or args.width <= 0 or args.speed <= 0:
        raise SystemExit("--duration-s, --fps, --width, and --speed must be positive")

    meta = iio.immeta(args.video, plugin="FFMPEG")
    source_fps = float(meta["fps"])
    first = Image.fromarray(iio.imread(args.video, index=0, plugin="FFMPEG"))
    first = fit(first, args.width)

    frame_count = max(1, round(args.duration_s * args.fps / args.speed))
    source_indices = [
        max(0, round((args.start_s + index * args.speed / args.fps) * source_fps))
        for index in range(frame_count)
    ]
    frames = [
        fit(Image.fromarray(iio.imread(args.video, index=index, plugin="FFMPEG")), args.width)
        for index in source_indices
    ]

    args.gif.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.gif,
        save_all=True,
        append_images=frames[1:],
        duration=round(1000 / args.fps),
        loop=0,
        optimize=True,
        disposal=2,
    )
    print(f"Wrote GIF: {args.gif} ({args.gif.stat().st_size:,} bytes)")

    if args.contact_sheet is not None:
        sample_count = min(args.sheet_frames, len(frames))
        selected = [frames[round(index * (len(frames) - 1) / max(1, sample_count - 1))]
                    for index in range(sample_count)]
        thumb_width = 240
        thumbs = [fit(frame, thumb_width) for frame in selected]
        cols = 4
        rows = (len(thumbs) + cols - 1) // cols
        label_height = 24
        sheet = Image.new(
            "RGB",
            (cols * thumb_width, rows * (thumbs[0].height + label_height)),
            "#101827",
        )
        draw = ImageDraw.Draw(sheet)
        for index, thumb in enumerate(thumbs):
            x = (index % cols) * thumb_width
            y = (index // cols) * (thumb.height + label_height)
            sheet.paste(thumb, (x, y))
            sample_s = args.start_s + source_indices[
                round(index * (len(source_indices) - 1) / max(1, sample_count - 1))
            ] / source_fps - args.start_s
            draw.text((x + 6, y + thumb.height + 4), f"{sample_s:.1f}s", fill="white")
        args.contact_sheet.parent.mkdir(parents=True, exist_ok=True)
        sheet.save(args.contact_sheet, quality=88)
        print(f"Wrote contact sheet: {args.contact_sheet}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
