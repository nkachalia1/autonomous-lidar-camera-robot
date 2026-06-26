#!/usr/bin/env python3
"""Compare GraphDECO rendered train views against their ground-truth images."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def psnr(mse: float) -> float:
    if mse <= 1e-12:
        return float("inf")
    return 20.0 * math.log10(255.0 / math.sqrt(mse))


def load_pair(gt_path: Path, render_path: Path) -> tuple[np.ndarray, np.ndarray]:
    gt = cv2.imread(str(gt_path), cv2.IMREAD_COLOR)
    render = cv2.imread(str(render_path), cv2.IMREAD_COLOR)
    if gt is None:
        raise ValueError(f"failed to read ground truth image: {gt_path}")
    if render is None:
        raise ValueError(f"failed to read render image: {render_path}")
    if gt.shape != render.shape:
        render = cv2.resize(render, (gt.shape[1], gt.shape[0]), interpolation=cv2.INTER_AREA)
    return gt, render


def image_metrics(gt: np.ndarray, render: np.ndarray) -> dict[str, float]:
    diff = gt.astype(np.float32) - render.astype(np.float32)
    mae = float(np.mean(np.abs(diff)))
    mse = float(np.mean(diff * diff))
    return {
        "mae": round(mae, 3),
        "mse": round(mse, 3),
        "psnr_db": round(psnr(mse), 3),
    }


def make_contact_sheet(
    pairs: list[tuple[Path, Path, dict[str, float]]],
    output: Path,
    *,
    max_pairs: int,
    thumb_width: int,
) -> None:
    selected = pairs
    if len(pairs) > max_pairs:
        indices = np.linspace(0, len(pairs) - 1, max_pairs).round().astype(int)
        selected = [pairs[index] for index in indices]

    rows: list[np.ndarray] = []
    font = cv2.FONT_HERSHEY_SIMPLEX
    for gt_path, render_path, metrics in selected:
        gt, render = load_pair(gt_path, render_path)
        diff = cv2.absdiff(gt, render)
        scale = thumb_width / gt.shape[1]
        thumb_height = max(1, int(round(gt.shape[0] * scale)))
        gt_thumb = cv2.resize(gt, (thumb_width, thumb_height), interpolation=cv2.INTER_AREA)
        render_thumb = cv2.resize(render, (thumb_width, thumb_height), interpolation=cv2.INTER_AREA)
        diff_thumb = cv2.resize(diff, (thumb_width, thumb_height), interpolation=cv2.INTER_AREA)
        diff_thumb = np.clip(diff_thumb * 3, 0, 255).astype(np.uint8)

        label_height = 34
        row = np.full((thumb_height + label_height, thumb_width * 3, 3), 245, dtype=np.uint8)
        row[label_height:, 0:thumb_width] = gt_thumb
        row[label_height:, thumb_width : 2 * thumb_width] = render_thumb
        row[label_height:, 2 * thumb_width : 3 * thumb_width] = diff_thumb

        label = (
            f"{gt_path.stem}: GT | render | diff x3; "
            f"MAE {metrics['mae']:.1f}; PSNR {metrics['psnr_db']:.1f} dB"
        )
        cv2.putText(row, label, (10, 23), font, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
        rows.append(row)

    sheet = np.vstack(rows)
    output.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(output), sheet)
    if not ok:
        raise RuntimeError(f"failed to write contact sheet: {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare GraphDECO render.py train-view output")
    parser.add_argument("train_dir", type=Path, help="Directory containing gt/ and renders/")
    parser.add_argument("--output", type=Path, required=True, help="Contact sheet PNG output")
    parser.add_argument("--json-output", type=Path, required=True, help="Metrics JSON output")
    parser.add_argument("--max-pairs", type=int, default=12)
    parser.add_argument("--thumb-width", type=int, default=360)
    args = parser.parse_args()

    gt_dir = args.train_dir / "gt"
    render_dir = args.train_dir / "renders"
    gt_paths = sorted(gt_dir.glob("*.png"))
    render_paths = sorted(render_dir.glob("*.png"))
    if not gt_paths:
        raise FileNotFoundError(f"no ground-truth PNGs found in {gt_dir}")
    if len(gt_paths) != len(render_paths):
        raise ValueError(f"GT/render count mismatch: {len(gt_paths)} vs {len(render_paths)}")

    pairs: list[tuple[Path, Path, dict[str, float]]] = []
    for gt_path, render_path in zip(gt_paths, render_paths, strict=True):
        if gt_path.name != render_path.name:
            raise ValueError(f"GT/render filename mismatch: {gt_path.name} vs {render_path.name}")
        gt, render = load_pair(gt_path, render_path)
        pairs.append((gt_path, render_path, image_metrics(gt, render)))

    summary = {
        "train_dir": str(args.train_dir),
        "frame_count": len(pairs),
        "mean_mae": round(float(np.mean([item[2]["mae"] for item in pairs])), 3),
        "median_mae": round(float(np.median([item[2]["mae"] for item in pairs])), 3),
        "mean_psnr_db": round(float(np.mean([item[2]["psnr_db"] for item in pairs])), 3),
        "median_psnr_db": round(float(np.median([item[2]["psnr_db"] for item in pairs])), 3),
        "frames": [
            {
                "name": gt_path.name,
                **metrics,
            }
            for gt_path, _render_path, metrics in pairs
        ],
    }

    make_contact_sheet(
        pairs,
        args.output,
        max_pairs=args.max_pairs,
        thumb_width=args.thumb_width,
    )
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {args.output}")
    print(f"Wrote {args.json_output}")
    print(f"Frames: {summary['frame_count']}")
    print(f"MAE mean/median: {summary['mean_mae']:.3f}/{summary['median_mae']:.3f}")
    print(f"PSNR mean/median dB: {summary['mean_psnr_db']:.3f}/{summary['median_psnr_db']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
