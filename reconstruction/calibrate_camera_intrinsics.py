#!/usr/bin/env python3
"""Calibrate camera intrinsics from checkerboard still images."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

try:
    import cv2
except ImportError as error:  # pragma: no cover - exercised by CLI environment
    raise SystemExit(
        "OpenCV is required for checkerboard calibration. Install "
        "`opencv-python-headless` or run with a Python environment that has cv2."
    ) from error


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def resize_width(image: np.ndarray, width: int) -> np.ndarray:
    height = int(round(image.shape[0] * width / image.shape[1]))
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def write_diagnostic_sheet(
    paths: list[Path],
    detections: dict[str, np.ndarray],
    statuses: dict[str, str],
    pattern_size: tuple[int, int],
    output: Path,
    columns: int,
    thumb_width: int,
) -> None:
    """Write a contact sheet with accepted corners drawn on the source images."""

    if columns <= 0:
        raise ValueError("diagnostic sheet column count must be positive")
    if thumb_width <= 0:
        raise ValueError("diagnostic thumbnail width must be positive")

    tiles: list[np.ndarray] = []
    label_height = 42
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            tile = np.full((thumb_width * 9 // 16 + label_height, thumb_width, 3), 255, np.uint8)
        else:
            corners = detections.get(path.name)
            if corners is not None:
                cv2.drawChessboardCorners(image, pattern_size, corners, True)
            thumb = resize_width(image, thumb_width)
            tile = np.full((thumb.shape[0] + label_height, thumb_width, 3), 255, np.uint8)
            tile[label_height:, :, :] = thumb

        status = statuses.get(path.name, "unknown")
        color = (0, 120, 0) if status == "used" else (0, 0, 220)
        cv2.putText(
            tile,
            path.name,
            (8, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            tile,
            status[:32],
            (8, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA,
        )
        tiles.append(tile)

    max_tile_height = max(tile.shape[0] for tile in tiles)
    rows = math.ceil(len(tiles) / columns)
    sheet = np.full((rows * max_tile_height, columns * thumb_width, 3), 245, np.uint8)
    for index, tile in enumerate(tiles):
        row = index // columns
        col = index % columns
        y = row * max_tile_height
        x = col * thumb_width
        sheet[y : y + tile.shape[0], x : x + tile.shape[1], :] = tile

    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), sheet):
        raise RuntimeError(f"Failed to write diagnostic sheet to {output}")


def image_paths(folder: Path) -> list[Path]:
    paths = sorted(path for path in folder.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
    if not paths:
        raise FileNotFoundError(f"No calibration images found in {folder}")
    return paths


def checkerboard_object_points(
    inner_cols: int,
    inner_rows: int,
    square_size_m: float,
) -> np.ndarray:
    points = np.zeros((inner_cols * inner_rows, 3), np.float32)
    grid = np.mgrid[0:inner_cols, 0:inner_rows].T.reshape(-1, 2)
    points[:, :2] = grid * square_size_m
    return points


def find_corners(gray: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, np.ndarray | None]:
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
        if found:
            return True, corners.astype(np.float32)

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, refined


def per_view_errors(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: tuple[np.ndarray, ...],
    tvecs: tuple[np.ndarray, ...],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> list[float]:
    errors: list[float] = []
    for object_point, image_point, rvec, tvec in zip(
        object_points,
        image_points,
        rvecs,
        tvecs,
    ):
        projected, _ = cv2.projectPoints(
            object_point,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        error = cv2.norm(image_point, projected, cv2.NORM_L2) / len(projected)
        errors.append(float(error))
    return errors


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    """Write a small YAML file without requiring PyYAML."""

    def scalar(value: Any) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, int):
            return str(value)
        if isinstance(value, float):
            if math.isfinite(value):
                return f"{value:.10g}"
            return str(value)
        return json.dumps(str(value))

    def emit(value: Any, indent: int = 0) -> list[str]:
        prefix = " " * indent
        lines: list[str] = []
        if isinstance(value, dict):
            for key, item in value.items():
                if isinstance(item, (dict, list)):
                    lines.append(f"{prefix}{key}:")
                    lines.extend(emit(item, indent + 2))
                else:
                    lines.append(f"{prefix}{key}: {scalar(item)}")
        elif isinstance(value, list):
            for item in value:
                if isinstance(item, (dict, list)):
                    lines.append(f"{prefix}-")
                    lines.extend(emit(item, indent + 2))
                else:
                    lines.append(f"{prefix}- {scalar(item)}")
        else:
            lines.append(f"{prefix}{scalar(value)}")
        return lines

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(emit(data)) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate camera intrinsics")
    parser.add_argument("images", type=Path, help="Folder containing checkerboard images")
    parser.add_argument("--inner-cols", type=int, default=7)
    parser.add_argument("--inner-rows", type=int, default=7)
    parser.add_argument("--square-size-mm", type=float, required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("config") / "camera_intrinsics_pi_camera_v2_1920x1080.yaml",
    )
    parser.add_argument(
        "--diagnostic-sheet",
        type=Path,
        help="Optional output image showing accepted and rejected checkerboard detections",
    )
    parser.add_argument("--diagnostic-columns", type=int, default=5)
    parser.add_argument("--diagnostic-thumb-width", type=int, default=360)
    parser.add_argument("--min-detections", type=int, default=10)
    args = parser.parse_args()

    if args.inner_cols <= 1 or args.inner_rows <= 1:
        parser.error("checkerboard inner corner counts must be greater than 1")
    if args.square_size_mm <= 0:
        parser.error("--square-size-mm must be positive")

    paths = image_paths(args.images.resolve())
    pattern_size = (args.inner_cols, args.inner_rows)
    square_size_m = args.square_size_mm / 1000.0
    template_object_points = checkerboard_object_points(
        args.inner_cols,
        args.inner_rows,
        square_size_m,
    )

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    used_images: list[str] = []
    rejected_images: list[str] = []
    detections: dict[str, np.ndarray] = {}
    statuses: dict[str, str] = {}
    image_size: tuple[int, int] | None = None

    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            rejected_images.append(f"{path.name}: unreadable")
            statuses[path.name] = "unreadable"
            continue
        height, width = image.shape[:2]
        if image_size is None:
            image_size = (width, height)
        elif image_size != (width, height):
            rejected_images.append(
                f"{path.name}: image size {(width, height)} does not match {image_size}"
            )
            statuses[path.name] = "size mismatch"
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = find_corners(gray, pattern_size)
        if not found or corners is None:
            rejected_images.append(f"{path.name}: checkerboard not detected")
            statuses[path.name] = "not detected"
            continue

        object_points.append(template_object_points.copy())
        image_points.append(corners)
        used_images.append(path.name)
        detections[path.name] = corners
        statuses[path.name] = "used"

    if image_size is None:
        raise RuntimeError("No readable calibration images")
    if args.diagnostic_sheet:
        write_diagnostic_sheet(
            paths,
            detections,
            statuses,
            pattern_size,
            args.diagnostic_sheet,
            args.diagnostic_columns,
            args.diagnostic_thumb_width,
        )
        print(f"Wrote diagnostic sheet: {args.diagnostic_sheet}")
    if len(used_images) < args.min_detections:
        raise RuntimeError(
            f"Only {len(used_images)} checkerboards detected; need at least "
            f"{args.min_detections}. Rejected: {rejected_images}"
        )

    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    view_errors = per_view_errors(
        object_points,
        image_points,
        rvecs,
        tvecs,
        camera_matrix,
        distortion,
    )

    output_data = {
        "schema_version": 1,
        "camera": "Raspberry Pi Camera Module 2",
        "calibration_image_size": {
            "width": image_size[0],
            "height": image_size[1],
        },
        "checkerboard": {
            "inner_corners": {
                "cols": args.inner_cols,
                "rows": args.inner_rows,
            },
            "square_size_mm": args.square_size_mm,
            "square_size_m": square_size_m,
        },
        "rms_reprojection_error_px": float(rms),
        "mean_per_view_error_px": float(np.mean(view_errors)),
        "max_per_view_error_px": float(np.max(view_errors)),
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(value) for value in camera_matrix.reshape(-1)],
        },
        "distortion_coefficients": {
            "model": "opencv_plumb_bob",
            "data": [float(value) for value in distortion.reshape(-1)],
        },
        "used_image_count": len(used_images),
        "rejected_image_count": len(rejected_images),
        "used_images": used_images,
        "rejected_images": rejected_images,
        "notes": [
            "Calibrated from Pi still images captured at 1920x1080.",
            "Use these intrinsics only for the same camera mode/resolution unless recalibrated.",
        ],
    }
    write_yaml(args.output, output_data)

    print(f"Images scanned: {len(paths)}")
    print(f"Checkerboards detected: {len(used_images)}")
    print(f"Rejected images: {len(rejected_images)}")
    print(f"RMS reprojection error: {rms:.4f} px")
    print(f"Mean per-view error: {np.mean(view_errors):.4f} px")
    print(f"Max per-view error: {np.max(view_errors):.4f} px")
    print(f"Wrote {args.output}")
    if rejected_images:
        print("Rejected:")
        for item in rejected_images:
            print(f"  - {item}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
