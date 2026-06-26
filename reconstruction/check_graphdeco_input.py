#!/usr/bin/env python3
"""Check whether a COLMAP export is ready for GraphDECO Gaussian Splatting.

This is a local, dependency-light smoke test.  It does not train Gaussian
splats and it does not import GraphDECO code.  Instead, it checks the folder
shape and camera model assumptions used by the official GraphDECO COLMAP
loader, then optionally writes ``sparse/0/points3D.ply`` from
``points3D.txt`` in the same field layout GraphDECO expects.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from render_colmap_export_preview import data_lines, parse_cameras, parse_images, parse_points


SUPPORTED_CAMERA_MODELS = {"PINHOLE", "SIMPLE_PINHOLE"}


def count_point_track_refs(points3d_txt: Path) -> int:
    """Count COLMAP POINT3D track references in a text model.

    A ``points3D.txt`` line stores 8 fixed fields, followed by zero or more
    ``IMAGE_ID POINT2D_IDX`` pairs.  Our current diagnostic export intentionally
    has no true COLMAP feature tracks, so this number is expected to be zero for
    the first splatting handoff.
    """

    total = 0
    for line in data_lines(points3d_txt):
        parts = line.split()
        total += max(0, len(parts) - 8) // 2
    return total


def write_graphdeco_points_ply(output: Path, points: list[dict[str, Any]]) -> None:
    """Write an ASCII PLY with GraphDECO-compatible vertex fields."""

    output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(points)}",
        "property float x",
        "property float y",
        "property float z",
        "property float nx",
        "property float ny",
        "property float nz",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "end_header",
    ]
    for point in points:
        x, y, z = point["xyz"]
        red, green, blue = point["rgb"]
        lines.append(
            f"{x:.9f} {y:.9f} {z:.9f} "
            f"0.000000000 0.000000000 0.000000000 "
            f"{red:d} {green:d} {blue:d}"
        )
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def inspect_ply(path: Path) -> dict[str, Any]:
    header: list[str] = []
    vertex_count: int | None = None
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            header.append(stripped)
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[-1])
            if stripped == "end_header":
                break
        data_rows = sum(1 for line in stream if line.strip())
    return {
        "path": str(path),
        "exists": path.exists(),
        "format": header[1] if len(header) > 1 else "",
        "vertex_count_header": vertex_count,
        "vertex_count_rows": data_rows,
        "has_graphdeco_fields": all(
            field in header
            for field in (
                "property float x",
                "property float y",
                "property float z",
                "property float nx",
                "property float ny",
                "property float nz",
                "property uchar red",
                "property uchar green",
                "property uchar blue",
            )
        ),
    }


def validate_graphdeco_input(
    export_dir: Path,
    *,
    write_ply: bool,
    overwrite_ply: bool,
) -> dict[str, Any]:
    export_dir = export_dir.resolve()
    image_dir = export_dir / "images"
    sparse_dir = export_dir / "sparse" / "0"
    cameras_txt = sparse_dir / "cameras.txt"
    images_txt = sparse_dir / "images.txt"
    points3d_txt = sparse_dir / "points3D.txt"
    points3d_ply = sparse_dir / "points3D.ply"

    required_text = [cameras_txt, images_txt, points3d_txt]
    missing_required = [str(path) for path in required_text if not path.exists()]
    if missing_required:
        return {
            "export_dir": str(export_dir),
            "ready_for_graphdeco_loader": False,
            "errors": [f"missing required COLMAP text file: {path}" for path in missing_required],
            "warnings": [],
        }

    cameras = parse_cameras(cameras_txt)
    images = parse_images(images_txt)
    points = parse_points(points3d_txt)

    missing_images = [
        image["name"]
        for image in images
        if not (image_dir / image["name"]).exists()
    ]
    camera_models = sorted({camera["model"] for camera in cameras.values()})
    unsupported_models = [
        model for model in camera_models if model not in SUPPORTED_CAMERA_MODELS
    ]
    bin_files = {
        "cameras.bin": (sparse_dir / "cameras.bin").exists(),
        "images.bin": (sparse_dir / "images.bin").exists(),
        "points3D.bin": (sparse_dir / "points3D.bin").exists(),
    }
    text_files = {
        "cameras.txt": cameras_txt.exists(),
        "images.txt": images_txt.exists(),
        "points3D.txt": points3d_txt.exists(),
    }

    errors: list[str] = []
    warnings: list[str] = []
    if not image_dir.exists():
        errors.append("missing top-level images/ directory")
    if missing_images:
        errors.append(f"{len(missing_images)} image file(s) referenced by images.txt are missing")
    if unsupported_models:
        errors.append(
            "unsupported camera model(s) for GraphDECO undistorted COLMAP loader: "
            + ", ".join(unsupported_models)
        )
    if not points:
        errors.append("points3D.txt has no sparse seed points")
    if not all(bin_files.values()):
        warnings.append(
            "one or more binary COLMAP files are missing; GraphDECO can fall back to text files"
        )

    track_refs = count_point_track_refs(points3d_txt)
    if track_refs == 0:
        warnings.append(
            "points3D.txt has zero COLMAP feature-track references; this is acceptable "
            "for a loader smoke test but weak for real splatting quality"
        )

    ply_written = False
    if write_ply and points and (overwrite_ply or not points3d_ply.exists()):
        write_graphdeco_points_ply(points3d_ply, points)
        ply_written = True
    elif write_ply and points3d_ply.exists():
        warnings.append("points3D.ply already exists; kept existing file")

    ply_info: dict[str, Any] | None = None
    if points3d_ply.exists():
        ply_info = inspect_ply(points3d_ply)
        if ply_info["vertex_count_header"] != len(points):
            errors.append("points3D.ply vertex count does not match points3D.txt")
        if not ply_info["has_graphdeco_fields"]:
            errors.append("points3D.ply does not contain GraphDECO-compatible vertex fields")

    return {
        "export_dir": str(export_dir),
        "ready_for_graphdeco_loader": not errors,
        "errors": errors,
        "warnings": warnings,
        "cameras": {
            "count": len(cameras),
            "models": camera_models,
        },
        "images": {
            "count": len(images),
            "directory": str(image_dir),
            "missing_count": len(missing_images),
            "missing_examples": missing_images[:10],
        },
        "sparse_points": {
            "count": len(points),
            "track_reference_count": track_refs,
        },
        "colmap_files": {
            "text": text_files,
            "binary": bin_files,
        },
        "points3d_ply": {
            "path": str(points3d_ply),
            "written_this_run": ply_written,
            "inspection": ply_info,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a COLMAP-style export for the official GraphDECO "
            "Gaussian Splatting loader"
        )
    )
    parser.add_argument("export_dir", type=Path, help="Directory with images/ and sparse/0/")
    parser.add_argument(
        "--json-output",
        type=Path,
        help="Optional path for a machine-readable readiness report",
    )
    parser.add_argument(
        "--no-write-ply",
        action="store_true",
        help="Check readiness without creating sparse/0/points3D.ply",
    )
    parser.add_argument(
        "--overwrite-ply",
        action="store_true",
        help="Rewrite sparse/0/points3D.ply if it already exists",
    )
    args = parser.parse_args()

    report = validate_graphdeco_input(
        args.export_dir,
        write_ply=not args.no_write_ply,
        overwrite_ply=args.overwrite_ply,
    )

    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"Wrote {args.json_output}")

    status = "PASS" if report["ready_for_graphdeco_loader"] else "FAIL"
    print(f"{status}: GraphDECO input readiness")
    print(f"Export: {report['export_dir']}")
    if "cameras" in report:
        print(f"Cameras: {report['cameras']['count']} ({', '.join(report['cameras']['models'])})")
        print(f"Images: {report['images']['count']} missing={report['images']['missing_count']}")
        print(
            "Sparse points: "
            f"{report['sparse_points']['count']} "
            f"track_refs={report['sparse_points']['track_reference_count']}"
        )
        print(f"points3D.ply: {report['points3d_ply']['path']}")
        print(f"PLY written this run: {report['points3d_ply']['written_this_run']}")
    for warning in report["warnings"]:
        print(f"WARNING: {warning}")
    for error in report["errors"]:
        print(f"ERROR: {error}")
    return 0 if report["ready_for_graphdeco_loader"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
