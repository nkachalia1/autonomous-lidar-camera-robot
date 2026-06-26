#!/usr/bin/env python3
"""Package a COLMAP-style export for a GraphDECO Gaussian Splatting smoke test.

The package intentionally contains only the files the downstream loader needs:
``images/`` and ``sparse/0`` plus small project metadata.  Raw capture folders
and larger intermediate artifacts are not included.
"""

from __future__ import annotations

import argparse
import json
import zipfile
from pathlib import Path
from typing import Iterable

from check_graphdeco_input import validate_graphdeco_input


DEFAULT_EXCLUDE_NAMES = {"raw_images", "sparse-bin", "preview.svg"}


def iter_package_files(export_dir: Path) -> Iterable[Path]:
    """Yield files that should be included in the uploadable dataset package."""

    image_dir = export_dir / "images"
    sparse_dir = export_dir / "sparse" / "0"
    metadata_candidates = [
        export_dir / "README.md",
        export_dir / "export_manifest.json",
        export_dir / "preview_validation.json",
        export_dir / "graphdeco_input_check.json",
    ]

    if image_dir.exists():
        yield from sorted(path for path in image_dir.rglob("*") if path.is_file())
    if sparse_dir.exists():
        yield from sorted(path for path in sparse_dir.rglob("*") if path.is_file())
    for path in metadata_candidates:
        if path.exists():
            yield path


def package_dataset(export_dir: Path, output: Path, *, overwrite_ply: bool) -> dict[str, object]:
    export_dir = export_dir.resolve()
    output = output.resolve()
    report = validate_graphdeco_input(
        export_dir,
        write_ply=True,
        overwrite_ply=overwrite_ply,
    )
    if not report["ready_for_graphdeco_loader"]:
        errors = "\n".join(f"- {error}" for error in report["errors"])
        raise ValueError(f"export is not ready for GraphDECO packaging:\n{errors}")

    files = list(dict.fromkeys(iter_package_files(export_dir)))
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, mode="w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in files:
            archive.write(path, path.relative_to(export_dir).as_posix())

    summary = {
        "export_dir": str(export_dir),
        "output": str(output),
        "ready_for_graphdeco_loader": report["ready_for_graphdeco_loader"],
        "warnings": report["warnings"],
        "file_count": len(files),
        "zip_size_bytes": output.stat().st_size,
        "images": report["images"],
        "cameras": report["cameras"],
        "sparse_points": report["sparse_points"],
        "points3d_ply": report["points3d_ply"],
        "included_roots": ["images/", "sparse/0/"],
        "excluded_roots": sorted(DEFAULT_EXCLUDE_NAMES),
    }
    summary_path = output.with_suffix(".package_manifest.json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    summary["package_manifest"] = str(summary_path)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create an uploadable ZIP for a GraphDECO Gaussian Splatting smoke test"
    )
    parser.add_argument("export_dir", type=Path, help="COLMAP-style export directory")
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output .zip path, usually under data/exports/gaussian-splatting/",
    )
    parser.add_argument(
        "--overwrite-ply",
        action="store_true",
        help="Regenerate sparse/0/points3D.ply before packaging",
    )
    args = parser.parse_args()

    summary = package_dataset(args.export_dir, args.output, overwrite_ply=args.overwrite_ply)
    print(f"Wrote {summary['output']}")
    print(f"Wrote {summary['package_manifest']}")
    print(f"Files: {summary['file_count']}")
    print(f"ZIP size: {summary['zip_size_bytes']} bytes")
    print(
        "Images: "
        f"{summary['images']['count']} missing={summary['images']['missing_count']}"
    )
    print(
        "Sparse points: "
        f"{summary['sparse_points']['count']} "
        f"track_refs={summary['sparse_points']['track_reference_count']}"
    )
    for warning in summary["warnings"]:
        print(f"WARNING: {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
