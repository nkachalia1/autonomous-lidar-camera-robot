#!/usr/bin/env python3
"""Run the Fuse lidar/camera GraphDECO 3DGS smoke test in Google Colab.

Upload this file to Colab, select a GPU runtime, then run:

    !python graphdeco_3dgs_smoke_colab.py

The script will:

1. verify CUDA/PyTorch visibility;
2. prompt for the packaged Fuse dataset ZIP if --zip is not provided;
3. clone the official GraphDECO/Inria gaussian-splatting repository;
4. install the CUDA submodules with --no-build-isolation;
5. unpack the dataset and verify the expected COLMAP/GraphDECO folder shape;
6. run a tiny 300-iteration training smoke test with --disable_viewer.

This is a compatibility test, not a quality benchmark.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path


DEFAULT_REPO_URL = "https://github.com/graphdeco-inria/gaussian-splatting"
DEFAULT_CONTENT_ROOT = Path("/content")
DEFAULT_REPO_DIR = DEFAULT_CONTENT_ROOT / "gaussian-splatting"
DEFAULT_DATASET_PARENT = DEFAULT_CONTENT_ROOT / "fuse_graphdeco_dataset"
DEFAULT_OUTPUT_PARENT = DEFAULT_CONTENT_ROOT / "fuse_3dgs_output"


def run(command: list[str], *, cwd: Path | None = None, allow_fail: bool = False) -> int:
    """Run a command with visible logs and optional failure tolerance."""

    location = f"  cwd={cwd}" if cwd else ""
    print("\n$ " + " ".join(shlex.quote(part) for part in command) + location, flush=True)
    result = subprocess.run(command, cwd=str(cwd) if cwd else None)
    if result.returncode and not allow_fail:
        raise subprocess.CalledProcessError(result.returncode, command)
    return result.returncode


def run_python(code: str) -> None:
    run([sys.executable, "-c", code])


def require_colab_or_zip(zip_path: str | None) -> Path:
    """Return a dataset ZIP path, prompting with Colab upload when needed."""

    if zip_path:
        path = Path(zip_path).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(path)
        return path

    try:
        from google.colab import files  # type: ignore
    except Exception as exc:  # pragma: no cover - only exercised outside Colab
        raise RuntimeError(
            "No --zip path was provided and google.colab.files is unavailable. "
            "Run this in Colab or pass --zip /path/to/dataset.zip."
        ) from exc

    print(
        "\nUpload the packaged dataset ZIP, for example:\n"
        "20260625T214456Z-steady-undistorted-graphdeco.zip\n",
        flush=True,
    )
    uploaded = files.upload()
    if not uploaded:
        raise RuntimeError("No dataset ZIP was uploaded")
    first_name = next(iter(uploaded.keys()))
    path = DEFAULT_CONTENT_ROOT / first_name
    if not path.exists():
        # Colab usually writes uploads into /content, but keep this fallback for
        # notebook kernels that use the current working directory.
        path = Path(first_name).resolve()
    if path.suffix.lower() != ".zip":
        raise ValueError(f"Uploaded file is not a .zip: {path}")
    return path


def print_cuda_diagnostics() -> None:
    print("\n=== CUDA / PyTorch diagnostics ===", flush=True)
    run(["nvidia-smi"], allow_fail=True)
    run_python(
        "import torch\n"
        "print('torch', torch.__version__)\n"
        "print('torch cuda', torch.version.cuda)\n"
        "print('cuda available', torch.cuda.is_available())\n"
        "print('cuda device count', torch.cuda.device_count())\n"
        "print('device', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none')\n"
        "raise SystemExit(0 if torch.cuda.is_available() else 2)\n"
    )
    run(["nvcc", "--version"])


def clone_graphdeco(repo_dir: Path, *, clean: bool) -> None:
    print("\n=== Clone GraphDECO gaussian-splatting ===", flush=True)
    if clean and repo_dir.exists():
        shutil.rmtree(repo_dir)
    if repo_dir.exists():
        print(f"Using existing repository: {repo_dir}", flush=True)
        return
    run(["git", "clone", DEFAULT_REPO_URL, "--recursive", str(repo_dir)])


def install_graphdeco(repo_dir: Path) -> None:
    print("\n=== Install Python dependencies and CUDA submodules ===", flush=True)
    run([sys.executable, "-m", "pip", "install", "-q", "--upgrade", "pip", "setuptools", "wheel"])
    run([sys.executable, "-m", "pip", "install", "-q", "plyfile", "tqdm", "opencv-python", "joblib", "ninja"])

    # GraphDECO setup.py files import torch. Colab's default pip build
    # isolation can hide the runtime torch install, so no-build-isolation is
    # deliberate here.
    run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "-v",
            "--no-build-isolation",
            "./submodules/diff-gaussian-rasterization",
        ],
        cwd=repo_dir,
    )
    run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "-v",
            "--no-build-isolation",
            "./submodules/simple-knn",
        ],
        cwd=repo_dir,
    )
    fused_status = run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "-v",
            "--no-build-isolation",
            "./submodules/fused-ssim",
        ],
        cwd=repo_dir,
        allow_fail=True,
    )
    if fused_status:
        print(
            "WARNING: fused-ssim failed to install. This is optional; "
            "GraphDECO train.py can fall back to Python SSIM.",
            flush=True,
        )


def unpack_dataset(zip_path: Path, dataset_parent: Path) -> Path:
    print("\n=== Unpack and validate dataset ===", flush=True)
    dataset_root = dataset_parent / zip_path.stem
    shutil.rmtree(dataset_root, ignore_errors=True)
    dataset_root.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path) as archive:
        archive.extractall(dataset_root)

    required = [
        dataset_root / "images",
        dataset_root / "sparse" / "0" / "cameras.bin",
        dataset_root / "sparse" / "0" / "images.bin",
        dataset_root / "sparse" / "0" / "points3D.bin",
        dataset_root / "sparse" / "0" / "points3D.ply",
    ]
    missing = [path for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError(
            "Dataset is missing required GraphDECO/COLMAP files:\n"
            + "\n".join(str(path) for path in missing)
        )

    images = sorted(path for path in (dataset_root / "images").iterdir() if path.is_file())
    sparse_files = sorted(path.name for path in (dataset_root / "sparse" / "0").iterdir())
    print(f"dataset_root: {dataset_root}", flush=True)
    print(f"image count: {len(images)}", flush=True)
    print(f"sparse files: {sparse_files}", flush=True)

    report_path = dataset_root / "graphdeco_input_check.json"
    if report_path.exists():
        report = json.loads(report_path.read_text(encoding="utf-8"))
        print(
            "readiness report: "
            f"ready={report.get('ready_for_graphdeco_loader')} "
            f"points={report.get('sparse_points', {}).get('count')} "
            f"track_refs={report.get('sparse_points', {}).get('track_reference_count')}",
            flush=True,
        )
    return dataset_root


def run_training(
    repo_dir: Path,
    dataset_root: Path,
    output_parent: Path,
    *,
    iterations: int,
    resolution: int,
) -> Path:
    print("\n=== Run GraphDECO training smoke test ===", flush=True)
    output_dir = output_parent / f"{dataset_root.name}-smoke"
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        "train.py",
        "-s",
        str(dataset_root),
        "-m",
        str(output_dir),
        "--images",
        "images",
        "--resolution",
        str(resolution),
        "--iterations",
        str(iterations),
        "--test_iterations",
        str(iterations),
        "--save_iterations",
        str(iterations),
        "--data_device",
        "cpu",
        "--disable_viewer",
    ]
    run(command, cwd=repo_dir)
    print(f"\nTraining smoke output: {output_dir}", flush=True)
    for path in sorted(output_dir.rglob("*"))[:80]:
        print(path.relative_to(output_dir), flush=True)
    return output_dir


def zip_output(output_dir: Path) -> Path:
    archive = Path(shutil.make_archive(str(output_dir), "zip", output_dir))
    print(f"\nOutput archive: {archive}", flush=True)
    try:
        from google.colab import files  # type: ignore

        files.download(str(archive))
    except Exception:
        print("Download manually if not running in Colab.", flush=True)
    return archive


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the Fuse/GraphDECO 3D Gaussian Splatting Colab smoke test"
    )
    parser.add_argument("--zip", help="Optional path to packaged Fuse GraphDECO dataset ZIP")
    parser.add_argument("--iterations", type=int, default=300, help="Training iterations")
    parser.add_argument(
        "--resolution",
        type=int,
        default=8,
        help="GraphDECO resolution argument; 8 means 1/8 input size",
    )
    parser.add_argument(
        "--repo-dir",
        type=Path,
        default=DEFAULT_REPO_DIR,
        help="Where to clone/use gaussian-splatting",
    )
    parser.add_argument(
        "--keep-repo",
        action="store_true",
        help="Reuse an existing gaussian-splatting checkout instead of deleting it first",
    )
    parser.add_argument(
        "--skip-install",
        action="store_true",
        help="Skip dependency/submodule install if already installed in this runtime",
    )
    parser.add_argument(
        "--no-download",
        action="store_true",
        help="Do not zip/download the output directory at the end",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if os.name == "nt":
        print("WARNING: this script is intended for Colab/Linux with CUDA.", flush=True)

    print_cuda_diagnostics()
    zip_path = require_colab_or_zip(args.zip)
    print(f"\nDataset ZIP: {zip_path}", flush=True)

    clone_graphdeco(args.repo_dir, clean=not args.keep_repo)
    if not args.skip_install:
        install_graphdeco(args.repo_dir)

    dataset_root = unpack_dataset(zip_path, DEFAULT_DATASET_PARENT)
    output_dir = run_training(
        args.repo_dir,
        dataset_root,
        DEFAULT_OUTPUT_PARENT,
        iterations=args.iterations,
        resolution=args.resolution,
    )
    if not args.no_download:
        zip_output(output_dir)

    print("\nPASS: GraphDECO training smoke test completed.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
