#!/usr/bin/env python3
"""Export lidar-anchored camera poses in a COLMAP-style text model.

This is a bridge artifact for downstream photogrammetry / Gaussian-splatting
experiments.  It does not run COLMAP, NeRF, or splatting itself.  It writes a
minimal text model with known camera poses, optional undistorted images, and an
optional sparse diagnostic point cloud.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from compare_camera_lidar_motion import load_intrinsics, prepare_frames
from render_sparse_fused_feature_map import (
    camera_center_world,
    camera_rotation_world_from_camera,
    load_rig_defaults,
)


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def load_intrinsics_payload(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return COLMAP Hamilton quaternion (qw, qx, qy, qz) from a rotation matrix."""

    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s

    quaternion = np.array([qw, qx, qy, qz], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    return tuple(float(value) for value in quaternion)


def colmap_pose_from_sample(
    sample: dict[str, Any],
    rig_values: dict[str, float],
) -> dict[str, Any]:
    center_world = camera_center_world(sample, rig_values["camera_height_m"])
    rotation_world_from_camera = camera_rotation_world_from_camera(
        float(sample["camera_pose"]["theta_rad"]),
        rig_values["camera_roll_deg"],
        rig_values["camera_pitch_deg"],
        rig_values["camera_yaw_deg"],
    )
    rotation_camera_from_world = rotation_world_from_camera.T
    translation = -rotation_camera_from_world @ center_world
    return {
        "center_world_m": center_world.tolist(),
        "rotation_camera_from_world": rotation_camera_from_world.tolist(),
        "qvec": rotation_matrix_to_quaternion(rotation_camera_from_world),
        "tvec": tuple(float(value) for value in translation),
    }


def format_floats(values: list[float] | tuple[float, ...]) -> str:
    return " ".join(f"{value:.12g}" for value in values)


def write_cameras_txt(
    path: Path,
    camera_model: str,
    width: int,
    height: int,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> list[float]:
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    dist = distortion.reshape(-1).tolist()
    k1 = float(dist[0]) if len(dist) > 0 else 0.0
    k2 = float(dist[1]) if len(dist) > 1 else 0.0
    p1 = float(dist[2]) if len(dist) > 2 else 0.0
    p2 = float(dist[3]) if len(dist) > 3 else 0.0
    k3 = float(dist[4]) if len(dist) > 4 else 0.0

    if camera_model == "PINHOLE":
        params = [fx, fy, cx, cy]
    elif camera_model == "OPENCV":
        params = [fx, fy, cx, cy, k1, k2, p1, p2]
    elif camera_model == "FULL_OPENCV":
        params = [fx, fy, cx, cy, k1, k2, p1, p2, k3, 0.0, 0.0, 0.0]
    else:
        raise ValueError(f"unsupported camera model: {camera_model}")

    text = [
        "# Camera list with one line of data per camera:",
        "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]",
        "# Number of cameras: 1",
        f"1 {camera_model} {width} {height} {format_floats(params)}",
        "",
    ]
    path.write_text("\n".join(text), encoding="utf-8")
    return params


def write_images_txt(
    path: Path,
    samples: list[dict[str, Any]],
    image_names: list[str],
    rig_values: dict[str, float],
) -> list[dict[str, Any]]:
    lines = [
        "# Image list with two lines of data per image:",
        "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME",
        "#   POINTS2D[] as (X, Y, POINT3D_ID)",
        f"# Number of images: {len(samples)}, mean observations per image: 0",
    ]
    exported: list[dict[str, Any]] = []
    for image_id, (sample, image_name) in enumerate(zip(samples, image_names), start=1):
        pose = colmap_pose_from_sample(sample, rig_values)
        qvec = pose["qvec"]
        tvec = pose["tvec"]
        lines.append(
            f"{image_id} {format_floats(qvec)} {format_floats(tvec)} 1 {image_name}"
        )
        lines.append("")
        exported.append(
            {
                "image_id": image_id,
                "sample_number": sample["sample_number"],
                "frame_index": sample["frame_index"],
                "video_time_s": sample["video_time_s"],
                "image_name": image_name,
                "center_world_m": pose["center_world_m"],
                "qvec": list(qvec),
                "tvec": list(tvec),
            }
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
    return exported


def write_points3d_txt(path: Path, points_payload: dict[str, Any] | None) -> int:
    lines = [
        "# 3D point list with one line of data per point:",
        "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[]",
    ]
    points = [] if points_payload is None else points_payload.get("points", [])
    lines.append(f"# Number of points: {len(points)}, mean track length: 0")
    for point_id, point in enumerate(points, start=1):
        x_m, y_m, z_m = [float(value) for value in point["point_m"]]
        red, green, blue = [int(value) for value in point.get("color_rgb", [200, 200, 200])]
        error = float(point.get("reprojection_error_px", 1.0))
        lines.append(
            f"{point_id} {x_m:.12g} {y_m:.12g} {z_m:.12g} "
            f"{red} {green} {blue} {error:.12g}"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
    return len(points)


def undistort_images(
    source_paths: list[Path],
    output_dir: Path,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    jpeg_quality: int,
) -> tuple[list[Path], np.ndarray]:
    output_dir.mkdir(parents=True, exist_ok=True)
    first = cv2.imread(str(source_paths[0]), cv2.IMREAD_COLOR)
    if first is None:
        raise RuntimeError(f"failed to read image: {source_paths[0]}")
    height, width = first.shape[:2]
    new_camera_matrix, _roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        distortion,
        (width, height),
        alpha=0.0,
        newImgSize=(width, height),
    )

    output_paths: list[Path] = []
    for source in source_paths:
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to read image: {source}")
        undistorted = cv2.undistort(image, camera_matrix, distortion, None, new_camera_matrix)
        output = output_dir / source.name
        ok = cv2.imwrite(
            str(output),
            undistorted,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
        )
        if not ok:
            raise RuntimeError(f"failed to write image: {output}")
        output_paths.append(output)
    return output_paths, new_camera_matrix


def copy_or_extract_images(
    session: Path,
    manifest: dict[str, Any],
    samples: list[dict[str, Any]],
    output_dir: Path,
    image_width: int,
    jpeg_quality: int,
    force_extract: bool,
    ffmpeg_path: Path | None,
    undistort: bool,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[list[Path], np.ndarray, np.ndarray]:
    if undistort:
        raw_dir = output_dir / "raw_images"
        image_dir = output_dir / "images"
        raw_paths = prepare_frames(
            session,
            manifest,
            samples,
            raw_dir,
            image_width,
            jpeg_quality,
            force_extract,
            ffmpeg_path,
        )
        image_paths, new_camera_matrix = undistort_images(
            raw_paths,
            image_dir,
            camera_matrix,
            distortion,
            jpeg_quality,
        )
        return image_paths, new_camera_matrix, np.zeros((0, 1), dtype=np.float64)

    image_dir = output_dir / "images"
    image_paths = prepare_frames(
        session,
        manifest,
        samples,
        image_dir,
        image_width,
        jpeg_quality,
        force_extract,
        ffmpeg_path,
    )
    return image_paths, camera_matrix, distortion


def read_image_size(path: Path) -> tuple[int, int]:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"failed to read image: {path}")
    height, width = image.shape[:2]
    return width, height


def non_comment_data_lines(path: Path) -> list[str]:
    return [
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("#")
    ]


def validate_export(
    output_dir: Path,
    image_paths: list[Path],
    exported_images: list[dict[str, Any]],
    point_count: int,
) -> dict[str, Any]:
    sparse_dir = output_dir / "sparse" / "0"
    camera_lines = non_comment_data_lines(sparse_dir / "cameras.txt")
    image_lines = non_comment_data_lines(sparse_dir / "images.txt")
    point_lines = non_comment_data_lines(sparse_dir / "points3D.txt")
    missing_images = [str(path) for path in image_paths if not path.exists() or path.stat().st_size == 0]
    quaternion_norms = [
        float(np.linalg.norm(np.array(image["qvec"], dtype=np.float64)))
        for image in exported_images
    ]
    return {
        "camera_records": len(camera_lines),
        "image_pose_records": len(image_lines),
        "point_records": len(point_lines),
        "expected_images": len(image_paths),
        "expected_points": point_count,
        "missing_images": missing_images,
        "min_quaternion_norm": min(quaternion_norms) if quaternion_norms else 0.0,
        "max_quaternion_norm": max(quaternion_norms) if quaternion_norms else 0.0,
        "passed": (
            len(camera_lines) == 1
            and len(image_lines) == len(image_paths)
            and len(point_lines) == point_count
            and not missing_images
            and all(abs(norm - 1.0) < 1e-6 for norm in quaternion_norms)
        ),
    }


def write_readme(output_dir: Path, manifest: dict[str, Any]) -> None:
    text = f"""# COLMAP-style export {manifest['export_id']}

This folder was generated from lidar-anchored camera poses. It is intended as a
handoff artifact for downstream photogrammetry or Gaussian-splatting tests.

Important caveats:

- This export did not run COLMAP feature matching or bundle adjustment.
- Camera poses come from the lidar ICP trajectory and rough camera-to-lidar rig
  measurements.
- Sparse points, when present, come from the project diagnostic triangulation,
  not from COLMAP.
- Treat this as an initialization/interop artifact, not as a final calibrated
  reconstruction.

Main paths:

- `images/`: exported camera frames.
- `sparse/0/cameras.txt`: COLMAP text camera model.
- `sparse/0/images.txt`: COLMAP text image poses.
- `sparse/0/points3D.txt`: optional sparse points.
- `export_manifest.json`: export metadata and validation summary.
"""
    (output_dir / "README.md").write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Export timestamped camera frames and lidar-derived poses as a COLMAP-style text model"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--pose-json", type=Path, required=True, help="Camera pose JSON")
    parser.add_argument("--intrinsics", type=Path, required=True, help="Camera intrinsics YAML")
    parser.add_argument("--output-dir", type=Path, required=True, help="Export folder")
    parser.add_argument("--points-json", type=Path, help="Optional sparse fused point JSON")
    parser.add_argument("--rig-config", type=Path, default=Path("config/rig_measurements.yaml"))
    parser.add_argument("--camera-height-m", type=float)
    parser.add_argument("--camera-roll-deg", type=float)
    parser.add_argument("--camera-pitch-deg", type=float)
    parser.add_argument("--camera-yaw-deg", type=float)
    parser.add_argument("--ffmpeg", type=Path, help="Optional ffmpeg executable path")
    parser.add_argument("--image-width", type=int, default=1920)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument(
        "--undistort-images",
        action="store_true",
        help="Write undistorted images and export a PINHOLE camera model",
    )
    parser.add_argument(
        "--camera-model",
        choices=["AUTO", "PINHOLE", "OPENCV", "FULL_OPENCV"],
        default="AUTO",
        help="COLMAP camera model. AUTO uses PINHOLE for undistorted images and FULL_OPENCV otherwise.",
    )
    args = parser.parse_args()

    if args.image_width <= 0:
        parser.error("--image-width must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")

    session = args.session.resolve()
    output_dir = args.output_dir.resolve()
    sparse_dir = output_dir / "sparse" / "0"
    sparse_dir.mkdir(parents=True, exist_ok=True)

    capture_manifest = load_json(session / "manifest.json")
    pose_payload = load_json(args.pose_json)
    samples = pose_payload["sampled_frames"]
    if not samples:
        raise ValueError("pose JSON contains no sampled frames")

    rig_values = load_rig_defaults(args.rig_config if args.rig_config.exists() else None)
    if args.camera_height_m is not None:
        rig_values["camera_height_m"] = args.camera_height_m
    if args.camera_roll_deg is not None:
        rig_values["camera_roll_deg"] = args.camera_roll_deg
    if args.camera_pitch_deg is not None:
        rig_values["camera_pitch_deg"] = args.camera_pitch_deg
    if args.camera_yaw_deg is not None:
        rig_values["camera_yaw_deg"] = args.camera_yaw_deg

    raw_camera_matrix, raw_distortion = load_intrinsics(args.intrinsics, args.image_width, round(args.image_width * 9 / 16))
    image_paths, export_camera_matrix, export_distortion = copy_or_extract_images(
        session,
        capture_manifest,
        samples,
        output_dir,
        args.image_width,
        args.jpeg_quality,
        args.force_extract,
        args.ffmpeg,
        args.undistort_images,
        raw_camera_matrix,
        raw_distortion,
    )
    width, height = read_image_size(image_paths[0])

    # Recompute intrinsics from the actual exported image dimensions in case the
    # codec produced a slightly different size than requested.
    raw_camera_matrix, raw_distortion = load_intrinsics(args.intrinsics, width, height)
    if args.undistort_images:
        # The images have already been undistorted. Re-run the same new-camera
        # matrix calculation at the actual size so the exported model is exact.
        export_camera_matrix, _roi = cv2.getOptimalNewCameraMatrix(
            raw_camera_matrix,
            raw_distortion,
            (width, height),
            alpha=0.0,
            newImgSize=(width, height),
        )
        export_distortion = np.zeros((0, 1), dtype=np.float64)
    else:
        export_camera_matrix = raw_camera_matrix
        export_distortion = raw_distortion

    camera_model = args.camera_model
    if camera_model == "AUTO":
        camera_model = "PINHOLE" if args.undistort_images else "FULL_OPENCV"
    if args.undistort_images and camera_model != "PINHOLE":
        raise ValueError("--undistort-images currently exports only PINHOLE camera models")

    camera_params = write_cameras_txt(
        sparse_dir / "cameras.txt",
        camera_model,
        width,
        height,
        export_camera_matrix,
        export_distortion,
    )
    image_names = [path.name for path in image_paths]
    exported_images = write_images_txt(
        sparse_dir / "images.txt",
        samples,
        image_names,
        rig_values,
    )
    points_payload = load_json(args.points_json) if args.points_json else None
    point_count = write_points3d_txt(sparse_dir / "points3D.txt", points_payload)
    validation = validate_export(output_dir, image_paths, exported_images, point_count)

    export_manifest = {
        "schema_version": 1,
        "export_id": output_dir.name,
        "session_id": capture_manifest["session_id"],
        "source_session": str(session),
        "source_pose_json": str(args.pose_json),
        "source_intrinsics": str(args.intrinsics),
        "source_points_json": str(args.points_json) if args.points_json else None,
        "format": "COLMAP text model",
        "coordinate_frame": "lidar ICP map x/y in meters; z is meters above lidar scan plane",
        "caveat": "Generated from lidar-derived poses and rough rig extrinsics; no COLMAP bundle adjustment has been run.",
        "image_count": len(image_paths),
        "point_count": point_count,
        "image_width": width,
        "image_height": height,
        "undistorted_images": args.undistort_images,
        "camera_model": camera_model,
        "camera_params": camera_params,
        "rig_values": rig_values,
        "images": exported_images,
        "paths": {
            "images": "images",
            "sparse_model": "sparse/0",
            "cameras": "sparse/0/cameras.txt",
            "images_txt": "sparse/0/images.txt",
            "points3D": "sparse/0/points3D.txt",
        },
        "validation": validation,
    }
    (output_dir / "export_manifest.json").write_text(
        json.dumps(export_manifest, indent=2) + "\n",
        encoding="utf-8",
    )
    write_readme(output_dir, export_manifest)

    print(f"Wrote export: {output_dir}")
    print(f"Images: {len(image_paths)} at {width}x{height}")
    print(f"Camera model: {camera_model}")
    print(f"Sparse points: {point_count}")
    print(f"Validation passed: {validation['passed']}")
    if validation["missing_images"]:
        print(f"Missing images: {validation['missing_images']}")
    return 0 if validation["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
