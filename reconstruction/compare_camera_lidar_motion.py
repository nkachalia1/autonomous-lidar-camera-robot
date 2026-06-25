#!/usr/bin/env python3
"""Compare monocular camera motion against the lidar ICP trajectory.

This is a validation/diagnostic tool, not a final visual-SLAM system.  A single
monocular camera cannot recover metric scale by itself, so the camera trajectory
is estimated up to arbitrary scale and then similarity-aligned to the lidar ICP
poses for visualization and residual checks.
"""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from render_camera_pose_contact_sheet import extract_frame, find_ffmpeg


Pose2 = tuple[float, float, float]


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def load_intrinsics(path: Path, image_width: int, image_height: int) -> tuple[np.ndarray, np.ndarray]:
    with path.open(encoding="utf-8") as stream:
        payload = yaml.safe_load(stream)

    original = payload["calibration_image_size"]
    original_width = float(original["width"])
    original_height = float(original["height"])
    scale_x = image_width / original_width
    scale_y = image_height / original_height

    matrix = np.array(payload["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    matrix[0, 0] *= scale_x
    matrix[0, 2] *= scale_x
    matrix[1, 1] *= scale_y
    matrix[1, 2] *= scale_y

    distortion = np.array(payload["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1, 1)
    return matrix, distortion


def frame_path_for_sample(frame_dir: Path, sample: dict[str, Any]) -> Path:
    sample_number = int(sample["sample_number"])
    frame_index = int(sample["frame_index"])
    return frame_dir / f"sample-{sample_number:02d}-frame-{frame_index:04d}.jpg"


def prepare_frames(
    session: Path,
    manifest: dict[str, Any],
    samples: list[dict[str, Any]],
    frame_dir: Path,
    thumbnail_width: int,
    jpeg_quality: int,
    force_extract: bool,
    ffmpeg_path: Path | None,
) -> list[Path]:
    frame_dir.mkdir(parents=True, exist_ok=True)
    video = session / manifest["camera"]["video"]
    ffmpeg = find_ffmpeg(ffmpeg_path)
    paths: list[Path] = []
    for sample in samples:
        output = frame_path_for_sample(frame_dir, sample)
        extract_frame(
            ffmpeg,
            video,
            int(sample["frame_index"]),
            output,
            thumbnail_width,
            jpeg_quality,
            force_extract,
        )
        paths.append(output)
    return paths


def load_gray(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"failed to read image: {path}")
    return image


def lidar_points_from_samples(samples: list[dict[str, Any]]) -> np.ndarray:
    return np.array(
        [
            [
                float(sample["camera_pose"]["x_m"]),
                float(sample["camera_pose"]["y_m"]),
            ]
            for sample in samples
        ],
        dtype=np.float64,
    )


def detect_features(image: np.ndarray, max_features: int, fast_threshold: int) -> tuple[list[Any], Any]:
    orb = cv2.ORB_create(nfeatures=max_features, fastThreshold=fast_threshold)
    return orb.detectAndCompute(image, None)


def match_descriptors(first: Any, second: Any, ratio: float) -> list[Any]:
    if first is None or second is None:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = matcher.knnMatch(first, second, k=2)
    matches = []
    for pair in pairs:
        if len(pair) != 2:
            continue
        match, neighbor = pair
        if match.distance < ratio * neighbor.distance:
            matches.append(match)
    return matches


def estimate_pair_motion(
    first_image: np.ndarray,
    second_image: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    max_features: int,
    fast_threshold: int,
    ratio: float,
    ransac_threshold_px: float,
    min_matches: int,
    min_pose_inliers: int,
) -> dict[str, Any]:
    first_keypoints, first_descriptors = detect_features(first_image, max_features, fast_threshold)
    second_keypoints, second_descriptors = detect_features(second_image, max_features, fast_threshold)
    matches = match_descriptors(first_descriptors, second_descriptors, ratio)

    result: dict[str, Any] = {
        "first_keypoints": len(first_keypoints),
        "second_keypoints": len(second_keypoints),
        "matches": len(matches),
        "essential_inliers": 0,
        "pose_inliers": 0,
        "success": False,
        "reason": "",
    }
    if len(matches) < min_matches:
        result["reason"] = f"not enough matches: {len(matches)} < {min_matches}"
        return result

    first_points = np.float32([first_keypoints[match.queryIdx].pt for match in matches]).reshape(-1, 1, 2)
    second_points = np.float32([second_keypoints[match.trainIdx].pt for match in matches]).reshape(-1, 1, 2)
    undistorted_first = cv2.undistortPoints(first_points, camera_matrix, distortion, P=camera_matrix)
    undistorted_second = cv2.undistortPoints(second_points, camera_matrix, distortion, P=camera_matrix)

    essential, essential_mask = cv2.findEssentialMat(
        undistorted_first,
        undistorted_second,
        camera_matrix,
        method=cv2.RANSAC,
        prob=0.999,
        threshold=ransac_threshold_px,
    )
    if essential is None or essential_mask is None:
        result["reason"] = "essential matrix failed"
        return result
    if essential.shape[0] > 3:
        essential = essential[:3, :]

    result["essential_inliers"] = int(np.count_nonzero(essential_mask))
    pose_inliers, rotation, translation, pose_mask = cv2.recoverPose(
        essential,
        undistorted_first,
        undistorted_second,
        camera_matrix,
        mask=essential_mask,
    )
    result["pose_inliers"] = int(pose_inliers)
    if pose_inliers < min_pose_inliers:
        result["reason"] = f"not enough pose inliers: {pose_inliers} < {min_pose_inliers}"
        return result

    # recoverPose returns the transform from camera i to camera i+1.  The
    # displacement of camera i+1 in camera-i coordinates is -R.T @ t.  Scale is
    # arbitrary for monocular geometry.
    camera_step = -rotation.T @ translation.reshape(3)
    result.update(
        {
            "success": True,
            "reason": "ok",
            "rotation": rotation.tolist(),
            "translation_unit": translation.reshape(3).tolist(),
            "camera_step_unit_in_previous_camera": camera_step.tolist(),
            "pose_mask_inliers": int(np.count_nonzero(pose_mask)) if pose_mask is not None else int(pose_inliers),
        }
    )
    return result


def build_camera_vo_path(pair_results: list[dict[str, Any]]) -> np.ndarray:
    rotation_world_from_camera = np.eye(3, dtype=np.float64)
    position = np.zeros(3, dtype=np.float64)
    positions = [position.copy()]
    for result in pair_results:
        if result["success"]:
            rotation = np.array(result["rotation"], dtype=np.float64)
            step_previous_camera = np.array(
                result["camera_step_unit_in_previous_camera"],
                dtype=np.float64,
            )
            position = position + rotation_world_from_camera @ step_previous_camera
            rotation_world_from_camera = rotation_world_from_camera @ rotation.T
        positions.append(position.copy())
    return np.vstack(positions)


def camera_path_candidates(camera_positions_3d: np.ndarray) -> list[tuple[str, np.ndarray]]:
    # OpenCV camera coordinates are x right, y down, z forward.  We compare
    # horizontal x/z motion against the lidar top-down map.  The lateral sign is
    # ambiguous until extrinsics are fully calibrated, so test both signs and
    # keep the lower-residual one.
    x = camera_positions_3d[:, 0]
    z = camera_positions_3d[:, 2]
    return [
        ("z_forward_x_right", np.column_stack([z, x])),
        ("z_forward_x_left", np.column_stack([z, -x])),
    ]


def similarity_align(source: np.ndarray, target: np.ndarray) -> dict[str, Any]:
    if len(source) < 2:
        raise ValueError("at least two points are required for similarity alignment")
    source_complex = source[:, 0] + 1j * source[:, 1]
    target_complex = target[:, 0] + 1j * target[:, 1]
    source_center = source_complex.mean()
    target_center = target_complex.mean()
    denominator = np.sum(np.abs(source_complex - source_center) ** 2)
    if denominator < 1e-12:
        raise ValueError("source points have near-zero spread")
    transform = (
        np.sum(np.conj(source_complex - source_center) * (target_complex - target_center))
        / denominator
    )
    translation = target_center - transform * source_center
    aligned_complex = transform * source_complex + translation
    aligned = np.column_stack([aligned_complex.real, aligned_complex.imag])
    residuals = np.linalg.norm(aligned - target, axis=1)
    return {
        "scale": float(abs(transform)),
        "rotation_deg": float(math.degrees(math.atan2(transform.imag, transform.real))),
        "translation": [float(translation.real), float(translation.imag)],
        "aligned": aligned,
        "rmse_m": float(math.sqrt(np.mean(residuals**2))),
        "max_error_m": float(np.max(residuals)),
        "residuals_m": residuals,
    }


def angle_between(first: np.ndarray, second: np.ndarray) -> float | None:
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if first_norm < 1e-9 or second_norm < 1e-9:
        return None
    dot = float(np.dot(first, second) / (first_norm * second_norm))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def choose_alignment(
    camera_positions_3d: np.ndarray,
    lidar_points: np.ndarray,
    pair_results: list[dict[str, Any]],
    min_lidar_step_m: float,
) -> dict[str, Any]:
    valid_pair_indices = [
        index
        for index, result in enumerate(pair_results)
        if result["success"]
        and np.linalg.norm(lidar_points[index + 1] - lidar_points[index]) >= min_lidar_step_m
    ]
    if not valid_pair_indices:
        raise ValueError("no successful moving frame pairs available for alignment")
    sample_indices = sorted(set(valid_pair_indices + [index + 1 for index in valid_pair_indices]))
    if len(sample_indices) < 2:
        raise ValueError("not enough moving samples available for alignment")

    best: dict[str, Any] | None = None
    for name, candidate in camera_path_candidates(camera_positions_3d):
        # Keep the moving-window transform but apply it to every point.
        source_complex = candidate[:, 0] + 1j * candidate[:, 1]
        moving_source = candidate[sample_indices]
        moving_target = lidar_points[sample_indices]
        moving_fit = similarity_align(moving_source, moving_target)
        transform_scale = moving_fit["scale"]
        transform_angle = math.radians(moving_fit["rotation_deg"])
        transform = transform_scale * complex(math.cos(transform_angle), math.sin(transform_angle))
        translation = complex(*moving_fit["translation"])
        aligned_complex = transform * source_complex + translation
        aligned_all = np.column_stack([aligned_complex.real, aligned_complex.imag])
        residuals_all = np.linalg.norm(aligned_all - lidar_points, axis=1)
        candidate_result = {
            "candidate_name": name,
            "sample_indices": sample_indices,
            "scale": moving_fit["scale"],
            "rotation_deg": moving_fit["rotation_deg"],
            "translation": moving_fit["translation"],
            "aligned_all": aligned_all,
            "moving_rmse_m": moving_fit["rmse_m"],
            "moving_max_error_m": moving_fit["max_error_m"],
            "all_rmse_m": float(math.sqrt(np.mean(residuals_all**2))),
            "all_max_error_m": float(np.max(residuals_all)),
            "residuals_all_m": residuals_all,
        }
        if best is None or candidate_result["moving_rmse_m"] < best["moving_rmse_m"]:
            best = candidate_result
    assert best is not None
    return best


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(points[1:] - points[:-1], axis=1)))


def render_svg(
    output: Path,
    session_id: str,
    lidar_points: np.ndarray,
    aligned_camera_points: np.ndarray,
    samples: list[dict[str, Any]],
    pair_summaries: list[dict[str, Any]],
    summary: dict[str, Any],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    all_points = np.vstack([lidar_points, aligned_camera_points])
    min_x, min_y = np.min(all_points, axis=0) - 0.15
    max_x, max_y = np.max(all_points, axis=0) + 0.15
    width, height = 1600, 1120
    left, right, top, bottom = 88, 1528, 138, 640
    plot_width = right - left
    plot_height = bottom - top
    scale = min(plot_width / max(max_x - min_x, 1e-6), plot_height / max(max_y - min_y, 1e-6))

    def px(point: np.ndarray) -> float:
        return left + (float(point[0]) - min_x) * scale

    def py(point: np.ndarray) -> float:
        return top + (max_y - float(point[1])) * scale

    def path_data(points: np.ndarray) -> str:
        return " ".join(
            f'{"M" if index == 0 else "L"} {px(point):.1f} {py(point):.1f}'
            for index, point in enumerate(points)
        )

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        (
            f'<text x="48" y="52" fill="#f8fafc" font-size="30" font-weight="700">'
            f'Session {html.escape(session_id)} camera-vs-lidar motion</text>'
        ),
        (
            f'<text x="48" y="84" fill="#94a3b8" font-size="16">'
            'Camera monocular VO is arbitrary-scale and similarity-aligned to lidar ICP for comparison.</text>'
        ),
        (
            f'<text x="48" y="110" fill="#fbbf24" font-size="14">'
            'Use this as a feature/motion sanity check, not as metric camera odometry.</text>'
        ),
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" rx="18" fill="#0d1726" stroke="#334155"/>',
    ]

    for x_m in range(math.floor(min_x), math.ceil(max_x) + 1):
        x = left + (x_m - min_x) * scale
        svg.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{bottom}" stroke="#223246"/>')
        svg.append(f'<text x="{x + 4:.1f}" y="{bottom + 23}" fill="#64748b" font-size="12">{x_m}m</text>')
    for y_m in range(math.floor(min_y), math.ceil(max_y) + 1):
        y = top + (max_y - y_m) * scale
        svg.append(f'<line x1="{left}" y1="{y:.1f}" x2="{right}" y2="{y:.1f}" stroke="#223246"/>')
        svg.append(f'<text x="30" y="{y + 4:.1f}" fill="#64748b" font-size="12">{y_m}m</text>')

    svg.extend(
        [
            f'<path d="{path_data(lidar_points)}" fill="none" stroke="#38bdf8" stroke-width="5" stroke-linejoin="round" stroke-linecap="round"/>',
            f'<path d="{path_data(aligned_camera_points)}" fill="none" stroke="#f97316" stroke-width="4" stroke-dasharray="10 8" stroke-linejoin="round" stroke-linecap="round"/>',
        ]
    )

    for index, (lidar_point, camera_point) in enumerate(zip(lidar_points, aligned_camera_points)):
        sample_number = int(samples[index]["sample_number"])
        svg.append(
            f'<line x1="{px(lidar_point):.1f}" y1="{py(lidar_point):.1f}" '
            f'x2="{px(camera_point):.1f}" y2="{py(camera_point):.1f}" '
            'stroke="#64748b" stroke-width="1" stroke-opacity="0.55"/>'
        )
        svg.append(
            f'<circle cx="{px(lidar_point):.1f}" cy="{py(lidar_point):.1f}" r="9" fill="#38bdf8" stroke="#020617" stroke-width="2"/>'
        )
        svg.append(
            f'<circle cx="{px(camera_point):.1f}" cy="{py(camera_point):.1f}" r="7" fill="#f97316" stroke="#020617" stroke-width="2"/>'
        )
        svg.append(
            f'<text x="{px(lidar_point) + 12:.1f}" y="{py(lidar_point) - 10:.1f}" fill="#e2e8f0" font-size="12" font-weight="700">{sample_number}</text>'
        )

    summary_y = 696
    svg.extend(
        [
            f'<rect x="48" y="{summary_y - 34}" width="1504" height="112" rx="16" fill="#0d1726" stroke="#334155"/>',
            f'<circle cx="78" cy="{summary_y - 6}" r="7" fill="#38bdf8"/>',
            f'<text x="94" y="{summary_y}" fill="#cbd5e1" font-size="15">lidar ICP camera poses</text>',
            f'<circle cx="288" cy="{summary_y - 6}" r="7" fill="#f97316"/>',
            f'<text x="304" y="{summary_y}" fill="#cbd5e1" font-size="15">aligned monocular camera VO</text>',
            (
                f'<text x="70" y="{summary_y + 34}" fill="#f8fafc" font-size="17" font-weight="700">'
                f'Successful pairs: {summary["successful_pairs"]}/{summary["pair_count"]}; '
                f'median pose inliers: {summary["median_pose_inliers"]:.0f}; '
                f'moving RMSE: {summary["moving_rmse_m"]:.3f} m; '
                f'median direction error: {summary["median_direction_error_deg"]:.1f} deg</text>'
            ),
            (
                f'<text x="70" y="{summary_y + 62}" fill="#94a3b8" font-size="14">'
                f'Camera candidate: {html.escape(summary["camera_candidate"])}; '
                f'alignment scale: {summary["alignment_scale"]:.3f}; '
                f'alignment rotation: {summary["alignment_rotation_deg"]:+.1f} deg.</text>'
            ),
        ]
    )

    table_y = 850
    headers = ["pair", "frames", "matches", "pose inliers", "lidar step", "direction error", "status"]
    column_x = [52, 160, 335, 470, 640, 790, 990]
    svg.append(f'<text x="48" y="{table_y - 32}" fill="#f8fafc" font-size="22" font-weight="700">Pair diagnostics</text>')
    for x, header in zip(column_x, headers):
        svg.append(f'<text x="{x}" y="{table_y}" fill="#94a3b8" font-size="13" font-weight="700">{header}</text>')
    for row, pair in enumerate(pair_summaries[:11], start=1):
        y = table_y + row * 22
        color = "#cbd5e1" if pair["success"] else "#fb7185"
        values = [
            f'{pair["pair_index"]}',
            f'{pair["first_frame"]}->{pair["second_frame"]}',
            f'{pair["matches"]}',
            f'{pair["pose_inliers"]}',
            f'{pair["lidar_step_m"]:.3f} m',
            "n/a" if pair["direction_error_deg"] is None else f'{pair["direction_error_deg"]:.1f} deg',
            pair["reason"],
        ]
        for x, value in zip(column_x, values):
            svg.append(f'<text x="{x}" y="{y}" fill="{color}" font-size="13">{html.escape(str(value))}</text>')

    svg.append("</svg>")
    output.write_text("\n".join(svg), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare camera-only visual motion with lidar ICP motion"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--pose-json", type=Path, required=True, help="Camera pose JSON from render_camera_pose_timeline.py")
    parser.add_argument("--intrinsics", type=Path, required=True, help="Camera intrinsics YAML")
    parser.add_argument("--output", type=Path, required=True, help="Output SVG comparison path")
    parser.add_argument("--json-output", type=Path, help="Optional machine-readable results JSON")
    parser.add_argument("--frame-dir", type=Path, help="Directory for sampled frame JPEGs")
    parser.add_argument("--ffmpeg", type=Path, help="Optional ffmpeg executable path")
    parser.add_argument("--thumbnail-width", type=int, default=640)
    parser.add_argument("--jpeg-quality", type=int, default=4)
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument("--max-features", type=int, default=2500)
    parser.add_argument("--fast-threshold", type=int, default=7)
    parser.add_argument("--ratio", type=float, default=0.75)
    parser.add_argument("--ransac-threshold-px", type=float, default=1.0)
    parser.add_argument("--min-matches", type=int, default=80)
    parser.add_argument("--min-pose-inliers", type=int, default=50)
    parser.add_argument("--min-lidar-step-m", type=float, default=0.025)
    args = parser.parse_args()

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    pose_payload = load_json(args.pose_json)
    samples = pose_payload["sampled_frames"]
    if len(samples) < 2:
        raise ValueError("at least two sampled frames are required")

    frame_dir = args.frame_dir
    if frame_dir is None:
        frame_dir = args.output.parent / f"{manifest['session_id']}-camera-samples"
    frame_paths = prepare_frames(
        session,
        manifest,
        samples,
        frame_dir,
        args.thumbnail_width,
        args.jpeg_quality,
        args.force_extract,
        args.ffmpeg,
    )
    images = [load_gray(path) for path in frame_paths]
    image_height, image_width = images[0].shape[:2]
    camera_matrix, distortion = load_intrinsics(args.intrinsics, image_width, image_height)
    lidar_points = lidar_points_from_samples(samples)

    pair_results: list[dict[str, Any]] = []
    for index, (first, second) in enumerate(zip(images, images[1:])):
        result = estimate_pair_motion(
            first,
            second,
            camera_matrix,
            distortion,
            args.max_features,
            args.fast_threshold,
            args.ratio,
            args.ransac_threshold_px,
            args.min_matches,
            args.min_pose_inliers,
        )
        result["pair_index"] = index + 1
        result["first_sample"] = int(samples[index]["sample_number"])
        result["second_sample"] = int(samples[index + 1]["sample_number"])
        result["first_frame"] = int(samples[index]["frame_index"])
        result["second_frame"] = int(samples[index + 1]["frame_index"])
        result["lidar_step_m"] = float(np.linalg.norm(lidar_points[index + 1] - lidar_points[index]))
        pair_results.append(result)

    camera_positions_3d = build_camera_vo_path(pair_results)
    alignment = choose_alignment(
        camera_positions_3d,
        lidar_points,
        pair_results,
        args.min_lidar_step_m,
    )
    aligned_camera_points = alignment["aligned_all"]

    direction_errors: list[float] = []
    pair_summaries: list[dict[str, Any]] = []
    for index, result in enumerate(pair_results):
        lidar_step = lidar_points[index + 1] - lidar_points[index]
        camera_step = aligned_camera_points[index + 1] - aligned_camera_points[index]
        direction_error = angle_between(camera_step, lidar_step)
        if (
            direction_error is not None
            and result["success"]
            and float(result["lidar_step_m"]) >= args.min_lidar_step_m
        ):
            direction_errors.append(direction_error)
        pair_summaries.append(
            {
                "pair_index": result["pair_index"],
                "first_frame": result["first_frame"],
                "second_frame": result["second_frame"],
                "first_keypoints": result["first_keypoints"],
                "second_keypoints": result["second_keypoints"],
                "matches": result["matches"],
                "essential_inliers": result["essential_inliers"],
                "pose_inliers": result["pose_inliers"],
                "success": result["success"],
                "reason": result["reason"],
                "lidar_step_m": result["lidar_step_m"],
                "direction_error_deg": direction_error,
            }
        )

    successful_pairs = [result for result in pair_results if result["success"]]
    pose_inliers = [float(result["pose_inliers"]) for result in successful_pairs]
    summary = {
        "session_id": manifest["session_id"],
        "sample_count": len(samples),
        "pair_count": len(pair_results),
        "successful_pairs": len(successful_pairs),
        "median_pose_inliers": float(np.median(pose_inliers)) if pose_inliers else 0.0,
        "median_direction_error_deg": float(np.median(direction_errors)) if direction_errors else float("nan"),
        "max_direction_error_deg": float(np.max(direction_errors)) if direction_errors else float("nan"),
        "camera_candidate": alignment["candidate_name"],
        "alignment_scale": alignment["scale"],
        "alignment_rotation_deg": alignment["rotation_deg"],
        "moving_rmse_m": alignment["moving_rmse_m"],
        "moving_max_error_m": alignment["moving_max_error_m"],
        "all_rmse_m": alignment["all_rmse_m"],
        "all_max_error_m": alignment["all_max_error_m"],
        "lidar_sample_path_length_m": path_length(lidar_points),
        "aligned_camera_sample_path_length_m": path_length(aligned_camera_points),
        "moving_alignment_sample_indices": alignment["sample_indices"],
    }

    render_svg(
        args.output,
        str(manifest["session_id"]),
        lidar_points,
        aligned_camera_points,
        samples,
        pair_summaries,
        summary,
    )

    payload = {
        "summary": summary,
        "samples": samples,
        "pair_summaries": pair_summaries,
        "lidar_points_m": lidar_points.tolist(),
        "camera_positions_unit_3d": camera_positions_3d.tolist(),
        "aligned_camera_points_m": aligned_camera_points.tolist(),
    }
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {args.output}")
    if args.json_output:
        print(f"Wrote {args.json_output}")
    print(f"Successful visual-motion pairs: {summary['successful_pairs']}/{summary['pair_count']}")
    print(f"Median pose inliers: {summary['median_pose_inliers']:.0f}")
    print(f"Moving alignment RMSE: {summary['moving_rmse_m']:.3f} m")
    print(f"Median direction error: {summary['median_direction_error_deg']:.1f} deg")
    print(f"Camera candidate: {summary['camera_candidate']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
