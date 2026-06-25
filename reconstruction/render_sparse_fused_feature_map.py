#!/usr/bin/env python3
"""Render a sparse lidar-anchored camera feature triangulation diagnostic.

The output is intentionally called a diagnostic, not a reconstruction.  It uses
the lidar ICP camera poses as metric camera centers, matches visual features
between neighboring camera frames, triangulates those matches with the rough
camera model, and renders a top-down/side-view sanity check.
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

from compare_camera_lidar_motion import (
    detect_features,
    load_intrinsics,
    match_descriptors,
    prepare_frames,
)


COLORS = [
    (56, 189, 248),
    (249, 115, 22),
    (167, 139, 250),
    (52, 211, 153),
    (251, 191, 36),
    (244, 114, 182),
    (34, 211, 238),
    (248, 113, 113),
    (190, 242, 100),
    (196, 181, 253),
]


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def load_rig_defaults(path: Path | None) -> dict[str, float]:
    defaults = {
        "camera_height_m": 0.0953,
        "camera_roll_deg": 0.0,
        "camera_pitch_deg": 0.0,
        "camera_yaw_deg": 0.0,
    }
    if path is None:
        return defaults
    with path.open(encoding="utf-8") as stream:
        payload = yaml.safe_load(stream)
    lidar_to_camera = payload.get("lidar_to_camera", {})
    if lidar_to_camera.get("camera_up_m") is not None:
        defaults["camera_height_m"] = float(lidar_to_camera["camera_up_m"])
    if lidar_to_camera.get("camera_roll_deg") is not None:
        defaults["camera_roll_deg"] = float(lidar_to_camera["camera_roll_deg"])
    if lidar_to_camera.get("camera_pitch_deg") is not None:
        defaults["camera_pitch_deg"] = float(lidar_to_camera["camera_pitch_deg"])
    if lidar_to_camera.get("camera_yaw_deg") is not None:
        defaults["camera_yaw_deg"] = float(lidar_to_camera["camera_yaw_deg"])
    return defaults


def rotation_x(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=np.float64,
    )


def rotation_y(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=np.float64,
    )


def rotation_z(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def camera_rotation_world_from_camera(
    rig_yaw_rad: float,
    camera_roll_deg: float,
    camera_pitch_deg: float,
    camera_yaw_deg: float,
) -> np.ndarray:
    """Return a camera-to-world rotation matrix.

    World frame: +x/+y are the lidar ICP map plane, +z is up.
    OpenCV camera frame: +x right, +y down, +z forward.
    """

    right = np.array([math.sin(rig_yaw_rad), -math.cos(rig_yaw_rad), 0.0])
    down = np.array([0.0, 0.0, -1.0])
    forward = np.array([math.cos(rig_yaw_rad), math.sin(rig_yaw_rad), 0.0])
    base = np.column_stack([right, down, forward])

    # Local camera-frame refinements.  The rig values are rough, so these should
    # be treated as diagnostics rather than final calibrated extrinsics.
    local = (
        rotation_z(math.radians(camera_roll_deg))
        @ rotation_x(math.radians(camera_pitch_deg))
        @ rotation_y(math.radians(camera_yaw_deg))
    )
    return base @ local


def camera_center_world(sample: dict[str, Any], camera_height_m: float) -> np.ndarray:
    pose = sample["camera_pose"]
    return np.array(
        [
            float(pose["x_m"]),
            float(pose["y_m"]),
            camera_height_m,
        ],
        dtype=np.float64,
    )


def projection_matrix(
    camera_matrix: np.ndarray,
    center_world: np.ndarray,
    rotation_world_from_camera: np.ndarray,
) -> np.ndarray:
    rotation_camera_from_world = rotation_world_from_camera.T
    translation = -rotation_camera_from_world @ center_world
    return camera_matrix @ np.column_stack([rotation_camera_from_world, translation])


def project_points(projection: np.ndarray, points_world: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack([points_world, np.ones(len(points_world), dtype=np.float64)])
    projected = (projection @ homogeneous.T).T
    return projected[:, :2] / projected[:, 2:3]


def camera_depths(
    points_world: np.ndarray,
    center_world: np.ndarray,
    rotation_world_from_camera: np.ndarray,
) -> np.ndarray:
    rotation_camera_from_world = rotation_world_from_camera.T
    camera_points = (rotation_camera_from_world @ (points_world - center_world).T).T
    return camera_points[:, 2]


def triangulation_angles_deg(points_world: np.ndarray, first_center: np.ndarray, second_center: np.ndarray) -> np.ndarray:
    first_vectors = points_world - first_center
    second_vectors = points_world - second_center
    first_vectors /= np.linalg.norm(first_vectors, axis=1, keepdims=True)
    second_vectors /= np.linalg.norm(second_vectors, axis=1, keepdims=True)
    dots = np.sum(first_vectors * second_vectors, axis=1)
    dots = np.clip(dots, -1.0, 1.0)
    return np.degrees(np.arccos(dots))


def undistorted_matched_points(
    keypoints_first: list[Any],
    keypoints_second: list[Any],
    matches: list[Any],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    first = np.float32([keypoints_first[match.queryIdx].pt for match in matches]).reshape(-1, 1, 2)
    second = np.float32([keypoints_second[match.trainIdx].pt for match in matches]).reshape(-1, 1, 2)
    first_undistorted = cv2.undistortPoints(first, camera_matrix, distortion, P=camera_matrix)
    second_undistorted = cv2.undistortPoints(second, camera_matrix, distortion, P=camera_matrix)
    return first_undistorted.reshape(-1, 2), second_undistorted.reshape(-1, 2)


def essential_inlier_mask(
    points_first: np.ndarray,
    points_second: np.ndarray,
    camera_matrix: np.ndarray,
    ransac_threshold_px: float,
) -> tuple[np.ndarray, int, int]:
    if len(points_first) < 8:
        return np.zeros(len(points_first), dtype=bool), 0, 0
    essential, mask = cv2.findEssentialMat(
        points_first.reshape(-1, 1, 2),
        points_second.reshape(-1, 1, 2),
        camera_matrix,
        method=cv2.RANSAC,
        prob=0.999,
        threshold=ransac_threshold_px,
    )
    if essential is None or mask is None:
        return np.zeros(len(points_first), dtype=bool), 0, 0
    if essential.shape[0] > 3:
        essential = essential[:3, :]
    pose_inliers, _rotation, _translation, pose_mask = cv2.recoverPose(
        essential,
        points_first.reshape(-1, 1, 2),
        points_second.reshape(-1, 1, 2),
        camera_matrix,
        mask=mask,
    )
    if pose_mask is None:
        inliers = mask.reshape(-1).astype(bool)
    else:
        inliers = pose_mask.reshape(-1).astype(bool)
    return inliers, int(np.count_nonzero(mask)), int(pose_inliers)


def triangulate_pair(
    first_image: np.ndarray,
    second_image: np.ndarray,
    first_sample: dict[str, Any],
    second_sample: dict[str, Any],
    first_center: np.ndarray,
    second_center: np.ndarray,
    first_rotation: np.ndarray,
    second_rotation: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    max_features: int,
    fast_threshold: int,
    ratio: float,
    ransac_threshold_px: float,
    max_reprojection_error_px: float,
    min_depth_m: float,
    max_range_m: float,
    min_triangulation_angle_deg: float,
    min_pose_inliers: int,
    min_lidar_step_m: float,
    pair_index: int,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    first_keypoints, first_descriptors = detect_features(first_image, max_features, fast_threshold)
    second_keypoints, second_descriptors = detect_features(second_image, max_features, fast_threshold)
    matches = match_descriptors(first_descriptors, second_descriptors, ratio)
    lidar_step_m = float(np.linalg.norm(second_center[:2] - first_center[:2]))

    summary: dict[str, Any] = {
        "pair_index": pair_index,
        "first_sample": int(first_sample["sample_number"]),
        "second_sample": int(second_sample["sample_number"]),
        "first_frame": int(first_sample["frame_index"]),
        "second_frame": int(second_sample["frame_index"]),
        "first_keypoints": len(first_keypoints),
        "second_keypoints": len(second_keypoints),
        "matches": len(matches),
        "essential_inliers": 0,
        "pose_inliers": 0,
        "triangulated_points": 0,
        "accepted_points": 0,
        "lidar_step_m": lidar_step_m,
        "reason": "ok",
    }
    if lidar_step_m < min_lidar_step_m:
        summary["reason"] = f"lidar step {lidar_step_m:.3f} m below minimum"
        return [], summary
    if len(matches) < 8:
        summary["reason"] = "not enough matches"
        return [], summary

    points_first, points_second = undistorted_matched_points(
        first_keypoints,
        second_keypoints,
        matches,
        camera_matrix,
        distortion,
    )
    inlier_mask, essential_inliers, pose_inliers = essential_inlier_mask(
        points_first,
        points_second,
        camera_matrix,
        ransac_threshold_px,
    )
    summary["essential_inliers"] = essential_inliers
    summary["pose_inliers"] = pose_inliers
    if pose_inliers < min_pose_inliers:
        summary["reason"] = f"pose inliers {pose_inliers} below minimum"
        return [], summary

    points_first = points_first[inlier_mask]
    points_second = points_second[inlier_mask]
    if len(points_first) < 2:
        summary["reason"] = "not enough inlier points"
        return [], summary

    first_projection = projection_matrix(camera_matrix, first_center, first_rotation)
    second_projection = projection_matrix(camera_matrix, second_center, second_rotation)
    homogeneous = cv2.triangulatePoints(
        first_projection,
        second_projection,
        points_first.T,
        points_second.T,
    ).T
    valid_w = np.abs(homogeneous[:, 3]) > 1e-9
    points_world = homogeneous[valid_w, :3] / homogeneous[valid_w, 3:4]
    points_first = points_first[valid_w]
    points_second = points_second[valid_w]
    summary["triangulated_points"] = len(points_world)
    if len(points_world) == 0:
        summary["reason"] = "triangulation produced no finite points"
        return [], summary

    reproj_first = project_points(first_projection, points_world)
    reproj_second = project_points(second_projection, points_world)
    error_first = np.linalg.norm(reproj_first - points_first, axis=1)
    error_second = np.linalg.norm(reproj_second - points_second, axis=1)
    reproj_error = np.maximum(error_first, error_second)
    depth_first = camera_depths(points_world, first_center, first_rotation)
    depth_second = camera_depths(points_world, second_center, second_rotation)
    ranges = np.linalg.norm(points_world[:, :2] - first_center[:2], axis=1)
    angles = triangulation_angles_deg(points_world, first_center, second_center)

    accept = (
        (depth_first > min_depth_m)
        & (depth_second > min_depth_m)
        & (ranges <= max_range_m)
        & (reproj_error <= max_reprojection_error_px)
        & (angles >= min_triangulation_angle_deg)
        & np.isfinite(points_world).all(axis=1)
    )

    accepted: list[dict[str, Any]] = []
    color = COLORS[(pair_index - 1) % len(COLORS)]
    for point, error, angle, range_m in zip(
        points_world[accept],
        reproj_error[accept],
        angles[accept],
        ranges[accept],
    ):
        accepted.append(
            {
                "pair_index": pair_index,
                "point_m": [float(point[0]), float(point[1]), float(point[2])],
                "reprojection_error_px": float(error),
                "triangulation_angle_deg": float(angle),
                "range_m": float(range_m),
                "color_rgb": list(color),
            }
        )
    summary["accepted_points"] = len(accepted)
    if not accepted:
        summary["reason"] = "all triangulated points rejected by filters"
    return accepted, summary


def path_points(samples: list[dict[str, Any]]) -> np.ndarray:
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


def svg_color(rgb: list[int] | tuple[int, int, int]) -> str:
    return f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"


def render_svg(
    output: Path,
    session_id: str,
    points: list[dict[str, Any]],
    samples: list[dict[str, Any]],
    pair_summaries: list[dict[str, Any]],
    summary: dict[str, Any],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    camera_xy = path_points(samples)
    point_xyz = np.array([point["point_m"] for point in points], dtype=np.float64) if points else np.empty((0, 3))
    if len(point_xyz):
        top_points = np.vstack([camera_xy, point_xyz[:, :2]])
    else:
        top_points = camera_xy
    min_x, min_y = np.min(top_points, axis=0) - 0.25
    max_x, max_y = np.max(top_points, axis=0) + 0.25
    world_width = max(max_x - min_x, 1e-6)
    world_height = max(max_y - min_y, 1e-6)

    side_origin = camera_xy[0]
    side_axis = camera_xy[-1] - camera_xy[0]
    side_length = float(np.linalg.norm(side_axis))
    if side_length < 1e-9:
        side_axis = np.array([1.0, 0.0])
    else:
        side_axis = side_axis / side_length
    if len(point_xyz):
        side_s = (point_xyz[:, :2] - side_origin) @ side_axis
        side_z = point_xyz[:, 2]
        min_s = min(float(np.min(side_s)), 0.0) - 0.25
        max_s = max(float(np.max(side_s)), side_length) + 0.25
        min_z = min(float(np.min(side_z)), -0.25) - 0.10
        max_z = max(float(np.max(side_z)), 0.30) + 0.10
    else:
        side_s = np.empty(0)
        min_s, max_s, min_z, max_z = -0.25, side_length + 0.25, -0.25, 0.35

    width, height = 1700, 1180
    top_left, top_top, top_right, top_bottom = 74, 140, 1040, 680
    side_left, side_top, side_right, side_bottom = 1110, 140, 1635, 680

    top_scale = min(
        (top_right - top_left) / world_width,
        (top_bottom - top_top) / world_height,
    )
    side_scale = min(
        (side_right - side_left) / max(max_s - min_s, 1e-6),
        (side_bottom - side_top) / max(max_z - min_z, 1e-6),
    )

    def top_px(x_m: float) -> float:
        return top_left + (x_m - min_x) * top_scale

    def top_py(y_m: float) -> float:
        return top_top + (max_y - y_m) * top_scale

    def side_px(s_m: float) -> float:
        return side_left + (s_m - min_s) * side_scale

    def side_py(z_m: float) -> float:
        return side_top + (max_z - z_m) * side_scale

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#07111f"/>',
        '<style>text { font-family: Inter, "Segoe UI", Arial, sans-serif; }</style>',
        (
            f'<text x="48" y="52" fill="#f8fafc" font-size="30" font-weight="700">'
            f'Session {html.escape(session_id)} sparse fused feature diagnostic</text>'
        ),
        (
            '<text x="48" y="84" fill="#94a3b8" font-size="16">'
            'Camera feature matches are triangulated with lidar-anchored camera poses. Diagnostic only: rough extrinsics.</text>'
        ),
        (
            '<text x="48" y="110" fill="#fbbf24" font-size="14">'
            'This is a sparse sanity check before dense 3D reconstruction, not a final room model.</text>'
        ),
        f'<rect x="{top_left}" y="{top_top}" width="{top_right - top_left}" height="{top_bottom - top_top}" rx="18" fill="#0d1726" stroke="#334155"/>',
        f'<rect x="{side_left}" y="{side_top}" width="{side_right - side_left}" height="{side_bottom - side_top}" rx="18" fill="#0d1726" stroke="#334155"/>',
        f'<text x="{top_left}" y="{top_top - 18}" fill="#f8fafc" font-size="20" font-weight="700">Top-down map frame</text>',
        f'<text x="{side_left}" y="{side_top - 18}" fill="#f8fafc" font-size="20" font-weight="700">Side view along motion axis</text>',
    ]

    for x_m in range(math.floor(min_x), math.ceil(max_x) + 1):
        x = top_px(float(x_m))
        svg.append(f'<line x1="{x:.1f}" y1="{top_top}" x2="{x:.1f}" y2="{top_bottom}" stroke="#223246"/>')
        svg.append(f'<text x="{x + 4:.1f}" y="{top_bottom + 22}" fill="#64748b" font-size="12">{x_m}m</text>')
    for y_m in range(math.floor(min_y), math.ceil(max_y) + 1):
        y = top_py(float(y_m))
        svg.append(f'<line x1="{top_left}" y1="{y:.1f}" x2="{top_right}" y2="{y:.1f}" stroke="#223246"/>')
        svg.append(f'<text x="{top_left - 42}" y="{y + 4:.1f}" fill="#64748b" font-size="12">{y_m}m</text>')

    for z_m in [round(value * 0.25, 2) for value in range(math.floor(min_z / 0.25), math.ceil(max_z / 0.25) + 1)]:
        y = side_py(z_m)
        svg.append(f'<line x1="{side_left}" y1="{y:.1f}" x2="{side_right}" y2="{y:.1f}" stroke="#223246"/>')
        svg.append(f'<text x="{side_left - 58}" y="{y + 4:.1f}" fill="#64748b" font-size="12">{z_m:+.2f}m</text>')

    path_data = " ".join(
        f'{"M" if index == 0 else "L"} {top_px(point[0]):.1f} {top_py(point[1]):.1f}'
        for index, point in enumerate(camera_xy)
    )
    svg.append(f'<path d="{path_data}" fill="none" stroke="#f8fafc" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>')
    for index, sample in enumerate(samples):
        x, y = camera_xy[index]
        svg.append(f'<circle cx="{top_px(x):.1f}" cy="{top_py(y):.1f}" r="9" fill="#f8fafc" stroke="#020617" stroke-width="2"/>')
        svg.append(f'<text x="{top_px(x) + 12:.1f}" y="{top_py(y) - 8:.1f}" fill="#f8fafc" font-size="12" font-weight="700">{sample["sample_number"]}</text>')

    if len(point_xyz):
        for index, point in enumerate(points):
            x_m, y_m, z_m = point["point_m"]
            color = svg_color(point["color_rgb"])
            opacity = 0.62 if point["reprojection_error_px"] <= summary["median_reprojection_error_px"] else 0.30
            svg.append(
                f'<circle cx="{top_px(x_m):.1f}" cy="{top_py(y_m):.1f}" r="2.2" '
                f'fill="{color}" fill-opacity="{opacity:.2f}"/>'
            )
        for point, s_m in zip(points, side_s):
            _x_m, _y_m, z_m = point["point_m"]
            color = svg_color(point["color_rgb"])
            opacity = 0.62 if point["reprojection_error_px"] <= summary["median_reprojection_error_px"] else 0.30
            svg.append(
                f'<circle cx="{side_px(float(s_m)):.1f}" cy="{side_py(z_m):.1f}" r="2.2" '
                f'fill="{color}" fill-opacity="{opacity:.2f}"/>'
            )

    camera_height = float(summary["camera_height_m"])
    svg.append(
        f'<line x1="{side_left}" y1="{side_py(camera_height):.1f}" '
        f'x2="{side_right}" y2="{side_py(camera_height):.1f}" stroke="#f8fafc" stroke-dasharray="8 8"/>'
    )
    svg.append(
        f'<text x="{side_left + 12}" y="{side_py(camera_height) - 8:.1f}" '
        f'fill="#f8fafc" font-size="13">camera height used: {camera_height:.3f} m above lidar plane</text>'
    )

    summary_y = 760
    svg.extend(
        [
            f'<rect x="48" y="{summary_y - 36}" width="1604" height="132" rx="16" fill="#0d1726" stroke="#334155"/>',
            (
                f'<text x="70" y="{summary_y}" fill="#f8fafc" font-size="18" font-weight="700">'
                f'Accepted sparse 3D points: {summary["accepted_points"]:,}; '
                f'accepted pairs: {summary["pairs_with_points"]}/{summary["pair_count"]}; '
                f'median reprojection error: {summary["median_reprojection_error_px"]:.2f}px; '
                f'median triangulation angle: {summary["median_triangulation_angle_deg"]:.2f}°</text>'
            ),
            (
                f'<text x="70" y="{summary_y + 32}" fill="#cbd5e1" font-size="15">'
                f'World extents: x {summary["x_min_m"]:+.2f}..{summary["x_max_m"]:+.2f} m, '
                f'y {summary["y_min_m"]:+.2f}..{summary["y_max_m"]:+.2f} m, '
                f'z {summary["z_min_m"]:+.2f}..{summary["z_max_m"]:+.2f} m.</text>'
            ),
            (
                f'<text x="70" y="{summary_y + 60}" fill="#94a3b8" font-size="14">'
                f'Filters: reprojection ≤ {summary["max_reprojection_error_px"]:.1f}px, '
                f'range ≤ {summary["max_range_m"]:.1f}m, '
                f'triangulation angle ≥ {summary["min_triangulation_angle_deg"]:.2f}°.</text>'
            ),
        ]
    )

    table_y = 940
    svg.append(f'<text x="48" y="{table_y - 34}" fill="#f8fafc" font-size="22" font-weight="700">Pair diagnostics</text>')
    headers = ["pair", "frames", "matches", "pose inliers", "accepted 3D", "lidar step", "reason"]
    columns = [52, 140, 300, 420, 570, 725, 860]
    for x, header in zip(columns, headers):
        svg.append(f'<text x="{x}" y="{table_y}" fill="#94a3b8" font-size="13" font-weight="700">{header}</text>')
    for row, pair in enumerate(pair_summaries[:14], start=1):
        y = table_y + row * 22
        color = "#cbd5e1" if pair["accepted_points"] > 0 else "#fb7185"
        values = [
            pair["pair_index"],
            f'{pair["first_frame"]}->{pair["second_frame"]}',
            pair["matches"],
            pair["pose_inliers"],
            pair["accepted_points"],
            f'{pair["lidar_step_m"]:.3f} m',
            pair["reason"],
        ]
        for x, value in zip(columns, values):
            svg.append(f'<text x="{x}" y="{y}" fill="{color}" font-size="13">{html.escape(str(value))}</text>')

    svg.append("</svg>")
    output.write_text("\n".join(svg), encoding="utf-8")


def write_ply(output: Path, points: list[dict[str, Any]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(points)}",
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "end_header",
    ]
    for point in points:
        x_m, y_m, z_m = point["point_m"]
        red, green, blue = point["color_rgb"]
        lines.append(f"{x_m:.6f} {y_m:.6f} {z_m:.6f} {red} {green} {blue}")
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def point_summary(
    points: list[dict[str, Any]],
    pair_summaries: list[dict[str, Any]],
    args: argparse.Namespace,
    rig_values: dict[str, float],
) -> dict[str, Any]:
    point_xyz = np.array([point["point_m"] for point in points], dtype=np.float64) if points else np.empty((0, 3))
    reprojection_errors = np.array([point["reprojection_error_px"] for point in points], dtype=np.float64)
    triangulation_angles = np.array([point["triangulation_angle_deg"] for point in points], dtype=np.float64)
    ranges = np.array([point["range_m"] for point in points], dtype=np.float64)
    if len(point_xyz):
        x_min, y_min, z_min = np.min(point_xyz, axis=0)
        x_max, y_max, z_max = np.max(point_xyz, axis=0)
    else:
        x_min = y_min = z_min = x_max = y_max = z_max = 0.0
    return {
        "accepted_points": len(points),
        "pair_count": len(pair_summaries),
        "pairs_with_points": sum(1 for pair in pair_summaries if pair["accepted_points"] > 0),
        "total_matches": int(sum(pair["matches"] for pair in pair_summaries)),
        "total_pose_inliers": int(sum(pair["pose_inliers"] for pair in pair_summaries)),
        "x_min_m": float(x_min),
        "x_max_m": float(x_max),
        "y_min_m": float(y_min),
        "y_max_m": float(y_max),
        "z_min_m": float(z_min),
        "z_max_m": float(z_max),
        "median_reprojection_error_px": float(np.median(reprojection_errors)) if len(reprojection_errors) else 0.0,
        "p95_reprojection_error_px": float(np.percentile(reprojection_errors, 95)) if len(reprojection_errors) else 0.0,
        "median_triangulation_angle_deg": float(np.median(triangulation_angles)) if len(triangulation_angles) else 0.0,
        "median_range_m": float(np.median(ranges)) if len(ranges) else 0.0,
        "camera_height_m": rig_values["camera_height_m"],
        "camera_roll_deg": rig_values["camera_roll_deg"],
        "camera_pitch_deg": rig_values["camera_pitch_deg"],
        "camera_yaw_deg": rig_values["camera_yaw_deg"],
        "max_reprojection_error_px": args.max_reprojection_error_px,
        "max_range_m": args.max_range_m,
        "min_triangulation_angle_deg": args.min_triangulation_angle_deg,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Triangulate sparse visual features using lidar-anchored camera poses"
    )
    parser.add_argument("session", type=Path, help="Downloaded capture session folder")
    parser.add_argument("--pose-json", type=Path, required=True, help="Camera pose JSON")
    parser.add_argument("--intrinsics", type=Path, required=True, help="Camera intrinsics YAML")
    parser.add_argument("--output", type=Path, required=True, help="Output SVG")
    parser.add_argument("--json-output", type=Path, help="Optional output JSON")
    parser.add_argument("--ply-output", type=Path, help="Optional output PLY point cloud")
    parser.add_argument("--frame-dir", type=Path, help="Directory for sampled frame JPEGs")
    parser.add_argument("--rig-config", type=Path, default=Path("config/rig_measurements.yaml"))
    parser.add_argument("--camera-height-m", type=float)
    parser.add_argument("--camera-roll-deg", type=float)
    parser.add_argument("--camera-pitch-deg", type=float)
    parser.add_argument("--camera-yaw-deg", type=float)
    parser.add_argument("--ffmpeg", type=Path, help="Optional ffmpeg executable path")
    parser.add_argument("--thumbnail-width", type=int, default=640)
    parser.add_argument("--jpeg-quality", type=int, default=4)
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument("--max-features", type=int, default=2500)
    parser.add_argument("--fast-threshold", type=int, default=7)
    parser.add_argument("--ratio", type=float, default=0.75)
    parser.add_argument("--ransac-threshold-px", type=float, default=1.0)
    parser.add_argument("--max-reprojection-error-px", type=float, default=6.0)
    parser.add_argument("--min-depth-m", type=float, default=0.15)
    parser.add_argument("--max-range-m", type=float, default=5.0)
    parser.add_argument("--min-triangulation-angle-deg", type=float, default=0.25)
    parser.add_argument("--min-pose-inliers", type=int, default=50)
    parser.add_argument("--min-lidar-step-m", type=float, default=0.025)
    args = parser.parse_args()

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    pose_payload = load_json(args.pose_json)
    samples = pose_payload["sampled_frames"]
    if len(samples) < 2:
        raise ValueError("at least two sampled frames are required")

    rig_values = load_rig_defaults(args.rig_config if args.rig_config.exists() else None)
    if args.camera_height_m is not None:
        rig_values["camera_height_m"] = args.camera_height_m
    if args.camera_roll_deg is not None:
        rig_values["camera_roll_deg"] = args.camera_roll_deg
    if args.camera_pitch_deg is not None:
        rig_values["camera_pitch_deg"] = args.camera_pitch_deg
    if args.camera_yaw_deg is not None:
        rig_values["camera_yaw_deg"] = args.camera_yaw_deg

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
    images = []
    for path in frame_paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"failed to read frame {path}")
        images.append(image)
    image_height, image_width = images[0].shape[:2]
    camera_matrix, distortion = load_intrinsics(args.intrinsics, image_width, image_height)

    centers = [camera_center_world(sample, rig_values["camera_height_m"]) for sample in samples]
    rotations = [
        camera_rotation_world_from_camera(
            float(sample["camera_pose"]["theta_rad"]),
            rig_values["camera_roll_deg"],
            rig_values["camera_pitch_deg"],
            rig_values["camera_yaw_deg"],
        )
        for sample in samples
    ]

    all_points: list[dict[str, Any]] = []
    pair_summaries: list[dict[str, Any]] = []
    for index, (first_image, second_image) in enumerate(zip(images, images[1:])):
        points, pair_summary = triangulate_pair(
            first_image,
            second_image,
            samples[index],
            samples[index + 1],
            centers[index],
            centers[index + 1],
            rotations[index],
            rotations[index + 1],
            camera_matrix,
            distortion,
            args.max_features,
            args.fast_threshold,
            args.ratio,
            args.ransac_threshold_px,
            args.max_reprojection_error_px,
            args.min_depth_m,
            args.max_range_m,
            args.min_triangulation_angle_deg,
            args.min_pose_inliers,
            args.min_lidar_step_m,
            index + 1,
        )
        all_points.extend(points)
        pair_summaries.append(pair_summary)

    summary = point_summary(all_points, pair_summaries, args, rig_values)
    payload = {
        "session_id": manifest["session_id"],
        "source_pose_json": str(args.pose_json),
        "source_intrinsics": str(args.intrinsics),
        "coordinate_frame": "lidar ICP map x/y in meters; z is meters above lidar scan plane",
        "warning": "Sparse diagnostic only. Uses rough camera height/roll/pitch/yaw and pairwise triangulation.",
        "summary": summary,
        "pair_summaries": pair_summaries,
        "points": all_points,
    }

    render_svg(args.output, str(manifest["session_id"]), all_points, samples, pair_summaries, summary)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if args.ply_output:
        write_ply(args.ply_output, all_points)

    print(f"Wrote {args.output}")
    if args.json_output:
        print(f"Wrote {args.json_output}")
    if args.ply_output:
        print(f"Wrote {args.ply_output}")
    print(f"Accepted sparse 3D points: {summary['accepted_points']}")
    print(f"Pairs with accepted points: {summary['pairs_with_points']}/{summary['pair_count']}")
    print(f"Median reprojection error: {summary['median_reprojection_error_px']:.2f} px")
    print(f"Median triangulation angle: {summary['median_triangulation_angle_deg']:.2f} deg")
    print(
        "Point extents: "
        f"x {summary['x_min_m']:+.2f}..{summary['x_max_m']:+.2f} m, "
        f"y {summary['y_min_m']:+.2f}..{summary['y_max_m']:+.2f} m, "
        f"z {summary['z_min_m']:+.2f}..{summary['z_max_m']:+.2f} m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
