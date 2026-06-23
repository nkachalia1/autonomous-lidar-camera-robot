#!/usr/bin/env python3
"""Validate a camera/lidar session recorded by pi/capture_session.py."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any


def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def percentiles(values: list[float]) -> tuple[float, float, float]:
    ordered = sorted(values)
    if not ordered:
        return 0.0, 0.0, 0.0

    def value_at(fraction: float) -> float:
        index = min(round((len(ordered) - 1) * fraction), len(ordered) - 1)
        return ordered[index]

    return value_at(0.5), value_at(0.95), max(ordered)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a recorded sensor session")
    parser.add_argument("session", type=Path)
    args = parser.parse_args()

    session = args.session.resolve()
    manifest = load_json(session / "manifest.json")
    camera_metadata = load_json(session / manifest["camera"]["metadata"])

    failures: list[str] = []
    warnings: list[str] = []

    if manifest["camera"]["exit_status"] != 0:
        failures.append(f"camera exit status is {manifest['camera']['exit_status']}")
    if manifest["lidar"]["exit_status"] != 0:
        failures.append(f"lidar exit status is {manifest['lidar']['exit_status']}")
    if not isinstance(camera_metadata, list) or not camera_metadata:
        failures.append("camera metadata is empty or not a JSON array")
        camera_metadata = []

    camera_timestamps_ns = [
        frame["SensorTimestamp"]
        for frame in camera_metadata
        if isinstance(frame, dict) and isinstance(frame.get("SensorTimestamp"), int)
    ]
    if len(camera_timestamps_ns) != len(camera_metadata):
        failures.append("one or more camera frames lack SensorTimestamp")

    lidar_timestamps_us: list[int] = []
    lidar_valid_counts: list[int] = []
    lidar_summary: dict[str, Any] | None = None
    with (session / manifest["lidar"]["scans"]).open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            try:
                item = json.loads(line)
            except json.JSONDecodeError as error:
                failures.append(f"invalid lidar JSON on line {line_number}: {error}")
                continue
            if item.get("type") == "scan":
                lidar_timestamps_us.append(item["timestamp_us"])
                lidar_valid_counts.append(item["valid_point_count"])
            elif item.get("type") == "summary":
                lidar_summary = item

    if not lidar_timestamps_us:
        failures.append("no lidar scans were recorded")
    if lidar_summary is None:
        failures.append("lidar summary record is missing")

    def strictly_increasing(values: list[int]) -> bool:
        return all(current > previous for previous, current in zip(values, values[1:]))

    if camera_timestamps_ns and not strictly_increasing(camera_timestamps_ns):
        failures.append("camera SensorTimestamp values are not strictly increasing")
    if lidar_timestamps_us and not strictly_increasing(lidar_timestamps_us):
        failures.append("lidar timestamps are not strictly increasing")

    camera_duration_s = (
        (camera_timestamps_ns[-1] - camera_timestamps_ns[0]) / 1e9
        if len(camera_timestamps_ns) > 1
        else 0.0
    )
    lidar_duration_s = (
        (lidar_timestamps_us[-1] - lidar_timestamps_us[0]) / 1e6
        if len(lidar_timestamps_us) > 1
        else 0.0
    )

    camera_gaps_ms = [
        (current - previous) / 1e6
        for previous, current in zip(camera_timestamps_ns, camera_timestamps_ns[1:])
    ]
    lidar_gaps_ms = [
        (current - previous) / 1e3
        for previous, current in zip(lidar_timestamps_us, lidar_timestamps_us[1:])
    ]

    camera_start_ns = camera_timestamps_ns[0] if camera_timestamps_ns else 0
    camera_end_ns = camera_timestamps_ns[-1] if camera_timestamps_ns else 0
    lidar_start_ns = lidar_timestamps_us[0] * 1000 if lidar_timestamps_us else 0
    lidar_end_ns = lidar_timestamps_us[-1] * 1000 if lidar_timestamps_us else 0
    overlap_s = max(
        0.0,
        (min(camera_end_ns, lidar_end_ns) - max(camera_start_ns, lidar_start_ns))
        / 1e9,
    )

    requested = manifest["requested_duration_seconds"]
    if camera_duration_s < requested * 0.95:
        warnings.append(
            f"camera timestamp duration {camera_duration_s:.1f}s is below 95% "
            f"of requested {requested}s"
        )
    if lidar_duration_s < requested * 0.95:
        warnings.append(
            f"lidar timestamp duration {lidar_duration_s:.1f}s is below 95% "
            f"of requested {requested}s"
        )
    if overlap_s < requested * 0.90:
        failures.append(
            f"camera/lidar monotonic timestamp overlap is only {overlap_s:.1f}s"
        )

    camera_p50, camera_p95, camera_max = percentiles(camera_gaps_ms)
    lidar_p50, lidar_p95, lidar_max = percentiles(lidar_gaps_ms)

    print(f"Session: {session}")
    print(f"Mode: {manifest['capture_mode']}")
    print(
        f"Camera: {len(camera_timestamps_ns)} frames, "
        f"{camera_duration_s:.3f}s, gaps ms p50/p95/max "
        f"{camera_p50:.3f}/{camera_p95:.3f}/{camera_max:.3f}"
    )
    print(
        f"Lidar: {len(lidar_timestamps_us)} scans, "
        f"{lidar_duration_s:.3f}s, gaps ms p50/p95/max "
        f"{lidar_p50:.3f}/{lidar_p95:.3f}/{lidar_max:.3f}"
    )
    if lidar_valid_counts:
        print(
            "Lidar valid returns per scan min/median/max: "
            f"{min(lidar_valid_counts)}/"
            f"{statistics.median(lidar_valid_counts):.0f}/"
            f"{max(lidar_valid_counts)}"
        )
    print(f"Shared monotonic-clock overlap: {overlap_s:.3f}s")
    print(
        "Geometry valid for reconstruction: "
        f"{manifest['geometry_valid_for_reconstruction']}"
    )

    for warning in warnings:
        print(f"WARNING: {warning}", file=sys.stderr)
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)

    if failures:
        return 1
    print("PASS: session files and timestamps are internally consistent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

