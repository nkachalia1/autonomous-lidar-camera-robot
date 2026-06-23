#!/usr/bin/env python3
"""Record a stationary camera/lidar session on a Raspberry Pi."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def command_text(command: list[str]) -> str:
    return shlex.join(command)


def compile_lidar_capture(script_dir: Path, sdk_root: Path, build_dir: Path) -> Path:
    executable = build_dir / "lidar_capture"
    source = script_dir / "lidar_capture.cpp"
    library = sdk_root / "output" / "Linux" / "Release" / "libsl_lidar_sdk.a"

    if not source.exists():
        raise FileNotFoundError(f"Missing lidar source: {source}")
    if not library.exists():
        raise FileNotFoundError(
            f"Missing SDK library: {library}. Run 'make -j2' in {sdk_root} first."
        )

    if executable.exists() and executable.stat().st_mtime >= max(
        source.stat().st_mtime, library.stat().st_mtime
    ):
        return executable

    build_dir.mkdir(parents=True, exist_ok=True)
    command = [
        "g++",
        "-std=c++17",
        "-O2",
        "-Wall",
        "-Wextra",
        f"-I{sdk_root / 'sdk' / 'include'}",
        f"-I{sdk_root / 'sdk' / 'src'}",
        str(source),
        str(library),
        "-lpthread",
        "-lrt",
        "-o",
        str(executable),
    ]
    print(f"Building lidar recorder:\n  {command_text(command)}")
    subprocess.run(command, check=True)
    return executable


def terminate(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Record a stationary timestamped camera/lidar session. "
            "Do not move an unmounted sensor setup."
        )
    )
    parser.add_argument("--duration", type=int, default=180, help="Seconds to record")
    parser.add_argument(
        "--sessions-root",
        type=Path,
        default=Path.home() / "fuse-data" / "sessions",
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--sdk-root", type=Path, default=Path.home() / "rplidar_sdk"
    )
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--framerate", type=int, default=15)
    parser.add_argument("--bitrate", type=int, default=4_000_000)
    parser.add_argument(
        "--capture-mode",
        default="stationary_unmounted_test",
        choices=[
            "stationary_unmounted_test",
            "mounted_rig_smoke",
            "mounted_calibration",
            "reconstruction_candidate",
        ],
        help=(
            "Human-readable capture purpose stored in manifest.json. "
            "This does not make geometry valid by itself."
        ),
    )
    parser.add_argument(
        "--geometry-valid-for-reconstruction",
        action="store_true",
        help=(
            "Mark the session as geometrically valid. Use only after the rig is "
            "rigid and calibrated."
        ),
    )
    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be positive")
    if (
        args.geometry_valid_for_reconstruction
        and args.capture_mode == "stationary_unmounted_test"
    ):
        parser.error(
            "--geometry-valid-for-reconstruction cannot be used with "
            "stationary_unmounted_test"
        )
    if not Path(args.port).exists():
        parser.error(f"lidar serial port does not exist: {args.port}")
    if shutil.which("rpicam-vid") is None:
        parser.error("rpicam-vid is not installed or not on PATH")
    if shutil.which("g++") is None:
        parser.error("g++ is not installed; install build-essential")

    script_dir = Path(__file__).resolve().parent
    build_dir = script_dir / "build"
    lidar_executable = compile_lidar_capture(
        script_dir, args.sdk_root.expanduser().resolve(), build_dir
    )

    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session_dir = args.sessions_root.expanduser().resolve() / session_id
    session_dir.mkdir(parents=True, exist_ok=False)

    camera_video = session_dir / "camera.h264"
    camera_metadata = session_dir / "camera_metadata.json"
    camera_log = session_dir / "camera.log"
    lidar_scans = session_dir / "lidar_scans.jsonl"
    lidar_log = session_dir / "lidar.log"
    manifest_path = session_dir / "manifest.json"

    camera_command = [
        "rpicam-vid",
        "--nopreview",
        "--timeout",
        str(args.duration * 1000),
        "--width",
        str(args.width),
        "--height",
        str(args.height),
        "--framerate",
        str(args.framerate),
        "--codec",
        "h264",
        "--bitrate",
        str(args.bitrate),
        "--output",
        str(camera_video),
        "--metadata",
        str(camera_metadata),
        "--metadata-format",
        "json",
    ]
    lidar_command = [
        str(lidar_executable),
        args.port,
        str(args.baudrate),
        str(args.duration),
        str(lidar_scans),
    ]

    start_wall_utc = datetime.now(timezone.utc).isoformat()
    start_monotonic_ns = time.monotonic_ns()
    camera_process: subprocess.Popen[bytes] | None = None
    lidar_process: subprocess.Popen[bytes] | None = None
    camera_status: int | None = None
    lidar_status: int | None = None

    print(f"Session directory: {session_dir}")
    print("Stationary capture only. Do not touch or move the hardware.")
    if not args.geometry_valid_for_reconstruction:
        print("Geometry flag: invalid until rigid rig calibration is recorded.")

    try:
        with camera_log.open("wb") as camera_stream, lidar_log.open(
            "wb"
        ) as lidar_stream:
            camera_process = subprocess.Popen(
                camera_command, stdout=camera_stream, stderr=subprocess.STDOUT
            )
            lidar_process = subprocess.Popen(
                lidar_command, stdout=lidar_stream, stderr=subprocess.STDOUT
            )

            while camera_process.poll() is None and lidar_process.poll() is None:
                time.sleep(1)

            if camera_process.poll() is not None and lidar_process.poll() is None:
                camera_status = camera_process.returncode
                if camera_status != 0:
                    print("Camera exited early; stopping lidar.", file=sys.stderr)
                    terminate(lidar_process)
            elif lidar_process.poll() is not None and camera_process.poll() is None:
                lidar_status = lidar_process.returncode
                if lidar_status != 0:
                    print("Lidar exited early; stopping camera.", file=sys.stderr)
                    terminate(camera_process)

            camera_status = camera_process.wait()
            lidar_status = lidar_process.wait()
    except KeyboardInterrupt:
        print("\nCapture interrupted; finalizing files.", file=sys.stderr)
        terminate(camera_process)
        terminate(lidar_process)
        camera_status = None if camera_process is None else camera_process.returncode
        lidar_status = None if lidar_process is None else lidar_process.returncode
    finally:
        terminate(camera_process)
        terminate(lidar_process)

    end_monotonic_ns = time.monotonic_ns()
    manifest = {
        "schema_version": 1,
        "session_id": session_id,
        "capture_mode": args.capture_mode,
        "geometry_valid_for_reconstruction": args.geometry_valid_for_reconstruction,
        "start_wall_utc": start_wall_utc,
        "start_monotonic_ns": start_monotonic_ns,
        "end_monotonic_ns": end_monotonic_ns,
        "requested_duration_seconds": args.duration,
        "camera": {
            "command": camera_command,
            "exit_status": camera_status,
            "video": camera_video.name,
            "metadata": camera_metadata.name,
            "log": camera_log.name,
            "width": args.width,
            "height": args.height,
            "framerate": args.framerate,
            "bitrate": args.bitrate,
        },
        "lidar": {
            "command": lidar_command,
            "exit_status": lidar_status,
            "scans": lidar_scans.name,
            "log": lidar_log.name,
            "port": args.port,
            "baudrate": args.baudrate,
        },
        "file_sizes_bytes": {
            path.name: path.stat().st_size
            for path in (
                camera_video,
                camera_metadata,
                camera_log,
                lidar_scans,
                lidar_log,
            )
            if path.exists()
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"Camera status: {camera_status}")
    print(f"Lidar status: {lidar_status}")
    print(f"Manifest: {manifest_path}")
    print(f"Session complete: {session_dir}")
    return 0 if camera_status == 0 and lidar_status == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
