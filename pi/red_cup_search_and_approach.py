#!/usr/bin/env python3
"""Search for a red cup, approach it, and stop with lidar safety.

This is the next staged autonomy experiment after
``red_cup_follow_continuous.py``.

Behavior:

1. keep the RPLIDAR front-sector stream alive continuously;
2. look for a red cup using the Pi Camera;
3. if the cup is not visible, rotate in place and keep scanning;
4. when the cup is visible, approach it using the known-good follower logic;
5. stop if the front lidar sector gets too close or the lidar stream goes stale;
6. optionally, after a failed room scan, make short exploratory moves.

The exploratory moves are intentionally gated behind ``--allow-explore``.
Run scan-only first, then enable exploration once the in-place search works.
"""

from __future__ import annotations

import argparse
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

try:
    from red_cup_follow_continuous import (
        DEFAULT_LIDAR_STREAM,
        DEFAULT_OUTPUT_DIR,
        DEFAULT_PORT,
        FrontDistanceState,
        Target,
        Tb6612Drive,
        bounded_unit,
        detect_red_target,
        positive_float,
        read_lidar_stream,
        start_lidar_stream,
    )
except ImportError as exc:  # pragma: no cover - only expected if copied alone.
    raise SystemExit(
        "This script reuses red_cup_follow_continuous.py.\n"
        "Copy both files to the Pi first:\n"
        "  scp pi\\red_cup_follow_continuous.py pi\\red_cup_search_and_approach.py "
        "pi5@pi5.local:/home/pi5/"
    ) from exc


Direction = Literal["left", "right", "center"]
Mode = Literal["scan", "approach", "recover", "explore", "done"]


@dataclass
class SearchMemory:
    mode: Mode = "scan"
    last_seen_direction: Direction = "right"
    last_seen_s: float = 0.0
    scan_started_s: float = 0.0
    explore_count: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Search for a red cup, approach it, and avoid front obstacles with "
            "continuous RPLIDAR safety."
        )
    )
    parser.add_argument(
        "--armed",
        action="store_true",
        help="Required safety acknowledgement; keep a hand near the motor switch.",
    )
    parser.add_argument(
        "--detect-only",
        action="store_true",
        help="Capture one image, choose the red target, write debug files, and do not move.",
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--lidar-stream", type=Path, default=Path(DEFAULT_LIDAR_STREAM))
    parser.add_argument("--front-center-deg", type=float, default=85.0)
    parser.add_argument("--front-half-width-deg", type=float, default=20.0)
    parser.add_argument("--stop-distance-m", type=positive_float, default=0.203)
    parser.add_argument("--max-run-s", type=positive_float, default=75.0)
    parser.add_argument("--stale-lidar-s", type=positive_float, default=1.0)

    parser.add_argument("--left-trim", type=bounded_unit, default=0.95)
    parser.add_argument("--right-trim", type=bounded_unit, default=0.85)
    parser.add_argument("--forward-speed", type=bounded_unit, default=0.48)
    parser.add_argument("--arc-slow", type=bounded_unit, default=0.52)
    parser.add_argument("--arc-fast", type=bounded_unit, default=0.60)
    parser.add_argument("--scan-turn-speed", type=bounded_unit, default=0.58)
    parser.add_argument(
        "--search-turn-pulse-s",
        type=positive_float,
        default=0.30,
        help=(
            "Duration of each scan/recovery turn pulse before stopping for the "
            "next camera frame. Default: 0.30."
        ),
    )
    parser.add_argument(
        "--search-camera-settle-s",
        type=positive_float,
        default=0.15,
        help=(
            "How long motors remain stopped before a scan/recovery camera "
            "capture. Default: 0.15."
        ),
    )
    parser.add_argument("--explore-forward-speed", type=bounded_unit, default=0.42)
    parser.add_argument("--reverse-left", action="store_true")
    parser.add_argument("--reverse-right", action="store_true")
    parser.add_argument(
        "--swap-steering",
        action="store_true",
        help=(
            "Swap logical left/right steering commands when the physical motor "
            "layout turns away from the camera target."
        ),
    )

    parser.add_argument("--center-deadband-px", type=int, default=100)
    parser.add_argument("--min-red-pixels", type=int, default=150)
    parser.add_argument("--component-seed-min-pixels", type=int, default=20)
    parser.add_argument("--target-min-width-px", type=int, default=8)
    parser.add_argument("--target-min-height-px", type=int, default=10)
    parser.add_argument("--target-min-center-y-frac", type=float, default=0.20)
    parser.add_argument("--target-max-width-frac", type=float, default=0.70)
    parser.add_argument("--target-max-height-frac", type=float, default=0.90)
    parser.add_argument("--target-min-aspect", type=float, default=0.20)
    parser.add_argument("--target-max-aspect", type=float, default=3.50)
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=360)
    parser.add_argument("--camera-timeout-ms", type=int, default=300)
    parser.add_argument("--red-min", type=int, default=100)
    parser.add_argument("--red-green-ratio", type=float, default=1.45)
    parser.add_argument("--red-blue-ratio", type=float, default=1.45)
    parser.add_argument("--detector-model", type=Path, default=None)
    parser.add_argument("--detector-labels", type=Path, default=None)
    parser.add_argument("--detector-target-labels", default="cup")
    parser.add_argument("--detector-target-class-ids", default="")
    parser.add_argument("--detector-confidence", type=float, default=0.35)
    parser.add_argument("--detector-label-offset", type=int, default=0)
    parser.add_argument("--detector-threads", type=int, default=2)
    parser.add_argument("--detector-min-red-fraction", type=float, default=0.03)
    parser.add_argument("--fallback-to-red-blob", action="store_true")

    parser.add_argument(
        "--scan-max-s",
        type=positive_float,
        default=18.0,
        help="How long to rotate in place before declaring scan-only search failed.",
    )
    parser.add_argument(
        "--lost-recover-s",
        type=positive_float,
        default=4.0,
        help="How long to keep turning toward the last cup direction after losing it.",
    )
    parser.add_argument(
        "--scan-direction",
        choices=["left", "right"],
        default="right",
        help="Initial in-place scan direction when no cup has been seen.",
    )
    parser.add_argument(
        "--allow-explore",
        action="store_true",
        help=(
            "After a failed in-place scan, allow short forward/turn moves to search "
            "from a new pose. Leave this off for the first test."
        ),
    )
    parser.add_argument(
        "--explore-clearance-m",
        type=positive_float,
        default=0.45,
        help="Minimum front distance needed before an exploratory forward move.",
    )
    parser.add_argument(
        "--explore-forward-s",
        type=positive_float,
        default=0.60,
        help="Duration of each exploratory forward move.",
    )
    parser.add_argument(
        "--explore-turn-s",
        type=positive_float,
        default=0.85,
        help="Duration of each exploratory in-place turn when front is blocked.",
    )
    parser.add_argument(
        "--max-explore-moves",
        type=int,
        default=6,
        help="Maximum exploratory moves after failed scans.",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--save-search-frames",
        action="store_true",
        help=(
            "Preserve each camera image, annotated detection image, and detector "
            "JSON under OUTPUT_DIR/red-cup-search-frames."
        ),
    )
    args = parser.parse_args()

    if not args.armed and not args.detect_only:
        parser.error("--armed is required; verify clear space and keep hand near switch")
    if args.detect_only:
        return args
    if not args.lidar_stream.exists():
        parser.error(f"lidar stream helper does not exist: {args.lidar_stream}")
    if not Path(args.port).exists():
        parser.error(f"lidar serial port does not exist: {args.port}")
    if args.max_explore_moves < 0:
        parser.error("--max-explore-moves must be >= 0")
    return args


def target_direction(target: Target, deadband_px: int) -> Direction:
    if target.error_x < -deadband_px:
        return "left"
    if target.error_x > deadband_px:
        return "right"
    return "center"


def command_direction(direction: Direction, *, swap_steering: bool) -> Direction:
    """Map a camera-frame direction to the calibrated drivetrain direction."""
    if not swap_steering or direction == "center":
        return direction
    return "right" if direction == "left" else "left"


def spin_in_place(
    drive: Tb6612Drive,
    *,
    direction: Direction,
    speed: float,
    swap_steering: bool = False,
) -> None:
    """Rotate about the robot center as much as the TT drivetrain allows."""
    direction = command_direction(direction, swap_steering=swap_steering)
    if direction == "left":
        drive.drive(-speed, speed)
    else:
        drive.drive(speed, -speed)


def search_arc(
    drive: Tb6612Drive,
    *,
    direction: Direction,
    slow: float,
    fast: float,
    swap_steering: bool = False,
) -> None:
    """Use the known-good gentle arc behavior to recover a recently seen target."""
    direction = command_direction(direction, swap_steering=swap_steering)
    if direction == "left":
        drive.drive(slow, fast)
    else:
        drive.drive(fast, slow)


def approach_target(
    drive: Tb6612Drive,
    args: argparse.Namespace,
    target: Target,
) -> Direction:
    direction = target_direction(target, args.center_deadband_px)
    motor_direction = command_direction(
        direction,
        swap_steering=args.swap_steering,
    )
    if motor_direction == "left":
        drive.drive(args.arc_slow, args.arc_fast)
    elif motor_direction == "right":
        drive.drive(args.arc_fast, args.arc_slow)
    else:
        drive.drive(args.forward_speed, args.forward_speed)
    return direction


def sleep_with_safety(duration_s: float, emergency_stop: threading.Event) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline and not emergency_stop.is_set():
        time.sleep(0.03)


def front_snapshot(state: FrontDistanceState) -> tuple[float | None, float]:
    closest_m, last_update_s, _ = state.snapshot()
    return closest_m, last_update_s


def archive_search_frame(args: argparse.Namespace, *, step: int) -> None:
    """Preserve one search observation instead of overwriting the debug files."""
    if not args.save_search_frames:
        return

    archive_dir = args.output_dir / "red-cup-search-frames"
    archive_dir.mkdir(parents=True, exist_ok=True)
    sources = {
        "camera.jpg": args.output_dir / "red-cup-continuous.jpg",
        "detection.jpg": args.output_dir / "red-cup-continuous-detection.jpg",
        "detection.json": args.output_dir / "red-cup-continuous-detection.json",
    }
    for suffix, source in sources.items():
        if source.exists():
            shutil.copy2(source, archive_dir / f"step-{step:03d}-{suffix}")


def main() -> int:
    args = parse_args()

    if args.detect_only:
        target = detect_red_target(args)
        debug_image = args.output_dir / "red-cup-continuous-detection.jpg"
        debug_json = args.output_dir / "red-cup-continuous-detection.json"
        if target is None:
            print("NO_TARGET_SELECTED")
        else:
            print(
                "TARGET_SELECTED "
                f"cx={target.cx} cy={target.cy} "
                f"error_x={target.error_x:.1f} pixels={target.red_pixels}"
            )
        print(f"Debug image: {debug_image}")
        print(f"Debug JSON: {debug_json}")
        return 0

    # On this chassis the steering test showed that Motor A/B correspond to
    # the opposite physical sides.  When steering is swapped, swap the
    # physical-side trim calibration as well; otherwise the physical left
    # wheel receives the weaker right-side trim and stalls during straight
    # approach.
    motor_a_trim = args.right_trim if args.swap_steering else args.left_trim
    motor_b_trim = args.left_trim if args.swap_steering else args.right_trim
    drive = Tb6612Drive(
        left_trim=motor_a_trim,
        right_trim=motor_b_trim,
        reverse_left=args.reverse_left,
        reverse_right=args.reverse_right,
    )
    front_state = FrontDistanceState()
    shutdown = threading.Event()
    emergency_stop = threading.Event()
    lidar_process: subprocess.Popen[str] | None = None
    lidar_log: Any | None = None
    memory = SearchMemory(
        last_seen_direction=args.scan_direction,
        scan_started_s=time.monotonic(),
    )

    def stop_for_safety(message: str) -> None:
        print(message, flush=True)
        emergency_stop.set()
        drive.stop()

    def safety_watchdog() -> None:
        while not shutdown.is_set() and not emergency_stop.is_set():
            closest_m, last_update_s, _ = front_state.snapshot()
            now = time.monotonic()
            if last_update_s > 0.0 and now - last_update_s > args.stale_lidar_s:
                stop_for_safety(
                    f"SAFETY STOP: lidar stream stale for {now - last_update_s:.2f}s"
                )
                return
            if closest_m is not None and closest_m <= args.stop_distance_m:
                stop_for_safety(
                    "SAFETY STOP: "
                    f"front={closest_m:.3f} m <= {args.stop_distance_m:.3f} m"
                )
                return
            time.sleep(0.03)

    try:
        lidar_process, lidar_log = start_lidar_stream(args)
        reader_thread = threading.Thread(
            target=read_lidar_stream,
            args=(lidar_process, front_state, shutdown),
            daemon=True,
        )
        watchdog_thread = threading.Thread(target=safety_watchdog, daemon=True)
        reader_thread.start()

        print("Waiting for lidar stream...")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            _, last_update_s = front_snapshot(front_state)
            if last_update_s > 0.0:
                break
            time.sleep(0.05)

        closest_m, last_update_s = front_snapshot(front_state)
        if last_update_s == 0.0:
            print("No lidar stream received. Not moving.", file=sys.stderr)
            return 3

        watchdog_thread.start()

        print(
            "Red cup search-and-approach. "
            f"mode=scan{' + explore' if args.allow_explore else ''}; "
            f"stop distance={args.stop_distance_m:.3f} m."
        )
        print(
            "Motor calibration: "
            f"swap_steering={args.swap_steering} "
            f"motor_a_trim={motor_a_trim:.2f} motor_b_trim={motor_b_trim:.2f}"
        )
        print(f"Initial front distance={closest_m}")
        print("Hand near the motor-battery switch. Starting in 2 seconds.")
        time.sleep(2.0)

        start_s = time.monotonic()
        step = 0

        while (
            time.monotonic() - start_s < args.max_run_s
            and not emergency_stop.is_set()
            and memory.mode != "done"
        ):
            step += 1
            now = time.monotonic()
            closest_m, _ = front_snapshot(front_state)

            # Object detection was unreliable while the chassis rotated
            # continuously.  Search therefore uses a deliberate
            # turn-stop-settle-capture sequence.  Approach mode remains
            # continuous once a target has been acquired.
            if memory.mode in ("scan", "recover"):
                drive.stop()
                sleep_with_safety(args.search_camera_settle_s, emergency_stop)
                if emergency_stop.is_set():
                    break

            target = detect_red_target(args)
            archive_search_frame(args, step=step)
            if emergency_stop.is_set():
                break

            if target is not None:
                direction = target_direction(target, args.center_deadband_px)
                if direction != "center":
                    memory.last_seen_direction = direction
                memory.last_seen_s = now
                memory.mode = "approach"
                print(
                    f"{step}: APPROACH front={closest_m} "
                    f"error_x={target.error_x:.1f} red_pixels={target.red_pixels} "
                    f"source={target.source} label={target.label} dir={direction}"
                )
                approach_target(drive, args, target)
                continue

            if memory.mode == "approach":
                memory.mode = "recover"
                print(f"{step}: target lost -> RECOVER {memory.last_seen_direction}")

            if memory.mode == "recover":
                if now - memory.last_seen_s <= args.lost_recover_s:
                    print(
                        f"{step}: RECOVER front={closest_m} "
                        f"turn={memory.last_seen_direction}"
                    )
                    search_arc(
                        drive,
                        direction=memory.last_seen_direction,
                        slow=args.arc_slow,
                        fast=args.arc_fast,
                        swap_steering=args.swap_steering,
                    )
                    sleep_with_safety(args.search_turn_pulse_s, emergency_stop)
                    drive.stop()
                    continue
                drive.stop()
                memory.mode = "scan"
                memory.scan_started_s = now
                print(f"{step}: recover timed out -> SCAN")

            if memory.mode == "scan":
                scan_elapsed_s = now - memory.scan_started_s
                if scan_elapsed_s <= args.scan_max_s:
                    print(
                        f"{step}: SCAN front={closest_m} "
                        f"turn={memory.last_seen_direction} "
                        f"elapsed={scan_elapsed_s:.1f}s"
                    )
                    spin_in_place(
                        drive,
                        direction=memory.last_seen_direction,
                        speed=args.scan_turn_speed,
                        swap_steering=args.swap_steering,
                    )
                    sleep_with_safety(args.search_turn_pulse_s, emergency_stop)
                    drive.stop()
                    continue

                drive.stop()
                if not args.allow_explore:
                    print("SCAN_FAILED_NO_TARGET: enable --allow-explore for step 2")
                    memory.mode = "done"
                    continue
                if memory.explore_count >= args.max_explore_moves:
                    print("EXPLORE_FAILED_NO_TARGET: max exploratory moves reached")
                    memory.mode = "done"
                    continue
                memory.mode = "explore"
                print(f"{step}: scan timed out -> EXPLORE move {memory.explore_count + 1}")

            if memory.mode == "explore":
                closest_m, _ = front_snapshot(front_state)
                memory.explore_count += 1
                if closest_m is not None and closest_m >= args.explore_clearance_m:
                    print(
                        f"{step}: EXPLORE forward front={closest_m:.3f} "
                        f"for {args.explore_forward_s:.2f}s"
                    )
                    drive.drive(args.explore_forward_speed, args.explore_forward_speed)
                    sleep_with_safety(args.explore_forward_s, emergency_stop)
                else:
                    print(
                        f"{step}: EXPLORE blocked front={closest_m}; "
                        f"turn {memory.last_seen_direction}"
                    )
                    spin_in_place(
                        drive,
                        direction=memory.last_seen_direction,
                        speed=args.scan_turn_speed,
                        swap_steering=args.swap_steering,
                    )
                    sleep_with_safety(args.explore_turn_s, emergency_stop)
                drive.stop()
                time.sleep(0.15)
                memory.mode = "scan"
                memory.scan_started_s = time.monotonic()

        if emergency_stop.is_set():
            print("STOP_CONDITION_REACHED")
        elif memory.mode == "done":
            print("SEARCH_FINISHED_WITHOUT_TARGET")
        else:
            print("MAX_RUN_SECONDS reached")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; stopping", file=sys.stderr)
        return 130
    finally:
        print("Stopping")
        shutdown.set()
        drive.stop()
        drive.close()
        if lidar_process is not None and lidar_process.poll() is None:
            lidar_process.send_signal(signal.SIGINT)
            try:
                lidar_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                lidar_process.kill()
        if lidar_log is not None:
            lidar_log.close()
        print(f"Debug image: {args.output_dir / 'red-cup-continuous-detection.jpg'}")


if __name__ == "__main__":
    raise SystemExit(main())
