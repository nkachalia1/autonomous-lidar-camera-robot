#!/usr/bin/env python3
"""Follow a red target using the Pi Camera and continuous RPLIDAR safety.

This is a small autonomy milestone, not a general navigation stack:

* camera: detect the horizontal direction of a red cup/target;
* lidar: continuously watch a forward sector and stop near obstacles;
* motors: use a TB6612 driver to arc left/right or drive forward.

Run only in a clear test area with one hand near the motor-battery switch.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

try:
    from gpiozero import DigitalOutputDevice, PWMOutputDevice
except ImportError as exc:  # pragma: no cover - only expected off the Pi.
    raise SystemExit(
        "gpiozero is required on the Raspberry Pi. Install with:\n"
        "  sudo apt install python3-gpiozero python3-lgpio"
    ) from exc


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
DEFAULT_LIDAR_STREAM = "/home/pi5/fuse-recorder/build/lidar_front_stream"
DEFAULT_OUTPUT_DIR = Path.home() / "sensor-tests"

PIN_STBY = 25

PIN_AIN1 = 23
PIN_AIN2 = 24
PIN_PWMA = 18

PIN_BIN1 = 5
PIN_BIN2 = 6
PIN_PWMB = 13


@dataclass(frozen=True)
class Target:
    cx: int
    cy: int
    error_x: float
    red_pixels: int


@dataclass(frozen=True)
class MotorCommand:
    left: float
    right: float


class Tb6612Drive:
    def __init__(
        self,
        *,
        left_trim: float,
        right_trim: float,
        reverse_left: bool = False,
        reverse_right: bool = False,
    ) -> None:
        self.left_trim = left_trim
        self.right_trim = right_trim
        self.reverse_left = reverse_left
        self.reverse_right = reverse_right
        self.stby = DigitalOutputDevice(PIN_STBY, initial_value=False)
        self.ain1 = DigitalOutputDevice(PIN_AIN1, initial_value=False)
        self.ain2 = DigitalOutputDevice(PIN_AIN2, initial_value=False)
        self.pwma = PWMOutputDevice(PIN_PWMA, initial_value=0.0)
        self.bin1 = DigitalOutputDevice(PIN_BIN1, initial_value=False)
        self.bin2 = DigitalOutputDevice(PIN_BIN2, initial_value=False)
        self.pwmb = PWMOutputDevice(PIN_PWMB, initial_value=0.0)

    def _set_one(
        self,
        *,
        speed: float,
        trim: float,
        reverse: bool,
        in1: DigitalOutputDevice,
        in2: DigitalOutputDevice,
        pwm: PWMOutputDevice,
    ) -> None:
        speed = max(-1.0, min(1.0, speed))
        if reverse:
            speed = -speed

        if speed > 0.0:
            in1.on()
            in2.off()
            pwm.value = min(1.0, speed * trim)
        elif speed < 0.0:
            in1.off()
            in2.on()
            pwm.value = min(1.0, -speed * trim)
        else:
            pwm.value = 0.0
            in1.off()
            in2.off()

    def drive(self, left: float, right: float) -> None:
        self.stby.on()
        self._set_one(
            speed=left,
            trim=self.left_trim,
            reverse=self.reverse_left,
            in1=self.ain1,
            in2=self.ain2,
            pwm=self.pwma,
        )
        # Stagger startup slightly; this helped the TT motors avoid stalls.
        time.sleep(0.08)
        self._set_one(
            speed=right,
            trim=self.right_trim,
            reverse=self.reverse_right,
            in1=self.bin1,
            in2=self.bin2,
            pwm=self.pwmb,
        )

    def stop(self) -> None:
        self.pwma.value = 0.0
        self.pwmb.value = 0.0
        self.ain1.off()
        self.ain2.off()
        self.bin1.off()
        self.bin2.off()
        self.stby.off()

    def close(self) -> None:
        self.stop()
        self.stby.close()
        self.ain1.close()
        self.ain2.close()
        self.pwma.close()
        self.bin1.close()
        self.bin2.close()
        self.pwmb.close()


class FrontDistanceState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.closest_m: float | None = None
        self.last_update_s = 0.0
        self.last_record: dict[str, Any] | None = None

    def update(self, record: dict[str, Any]) -> None:
        with self._lock:
            closest = record.get("closest_m")
            self.closest_m = float(closest) if closest is not None else None
            self.last_update_s = time.monotonic()
            self.last_record = record

    def snapshot(self) -> tuple[float | None, float, dict[str, Any] | None]:
        with self._lock:
            return self.closest_m, self.last_update_s, self.last_record


def bounded_unit(value: str) -> float:
    parsed = float(value)
    if not 0.0 <= parsed <= 1.0:
        raise argparse.ArgumentTypeError("value must be between 0.0 and 1.0")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def read_lidar_stream(
    process: subprocess.Popen[str],
    state: FrontDistanceState,
    shutdown: threading.Event,
) -> None:
    assert process.stdout is not None
    for line in process.stdout:
        if shutdown.is_set():
            break
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get("type") == "front":
            state.update(record)


def detect_red_target(args: argparse.Namespace) -> Target | None:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    image_path = args.output_dir / "red-cup-continuous.jpg"
    debug_path = args.output_dir / "red-cup-continuous-detection.jpg"

    subprocess.run(
        [
            "rpicam-still",
            "--nopreview",
            "--immediate",
            "--timeout",
            str(args.camera_timeout_ms),
            "--width",
            str(args.image_width),
            "--height",
            str(args.image_height),
            "-o",
            str(image_path),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    image = Image.open(image_path).convert("RGB")
    arr = np.array(image).astype(np.int16)
    red = arr[:, :, 0]
    green = arr[:, :, 1]
    blue = arr[:, :, 2]

    mask = (
        (red > args.red_min)
        & (red > green * args.red_green_ratio)
        & (red > blue * args.red_blue_ratio)
    )
    ys, xs = np.where(mask)

    draw = ImageDraw.Draw(image)
    width, height = image.size
    draw.line([(width // 2, 0), (width // 2, height)], fill=(0, 0, 255), width=3)

    if len(xs) < args.min_red_pixels:
        image.save(debug_path)
        return None

    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    cx = int(xs.mean())
    cy = int(ys.mean())
    error_x = cx - width / 2

    draw.rectangle([(x0, y0), (x1, y1)], outline=(0, 255, 0), width=4)
    draw.line([(cx, 0), (cx, height)], fill=(0, 255, 0), width=3)
    draw.ellipse([(cx - 8, cy - 8), (cx + 8, cy + 8)], outline=(0, 255, 0), width=4)
    image.save(debug_path)

    return Target(cx=cx, cy=cy, error_x=float(error_x), red_pixels=int(len(xs)))


def start_lidar_stream(args: argparse.Namespace) -> tuple[subprocess.Popen[str], Any]:
    lidar_log_path = args.output_dir / "red-cup-lidar-stream.log"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    lidar_log = lidar_log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            str(args.lidar_stream),
            args.port,
            str(args.baudrate),
            str(args.front_center_deg),
            str(args.front_half_width_deg),
        ],
        stdout=subprocess.PIPE,
        stderr=lidar_log,
        text=True,
        bufsize=1,
    )
    return process, lidar_log


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Continuously follow a red cup with persistent lidar safety."
    )
    parser.add_argument("--armed", action="store_true", help="Required safety acknowledgement.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--lidar-stream", type=Path, default=Path(DEFAULT_LIDAR_STREAM))
    parser.add_argument("--front-center-deg", type=float, default=85.0)
    parser.add_argument("--front-half-width-deg", type=float, default=20.0)
    parser.add_argument("--stop-distance-m", type=positive_float, default=0.203)
    parser.add_argument("--max-run-s", type=positive_float, default=45.0)
    parser.add_argument("--stale-lidar-s", type=positive_float, default=1.0)
    parser.add_argument("--left-trim", type=bounded_unit, default=0.95)
    parser.add_argument("--right-trim", type=bounded_unit, default=0.85)
    parser.add_argument("--forward-speed", type=bounded_unit, default=0.48)
    parser.add_argument("--arc-slow", type=bounded_unit, default=0.52)
    parser.add_argument("--arc-fast", type=bounded_unit, default=0.60)
    parser.add_argument("--center-deadband-px", type=int, default=100)
    parser.add_argument("--min-red-pixels", type=int, default=150)
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=360)
    parser.add_argument("--camera-timeout-ms", type=int, default=300)
    parser.add_argument("--red-min", type=int, default=100)
    parser.add_argument("--red-green-ratio", type=float, default=1.45)
    parser.add_argument("--red-blue-ratio", type=float, default=1.45)
    parser.add_argument("--max-lost-steps", type=int, default=12)
    parser.add_argument("--reverse-left", action="store_true")
    parser.add_argument("--reverse-right", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    if not args.armed:
        parser.error("--armed is required; verify a clear area and keep hand near switch")
    if not args.lidar_stream.exists():
        parser.error(f"lidar stream helper does not exist: {args.lidar_stream}")
    if not Path(args.port).exists():
        parser.error(f"lidar serial port does not exist: {args.port}")

    drive = Tb6612Drive(
        left_trim=args.left_trim,
        right_trim=args.right_trim,
        reverse_left=args.reverse_left,
        reverse_right=args.reverse_right,
    )
    front_state = FrontDistanceState()
    shutdown = threading.Event()
    emergency_stop = threading.Event()
    lidar_process: subprocess.Popen[str] | None = None
    lidar_log = None

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
            _, last_update_s, _ = front_state.snapshot()
            if last_update_s > 0.0:
                break
            time.sleep(0.05)

        closest_m, last_update_s, _ = front_state.snapshot()
        if last_update_s == 0.0:
            print("No lidar stream received. Not moving.", file=sys.stderr)
            return 3

        watchdog_thread.start()

        print(
            "Continuous red cup follow. "
            f"Stop distance={args.stop_distance_m:.3f} m. Hand ready."
        )
        print(f"Initial front distance={closest_m}")
        time.sleep(2.0)

        start_s = time.monotonic()
        step = 0
        last_seen_direction = "center"
        lost_count = 0

        while (
            time.monotonic() - start_s < args.max_run_s
            and not emergency_stop.is_set()
        ):
            step += 1
            closest_m, _, _ = front_state.snapshot()
            if closest_m is None:
                print(f"{step}: no front lidar distance -> stop")
                drive.stop()
                time.sleep(0.1)
                continue

            target = detect_red_target(args)
            if emergency_stop.is_set():
                break

            if target is None:
                lost_count += 1
                print(
                    f"{step}: no red cup; front={closest_m:.3f} "
                    f"-> search {last_seen_direction}"
                )
                if lost_count > args.max_lost_steps:
                    print("lost too long -> stop")
                    drive.stop()
                    continue
                if last_seen_direction == "right":
                    drive.drive(args.arc_fast, args.arc_slow)
                else:
                    drive.drive(args.arc_slow, args.arc_fast)
                continue

            lost_count = 0
            print(
                f"{step}: front={closest_m:.3f} "
                f"cup error_x={target.error_x:.1f} "
                f"red_pixels={target.red_pixels}"
            )

            if target.error_x < -args.center_deadband_px:
                last_seen_direction = "left"
                drive.drive(args.arc_slow, args.arc_fast)
            elif target.error_x > args.center_deadband_px:
                last_seen_direction = "right"
                drive.drive(args.arc_fast, args.arc_slow)
            else:
                last_seen_direction = "center"
                drive.drive(args.forward_speed, args.forward_speed)

        if emergency_stop.is_set():
            print("STOP_CONDITION_REACHED")
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
