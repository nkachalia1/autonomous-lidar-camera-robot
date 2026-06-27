#!/usr/bin/env python3
"""Short open-loop drive test for the TB6612 robot base.

Run this only after ``tb6612_motor_smoke_test.py`` has passed with the wheels
lifted.  The first floor test should use very low speed, a clear area, and a
hand near the motor-battery switch.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

try:
    from gpiozero import DigitalOutputDevice, PWMOutputDevice
except ImportError as exc:  # pragma: no cover - only expected off the Pi.
    raise SystemExit(
        "gpiozero is required on the Raspberry Pi. Install with:\n"
        "  sudo apt install python3-gpiozero python3-lgpio"
    ) from exc


PIN_STBY = 25

PIN_AIN1 = 23
PIN_AIN2 = 24
PIN_PWMA = 18

PIN_BIN1 = 5
PIN_BIN2 = 6
PIN_PWMB = 13


@dataclass(frozen=True)
class MotorPins:
    in1: int
    in2: int
    pwm: int


class Tb6612Motor:
    def __init__(self, pins: MotorPins, *, reverse: bool = False) -> None:
        self.reverse = reverse
        self.in1 = DigitalOutputDevice(pins.in1, initial_value=False)
        self.in2 = DigitalOutputDevice(pins.in2, initial_value=False)
        self.pwm = PWMOutputDevice(pins.pwm, initial_value=0.0)

    def drive(self, speed: float) -> None:
        speed = max(-1.0, min(1.0, speed))
        if self.reverse:
            speed = -speed

        if speed > 0:
            self.in1.on()
            self.in2.off()
            self.pwm.value = speed
        elif speed < 0:
            self.in1.off()
            self.in2.on()
            self.pwm.value = -speed
        else:
            self.stop()

    def stop(self) -> None:
        self.pwm.value = 0.0
        self.in1.off()
        self.in2.off()

    def close(self) -> None:
        self.stop()
        self.pwm.close()
        self.in1.close()
        self.in2.close()


def bounded_speed(value: str) -> float:
    parsed = float(value)
    if not -0.5 <= parsed <= 0.5:
        raise argparse.ArgumentTypeError("speed must be between -0.5 and +0.5")
    return parsed


def bounded_duration(value: str) -> float:
    parsed = float(value)
    if not 0.1 <= parsed <= 5.0:
        raise argparse.ArgumentTypeError("duration must be between 0.1 and 5.0 seconds")
    return parsed


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run one short open-loop motor command through the TB6612."
    )
    parser.add_argument(
        "--armed",
        action="store_true",
        help="Required safety acknowledgement.",
    )
    parser.add_argument(
        "--left-speed",
        type=bounded_speed,
        required=True,
        help="Left motor speed from -0.5 to +0.5.",
    )
    parser.add_argument(
        "--right-speed",
        type=bounded_speed,
        required=True,
        help="Right motor speed from -0.5 to +0.5.",
    )
    parser.add_argument(
        "--duration-s",
        type=bounded_duration,
        default=1.0,
        help="Drive duration, max 5 seconds. Default: 1.0.",
    )
    parser.add_argument("--reverse-left", action="store_true")
    parser.add_argument("--reverse-right", action="store_true")
    args = parser.parse_args()

    if not args.armed:
        parser.error("--armed is required; verify a clear area and keep hand near switch")

    stby = DigitalOutputDevice(PIN_STBY, initial_value=False)
    left = Tb6612Motor(
        MotorPins(PIN_AIN1, PIN_AIN2, PIN_PWMA), reverse=args.reverse_left
    )
    right = Tb6612Motor(
        MotorPins(PIN_BIN1, PIN_BIN2, PIN_PWMB), reverse=args.reverse_right
    )

    def stop_all() -> None:
        left.stop()
        right.stop()
        stby.off()

    try:
        stop_all()
        print("Starting in 2 seconds. Keep one hand near the motor-battery switch.")
        time.sleep(2.0)
        stby.on()
        left.drive(args.left_speed)
        right.drive(args.right_speed)
        print(
            f"Driving left={args.left_speed:+.2f}, right={args.right_speed:+.2f} "
            f"for {args.duration_s:.1f}s"
        )
        time.sleep(args.duration_s)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; stopping motors", file=sys.stderr)
        return 130
    finally:
        stop_all()
        left.close()
        right.close()
        stby.close()


if __name__ == "__main__":
    raise SystemExit(main())
