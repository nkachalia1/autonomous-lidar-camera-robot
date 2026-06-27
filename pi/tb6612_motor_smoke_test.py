#!/usr/bin/env python3
"""Safe TB6612 motor-driver smoke test for the Raspberry Pi robot.

This is a bench test, not an autonomous driving script.  Keep the robot's wheels
lifted off the table/floor.  The Raspberry Pi should be powered from USB-C, and
the motor battery should feed only the TB6612 VM/GND motor-power side.
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
    name: str
    in1: int
    in2: int
    pwm: int


MOTOR_A = MotorPins("A", PIN_AIN1, PIN_AIN2, PIN_PWMA)
MOTOR_B = MotorPins("B", PIN_BIN1, PIN_BIN2, PIN_PWMB)


class Tb6612Motor:
    def __init__(self, pins: MotorPins, *, reverse: bool = False) -> None:
        self.pins = pins
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


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def bounded_speed(value: str) -> float:
    parsed = float(value)
    if not 0.0 < parsed <= 0.6:
        raise argparse.ArgumentTypeError("speed must be > 0.0 and <= 0.6")
    return parsed


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Bench-test two TT motors through a TB6612 driver."
    )
    parser.add_argument(
        "--armed",
        action="store_true",
        help="Required safety acknowledgement. Wheels must be lifted.",
    )
    parser.add_argument(
        "--speed",
        type=bounded_speed,
        default=0.25,
        help="PWM duty cycle for the smoke test, 0.0..0.6. Default: 0.25.",
    )
    parser.add_argument(
        "--duration-s",
        type=positive_float,
        default=1.0,
        help="Seconds per motor phase. Default: 1.0.",
    )
    parser.add_argument(
        "--countdown-s",
        type=positive_float,
        default=3.0,
        help="Seconds to wait before enabling STBY. Default: 3.0.",
    )
    parser.add_argument(
        "--reverse-a",
        action="store_true",
        help="Invert Motor A direction in software.",
    )
    parser.add_argument(
        "--reverse-b",
        action="store_true",
        help="Invert Motor B direction in software.",
    )
    args = parser.parse_args()

    if not args.armed:
        parser.error("--armed is required; lift the wheels and verify wiring first")

    stby = DigitalOutputDevice(PIN_STBY, initial_value=False)
    motor_a = Tb6612Motor(MOTOR_A, reverse=args.reverse_a)
    motor_b = Tb6612Motor(MOTOR_B, reverse=args.reverse_b)

    def stop_all() -> None:
        motor_a.stop()
        motor_b.stop()
        stby.off()

    try:
        stop_all()
        print("Safety check:")
        print("  - Pi powered by USB-C")
        print("  - motor battery feeds TB6612 VM/GND only")
        print("  - robot wheels are lifted")
        print(f"Turn motor battery ON within {args.countdown_s:.1f}s if it is off.")
        time.sleep(args.countdown_s)

        print("Enabling TB6612 STBY")
        stby.on()
        time.sleep(0.3)

        print(f"Motor A forward at {args.speed:.2f}")
        motor_a.drive(args.speed)
        time.sleep(args.duration_s)
        motor_a.stop()
        time.sleep(0.5)

        print(f"Motor B forward at {args.speed:.2f}")
        motor_b.drive(args.speed)
        time.sleep(args.duration_s)
        motor_b.stop()
        time.sleep(0.5)

        print(f"Both motors forward at {args.speed:.2f}")
        motor_a.drive(args.speed)
        motor_b.drive(args.speed)
        time.sleep(args.duration_s)

        print("Smoke test complete")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; stopping motors", file=sys.stderr)
        return 130
    finally:
        stop_all()
        motor_a.close()
        motor_b.close()
        stby.close()


if __name__ == "__main__":
    raise SystemExit(main())
