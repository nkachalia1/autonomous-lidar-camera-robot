# Motorized Motion Bring-up

This is the next milestone after manual-push captures. The reconstruction
pipeline can already produce a recognizable room result, but repeated manual
pushes produced inconsistent camera/lidar motion. Controlled motor motion is
now the smallest practical step toward repeatable room and hallway mapping.

## Hypothesis

If the robot base can drive both wheel motors slowly and repeatably through a
TB6612 driver, then the capture path can become smoother than hand-pushed
motion and the camera/lidar trajectory diagnostic should become more stable.

## Success Criteria

Minimum pass:

- Pi remains powered by USB-C and does not reboot when motors start;
- motor battery powers only the TB6612 `VM` motor-power input;
- TB6612 logic is powered by Pi 3.3 V on `VCC`;
- Pi ground, battery negative, and TB6612 ground are common;
- wheel-off-table smoke test spins Motor A, Motor B, then both motors;
- open-loop drive test moves slowly for 1 second and stops.

Do not run a reconstruction capture from motor motion until the motor-only
tests pass.

## Required Wiring

Use the Raspberry Pi physical pin numbers below. The GPIO names are BCM GPIO
numbers.

Power:

| Connection | Destination |
|---|---|
| Pi physical pin 1, 3.3 V | TB6612 `VCC` |
| Pi physical pin 6, GND | TB6612 `GND` |
| Battery red positive | on/off switch middle pin |
| Switch outer pin | TB6612 `VM` |
| Battery black negative | TB6612 `GND` |

All grounds must be connected together:

```text
Pi GND ----- TB6612 GND ----- battery black negative
```

Control:

| TB6612 pin | Raspberry Pi GPIO | Pi physical pin |
|---|---:|---:|
| `PWMA` | GPIO18 | 12 |
| `AIN1` | GPIO23 | 16 |
| `AIN2` | GPIO24 | 18 |
| `STBY` | GPIO25 | 22 |
| `BIN1` | GPIO5 | 29 |
| `BIN2` | GPIO6 | 31 |
| `PWMB` | GPIO13 | 33 |

Motors:

| Motor | TB6612 outputs |
|---|---|
| Left motor | `AO1`, `AO2` |
| Right motor | `BO1`, `BO2` |

If a motor spins backward, swap that motor's two output wires or use the
software reverse flag.

## Hardware Notes

- Do not connect a motor directly to Raspberry Pi GPIO.
- Do not connect battery positive to Raspberry Pi 3.3 V, 5 V, or GPIO pins.
- TB6612 header pins must be soldered or the board must be pre-soldered.
  Glue is not an electrical connection.
- Female-female jumper wires are acceptable for Pi GPIO control signals.
- Motor wires and battery wires should be more secure than GPIO jumpers because
  they carry motor current.
- Wheels must be lifted for the first smoke test.

## Pi Setup

On Raspberry Pi OS, make sure GPIO support is installed:

```bash
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio
```

Copy the current `pi/` scripts to the Pi if needed. From Windows:

```powershell
scp pi\tb6612_motor_smoke_test.py pi5@pi5.local:/home/pi5/
scp pi\open_loop_drive_test.py pi5@pi5.local:/home/pi5/
```

If `.local` name resolution fails, use the Pi IP address.

## Wheel-off-table Smoke Test

1. Pi powered by USB-C.
2. Motor battery switch off.
3. Wheels lifted.
4. SSH into the Pi.
5. Run:

```bash
python3 ~/tb6612_motor_smoke_test.py --armed --speed 0.25
```

When the script prompts you, turn the motor battery switch on.

Expected:

- Motor A spins for about 1 second;
- Motor B spins for about 1 second;
- both motors spin for about 1 second;
- both motors stop.

If nothing moves:

- confirm the TB6612 board has soldered headers;
- confirm battery voltage reaches `VM` and `GND`;
- confirm `VCC` is Pi 3.3 V, not motor battery voltage;
- confirm `STBY` is wired to GPIO25 / physical pin 22;
- briefly test each motor directly on the battery for less than 1 second.

If a wheel spins backward, note which one. Do not rewire immediately; it can be
corrected with `--reverse-a`, `--reverse-b`, `--reverse-left`, or
`--reverse-right` after identifying the mapping.

## First Open-loop Floor Test

Only run this after the wheel-off-table smoke test passes.

Use a clear floor area, very low speed, and keep one hand near the motor-battery
switch:

```bash
python3 ~/open_loop_drive_test.py \
  --armed \
  --left-speed 0.20 \
  --right-speed 0.20 \
  --duration-s 1.0
```

If it drives backward, rerun with both speeds negative or use reverse flags.
If it curves, reduce the faster side slightly, for example:

```bash
python3 ~/open_loop_drive_test.py \
  --armed \
  --left-speed 0.18 \
  --right-speed 0.22 \
  --duration-s 1.0
```

## Next Reconstruction Use

Do not start with lidar/camera reconstruction. The next reconstruction-related
test should be:

1. motor-only smoke test;
2. one-second floor drive;
3. three-second slow straight/arc drive;
4. then a short synchronized capture while the robot drives itself.

The expected improvement is not speed. The expected improvement is repeatable,
boring, low-jerk motion.
