# Autonomous Lidar-Camera Robot

An experimental Raspberry Pi robot that combines a SLAMTEC RPLIDAR A1M8, a
Raspberry Pi Camera Module 2, and a Raspberry Pi 5. The project has two linked
goals:

1. autonomously search for a visible target while using lidar as a front
   obstacle-safety sensor; and
2. capture synchronized camera and lidar data that can later reconstruct a
   static room and hallway in 3D on a Windows workstation.

> This is an actively developed prototype. The robot must be supervised, used
> in a clear indoor test area, and have a reachable motor-battery switch.

## Demo

The complete 38-second test is shown at 2x speed. The robot scans, identifies
the yellow tape-measure target with the Pi camera, steers toward it, and relies
on RPLIDAR front-sector distance for a conservative stop.

![Robot scanning and approaching a yellow tape measure](docs/assets/yellow-target-demo.gif)

## Current Status

| Capability | Status | Notes |
| --- | --- | --- |
| Pi Camera v2 acquisition | Working | IMX219 camera is available through `rpicam-*`. |
| RPLIDAR A1M8 acquisition | Working | Serial lidar scans are timestamped from the Pi. |
| Continuous front-obstacle safety | Working | Stops motion if the lidar front sector reaches the configured distance. |
| TB6612 differential drive | Working, calibrated per run | Motor trims and steering direction remain chassis-specific. |
| Yellow tape-measure search/approach | Working prototype | Direct color-component detection; requires clear lighting and supervision. |
| COCO object detection | Experimental | EfficientDet Lite0 can be used, but small or partially visible cups/bottles are not yet reliable. |
| Lidar-camera 3D reconstruction | Baseline complete | Offline camera/lidar alignment and Gaussian-splatting experiments are in progress. |
| Autonomous room/hallway navigation | Not yet complete | Requires reliable exploration, odometry/SLAM, and repeatable sensor mounting. |

## Hardware

- Raspberry Pi 5 (8 GB), powered by a USB-C power bank
- Raspberry Pi Camera Module 2 / Sony IMX219
- SLAMTEC RPLIDAR A1M8 with CP2102 USB-to-UART adapter
- Two-wheel differential-drive chassis
- TB6612FNG motor driver and separate motor battery pack
- Windows workstation for calibration, reconstruction, and GPU experiments

The Pi and the motors use separate power sources. Motor power is never routed
through Pi GPIO pins. The TB6612 driver shares a ground with the Pi and uses
GPIO only for low-current control signals.

## Robot Control Loop

```text
Pi Camera -> color/object target detector -> steering decision --+
                                                            |     |
RPLIDAR front sector -> distance safety watchdog ----------+--> TB6612 -> wheels
```

For the yellow-target experiment, the robot:

1. rotates in short pulses and captures a camera frame after each pulse;
2. requires two consecutive yellow detections before beginning approach;
3. uses target horizontal position to steer with differential wheel speeds;
4. holds briefly after one missed frame before recovering toward the last seen
   direction; and
5. stops whenever the lidar front sector is too close or lidar data becomes
   stale.

## Quick Start: Yellow Target Search

On the Pi, ensure the motor-battery switch is reachable and the test area is
clear. The following command expects the Pi project checkout at
`~/fuse-project` and the TensorFlow Lite virtual environment at `~/fuse-venv`.

```bash
cd ~/fuse-project
git pull --ff-only
cp pi/red_cup_follow_continuous.py pi/red_cup_search_and_approach.py ~/

~/fuse-venv/bin/python ~/red_cup_search_and_approach.py \
  --armed \
  --swap-steering \
  --right-trim 1.00 \
  --color-target yellow \
  --min-red-pixels 300 \
  --target-confirm-frames 2 \
  --target-lost-frames 2 \
  --stop-distance-m 0.35 \
  --forward-speed 0.50 \
  --scan-turn-speed 0.85 \
  --arc-slow 0.60 \
  --arc-fast 0.95 \
  --search-turn-pulse-s 0.45 \
  --search-camera-settle-s 0.20 \
  --scan-max-s 30 \
  --max-run-s 45 \
  --save-search-frames
```

The values above are a cautious tuning baseline, not a universal calibration.
Check that the initial log reports `motor_a_trim=1.00`; stop immediately if the
robot moves unexpectedly. See [the robot control procedure](docs/RED_CUP_FOLLOWING.md)
and [motorized-motion bring-up](docs/MOTORIZED_MOTION_BRINGUP.md) before changing
GPIO wiring or motor power.

## Reconstruction Architecture

The RPLIDAR A1M8 is a 2D scanner. A 3D reconstruction therefore requires a
rigid sensor mount and controlled motion through a static environment. The
initial architecture is deliberately loose-coupled:

```text
Rigid moving Pi rig
  camera frames + monotonic timestamps
  lidar scans  + monotonic timestamps
                |
                v
Windows workstation
  calibration -> camera/lidar trajectories -> metric alignment
              -> sparse/dense export -> 3D Gaussian Splatting experiments
```

Raw sessions are preserved on the Pi and copied to the workstation for
reproducible processing. Camera intrinsics and camera-to-lidar extrinsics are
versioned configuration inputs rather than hidden tuning constants.

## Repository Layout

- [`pi/`](pi/) - Pi-side sensor capture, lidar diagnostics, motor tests, and
  target-following scripts.
- [`reconstruction/`](reconstruction/) - Windows-side calibration, mapping,
  fusion, visualization, and export tools.
- [`config/`](config/) - Camera and rig calibration inputs.
- [`docs/`](docs/) - Build notes, safety procedures, experiments, and roadmap.
- [`notebooks/`](notebooks/) - Colab handoff for GraphDECO / 3D Gaussian
  Splatting experiments.
- `data/` - Local recordings and generated artifacts; intentionally ignored by
  Git except for small documented fixtures.

## Safety and Limitations

- Do not run autonomous tests near stairs, people, pets, cables, or fragile
  objects.
- Keep a hand near the motor-battery switch during every `--armed` run.
- Lidar safety currently observes only one horizontal scan plane and only the
  configured front sector. It does not detect every hazard.
- A monocular camera and 2D lidar do not independently produce a dense metric
  3D room model. The reconstruction pipeline depends on rigid mounting,
  calibration, and slow controlled motion.
- Fast turns, rolling shutter, blank walls, glass, darkness, and repeated
  hallway textures degrade visual reconstruction and target tracking.

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Hardware bring-up](docs/HARDWARE_BRINGUP.md)
- [Rig assembly and measurements](docs/RIG_ASSEMBLY.md)
- [Camera calibration](docs/CAMERA_INTRINSICS.md)
- [Controlled room motion](docs/CONTROLLED_ROOM_MOTION.md)
- [3D Gaussian Splatting handoff](docs/GAUSSIAN_SPLATTING_HANDOFF.md)
- [Experiment log](docs/EXPERIMENT_LOG.md)
- [Project roadmap](docs/ROADMAP.md)

## References

- [SLAMTEC RPLIDAR A1](https://www.slamtec.com/en/lidar/a1)
- [Raspberry Pi camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)
- [Raspberry Pi camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html)
