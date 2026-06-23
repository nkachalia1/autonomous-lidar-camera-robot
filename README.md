# Fuse Lidar and Camera

This project will combine:

- SLAMTEC RPLIDAR A1M8, a 360-degree **2D** lidar;
- Raspberry Pi 5 with 8 GB RAM;
- Raspberry Pi Camera Module 2 (Sony IMX219);
- a Windows workstation for offline reconstruction.

The first target is a 3D reconstruction of one static room. The second target is
a repeatable capture of an entire hallway.

## Feasibility

The hardware can produce a useful room reconstruction, with one essential
condition: the lidar and camera must be rigidly mounted and moved through the
static room. A stationary A1M8 measures only one horizontal plane, and a single
monocular camera has no direct metric depth measurement.

The initial pipeline will use:

1. camera imagery for visual features and dense/sparse 3D geometry;
2. lidar scan matching for metric horizontal motion and room dimensions;
3. calibrated camera-to-lidar geometry and timestamps to align both trajectories;
4. workstation-side optimization and export to PLY or OBJ.

This is deliberately a loose-coupled first version. A tightly coupled visual-
lidar optimizer can follow after the basic measurements are trustworthy.

## System Shape

```text
Rigid moving rig
  RPLIDAR A1M8 ─┐
                ├─ Raspberry Pi recorder ─ session directory ─┐
  Pi Camera v2 ─┘                                             │
                                                              ▼
Windows workstation: calibration → trajectories → alignment → 3D model
```

Raspberry Pi OS remains the acquisition operating system for the room MVP.
Heavy reconstruction runs on the Windows workstation. ROS 2 on 64-bit Ubuntu
24.04 is reserved as a later migration path, because official ROS 2 Jazzy
packages target Ubuntu Noble and ROS recommends 64-bit Ubuntu as the simplest
supported Raspberry Pi configuration.

## Milestones

| Milestone | Result |
| --- | --- |
| 0. Hardware bring-up | Repeatable camera image and valid 360-degree lidar scan |
| 1. Synchronized recorder | Timestamped, replayable camera and lidar session |
| 2. Independent reconstruction | Camera reconstruction plus lidar 2D map/trajectory |
| 3. Metric fusion | Visual trajectory aligned to lidar scale and frame |
| 4. Room acceptance test | Exported 3D model with measured room dimensions |
| 5. Hallway hardening | Longer capture, loop closure, motion robustness |

See [docs/ROADMAP.md](docs/ROADMAP.md) for acceptance criteria.

## Immediate Next Step

Complete [docs/HARDWARE_BRINGUP.md](docs/HARDWARE_BRINGUP.md). Do not start
fusion work until both sensors independently produce valid data and the physical
mount is rigid.

## Known Limitations

- Camera Module 2 uses a rolling shutter; fast handheld movement can bend
  geometry.
- Blank walls, glass, mirrors, darkness, and repetitive hallway textures are
  difficult for visual reconstruction.
- The A1M8 contributes metric geometry only in its scan plane unless the rig
  changes height or tilt.
- Software synchronization is sufficient for a slow room MVP, but hallway
  performance may justify an IMU, wheel odometry, or improved synchronization.

## Official References

- [SLAMTEC RPLIDAR A1](https://www.slamtec.com/en/lidar/a1)
- [SLAMTEC ROS 2 driver](https://github.com/Slamtec/sllidar_ros2)
- [Raspberry Pi Camera Module documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)
- [Raspberry Pi camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [ROS 2 Jazzy on Raspberry Pi](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html)
- [ROS 2 Jazzy Ubuntu packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

