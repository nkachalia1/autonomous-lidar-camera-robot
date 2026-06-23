# Architecture

## Design Decision

For the room MVP, the Raspberry Pi is a synchronized data recorder and the
Windows machine is the reconstruction workstation.

This split:

- keeps the supported Raspberry Pi camera stack (`rpicam-apps`, `libcamera`, and
  Picamera2) available on Raspberry Pi OS;
- avoids making real-time dense reconstruction a Pi requirement;
- lets us preserve raw data and iterate on algorithms without rescanning the
  room after every code change.

## Capture Rig

The camera and lidar must share a rigid mount. Movement between them invalidates
the extrinsic calibration.

Recommended physical arrangement for the first experiment:

- lidar scan plane approximately level and 0.8-1.2 m above the floor;
- camera facing forward with an unobstructed view;
- both devices fixed to one plate or frame;
- known approximate translation and rotation measured before calibration;
- slow movement on a cart or careful handheld movement.

The lidar and camera do not need overlapping fields of view at every instant,
but overlap makes calibration and visual validation easier.

## Coordinate Frames

Use these conceptual frames:

- `world`: reconstruction frame; Z points up;
- `lidar`: origin at the lidar measurement center;
- `camera`: optical frame used by the camera model;
- `rig`: optional convenience frame fixed to the mounting plate.

Store transforms with an unambiguous name and direction. For example,
`T_camera_lidar` maps a point expressed in `lidar` coordinates into `camera`
coordinates. Every file containing a transform must state this convention.

## Session Data Contract

Each capture session will eventually contain:

```text
data/sessions/<session-id>/
  manifest.json
  camera/
    frames/                 # or an encoded video plus exact frame timestamps
    timestamps.csv
  lidar/
    scans.jsonl.zst         # timestamp, angles, ranges, quality
  calibration/
    camera_intrinsics.yaml
    T_camera_lidar.yaml
  notes.md
```

All timestamps originate from the Raspberry Pi monotonic clock. Wall-clock time
is metadata only and must not be used to synchronize samples.

The manifest records:

- software and schema versions;
- sensor settings;
- hostname and OS information;
- start and stop times;
- dropped-frame and rejected-scan counts;
- calibration identifiers;
- free-form capture notes.

## Reconstruction Pipeline

### 1. Camera calibration

Estimate focal lengths, principal point, and distortion parameters from a
printed calibration target. Lock focus before collecting calibration and room
data.

### 2. Lidar trajectory and map

Use scan matching or 2D SLAM to estimate metric planar motion and a room outline.
This result establishes horizontal scale.

### 3. Visual trajectory and geometry

Use structure from motion or monocular visual SLAM to estimate camera poses and
reconstruct visible surfaces. Monocular scale is initially arbitrary.

### 4. Trajectory alignment

Associate visual poses with lidar poses by timestamp. Estimate a similarity
transform that aligns the visual trajectory to the metric lidar trajectory.
Reject intervals with poor visual tracking or insufficient rig motion.

### 5. Fusion and export

- transform lidar returns into the common world frame;
- color visible lidar points from nearby camera frames;
- scale and orient the visual reconstruction;
- optionally use lidar wall constraints during pose refinement;
- export point clouds and meshes with a reconstruction report.

## Why Not Claim Dense 3D from the Lidar?

The A1M8 samples a plane. As a level rig moves, it repeatedly observes a
horizontal slice of walls and objects. It strongly constrains the floor-plan and
metric trajectory, but the camera must reconstruct most vertical detail, the
floor, and the ceiling.

If the rig stays at one location, mechanically tilting the lidar can sweep
multiple planes, but that is a different acquisition design and requires a
measured tilt angle for every scan.

## Hallway Evolution

Hallways amplify visual failure modes: repetitive doors, blank walls, long
straight motion, and weak parallax. Before hallway work, evaluate:

- an IMU for visual-inertial tracking;
- a wheeled cart and wheel odometry;
- deliberate loop closures;
- textured markers in visually blank areas;
- ROS 2 recording and visualization on Ubuntu 24.04;
- stereo or depth hardware if dense, reliable geometry becomes the priority.

