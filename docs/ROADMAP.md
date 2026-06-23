# Roadmap

## Milestone 0: Hardware Bring-up

Goal: prove both sensors work independently on the Raspberry Pi.

Status: electronic bring-up and simultaneous ten-minute load test passed on
2026-06-22. A preliminary 0.127 m camera-to-lidar center distance has been
recorded. A rigid, electrically protected mount and resolved three-axis
measurements remain required.

Done when:

- Raspberry Pi Camera Module 2 is detected;
- a sharp, correctly exposed still image is saved;
- the A1M8 appears as a stable serial device;
- one full scan contains plausible ranges through 360 degrees;
- the Pi shows no power or thermal throttling during a ten-minute test;
- the camera and lidar are mounted rigidly.

Procedure: [HARDWARE_BRINGUP.md](HARDWARE_BRINGUP.md).

## Milestone 1: Synchronized Recorder

Goal: record both sensors with one timestamp domain.

Deliverables:

- Raspberry Pi command-line recorder;
- versioned session manifest;
- camera timestamps and lidar scan timestamps from the monotonic clock;
- clean shutdown that finalizes the manifest;
- dropped-sample counters;
- a desktop validation command that checks timestamps and files.

Acceptance test:

- record a 60-second session;
- replay all valid frames and scans;
- no timestamp reversal;
- reported duration differs by less than one camera frame interval;
- sensor gaps are identified rather than silently hidden.

## Milestone 2: Independent Geometry

Goal: validate each geometric subsystem before fusion.

Deliverables:

- camera intrinsic calibration and reprojection report;
- lidar 2D map and metric trajectory;
- camera trajectory and sparse or dense point cloud;
- visualizations for both trajectories.

Acceptance test:

- reconstruct a short loop in one room;
- lidar map closes without a gross duplicated wall;
- camera trajectory remains tracked for most of the capture;
- at least three tape-measured room dimensions are recorded for validation.

## Milestone 3: Metric Fusion

Goal: place camera geometry and lidar geometry in one metric frame.

Deliverables:

- calibrated or measured camera-to-lidar transform;
- timestamp association;
- visual-to-lidar trajectory alignment;
- fused PLY point cloud;
- report containing alignment residuals and rejected intervals.

Acceptance test:

- projected lidar points coincide with corresponding wall/object boundaries in
  selected camera frames;
- visual scale is stable across the capture;
- fused trajectory does not split one wall into obvious duplicate surfaces.

## Milestone 4: Room Reconstruction

Goal: produce a repeatable 3D model of a static room.

Capture protocol:

- good, steady lighting;
- slow movement with translation and gentle turns;
- revisit the start area for loop closure;
- avoid people or moving objects;
- include textured objects near blank walls when needed.

Initial success criteria:

- model opens in a standard point-cloud viewer;
- major walls, doorway, floor relationship, and large furniture are recognizable;
- horizontal room dimensions agree with tape measurements within 5% or 0.10 m,
  whichever is larger;
- a second capture can be processed without editing source code.

These thresholds are engineering targets, not claimed A1M8 accuracy.

## Milestone 5: Hallway Reconstruction

Goal: extend the room pipeline to a long, visually repetitive environment.

Before coding, run a failure analysis using the room logs. Likely additions are
an IMU, wheeled cart, wheel odometry, markers, ROS 2 bagging, or a depth/stereo
camera.

Acceptance criteria will be defined after measuring hallway length, lighting,
surface texture, and expected output quality.
