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

Status: a stationary, explicitly non-geometric recorder prototype is ready for
mounted development. Its 30-second and three-minute sessions passed file,
timestamp-ordering, cadence, and shared-clock-overlap validation on 2026-06-23.
The three-minute run exposed one camera gap and one combined/delayed lidar scan;
both are now reported explicitly by the validator.

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

Status: robot-chassis rig session `20260623T202904Z` passed a 30-second
camera/lidar smoke test with no detected cadence anomalies. The next task is a
first-pass projection overlay using rough physical measurements.

Deliverables:

- calibrated or measured camera-to-lidar transform;
- diagnostic image with 2D lidar points projected onto a camera frame;
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

Current blocker: repeated manual-push captures are not repeatable enough. The
best smooth-arc manual capture produced a useful held-out GraphDECO result, but
later manual repeats had much worse camera/lidar motion agreement. Controlled
motorized motion is now required before spending more time on room captures.

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

## Milestone 4A: Controlled Motorized Motion

Goal: make the robot drive slowly and repeatably before attempting autonomous
mapping.

Deliverables:

- soldered or pre-soldered TB6612 motor-driver wiring;
- safe wheel-off-table motor smoke test;
- one-second open-loop floor drive;
- low-speed straight and shallow-arc commands;
- motor command logs in the session manifest or experiment log.

Acceptance test:

- Pi does not reboot when motors run;
- both motors can be stopped reliably from software and by the battery switch;
- robot can drive a slow, boring, repeatable 18 to 24 inch path;
- camera/lidar motion diagnostic is near the current best manual result
  (`0.031 m` moving RMSE and `16.0 deg` median direction error).

## Milestone 5: Hallway Reconstruction

Goal: extend the room pipeline to a long, visually repetitive environment.

Autonomous hallway work should start only after controlled motorized room motion
is repeatable. The 2D lidar will be used for obstacle avoidance and metric
localization; the Pi camera provides visual reconstruction images.

Before coding, run a failure analysis using the room logs. Likely additions are
an IMU, wheeled cart, wheel odometry, markers, ROS 2 bagging, or a depth/stereo
camera.

Acceptance criteria will be defined after measuring hallway length, lighting,
surface texture, and expected output quality.
