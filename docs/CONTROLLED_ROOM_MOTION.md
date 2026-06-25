# Controlled Room Motion Test

This is the first reconstruction-style capture. It is not hallway SLAM yet.
The goal is to produce a metric top-down lidar sweep from a short, measured,
mostly straight motion.

## Hypothesis

If the rigid rig is moved straight forward by a measured distance while the
camera and lidar record from the same Raspberry Pi monotonic clock, then a
simple assumed-trajectory renderer should place lidar returns into a coherent
top-down room slice. Large walls and furniture should not smear wildly unless
the rig rotated, slipped, or the lidar angle/extrinsics are still poor.

## Success Criteria

- The Raspberry Pi recorder exits with camera status `0` and lidar status `0`.
- Desktop validation passes timestamp ordering and overlap checks.
- The assumed-motion SVG opens and shows recognizable room structure.
- The run records what was measured: path length, motion start/end timing, and
  obvious failure symptoms.

This test does not claim full 3D reconstruction. It validates the motion capture
pipeline and the lidar-side metric geometry first.

## Physical Setup

Use the current robot-chassis rig and current provisional calibration:

- camera intrinsics: `config/camera_intrinsics_pi_camera_v2_1920x1080.yaml`;
- lidar angle offset: `+125 deg`;
- camera roll/pitch for overlay work: `roll=-2 deg`, `pitch=-1 deg`.

Before starting:

1. Put the rig on the floor or a large stable surface where it can roll straight.
2. Mark a start line and an end line exactly 24 inches apart.
3. Point the camera/lidar forward along the travel direction.
4. Keep the room static: no people walking through, no moving doors, no fan
   pointed at curtains.
5. Add visual texture if possible, but the lidar side mainly needs opaque
   surfaces.

## Capture Protocol

For the first run:

- total duration: 30 seconds;
- first 5 seconds: hold still at the start line;
- next 20 seconds: slowly move straight forward from start to end;
- last 5 seconds: hold still at the end line;
- travel distance: 24 inches = 0.6096 m.

Avoid turning the rig. A slight wiggle is okay for a diagnostic; a visible turn
will smear the assumed-motion map.

## Raspberry Pi Command

SSH into the Pi and run:

```bash
cd ~/fuse-recorder
python3 capture_session.py \
  --duration 30 \
  --capture-mode reconstruction_candidate \
  --geometry-valid-for-reconstruction
```

When the command prints `Session directory: ...`, get ready:

1. Hold still until about 5 seconds after recording starts.
2. Push/roll straight to the 24-inch end mark over about 20 seconds.
3. Hold still for the final 5 seconds.

After it finishes, record the session ID printed at the end.

## Copy to Windows

In PowerShell, replace `SESSION_ID`:

```powershell
$session = "SESSION_ID"
scp -r pi5@pi5.local:/home/pi5/fuse-data/sessions/$session "$HOME\Downloads\"
```

## Validate

```powershell
cd "C:\Users\Neel\Documents\Fuse Lidar and Camera"
python reconstruction\validate_session.py "$HOME\Downloads\$session"
```

## Render Assumed-motion Lidar Map

```powershell
python reconstruction\render_assumed_motion_lidar_map.py `
  "$HOME\Downloads\$session" `
  --output "data\room-motion\$session-straight-24in.svg" `
  --ply-output "data\room-motion\$session-straight-24in.ply" `
  --path-length-m 0.6096 `
  --motion-start-s 5 `
  --motion-end-s 25 `
  --lidar-angle-offset-deg 125
```

Open the SVG first. If it looks plausible, the PLY can be opened in a point
cloud viewer as a flat 2D point cloud.

## Render ICP-estimated Lidar Map

After the assumed-motion diagnostic, render a first-pass lidar odometry map.
This estimates a 2D trajectory from scan-to-scan ICP during the marked motion
window instead of assuming the rig moved perfectly straight.

```powershell
python reconstruction\render_icp_lidar_map.py `
  "$HOME\Downloads\$session" `
  --output "data\room-motion\$session-icp-map.svg" `
  --png-output "data\room-motion\$session-icp-map.png" `
  --ply-output "data\room-motion\$session-icp-map.ply" `
  --trajectory-output "data\room-motion\$session-icp-trajectory.json" `
  --motion-start-s 5 `
  --motion-end-s 25 `
  --lidar-angle-offset-deg 125
```

For the current 24-inch protocol, the physical travel target is 0.6096 m. A
reasonable first ICP result should estimate a similar path length and only a
small net rotation. It does not need to be exact: hand pushing, wheel slip,
scene symmetry, and glass/window returns can all bias the estimate.

By default, the ICP renderer skips oversized lidar scans whose valid return
count is more than 1.5 times the session median. This matches the validation
warning threshold and prevents one merged/oversized scan from influencing the
trajectory. Use `--max-valid-points-ratio 0` only when intentionally debugging
raw scan anomalies.

## Reference-board Scale Check

Use this after an ICP map when the trajectory estimate is repeatable but short
of the physical 24-inch travel distance.

Physical setup:

1. Keep the 24-inch start/end travel marks.
2. Put two flat opaque vertical reference boards or cardboard box faces on one
   side of the travel path.
3. Align board A with the start mark and board B with the end mark, so the
   measured separation between their front faces is also 24 inches
   (`0.6096 m`).
4. Keep both boards tall enough to intersect the lidar scan plane and wide
   enough to make clear line segments in the top-down lidar map.
5. Keep the boards out of the robot path so the rig does not bump or drag.

After capture, validation, and ICP rendering, label straight line candidates:

```powershell
python reconstruction\detect_lidar_landmarks.py `
  "$HOME\Downloads\$session" `
  --trajectory "data\room-motion\$session-icp-trajectory.json" `
  --output "data\room-motion\$session-landmark-candidates.svg" `
  --json-output "data\room-motion\$session-landmark-candidates.json" `
  --lidar-angle-offset-deg 125
```

Open the labeled SVG and identify the two candidate IDs that correspond to the
reference boards. Then measure their mapped center distance:

```powershell
python reconstruction\detect_lidar_landmarks.py `
  "$HOME\Downloads\$session" `
  --trajectory "data\room-motion\$session-icp-trajectory.json" `
  --output "data\room-motion\$session-landmark-candidates.svg" `
  --json-output "data\room-motion\$session-landmark-candidates.json" `
  --lidar-angle-offset-deg 125 `
  --measure-ids A,B `
  --expected-distance-m 0.6096
```

Replace `A,B` with the two numbered candidate IDs from the SVG. If the measured
distance is close to `0.6096 m` but the ICP path is short, the map scale is
reasonable and the path-length estimate is biased. If the measured board
distance is also short, the reconstruction is compressing metric scale.

## Expected Failure Symptoms

- The map looks like several rotated copies of the same wall: the rig turned.
- The map stretches/compresses along the path: the 24-inch distance or timing is
  wrong.
- The map is mostly window/glass artifacts: choose more opaque targets or a
  different room direction.
- Validation reports missing frame/scan intervals: repeat the capture before
  tuning geometry.
- The ICP map path is nearly zero length: consecutive scans are too similar, the
  motion window is wrong, or scan matching is stuck in a local minimum.
- The ICP map path rotates sharply or curls back on itself: the rig turned,
  the scene is ambiguous, or the ICP correspondence thresholds are too loose.
- The landmark detector labels walls/furniture instead of the reference boards:
  move the boards closer, make them wider, or temporarily remove nearby clutter.
