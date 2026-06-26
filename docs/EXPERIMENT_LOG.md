# Experiment Log

Copy the template for every hardware or reconstruction experiment.

## 2026-06-22 Accidental Conductive Contact and Restart

### Event

During preparation for the simultaneous camera/lidar load test, a metal object
contacted the powered setup and the Raspberry Pi restarted. The SSH connection
was lost. The exact contact location was not recorded.

### Post-restart Measurements

- Raspberry Pi booted and SSH access returned;
- temperature: 45.5 degrees Celsius;
- throttling flags for the new boot: `0x0`;
- kernel log showed EXT4 orphan cleanup after the unclean shutdown;
- the root filesystem subsequently remounted read/write;
- the supplied kernel log contained no SD-card I/O, EXT4 corruption,
  undervoltage, or reset errors;
- boot warnings shown for ALSA, Bluetooth, Netplan permissions, firmware, and
  Wi-Fi do not indicate damage from this event.

### Result

Recovered, pending a controlled storage and sensor recheck. The `0x0`
throttling value applies to the current boot and cannot prove what electrically
happened at the instant of contact.

### Next Action

Run a read-only filesystem status check and a clean reboot. With power
disconnected, inspect and mechanically protect the exposed electronics before
reconnecting the lidar and repeating short sensor tests.

### Follow-up: Lidar Motor Behavior

After recovery, no `ultra_simple`, `simple_grabber`, or camera process was
running and no process owned `/dev/ttyUSB0`. Running `simple_grabber` stopped
the motor briefly during its cleanup, but the motor resumed after the program
closed the serial port.

This matches the SDK's Linux/DTR control path for models without direct motor
speed control: opening the serial channel clears DTR to spin the motor, while a
zero motor-speed request sets DTR. Closing the serial descriptor does not hold
that stop state, so this adapter returns to spinning while it remains powered.
This is treated as observed adapter behavior, not evidence of a lingering
process.

## 2026-06-22 First Ten-minute Load Attempt

### Hypothesis

The camera and lidar should operate together for ten minutes without
undervoltage, thermal throttling, USB disconnects, or sensor errors.

### Result

Partial pass. The lidar ran for the requested ten minutes and was terminated by
`timeout` with the expected status 124. Its stderr log was empty. The camera
initialized and selected its 1920x1080 mode, but exited immediately with status
255 because this `rpicam-vid` build could not infer an output format for
`/dev/null`.

Post-test measurements:

- temperature: 49.4 degrees Celsius;
- throttling flags: `0x0`;
- storage: 105 GB available;
- lidar stderr: 0 bytes;
- no USB disconnect was present in the supplied kernel excerpt.

The kernel logged CP210x control-request timeouts (`request 0x12`, status
`-110`). At least one occurred around lidar shutdown. Since scan acquisition
continued for ten minutes and there was no USB disconnect, these are recorded
as motor/DTR control warnings pending further observation, not as proof of scan
data loss.

### Next Action

Send the H.264 byte stream to standard output and redirect that stream to
`/dev/null`, leaving diagnostic output in a log. Validate the corrected camera
command briefly, then repeat the simultaneous ten-minute test.

### Camera Command Correction

The corrected camera command wrote an explicit `.h264` output file for 15
seconds at 1920x1080, 15 fps, and a requested 4 Mbit/s bitrate.

- camera exit status: 0;
- output size: 7.5 MB;
- encoder-reported average rate: approximately 4309 kbit/s;
- no camera error was present in the supplied log tail.

The camera command is ready for the simultaneous ten-minute repetition.

### Successful Simultaneous Repetition

The corrected camera and lidar commands ran concurrently for ten minutes.

- camera exit status: 0;
- camera output: 290 MB H.264 file at 1920x1080 and 15 fps;
- camera encoder-reported average rate: approximately 4057 kbit/s;
- lidar exit status: 124, expected from the ten-minute `timeout`;
- lidar stderr: 0 bytes;
- final temperature: 47.7 degrees Celsius;
- final throttling flags: `0x0`;
- root filesystem: 115 GB total, 5.5 GB used, 105 GB available;
- the final filtered kernel command produced no matching disconnect, voltage,
  MMC, EXT4, or I/O error lines after successful sudo authentication.

The stray `^[[A^[[B` characters in the terminal transcript are arrow-key escape
sequences and did not affect either process.

### Result

Pass. Both sensors operated concurrently for ten minutes without a reported
camera failure, lidar acquisition error, undervoltage, thermal throttle, storage
problem, or relevant kernel error. Milestone 0 electronic bring-up and load
testing are complete. The remaining Milestone 0 item is a rigid, protected
sensor mount suitable for calibration and motion.

### Saved Lidar Visualization

A later single-scan capture contained 1,052 angular samples and 795 valid
nonzero returns covering the full 0-360 degree interval. Valid ranges were
0.234-7.584 m, with a median of 0.708 m. The dependency-free plotter at
`reconstruction/tools/plot_lidar_scan.py` generated the top-down visualization
documented in `docs/MILESTONE0_RESULTS.md`.

### Preliminary Rig Measurement

The user measured approximately 5 inches (127 mm or 0.127 m) from the lidar
spinner center to the camera optical center. This is recorded as a scalar
center-to-center distance only. Its three translation components and camera
orientation remain unknown until the final rigid mount is assembled and
measured.

An annotated follow-up photo showed the 5-inch separation as diagonal in the
current tabletop view and a separate 4.5-inch lidar-to-Pi distance. The latter
is packaging information, not an extrinsic calibration measurement. The photo
also confirmed that the camera, Pi, lidar, USB adapter, and cables are not yet
mechanically constrained as one rig.

### Stationary Timestamp Recorder Prototype

A pre-mount recorder was prepared to test camera and lidar timestamp plumbing
without moving the loose setup. It records camera frame metadata and complete
lidar scans in the Raspberry Pi monotonic-clock domain and marks the session as
invalid for geometric reconstruction. The requested validation duration was
reduced from ten minutes to three minutes, following a 30-second smoke test.

### 30-second Timestamp Recorder Smoke Test

Session `20260623T035923Z` was recorded on the Raspberry Pi.

- the custom lidar recorder compiled successfully against the installed
  SLAMTEC SDK;
- session directory creation succeeded;
- camera exit status: 0;
- lidar exit status: 0;
- manifest finalization succeeded.

Process-level capture passed. File counts, monotonic timestamp ordering, sensor
overlap, and sample gaps remain to be checked with
`reconstruction/validate_session.py` before the three-minute capture.

The downloaded session subsequently passed desktop validation:

- camera: 443 frames over 29.461 seconds;
- camera frame gap p50/p95/max:
  66.655/66.656/66.660 ms, approximately 15 fps;
- lidar: 212 complete scans over 28.584 seconds;
- lidar scan gap p50/p95/max:
  135.478/135.853/136.099 ms, approximately 7.38 scans/s;
- lidar valid returns per scan min/median/max: 798/816/841;
- shared monotonic-clock overlap: 28.584 seconds;
- all timestamps were internally consistent;
- geometry remained explicitly invalid for reconstruction because the sensors
  were not rigidly mounted.

The smoke test fully passed and the stationary three-minute capture is cleared.

### Three-minute Stationary Timestamp Capture

Session `20260623T040231Z` completed on the Raspberry Pi.

- requested duration: 180 seconds;
- camera exit status: 0;
- lidar exit status: 0;
- manifest finalization succeeded;
- the sensors remained in stationary, unmounted-test mode;
- the session remains invalid for geometric reconstruction.

Process-level capture passed. Desktop validation of timestamp duration, cadence,
and shared-clock overlap remains required.

The downloaded session passed desktop validation:

- camera: 2,689 frames over 179.434 seconds;
- camera gap p50/p95/max:
  66.654/66.656/333.275 ms;
- one camera gap event occurred after frame 532, corresponding to approximately
  four missing nominal frame intervals;
- lidar: 1,319 scans over 178.675 seconds;
- lidar gap p50/p95/max:
  135.467/135.860/270.959 ms;
- one lidar double-length interval occurred after scan 1076;
- that lidar scan contained 1,627 valid returns versus a median of 816,
  indicating two revolutions were likely combined rather than silently lost;
- lidar recorder reported zero timeouts and zero rejected scans;
- shared monotonic-clock overlap: 178.596 seconds;
- geometry remained invalid for reconstruction because the sensors were not
  rigidly mounted.

Result: pass with two explicitly recorded cadence anomalies. The synchronized
timestamp and session-file pipeline is ready for mounted capture development.

### Robot-chassis Rig 30-second Smoke Test

Session `20260623T202904Z` was captured after moving the camera, lidar, and
Raspberry Pi onto a shared robot-chassis platform.

- camera: 443 frames over 29.462 seconds;
- camera frame gap p50/p95/max:
  66.655/66.656/66.662 ms;
- camera gap events above 1.5x nominal: 0;
- lidar: 202 scans over 28.593 seconds;
- lidar scan gap p50/p95/max:
  142.218/142.751/143.120 ms;
- lidar gap events above 1.5x nominal: 0;
- lidar valid returns per scan min/median/max: 834/850/878;
- oversized lidar scans above 1.5x median returns: 0;
- shared monotonic-clock overlap: 28.495 seconds;
- geometry remained marked invalid for reconstruction because the
  lidar-to-camera transform has not been measured or calibrated.

Result: pass. The mounted platform is stable enough for the next milestone:
first-pass lidar-to-camera projection overlay.

### First Mounted Calibration Overlay

Session `20260623T212628Z` was captured with visual calibration targets already
in the scene.

- capture mode: `mounted_calibration`;
- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 215 scans over 28.619 seconds;
- lidar gap events above 1.5x nominal: 0;
- lidar valid returns per scan min/median/max: 805/818/829;
- shared monotonic-clock overlap: 28.619 seconds;
- overlay frame/scan timestamp delta: +0.3 ms;
- geometry remained marked invalid for reconstruction pending extrinsic
  calibration.

The first angle sweep showed a clean projected scan-plane band. Visual
inspection suggested that window tape favored approximately -90 to -60 degrees,
while the opaque wall/pegboard favored approximately -150 to -120 degrees and
+120 to +150 degrees. Because glass and bright windows can produce unreliable
lidar returns, the next calibration pass should prioritize opaque pegboard,
wall, cardboard, and box edges.

Result: pass for synchronized mounted calibration capture. Calibration remains
inconclusive until a single opaque target provides an unambiguous best angle.

### Camera Re-leveling and Fine Angle Sweep

After the camera was physically adjusted to reduce tilt, session
`20260623T215235Z` was captured in `mounted_calibration` mode.

- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 219 scans over 28.756 seconds;
- lidar gap events above 1.5x nominal: 0;
- lidar valid returns per scan min/median/max: 806/818/829;
- shared monotonic-clock overlap: 28.716 seconds;
- geometry remained marked invalid for reconstruction pending calibration.

The coarse sweep favored approximately -120 and +120 degrees. A fine sweep
around +120 degrees favored +115 degrees. This is recorded as the provisional
software `lidar_angle_offset_deg`. It does not require physically rotating the
lidar.

Next action: keep `lidar_angle_offset_deg=115` fixed and sweep camera roll to
align the projected scan line with the left, middle, and right opaque tape
targets.

Roll sweep follow-up: with `lidar_angle_offset_deg=115`, the user reported
`roll=-2` and `roll=-4` as the best overlays. In both, the projected line was
slightly below the left pegboard tape and crossed through the middle pegboard
tape. Record `roll=-3` as a provisional midpoint. The remaining left-side
offset may be caused by non-level tape placement, camera pitch/height error,
approximate intrinsics, or target-plane perspective rather than roll alone.

Pitch follow-up: with `lidar_angle_offset_deg=115` and `roll=-2`, pitch sweeps
showed that no single value perfectly matched both the left and middle tape
targets. `pitch=0` was very slightly under the left tape and above the middle
tape. `pitch=-2` and nearby negative values improved the middle tape but left
the left tape low. Stop hand tuning and record a practical provisional
calibration of `roll=-2` and `pitch=-1`. The next higher-value step is camera
intrinsic calibration rather than further manual overlay fitting.

Follow-up observation: the middle tape target was physically one peg higher on
the pegboard than the left tape target. That explains the apparent left/middle
vertical disagreement and makes the provisional overlay fit more credible than
it initially appeared.

### Pi Camera V2 Intrinsic Calibration

Hypothesis: checkerboard stills from the mounted Raspberry Pi Camera Module 2
should produce usable focal length, principal point, and distortion parameters
for the same 1920x1080 capture mode used by the synchronized recorder.

Setup:

- camera: Raspberry Pi Camera Module 2 mounted on the robot-chassis rig;
- target: physical 8x8 chessboard, 7x7 inner corners;
- measured square size: 1 3/16 inches, or 30.1625 mm;
- captured images: `checker-01.jpg` through `checker-75.jpg`;
- local source folder: `C:\Users\Neel\Downloads\camera-intrinsics`;
- calibration output:
  `config/camera_intrinsics_pi_camera_v2_1920x1080.yaml`;
- diagnostic output:
  `data/calibration/camera-intrinsics-detection-sheet-75.jpg`.

Command:

```text
python reconstruction/calibrate_camera_intrinsics.py
  C:\Users\Neel\Downloads\camera-intrinsics
  --inner-cols 7
  --inner-rows 7
  --square-size-mm 30.1625
  --output config\camera_intrinsics_pi_camera_v2_1920x1080.yaml
  --diagnostic-sheet data\calibration\camera-intrinsics-detection-sheet-75.jpg
```

Measurements:

- images scanned: 75;
- checkerboards detected: 16;
- rejected images: 59;
- RMS reprojection error: 0.3725 px;
- mean per-view reprojection error: 0.0505 px;
- max per-view reprojection error: 0.0827 px;
- camera matrix:
  `fx=2626.621198`, `fy=2619.644683`, `cx=864.0909643`,
  `cy=533.5666116`;
- distortion model: OpenCV plumb-bob;
- distortion coefficients:
  `[0.04778594505, 1.696891898, -0.01610473299, -0.01806610254, -7.241881095]`.

Result: pass for a provisional intrinsic calibration. The reprojection error is
low and the result is saved as versioned configuration. The high rejection count
and large higher-order distortion terms suggest the wooden board, window
backlighting, and pose diversity are not ideal, so these intrinsics should be
validated against the next lidar-camera overlay before being treated as final.

Next action: update the lidar-camera overlay tools to consume the calibrated
camera matrix instead of approximate field-of-view values, then rerender the
mounted calibration overlay with `lidar_angle_offset_deg=115`, `roll=-2`, and
`pitch=-1`.

Follow-up: `project_lidar_overlay.py` and `render_lidar_angle_sweep.py` now
accept `--camera-intrinsics` YAML input and apply OpenCV plumb-bob distortion
when projecting lidar returns. The calibrated single overlay and a calibrated
`105,110,115,120,125` degree angle sweep were generated for session
`20260623T215235Z` under `data/calibration/`. These SVGs remain local generated
artifacts and are not tracked by Git.

User visual inspection selected `+125` degrees as the best calibrated-intrinsics
angle panel. Update `config/rig_measurements.yaml` to use
`provisional_lidar_angle_offset_deg=125`. The previous `+115` value is retained
in the log as the best pre-intrinsics/FOV-based estimate, not as the current
working value. A matching single-overlay SVG was generated as
`data/calibration/20260623T215235Z-overlay-calibrated-angle-125.svg`, with 96
projected returns inside the image and a +1.2 ms frame/scan timestamp delta.

### Controlled Room Motion Preparation

Hypothesis: before attempting full room or hallway reconstruction, a short,
measured straight-line rig motion should produce a coherent diagnostic lidar map
when scans are placed along a known 24-inch path. This verifies motion capture
plumbing and lidar metric geometry without claiming SLAM.

Prepared:

- runbook: `docs/CONTROLLED_ROOM_MOTION.md`;
- renderer: `reconstruction/render_assumed_motion_lidar_map.py`;
- Pi recorder message updated so `reconstruction_candidate` captures no longer
  print the stale stationary-only warning.

Desktop smoke test: the assumed-motion renderer was run against existing
stationary calibration session `20260623T215235Z` only to verify the code path.
It wrote an SVG and PLY under `data/room-motion/`, using 110 of 219 scans and
producing 86,409 output points. Because that source session was stationary, the
output is not interpreted as a reconstruction result.

Next action: capture a real 30-second controlled motion session with a measured
24-inch straight push, then validate and render the assumed-motion lidar map.

### First Controlled Room Motion Capture

Session `20260624T001140Z` was captured in `reconstruction_candidate` mode with
the geometry-valid flag set. The intended protocol was a 30-second capture with
a 24-inch/0.6096 m straight motion from 5 seconds to 25 seconds.

Desktop validation:

- camera exit status: 0;
- lidar exit status: 0;
- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 226 scans over 28.734 seconds;
- lidar gap events above 1.5x nominal: 1, after scan 21;
- oversized lidar scans above 1.5x median returns: 1, scan 21 with 1,670
  returns versus median 800;
- shared monotonic-clock overlap: 28.734 seconds;
- geometry valid for reconstruction: true;
- validation result: pass with one lidar cadence anomaly.

The assumed-motion renderer produced:

- `data/room-motion/20260624T001140Z-straight-24in.svg`;
- `data/room-motion/20260624T001140Z-straight-24in.ply`;
- `data/room-motion/20260624T001140Z-straight-24in-preview.png`;
- output points: 88,507;
- used scans: 113 of 226;
- filtered returns: 1,884 of 90,391 considered returns.

Result: pass for synchronized reconstruction-candidate capture and desktop map
generation. The first assumed-motion map is not yet a clean room
reconstruction. It shows recognizable repeated structures, but also
fan-shaped/smeared point bands near the rig, which is consistent with rig
rotation, uncertain motion timing, or the limitations of forcing all scans onto
a straight-line trajectory. The one early oversized lidar scan should be watched
but is not by itself a capture failure.

Next action: repeat a controlled motion test with stronger constraints:

- use a longer measured path, preferably 48 inches/1.2192 m;
- physically guide the wheels along a straight edge or wall/board;
- keep the rig orientation fixed during the push;
- keep the same 5-second still, 20-second motion, 5-second still timing; and
- render the same assumed-motion map to see whether the fan-shaped smearing
  collapses into sharper wall/object bands.

### Guided 24-inch Controlled Room Motion Capture

The power cable limited practical travel to 24 inches, so the second controlled
motion capture repeated the 0.6096 m path with better guidance rather than
forcing a 48-inch path under cable tension.

Session `20260624T002731Z` was captured in `reconstruction_candidate` mode with
the geometry-valid flag set.

Desktop validation:

- camera exit status: 0;
- lidar exit status: 0;
- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 227 scans over 28.731 seconds;
- lidar gap events above 1.5x nominal: 0;
- oversized lidar scans above 1.5x median returns: 0;
- lidar valid returns per scan min/median/max: 677/795/833;
- shared monotonic-clock overlap: 28.724 seconds;
- geometry valid for reconstruction: true;
- validation result: pass with no cadence anomalies.

The assumed-motion renderer produced:

- `data/room-motion/20260624T002731Z-straight-24in-guided.svg`;
- `data/room-motion/20260624T002731Z-straight-24in-guided.ply`;
- `data/room-motion/20260624T002731Z-straight-24in-guided-preview.png`;
- output points: 85,502;
- used scans: 114 of 227;
- filtered returns: 2,644 of 88,146 considered returns.

Result: pass for sensor recording quality, but still inconclusive for clean room
mapping. Compared with the first controlled motion session, this run eliminated
the lidar cadence anomaly, which confirms the synchronized recorder is healthy.
However, the assumed-motion map still contains fan-shaped/smeared point bands.
The remaining error is therefore more likely trajectory/orientation error or
the limits of the straight-line assumption than a timestamp or lidar data-loss
problem.

Next action: stop relying on hand-assumed trajectory for reconstruction
quality. The next engineering slice should estimate rig motion from the lidar
scans themselves, starting with 2D scan-to-scan matching/ICP or adding a simple
wheel-odometry/encoder measurement. Until that exists, additional hand-pushed
captures will mostly test operator discipline rather than improve the map.

### First ICP Lidar Odometry Map

A first workstation-side scan-matching renderer,
`reconstruction/render_icp_lidar_map.py`, was added and run against the guided
24-inch motion session `20260624T002731Z`.

Hypothesis: scan-to-scan ICP should estimate a rough 2D trajectory from the
lidar data and reduce the fan-shaped smearing produced by the assumed-straight
trajectory renderer.

Command summary:

```powershell
python reconstruction\render_icp_lidar_map.py `
  "$HOME\Downloads\20260624T002731Z" `
  --output "data\room-motion\20260624T002731Z-icp-map.svg" `
  --png-output "data\room-motion\20260624T002731Z-icp-map.png" `
  --ply-output "data\room-motion\20260624T002731Z-icp-map.ply" `
  --trajectory-output "data\room-motion\20260624T002731Z-icp-trajectory.json" `
  --motion-start-s 5 `
  --motion-end-s 25 `
  --lidar-angle-offset-deg 125
```

Measurements:

- input scans: 227;
- selected scans for ICP: 40;
- ICP steps: 39;
- rejected ICP steps: 0;
- estimated path length: 0.553 m;
- estimated net displacement: 0.541 m;
- estimated net rotation: -1.72 degrees;
- map output points: 77,958.

A stride sweep produced stable path-length estimates from 0.538 m to 0.561 m
for match strides 2, 3, 4, 5, 6, and 8. The physical target path was 24 inches
(0.6096 m), so the first ICP estimate is about 8 to 12 percent short depending
on stride. This may be caused by actual motion timing, wheel slip, scene
ambiguity, or ICP bias.

Visual result: the ICP-estimated map is materially less smeared than the
assumed-straight map. The old map fans into repeated colored wedges near the
rig, while the ICP result collapses much of that into thinner structures and
estimates a slight arc, which is plausible for a hand-pushed robot chassis.

Result: pass for a first scan-matching reconstruction slice. This is still not
full SLAM: there is no loop closure, occupancy grid, global map optimization, or
camera fusion yet.

Next action: use the ICP trajectory as the lidar-side baseline for future room
captures, then add either stronger scan-matching validation metrics or simple
wheel odometry so the trajectory can be checked against an independent motion
measurement.

### ICP Lidar Odometry Repeatability Run

Session `20260625T211553Z` repeated the 24-inch reconstruction-candidate motion
test. Desktop validation passed with one lidar timing anomaly:

- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 224 scans over 28.782 seconds;
- lidar gap events above 1.5x nominal: 1;
- oversized lidar scans above 1.5x median returns: 1;
- lidar valid returns per scan min/median/max: 661/718/1398;
- shared monotonic-clock overlap: 28.735 seconds;
- geometry valid for reconstruction: true.

The first ICP render, before adding an oversized-scan filter, estimated:

- selected scans for ICP: 40;
- ICP steps: 39;
- rejected ICP steps: 0;
- estimated path length: 0.557 m;
- estimated net displacement: 0.497 m;
- estimated net rotation: -0.31 degrees;
- map output points: 72,285.

Because validation identified scan 141 as oversized and it fell inside the
motion window, the ICP renderer was updated to skip scans whose
`valid_point_count` is more than 1.5 times the session median by default. With
that filter enabled, the repeat run estimated:

- skipped oversized scans: 1;
- estimated path length: 0.571 m;
- estimated net displacement: 0.499 m;
- estimated net rotation: 1.03 degrees;
- map output points: 72,285.

Result: pass for repeatability. The fresh run is consistent with the previous
guided run (`0.553 m` estimated path length) and again improves the visual map
relative to the assumed-straight renderer. Both ICP runs remain shorter than the
24-inch physical target (`0.6096 m`), so scale/trajectory bias is still present.
This is acceptable for the current milestone because the objective was reducing
smear and proving repeatable lidar odometry, not final metric SLAM.

Next action: keep ICP as the lidar-side baseline and add an independent motion
check next: either manually measured start/end pose markers in the map, a
wheel-encoder/odometry sensor, or a constrained straight-track capture where
the rig cannot yaw.

### Constrained 24-inch ICP Run

Session `20260625T212639Z` repeated the 24-inch reconstruction-candidate motion
test with the rig physically constrained to reduce yaw. Desktop validation was
clean:

- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 225 scans over 28.730 seconds;
- lidar gap events above 1.5x nominal: 0;
- oversized lidar scans above 1.5x median returns: 0;
- lidar valid returns per scan min/median/max: 686/720/794;
- shared monotonic-clock overlap: 28.694 seconds;
- geometry valid for reconstruction: true.

The default ICP render over the 5 to 25 second motion window estimated:

- selected scans for ICP: 40;
- ICP steps: 39;
- rejected ICP steps: 0;
- skipped oversized scans: 0;
- estimated path length: 0.469 m;
- estimated net displacement: 0.442 m;
- estimated net rotation: 3.67 degrees;
- map output points: 72,565.

An ICP timing-window sweep showed that using the whole capture increased the
estimate only to 0.525 m. Therefore the 5 to 25 second window is not the main
cause of the short trajectory estimate. The constrained run still undershot the
24-inch physical target of 0.6096 m.

Visual result: the ICP map remains less fan-shaped than the assumed-straight
map, but the central bands are still visibly smeared. The constrained setup did
not produce a more accurate metric path than the previous guided push.

Result: mixed. Pass for clean sensor capture and for confirming ICP remains
better than the assumed-straight renderer. Fail for the hypothesis that a
simple physical yaw constraint would move the ICP path estimate closer to the
24-inch ground truth.

Next action: add an independent motion measurement instead of relying on ICP
alone. The two smallest options are (1) visible start/end landmarks in the lidar
map with a measured separation, or (2) wheel odometry/encoders. For a no-new-
hardware test, place two or more opaque vertical reference boards at measured
positions and verify whether their mapped locations preserve the known distance.

### Reference-board Lidar Map Scale Check

Session `20260625T214456Z` repeated the reconstruction-candidate motion test
with two reference boards/box faces placed 24 inches apart as independent lidar
landmarks. Desktop validation was clean:

- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 226 scans over 28.716 seconds;
- lidar gap events above 1.5x nominal: 0;
- oversized lidar scans above 1.5x median returns: 0;
- lidar valid returns per scan min/median/max: 626/708/808;
- shared monotonic-clock overlap: 28.716 seconds;
- geometry valid for reconstruction: true.

The ICP trajectory for this session estimated:

- selected scans for ICP: 40;
- ICP steps: 39;
- rejected ICP steps: 0;
- skipped oversized scans: 0;
- estimated path length: 0.585 m;
- estimated net displacement: 0.565 m;
- estimated net rotation: 4.24 degrees.

The automatic line-candidate detector labeled large walls/furniture rather than
the small reference boards, so the manual measurement picker was used. The user
picked the center of the two visible board return clusters:

```text
A: (-0.162, 0.280) m
B: (-0.716, -0.003) m
Measured distance: 0.621 m
Delta vector: dx=-0.553 m, dy=-0.283 m
Expected distance: 0.610 m
Error: +0.012 m (+1.9%)
```

Result: pass for map scale. The measured board-to-board distance in the ICP map
is within about 2 percent of the tape-measured 24-inch separation (`0.6096 m`).
This strongly suggests that the reconstructed lidar map scale is basically
metric, while the accumulated ICP path-length estimate can still be biased low.

Next action: use reference landmarks as the scale/metric sanity check for room
captures. Do not over-correct map scale from ICP path length alone. The next
engineering improvement should either fuse wheel odometry with lidar ICP or
derive a more robust trajectory quality metric from landmark consistency.

### Camera-frame Pose Timeline from ICP Trajectory

The first camera/lidar fusion artifact was generated from the same clean
reference-board session, `20260625T214456Z`. This test associates Pi Camera v2
frame timestamps with the ICP-derived lidar trajectory using the shared
Raspberry Pi monotonic clock.

Command run on the Windows workstation:

```text
python reconstruction\render_camera_pose_timeline.py `
  "$HOME\Downloads\20260625T214456Z" `
  --trajectory "data\room-motion\20260625T214456Z-icp-trajectory.json" `
  --output "data\fusion\20260625T214456Z-camera-pose-timeline.svg" `
  --json-output "data\fusion\20260625T214456Z-camera-poses.json" `
  --sample-count 12 `
  --lidar-angle-offset-deg 125
```

Outputs:

- `data/fusion/20260625T214456Z-camera-pose-timeline.svg`;
- `data/fusion/20260625T214456Z-camera-poses.json`.

Measurements:

- camera frames available: 443;
- sampled camera poses: 12;
- first three sampled frames were clamped to the initial ICP pose because the
  useful ICP motion window starts after the initial still period;
- later sampled poses follow the estimated path from about `(-0.117, -0.017) m`
  at 8.065 seconds to about `(-0.553, -0.299) m` at the end of the trajectory;
- final frames were clamped to the final ICP pose after motion ended.

Result: pass for timestamp/pose association. This is not a 3D reconstruction
yet, but it proves that camera frames can be assigned metric 2D rig poses from
the lidar trajectory. The camera offset used in this artifact is still the rough
measured offset, not a finalized extrinsic calibration.

Next action: extract a small set of actual camera still frames at the sampled
indices, then render a contact sheet beside the top-down pose timeline so each
image can be inspected with its estimated capture pose.

### Camera Pose Contact Sheet

A follow-up workstation artifact was generated from the pose JSON above. Since
neither system `ffmpeg` nor OpenCV was available in the project Python
environment, the lightweight `imageio-ffmpeg` Python package was installed to
provide an ffmpeg executable for H.264 frame extraction.

Command run on the Windows workstation:

```text
python reconstruction\render_camera_pose_contact_sheet.py `
  "$HOME\Downloads\20260625T214456Z" `
  --pose-json "data\fusion\20260625T214456Z-camera-poses.json" `
  --output "data\fusion\20260625T214456Z-camera-pose-contact-sheet.svg" `
  --thumbnail-width 640 `
  --jpeg-quality 4
```

Outputs:

- `data/fusion/20260625T214456Z-camera-pose-contact-sheet.svg`;
- 12 extracted JPEG thumbnails in
  `data/fusion/20260625T214456Z-camera-samples/`.

Measurements:

- sampled frames extracted: 12;
- extracted frame indices: 0, 40, 80, 121, 161, 201, 241, 281, 321, 362,
  402, and 442;
- contact-sheet SVG size: about 208 KB;
- manual thumbnail inspection confirmed a valid decoded camera image.

Result: pass. Each sampled camera frame now has a visual thumbnail and an
estimated 2D camera pose from the ICP lidar trajectory. This gives us a
human-checkable fusion artifact before attempting any 3D textured geometry.

Next action: use the contact sheet to decide whether the camera view has enough
texture and overlap for visual feature tracking. If it does, estimate a
camera-only motion track and compare it against the lidar ICP trajectory.

### Camera-only Motion Compared with Lidar ICP

The sampled camera thumbnails from `20260625T214456Z` were used for a
monocular visual-odometry sanity check. OpenCV ORB features were matched between
neighboring sampled frames, an essential matrix was estimated for each pair, and
successful arbitrary-scale camera steps were similarity-aligned to the lidar ICP
camera poses. Because a monocular camera has no metric scale by itself, this
test checks feature health and motion-direction consistency rather than treating
the camera-only path as an independent metric measurement.

Command run on the Windows workstation:

```text
python reconstruction\compare_camera_lidar_motion.py `
  "$HOME\Downloads\20260625T214456Z" `
  --pose-json "data\fusion\20260625T214456Z-camera-poses.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\20260625T214456Z-camera-lidar-motion.svg" `
  --json-output "data\fusion\20260625T214456Z-camera-lidar-motion.json"
```

Outputs:

- `data/fusion/20260625T214456Z-camera-lidar-motion.svg`;
- `data/fusion/20260625T214456Z-camera-lidar-motion.json`.

Measurements:

- sampled frames: 12;
- neighboring frame pairs: 11;
- successful visual-motion pairs: 6;
- median successful-pair pose inliers: 270.5;
- moving-window alignment RMSE after arbitrary-scale similarity fit: 0.023 m;
- all-sample RMSE after applying that fit: 0.034 m;
- median moving-pair direction error: 9.1 degrees;
- maximum moving-pair direction error: 29.4 degrees;
- lidar sampled path length: 0.572 m;
- aligned camera sampled path length: 0.566 m.

The rejected pairs were mostly the initial/final still portions where lidar
motion was zero or near zero and monocular pose recovery had too little parallax.
The middle moving portion produced hundreds of ORB matches per pair and enough
essential-matrix inliers for a plausible direction comparison.

Result: pass for a first camera-motion sanity check. The Pi Camera v2 frames
contain enough texture and overlap in this room capture for sparse feature
tracking during the moving portion. This does not yet produce metric camera
odometry or 3D reconstruction, but it supports using camera feature tracks as a
diagnostic against lidar ICP.

Next action: decode a denser sequence over only the motion window, run
frame-to-frame visual odometry at shorter intervals, and compare the camera
heading changes against lidar ICP without including the stationary start/end
segments.

### Dense Camera Motion over the Steady-motion Window

The camera-only motion comparison was repeated using denser samples inside the
actual moving part of session `20260625T214456Z`, instead of sampling across the
stationary start and stop. A small sampling sweep showed that `7` to `20`
seconds with `9` camera samples gave the cleanest direction agreement without
including the start/stop wobble.

Command run on the Windows workstation:

```text
python reconstruction\render_camera_pose_timeline.py `
  "$HOME\Downloads\20260625T214456Z" `
  --trajectory "data\room-motion\20260625T214456Z-icp-trajectory.json" `
  --output "data\fusion\20260625T214456Z-camera-pose-timeline-motion-steady.svg" `
  --json-output "data\fusion\20260625T214456Z-camera-poses-motion-steady.json" `
  --sample-count 9 `
  --sample-start-s 7 `
  --sample-end-s 20 `
  --lidar-angle-offset-deg 125

python reconstruction\compare_camera_lidar_motion.py `
  "$HOME\Downloads\20260625T214456Z" `
  --pose-json "data\fusion\20260625T214456Z-camera-poses-motion-steady.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\20260625T214456Z-camera-lidar-motion-steady.svg" `
  --json-output "data\fusion\20260625T214456Z-camera-lidar-motion-steady.json" `
  --min-lidar-step-m 0.025
```

Outputs:

- `data/fusion/20260625T214456Z-camera-pose-timeline-motion-steady.svg`;
- `data/fusion/20260625T214456Z-camera-poses-motion-steady.json`;
- `data/fusion/20260625T214456Z-camera-lidar-motion-steady.svg`;
- `data/fusion/20260625T214456Z-camera-lidar-motion-steady.json`.

Measurements:

- sampled frames: 9, from frame 105 at 6.999 seconds to frame 300 at
  19.996 seconds;
- neighboring frame pairs: 8;
- successful visual-motion pairs: 7;
- median successful-pair pose inliers: 134;
- moving-window alignment RMSE after arbitrary-scale similarity fit: 0.009 m;
- all-sample RMSE after applying that fit: 0.020 m;
- median moving-pair direction error: 4.1 degrees;
- maximum moving-pair direction error: 5.4 degrees;
- lidar sampled path length: 0.475 m;
- aligned camera sampled path length: 0.436 m.

The only rejected pair was the last pair, frame 276 to frame 300, with 13 pose
inliers after essential-matrix recovery. All earlier steady-motion pairs passed
with 53 to 198 pose inliers and direction errors below 5.5 degrees.

Result: pass. Restricting the comparison to the steady-motion portion gives a
substantially cleaner camera/lidar direction check than sampling across the full
session. The camera trajectory remains arbitrary-scale monocular VO, but it
agrees well with the lidar ICP direction during the usable motion segment.

Next action: promote this steady-window comparison as the current validation
procedure for room captures. The next reconstruction step should use these
camera poses to back-project sparse tracked image features into a lidar-assisted
2.5D/3D diagnostic view, while keeping the lidar map as the metric anchor.

### Sparse Lidar-anchored Camera Feature Diagnostic

The steady-window camera poses from `20260625T214456Z` were used to triangulate
sparse camera feature matches into the lidar ICP map frame. This is the first
visual geometry artifact that combines camera image features, calibrated camera
intrinsics, rough camera-to-lidar rig measurements, and the metric lidar
trajectory. It is intentionally treated as a diagnostic sparse point cloud, not
as a final calibrated 3D reconstruction.

Command run on the Windows workstation:

```text
python reconstruction\render_sparse_fused_feature_map.py `
  "$HOME\Downloads\20260625T214456Z" `
  --pose-json "data\fusion\20260625T214456Z-camera-poses-motion-steady.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\20260625T214456Z-sparse-fused-feature-map.svg" `
  --json-output "data\fusion\20260625T214456Z-sparse-fused-feature-map.json" `
  --ply-output "data\fusion\20260625T214456Z-sparse-fused-feature-map.ply"
```

Outputs:

- `data/fusion/20260625T214456Z-sparse-fused-feature-map.svg`;
- `data/fusion/20260625T214456Z-sparse-fused-feature-map.json`;
- `data/fusion/20260625T214456Z-sparse-fused-feature-map.ply`.

Measurements:

- accepted sparse 3D points: 327;
- frame pairs with accepted points: 6 of 8;
- total ORB matches across pairs: 2,729;
- total pose inliers across pairs: 885;
- median reprojection error: 3.48 px;
- 95th-percentile reprojection error: 5.46 px;
- median triangulation angle: 1.56 degrees;
- median point range from camera: 1.07 m;
- point extents in the lidar ICP map frame:
  - x: +0.07 to +4.57 m;
  - y: -1.46 to +0.05 m;
  - z: +0.01 to +0.61 m above the lidar scan plane.

A zero-roll/zero-pitch comparison produced fewer accepted points (294) and a
higher median reprojection error (3.84 px), so the current rough rig config
(`camera_roll_deg=-2`, `camera_pitch_deg=-1`, `camera_height_m=0.0953`) was kept
for the main diagnostic output. Pair 8 was rejected because essential-matrix
pose recovery produced only 13 pose inliers. Pair 3 had enough pose inliers but
all triangulated points failed the metric/depth/reprojection filters.

Result: pass for a first sparse fused-geometry diagnostic, with important
limitations. The output demonstrates that tracked camera features can be
triangulated into a lidar-anchored metric frame for the steady-motion segment.
However, the point heights and positions still depend on rough extrinsics, so
this must not be treated as an accurate room model yet.

Next action: improve the extrinsic calibration target and repeat this diagnostic
on a scene with more deliberate visual landmarks at known heights/depths. Use
the PLY/SVG point cloud to check whether those landmarks appear in the expected
relative locations before attempting denser reconstruction.

### COLMAP-style Export for Downstream Gaussian-splatting Tests

The steady-window camera frames and lidar-anchored poses from
`20260625T214456Z` were exported as a COLMAP-style text model. This is a bridge
artifact for future photogrammetry or Gaussian-splatting experiments; it does
not mean COLMAP, NeRF, or Gaussian splatting has been run yet.

Command run on the Windows workstation:

```text
python reconstruction\export_colmap_camera_poses.py `
  "$HOME\Downloads\20260625T214456Z" `
  --pose-json "data\fusion\20260625T214456Z-camera-poses-motion-steady.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --points-json "data\fusion\20260625T214456Z-sparse-fused-feature-map.json" `
  --output-dir "data\exports\colmap\20260625T214456Z-steady-undistorted" `
  --undistort-images `
  --image-width 1920 `
  --jpeg-quality 95
```

Outputs:

- `data/exports/colmap/20260625T214456Z-steady-undistorted/images/`;
- `data/exports/colmap/20260625T214456Z-steady-undistorted/sparse/0/cameras.txt`;
- `data/exports/colmap/20260625T214456Z-steady-undistorted/sparse/0/images.txt`;
- `data/exports/colmap/20260625T214456Z-steady-undistorted/sparse/0/points3D.txt`;
- `data/exports/colmap/20260625T214456Z-steady-undistorted/export_manifest.json`.

Measurements:

- exported images: 9;
- image size: 1920x1080;
- images were undistorted before export;
- exported camera model: `PINHOLE`;
- sparse seed points exported: 327;
- validation result: pass;
- validation checks:
  - one camera record;
  - nine image pose records;
  - 327 point records;
  - no missing/empty image files;
  - quaternion norms exactly 1.0 within the exporter tolerance.

Result: pass for a COLMAP-style handoff artifact. The export has the folder
shape and text files expected by many COLMAP-based downstream tools:
`images/` plus `sparse/0/`. The important caveat is that these poses come from
lidar ICP and rough rig extrinsics, not from COLMAP bundle adjustment.

Next action: try importing this export into a downstream viewer or Gaussian
splatting pipeline. If the downstream tool rejects the text model or the
`PINHOLE` undistorted images, add a second export mode with raw images and a
`FULL_OPENCV` camera model.

### Local COLMAP Text-model Preview

The workstation did not have a `colmap` executable available on `PATH`, so the
first import test used a lightweight local parser for the exported COLMAP text
model. The parser reads `cameras.txt`, `images.txt`, and `points3D.txt`, rebuilds
camera centers from COLMAP `qvec`/`tvec`, checks image-file presence and
quaternion norms, and renders a preview SVG.

Command run on the Windows workstation:

```text
python reconstruction\render_colmap_export_preview.py `
  "data\exports\colmap\20260625T214456Z-steady-undistorted" `
  --output "data\exports\colmap\20260625T214456Z-steady-undistorted\preview.svg" `
  --json-output "data\exports\colmap\20260625T214456Z-steady-undistorted\preview_validation.json"
```

Outputs:

- `data/exports/colmap/20260625T214456Z-steady-undistorted/preview.svg`;
- `data/exports/colmap/20260625T214456Z-steady-undistorted/preview_validation.json`.

Measurements:

- validation result: pass;
- parsed cameras/images/points: 1 / 9 / 327;
- missing images: 0;
- reconstructed camera path length from COLMAP poses: 0.475 m;
- quaternion norm range: 0.9999999999995166 to 1.0000000000004476;
- median point reprojection/error field from `points3D.txt`: 3.48 px;
- point bounds:
  - x: +0.07 to +4.57 m;
  - y: -1.46 to +0.05 m;
  - z: +0.01 to +0.61 m;
- camera-center bounds:
  - x: -0.50 to -0.10 m;
  - y: -0.26 to -0.01 m;
  - z: +0.0953 m.

Result: pass for local text-model import and preview. The export is internally
coherent when parsed independently from the exporter, and the recovered camera
path length matches the steady-window lidar/camera pose path. This still does
not prove compatibility with a real COLMAP binary database or a Gaussian
splatting trainer.

Next action: either install/run real COLMAP to convert the text model to binary,
or choose a Gaussian-splatting implementation and inspect its expected input
format before adapting the export.

### GraphDECO Gaussian-splatting Input Check and WSL COLMAP Conversion

The official GraphDECO/Inria 3D Gaussian Splatting repository was inspected as
the first downstream target. Its README trains from a source path containing a
COLMAP or NeRF Synthetic dataset. The source loader first looks for binary
COLMAP files in `sparse/0`, then falls back to text `images.txt` and
`cameras.txt`; its text intrinsics reader asserts `PINHOLE`, matching the
current undistorted export.

The user's Start Menu shortcut was resolved to:

```text
C:\Program Files\WSL\wslg.exe -d Ubuntu-22.04 --cd "~" -- colmap gui
```

Running COLMAP from WSL confirmed:

- COLMAP version: 3.7;
- CUDA: not available in that WSL build;
- `model_converter` and `model_analyzer` commands are available.

Real COLMAP conversion of the export passed. The text model in
`data/exports/colmap/20260625T214456Z-steady-undistorted/sparse/0` was converted
to binary with `colmap model_converter --output_type BIN`, producing:

- `cameras.bin`: 64 bytes;
- `images.bin`: 881 bytes;
- `points3D.bin`: 16,685 bytes.

The binary files were copied into `sparse/0` beside the text files so downstream
tools can use either format. `colmap model_analyzer` on `sparse/0` reported:

```text
Cameras: 1
Images: 9
Registered images: 9
Points: 327
Observations: 0
Mean track length: 0.000000
Mean observations per image: 0.000000
Mean reprojection error: 3.371872px
```

Result: pass for real COLMAP text-to-binary compatibility and pass for
GraphDECO input-shape compatibility. Caveat: the model has zero COLMAP feature
observations because the sparse seed points came from the project diagnostic
triangulator and are not linked to `images.txt` keypoint tracks. This is likely
acceptable for a GraphDECO loader/training smoke test, but it is not a complete
COLMAP SfM reconstruction.

Next action: run a minimal GraphDECO training smoke test from this export folder
in a separate environment. Expect the first useful outcome to be either "loader
accepts the dataset and starts optimization" or a concrete loader error that
defines the next adapter change.

### GraphDECO Loader-readiness Smoke Test

A local GraphDECO input checker was added for the current COLMAP-style export.
This is not Gaussian-splat training. It validates the folder shape and camera
model assumptions used by the official GraphDECO loader and writes
`sparse/0/points3D.ply` from `points3D.txt` with the vertex fields GraphDECO
expects on first load.

Command:

```text
python reconstruction/check_graphdeco_input.py \
  data/exports/colmap/20260625T214456Z-steady-undistorted \
  --json-output data/exports/colmap/20260625T214456Z-steady-undistorted/graphdeco_input_check.json
```

Artifacts:

- `reconstruction/check_graphdeco_input.py`;
- `data/exports/colmap/20260625T214456Z-steady-undistorted/graphdeco_input_check.json`;
- `data/exports/colmap/20260625T214456Z-steady-undistorted/sparse/0/points3D.ply`.

Measurements:

- GraphDECO input readiness: pass;
- camera model: `PINHOLE`;
- cameras: 1;
- image references: 9;
- missing images: 0;
- sparse seed points: 327;
- `points3D.ply` vertices in header/rows: 327/327;
- `points3D.ply` fields: `x`, `y`, `z`, `nx`, `ny`, `nz`, `red`, `green`,
  `blue`;
- COLMAP feature-track references: 0.

Result: pass for a local GraphDECO loader-readiness smoke test. The current
export has the expected `images/`, `sparse/0`, `PINHOLE` intrinsics, binary
COLMAP files, text COLMAP files, and GraphDECO-compatible `points3D.ply`.
Training was not run because no existing GraphDECO install was found in the
likely Windows/WSL locations and CUDA was not visible through `nvidia-smi` on
Windows or WSL.

Caveat: zero COLMAP feature-track references remains the main quality risk. The
artifact is ready for a loader/training smoke test in a GraphDECO environment,
but a useful splat will probably need more views and/or true visual feature
tracks.

Next action: set up or locate a GraphDECO Gaussian Splatting environment, then
run the smallest training command against this export. If training starts, judge
only loader compatibility first; visual quality is a separate capture and pose
quality problem.

### GraphDECO GPU Notebook and Dataset Package

A GPU-notebook smoke-test path was added because the Windows/WSL workstation did
not expose `nvidia-smi`, while GraphDECO training requires CUDA-capable PyTorch
extensions. The target runtime for the first live training test is Google Colab
or equivalent with a T4 GPU. A T4 is acceptable for this tiny 9-image, low
resolution, 300-iteration smoke test; it is not a final-quality room training
target.

Added files:

- `reconstruction/package_graphdeco_dataset.py`;
- `notebooks/GraphDECO_3DGS_Smoke_Test.ipynb`;
- `docs/GRAPHDECO_GPU_SMOKE_TEST.md`.

Local checks:

```text
python -m py_compile reconstruction/package_graphdeco_dataset.py reconstruction/check_graphdeco_input.py
Get-Content -Raw notebooks/GraphDECO_3DGS_Smoke_Test.ipynb | ConvertFrom-Json
```

Packaging command:

```text
python reconstruction/package_graphdeco_dataset.py \
  data/exports/colmap/20260625T214456Z-steady-undistorted \
  --output data/exports/gaussian-splatting/20260625T214456Z-steady-undistorted-graphdeco.zip
```

Generated artifacts:

- `data/exports/gaussian-splatting/20260625T214456Z-steady-undistorted-graphdeco.zip`;
- `data/exports/gaussian-splatting/20260625T214456Z-steady-undistorted-graphdeco.package_manifest.json`.

Measurements:

- package files: 20;
- package size: 1,764,364 bytes;
- image files: 9;
- missing image references: 0;
- sparse seed points: 327;
- COLMAP feature-track references: 0;
- included loader inputs: `images/`, `sparse/0/cameras.bin`,
  `sparse/0/images.bin`, `sparse/0/points3D.bin`, and
  `sparse/0/points3D.ply`.

Result: pass for notebook preparation and dataset packaging. Actual GraphDECO
training has not yet been run. The next hardware/software dependency is a Colab
or other CUDA runtime.

Next action: open `notebooks/GraphDECO_3DGS_Smoke_Test.ipynb` in Colab, select a
T4 GPU, upload the generated ZIP, and run all cells until either the trainer
reaches iteration 300 or produces a concrete loader/build error.

### Colab T4 GraphDECO First Install Attempt

The user ran the first notebook on Google Colab with a T4 GPU. Repository clone
and recursive submodule checkout succeeded. The first install command failed
while pip was preparing the `diff-gaussian-rasterization` wheel:

```text
Getting requirements to build wheel did not run successfully.
Failed to build 'file:///content/gaussian-splatting/submodules/diff-gaussian-rasterization'
```

The user then ran the training cell, but `train.py` exited with status 1 because
the required CUDA rasterizer extension had not been installed. This did not test
the project dataset yet.

Result: fail for GraphDECO environment setup, not for lidar/camera export
format. The likely cause is pip build isolation hiding Colab's installed
`torch` from a GraphDECO CUDA submodule whose `setup.py` imports
`torch.utils.cpp_extension`.

Notebook update:

- install CUDA submodules one at a time;
- use `--no-build-isolation` for `diff-gaussian-rasterization`, `simple-knn`,
  and optional `fused-ssim`;
- print PyTorch/CUDA and `nvcc` versions before compiling;
- pass `--disable_viewer` to `train.py` for notebook runs.

An equivalent uploadable Colab Python runner was added:

- `notebooks/graphdeco_3dgs_smoke_colab.py`.

It performs CUDA diagnostics, prompts for the packaged dataset ZIP, clones
GraphDECO, installs CUDA submodules using the patched build path, validates the
dataset folder, runs the 300-iteration training smoke test, and zips/downloads
the output directory.

After the user reported difficulty uploading the `.py` file to Colab, a
Colab-native notebook was added as the recommended path:

- `notebooks/GraphDECO_3DGS_Colab_T4_Smoke.ipynb`.

This notebook avoids `.py` upload entirely and keeps the same fixed install
logic, dataset validation, `--disable_viewer`, and 300-iteration smoke test.

The first T4 run of the new notebook confirmed CUDA visibility:

```text
torch 2.11.0+cu128
torch cuda 12.8
cuda available True
device Tesla T4
```

but failed inside the install cell with:

```text
NameError: name 'PY' is not defined
```

This was caused by using a shell heredoc (`python - <<'PY'`) inside a Jupyter
cell. Colab split the heredoc body across notebook execution contexts, so the
terminator `PY` was interpreted as Python code. Both GraphDECO Colab notebooks
were patched to remove heredocs and run the PyTorch/CUDA diagnostics as normal
Python cell lines. The install cell now also pins `setuptools<82` to satisfy the
current Colab Torch runtime constraint.

Next action: rerun the patched install cell in Colab from
`/content/gaussian-splatting`, then rerun the 300-iteration training cell. If it
still fails, preserve the verbose build output from the first failing extension.

### Colab T4 GraphDECO CUDA Extension Build Passed

The user reran the patched Colab install cell on a T4 runtime. CUDA and PyTorch
were visible:

```text
torch 2.11.0+cu128
torch cuda 12.8
cuda available True
device Tesla T4
nvcc release 12.8, V12.8.93
```

The GraphDECO CUDA submodules built and installed:

- `diff_gaussian_rasterization-0.0.0`;
- `simple_knn-0.0.0`;
- `fused_ssim-0.0.0`.

The remaining pip resolver message about `ipython`/`jedi` is treated as
non-blocking because the CUDA extension builds completed successfully.

Result: pass for GraphDECO environment setup on Colab T4. This still has not
tested the Fuse dataset or training loop; the next cells should unpack/validate
the uploaded dataset ZIP and run the 300-iteration training smoke test.

Next action: run the dataset validation cell, then the training cell. Success is
defined as `train.py` reaching iteration 300 and writing the output model
directory.

### Colab T4 GraphDECO 300-iteration Smoke Test Passed

The user ran the remaining Colab cells after the CUDA extension build succeeded.
Cell 6 downloaded the GraphDECO output archive. The downloaded archive was found
locally at:

```text
C:\Users\Neel\Downloads\20260625T214456Z-steady-undistorted-graphdeco-smoke.zip
```

It was copied into the ignored project data directory:

```text
data/exports/gaussian-splatting/20260625T214456Z-steady-undistorted-graphdeco-smoke.zip
```

Local ZIP inspection found:

- files in archive: 8;
- `cameras.json`: present;
- camera count: 9;
- `input.ply`: present;
- `input.ply` vertices: 327;
- `point_cloud/iteration_300/point_cloud.ply`: present;
- iteration-300 point-cloud vertices: 327;
- TensorBoard event file: present.

Result: pass for the first end-to-end GraphDECO Gaussian Splatting smoke test.
The official trainer accepted the exported COLMAP-style dataset, ran to
iteration 300 on a Colab T4 GPU, and produced a model output archive.

Caveat: this pass proves compatibility of the handoff path, not reconstruction
quality. The output still has 327 points at iteration 300, matching the sparse
input seed count, so this tiny 9-view diagnostic dataset should not be judged as
a useful room model.

Next action: either inspect the iteration-300 `point_cloud.ply` visually as a
sanity check, or collect a more splatting-friendly dataset with more views,
texture, and stronger camera poses before trying longer training.

### Next 3DGS Capture Plan

A follow-up runbook was added for a better Gaussian-splatting dataset:

- `docs/NEXT_3DGS_CAPTURE.md`.

The next experiment should use a 75-second `reconstruction_candidate` capture
with a deliberately textured scene, slow 18 to 24 inch motion, and 30 sampled
camera poses from the moving window. The first success target remains a
300-iteration GraphDECO Colab T4 smoke pass, but the expected quality should be
better than the 9-view diagnostic dataset.

The GraphDECO Colab notebooks were also relaxed to accept either binary or text
COLMAP sparse model files. This matches GraphDECO's fallback behavior and avoids
requiring a WSL COLMAP conversion before every new dataset package.

Next action: perform the new capture using the runbook, validate the session,
export `SESSION_ID-30-undistorted`, package it, and rerun the Colab smoke test.

### 30-view 3DGS Candidate Capture and Export

Session `20260626T010718Z` was captured with the 75-second reconstruction
candidate protocol and copied to the Windows workstation. Validation reported:

- camera frames: 1118 over 74.456 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar scans: 563 over 73.695 seconds;
- lidar gap events above 1.5x nominal: 1;
- oversized lidar scans: 1;
- shared monotonic-clock overlap: 73.257 seconds;
- geometry valid for reconstruction: true.

The one lidar issue was localized to scan 333:

```text
gap after scan 333: 262.245 ms
scan 333: 1473 returns
```

ICP was run over the 5 to 70 second motion window. It skipped the oversized scan
and produced:

- selected scans for ICP: 125;
- ICP steps: 124;
- rejected ICP steps: 0;
- skipped oversized scans: 1;
- estimated path length: 0.684 m;
- estimated net displacement: 0.610 m;
- estimated net rotation: 0.60 degrees;
- map output points: 186,936.

Thirty camera poses were sampled from 5 to 70 seconds. The last several samples
were nearly stationary near the end of the move, but the overall visual-motion
diagnostic was strong:

- successful visual-motion pairs: 23/29;
- median pose inliers: 246;
- moving alignment RMSE: 0.031 m;
- median direction error: 12.1 degrees;
- selected camera candidate: `z_forward_x_right`.

Sparse fused feature map:

- accepted sparse 3D points: 1221;
- pairs with accepted points: 8/29;
- median reprojection error: 2.96 px;
- median triangulation angle: 1.51 degrees;
- point extents: x -0.17..+4.08 m, y -1.73..-0.05 m, z -0.15..+0.50 m.

COLMAP-style export:

```text
data/exports/colmap/20260626T010718Z-30-undistorted
```

Export measurements:

- images: 30 at 1920x1080;
- camera model: `PINHOLE`;
- sparse points: 1221;
- export validation: pass;
- preview validation: pass;
- camera path length from exported poses: 0.643 m;
- missing images: 0.

GraphDECO package:

```text
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-graphdeco.zip
```

Package measurements:

- files: 38;
- package size: 6,945,487 bytes;
- image references: 30;
- missing images: 0;
- sparse points: 1221;
- COLMAP track references: 0.

Result: pass for capture, lidar ICP, camera pose sampling, visual-motion
diagnostic, sparse feature map, COLMAP-style export, and GraphDECO input
packaging. This is a materially better splatting candidate than the first
9-view diagnostic dataset.

Caveats: the dataset still has zero COLMAP feature-track references, and the
last several camera samples are nearly stationary. If the Colab output remains
weak, resample the moving window earlier, for example 5 to 54 seconds, or
capture with steadier non-stop motion.

Next action: upload
`data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-graphdeco.zip`
to the Colab T4 notebook and run the 300-iteration smoke test.

### 30-view GraphDECO Colab Smoke Test Passed

The 30-view candidate package was uploaded to the Colab T4 notebook and trained
for 300 iterations. Cell 6 downloaded:

```text
C:\Users\Neel\Downloads\20260626T010718Z-30-undistorted-graphdeco (2)-smoke.zip
```

The archive was copied into the ignored project data directory as:

```text
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-graphdeco-smoke.zip
```

Local ZIP inspection found:

- `cameras.json`: present;
- camera count: 30;
- `input.ply`: present;
- input vertices: 1221;
- `point_cloud/iteration_300/point_cloud.ply`: present;
- iteration-300 vertices: 1221;
- iteration-300 PLY size: 304,337 bytes;
- TensorBoard event file: present.

Compared with the first 9-view smoke test:

- cameras: 9 -> 30;
- input vertices: 327 -> 1221;
- iteration-300 vertices: 327 -> 1221;
- iteration-300 PLY size: 82,624 -> 304,337 bytes.

Result: pass for the improved 30-view GraphDECO smoke test. The export/training
path is repeatable, and the candidate dataset is materially richer than the
first diagnostic dataset.

Caveat: 300 iterations is still a compatibility/early-output test. The point
count remaining equal to the input seed count is expected this early and does
not by itself indicate visual quality. The next decision should be based on a
visual inspection of the output point cloud or a longer training run.

Next action: extract and inspect
`point_cloud/iteration_300/point_cloud.ply`; if the orientation and gross
structure are plausible, run a longer Colab training test, for example 3,000
iterations, on the same packaged dataset.

### 30-view GraphDECO Point-cloud Preview

The 300-iteration GraphDECO output archive for `20260626T010718Z` was extracted
locally and inspected with a lightweight diagnostic renderer. This renderer reads
ASCII COLMAP seed PLY files and binary GraphDECO Gaussian PLY files, but it is
not a full Gaussian splat renderer.

Added tool:

```text
reconstruction/render_graphdeco_point_cloud_preview.py
```

Generated ignored preview artifacts:

```text
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-input-preview.png
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-iter300-preview.png
```

Seed `input.ply` measurements:

- vertices: 1221;
- bounds: `x=-0.167..+4.085 m`, `y=-1.733..-0.050 m`,
  `z=-0.153..+0.495 m`;
- median opacity used for preview: 0.82.

Iteration-300 GraphDECO PLY measurements:

- vertices: 1221;
- bounds: `x=-0.159..+4.080 m`, `y=-1.736..-0.052 m`,
  `z=-0.170..+0.498 m`;
- opacity min/median/max after sigmoid and clamping: `0.120/0.120/0.950`.

Visual result: pass for gross-structure sanity. The input seed and
iteration-300 output both show a coherent elongated structure instead of random
noise, and the iteration-300 bounds remain close to the seed bounds. The
300-iteration result is still very early: point count did not densify yet and
the median opacity is low, so quality should not be judged from this run alone.

Next action: run a longer Colab T4 training test on the same 30-view dataset,
starting with 3,000 iterations. Use `--resolution 4` if memory allows, and fall
back to `--resolution 8` if Colab reports a CUDA out-of-memory error.

### 30-view GraphDECO 3,000-iteration Training Preview

The same 30-view package was trained in Colab T4 for 3,000 iterations at
`--resolution 4`. The user reported successful completion and downloaded:

```text
/content/fuse_3dgs_output/20260626T010718Z-30-undistorted-graphdeco (2)-iter3000.zip
```

The archive was copied locally as:

```text
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-graphdeco-iter3000.zip
```

The full iteration-3000 PLY preview measured:

- vertices: 69,321, up from the 1,221-point seed;
- bounds: `x=-0.416..+5.462 m`, `y=-3.179..+3.682 m`,
  `z=-3.709..+2.099 m`;
- opacity min/median/max after display clamping:
  `0.120/0.175/0.950`.

The large `y` and `z` bounds indicate floaters/outliers, so the preview tool was
extended with optional opacity and central-percentile filtering. A stricter core
preview using `--min-opacity 0.3 --central-percentile 98` measured:

- core preview vertices: 27,860 of 69,321;
- core bounds: `x=-0.264..+4.975 m`, `y=-1.643..+0.304 m`,
  `z=-0.477..+0.292 m`;
- core opacity min/median/max: `0.300/0.939/0.950`.

Result: pass for 3DGS densification and plausible high-confidence core
structure. The model is not yet judged as visually good. The unfiltered output
contains substantial floaters, and the current diagnostic preview is not a true
Gaussian renderer.

Next action: render train-view images from the GraphDECO model in Colab and
compare them to the corresponding camera frames. If rendered views resemble the
room, proceed to a longer/better capture. If they are smeared or misregistered,
improve camera pose sampling and/or scene texture before longer training.

### 30-view GraphDECO 3,000-iteration Render Comparison

GraphDECO `render.py` was run in Colab for the 3,000-iteration model and
returned a train-view render archive:

```text
C:\Users\Neel\Downloads\20260626T010718Z-30-undistorted-graphdeco (2)-iter3000-train-renders.zip
```

The archive contained:

- `ours_3000/gt`: 30 ground-truth train frames;
- `ours_3000/renders`: 30 rendered train frames.

A local comparison helper was added:

```text
reconstruction/compare_graphdeco_renders.py
```

It creates a side-by-side contact sheet with ground truth, render, and amplified
difference images, plus a JSON metrics summary. For this run:

- frame count: 30;
- mean/median MAE: `12.775/10.128` image gray levels;
- mean/median PSNR: `21.025/21.609 dB`;
- best frame by PSNR: `00007.png`, PSNR `29.440 dB`, MAE `4.456`;
- worst frame by PSNR: `00022.png`, PSNR `12.313 dB`, MAE `42.042`.

Visual result: pass for the first rendered-view reconstruction sanity check. The
renders are blurry and contain ghosting, but they visibly resemble the real room
and recover the main training-view objects. The quality is uneven across the
camera sequence, with some middle frames substantially better than edge/late
frames.

Result: the lidar-anchored camera pose export is good enough to train a
recognizable 3DGS model on training views. It is not yet a clean or complete
room reconstruction, and novel-view quality has not been validated.

Next action: run one longer `--resolution 4` training attempt, for example
7,000 iterations, then render train views again and compare metrics. If the
same frames remain smeared, prioritize a better capture and pose window rather
than simply training longer.

### 30-view GraphDECO 7,000-iteration Render Comparison

The same 30-view package was trained in Colab T4 for 7,000 iterations at
`--resolution 4`, then train views were rendered with GraphDECO `render.py`.
The downloaded archives were copied into the ignored project data directory as:

```text
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-graphdeco-iter7000.zip
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-graphdeco-iter7000-train-renders.zip
```

Iteration-7000 PLY preview:

- vertices: 167,138, up from 69,321 at iteration 3,000 and 1,221 in the seed;
- full bounds: `x=-0.388..+4.412 m`, `y=-1.758..+1.529 m`,
  `z=-0.737..+1.014 m`;
- full opacity min/median/max: `0.120/0.120/0.950`.

Filtered core preview using `--min-opacity 0.3 --central-percentile 98`:

- core preview vertices: 25,813 of 167,138;
- core bounds: `x=-0.353..+3.317 m`, `y=-1.185..+0.271 m`,
  `z=-0.386..+0.205 m`;
- core opacity min/median/max: `0.300/0.462/0.950`.

Rendered train-view comparison:

- frame count: 30;
- mean/median MAE: `7.617/3.953` image gray levels;
- mean/median PSNR: `27.672/28.085 dB`;
- best frame by PSNR: `00016.png`, PSNR `37.914 dB`, MAE `1.777`;
- worst frame by PSNR: `00022.png`, PSNR `11.941 dB`, MAE `44.797`.

Compared with the 3,000-iteration run:

- median MAE improved from `10.128` to `3.953`;
- median PSNR improved from `21.609 dB` to `28.085 dB`;
- visual renders are substantially sharper and more faithful for the middle of
  the sequence;
- frame `00022.png` remains poor, so some part of the capture or pose sequence
  is still unreliable.

Result: pass for a recognizable, materially improved training-view 3DGS
reconstruction from lidar-anchored camera poses. This is still not a validated
novel-view reconstruction because all rendered views used here were also
training views.

Next action: run a held-out-view validation in Colab by training with
`--eval`, then render both train and test views. In the current GraphDECO
COLMAP loader this uses the LLFF holdout path with its default hold interval.
Test-view quality, not train-view quality, should decide whether this capture is
good enough for the room MVP.

### 30-view GraphDECO 7,000-iteration Held-out Validation

The 30-view package was retrained in Colab with GraphDECO `--eval` at
7,000 iterations and `--resolution 4`. The downloaded model and render bundles
were copied into the ignored project data directory as:

```text
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-graphdeco-eval-iter7000.zip
data/exports/gaussian-splatting/20260626T010718Z-30-undistorted-graphdeco-eval-iter7000-render-views.zip
```

The eval render bundle contained 26 train views and 4 held-out test views.

Train-view comparison:

- frame count: 26;
- mean/median MAE: `8.332/6.405` image gray levels;
- mean/median PSNR: `25.349/23.075 dB`.

Held-out test-view comparison:

- frame count: 4;
- mean/median MAE: `26.726/25.102` image gray levels;
- mean/median PSNR: `15.700/15.062 dB`.

The held-out contact sheet showed that one held-out view was passable, but the
others had major ghosting and warped objects. The eval model point cloud did not
catastrophically diverge:

- vertices: 177,875;
- full bounds: `x=-0.384..+4.613 m`, `y=-1.731..+1.409 m`,
  `z=-0.897..+0.980 m`;
- filtered core vertices: 25,479 of 177,875;
- filtered core bounds: `x=-0.311..+4.043 m`, `y=-1.128..+0.307 m`,
  `z=-0.375..+0.220 m`.

Result: fail for robust novel/held-out view synthesis, but pass as a useful
diagnostic. The model can reproduce many training views, yet generalizes poorly
between the sparse 30 selected views. This points to insufficient/even view
coverage and remaining pose-window issues, not a total training crash.

Next action: before collecting new hardware data, re-export the same raw session
with denser camera sampling from the actual moving window and rerun held-out
validation.

### 48-view Stable-window GraphDECO Re-export

The original 30-view export included a nearly stationary tail: samples 24 through
30 moved only about 1 to 6 mm per step. A software-only re-export was generated
from the same raw session using 48 samples between 9 and 52 seconds, avoiding
the initial edge frame and the stationary tail.

Commands generated these ignored artifacts:

```text
data/fusion/20260626T010718Z-camera-poses-48stable.json
data/fusion/20260626T010718Z-camera-lidar-motion-48stable.json
data/fusion/20260626T010718Z-sparse-fused-feature-map-48stable.json
data/exports/colmap/20260626T010718Z-48stable-undistorted
data/exports/gaussian-splatting/20260626T010718Z-48stable-undistorted-graphdeco.zip
```

Pose/motion check:

- sampled camera frames: 48;
- sample window: 8.999 to 51.993 seconds;
- successful visual-motion pairs: 39 of 47;
- median pose inliers: 148;
- moving alignment RMSE: `0.030 m`;
- median direction error: `18.0 deg`.

Sparse fused feature map:

- accepted sparse 3D points: 1,454;
- pairs with accepted points: 17 of 47;
- median reprojection error: `2.72 px`;
- median triangulation angle: `1.04 deg`;
- point extents: `x=-0.38..+2.00 m`, `y=-0.97..+0.02 m`,
  `z=-0.09..+0.33 m`.

GraphDECO package check:

- camera images: 48 at 1920x1080;
- camera model: `PINHOLE`;
- sparse points: 1,454;
- missing images: 0;
- ZIP size: 10,633,316 bytes;
- expected warning remains: zero COLMAP feature-track references.

Result: pass for a denser, cleaner, stable-window dataset package. This is the
next best Colab candidate because it targets the actual held-out failure mode
without requiring another Pi capture.

Next action: upload
`data/exports/gaussian-splatting/20260626T010718Z-48stable-undistorted-graphdeco.zip`
to Colab, train with `--eval` at 7,000 iterations, render train/test views, and
compare held-out metrics against the 30-view eval baseline.

### 48-view Stable-window GraphDECO Held-out Validation

The 48-stable package was uploaded to Colab and trained with GraphDECO `--eval`
for 7,000 iterations at `--resolution 4`. The downloaded bundles were copied
into the ignored project data directory as:

```text
data/exports/gaussian-splatting/20260626T010718Z-48stable-undistorted-graphdeco-eval-iter7000.zip
data/exports/gaussian-splatting/20260626T010718Z-48stable-undistorted-graphdeco-eval-iter7000-render-views.zip
```

The eval render bundle contained 42 train views and 6 held-out test views.

Train-view comparison:

- frame count: 42;
- mean/median MAE: `4.537/3.960` image gray levels;
- mean/median PSNR: `30.576/30.227 dB`;
- best frame: `00038.png`, PSNR `42.323 dB`, MAE `1.223`;
- worst frame: `00005.png`, PSNR `21.532 dB`, MAE `11.159`.

Held-out test-view comparison:

- frame count: 6;
- mean/median MAE: `19.173/19.214` image gray levels;
- mean/median PSNR: `17.125/16.887 dB`;
- best frame: `00004.png`, PSNR `20.025 dB`, MAE `12.570`;
- worst frame: `00000.png`, PSNR `15.244 dB`, MAE `25.579`.

Compared with the 30-view eval baseline:

- train median MAE improved from `6.405` to `3.960`;
- train median PSNR improved from `23.075 dB` to `30.227 dB`;
- held-out median MAE improved from `25.102` to `19.214`;
- held-out median PSNR improved from `15.062 dB` to `16.887 dB`.

The 48-stable model point cloud was stable and compact:

- vertices: 195,146;
- full bounds: `x=-0.417..+2.358 m`, `y=-1.259..+0.354 m`,
  `z=-0.277..+0.665 m`;
- filtered core vertices: 23,710 of 195,146;
- filtered core bounds: `x=-0.398..+0.850 m`, `y=-0.369..+0.052 m`,
  `z=+0.010..+0.140 m`.

Visual result: the 48-stable train views are much cleaner than the 30-view eval
run. Held-out views are visibly improved, but still show ghosting and soft/warped
objects. The model is learning the scene and the denser stable window helped,
but it still does not generalize cleanly enough for a robust room MVP.

Result: partial pass. Software-side resampling improved the model, but the
remaining limitation is capture geometry and pose quality. More training alone
is unlikely to fix the held-out failure; the next high-value step is a new
capture with better parallax and more even view coverage.

Next action: collect a new reconstruction candidate capture with a deliberate
sideways arc or two-lane pass around the target area, keeping objects visible
throughout the motion and avoiding a stationary tail. Export 48 to 72 stable
views and repeat the `--eval` held-out validation.

### Room MVP Capture `20260626T030750Z` and 60-view Package

Session `20260626T030750Z` was captured using the next room MVP protocol and
copied to the Windows workstation. Validation passed:

- mode: `reconstruction_candidate`;
- camera: 1,343 frames over 89.449 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 683 scans over 88.718 seconds;
- lidar gap events above 1.5x nominal: 0;
- lidar valid returns per scan min/median/max: 645/723/809;
- oversized lidar scans: 0;
- shared monotonic-clock overlap: 88.359 seconds;
- geometry valid for reconstruction: true.

ICP trajectory reconstruction:

- selected scans for ICP: 145;
- ICP steps/rejected steps: 144/0;
- skipped oversized scans: 0;
- estimated path length: 0.918 m;
- estimated net displacement: 0.581 m;
- estimated net rotation: 5.07 degrees;
- map output points: 218,708.

The 60-stable camera pose export used frames from 7.998 to 74.985 seconds. The
trajectory shows a more deliberate curved path than the prior capture, with
about 0.45 m net camera displacement across the sampled window and no stationary
tail.

Sparse fused feature map:

- accepted sparse 3D points: 2,181;
- pairs with accepted points: 16 of 59;
- median reprojection error: 1.43 px;
- median triangulation angle: 5.70 degrees;
- point extents: `x=-0.40..+3.57 m`, `y=-0.32..+1.73 m`,
  `z=+0.01..+0.30 m`.

This is a better sparse geometry input than the previous 48-stable package:
more seed points and much stronger triangulation angle. The pair contribution
rate remains low, but the accepted pairs are geometrically more useful.

GraphDECO package:

```text
data/exports/gaussian-splatting/20260626T030750Z-60stable-undistorted-graphdeco.zip
```

Package/check results:

- images: 60 at 1920x1080;
- camera model: `PINHOLE`;
- sparse points: 2,181;
- missing images: 0;
- ZIP size: 12,914,510 bytes;
- GraphDECO input readiness: pass;
- expected warning remains: zero COLMAP feature-track references.

Result: pass for capture quality, trajectory reconstruction, 60-view export, and
GraphDECO packaging. This is the strongest candidate package so far for
held-out 3DGS validation.

Next action: upload
`data/exports/gaussian-splatting/20260626T030750Z-60stable-undistorted-graphdeco.zip`
to Colab, train with `--eval` at 7,000 iterations, render train/test views, and
compare held-out metrics against the 48-stable baseline.

### Room MVP Capture `20260626T030750Z` 60-view Held-out Validation

The 60-stable package was uploaded to Colab and trained with GraphDECO `--eval`
for 7,000 iterations at `--resolution 4`. The downloaded bundles were copied
into the ignored project data directory as:

```text
data/exports/gaussian-splatting/20260626T030750Z-60stable-undistorted-graphdeco-eval-iter7000.zip
data/exports/gaussian-splatting/20260626T030750Z-60stable-undistorted-graphdeco-eval-iter7000-render-views.zip
```

The eval render bundle contained 52 train views and 8 held-out test views.

Train-view comparison:

- frame count: 52;
- mean/median MAE: `6.211/5.543` image gray levels;
- mean/median PSNR: `27.303/26.959 dB`;
- best frame: `00047.png`, PSNR `37.880 dB`, MAE `1.852`;
- worst frame: `00019.png`, PSNR `18.296 dB`, MAE `16.428`.

Held-out test-view comparison:

- frame count: 8;
- mean/median MAE: `21.992/19.890` image gray levels;
- mean/median PSNR: `16.796/16.905 dB`;
- best frame: `00000.png`, PSNR `21.214 dB`, MAE `10.907`;
- worst frame: `00003.png`, PSNR `13.334 dB`, MAE `36.930`.

Compared with the previous 48-stable eval baseline:

- train median MAE worsened from `3.960` to `5.543`;
- train median PSNR worsened from `30.227 dB` to `26.959 dB`;
- held-out median MAE worsened slightly from `19.214` to `19.890`;
- held-out median PSNR was essentially unchanged, `16.887 dB` to `16.905 dB`.

Visual result: one held-out frame is decent, but several still show heavy
ghosting, duplicated objects, and warped foreground/background alignment. The
new capture improved raw parallax and sparse triangulation, but it did not
produce cleaner held-out 3DGS views.

Result: fail for the room-MVP held-out gate. The best explanation is no longer
just "not enough parallax"; the remaining bottleneck is likely pose quality and
trajectory consistency from the 2D lidar ICP path, especially through the curved
sections. More Gaussian-splatting training alone is unlikely to solve this.

Next action: inspect camera/lidar motion diagnostics for the 60-view sequence
and either narrow the export to the best monotonic segment, or add a visual pose
refinement path before the next Colab run.

### Room MVP Capture `20260626T030750Z` Camera/Lidar Motion Diagnostic

The missing camera-vs-lidar motion diagnostic was run for the 60-stable pose
set:

```text
data/fusion/20260626T030750Z-camera-lidar-motion-60stable.svg
data/fusion/20260626T030750Z-camera-lidar-motion-60stable.json
```

Summary:

- successful visual-motion pairs: 34 of 59;
- median pose inliers: 138;
- moving alignment RMSE: 0.063 m;
- median direction error: 43.9 degrees;
- max direction error: 167.7 degrees;
- lidar sample path length: 0.841 m;
- aligned camera sample path length: 1.279 m;
- camera candidate: `z_forward_x_right`.

Per-pair inspection showed scattered good pairs but no long clean contiguous
window. Several adjacent frame pairs disagreed by 80 to 170 degrees even when
the visual matcher found many inliers. This supports the held-out 3DGS result:
the model is not merely undertrained; the exported camera poses are inconsistent
with image motion through enough of the curved path to cause ghosting.

Result: fail for the 60-stable pose sequence. Do not keep training this package
as-is. The next software-only check should try coarser sampling from the same
raw capture to reduce tiny-baseline direction noise and avoid over-representing
unstable curved sections.

Next action: generate a 36-view coarse pose set from roughly 10 to 72 seconds,
run `compare_camera_lidar_motion.py` with `--min-lidar-step-m 0.01`, and only
package/retrain if the diagnostic improves substantially.

### Room MVP Capture `20260626T030750Z` 36-view Coarse Diagnostic

A coarser 36-view pose set was generated from the same raw capture between
roughly 10 and 72 seconds to test whether fewer, more separated views would
reduce tiny-baseline direction noise from the 60-view export.

Artifacts:

```text
data/fusion/20260626T030750Z-camera-poses-36coarse.json
data/fusion/20260626T030750Z-camera-lidar-motion-36coarse.svg
data/fusion/20260626T030750Z-camera-lidar-motion-36coarse.json
```

Diagnostic result:

- sampled camera frames: 36;
- successful visual-motion pairs: 24 of 35;
- median pose inliers: 152;
- moving alignment RMSE: 0.067 m;
- median direction error: 49.6 degrees;
- camera candidate: `z_forward_x_right`.

This is worse than the 60-view diagnostic's 0.063 m RMSE and 43.9 degree median
direction error. Per-pair inspection found a few isolated good pairs, but no
long clean contiguous sequence suitable for GraphDECO retraining. The user
reported that the robot was moved in a snake-like path, curving left, then right,
then left while moving forward. That motion is consistent with the diagnostic:
the 2D lidar ICP trajectory cannot supply camera poses stable enough for clean
held-out Gaussian-splat views through repeated steering-direction changes.

Result: fail. Do not package or train the 36coarse export.

Next action: collect a new physical capture using one boring, smooth, shallow
arc only. Do not snake, reverse steering direction, or oscillate left/right
during the moving window.

### Smooth-arc Capture `20260626T041136Z` and 60-view Package

Session `20260626T041136Z` was captured after changing the motion instruction
from a snake-like path to one smoother shallow arc. The session was copied to
the Windows workstation and validated successfully:

- mode: `reconstruction_candidate`;
- camera: 1,343 frames over 89.450 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 684 scans over 88.670 seconds;
- lidar gap events above 1.5x nominal: 0;
- lidar valid returns per scan min/median/max: 655/725/791;
- oversized lidar scans: 0;
- shared monotonic-clock overlap: 88.305 seconds;
- geometry valid for reconstruction: true.

ICP trajectory reconstruction:

- selected scans for ICP: 146;
- ICP steps/rejected steps: 145/0;
- skipped oversized scans: 0;
- estimated path length: 0.733 m;
- estimated net displacement: 0.457 m;
- estimated net rotation: -19.17 degrees;
- map output points: 226,683.

The 60-stable camera pose export used frames from 7.998 to 74.986 seconds.
Compared with the snake-like `20260626T030750Z` capture, the trajectory is
smoother and the camera/lidar motion diagnostic is much more consistent:

- successful visual-motion pairs: 24 of 59;
- median pose inliers: 90;
- moving alignment RMSE: 0.031 m;
- median direction error: 16.0 degrees;
- camera candidate: `z_forward_x_right`.

This is a major improvement over the snake-like 60-stable diagnostic:

- moving alignment RMSE: `0.063 m -> 0.031 m`;
- median direction error: `43.9 deg -> 16.0 deg`.

Sparse fused feature map:

- accepted sparse 3D points: 655;
- pairs with accepted points: 9 of 59;
- median reprojection error: 1.89 px;
- median triangulation angle: 0.52 degrees;
- point extents: `x=-0.32..+2.16 m`, `y=-1.03..-0.01 m`,
  `z=-0.02..+0.25 m`.

The sparse geometry is weaker than the prior 48-stable package despite the
better camera/lidar motion agreement. The likely reason is insufficient visual
feature/parallax coverage in the selected 60-view window, plus a low-motion tail
near the end of the sampled sequence.

GraphDECO package:

```text
data/exports/gaussian-splatting/20260626T041136Z-60stable-undistorted-graphdeco.zip
```

Package/check results:

- images: 60 at 1920x1080;
- camera model: `PINHOLE`;
- sparse points: 655;
- missing images: 0;
- ZIP size: 10,933,541 bytes;
- GraphDECO input readiness: pass;
- expected warning remains: zero COLMAP feature-track references.

Result: partial pass. This is the best camera/lidar motion agreement so far,
but the sparse seed geometry is weak. The package is valid for a GraphDECO eval
run, but if held-out results are poor, the next fix should be more textured
features and usable parallax, not more snake-like steering.

Next action: run GraphDECO `--eval` at 7,000 iterations on the valid package, or
first try a software-only moving-window re-export that removes the low-motion
tail.

### Lidar-height Target Retest After Camera Adjustment

The camera and tape targets were physically adjusted, then session
`20260625T195941Z` was captured in `mounted_calibration` mode. Desktop
validation passed:

- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 222 scans over 28.771 seconds;
- lidar gap events above 1.5x nominal: 0;
- lidar valid returns per scan min/median/max: 809/821/835;
- shared monotonic-clock overlap: 28.758 seconds.

Visual overlay sweeps were generated from
`lidar-height-target.jpg`. After the camera adjustment, the older working
values `lidar_angle_offset_deg=125`, `roll=-2`, and `pitch=-1` no longer aligned
with the tape targets. The user selected the concentrated blue line at
`lidar_angle_offset_deg=155` as the best angle panel and the right concentrated
pink line in `pitch=+8` as closest to the tape height. A follow-up roll sweep at
`angle=155` and `pitch=+8` found `roll=+2` as the best tested roll, but only the
concentrated pink line on the left crossed the left-most tape marker; markers 2,
3, and 4 were not crossed.

Result: pass for recording quality but inconclusive for extrinsic calibration.
Because one marker can be matched while the other markers cannot, the four-marker
setup is not a reliable single calibration constraint. The current targets appear
to span different surfaces/depths or surfaces that are not all visible to the
same lidar scan-plane returns. Do not keep overfitting roll/pitch/yaw against
this target.

Next action: build a single-plane calibration target: one flat, opaque board or
box face, perpendicular enough to the lidar/camera view, with one continuous
horizontal tape stripe at measured lidar scan height and vertical strips crossing
that stripe. Re-capture once that single-plane target is set up.

### Single-plane Cardboard Target Follow-up

Session `20260625T202308Z` was captured after setting up a larger single-plane
cardboard-box target with a horizontal tape stripe. Desktop validation passed
with one lidar timing anomaly:

- camera: 443 frames over 29.461 seconds;
- camera gap events above 1.5x nominal: 0;
- lidar: 222 scans over 28.736 seconds;
- lidar gap events above 1.5x nominal: 1;
- oversized lidar scans above 1.5x median returns: 1;
- lidar valid returns per scan min/median/max: 796/806/1620;
- shared monotonic-clock overlap: 28.732 seconds.

The first full close-box angle sweep suggested that `lidar_angle_offset_deg=143`
placed the most pink, near-range returns on the cardboard target, but the returns
were visibly curved rather than a single straight row and sat slightly above the
tape stripe. Follow-up overlays isolated an approximate cardboard-front inlier
segment from raw lidar angles 190 to 240 degrees and distance 0.35 to 1.0 meters.
The user reported that `camera_up_m` between 0.100 and 0.105 meters was best, and
that `roll=+6 deg` was the best tested roll. This candidate crosses the middle
and right tape markers, but stops short of the left marker. The visible points
still form a curve instead of one straight line.

Result: partial calibration candidate only. The current best visual candidate is
approximately `lidar_angle_offset_deg=143`, `camera_up_m=0.1025`,
`pitch=+8 deg`, and `roll=+6 deg`, using the calibrated Pi Camera v2
intrinsics. Do not promote these values to the main rig configuration yet because
the target does not provide a full-width single-plane constraint.

Next action: either accept this as a temporary middle/right alignment for
non-textured lidar experiments, or rebuild the calibration target as a larger,
uninterrupted, flat opaque board at lidar height so the same physical surface
produces returns across the left, middle, and right markers.

## 2026-06-22 Initial Sensor Detection

### Hypothesis

The Raspberry Pi 5 should boot a 64-bit operating system without power
throttling, detect the Camera Module 2, save a still image, and enumerate the
RPLIDAR A1M8 USB serial adapter.

### Setup

- hardware: Raspberry Pi 5 8 GB, Camera Module 2 (IMX219), RPLIDAR A1M8;
- operating system: 64-bit Raspberry Pi OS; exact release not yet recorded;
- Raspberry Pi user and hostname: `pi5@pi5`;
- mount revision: sensors connected but not yet rigidly mounted;
- software revision: initial bring-up before acquisition software;
- calibration IDs: none.

### Commands

```text
getconf LONG_BIT
vcgencmd get_throttled
rpicam-hello --list-cameras
mkdir -p ~/sensor-tests
rpicam-still --nopreview -o ~/sensor-tests/camera.jpg
lsusb
ls -l /dev/serial/by-id/ 2>/dev/null || true
sudo dmesg --ctime | tail -n 50
```

### Measurements

- operating-system word size: 64-bit;
- throttling flags: `0x0`;
- camera: Sony IMX219 detected at up to 3280x2464;
- camera capture: completed successfully at 3280x2464;
- downloaded camera sample: `camera-2.jpg`, 3280x2464, approximately 870 KB;
- camera sample inspection: geometry and orientation are coherent, fine edges
  are visible, and there is no obvious cable corruption; image is usable for
  bring-up but has a green cast, low-light noise/softness, and a large
  low-texture wall area;
- lidar USB adapter: Silicon Labs CP210x, USB ID `10c4:ea60`;
- stable lidar serial path:
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`;
- kernel log: no camera or USB disconnect errors in the supplied tail;
- lidar serial number: `439AECF0C3E09ED2A0EA98F3D0424110`;
- lidar firmware: 1.29;
- lidar hardware revision: 7;
- lidar health: OK, error code 0;
- lidar scan: one complete scan returned and rendered as a non-empty histogram;
- idle/bring-up temperature: 46.6 degrees Celsius;
- throttling after independent sensor tests: `0x0`;
- root filesystem: 115 GB total, 5.2 GB used, 105 GB available (5% used);
- detailed lidar range statistics and scan frequency: not yet measured.

The camera process emitted `Nc30` and `Nc12` pixel-format warnings, but completed
the still capture. These warnings are not treated as a failure at this stage.

### Result

Pass for independent electronic detection and basic operation. The Raspberry Pi
power baseline, camera detection/capture, usable camera image, lidar
communication, lidar health, motor control, and one scan succeeded. Sustained
simultaneous operation has not yet been validated. Better, steadier lighting and
more visual texture will be needed for reconstruction captures.

### Next Action

Inspect `camera.jpg`, record the remaining system baseline, and run both sensors
together for ten minutes while checking temperature, throttling, USB stability,
and storage.

## Template

### Experiment ID

`YYYY-MM-DD-short-name`

### Hypothesis

What should happen, and why?

### Setup

- hardware and mount revision:
- operating system:
- software revision/commit:
- calibration IDs:
- room and lighting:

### Commands

```text
Exact commands used.
```

### Measurements

- duration:
- camera frames and dropped frames:
- lidar scans and rejected scans:
- temperature and throttling:
- reconstruction metrics:
- tape-measured reference dimensions:

### Result

Pass, fail, or inconclusive. Include observed evidence.

### Next Action

One small, testable follow-up.
