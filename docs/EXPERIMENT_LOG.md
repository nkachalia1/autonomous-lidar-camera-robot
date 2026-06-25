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
