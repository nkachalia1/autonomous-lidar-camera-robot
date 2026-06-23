# Lidar-Camera Projection Overlay

The first fusion milestone is not a full 3D reconstruction. It is a diagnostic
image: draw one 2D lidar scan on top of a camera frame and check whether the
points land on plausible surfaces.

If the overlay is wrong, dense reconstruction will be wrong too. This step is
the calibration flashlight.

## What Passed So Far

The first robot-chassis rig smoke test, session `20260623T202904Z`, passed:

- camera: 443 frames over 29.462 seconds;
- camera gap events: 0;
- lidar: 202 scans over 28.593 seconds;
- lidar gap events: 0;
- lidar valid returns per scan min/median/max: 834/850/878;
- shared monotonic-clock overlap: 28.495 seconds.

That makes the rig stable enough for geometry work.

For the next mounted smoke or calibration capture, label the session more
accurately:

```bash
cd ~/fuse-recorder
python3 capture_session.py --duration 30 --capture-mode mounted_rig_smoke
```

Do not add `--geometry-valid-for-reconstruction` yet. We only use that flag
after the projection overlay and calibration are trustworthy.

## Required Measurements

Measure the camera optical center relative to the lidar measurement center.
Use the camera lens center, not the camera board center. Use the center of the
lidar spinner/top cap as the best visible proxy for the lidar measurement
center.

Coordinate convention:

- `camera_forward_m`: positive if the camera lens is forward of the lidar center;
- `camera_left_m`: positive if the camera lens is left of the lidar center;
- `camera_up_m`: positive if the camera lens is above the lidar scan plane.

For the current photographed rig, `camera_up_m` is likely the largest and most
important value.

Also record rough angles:

- `yaw`: camera pointing left/right relative to rig forward;
- `pitch`: camera tilted up/down;
- `roll`: camera rotated clockwise/counterclockwise in the image.

Approximate values are fine for the first overlay.

## Capture a Background Image

Keep the rig stationary. On the Raspberry Pi:

```bash
mkdir -p ~/fuse-data/calibration
rpicam-still --nopreview --width 1920 --height 1080 \
  -o ~/fuse-data/calibration/rig-calib.jpg
```

Then copy the still to the downloaded session folder on Windows:

```powershell
scp pi5@pi5.local:/home/pi5/fuse-data/calibration/rig-calib.jpg `
  "$HOME\Downloads\20260623T202904Z\"
```

Because the rig is stationary, this still can be paired with the smoke-test
lidar scans for the first overlay. Later, we will use exact video frames.

## Render the First Overlay

Replace the three translation numbers with measured values:

```powershell
cd "C:\Users\Neel\Documents\Fuse Lidar and Camera"

python reconstruction\project_lidar_overlay.py `
  "$HOME\Downloads\20260623T202904Z" `
  --background "$HOME\Downloads\20260623T202904Z\rig-calib.jpg" `
  --output "data\calibration\20260623T202904Z-overlay.svg" `
  --camera-forward-m 0.00 `
  --camera-left-m 0.00 `
  --camera-up-m 0.30 `
  --lidar-angle-offset-deg 0
```

Open the SVG in a browser. The first result will probably be wrong; that is
expected.

Tune in this order:

1. `--lidar-angle-offset-deg`: rotates the projected scan around the lidar.
2. `--camera-up-m`: moves the projected lidar row vertically.
3. `--camera-forward-m` and `--camera-left-m`: shift/parallax-correct points.
4. `--yaw-deg`, `--pitch-deg`, and `--roll-deg`: compensate for camera angle.

For a good first pass, nearby lidar points should land along real object or wall
intersections at the lidar scan height. The overlay does not need to be perfect
yet.

## Interpreting Failure

- No points visible: likely angle offset, forward direction, or camera pitch is
  badly wrong.
- Points form a row but sit too high/low: adjust camera height or pitch.
- Points are mirrored left/right: angle convention or lidar front direction is
  wrong.
- Points align in the center but drift at edges: camera intrinsics or lens
  distortion need calibration.

The script currently uses approximate Pi Camera v2 intrinsics derived from field
of view. Proper checkerboard calibration is still required before reconstruction.
