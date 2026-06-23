# Camera Intrinsic Calibration

The overlay work is now limited by approximate camera intrinsics. The next
milestone is to estimate the Raspberry Pi Camera Module 2 focal lengths,
principal point, and distortion from a printed checkerboard.

## Target

Use a flat checkerboard with known square size. Good starting options:

- 9x6 inner corners;
- 25 mm square size;
- printed on paper and taped flat to cardboard or foam board.

Measure the actual printed square size with a ruler. Use that measured value,
not the nominal printer setting.

The current project calibration uses a physical 8x8 chessboard, which has 7x7
inner corners. The measured square size is 1 3/16 inches, or 30.1625 mm.

## Capture Protocol

Keep the camera mounted on the rig. The lidar is not needed for this step.

Take 20-30 stills with the checkerboard:

- near the center of the image;
- near each corner and edge of the image;
- tilted left/right/up/down;
- at multiple distances;
- fully visible in every frame;
- sharp, with no motion blur.

Avoid glare and curved paper. A flat target matters.

On the Raspberry Pi:

```bash
mkdir -p ~/fuse-data/camera-intrinsics
```

Capture one image at a time:

```bash
rpicam-still --nopreview --width 1920 --height 1080 \
  -o ~/fuse-data/camera-intrinsics/checker-01.jpg
```

Increment the filename for each image.

Copy the folder to Windows:

```powershell
scp -r pi5@pi5.local:/home/pi5/fuse-data/camera-intrinsics "$HOME\Downloads\"
```

## Run Calibration

From the Windows workstation, run the calibration with an environment that has
OpenCV installed. The existing `3D Scene Reconstruction` virtual environment is
sufficient:

```powershell
cd "C:\Users\Neel\Documents\Fuse Lidar and Camera"

& "C:\Users\Neel\Documents\3D Scene Reconstruction\.venv-win\Scripts\python.exe" `
  reconstruction\calibrate_camera_intrinsics.py `
  "$HOME\Downloads\camera-intrinsics" `
  --inner-cols 7 `
  --inner-rows 7 `
  --square-size-mm 30.1625 `
  --output "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --diagnostic-sheet "data\calibration\camera-intrinsics-detection-sheet-75.jpg"
```

The diagnostic sheet is generated under `data/`, so it is intentionally not
tracked by Git. Use it to confirm that accepted checkerboards have visible
corner detections and rejected images failed for understandable reasons.

## Use Intrinsics in Lidar Overlays

After calibration, pass the saved intrinsics file to overlay tools:

```powershell
python reconstruction\project_lidar_overlay.py `
  "$HOME\Downloads\20260623T215235Z" `
  --background "$HOME\Downloads\20260623T215235Z\rig-calib-target.jpg" `
  --output "data\calibration\20260623T215235Z-overlay-calibrated.svg" `
  --camera-intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --camera-forward-m -0.0737 `
  --camera-left-m 0.0051 `
  --camera-up-m 0.0953 `
  --lidar-angle-offset-deg 125 `
  --roll-deg -2 `
  --pitch-deg -1
```

For angle sweeps, add the same `--camera-intrinsics` argument to
`reconstruction\render_lidar_angle_sweep.py`.

## Success Criteria

The calibration is useful when:

- most images detect the checkerboard corners;
- reprojection error is low and consistent;
- the resulting intrinsics are saved in the repo under `config/`;
- a new lidar-camera overlay uses the calibrated intrinsics instead of field of
  view approximations.

Until this is done, `config/rig_measurements.yaml` remains provisional.
