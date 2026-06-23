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

## Success Criteria

The calibration is useful when:

- most images detect the checkerboard corners;
- reprojection error is low and consistent;
- the resulting intrinsics are saved in the repo under `config/`;
- a new lidar-camera overlay uses the calibrated intrinsics instead of field of
  view approximations.

Until this is done, `config/rig_measurements.yaml` remains provisional.
