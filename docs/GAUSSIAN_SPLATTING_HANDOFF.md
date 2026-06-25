# Gaussian Splatting Handoff

This note records the first compatibility check between our lidar/camera export
and the official GraphDECO/Inria 3D Gaussian Splatting implementation.

## Current status

We have not run NeRF or Gaussian splatting yet.

The current handoff artifact is:

```text
data/exports/colmap/20260625T214456Z-steady-undistorted/
```

It contains:

- `images/`: 9 undistorted 1920x1080 JPEG frames from the steady-motion window;
- `sparse/0/cameras.txt`: one `PINHOLE` camera model;
- `sparse/0/images.txt`: 9 lidar-anchored camera poses;
- `sparse/0/points3D.txt`: 327 sparse diagnostic seed points;
- `export_manifest.json`: project-specific metadata;
- `preview.svg` and `preview_validation.json`: local import/preview result.

The local parser/previewer passed:

- cameras/images/points parsed: 1 / 9 / 327;
- missing image files: 0;
- recovered camera path length: 0.475 m;
- quaternion norms: unit length within numerical tolerance.

After resolving the Windows Start Menu shortcut, COLMAP was also found inside
the `Ubuntu-22.04` WSL distro:

```text
C:\Program Files\WSL\wslg.exe -d Ubuntu-22.04 --cd "~" -- colmap gui
```

CLI check:

```text
wsl.exe -d Ubuntu-22.04 -- colmap -h
```

Result:

- COLMAP version: 3.7;
- CUDA: not available in that WSL build;
- `model_converter` is available.

Real COLMAP conversion passed:

```powershell
& "C:\Windows\System32\wsl.exe" -d Ubuntu-22.04 -- bash -lc `
  'mkdir -p /mnt/c/Users/Neel/Documents/Fuse\ Lidar\ and\ Camera/data/exports/colmap/20260625T214456Z-steady-undistorted/sparse-bin/0; colmap model_converter --input_path /mnt/c/Users/Neel/Documents/Fuse\ Lidar\ and\ Camera/data/exports/colmap/20260625T214456Z-steady-undistorted/sparse/0 --output_path /mnt/c/Users/Neel/Documents/Fuse\ Lidar\ and\ Camera/data/exports/colmap/20260625T214456Z-steady-undistorted/sparse-bin/0 --output_type BIN'
```

The generated binary model files were copied back into `sparse/0` beside the
text files:

- `sparse/0/cameras.bin`;
- `sparse/0/images.bin`;
- `sparse/0/points3D.bin`.

COLMAP `model_analyzer` on `sparse/0` reported:

- cameras: 1;
- images / registered images: 9 / 9;
- points: 327;
- observations: 0;
- mean track length: 0;
- mean observations per image: 0;
- mean reprojection error: 3.371872 px.

The zero observations/track length are expected because our sparse seed points
come from the diagnostic triangulator and are not linked to COLMAP keypoint
tracks. This is acceptable for a GraphDECO loader smoke test, but it is not a
full SfM reconstruction.

## Official input requirements inspected

Primary sources checked:

- GraphDECO/Inria `gaussian-splatting` README:
  <https://github.com/graphdeco-inria/gaussian-splatting>
- GraphDECO/Inria loader source:
  <https://github.com/graphdeco-inria/gaussian-splatting/blob/main/scene/dataset_readers.py>
- GraphDECO/Inria COLMAP loader source:
  <https://github.com/graphdeco-inria/gaussian-splatting/blob/main/scene/colmap_loader.py>
- COLMAP output-format documentation:
  <https://colmap.github.io/format.html>
- COLMAP command-line documentation:
  <https://colmap.github.io/cli.html>

Relevant conclusions:

- GraphDECO training is launched with a source dataset path, commonly:

  ```text
  python train.py -s <path to COLMAP or NeRF Synthetic dataset>
  ```

- The default image folder name is `images`.
- The GraphDECO Python loader first tries `sparse/0/images.bin` and
  `sparse/0/cameras.bin`, then falls back to text files `images.txt` and
  `cameras.txt` if binary files are unavailable.
- The text intrinsics reader asserts `PINHOLE`; the higher-level camera loader
  handles `PINHOLE` and `SIMPLE_PINHOLE` for undistorted datasets.
- The loader will convert `points3D.txt` or `points3D.bin` into
  `sparse/0/points3D.ply` the first time the scene is opened.
- COLMAP itself defines text sparse models as `cameras.txt`, `images.txt`, and
  `points3D.txt`, with poses in `images.txt` using world-to-camera `qvec/tvec`.

## Compatibility assessment

Our current export is a reasonable first GraphDECO input candidate because:

- it uses the expected top-level `images/` folder;
- it uses `sparse/0/`;
- it exports `PINHOLE` intrinsics;
- it exports undistorted images;
- it exports text COLMAP model files, which the GraphDECO loader can fall back
  to when binary files are absent.

Important caveats:

- The poses come from lidar ICP plus rough camera-to-lidar extrinsics, not from
  COLMAP bundle adjustment.
- The sparse seed points come from our diagnostic triangulation, not true COLMAP
  tracks.
- The current scene has only 9 views over a short, low camera path. This is good
  enough for an input-format test, but probably not enough for a nice splat.
- Real COLMAP GUI/import or `model_converter` may be stricter about missing
  2D observation tracks than GraphDECO's Python loader.

## If COLMAP is installed

`colmap` was not found on this shell's Windows `PATH`, but it is available in
the `Ubuntu-22.04` WSL distro through the Start Menu shortcut. If using a
different COLMAP install, set the path explicitly:

```powershell
$COLMAP = "C:\path\to\COLMAP.bat"    # Windows release commonly uses COLMAP.bat
# or
$COLMAP = "C:\path\to\colmap.exe"

& $COLMAP -h
```

Optional strict import/conversion test:

```powershell
$export = "C:\Users\Neel\Documents\Fuse Lidar and Camera\data\exports\colmap\20260625T214456Z-steady-undistorted"
New-Item -ItemType Directory -Force "$export\sparse-bin\0"

& $COLMAP model_converter `
  --input_path "$export\sparse\0" `
  --output_path "$export\sparse-bin\0" `
  --output_type BIN
```

If this succeeds, we have stronger evidence that real COLMAP accepts the model.
If it fails, keep the GraphDECO text path as the first target and add stricter
track export later.

## First GraphDECO training smoke command

After cloning and setting up the official GraphDECO environment, try a very
small smoke run before any serious training:

```powershell
$DATA = "C:\Users\Neel\Documents\Fuse Lidar and Camera\data\exports\colmap\20260625T214456Z-steady-undistorted"
$OUT = "C:\Users\Neel\Documents\Fuse Lidar and Camera\data\exports\gaussian-splatting\20260625T214456Z-smoke"

python train.py `
  -s "$DATA" `
  -m "$OUT" `
  --images images `
  --resolution 2 `
  --iterations 1000
```

Expected outcomes:

- Best case: GraphDECO accepts the dataset and starts optimization.
- Acceptable first failure: loader complains about COLMAP text format or sparse
  points. Then we adapt the exporter.
- Bad-but-informative result: training runs but the splat looks poor. That means
  the format path works, but the capture/pose quality needs improvement.

## Recommended next capture for splatting

For an actually useful Gaussian-splatting attempt, collect a better dataset:

- 60 to 120 seconds instead of 30 seconds;
- slow, steady motion;
- more lateral viewpoint change;
- camera aimed at textured, non-flat objects;
- avoid mostly carpet/blank wall views;
- keep the rig as rigid as possible;
- include reference boards or visible markers at known locations for metric
  sanity checks.
