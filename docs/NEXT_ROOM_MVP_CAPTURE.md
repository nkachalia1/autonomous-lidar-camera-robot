# Next Room MVP Capture

This is the next hardware experiment after the successful smooth-arc
`20260626T041136Z` GraphDECO eval run. The pipeline can now produce a
recognizable held-out 3DGS result from lidar-anchored camera poses. The next
capture should test repeatability and improve feature/parallax coverage, not
simply train longer.

## Hypothesis

If the camera observes the target area from one smooth shallow arc, with more
visible texture at multiple depths and no stationary tail, then held-out
GraphDECO renders should match or improve over the current smooth-arc baseline.

## Success Criteria

Minimum pass:

- session validates with no camera gaps;
- lidar has no major gaps or oversized scan burst clusters;
- ICP path is smooth and plausible;
- camera/lidar motion diagnostic stays near the current best result
  (`0.031 m` moving alignment RMSE and `16.0 deg` median direction error);
- 72 stable camera poses are exported from the moving part only;
- GraphDECO `--eval` training reaches 7,000 iterations;
- held-out test median PSNR is near or above the current smooth-arc baseline of
  `20.399 dB`.

Stretch target:

- held-out test median PSNR above `22 dB`;
- held-out test median MAE below `10`;
- held-out render contact sheet visibly preserves the main objects without
  large double images.

## Scene Setup

Use the same type of textured scene, but make it easier for novel views:

- keep the checkerboard/pegboard visible through the whole motion;
- keep the small white/black/red/yellow objects visible through the whole
  motion;
- add one or two more textured objects at a different depth;
- avoid shiny glass/window as a main feature;
- keep lighting bright and steady;
- keep the camera mast rigid and do not touch it during capture.

## Motion

Use one smooth, shallow forward arc while keeping the target in view. Do not use
a snake path, S-curve, two-lane pass, or repeated left/right steering changes.
The previous snake-like capture created enough heading inconsistency that the
2D-lidar-derived camera poses failed held-out validation.

Recommended with the current 24-inch power-cable limit:

- total duration: 90 seconds;
- first 5 seconds: still;
- next 65 to 70 seconds: slow continuous motion with one gentle steering
  direction;
- final 5 seconds: still;
- path length: roughly 18 to 24 inches;
- direction: forward plus a slight, constant left or right curve so the target
  stays visible;
- avoid stopping mid-run;
- avoid reversing the steering direction mid-run;
- avoid sudden rotations, bumps, or wheel slip.

The path should feel boring. Boring is good here: one continuous curve is easier
for the 2D lidar ICP trajectory than a visually interesting snake path.

## Raspberry Pi Capture

Power on the Pi, SSH in, then run:

```bash
cd ~/fuse-recorder
python3 capture_session.py \
  --duration 90 \
  --capture-mode reconstruction_candidate \
  --geometry-valid-for-reconstruction
```

Record the printed session ID.

## Windows Processing

From the project root:

```powershell
cd "C:\Users\Neel\Documents\Fuse Lidar and Camera"
$session = "SESSION_ID"

scp -r pi5@pi5.local:/home/pi5/fuse-data/sessions/$session "$HOME\Downloads\"

python reconstruction\validate_session.py "$HOME\Downloads\$session"
```

Reconstruct the lidar trajectory:

```powershell
python reconstruction\render_icp_lidar_map.py `
  "$HOME\Downloads\$session" `
  --output "data\room-motion\$session-icp-map.svg" `
  --png-output "data\room-motion\$session-icp-map.png" `
  --ply-output "data\room-motion\$session-icp-map.ply" `
  --trajectory-output "data\room-motion\$session-icp-trajectory.json" `
  --motion-start-s 5 `
  --motion-end-s 80 `
  --lidar-angle-offset-deg 125
```

Export 72 stable camera poses from the moving window:

```powershell
python reconstruction\render_camera_pose_timeline.py `
  "$HOME\Downloads\$session" `
  --trajectory "data\room-motion\$session-icp-trajectory.json" `
  --output "data\fusion\$session-camera-pose-timeline-72stable.svg" `
  --json-output "data\fusion\$session-camera-poses-72stable.json" `
  --sample-count 72 `
  --sample-start-s 8 `
  --sample-end-s 75 `
  --lidar-angle-offset-deg 125
```

Run visual checks:

```powershell
python reconstruction\render_camera_pose_contact_sheet.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-72stable.json" `
  --output "data\fusion\$session-camera-pose-contact-sheet-72stable.svg" `
  --thumbnail-width 640 `
  --jpeg-quality 4

python reconstruction\compare_camera_lidar_motion.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-72stable.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\$session-camera-lidar-motion-72stable.svg" `
  --json-output "data\fusion\$session-camera-lidar-motion-72stable.json" `
  --min-lidar-step-m 0.005
```

Build sparse points and export GraphDECO package:

```powershell
python reconstruction\render_sparse_fused_feature_map.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-72stable.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\$session-sparse-fused-feature-map-72stable.svg" `
  --json-output "data\fusion\$session-sparse-fused-feature-map-72stable.json" `
  --ply-output "data\fusion\$session-sparse-fused-feature-map-72stable.ply" `
  --min-lidar-step-m 0.005

python reconstruction\export_colmap_camera_poses.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-72stable.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --points-json "data\fusion\$session-sparse-fused-feature-map-72stable.json" `
  --output-dir "data\exports\colmap\$session-72stable-undistorted" `
  --undistort-images `
  --image-width 1920 `
  --jpeg-quality 95

python reconstruction\check_graphdeco_input.py `
  "data\exports\colmap\$session-72stable-undistorted" `
  --json-output "data\exports\colmap\$session-72stable-undistorted\graphdeco_input_check.json"

python reconstruction\package_graphdeco_dataset.py `
  "data\exports\colmap\$session-72stable-undistorted" `
  --output "data\exports\gaussian-splatting\$session-72stable-undistorted-graphdeco.zip"
```

## Colab Validation

Use GraphDECO with `--eval` and 7,000 iterations. Compare both train and test
render bundles with `reconstruction/compare_graphdeco_renders.py`.

Do not judge success from train views only. The held-out test contact sheet is
the gate for the room MVP.
