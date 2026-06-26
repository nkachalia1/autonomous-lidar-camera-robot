# Next Room MVP Capture

This is the next hardware experiment after the 48-view stable-window
GraphDECO eval run. The pipeline can now train recognizable 3DGS models from
lidar-anchored camera poses, but held-out views are still ghosted. The next
capture should target view coverage and parallax, not more raw training time.

## Hypothesis

If the camera observes the target area from more evenly spaced viewpoints with
more sideways parallax and no stationary tail, then held-out GraphDECO renders
should improve over the 48-stable baseline.

## Success Criteria

Minimum pass:

- session validates with no camera gaps;
- lidar has no major gaps or oversized scan burst clusters;
- ICP path is smooth and plausible;
- 48 to 72 stable camera poses are exported from the moving part only;
- GraphDECO `--eval` training reaches 7,000 iterations;
- held-out test median PSNR improves above the 48-stable baseline of
  `16.887 dB`.

Stretch target:

- held-out test median PSNR above `20 dB`;
- held-out test median MAE below `15`;
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

Use a smooth sideways arc or shallow two-lane pass instead of a mostly straight
push with a long stationary tail.

Recommended with the current 24-inch power-cable limit:

- total duration: 90 seconds;
- first 5 seconds: still;
- next 65 to 70 seconds: slow continuous motion;
- final 5 seconds: still;
- path length: roughly 18 to 24 inches;
- direction: mostly sideways relative to the target, with a gentle arc so the
  target stays centered;
- avoid stopping mid-run;
- avoid sudden rotations, bumps, or wheel slip.

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

Export 60 stable camera poses as the first attempt:

```powershell
python reconstruction\render_camera_pose_timeline.py `
  "$HOME\Downloads\$session" `
  --trajectory "data\room-motion\$session-icp-trajectory.json" `
  --output "data\fusion\$session-camera-pose-timeline-60stable.svg" `
  --json-output "data\fusion\$session-camera-poses-60stable.json" `
  --sample-count 60 `
  --sample-start-s 8 `
  --sample-end-s 75 `
  --lidar-angle-offset-deg 125
```

Run visual checks:

```powershell
python reconstruction\render_camera_pose_contact_sheet.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-60stable.json" `
  --output "data\fusion\$session-camera-pose-contact-sheet-60stable.svg" `
  --thumbnail-width 640 `
  --jpeg-quality 4

python reconstruction\compare_camera_lidar_motion.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-60stable.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\$session-camera-lidar-motion-60stable.svg" `
  --json-output "data\fusion\$session-camera-lidar-motion-60stable.json" `
  --min-lidar-step-m 0.005
```

Build sparse points and export GraphDECO package:

```powershell
python reconstruction\render_sparse_fused_feature_map.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-60stable.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\$session-sparse-fused-feature-map-60stable.svg" `
  --json-output "data\fusion\$session-sparse-fused-feature-map-60stable.json" `
  --ply-output "data\fusion\$session-sparse-fused-feature-map-60stable.ply" `
  --min-lidar-step-m 0.005

python reconstruction\export_colmap_camera_poses.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-60stable.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --points-json "data\fusion\$session-sparse-fused-feature-map-60stable.json" `
  --output-dir "data\exports\colmap\$session-60stable-undistorted" `
  --undistort-images `
  --image-width 1920 `
  --jpeg-quality 95

python reconstruction\check_graphdeco_input.py `
  "data\exports\colmap\$session-60stable-undistorted" `
  --json-output "data\exports\colmap\$session-60stable-undistorted\graphdeco_input_check.json"

python reconstruction\package_graphdeco_dataset.py `
  "data\exports\colmap\$session-60stable-undistorted" `
  --output "data\exports\gaussian-splatting\$session-60stable-undistorted-graphdeco.zip"
```

## Colab Validation

Use GraphDECO with `--eval` and 7,000 iterations. Compare both train and test
render bundles with `reconstruction/compare_graphdeco_renders.py`.

Do not judge success from train views only. The held-out test contact sheet is
the gate for the room MVP.
