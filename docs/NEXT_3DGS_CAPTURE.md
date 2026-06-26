# Next 3DGS Capture

This is the next experiment after the first GraphDECO smoke-test pass. The goal
is to collect a better input dataset for Gaussian splatting while keeping the
rig and software path simple.

## Hypothesis

If we capture a more textured scene with more camera viewpoints, then the same
lidar-anchored COLMAP export and GraphDECO Colab workflow should still pass, and
the output should have more useful visual structure than the 9-view diagnostic
dataset.

## Success criteria

Minimum pass:

- Raspberry Pi capture validates with no camera/lidar timestamp gaps;
- ICP trajectory is plausible, with no rejected ICP steps;
- 24 to 36 camera poses are exported;
- GraphDECO Colab T4 training reaches iteration 300;
- output archive contains `point_cloud/iteration_300/point_cloud.ply`.

Better-than-before signs:

- sparse feature map has more than 327 accepted points;
- several neighboring camera pairs have useful feature matches/inliers;
- Colab output point cloud has a visibly more scene-like shape.

## Scene setup

Do not aim at a blank wall. Build a deliberately textured target area:

- pegboard, cardboard box, books, printed pages, checkerboard, painter's tape,
  or sticky notes;
- at least two depth layers, for example a box in front of a wall/pegboard;
- avoid glass/window as the main target;
- keep lighting steady and bright;
- keep people and moving objects out of the scene.

## Motion

Use a slow, smooth pass within the power-cable limit:

- total capture: 75 seconds;
- first 5 seconds: rig still;
- next 55 to 60 seconds: move slowly;
- final 5 seconds: rig still;
- translate roughly 18 to 24 inches total;
- keep the camera aimed at the textured target;
- avoid sharp turns, bumps, and flexing the camera mast.

This is still not ideal Gaussian-splatting motion, but it is a controlled next
step with the current tethered rig.

## Raspberry Pi capture

Power on the Pi and SSH in:

```powershell
ssh pi5@pi5.local
```

On the Pi:

```bash
cd ~/fuse-recorder
python3 capture_session.py \
  --duration 75 \
  --capture-mode reconstruction_candidate \
  --geometry-valid-for-reconstruction
```

Write down the printed session ID, for example:

```text
20260626T012345Z
```

## Copy and validate on Windows

In PowerShell from the project root:

```powershell
cd "C:\Users\Neel\Documents\Fuse Lidar and Camera"
$session = "SESSION_ID"

scp -r pi5@pi5.local:/home/pi5/fuse-data/sessions/$session "$HOME\Downloads\"

python reconstruction\validate_session.py "$HOME\Downloads\$session"
```

Pass criteria:

- `Geometry valid for reconstruction: True`;
- no camera/lidar gap events;
- no oversized lidar scans.

## Reconstruct lidar trajectory

```powershell
python reconstruction\render_icp_lidar_map.py `
  "$HOME\Downloads\$session" `
  --output "data\room-motion\$session-icp-map.svg" `
  --png-output "data\room-motion\$session-icp-map.png" `
  --ply-output "data\room-motion\$session-icp-map.ply" `
  --trajectory-output "data\room-motion\$session-icp-trajectory.json" `
  --motion-start-s 5 `
  --motion-end-s 70 `
  --lidar-angle-offset-deg 125
```

If the path looks curled, jumps, or doubles back unexpectedly, stop and review
before exporting to GraphDECO.

## Sample camera poses

Start with 30 camera samples. If visual matching is poor, retry with 24 or 36.

```powershell
python reconstruction\render_camera_pose_timeline.py `
  "$HOME\Downloads\$session" `
  --trajectory "data\room-motion\$session-icp-trajectory.json" `
  --output "data\fusion\$session-camera-pose-timeline-30.svg" `
  --json-output "data\fusion\$session-camera-poses-30.json" `
  --sample-count 30 `
  --sample-start-s 5 `
  --sample-end-s 70 `
  --lidar-angle-offset-deg 125
```

Optional visual sanity contact sheet:

```powershell
python reconstruction\render_camera_pose_contact_sheet.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-30.json" `
  --output "data\fusion\$session-camera-pose-contact-sheet-30.svg" `
  --thumbnail-width 640 `
  --jpeg-quality 4
```

## Visual-motion and sparse feature checks

```powershell
python reconstruction\compare_camera_lidar_motion.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-30.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\$session-camera-lidar-motion-30.svg" `
  --json-output "data\fusion\$session-camera-lidar-motion-30.json" `
  --min-lidar-step-m 0.025
```

```powershell
python reconstruction\render_sparse_fused_feature_map.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-30.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --output "data\fusion\$session-sparse-fused-feature-map-30.svg" `
  --json-output "data\fusion\$session-sparse-fused-feature-map-30.json" `
  --ply-output "data\fusion\$session-sparse-fused-feature-map-30.ply"
```

## Export COLMAP-style dataset

```powershell
python reconstruction\export_colmap_camera_poses.py `
  "$HOME\Downloads\$session" `
  --pose-json "data\fusion\$session-camera-poses-30.json" `
  --intrinsics "config\camera_intrinsics_pi_camera_v2_1920x1080.yaml" `
  --points-json "data\fusion\$session-sparse-fused-feature-map-30.json" `
  --output-dir "data\exports\colmap\$session-30-undistorted" `
  --undistort-images `
  --image-width 1920 `
  --jpeg-quality 95
```

Check and package for Colab:

```powershell
python reconstruction\check_graphdeco_input.py `
  "data\exports\colmap\$session-30-undistorted" `
  --json-output "data\exports\colmap\$session-30-undistorted\graphdeco_input_check.json"
```

```powershell
python reconstruction\package_graphdeco_dataset.py `
  "data\exports\colmap\$session-30-undistorted" `
  --output "data\exports\gaussian-splatting\$session-30-undistorted-graphdeco.zip"
```

## Colab

Use:

```text
notebooks/GraphDECO_3DGS_Colab_T4_Smoke.ipynb
```

Upload:

```text
data\exports\gaussian-splatting\$session-30-undistorted-graphdeco.zip
```

Run the notebook cells. The first pass should stay at 300 iterations. If it
passes, then try a longer 3,000 to 7,000 iteration run only after inspecting the
300-iteration output.
