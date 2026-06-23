# Stationary Milestone 1 Capture

This test may be run before the rigid mount is complete only if every component
remains stationary on a nonconductive surface. Do not touch, carry, calibrate,
or reconstruct from this session.

The purpose is to verify:

- encoded camera recording;
- per-frame camera `SensorTimestamp` metadata;
- complete lidar scans with SDK timestamps;
- shared Raspberry Pi monotonic-clock overlap;
- clean session finalization and validation.

The official Raspberry Pi camera application writes a JSON metadata object for
each encoded frame when `--metadata` is enabled. The SLAMTEC SDK provides the
timestamp of the first point in each complete scan. Both timestamps use the
Raspberry Pi monotonic clock on Linux, in nanoseconds and microseconds
respectively.

## Copy the Recorder to the Pi

From Windows PowerShell in the repository:

```powershell
ssh pi5@pi5.local "mkdir -p /home/pi5/fuse-recorder"

scp .\pi\capture_session.py .\pi\lidar_capture.cpp `
  pi5@pi5.local:/home/pi5/fuse-recorder/
```

## Run a 30-second Smoke Test

On the Raspberry Pi:

```bash
cd ~/fuse-recorder
python3 capture_session.py --duration 30
```

For a mounted-but-not-yet-calibrated rig, label the purpose explicitly:

```bash
python3 capture_session.py --duration 30 --capture-mode mounted_rig_smoke
```

The geometry flag should remain false until the rigid transform is measured or
calibrated.

Expected final output:

```text
Camera status: 0
Lidar status: 0
Session complete: /home/pi5/fuse-data/sessions/<session-id>
```

The lidar may continue spinning after the recorder exits because of the USB
adapter's DTR behavior.

## Run the Stationary Three-minute Test

Only after the 30-second test succeeds:

```bash
cd ~/fuse-recorder
python3 capture_session.py --duration 180
```

Do not move the setup. This session is marked
`geometry_valid_for_reconstruction: false` in its manifest.

## Download the Session

The recorder prints the exact session directory. From Windows PowerShell,
replace `<session-id>`:

```powershell
scp -r pi5@pi5.local:/home/pi5/fuse-data/sessions/<session-id> `
  "$HOME\Downloads\"
```

Validate it from the repository:

```powershell
python reconstruction\validate_session.py `
  "$HOME\Downloads\<session-id>"
```

Attach the validator output and `manifest.json`. Do not upload the large H.264
file unless visual inspection is needed.

## Expected Data

```text
<session-id>/
  manifest.json
  camera.h264
  camera_metadata.json
  camera.log
  lidar_scans.jsonl
  lidar.log
```

At 15 fps, a three-minute camera capture should contain approximately 2,700
frame metadata records. Lidar scan count depends on actual motor speed and scan
mode.

## Official Software References

- [Raspberry Pi camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [Raspberry Pi rpicam-apps](https://github.com/raspberrypi/rpicam-apps)
- [SLAMTEC RPLIDAR SDK](https://github.com/Slamtec/rplidar_sdk)
