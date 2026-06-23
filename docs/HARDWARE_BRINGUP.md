# Raspberry Pi Hardware Bring-up

Run these checks on the Raspberry Pi before implementing sensor fusion.

## Required Physical Items

- Raspberry Pi 5 with suitable cooling and a stable power supply;
- 128 GB microSD card with 64-bit Raspberry Pi OS;
- Camera Module 2;
- **Standard-Mini 22-pin-to-15-pin camera cable** for Raspberry Pi 5;
- RPLIDAR A1M8 with its USB interface/adapter and cable;
- rigid mounting plate or frame.

Raspberry Pi 5 uses a mini 22-pin CAM/DISP connector while Camera Module 2 uses
the standard 15-pin connector. Power the Pi off before connecting or changing
the camera cable.

## 1. Record the Baseline

```bash
uname -a
cat /etc/os-release
getconf LONG_BIT
df -h /
vcgencmd get_throttled
vcgencmd measure_temp
```

Expected:

- 64-bit OS (`getconf LONG_BIT` prints `64`);
- adequate free storage;
- `get_throttled=0x0` before the load test.

Do not continue if the filesystem is nearly full or the Pi reports current
undervoltage.

## 2. Camera Test

Update Raspberry Pi OS through its normal package-management process before
debugging camera compatibility.

List cameras:

```bash
rpicam-hello --list-cameras
```

Capture a still without opening a preview window:

```bash
mkdir -p ~/sensor-tests
rpicam-still --nopreview --output ~/sensor-tests/camera.jpg
file ~/sensor-tests/camera.jpg
```

Inspect the image for:

- correct orientation;
- sharp focus at typical wall distance;
- acceptable exposure;
- no cable-related corruption.

Camera Module 2 focus is mechanically adjustable. Once focus is set for the
room, do not change it between intrinsic calibration and room capture.

If no camera is detected:

1. shut down and disconnect power;
2. reseat both cable ends;
3. confirm the cable is a camera cable, not a display cable;
4. try the other `CAM/DISP` connector;
5. rerun `rpicam-hello --list-cameras`.

Use current `rpicam-*` tools. The legacy `raspistill`, `raspivid`, and original
Picamera stack are unsupported on current Raspberry Pi OS.

## 3. Lidar USB Test

Connect the A1M8 through its supplied USB interface.

```bash
lsusb
ls -l /dev/serial/by-id/ 2>/dev/null || true
dmesg --ctime | tail -n 50
```

Prefer a stable `/dev/serial/by-id/...` path over `/dev/ttyUSB0`, because USB
device numbers can change after a reboot.

Do not use permanent `chmod 777` permissions. The recorder setup will add a
narrow udev rule or use the appropriate serial-device group after the USB
vendor/product identifiers are observed.

The first lidar software test will use SLAMTEC's SDK or ROS 2 driver as a known
reference. Record:

- exact serial device path;
- USB vendor/product ID from `lsusb`;
- detected model, hardware version, and firmware version;
- scan frequency;
- number of valid returns;
- nearest and farthest plausible range.

## 4. Ten-minute Load Check

Run the camera preview/capture and lidar scan at the same time for ten minutes.
Then check:

```bash
vcgencmd get_throttled
vcgencmd measure_temp
df -h /
```

Record any:

- undervoltage;
- thermal throttling;
- USB disconnects;
- camera frame errors;
- lidar timeouts.

## 5. Mount Check

The mount passes when:

- neither sensor moves when the rig is gently carried and set down;
- the lidar has a clear 360-degree scan plane;
- the camera cable cannot brush the spinning lidar;
- the lidar scan plane is approximately level;
- the camera and lidar positions can be measured to within a few millimeters;
- the mounting arrangement can be reproduced after maintenance.

Photograph the assembled rig from the front, side, and top with a ruler visible.
Those images will help define the initial transform before formal calibration.

## Bring-up Report

Add the command output, observations, and sample filenames to
[EXPERIMENT_LOG.md](EXPERIMENT_LOG.md). Do not commit large images or raw
recordings; place them under `data/`.

