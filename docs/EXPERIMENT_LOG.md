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
