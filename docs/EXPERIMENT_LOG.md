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
