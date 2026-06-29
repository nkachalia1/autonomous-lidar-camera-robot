# Red Cup Following with Continuous Lidar Safety

This milestone turns the robot from a reconstruction rig into a small
autonomous behavior testbed:

- the Pi Camera detects a red cup/target direction;
- the RPLIDAR A1M8 continuously watches the forward sector;
- the TB6612 drives the wheels toward the cup;
- the robot stops when the lidar sees a close front obstacle.

This is intentionally simpler than SLAM. It is a useful stepping stone toward
repeatable autonomous motion for later room/hallway reconstruction.

## Current Known-good Values

These values were tested on 2026-06-28 after the robot was moved to untethered
Pi power.

| Parameter | Value | Notes |
|---|---:|---|
| lidar front center | `85 deg` | From `lidar_front_probe.py` with a box/cup in front |
| front sector half width | `20 deg` | Watches `65..105 deg` |
| stop distance | `0.203 m` | 8 inches; safer than the 6 inch target |
| left trim | `0.95` | After fixing a loose left motor wire |
| right trim | `0.85` | Reduces faster right side |
| forward speed | `0.48` | Works without being too jumpy |
| arc slow | `0.52` | Kept above motor stall threshold |
| arc fast | `0.60` | Gentle search/turn bias |

Observed behavior:

- robot detected the red cup and followed it;
- lidar remained continuously spinning;
- safety stop triggered at about `0.175 m` with an `0.203 m` threshold;
- the final stop was about 6.9 inches because the robot coasted slightly after
  the watchdog stop.

Keep the stop distance at 8 inches until the mechanical build is more rigid and
motor response is more repeatable.

## Build the Continuous Lidar Helper

Copy `pi/lidar_front_stream.cpp` to the Pi if needed:

```powershell
scp pi\lidar_front_stream.cpp pi5@pi5.local:/home/pi5/fuse-recorder/
```

On the Pi:

```bash
mkdir -p ~/fuse-recorder/build

g++ -std=c++17 -O2 -Wall -Wextra \
  -I/home/pi5/rplidar_sdk/sdk/include \
  -I/home/pi5/rplidar_sdk/sdk/src \
  ~/fuse-recorder/lidar_front_stream.cpp \
  /home/pi5/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a \
  -lpthread -lrt \
  -o ~/fuse-recorder/build/lidar_front_stream
```

Quick stream test:

```bash
PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

~/fuse-recorder/build/lidar_front_stream "$PORT" 115200 85 20
```

Expected: one JSON object per scan with a `closest_m` field. Press `Ctrl+C` to
stop. The lidar should spin continuously while this process runs.

## Run the Red Cup Follower

Copy the controller to the Pi if needed:

```powershell
scp pi\red_cup_follow_continuous.py pi5@pi5.local:/home/pi5/
```

Make sure the motor battery switch is on, the Pi is on the power bank, the cup
is 3 to 5 feet in front of the robot, and your hand is near the motor-battery
switch.

```bash
python3 ~/red_cup_follow_continuous.py --armed
```

The defaults match the known-good values above. To be more conservative:

```bash
python3 ~/red_cup_follow_continuous.py \
  --armed \
  --stop-distance-m 0.254 \
  --forward-speed 0.45
```

## Expected Output

Good run shape:

```text
Waiting for lidar stream...
Continuous red cup follow. Stop distance=0.203 m. Hand ready.
Initial front distance=0.802
1: front=0.802 cup error_x=201.0 red_pixels=6052
...
SAFETY STOP: front=0.175 m <= 0.203 m
STOP_CONDITION_REACHED
Stopping
```

The robot should continue moving/searching until one of these happens:

- lidar front distance is at/below the stop threshold;
- the run hits `--max-run-s`;
- the user presses `Ctrl+C`;
- the lidar stream becomes stale.

## Failure Modes

| Symptom | Likely cause | Fix |
|---|---|---|
| only one wheel moves | loose motor wire or one side below stall threshold | run `left_right_motor_check.py`; reseat motor outputs |
| lidar starts/stops repeatedly | using old short-capture script | use `lidar_front_stream` helper |
| robot searches forever | camera lost red cup | place cup farther away; lower camera angle; improve lighting |
| robot stops too late | threshold too low / coasting | increase `--stop-distance-m` to `0.254` |
| robot is too fast | motor speed too high | lower `--forward-speed`, but keep above stall threshold |
| no red target found | threshold/lighting issue | inspect `~/sensor-tests/red-cup-continuous-detection.jpg` |

## Why This Matters for Reconstruction

This behavior is not the final reconstruction pipeline, but it proves the robot
can:

1. run untethered;
2. read camera and lidar at the same time;
3. command motors while watching lidar safety;
4. perform repeatable autonomous motion.

That is the missing foundation for later hallway/room captures. Once this
motion becomes reliable, the same robot base can drive smoother trajectories
for synchronized lidar/camera reconstruction sessions.
