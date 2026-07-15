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

## Step 2: Search, Then Approach

Once the direct follower works, the next behavior is:

1. if the red cup is visible, approach it;
2. if the red cup is not visible, turn briefly, stop, let the camera settle, and
   scan with the camera;
3. if the cup appears during the scan, approach it;
4. stop if lidar sees a close front obstacle.

Copy both Python files to the Pi because the search controller imports the
known-good motor, camera, and lidar helpers from `red_cup_follow_continuous.py`:

```powershell
scp pi\red_cup_follow_continuous.py pi\red_cup_search_and_approach.py pi5@pi5.local:/home/pi5/
```

Run scan-only mode first. Put the red cup outside the camera view but inside the
room, leave at least 3 feet of clear floor around the robot, turn the motor
battery on, and keep one hand near the switch.

```bash
python3 ~/red_cup_search_and_approach.py --armed
```

Expected scan-only behavior:

```text
Waiting for lidar stream...
Red cup search-and-approach. mode=scan; stop distance=0.203 m.
1: SCAN front=0.75 turn=right elapsed=0.0s
...
7: APPROACH front=0.68 error_x=140.0 red_pixels=5200 dir=right
...
SAFETY STOP: front=0.18 m <= 0.203 m
STOP_CONDITION_REACHED
```

If the robot scans for about 18 seconds and never sees the cup, it stops with:

```text
SCAN_FAILED_NO_TARGET: enable --allow-explore for step 2
SEARCH_FINISHED_WITHOUT_TARGET
```

That is a safe failure. It means the camera did not detect the cup from the
starting pose.

Only after scan-only mode works should exploratory moves be enabled:

```bash
python3 ~/red_cup_search_and_approach.py \
  --armed \
  --allow-explore \
  --max-explore-moves 3
```

Exploration is deliberately cautious:

- if the front lidar distance is at least `0.45 m`, the robot moves forward for
  `0.60 s`, stops, and scans again;
- if front distance is blocked, the robot turns in place and scans again;
- the same lidar watchdog stops the robot at `0.203 m`;
- the robot stops after the configured number of exploratory moves if it still
  cannot find the cup.

Use `--allow-explore` only on open floor. This is still not full SLAM or global
path planning; it is a small measurable step toward autonomous search.

Search defaults to `0.30 s` turn pulses and a `0.15 s` stopped-camera settling
period. This prevents the detector from inferring on every frame while the
camera is rotating. The values can be changed with
`--search-turn-pulse-s` and `--search-camera-settle-s`.

If a positive camera error (`dir=right`) makes the robot turn away from the
target, add `--swap-steering`. This preserves camera-frame directions in logs
while swapping the left/right drivetrain command and physical-side trim
assignments. Verify calibration by checking that the magnitude of `error_x`
decreases after a steering command and that both wheels start during centered
forward motion.

For a diagnostic scan, add `--save-search-frames`. Every sampled camera image,
annotated detector image, and detector JSON file is then preserved under
`~/sensor-tests/red-cup-search-frames/`. Without this option, the normal debug
files are overwritten and describe only the final sampled frame.

## Step 3: COCO SSD Cup Detection + Red Filter

The first red-cup follower could chase any red object. The improved behavior can
use a TensorFlow Lite COCO SSD MobileNet model:

1. detect object bounding boxes;
2. keep only boxes labelled `cup`;
3. require the selected cup box to contain enough red pixels;
4. approach that cup while lidar stays in continuous safety mode.

This is the right method for the current hardware: Raspberry Pi 5 plus standard
Pi Camera Module v2. The official Raspberry Pi AI Camera is a different camera
with a Sony IMX500 inference sensor; if we upgrade to that hardware later,
`rpicam-detect` is worth revisiting. For the current camera, use TensorFlow Lite
on the Pi CPU.

Copy the updated controller and setup helper to the Pi:

```powershell
scp pi\red_cup_follow_continuous.py pi\red_cup_search_and_approach.py pi\setup_coco_ssd_tflite.sh pi5@pi5.local:/home/pi5/
```

Install the TFLite runtime and download the COCO SSD MobileNet model:

```bash
bash ~/setup_coco_ssd_tflite.sh
```

If apt does not provide `python3-tflite-runtime`, the helper creates a local
virtual environment at `~/fuse-venv` and installs `tflite-runtime` there. In
that case, use `~/fuse-venv/bin/python` for detector-enabled runs.

Run search-and-approach with the object detector integrated:

```bash
~/fuse-venv/bin/python ~/red_cup_search_and_approach.py \
  --armed \
  --detector-model ~/models/coco_ssd_mobilenet_v1/detect.tflite \
  --detector-labels ~/models/coco_ssd_mobilenet_v1/labelmap.txt
```

Expected output now includes the detector source:

```text
7: APPROACH front=0.68 error_x=140.0 red_pixels=5200 source=detector label=cup dir=right
```

The robot should not chase a red object unless the object detector also labels
that box as `cup`. If the model fails to find the cup, the robot scans or stops
instead of falling back to red-only behavior. Red-only fallback is available but
should be used deliberately:

```bash
~/fuse-venv/bin/python ~/red_cup_search_and_approach.py \
  --armed \
  --detector-model ~/models/coco_ssd_mobilenet_v1/detect.tflite \
  --detector-labels ~/models/coco_ssd_mobilenet_v1/labelmap.txt \
  --fallback-to-red-blob
```

## Failure Modes

| Symptom | Likely cause | Fix |
|---|---|---|
| only one wheel moves | loose motor wire or one side below stall threshold | run `left_right_motor_check.py`; reseat motor outputs |
| lidar starts/stops repeatedly | using old short-capture script | use `lidar_front_stream` helper |
| robot searches forever | camera lost red cup | place cup farther away; lower camera angle; improve lighting |
| robot stops too late | threshold too low / coasting | increase `--stop-distance-m` to `0.254` |
| robot is too fast | motor speed too high | lower `--forward-speed`, but keep above stall threshold |
| no red target found | threshold/lighting issue | inspect `~/sensor-tests/red-cup-continuous-detection.jpg` |
| scan rotates the wrong way | drivetrain polarity or scan direction mismatch | rerun with `--scan-direction left` |
| `dir=right` moves the target farther right | physical motor-side mapping is reversed | add `--swap-steering` |
| scan turn stalls one wheel | in-place turn speed below stall threshold | try `--scan-turn-speed 0.62` |
| scan passes the cup without detecting it | camera is moving or scan steps are too large | use the turn-stop-capture controller; reduce `--search-turn-pulse-s` |
| exploration feels unsafe | open-loop moves are too long | lower `--max-explore-moves`, omit `--allow-explore`, or reduce `--explore-forward-s` |
| model import fails | TensorFlow Lite runtime missing | run `bash ~/setup_coco_ssd_tflite.sh` |
| visible cup is ignored | confidence too strict or lighting poor | try `--detector-confidence 0.25` and improve lighting |
| red object is detected but not approached | detector did not label it as `cup` | check `~/sensor-tests/red-cup-continuous-detection.json` |
| wrong class labels | label map offset mismatch | try `--detector-label-offset 1` |

## Yellow Tape Measure Search

The same search-and-approach controller can now use a simple yellow blob
detector. This is better than COCO object detection for a yellow tape measure,
because a tape measure is not a reliable COCO class.

Copy the updated controller files to the Pi:

```powershell
scp pi\red_cup_follow_continuous.py pi\red_cup_search_and_approach.py pi5@pi5.local:/home/pi5/
```

Run a no-motion detection check first:

```bash
python3 ~/red_cup_search_and_approach.py \
  --detect-only \
  --color-target yellow \
  --min-red-pixels 300
```

Expected result:

```text
TARGET_SELECTED cx=... cy=... error_x=... pixels=...
```

Inspect the debug image if needed:

```text
~/sensor-tests/red-cup-continuous-detection.jpg
```

Then run a cautious scan-and-approach test:

```bash
python3 ~/red_cup_search_and_approach.py \
  --armed \
  --swap-steering \
  --color-target yellow \
  --min-red-pixels 300 \
  --stop-distance-m 0.35 \
  --forward-speed 0.75 \
  --scan-turn-speed 0.85 \
  --arc-slow 0.72 \
  --arc-fast 0.85 \
  --search-turn-pulse-s 0.45 \
  --search-camera-settle-s 0.20 \
  --scan-max-s 30 \
  --max-run-s 45 \
  --save-search-frames
```

The log should show `source=yellow_component` and `label=yellow_blob` once the
tape measure is selected. Keep a hand near the motor-battery switch. If the
motors do not move even at these speeds, stop and debug `VM/GND/STBY` or motor
wiring before changing perception thresholds.

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
