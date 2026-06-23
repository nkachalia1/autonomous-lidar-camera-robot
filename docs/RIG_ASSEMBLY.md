# Rigid Sensor Rig

## Current Measurement

The approximate distance from the lidar spinner center to the camera optical
center is:

- 5 inches;
- 127 mm;
- 0.127 m.

This is a useful baseline length, but it is not yet a camera-to-lidar transform.
We still need to resolve that distance into forward/back, left/right, and
up/down components after the final mount is tightened.

The preliminary value is recorded in
[`config/rig_measurements.yaml`](../config/rig_measurements.yaml).

## Recommended Arrangement

- Place the lidar above the camera.
- Keep the lidar scan plane horizontal and unobstructed through 360 degrees.
- Point the camera forward, approximately level.
- Keep the camera body and ribbon cable below the lidar scan plane.
- Fix both sensors to the same rigid plate; do not use separate tripods.
- Put the Raspberry Pi in a case or on nonconductive standoffs.
- Strain-relieve the camera ribbon and lidar USB cable.
- Add a handle that does not flex the sensor plate.

A vertical separation of approximately 127 mm is acceptable if that is how the
5-inch measurement is arranged. It gives the camera a clear view while keeping
the lidar scan plane above most nearby rig components.

## Mechanical Pass Criteria

The rig passes when:

1. the camera cannot rotate or slide when gently pushed;
2. the lidar base cannot rotate relative to the camera;
3. no cable can enter the lidar rotor or scan plane;
4. the Pi cannot contact exposed metal;
5. the rig can be lifted by its handle without visible flex;
6. the lidar remains approximately level when held naturally;
7. the camera view is not blocked by the lidar or mounting plate.

Do not use tape or a flexible camera ribbon as a structural restraint.

## Measurement Procedure

After assembly, power the rig off and measure from the lidar measurement center
to the approximate camera lens center.

Use the lidar frame:

- `+x`: lidar 0-degree direction;
- `+y`: left when looking in the lidar 0-degree direction;
- `+z`: upward.

Record:

| Measurement | Meaning |
| --- | --- |
| `camera_forward_m` | Camera forward/back relative to lidar center |
| `camera_left_m` | Camera left/right relative to lidar center |
| `camera_up_m` | Camera above/below lidar center |
| roll | Camera clockwise tilt when looking forward |
| pitch | Camera pointing up/down |
| yaw | Camera pointing left/right |

Negative values indicate the opposite direction. For example, a camera 127 mm
below the lidar has `camera_up_m: -0.127`.

The three translation components should approximately satisfy:

```text
sqrt(forward² + left² + up²) ≈ 0.127 m
```

## Evidence Needed

Take three photos after tightening the mount:

1. front view;
2. side view;
3. top view.

Include a ruler or tape measure in each useful measurement direction. Also note
whether the rig will be handheld or placed on a cart.

These measurements establish an initial transform. Formal calibration will
refine it later.

