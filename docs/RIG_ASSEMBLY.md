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

## Review of the Current Tabletop Arrangement

The annotated photo shows:

- approximately 5 inches from the lidar center to the camera;
- approximately 4.5 inches from the lidar center to a point on the Raspberry Pi;
- a diagonal camera-to-lidar placement in the photographed view.

Only the lidar-to-camera measurement contributes to sensor extrinsics. The
lidar-to-Pi distance is useful for packaging but does not affect geometric
fusion.

The photographed arrangement is not yet a rig:

- the lidar, Pi, and camera are not fixed to one plate;
- the camera appears to be supported by its PCB edge and ribbon cable;
- the Pi has exposed conductive surfaces;
- the cables have no strain relief;
- moving any component would change the geometry.

Do not carry or calibrate this arrangement.

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

## Simple Prototype Layout

Use one nonconductive base plate approximately 200 x 150 mm (8 x 6 inches) or
larger:

1. mount the lidar near the rear/center of the plate;
2. mount the camera at the front edge in a rigid camera bracket;
3. mount the Pi beside or behind the camera on nylon standoffs;
4. keep the camera lens and all mounting hardware below the lidar scan plane;
5. secure the USB adapter board and both cables to the plate;
6. add a handle underneath or behind the sensors.

Suitable prototype plate materials include plywood, acrylic, polycarbonate, or
a rigid 3D-printed plate. Cardboard is acceptable only for planning hole
locations, not for calibration or room capture.

Use screws and standoffs where possible. Reusable hook-and-loop straps may hold
the power bank or cable slack, but must not locate the camera or lidar.

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
