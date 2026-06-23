# Decision Log

## ADR-001: Raspberry Pi OS Capture, Windows Reconstruction

Status: accepted for the room MVP.

Decision:

- retain 64-bit Raspberry Pi OS for sensor acquisition;
- use the supported Raspberry Pi camera stack;
- record timestamped raw sessions;
- transfer sessions to the Windows workstation for reconstruction;
- reconsider Ubuntu 24.04 and ROS 2 Jazzy for the hallway phase.

Rationale:

- the user already has Raspberry Pi OS installed;
- camera bring-up is lowest risk with Raspberry Pi's supported software;
- dense reconstruction is iterative and more practical offboard;
- ROS 2 Jazzy binary packages officially target Ubuntu 24.04, not Raspberry Pi
  OS;
- no operating-system migration should occur before basic sensor health is
  established.

Consequences:

- the first recorder will not require ROS 2;
- data formats and coordinate frames must be explicit enough to bridge to ROS 2
  later;
- real-time dense reconstruction is out of scope for the room MVP.

## ADR-002: Loose Coupling Before Tight Coupling

Status: accepted.

Decision:

1. estimate the lidar and visual trajectories independently;
2. align them by timestamps and calibrated rigid geometry;
3. use lidar to establish metric horizontal scale;
4. add joint optimization only after the loose-coupled pipeline is measured.

Rationale:

- each sensor can be debugged independently;
- calibration and timestamp failures remain visible;
- the first useful room model requires much less custom optimization code;
- preserved raw sessions allow later algorithm upgrades.

