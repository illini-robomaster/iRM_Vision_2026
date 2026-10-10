# CBoard Compatibility Requirements Checklist

## 1. Scope

This document specifies compatibility with the repository's current
`io::CBoard` CAN interface. It does **not** require a particular board model or
IMU chip. Board capabilities remain unverified until the firmware team completes
the checklist and hardware testing.

| Application | CBoard dependency |
| --- | --- |
| `intrinsic_calibration_gui` | **None**: camera only; no pose provider is constructed |
| Offline `calibrate_camera` | **None**: image files only |
| Existing `capture` | Continuous quaternion input required by the implementation |
| Offline hand–eye calibration | No live board; recorded image/quaternion pairs required |
| Live `handeye_test` | Continuous quaternion input required |
| Applications using CBoard for auto-aim | Orientation, application-specific telemetry, command reception |

The existing `capture` waits for orientation before starting the camera. This
is an implementation dependency, not a mathematical requirement of intrinsic
calibration. The new intrinsic GUI bypasses this dependency. Its future hand–eye
interface stores optional wxyz orientation and image timestamps, but hand–eye
solving and CBoard integration are not enabled.

## 2. Minimum calibration-capture checklist

Fill each status with **Supported / Missing / Needs adaptation**.

| ID | Requirement | Board status |
| --- | --- | --- |
| CAL-01 | Provide a fused orientation quaternion; raw gyro/accelerometer data alone does not match this interface. | |
| CAL-02 | Continuously stream quaternion frames without a per-sample host request. | |
| CAL-03 | Match the payload in Section 3, or implement a firmware/host adapter. | |
| CAL-04 | Use the configured quaternion CAN ID. | |
| CAL-05 | Document quaternion direction, axis directions, handedness and attitude reference. | |
| CAL-06 | Keep the IMU rigidly mounted relative to the camera/gimbal during hand–eye acquisition. | |
| CAL-07 | Avoid unexpected attitude-reference resets within one acquisition session. | |
| CAL-08 | Provide stable orientation estimates for stationary captures. | |

Motor control, firing control, bullet-speed feedback and referee-system
integration are **not required for calibration capture**. Hand–eye acquisition
also assumes a fixed board in the world and a stationary gimbal origin: the
current solvers set gimbal/world translation to zero.

## 3. Transport and orientation protocol

### Transport

- Host transport: Linux SocketCAN, normally `can0`.
- Classical CAN data frame with an **8-byte payload**, not a CAN FD payload.
- Standard IDs without extended-ID flags for drop-in compatibility with the
  current direct `can_id` comparisons.
- Host and board bitrate must match. The C++ code does not configure bitrate.
  The repository includes a 1 Mbit/s setup example; confirm actual deployment.
- Valid DLC and frame type must be guaranteed by the sender: the current CBoard
  callback does not validate DLC before decoding eight bytes.

### Quaternion frame: CBoard → Host

CAN ID: `quaternion_canid` in the selected YAML.

| Bytes | Component | Encoding |
| --- | --- | --- |
| 0–1 | qx | Signed 16-bit, big-endian |
| 2–3 | qy | Signed 16-bit, big-endian |
| 4–5 | qz | Signed 16-bit, big-endian |
| 6–7 | qw | Signed 16-bit, big-endian |

```text
decoded component = signed integer / 10000.0
rejection condition = abs(qx² + qy² + qz² + qw² - 1) > 0.01
```

Identity orientation is transmitted as `00 00 00 00 00 00 27 10`.
**CAN order is xyzw; legacy calibration TXT order is wxyz.**

### Coordinate convention

The quaternion is interpreted as **IMU body → IMU reference**:

```text
R_imubody2imuabs = quaternion.toRotationMatrix()
R_gimbal2world = transpose(R_gimbal2imubody)
                 * R_imubody2imuabs * R_gimbal2imubody
```

`R_gimbal2imubody` is a supplied configuration matrix, not automatically
estimated by the calibration tools.

- [ ] Confirm body-to-reference quaternion direction (not its inverse).
- [ ] Document positive X/Y/Z and positive rotation directions.
- [ ] Document attitude-reference initialization and reset behavior.
- [ ] Verify yaw/pitch/roll changes on the host against known physical motions.
- [ ] Provide the correct host-side mounting/axis mapping matrix.

## 4. Timing and streaming

| Item | Current host behavior |
| --- | --- |
| Startup | Blocks until two valid quaternion samples arrive |
| Image alignment | Host receive timestamps plus quaternion SLERP |
| Firmware timestamp | Not transmitted/decoded by this payload |
| Stream continuity | Required; interpolation waits for a sample later than the image timestamp |
| Minimum rate | No numerical threshold is enforced by the code |
| Latency | Receive time is not necessarily sensor measurement time |
| Stream loss | Queue API has no bounded wait timeout or complete fail-safe handling |

Please measure/report:

| Property | Board value |
| --- | --- |
| IMU sampling rate | ___ Hz |
| Attitude-estimator update rate | ___ Hz |
| Quaternion CAN transmission rate | ___ Hz |
| Measurement-to-transmission latency | ___ ms / Unknown |
| Transmission interval jitter | ___ ms / Unknown |
| Stationary attitude drift | ___ / Unknown |
| Behavior after IMU or CAN failure | ___ |

No fixed transmission rate is inferred from the existing code. The necessary
rate depends on camera timing and motion. Stationary captures reduce sensitivity
to timing error. Hardware timestamps, acknowledgements, sequence numbers and
application-level checksums are not part of the current eight-byte protocol.

## 5. Additional telemetry for CBoard-based auto-aim

Not required for intrinsic/hand–eye calibration. CAN ID: `bullet_speed_canid`.

| Bytes | Field | Decoding |
| --- | --- | --- |
| 0–1 | Bullet speed | Signed int16, big-endian, /100; m/s |
| 2 | Operating mode | Byte enum below |
| 3 | Shooting mode | Byte enum below |
| 4–5 | `ft_angle` | Signed int16, big-endian, /10000; radians, UAV-specific |
| 6–7 | Reserved | Not used by current decoder |

| Operating-mode value | Meaning |
| --- | --- |
| 0 | Idle |
| 1 | Auto aim |
| 2 | Small buff |
| 3 | Big buff |
| 4 | Outpost |

| Shooting-mode value | Meaning |
| --- | --- |
| 0 | Left shoot |
| 1 | Right shoot |
| 2 | Both shoot |

- [ ] Bullet-speed telemetry available if the selected application requires it.
- [ ] Mode values match host definitions.
- [ ] Shooting-mode/UAV fields supported where applicable.
- [ ] Enum values are always valid: current host code does not range-check them
  before indexing mode-name arrays.

## 6. Command reception for CBoard-based auto-aim

Host → CBoard, CAN ID `send_canid`, DLC 8.

| Bytes | Field | Encoding |
| --- | --- | --- |
| 0 | Control enable | 0 or 1 |
| 1 | Shoot request | 0 or 1 |
| 2–3 | Yaw | Signed int16, big-endian; host value ×10000 |
| 4–5 | Pitch | Signed int16, big-endian; host value ×10000 |
| 6–7 | Horizontal-distance field | Signed int16, big-endian; host value ×10000; UAV-specific |

Yaw/pitch follow the repository's radian-based convention. Confirm angle
reference, signs and wrapping against the selected application before enabling
motion. The signed int16 /10000 encoding represents approximately **−3.2768 to
+3.2767**. The host does not clamp before conversion. In particular, review the
distance field's units, scaling and range before UAV integration.

Firmware safety requirements to confirm (not guarantees of the host code):

- [ ] Define control-disabled behavior.
- [ ] Define level-versus-edge semantics of the shoot request.
- [ ] Implement a command-loss watchdog.
- [ ] Enforce motion limits and firing interlocks.
- [ ] Reject malformed commands.
- [ ] Enter a safe state when communication is lost.

## 7. Configuration compatibility

The current CBoard constructor requires all four keys even for quaternion-only
calibration capture:

```yaml
quaternion_canid: 0x01
bullet_speed_canid: 0x110
send_canid: 0xff
can_interface: "can0"
```

IDs differ between existing configurations:

| Configuration | Quaternion ID | Telemetry ID | Command ID |
| --- | --- | --- | --- |
| Calibration | 0x01 | 0x110 | 0xFF |
| Demo | 0x100 | 0x101 | 0xFF |

Do not assume IDs are universal. Match the actual firmware and selected YAML.
USB/UART-only orientation is **not drop-in compatible with `io::CBoard`**;
it requires an adapter or another host interface. Other repository IMU/gimbal
classes are separate interfaces, not automatic fallbacks for CBoard.

## 8. Firmware-team response form

```text
CAN interface available:
Classical CAN supported:
CAN bitrate:
Quaternion CAN ID:
Quaternion payload format:
Quaternion direction:
IMU axis convention:
Quaternion transmission rate:
Attitude reference/reset behavior:
Bullet-speed telemetry available:
Operating-mode telemetry available:
Command reception available:
Command timeout / fail-safe behavior:
Required firmware changes:
Required host-side adaptations:
```

## 9. Source references and verification limits

- `/home/irm/iRM-Vision-2026/io/cboard.cpp`
- `/home/irm/iRM-Vision-2026/io/cboard.hpp`
- `/home/irm/iRM-Vision-2026/io/socketcan.hpp`
- `/home/irm/iRM-Vision-2026/io/command.hpp`
- `/home/irm/iRM-Vision-2026/tools/thread_safe_queue.hpp`
- `/home/irm/iRM-Vision-2026/calibration/capture.cpp`
- `/home/irm/iRM-Vision-2026/calibration/intrinsic_session.hpp`
- `/home/irm/iRM-Vision-2026/configs/calibration.yaml`
- `/home/irm/iRM-Vision-2026/configs/demo.yaml`

Protocol requirements were checked against source code. No claim is made that
the team's current board firmware already implements them. Real CAN, IMU and
actuation compatibility must be verified separately. Camera-only GUI tests do
not verify CBoard firmware.