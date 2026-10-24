# Relative-angle (visual-servo) CAN output

Vision reports the target's **newly observed angular error in each image**
without requiring motor encoders or IMU orientation. This is a measurement,
not an absolute target position or a mandatory movement step. Firmware selects
its own position/velocity control law and must never repeatedly accumulate the
last observation on a timer.

**October 24, 2026 update:** handshake, sessions, remote enable/color, and board
acceptance are specified in `vision_cboard_protocol.md`,
`vision_cboard_acceptance.md`, and `vision_cboard_handoff.md` in this directory.
The entry point defaults to v1. Live IDs must be explicit and confirmed;
`0xFE` below is an example, not an approved hardware assignment.

Paths in this document are relative to `/home/irm/iRM-Vision-2026` unless absolute.

## 1. Relative versus legacy absolute output

| Property | `can_command_mode: absolute` | `can_command_mode: relative` |
|---|---|---|
| ID | `send_canid`, e.g. 0xff in standard4 | Explicit `relative_canid`, agreed with firmware |
| Meaning | World-frame absolute yaw/pitch | Target Δyaw/Δpitch relative to the selected reference axis |
| Feedback dependency | Pose feedback for the existing world-frame pipeline | No motor/IMU orientation required for camera-optical output |
| Source | Aimer, EKF and ballistic prediction | CurrentRelativeEstimator by default; legacy VisualServo tracker path separately available |
| Temporal meaning | Estimated/predicted target state | Observation of this image, no prediction |
| Lost target | control=0, angles zero | control=0, errors forced to zero |
| Expired observation | Not this interface's contract | Invalid observation, control=0 |

`CBoard::send()` selects one format, never both. Existing CBoard callers default
to absolute if the mode key is absent; the dedicated relative entry point validates
that relative mode was explicitly selected.

## 2. Reference frames: do not conflate optical and gimbal errors

`relative_output_frame` selects the frame; the older `relative_reference` key
remains compatible but has lower precedence. Startup logs identify the selection.

### 2.1 `camera_optical` — default, no hand–eye extrinsics required

The origin is the camera optical center; the reference is optical +z. PnP uses
the four armor corners, `camera_matrix`, and `distort_coeffs` to obtain camera
coordinates (x right, y down, z forward). Convert to forward-left-up:

```text
(x_f, y_f, z_f) = (z_cam, -x_cam, -y_cam)
Δyaw   = atan2(y_f, x_f)                  # positive left
Δpitch = -atan2(z_f, sqrt(x_f² + y_f²))   # negative up
```

This measures target displacement from the optical axis and is the correct
choice on this machine before hand–eye calibration.

### 2.2 `gimbal` — calibrated extrinsics required

Transform first, then use the same angular formulas:

```text
xyz_gimbal = R_camera2gimbal * xyz_camera + t_camera2gimbal
```

Missing `R_camera2gimbal` or `t_camera2gimbal` raises a construction error, not
a silent optical-frame fallback. The two definitions coincide only when the
camera optical center is at the gimbal origin and its optical axis aligns with
gimbal +x. The approximately [95,95,51] mm translation in the demo config can
produce meaningful near-range differences. A synthetic unit test explicitly
demonstrates that identity extrinsics do not reproduce camera-optical errors.

Neither output is a world absolute angle. The up-negative pitch convention
matches the existing Aimer/CBoard convention (`pitch = -trajectory.pitch`).
If firmware uses up-positive pitch, invert in **one place only**: vision sign
configuration or firmware, never both.

## 3. Relative CAN payload

Classical CAN, standard ID, DLC=8; multibyte values are big-endian.

```text
ID: relative_canid (explicit and confirmed; 0xFE is only an example)
byte0: bit0 = control (1: valid current observation; 0: invalidate measurement)
       bits1..7 = 0, including shoot
byte1: 0, reserved
byte2..3: Δyaw, signed int16, 1e-4 rad units
byte4..5: Δpitch, signed int16, 1e-4 rad units
byte6..7: 0, reserved; no legacy UAV horizontal-distance field
```

Example: yaw=+0.1234 rad, pitch=-0.0567 rad, control=1:

```text
ID 0xFE data = 01 00 04 D2 FD C9 00 00
```

Resolution is 1e-4 rad ≈0.0057°. Out-of-range values saturate at +32767/-32768
rather than wrapping (approximately ±3.2768 rad). Quantization does not accumulate
over time: the next image produces a new observation. Existing absolute-angle
int16 decoding may be reused, but the **semantics become measured relative error**.

### 3.1 Encoder-enforced safety

`io::encode_relative_command()` and `io::quantize_angle_1e4()` are the shared
encoding/quantization implementations used by CBoard and estimator TX fields.

| Input | Encoded result |
|---|---|
| control=0, even with nonzero supplied angles | All-zero stop |
| NaN/Inf yaw or pitch | Clear control, zero angles |
| Finite angles outside int16 range | Saturation, no wraparound |
| Any shoot request | shoot forced to zero in relative mode |

`CBoard::send()` returns bool; success requires a complete SocketCAN frame write.
`SocketCAN::write()` checks `sizeof(can_frame)` and reports errno. This is kernel
acceptance, not a board ACK. Legacy callers may ignore the result; relative_aim
records link successes/failures. CSV `publish_ok` is publication, not bus delivery.

### 3.2 Firmware decoding illustration — not motion-control code

```c
/* Decode only after checking standard-frame ID, DLC, reserved bits,
   current session, local enable, and watchdog state. */
void on_relative_cmd(const uint8_t data[8]) {
    if (!(data[0] & 0x01)) {
        /* Invalidate the stored measurement; never continue with old angles. */
        return;
    }
    int16_t dyaw = (int16_t)((data[2] << 8) | data[3]);
    int16_t dpitch = (int16_t)((data[4] << 8) | data[5]);
    float delta_yaw = dyaw / 10000.0f;
    float delta_pitch = dpitch / 10000.0f; /* radians, negative up */
    /* Pass the measurement to the firmware's own limited controller.
       Clear validity on stop, lost target, or receive timeout. */
}
```

Update each measurement once per received frame. Never add it repeatedly on
a timer, or directly add it to current/torque. Implement a receive watchdog
(e.g. 100 ms, tuned for deployment), plus position/velocity/acceleration limits.
Account for image and actuation latency. A stop must invalidate stored measurements.
Session/enable gating remains mandatory because the angle frame has no session.

## 4. Vision configuration

### 4.1 Entry point / CAN

| Key | Default | Meaning |
|---|---|---|
| `can_command_mode` | `absolute` | Dedicated entry requires explicit relative |
| `relative_canid` | No implicit entry-point default | Standard ID 0..0x7FF, collision-free and confirmed |
| `cboard_wait_imu` | true | Must be false for this no-IMU relative entry; imu_at returns identity |
| `vision_protocol` | v1 | Session/control protocol; explicit legacy has no remote enable/color |
| `vision_can_ids_confirmed` | false | Required outside rtnetlink-verified vcan |
| `relative_send_interval_ms` | 0 | Minimum interval for valid sends; zero means every eligible image; stops bypass it |
| `relative_tx_rate_hz` | 0 | Positive value takes precedence over the interval |

The three handshake/control IDs are explicit in v1; see the v1 protocol for
collision rules. `cboard_wait_imu: false` changes only quaternion waiting and
identity pose behavior; other CBoard telemetry is still parsed.

### 4.2 Current-frame geometry and freshness

| Key | Default | Meaning |
|---|---|---|
| `relative_output_frame` / compatibility `relative_reference` | camera_optical | Optical or calibrated gimbal frame |
| `relative_max_frame_age_ms` | 120 | Geometry-path age limit; expired images are stale/invalid |
| `relative_frame_timeout_ms` | 100 (120 in current configs) | Independent v1 observation timeout |
| `vision_control_timeout_ms` | 500 | Session control-heartbeat timeout |
| `relative_min_confidence` | min_confidence | Additional confidence filter |
| `relative_max_reprojection_px` | 15 | Maximum mean PnP reprojection error |
| `relative_min_depth_m` | 0.1 | Minimum positive camera depth in meters |
| `relative_filter_color` | true when enemy_color exists | Retain the selected enemy color |
| `enemy_color` | absent/no configured filter | red / blue / extinguish / purple for standalone selection; v1 CONTROL supports red/blue |

### 4.3 Signs, offsets, and optional legacy shaping

`relative_tx_shape` defaults false: current measurements receive signs, static
offsets, and quantization only. Gain/deadband/step limits require explicit true.
The current and legacy helpers share `load_servo_limits()` configuration.

| Key | Default | Meaning |
|---|---|---|
| `servo_gain` | 1.0 | Legacy proportional shaping factor |
| `servo_deadband_deg` | 0.1 | Legacy deadband; use zero for unmodified comparison |
| `servo_max_step_deg` | 20 | Legacy per-frame magnitude limit, not mechanical safety |
| `servo_yaw_sign` / `servo_pitch_sign` | 1 | Set -1 if the agreed sign requires inversion |
| `servo_yaw_offset_deg` / `servo_pitch_offset_deg` | 0 | Static alignment offsets; pitch may approximate drop but is not a ballistic solver |

## 5. Build, inspect, and integrate

```bash
cd /home/irm/iRM-Vision-2026
make -C /home/irm/iRM-Vision-2026/build relative_aim relative_servo_test camera_test -j4
/home/irm/iRM-Vision-2026/build/relative_servo_test
# Offline geometry/HUD/CSV; opens no CAN socket:
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current_replay.yaml -dry-run -n=150 -csv=/tmp/r.csv -stream=8080
```

For hardware, complete the current handoff first: approve IDs, bitrate, signs,
watchdogs, and motion protection; initially prohibit actuation and inspect
traffic independently. A legacy capture example is `candump -tz can0,0FE:7FF`;
replace 0FE with the actual approved angle ID and capture the control IDs too.
Do not transmit on a physical bus without the required peers/ACK arrangement.

Normal SIGINT/frame-limit exit attempts a best-effort stop with bounded retries.
Camera starvation/observation expiry stops independently in v1; an industrial
queue read checks SIGINT after at most 50 ms. File decoding/SDK destruction has
no hard deadline. Failed sends are counted/reported; SocketCAN can reconnect.
Never rely on shutdown delivery instead of the firmware watchdog.

Without quaternion traffic, `cboard_wait_imu: false` is mandatory; the entry
validates it before constructing CBoard rather than hanging at Waiting for q.
Use `-c=<path>`, not `-c <path>`. Exit live cameras with Ctrl+C, not default SIGTERM.

## 6. Tuning order

1. Confirm reference axis and yaw/pitch signs. Target left means positive yaw;
   up means negative pitch under the documented convention. Invert only once.
2. Start with firmware-side low-speed/position/acceleration limits and conservative
   gains. Verify that motion reduces the observed error.
3. Tune firmware deadband/filtering to image noise and end-to-end latency.
4. Only if explicitly choosing legacy shaping, set `relative_tx_shape: true`,
   then tune deadband (historically 0.1°, or 0.2–0.3° for noise), gain (historical
   starting point 0.3), and magnitude limit. These are not default measurement behavior.
5. Apply static offsets only after alignment verification. They do not replace
   hand–eye calibration or range-dependent ballistic compensation.

## 7. Known limitations

1. No ballistic lead/time-of-flight prediction. Both paths aim at observed armor
   centers; moving/rotating targets can lag. Lead needs a separate estimator or
   a properly transformed prediction with the required pose information.
2. Current-frame geometry has no temporal filtering. Single-image PnP jitter
   reaches output; use an appropriate firmware filter if needed.
3. There is no absolute position reference for fixed-point scanning. After
   resets, the image establishes the new observed relative relationship.
4. No target, wrong color, invalid corners/reprojection, or stale observation
   invalidates output. Physical stopping/holding depends on firmware behavior.
5. Current selection is deterministic but not target-locked: nearest-to-center
   armor can change between images; CSV target IDs expose switches.
6. The legacy tracker requires consecutive detections, suppresses the first
   command after a target switch, and may diverge on jittery footage. Its raw/TX
   reporting includes shaping inside `VisualServo::aim()`. It is not supported
   for live v1 dynamic-color operation.
7. No firing decision: shoot is always zero. Relative error alone is not a
   complete firing interlock or proof of alignment.
8. Image timestamps are host buffer-receipt/file-read times, not exposure times.
   Age measures detection/queuing latency, excluding exposure/transport delay.
9. No session in angle payloads; queued frames cannot be withdrawn. Local firmware
   gating and watchdogs remain necessary. See the v1 protocol's safety boundary.

## 8. Related implementation and records

| File under `/home/irm/iRM-Vision-2026` | Purpose |
|---|---|
| `io/command.hpp` | Command and shared quantizer |
| `io/cboard.{hpp,cpp}`, `io/socketcan.hpp` | Mode dispatch, relative encoding, checked transport |
| `io/vision_cboard.{hpp,cpp}` | v1 session/control link |
| `tasks/auto_aim/current_relative.{hpp,cpp}` | Current-frame filtering, selection, PnP, freshness |
| `tasks/auto_aim/visual_servo.{hpp,cpp}` | Legacy angles/shape/aim helpers |
| `src/relative_aim.cpp` | Entry point, dry-run, CSV, stream |
| `tests/relative_servo_test.cpp` | Software tests |
| `configs/relative_current.yaml` | Actual-camera configuration; IDs still require approval |
| `configs/relative_current_replay.yaml` | Demo replay, not actual-camera intrinsics |
| `documentation/relative_acceptance.md` | Historical cable/geometry/vcan measurements |
| `documentation/handoff_relative_servo.md` | Historical development state and diagnostics |