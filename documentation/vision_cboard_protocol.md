# Vision ↔ CBoard v1 protocol — October 24, 2026

## 1. Scope and safety boundaries

Vision reports target Δyaw/Δpitch relative to the selected reference axis in
the **current image**. It does not predict, require motor pose, or define the
firmware control law. Output is not an absolute position, velocity command, or
step that must be accumulated once per frame. The encoder always forces `shoot=0`.

The default is `camera_optical`: camera optical center as origin and optical
axis as reference. Yaw is positive left; pitch is negative up, in radians.
`gimbal` output requires calibrated hand–eye extrinsics and must not be described
as optical-axis error.

Use Classical CAN, standard 11-bit IDs, DLC=8. Extended, RTR, error, and incomplete
frames are rejected. All four IDs must be distinct and must not occupy the
quaternion, bullet-speed, or legacy absolute-command IDs. ID 0x000 is valid.
The configured `0xFE/0x310/0x311/0x312` values are candidates, **not approved
hardware assignments**. There is no authentication, replay protection, or bus
access control. A session separates restarts/control contexts; it is not a security token.

## 2. Wire format

All multibyte integers are big-endian.

| Direction / configured ID | byte0 | byte1 | byte2..5 | byte6 | byte7 |
|---|---|---|---|---|---|
| Firmware→vision `vision_handshake_request_canid` | 01 HELLO | 01 version | session uint32 | 00 | 00 |
| Vision→firmware `vision_handshake_response_canid` | 02 ACK | 01 version | echoed session | status | 00 |
| Firmware→vision `vision_control_canid` | 03 CONTROL | 01 version | session uint32 | flags | seq uint8 |

ACK status: 0 READY, 1 NOT_READY, 2 VERSION_MISMATCH.
Flags = enable(bit0) | color(bits1..3), with color=1 red, 2 blue. Bits4..7 must
be zero. Legal flags: red stop=02, red enable=03, blue stop=04, blue enable=05.
A stop must also specify a legal color.

Increment `seq` for every CONTROL transmission, modulo 256. The first value is
arbitrary; subsequent accepted forward differences are 1..127. Duplicate and
out-of-order CONTROL frames neither change state nor refresh the heartbeat.
Create a new session when resetting the sequence.

Relative-angle frames use `relative_canid` and have no message-type byte:

| Bytes | Meaning |
|---|---|
| 0 | bit0 control: 1=current observation valid, 0=invalidate measurement; all other bits zero, including shoot |
| 1 | 00 reserved |
| 2..3 | Δyaw signed int16 BE, units 1e-4 rad |
| 4..5 | Δpitch signed int16 BE, units 1e-4 rad |
| 6..7 | 00 00 reserved |

Example session=0x12345678: HELLO `01 01 12 34 56 78 00 00`; READY ACK
`02 01 12 34 56 78 00 00`; blue enable, seq=7: `03 01 12 34 56 78 05 07`.
Valid yaw=0.1234, pitch=-0.0567: `01 00 04 D2 FD C9 00 00`.
A stop is eight zero bytes. NaN/Inf also encode as a stop. Out-of-range values
saturate at int16 limits rather than wrapping.

**Relative-angle frames contain no session/sequence.** Firmware must gate them
locally and clear receive caches/valid measurements on stop or session change.
Software cannot identify old valid angles already in kernel or hardware TX
queues. Absolute wire-level session isolation requires a new payload layout;
v1 must not be claimed to provide it.

## 3. Handshake and state machine

1. On startup/restart, firmware generates a new session, prohibits motion, and
   periodically sends HELLO; a 100 ms retry interval is suggested.
2. Until the model, geometry configuration, and first fresh nonempty BGR image
   are ready, vision replies NOT_READY and creates no session. Unknown versions
   receive VERSION_MISMATCH. Malformed HELLO, including reserved bytes, is discarded.
3. READY enters `wait_control`. Firmware validates ACK ID, version, session,
   status, and reserved bytes, then sends incrementing CONTROL approximately
   every 80 ms, including while stopped, below the 500 ms vision control timeout.
4. enable=1→`active`; enable=0→`stopped`. Only active permits valid observations.
   Same-session HELLO is idempotent: it does not clear color/enable or refresh
   the heartbeat. A new session invalidates old samples; if valid output was
   previously sent, a stop precedes the ACK.
5. Control timeout in `wait_control`, `active`, or `stopped` invalidates the
   session. A new HELLO is required.
6. Malformed version, DLC, type, reserved bits, or illegal color on the CONTROL
   ID immediately fail closed: invalidate the session without refreshing the
   heartbeat. Structurally legal old-session CONTROL is simply rejected.
   Unrelated IDs do not affect state.

## 4. Vision processing and concurrency

The current path filters color/confidence/keypoints, sorts deterministically by
image-center distance, tries at most eight geometric candidates, runs PnP using
the selected armor's own four corners, then applies signs/static offsets and
1e-4 rad quantization. With default `relative_tx_shape: false`, gain, deadband,
and step limits do not alter measurements. Explicit true enables legacy shaping.
Live v1 supports only current; tracker lacks dynamic color and is rejected at startup.

The receive callback only copies into a bounded queue of capacity 64. Overflow
clears the queue and invalidates the session. An independent 5 ms thread processes
reception and transmission. Each valid sample has at most one send attempt; old
angles are not repeatedly transmitted. Stops/invalid observations bypass valid
output rate limits. Failed stops remain pending for retry. Normal shutdown makes
at most five stop attempts; each SocketCAN write has a 20 ms timeout.
A successful write means kernel acceptance, **not firmware reception/execution ACK**.

Generation is captured before image acquisition. Session/enable/color changes
increment it, rejecting in-flight old detections. Cached images captured before
the control change are also rejected. Transmission freshness uses **image time**,
not detection completion, which must not rejuvenate an old image. Observation
expiry independently sends a stop without waiting for another image. Production
transmission holds the state lock through eligibility checking and bounded
write; receive-queue processing and transmission are serialized. Frames already
submitted to the kernel cannot be withdrawn.

Industrial-camera queue reads wait at most 50 ms before checking SIGINT.
Local-file decoding and SDK destruction have no hard real-time deadline guarantee.
SIGINT shutdown attempts a best-effort stop. SIGKILL, power loss, process crashes,
and bus disconnection require a firmware-local watchdog.

## 5. Configuration and diagnostics

The entry point defaults to `vision_protocol: v1`. Explicit `legacy` has no
handshake/remote color/enable control and cannot bypass hardware-ID confirmation.
Both modes require explicit `relative_canid`. `vision_can_ids_confirmed: true`
records manual confirmation of all actual IDs. Only rtnetlink-verified kind=vcan
is exempt; loopback, Docker, and unknown interfaces are not.

`relative_output_frame: camera_optical|gimbal` selects coordinates only and takes
precedence over compatibility key `relative_reference`. Live transmission
requires `can_command_mode: relative` and `cboard_wait_imu: false`.
`vision_control_timeout_ms` defaults to 500; `relative_frame_timeout_ms` defaults
to 100, with 120 in current configs. `relative_tx_rate_hz` defaults to 0/unlimited
and, when positive, takes precedence over `relative_send_interval_ms`.
Timeouts must be finite and positive; rates/intervals finite and nonnegative.

Dry-run constructs no CBoard and opens no CAN socket. `-dry-enable=0|1` and
`-dry-color=red|blue` explicitly simulate enable/color. CSV TX fields are encoded
measurements; `publish_ok` is not successful bus transmission. Inspect link
statistics and independent captures for actual sends. `tx_frames` counts all
relative/stop frames between CSV rows, not a one-to-one send result for that row.

## 6. Two-end integration table

Only `src/relative_aim.cpp` (target `relative_aim`) loads the keys in this section.
`src/uav.cpp`, `src/uav_debug.cpp`, `src/sentry*.cpp`, `src/standard*.cpp` and
`src/auto_buff_debug*.cpp` still call `CBoard::send()` in `absolute` mode via
`send_canid`. Editing `configs/relative_current.yaml` does not migrate them.

### 6.1 Wire, keys, and direction

Candidate values are the ones currently in `configs/relative_current.yaml`. They are
**not** approved hardware assignments: `vision_can_ids_confirmed` stays `false` until
the whole bus ID map is agreed with the MCU. Do not shift them onto the MCU's
`0x320–0x322` block (`board_state_can_protocol.md`) without that confirmation.

| Direction | Config key | Candidate ID | Payload (DLC 8, big-endian) | When |
|---|---|---|---|---|
| MCU → vision | `vision_handshake_request_canid` | `0x310` | HELLO `01 01 session(4) 00 00` | on boot/restart, retry ≈100 ms |
| vision → MCU | `vision_handshake_response_canid` | `0x311` | HELLO_ACK `02 01 session(4) status 00` | one per HELLO |
| MCU → vision | `vision_control_canid` | `0x312` | CONTROL `03 01 session(4) flags seq` | heartbeat while enabled |
| vision → MCU | `relative_canid` | `0xFE` | angle frame, see §2 | ≤1 per valid observation, plus stops |
| MCU → vision | `quaternion_canid` | `0x100` | `w,x,y,z` int16 ÷1e4 | legacy telemetry; unused by this path |
| MCU → vision | `bullet_speed_canid` | `0x101` | speed/mode/shoot-mode/ft_angle | legacy telemetry |

### 6.2 Shared numeric contract and who owns what

| Key | Default | Vision behaviour | MCU must also do |
|---|---|---|---|
| `vision_control_timeout_ms` | 500 | expire session, send stop, require new HELLO | keep CONTROL flowing while enabled |
| `relative_frame_timeout_ms` | 120 | send stop when the observation expires | own receive watchdog (≈100 ms) |
| `relative_tx_rate_hz` / `relative_send_interval_ms` | 0 / 0 | optional rate cap; **stops bypass it** | never assume a fixed frame rate |
| `relative_tx_shape` | false | pure measurement (sign + static offset + quantization only) | implement all gain/deadband/limits itself |
| `vision_can_ids_confirmed` | false | refuse to transmit on non-vcan interfaces | record the approved ID map |
| `relative_output_frame` | `camera_optical` | references the optical axis, no extrinsics | treat Δ as error vs. its own reference axis |

### 6.3 MCU-side gating checklist (all mandatory)

1. Accept only Classical CAN, standard 11-bit IDs, `DLC == 8`; drop extended/RTR/error frames.
2. Match the configured ID exactly; unknown IDs change no state.
3. Domain-separate by ID: HELLO / CONTROL / angle frames are three different IDs, not one
   with a type byte (only HELLO, ACK, CONTROL carry a message-type byte).
4. CONTROL: accept only the **current** session; a foreign session changes nothing and does
   not refresh the heartbeat.
5. CONTROL: `seq` forward difference must be 1..127 (mod 256); duplicates and reordering are
   ignored. A new session resets `seq`.
6. CONTROL: `flags` bits 4..7 must be zero and colour must be 1 (red) or 2 (blue); a stop must
   still carry a legal colour. Anything else fails closed immediately (do not wait for timeout).
7. Control heartbeat watchdog: on expiry, stop actuating on vision data and invalidate the
   stored measurement.
8. Angle frames have **no session field**: gate them on local session/enable/colour plus the
   receive watchdog. Never "verify" the session from the angle payload — it is not there.
9. `control=0` or eight zero bytes → invalidate the stored measurement at once; never keep
   driving on the previous angles.
10. Consume each frame once. Never accumulate the last observation on a timer or add it
    directly to current/torque.
11. No firing decision: relative frames force `shoot = 0`, and `byte1` is not a shoot flag.

### 6.4 Repeatable vision-side acceptance

`scripts/vision_cboard_sim.py` is an independent MCU peer for a verified `vcan`
interface (Python standard library only). It derives all IDs from the yaml, sends
periodic HELLO, enables/disables, switches colour and session, drops the CONTROL
heartbeat, decodes and prints every angle frame with its receive time, and asserts:

- no valid angle frame before handshake + enable;
- disable / colour change / malformed CONTROL stop valid output immediately;
- CONTROL silence expires the session, and the old session is rejected afterwards;
- a new session re-handshakes and resumes;
- one observation produces at most one valid frame; stops are not rate-limited;
- when image acquisition is throttled below the observation timeout, vision emits an
  independent stop between images;
- every valid wire frame matches the published CSV in order (int16 values included);
- SIGINT shutdown writes a stop frame.

See `documentation/relative_acceptance.md` for recorded runs and
`scripts/run_local_tests.sh` cases 10–11 for the local invocation.
