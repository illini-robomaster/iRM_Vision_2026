# Jetson ↔ CBoard board-state CAN protocol v1

Date: October 24, 2026

## 1. Scope

This is a Jetson-initiated telemetry session. After a three-message handshake,
CBoard streams the same 29-byte `board_state` payload used by the minipc UART
link: quaternion (wxyz), measured offset-corrected motor yaw/pitch, bullet
speed, and one opaque mode byte.

This protocol does not enable motors, authorize firing, or replace the separate
vision↔CBoard relative-angle control protocol. Telemetry must remain available
when control is disabled. Use disjoint CAN IDs and one SocketCAN owner when the
protocols share a process and physical bus.

Use Classical CAN with standard 11-bit IDs and DLC=8. CAN FD, extended IDs,
RTR, and error frames are not part of this protocol. The configured
`0x320/0x321/0x322` defaults were verified on the current Type C CAN1 ↔ Jetson
`can0` bus with motor traffic. They remain disjoint from legacy, motor, and
separate vision-control IDs. Recheck the allocation before enabling them on a
different robot or bus.

## 2. CAN IDs and roles

| Default ID | Direction | Purpose |
|---|---|---|
| `0x320` | Jetson → CBoard | START and final ACK |
| `0x321` | CBoard → Jetson | ACK + status + color |
| `0x322` | CBoard → Jetson | board-state fragments |

All three IDs must be distinct and in `0x000..0x7FF`.

## 3. Handshake frames

Handshake integers are big-endian. Every frame is exactly eight bytes.

| Message | byte0 | byte1 | byte2..5 | byte6 | byte7 |
|---|---|---|---|---|---|
| START | `0x01` | version `0x01` | session uint32 | `0x00` | `0x00` |
| ACK+color | `0x02` | version `0x01` | echoed session | status | color |
| final ACK | `0x03` | version `0x01` | echoed session | `0x00` | `0x00` |

Status values:

- `0`: READY — IMU data and calibration are ready; telemetry may start after
  the final ACK.
- `1`: NOT_READY — keep handshaking; do not stream telemetry.
- `2`: VERSION_MISMATCH — do not stream telemetry.

Color values are `0=unspecified`, `1=red`, and `2=blue`. READY requires red or
blue. Color is fixed for a session; changing it requires a new session. The
protocol color values are not interchangeable with `auto_aim::Color`
(`0=red`, `1=blue`).

For session `0x12345678` and blue:

```text
START:      01 01 12 34 56 78 00 00
READY ACK:  02 01 12 34 56 78 00 02
final ACK:  03 01 12 34 56 78 00 00
```

### 3.1 Jetson behavior

1. Generate a fresh nonzero uint32 session on startup and whenever a new
   session is required.
2. Send START immediately and retry every 100 ms until a matching READY
   response arrives. Ignore malformed frames, wrong sessions, and unrelated
   IDs. NOT_READY and VERSION_MISMATCH do not complete the handshake.
3. Send the final ACK for a matching READY response. Consider the session
   streaming only after the complete frame is queued to SocketCAN. If the final
   ACK is lost, a repeated READY response causes another final ACK; do not
   reset telemetry or color for that duplicate.
4. Accept telemetry only for the active session. Invalidate partial and latest
   state when changing sessions.
5. If no complete valid snapshot arrives for 500 ms, mark the stream stale and
   begin a new session. If the final-ACK phase itself cannot complete within
   500 ms (for example, CBoard rebooted while every final-ACK write failed),
   also begin a new session rather than retrying an orphaned ACK forever. The
   retry and timeout values are configurable.

SocketCAN write success means queued to the kernel, not received or executed
by CBoard. The application handshake is still required.

### 3.2 CBoard behavior

1. Wait for START without streaming. A new session stops the old stream and
   invalidates any unsent fragments from the old session.
2. Reply ACK+color with the current readiness. While waiting for the final
   ACK, repeat the response every 100 ms.
3. A matching final ACK starts streaming only when READY. A duplicate final
   ACK is idempotent. A same-session repeated START requests another response
   but does not reset an already streaming session.
4. Do not publish IMU data before `DataReady() && CaliDone()`. Suspend telemetry
   whenever readiness is lost; never send a fabricated quaternion.
5. Accept a new START at any time. There is no continuing Jetson heartbeat in
   v1; CBoard may therefore keep streaming until reset or a new session. The
   Jetson telemetry timeout is what initiates recovery.

Handshake retries, telemetry, and motor control must not be able to block one
another indefinitely. Firmware uses bounded/nonblocking CAN queueing and ISR
to task handoff for this link.

## 4. Board-state fragmentation

The 29-byte payload is sent in five frames on the data CAN ID. There is no UART
`ST`/`ED`, UART CRC8, or additional application CRC: Classical CAN already
provides per-frame integrity. Loss and stale-sample detection use the session
tag, sample sequence, fragment mask, and timeouts.

Each fragment has two metadata bytes and six payload bytes:

| Bytes | Meaning |
|---|---|
| 0 | bits7..5 fragment index `0..4`; bits4..0 sample sequence |
| 1 | session tag = `session_id & 0xFF` |
| 2..7 | six bytes of the little-endian `board_state` image |

The payload image is exactly the UART `BoardState`/`board_state_t` layout:

```text
float q[4] @0 (wxyz), float yaw @16, float pitch @20,
float bullet_speed @24, uint8_t mode[1] @28; total 29 bytes
```

Both targets are little-endian, so these payload bytes are copied without
byte swapping. This little-endian float rule applies only to the payload;
handshake integers remain big-endian.

For fragment `f`, copy six payload bytes starting at offset `6*f`. Fragment 4
is the exception: only five real bytes remain (`24..28`), and frame byte7 must
be zero padding. Every fragment still has DLC=8.

Sample sequence increments modulo 32. For sample sequence 7 and session tag
`0x78`, metadata byte0 is `07/27/47/67/87` for fragments 0 through 4, and
metadata byte1 is always `78`.

### 4.1 Receiver rules

- Require DLC=8, fragment index `0..4`, the active session tag, and zero padding
  in fragment 4.
- Fragments may arrive in any order. Keep at most one partial sample, identified
  by its sequence and session tag.
- A newer sequence abandons the previous partial sample. Relative to the current
  or last completed sample, only forward modulo-32 differences `1..15` are new;
  zero and differences `16..31` are duplicate/older and rejected. A conflicting
  duplicate fragment invalidates the partial sample; an identical duplicate is
  harmless.
- A partial sample older than 50 ms is discarded.
- Publish only after fragments 0 through 4 all arrive. Validate finite values
  and a nondegenerate quaternion, normalize the quaternion, and timestamp the
  completed sample at reception of its final missing fragment.
- Never interpolate or expose a partial/torn sample. Clear reassembly and IMU
  history on session changes.

At the nominal 100 Hz, five fragments make 500 CAN frames/s. The data source
must not start the next sample until all five fragments of the current sample
have been queued; bus congestion may reduce the effective rate but must not
cause overlapping sample sequences on the wire.

## 5. Integration checks

Keep the CBoard and board-state keys in one YAML so the interface and legacy-ID
collision checks see the same allocation. `BoardStateCanLink` also compares its
configured interface with the actual CBoard SocketCAN interface.

For a hardware-independent end-to-end check, create `vcan42`, then run the
independent Python peer and the live test:

```bash
sudo modprobe vcan
sudo ip link add dev vcan42 type vcan   # skip if it already exists
sudo ip link set up vcan42
python3 scripts/board_state_can_sim.py --interface vcan42 --seconds 8 &
./build/board_state_can_test -c=configs/board_state_can_vcan.yaml
```

The simulator also supports `--not-ready-ms` and `--stall-after-ms/--stall-ms`
for handshake-delay and reconnect tests. The live test fails when no complete
sample arrives or the final stream is stale. `configs/board_state_can.yaml` is
the physical CAN1/`can0` configuration; its IDs are confirmed on the current
test bus and must be rechecked before deployment on another robot.

The Embedded 2023 example has two targets from the same source:
`example_minipc_cboardcan.elf` (Jetson link on CAN1) and
`example_minipc_cboardcan_can2.elf` (Jetson link on CAN2). Gimbal motors remain
on CAN1 in both variants. Select the port that is physically wired; a no-ACK
bus-off is a physical-link failure, not a protocol timeout.
