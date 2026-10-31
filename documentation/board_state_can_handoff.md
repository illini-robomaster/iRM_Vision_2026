# Board-state CAN handoff — Jetson / Vision 2026

Date: October 24, 2026
Status: CAN payload aligned to Embedded 2026 (21 bytes, four fragments).
The historical physical Type C results below are not verification of the
current MC02 target/payload; repeat hardware acceptance before deployment.

The authoritative byte-level specification is
[board_state_can_protocol.md](board_state_can_protocol.md). This document is the
integration and bring-up guide. The Embedded-side counterpart is
`iRM_Embedded_2023/docs/board_state_can_handoff.md`.

## 1. Baseline and hardware provenance

Current source counterpart: `iRM_Embedded_2026/shared/libraries/board_state_can_protocol.h`
and `minipc_protocol.h`, integrated in `vehicles/Omni/eoh_mc02.cc` (`eoh_infant`,
DM_MC_02 FDCAN1). Current physical interoperability is pending.

The following commits and physical results are historical:

| Repository | Branch | Implementation commit |
|---|---|---|
| `iRM_Vision_2026` | `ywh114/hw_vision2026` | `f26090dfdecd68b9fbb9ea3045b6c7273761382a` |
| `iRM_Embedded_2023` | `ywh114/hw_vision2026` | `12825bddfd5e8672fd1de6d0c5cf4714697d703d` |

This is a **Jetson-initiated telemetry link**, not a replacement for the existing
vision↔CBoard angle-control session. It does not enable motors, authorize fire,
or assign semantics to the opaque mode byte.

Verified physical configuration:

- CBoard firmware target: `example_minipc_cboardcan.elf` using **CAN1**.
- Jetson interface: `can0`, Classical CAN, 1 Mbps, standard 11-bit IDs, DLC=8.
- IDs: `0x320` Jetson→CBoard, `0x321` CBoard→Jetson handshake response,
  `0x322` CBoard→Jetson telemetry fragments.
- Example color: blue. Example bullet speed and mode are documented placeholders
  on the firmware side.

## 2. What was added

| Path | Responsibility |
|---|---|
| `io/board_state_can.hpp/.cpp` | Wire codec, session FSM, reassembly, validation, timeout recovery, IMU history, SocketCAN adapter |
| `io/cboard.hpp/.cpp` | Independent frame-handler registration/removal and actual interface reporting |
| `io/vision_cboard.hpp/.cpp` | Own-handler lifecycle and nonblocking receive queue; existing control wire protocol unchanged |
| `configs/board_state_can.yaml` | Verified physical CAN1/`can0` configuration |
| `configs/board_state_can_vcan.yaml` | vcan-only integration configuration |
| `scripts/board_state_can_sim.py` | Independent stdlib CBoard simulator, including delayed-ready and stream-stall cases |
| `tests/board_state_can_test.cpp` | Offline golden/fault-injection tests plus live SocketCAN verification |
| `documentation/board_state_can_protocol.md` | Authoritative protocol v1 specification |
| `.github/workflows/build-x86.yml` | Portable builds, offline protocol tests, and vcan smoke test |

Do not move the new telemetry into `VisionLinkCore`. Angle-control stop behavior
and high-rate telemetry have different latency and safety requirements.

## 3. Wire and timing summary

Handshake:

1. Jetson sends START `01 01 session_BE 00 00` on `0x320`.
2. CBoard responds on `0x321` with
   `02 01 session_BE status color`. Status is READY=0, NOT_READY=1,
   VERSION_MISMATCH=2. READY requires red=1 or blue=2.
3. Jetson sends final ACK `03 01 session_BE 00 00` on `0x320`.
4. CBoard streams after the final ACK. A repeated READY causes the Jetson to
   repeat the final ACK, covering final-ACK loss.

Retries and recovery:

- START/response/final-ACK retry: 100 ms.
- Partial-fragment expiry: 50 ms.
- No complete sample for 500 ms: start a new session.
- Final-ACK phase unsuccessful for 500 ms: also start a new session; do not
  retry an orphaned final ACK forever.
- There is no continuing Jetson heartbeat in v1. CBoard accepts a new START at
  any time and otherwise continues streaming until reset/readiness loss.

The 21-byte little-endian `BoardStateCanPayload` is sent in four frames on
`0x322`:

```text
float q[4] @0 (wxyz), float bullet_speed @16, uint8_t mode[1] @20
```

Fragment byte0 is `(fragment_index << 5) | (sample_seq & 0x1F)`; byte1 is
`session_id & 0xFF`; bytes2..7 carry six payload bytes. Fragment 3 has three real
payload bytes and three zero pad bytes (frame bytes5..7). There is no UART `ST`/`ED` or application CRC
on CAN.

At 100 Hz this is four frames/sample, or 400 CAN frames/s. The receiver publishes
only complete, same-session samples with finite values and a nondegenerate
quaternion. Out-of-order fragments are supported; torn, stale, conflicting, and
partial samples are not published.

## 4. Jetson integration

### 4.1 Ownership and lifetime

Use one `io::CBoard` as the SocketCAN owner. `BoardStateCanLink` subscribes to
its two uplink IDs and sends through `CBoard::send_raw()`; it does not open a
second socket. `VisionCboardLink` uses a different subscription ID, so stopping
one link does not unsubscribe the other.

`CBoard` must outlive every link using it. Call `stop()` before destruction in
normal application shutdown; destructors also perform best-effort cleanup.

### 4.2 Minimal use

```cpp
#include "io/board_state_can.hpp"

auto config = io::load_board_state_can_config(config_path);
io::CBoard board(config_path);
io::BoardStateCanLink link(config, board);
link.start();

// Poll these from the consumer; none waits for the next CAN packet.
const auto snapshot = link.snapshot();
if (snapshot.state_valid) {
  const auto state = link.state();
  const Eigen::Quaterniond q_attitude = state.q;
  // state.yaw/state.pitch are NaN: current CAN v1 has no motor angles.
  const float bullet_speed = state.bullet_speed;
  const uint8_t opaque_mode = state.mode[0];
}

const auto q_at_frame = link.imu_at(camera_frame_timestamp);
link.stop();
```

Important API semantics:

- Always check `snapshot().state_valid` before treating `state()` as fresh.
- CAN `state().yaw/pitch` are unavailable (NaN), not measured zero. Relative
  aiming needs no motor angles. The UART 29-byte `BoardState` is unchanged;
  absolute-pose feedback must be negotiated separately, not inferred from q.
- `state().timestamp` is the Jetson reception time of the final missing
  fragment. The v1 payload has no MCU source timestamp.
- `imu_at(timestamp)` interpolates bounded completed-sample history and never
  exposes a partial sample.
- `valid_packet_count()` counts complete accepted samples, not raw CAN frames.
- `dropped_frame_count()` counts telemetry dropped by the userspace bounded
  queue; it is not the kernel CAN drop counter.
- `stats()` separates malformed, session, version, color, fragment, timeout,
  and send-failure counts.
- Protocol color is red=1/blue=2. It is **not** `auto_aim::Color`'s numeric
  layout; convert explicitly at the consumer boundary.
- `mode[0]` is opaque. Do not interpret it in the protocol layer.
- `link.send(command)` uses the pre-existing `CBoard::send()` command path
  (absolute in the supplied config) and is not part of this telemetry handshake.
  Do not use it as an enable or authorization mechanism.

`configs/board_state_can.yaml` deliberately sets `cboard_wait_imu: false`.
That disables the legacy CBoard blocking quaternion wait; use
`BoardStateCanLink::imu_at()`, not the legacy `CBoard::imu_at()`, in this mode.

### 4.3 Configuration keys

| Key | Current value / meaning |
|---|---|
| `board_state_can_control_canid` | `0x320` |
| `board_state_can_response_canid` | `0x321` |
| `board_state_can_data_canid` | `0x322` |
| `board_state_can_retry_ms` | `100.0` |
| `board_state_can_fragment_timeout_ms` | `50.0` |
| `board_state_can_stream_timeout_ms` | `500.0` |
| `board_state_can_ids_confirmed` | `true` for the verified current physical bus |
| `can_interface` | `can0`, or `vcan42` only in the vcan config |

The link rejects ID collisions with known legacy, motor, and vision-control
IDs. It also rejects a configured interface that differs from the actual
`CBoard` interface. Recheck the allocation before using another robot or bus;
do not copy `ids_confirmed: true` blindly.

## 5. Build and verification

### 5.1 Offline and native checks

```bash
make -C build board_state_can_test minipc_test relative_servo_test relative_aim -j$(nproc)
./build/board_state_can_test
./build/minipc_test
./build/relative_servo_test
```

Current Embedded 2026 alignment validation (Jetson aarch64):

- The build command above passes for all four targets.
- `board_state_can_test`: all checks pass, including independent fixed four-frame
  firmware wire vectors, publication on fragment 3, rejection of old fragment 4,
  all three tail padding bytes, and unavailable motor-angle markers.
- `minipc_test`: all checks pass; the 29-byte UART format is unchanged.
- `relative_servo_test`: 131 checks, 0 failures; relative-angle control unchanged.
- Python simulator syntax check passes (`python3 -m py_compile`).
- vcan integration was not rerun: no vcan interface exists and passwordless sudo
  is unavailable. No physical CAN transmission or firmware flashing was done.

Historical recorded results (not acceptance of the current MC02 payload):

- x86: CAN/UART offline suites and ASan/UBSan CAN suite pass.
- Jetson aarch64: CAN/UART suites pass; `relative_aim` builds;
  `relative_servo_test` reports **128 checks, 0 failures**.
- vcan: normal stream, delayed NOT_READY→READY, and stall→new-session recovery
  pass with a healthy final stream.

The separate camera/model-driven `vision_cboard_sim.py` acceptance script was
not completed in this worktree because the selected TensorRT engine file was
absent. This does not affect the board-state tests above; do not hide that
dependency when running the older end-to-end acceptance script.

### 5.2 vcan integration

```bash
sudo modprobe vcan
sudo ip link add dev vcan42 type vcan   # skip if it already exists
sudo ip link set up vcan42

python3 scripts/board_state_can_sim.py --interface vcan42 --seconds 8 &
sim_pid=$!
sleep 0.2
./build/board_state_can_test -c=configs/board_state_can_vcan.yaml
wait "$sim_pid"
```

Recovery variant:

```bash
python3 scripts/board_state_can_sim.py \
  --interface vcan42 --seconds 8 --stall-after-ms 1500 --stall-ms 1000 &
sim_pid=$!
sleep 0.2
./build/board_state_can_test -c=configs/board_state_can_vcan.yaml
wait "$sim_pid"
```

The live test fails if no complete sample arrives **or the final stream is
stale**. A few packets before a disconnect therefore cannot produce a false
pass.

### 5.3 Physical CAN1 verification

Flash the Embedded CAN1 target first, ensure the CBoard/splitter are powered,
then run:

```bash
./build/board_state_can_test -c=configs/board_state_can.yaml
```

Recorded physical result after replacing a bad CAN cable:

- Three independent runs, each **500 complete samples in 5 seconds = 100 Hz**.
- Zero userspace dropped frames, zero rejected frames, and zero CAN error
  counters.
- Live quaternion changed with the board; measured motor yaw/pitch, bullet
  speed 15, and mode 1 matched the firmware example.
- Reopening the Jetson test created a fresh session and streaming resumed.
- Motor traffic remained active during verification; this was not a closed-loop
  motion or firing test.

## 6. Troubleshooting

### Jetson enters BUS-OFF / no ACK

This is below the protocol layer. During bring-up, a bad cable produced
`BUS-OFF`, `tx=248`, and no responses on both firmware port variants. Replacing
the cable restored an error-active bus immediately. Check cable, CANH/CANL,
common ground, power, termination, and whether the splitter actually reaches
CBoard CAN1.

After fixing the physical fault, reset the interface before retrying:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

Do not increase protocol timeouts to compensate for a no-ACK physical bus.

### Handshake stays NOT_READY

The firmware is waiting for `DataReady() && CaliDone()`. Keep the board powered
and stationary for its normal IMU initialization/calibration. Do not publish a
fabricated quaternion or bypass readiness.

### READY/streaming starts but no valid state

Check IDs, DLC=8, session IDs, fragment sequence, `rejected_*` counters,
`partial_timeouts`, and CAN statistics. A failed final-ACK phase now starts a
new session after 500 ms; older code could wait indefinitely and should not be
restored.

### CAN2 firmware responds on the wrong bus

The current verified wiring is CAN1. `example_minipc_cboardcan_can2.elf` is an
alternative for different wiring, not a second simultaneously active link.
Keep gimbal motors on CAN1 in either firmware variant.

## 7. Remaining integration work

- Replace the firmware example's fixed blue color, bullet speed 15, and mode 1
  with real robot/referee/application sources.
- Recheck IDs and interface names on every robot before enabling the link.
- Instantiate `BoardStateCanLink` in the application that needs the telemetry;
  `relative_aim` is not automatically a board-state consumer.
- If tighter attitude timing is required, add an MCU sample timestamp in a
  versioned protocol update rather than treating reception time as sensor time.
- Perform any future closed-loop motion/fire test separately with the robot's
  normal mechanical and referee safety procedures. This telemetry acceptance
  test is not authorization for either.
