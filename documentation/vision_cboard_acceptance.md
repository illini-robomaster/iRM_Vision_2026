# Vision ↔ CBoard acceptance record

Date: October 24, 2026. Workspace: `/home/irm/iRM-Vision-2026`, Jetson/aarch64.
Runtime logs selected TensorRT and the existing local FP16 engine. No CBoard
was connected and no frames were sent on physical `can0` during this work.
A–E below are delivery-specific verification levels, not external certification
or a complete safety proof.

| Level | Scope | Result |
|---|---|---|
| A | Minimal builds; protocol/geometry/state/configuration/concurrency transaction tests | PASS: 128 checks, 0 failures |
| B | Actual TensorRT replay, current-frame angle CSV | PASS: 60 frames, 55 valid, no CAN transmission |
| C | Independent vcan firmware peer + actual vision process; handshake/control/stop/wire comparison | PASS; evidence below |
| D | Physical CAN and actual firmware; disconnects, resets, signs, timeouts | **Pending** |
| E | Real camera and mechanical closed loop; motion limits and firing interlocks | **Pending** |

## Commands actually executed and evidence

Executed from the workspace above. Commands returned zero, except expected
timeouts inside the regression script. These are records of the implementation
verification, not claims that all tests were repeated during English translation.

```bash
make -C /home/irm/iRM-Vision-2026/build relative_servo_test relative_aim camera_test -j4
/home/irm/iRM-Vision-2026/build/relative_servo_test
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current_replay.yaml -dry-run -n=60 -csv=/tmp/current_resume.csv
unshare -Urn sh -c 'ip link add vcan_test type vcan && ip link set vcan_test up && python3 /home/irm/iRM-Vision-2026/scripts/vision_cboard_sim.py --output=/tmp/vision_final_vcan'
LOG_DIR=/tmp/vision_resume_regression bash /home/irm/iRM-Vision-2026/scripts/run_local_tests.sh
```

- Build/test evidence: `/tmp/vision_final_build.log`, `/tmp/vision_final_unit.log`:
  all three targets built; 128 checks, 0 failures.
- Replay: `/tmp/current_resume.log`, `/tmp/current_resume.csv`: 60 frames,
  55 valid; mean detection 26.8 ms, observation-age p50=27.4 ms, p95=31.3 ms.
  No CBoard constructed; publication count zero. This replay ran concurrently
  with the first vcan test; latency is not an exclusive-use performance baseline.
  Additional replay runs with `-dry-enable=0 -n=12` and `-dry-color=red -n=12`
  each returned zero with 12 frames, zero valid, no CAN sends. Logs:
  `/tmp/vision_disabled.log`, `/tmp/vision_red.log`.
- vcan: `/tmp/vision_final_vcan_result.log`, `/tmp/vision_final_vcan.{log,csv,json}`:
  VCAN PASS, 42 valid frames in the blue-enable phase; overall 162 image frames,
  83 valid-angle writes, 71 stops, 3 ACKs, zero send failures. Accepted publications
  were 84, not necessarily equal to 83 sends: asynchronous replacement/shutdown
  can discard an unsent sample. Every actual valid wire frame matched the ordered
  successful-publication CSV sequence. Stops were all zero; shoot=0. One control
  timeout, four old-session rejections, one malformed CONTROL rejection; independent
  peer received the normal SIGINT shutdown stop.
  After final raw-ID/receive-frame-flag checks, the full build and vcan test were
  repeated successfully: `/tmp/vision_delivery_vcan_result.log` and
  `/tmp/vision_delivery_vcan.{log,csv,json}`. This run had 158 images, 83 accepted
  publications, 82 valid-angle sends, 69 stops, 2 ACKs, zero send failures, and
  42 valid frames in the blue-enable phase. All six test groups passed, including
  wire/CSV comparison and SIGINT stop reception.
- Regression: `/tmp/vision_resume_regression.log`: 5 PASS, 0 FAIL, 2 SKIP.
  Planner/camera/DM were bounded runs, internal rc=124 with expected log markers;
  detection bench and traditional refinement returned zero. `auto_aim_test` lacked
  DISPLAY/Wayland/xvfb; ORT-vs-cv::dnn A/B had fewer than two usable backends and
  was **not verified**. Existing regression executables were not all rebuilt;
  this is not a full-repository rebuild result.

## Covered boundaries

- NOT_READY/READY/version mismatch; idempotent same-session HELLO; stop before
  ACK when switching an active session.
- Disable/resume and red/blue override; rejection of old generation and cached
  images captured before a control change.
- Sequence wrap/out-of-order rejection; 500 ms timeout also in wait_control.
- Malformed DLC/type/version/reserved bits and illegal stop color fail closed;
  structurally valid old-session control is rejected without changing state.
- Independent observation-expiry stop; invalid observations bypass rate limits;
  failed-stop retry; shutdown stop takes priority over ACK.
- Injected blocking sender verifies that control changes wait through the
  eligibility/write transaction, then stop precedes further valid observations.
- Separate coordinate/protocol keys; legal 0x000 preserved; loopback/unknown
  interfaces cannot impersonate vcan; NaN timeout rejected.
- Geometry fallback uses distinct target numbers and asserts selection of the
  second candidate, rather than comparing an identical target ID.

## Limits and pending acceptance

1. vcan does not verify physical bitrate, ACK, arbitration congestion, or bus-off.
   SocketCAN write success is not proof of firmware execution.
2. No actual CBoard, motor, or hand–eye calibration. Keep `camera_optical` and
   `vision_can_ids_confirmed: false` until hardware agreement/testing.
3. Send failures were injected in unit tests; physical send failure, hardware
   resets, and residual hardware TX queues were not tested.
4. Capacity-64 reception and overflow fail-closed are implemented, but no dedicated
   flood/starvation/latency-bound acceptance was run. No TSAN/ASAN was run; full
   repository thread safety is not proven. Historical callers remain responsible
   for synchronization of public CBoard telemetry fields.
5. Industrial-camera queue waits are bounded at 50 ms; SDK destruction/file
   decoding have no hard deadline. Real camera unplug/SIGINT awaits D/E.
6. Relative-angle payloads contain no session; queued kernel/hardware frames
   cannot be withdrawn. Firmware needs local gating, cache invalidation, and a
   watchdog. A no-old-session-on-wire guarantee requires a protocol revision.
7. Measurements are unshaped by default. Firmware must not blindly adopt the
   old per-frame accumulation example or rely on vision deadband/step limits
   for mechanical protection.

Before hardware connection, complete `vision_cboard_handoff.md` in this directory.
Do not claim hardware closed-loop completion before D/E pass.