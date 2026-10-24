# Relative-angle visual-servo handoff — historical development snapshot

Original snapshot: October 17, 2026; current-frame update: October 24, 2026.
Environment: Jetson/aarch64, `/home/irm/iRM-Vision-2026`. The existing build is
configured; do not delete it. The old `/home/irm/sp_vision_25` symlink is needed
by the cached build paths.

**For current integration, start with `vision_cboard_handoff.md`,
`vision_cboard_protocol.md`, and `vision_cboard_acceptance.md` in this directory.**
This English edition consolidates the original work log and superseded HUD
proposal. It preserves historical results and diagnostic caveats; old commands
and check counts are not the current acceptance baseline.

## 0. Completed follow-up

The proposed raw/TX HUD and per-frame CSV are implemented in the dedicated
`/home/irm/iRM-Vision-2026/src/relative_aim.cpp`, reusing `tools::MjpegServer`.
They were **not** added to `detect_freq_visual_test`, whose detection-frequency
responsibility and `-dump` CSV schema remain unchanged.

```bash
cd /home/irm/iRM-Vision-2026
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current.yaml -stream=8080
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current.yaml -csv=/tmp/r.csv
```

These live commands require approved IDs and a ready/enabled v1 session.
Use the replay config plus `-dry-run` for camera/CAN-free inspection.

### 0.1 October 24 current-frame implementation

| Item | Recorded status |
|---|---|
| `CurrentRelativeEstimator`: filtering, deterministic selection, PnP, reference frame, freshness, quantization | Implemented and unit-tested |
| `relative_aim`: current/tracker source, CSV, MJPEG, display, best-effort shutdown stop | Implemented |
| `camera_optical` default; `gimbal` requires extrinsics and rejects missing keys | Implemented |
| `CBoard::send()` bool result; disabled/NaN/Inf encoding clears angles | Implemented |
| Full SocketCAN write checking, errno, fd reset after close, EINTR/EAGAIN handling | Implemented |
| Hardware/replay current-frame configs | Added |
| Initial expanded self-test | 54 checks, 0 failures |
| Replay | 150 images, 124 valid (82.7%), detection 23.3 ms/frame |
| Independent vcan capture | 121 frames, all `0FE [8]`; 99 valid frames matched CSV, zero mismatches |
| New USB3.0 cable | 1280×1024, 120.5 fps; 3×60 s, p95 ≤8.5 ms; drops 0/0/1 |
| Intrinsics ownership | Camera SN 043070620316 confirmed across eight calibration sessions; exact config match |
| Old-cable controlled A/B, real CBoard, gimbal closed loop, firing | Not verified |

The old cable is USB 2.0 Type-B without screw locks; it was unavailable for A/B.
See `relative_acceptance.md` for the historical measurements. The later v1 test
baseline is 128 checks, not 54; see `vision_cboard_acceptance.md`.

## 1. Original implementation state

The original command channel added:

- `can_command_mode: absolute/relative`, defaulting to absolute for old callers.
- `encode_relative_command()`: configurable standard CAN ID, DLC 8, signed int16
  big-endian yaw/pitch at 1e4 units/rad, saturation rather than wraparound.
- `cboard_wait_imu`, default true. False bypasses waiting for quaternion frames
  and makes `imu_at()` return identity.
- `VisualServo::angles()`, `shape()`, and `aim()`; a tracker-based entry point
  with `-dry-run`/`-n`; optional keys in `configs/standard4.yaml`.
- Initial unit coverage: 22 checks, 0 failures; the program built and ran.

That snapshot initially could not produce nonzero demo output because the
tracker remained lost. The later default `-source=current` bypasses tracker
locking and solves PnP from each detection's own corners. Valid first-frame
observations are possible; an actual first image may still contain no valid target.

The firmware handler, real mechanical loop, and firing were not implemented or
verified in that snapshot. The original commit was `44d15df`, reported pushed
with local/remote SHA matching. That is a historical commit, not today's HEAD.

## 2. Original change inventory

Paths below are relative to `/home/irm/iRM-Vision-2026`:

- `CMakeLists.txt`: `relative_aim` and `relative_servo_test` targets.
- `tasks/auto_aim/CMakeLists.txt`: `visual_servo.cpp` in the auto_aim library.
- `io/cboard.hpp`: command mode, encoder declaration, relative-mode query.
- `io/cboard.cpp`: config loading, send dispatch, relative encoding, IMU wait.
- `configs/standard4.yaml`: optional configuration keys and comments.
- `src/relative_aim.cpp`, `tasks/auto_aim/visual_servo.{hpp,cpp}`,
  `tests/relative_servo_test.cpp`, and the relative CAN protocol document.

The user-local `/home/irm/iRM-Vision-2026/.vscode/c_cpp_properties.json` change
was deliberately left uncommitted. Preserve it; do not stage or discard it.

## 3. Original checks and geometry samples

The initial build and 22-check test returned zero:

```bash
make -C /home/irm/iRM-Vision-2026/build relative_aim relative_servo_test -j4
/home/irm/iRM-Vision-2026/build/relative_servo_test
```

The old demo run with `configs/offline.yaml` and the tracker path produced only
`no command (tracker=lost, armors=0/1)`. This was a tracker-path issue, not proof
that per-frame geometry was wrong. Temporary diagnostics, subsequently removed,
showed these gimbal-frame F-L-U positions in meters:

| Image | `xyz_in_gimbal` | Analytical angles at gain=1, deadband=0.1° |
|---|---|---|
| 1 | (0.975, 0.035, 0.100) | yaw ≈ +2.06°, pitch ≈ -5.86° |
| 3 | (1.010, 0.013, 0.010) | yaw ≈ +0.74°, pitch ≈ -0.57° |
| 4 | (1.002, 0.024, 0.027) | yaw ≈ +1.37°, pitch ≈ -1.54° |
| 5 | (1.022, 0.024, 0.004) | yaw ≈ +1.35°, pitch ≈ -0.22° |

These are historical gimbal-frame examples, not camera-optical reference values.

## 4. Superseded HUD proposal and retained design lessons

The original proposal would have added `-servo` to `detect_freq_visual_test`,
constructed Solver/Tracker/VisualServo after YOLO, and drawn two lines:

```text
servo raw  dyaw +1.35  dpitch -0.22 deg  armor sentry/small  norm(0.61,0.59)
servo cmd  dyaw +1.35  dpitch -0.22 deg  control 1  tracker tracking  targets 1
```

Raw would be computed independently of tracker lock; command would show actual
shaped output, yellow when disabled and green when valid. Proposed positions
were y=150/180 below existing metrics at 30/60/90/120, mutually exclusive with
the NO YOLO line. Periodic logs would also show raw/TX, control, tracker state,
and target count. This option was never the final interface: **do not use
`detect_freq_visual_test -servo` as a supported command**.

Retained implementation lessons:

1. Copy `armors` before passing them to `Tracker::track()`: it removes non-enemy
   colors in place. Sharing the drawing list can make boxes disappear.
2. For diagnostic demo tracking, explicitly consider `use_enemy_color=false`;
   the default true can filter every blue armor if configured color disagrees.
   Production v1 current-frame output instead obeys remote color control.
3. Set `R_gimbal2world` to identity before the first solve in no-IMU tracker
   diagnostics. This does not calibrate camera-to-gimbal extrinsics.
4. Preserve detection `-dump` headers: `csv_ab.py` uses `csv.DictReader` and
   `replay_ab.sh` depends on those names. Relative-angle CSV belongs to its own entry point.

Current inspection command:

```bash
cd /home/irm/iRM-Vision-2026
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current_replay.yaml -dry-run -n=150 -csv=/tmp/d.csv -stream=8080
curl -s -o /tmp/snap.jpg http://127.0.0.1:8080/snapshot.jpg
```

Expected: selected-target box and raw/TX angles varying with image position.
Camera-optical errors approach zero at the optical axis; signs are left-positive
yaw and up-negative pitch. Do not infer physical closed-loop stability from this.

## 5. Historical tracker diagnostic: lost state and NaN divergence

The current-frame path bypassed this blocker on October 24. Tracker behavior
still needs separate investigation if that legacy path is used.

- With offline defaults `min_confidence=0.8`, `min_detect_count=5`, detections
  alternated between zero and one; individual PnP solutions were plausible.
- A temporary derived config using 0.6 and 3 could enter tracking, then lose it
  on the next image. Debug logs showed `[Target] r=nan, l=nan` from `diverged()`.
- **Unconfirmed hypothesis, not a demonstrated root cause:** `Target::update()`
  declared `int id;` and assigned it only when the angle-error comparison passed.
  If candidate values were NaN, comparisons could all fail, leaving an invalid
  index and propagating NaN. The first NaN source was not located; confidence/
  keypoint jitter or monocular IPPE degeneracy were possible explanations only.

If investigating, first instrument EKF state, angle error, and selected ID, then
determine whether NaN appears immediately or after updates. Remove diagnostics
afterward. Do not change tracker algorithms based on this hypothesis: confirm
the cause and provide the required impact analysis before any architectural edit.
Record stable real-camera footage for numeric comparisons when hardware is available.

## 6. Hardware integration checklist

The original checklist suggested per-frame accumulation and vision-side gain/
step protection. **That is not the current v1 control contract.** Output is a
measurement, unshaped by default, and firmware owns the limited controller.

Use the current handoff: agree all four IDs and bitrate, prohibit motion, capture
traffic independently, verify handshake/enable/color/timeout/reset/disconnect/
SIGINT, then confirm signs and perform a low-speed limited loop. Do not enable
the mechanical loop before firmware is integrated. Relative output always has
shoot=0; firing remains unverified.

`relative_aim` refuses a non-relative command mode; no-quaternion operation
requires `cboard_wait_imu: false`. Short options require `-c=<path>`, not `-c <path>`.
Use Ctrl+C or `timeout -s INT`, not default SIGTERM, for live-camera shutdown.

## 7. Remaining firmware work

The original host-only snapshot did not provide a deployed board handler.
Firmware must implement current v1 framing/state rules, local gating,
measurement invalidation, watchdogs, motion protection, and collision-free IDs.
The legacy `0xFE` example is not an approved hardware allocation.

## 8. Delivery discipline

Follow `/home/irm/iRM-Vision-2026/AGENTS.md`: targeted build and real test results,
no temporary debug output, no incompatible Eigen/fmt APIs, explicit-path staging,
focused commits, push, and local/remote SHA comparison. Do not stage build/log/
third-party/engine artifacts or the user's VS Code settings. Historical planned
HUD/tracker follow-up commits are not outstanding work commitments.

## 9. Quick reference

| Topic | Location under `/home/irm/iRM-Vision-2026` |
|---|---|
| Current firmware handoff / state protocol | `documentation/vision_cboard_handoff.md`, `documentation/vision_cboard_protocol.md` |
| Current acceptance | `documentation/vision_cboard_acceptance.md` |
| Geometry / relative payload | `documentation/relative_can_protocol.md` |
| Initial cable/replay/vcan evidence | `documentation/relative_acceptance.md` |
| Current observations / deterministic selection | `tasks/auto_aim/current_relative.{hpp,cpp}` |
| Legacy angle/shaping/aim helpers | `tasks/auto_aim/visual_servo.{hpp,cpp}` |
| Relative encoding / shared quantizer | `io/cboard.cpp`, `io/command.hpp` |
| End-to-end entry / HUD / CSV | `src/relative_aim.cpp` |
| Unit tests | `tests/relative_servo_test.cpp` |