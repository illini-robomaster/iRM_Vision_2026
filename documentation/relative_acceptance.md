# Relative-angle pipeline acceptance record — October 24, 2026

Historical measurements of new USB3.0 cable → camera → detection → current-frame
angles → CAN bytes on Jetson Orin Nano/aarch64 in `/home/irm/iRM-Vision-2026`.
Numbers below are the recorded runs, not inferred performance. Unperformed
checks are explicitly marked unverified. Translation does not imply rerunning
these hardware/replay measurements.

For the **later v1 session/control acceptance**, use
`vision_cboard_acceptance.md`. This record's 54-check and legacy-vcan results
precede that baseline. Geometry is described in `relative_can_protocol.md`;
development history in `handoff_relative_servo.md`.

## 0. Summary

| Item | Recorded result |
|---|---|
| New USB3.0 cable | Enumerated at 5 Gbit/s; 1280×1024, 120.5 fps, 3×60 s without >50 ms intervals; drops 0/0/1 |
| Controlled old-cable A/B | **Unverified**: old USB 2.0 Type-B cable without screw locks was unavailable |
| Detection→PnP→angles→quantization | 150 replay images, 124 valid (82.7%); TX differs from raw by ≤0.5×1e-4 rad |
| Independent vcan byte capture | 121 frames, all `0FE [8]`; 99 valid frames matched CSV, zero mismatches |
| Headless HUD | snapshot HTTP 200, image/jpeg; selected target green; ASCII angles readable |
| Actual CBoard / mechanical loop / firing | **Unverified**: no board, motors, or encoders |

## 1. Environment

| Item | Value |
|---|---|
| Platform | Jetson Orin Nano, aarch64, Ubuntu 20.04 / JetPack 5.1.3 |
| Camera | MindVision MV-SUA133GC, f622:0001, SN 043070620316 |
| Connection | New USB3.0 Type-B cable → Genesys hub → bus 2 addr 4, speed=5000 |
| Inference | TensorRT 8.5.2, local assets/yolov5_0526_fp16.engine; no ORT; OpenCV 4.5.4 DNN cannot execute this graph |
| CAN | No CBoard; physical can0 UP at 1 Mbit/s, **no transmission on can0** in this work |

## 2. Cable and camera acquisition

The previous `camera_test` logged every image at info level while the logger
flushed on info, perturbing high-rate measurement. The revised default logs
once per second and summarizes interval p50/p95/p100, standard-deviation jitter,
and >50 ms intervals. `-v` retains per-image output; `-w` warmup defaults to 5 s;
`-t` exits normally so the SDK reports total/valid/dropped images. Do not use
default SIGTERM to terminate an industrial-camera test.

Recorded commands, three runs with 5 s warmup and 60 s statistics:

```bash
cd /home/irm/iRM-Vision-2026
for i in 1 2 3; do
  /home/irm/iRM-Vision-2026/build/camera_test -c=/home/irm/iRM-Vision-2026/configs/mv_sua133gc.yaml -w=5 -t=65 > /tmp/p1_run$i.log 2>&1
done
```

| Run | Images | Duration | Mean fps | p50 ms → fps | p95 ms | p100 ms | Jitter std ms | >50 ms | SDK drops |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 7229 | 60.01 s | 120.47 | 8.29 → 120.67 | 8.46 | 23.99 | 0.40 | 0 | 0 (7854/7854) |
| 2 | 7240 | 60.01 s | 120.65 | 8.29 → 120.70 | 8.49 | 15.60 | 0.33 | 0 | 0 (7854/7854) |
| 3 | 7219 | 60.01 s | 120.30 | 8.29 → 120.69 | 8.50 | 25.41 | 0.57 | 0 | 1 (7856/7855, initial image) |

Mean rates were 120.30–120.65 fps with p95 ≤8.5 ms. Raw 1280×1024 Bayer8 is
approximately 1.25 MB/image ×120 fps ≈150 MB/s, within USB3.0 Gen1 bandwidth.
The earlier same-camera USB2.0/480 Mbit/s baseline was approximately 35.7 fps,
about 3.4× lower. **This is cross-session comparison, not controlled A/B proof
of the benefit from replacing the cable.** Both old and new camera-side plugs
were Type-B without locks. A proper A/B still requires the old cable, same
machine/port/preset, and the same warmup/duration.

## 3. Intrinsics and calibration

### 3.1 Ownership confirmed: SN 043070620316

| Evidence under `/home/irm/iRM-Vision-2026` | Recorded finding |
|---|---|
| logs/2026-10-10_16-23-58.log:6 | MV-SUA133GC, USB3.0 interface, CMOS 1.3M, SN 043070620316, instance 0 |
| Same log :14 and :21 | USB2.0 high-speed 480 Mbit/s; effective resolution 1280×1024 |
| Eight calibration sessions | records/calibration/2026-10-10_15-44-49_0 through _16-23-58_0 all used that SN and resolution |
| Serial-number log scan | 165 occurrences of 043070620316; another 3 of 041080420478, a briefly connected camera on October 17 not used for calibration |
| Field comparison | camera_matrix/distort_coeffs in the final intrinsics.yaml exactly match configs/relative_current.yaml |

Reproduce the inspection:

```bash
cd /home/irm/iRM-Vision-2026
grep -n 'SN ' /home/irm/iRM-Vision-2026/logs/2026-10-10_16-23-58.log | head -1
grep -rhoE 'SN [0-9]{12}' /home/irm/iRM-Vision-2026/logs/*.log | sort | uniq -c | sort -rn
diff <(grep -o 'camera_matrix: .*' /home/irm/iRM-Vision-2026/records/calibration/2026-10-10_16-23-58_0/intrinsics.yaml) \
     <(grep -o 'camera_matrix: .*' /home/irm/iRM-Vision-2026/configs/relative_current.yaml) && echo SAME
```

Correction: the earlier verbal reference to SN `042160220145` was incorrect.
The recorded repository-wide scan, including logs/records/docs/configs, had
zero matches; it was not evidence of a camera used for project calibration.
The intrinsics discussed here belong to `043070620316`.

### 3.2 Calibration details

- Source: records/calibration/2026-10-10_16-23-58_0; RMS 0.224 px,
  mean_error_px=0.189, 1280×1024, preset 0 `1280X1024 Max`.
- Calibration used the old USB2.0 unlocked Type-B cable (480 Mbit/s, about
  35.7 fps). No recalibration was done for this cable change: lens, sensor,
  ROI, and pixel geometry were unchanged; only transport bandwidth/rate changed.
- Actual-camera hand–eye extrinsics remain **uncalibrated**. Optical output
  does not require them; selecting gimbal without the required keys fails at startup.

## 4. Offline tests and dry-run

### 4.1 Initial expanded unit test

```bash
make -C /home/irm/iRM-Vision-2026/build relative_servo_test -j4
/home/irm/iRM-Vision-2026/build/relative_servo_test
# Historical result: 54 checks, 0 failures, exit 0.
```

Covered signs (left positive/up negative), shaping/signs/offsets, byte layout,
disabled/NaN/Inf zeroing, saturation, optical/gimbal geometry including the
identity-extrinsics counterexample, distortion, minimum depth, reprojection
rejection, deterministic target choice, and stale images.

### 4.2 Replay without CAN

```bash
cd /home/irm/iRM-Vision-2026
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current_replay.yaml -dry-run -n=150 -csv=/tmp/rel_dry.csv
```

Recorded with assets/demo/demo.avi and TensorRT FP16:

| Metric | Value |
|---|---|
| Images | 150 |
| Valid | 124 (82.7%) |
| Invalid reasons | no_target 24, out_of_image 2 |
| Detection | 23.3 ms/image |
| Observation age | p50 23.3 ms, p95 24.1 ms |
| max abs(TX_int − raw×1e4) | 0.50, half an LSB: quantization only |

Historical CSV examples, angular columns in degrees:

```csv
frame,frame_t_ms,age_ms,candidates,target_id,valid,reason,raw_yaw_deg,raw_pitch_deg,tx_yaw_deg,tx_pitch_deg,tx_yaw_1e4,tx_pitch_1e4,reproj_px
2,90.25,22.66,1,blue:sentry:small@863/451,1,ok,-3.4802,-2.5057,-3.4781,-2.5040,-607,-437,1.590
4,158.16,22.73,1,blue:sentry:small@896/635,1,ok,-4.5750,3.2912,-4.5725,3.2890,-798,574,1.166
```

Unlike the old always-lost tracker run, current uses each detection's own
corners for PnP and does not wait for target lock.

### 4.3 Headless HUD

```bash
cd /home/irm/iRM-Vision-2026
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current_replay.yaml -dry-run -n=400 -stream=8099 &
curl -s -o /tmp/hud.jpg -w '%{http_code} %{content_type} %{size_download}\n' http://127.0.0.1:8099/snapshot.jpg
# Recorded: 200 image/jpeg 132175
```

Snapshot: `src=current ref=camera_optical frame=115`, raw and TX yaw=-6.22,
pitch=-0.10 deg, valid=1, reason=ok, cand=1, target=blue:sentry:small@952/528,
age=24 ms, detect=24 ms, sent=0/ok=0/fail=0; selected armor outlined green.
Text was ASCII for Hershey fonts. Zero sends are expected in dry-run.

## 5. Historical CAN software acceptance

An isolated user/network namespace provided vcan0 without disturbing physical
can0 (1 Mbit/s, ERROR-ACTIVE). There was no board to ACK real traffic.
The following is the recorded **pre-v1** command, not the current session test:

```bash
cd /home/irm/iRM-Vision-2026
awk '{if ($1=="can_interface:") print "can_interface: \"vcan0\""; else print}' \
  /home/irm/iRM-Vision-2026/configs/relative_current_replay.yaml > /tmp/rel_vcan.yaml
unshare -Urn --map-root-user bash -c '
  ip link add dev vcan0 type vcan && ip link set up vcan0
  candump -tz vcan0 > /tmp/candump.txt &
  sleep 1
  /home/irm/iRM-Vision-2026/build/relative_aim -c=/tmp/rel_vcan.yaml -n=120 -csv=/tmp/rel_vcan.csv
'
```

| Metric | Recorded value |
|---|---|
| Program | 120 images; 99 valid (82.5%); 120 successful sends, zero failures/rate skips |
| Shutdown stop | Successful, 0.02 ms |
| Independent capture | 121 frames, all `0FE [8]` |
| control=1 / control=0 | 99 / 22 (21 invalid images + shutdown) |
| shoot set | Zero frames |
| Valid wire values versus CSV tx_*_1e4 | Zero mismatches across 99 frames |
| First frame | Eight zero bytes, no_target |

```text
(000.033910) vcan0 0FE [8] 01 00 FD A1 FE 4B 00 00
                         yaw=-607 (-3.478°), pitch=-437 (-2.504°)
(004.126923) vcan0 0FE [8] 00 00 00 00 00 00 00 00  # shutdown stop
```

Current v1 replay requires an independent handshake/control peer. Use
`/home/irm/iRM-Vision-2026/scripts/vision_cboard_sim.py` and the command recorded
in `vision_cboard_acceptance.md`, not the historical no-handshake example.
Physical can0 was deliberately not used: without another ACK-capable node,
transmission would retry and increase error counters.

## 6. Observability added during this work

| Capability | Implementation under `/home/irm/iRM-Vision-2026` |
|---|---|
| Image/time/age/target/raw/TX/validity/reason/timing CSV | src/relative_aim.cpp, -csv |
| ASCII HUD and MJPEG | src/relative_aim.cpp, -stream / -d |
| Checked send result | CBoard::send bool, complete SocketCAN write and errno |
| Safe quantization | encode_relative_command / quantize_angle_1e4 |
| Preconstruction config validation | relative_aim command mode / IMU-wait / ID checks |
| Transport robustness | io/socketcan.hpp fd reset, EINTR/EAGAIN, incomplete-frame rejection |

## 7. Hardware acceptance — not verified

- [ ] Deploy the current v1 firmware handler and agree all IDs; 0xFE is not approved.
- [ ] Independently capture DLC=8, reserved zeros, signed int16 BE and shoot=0.
- [ ] Implement local session/enable gating, cache invalidation, and receive watchdog.
- [ ] Check left/right and up/down directions; invert signs only once.
- [ ] Verify stopping on no target, disabled control, heartbeat loss, and disconnect.
- [ ] Use firmware-limited low-speed control; check convergence and absence of oscillation.
- [ ] Keep firing disabled: no firing decision was implemented here.

The original record suggested per-frame accumulation and servo_gain=0.2 /
servo_max_step_deg=3. Those are historical recommendations, **not the current
v1 measurement contract**. Default output is unshaped and firmware owns safety.

## 8. Unverified items and historical gaps

1. Controlled cable A/B remains pending; use the same machine/port/preset and
   `-w=5 -t=65`. Avoid default SIGTERM termination.
2. Real can0/board parsing and mechanics were not tested by vcan.
3. Closed-loop stability needs motors/encoders; changing image errors is not proof.
4. Actual hand–eye extrinsics are uncalibrated; gimbal geometry has synthetic tests only.
5. Host image timestamps omit exposure/transport delay.
6. The initial snapshot noted a SocketCAN write-versus-close/reconnect race.
   That was a historical transport observation, not a current whole-repository
   concurrency guarantee. See current v1 acceptance for serialized transactions,
   untested flood/starvation cases, and the absence of sanitizer verification.