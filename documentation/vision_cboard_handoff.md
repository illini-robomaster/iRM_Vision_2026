# Firmware integration handoff: vision ↔ CBoard

Protocol specification: `vision_cboard_protocol.md` in this directory.
Verification scope and evidence: `vision_cboard_acceptance.md`.

## Joint checklist before connecting hardware

Do not set `vision_can_ids_confirmed` until all applicable items are complete.

- [ ] Agree on all four standard CAN IDs (relative angles, HELLO, ACK, CONTROL),
      directions, bitrate, termination resistors, and interface name.
- [ ] Agree on the current `camera_optical` origin/reference axis: yaw positive
      left, pitch negative up; radians encoded as signed int16 BE in 1e-4 rad units.
- [ ] No hand–eye calibration has been performed. Do not interpret the output as
      gimbal-frame or world-frame absolute angles. Candidate IDs are not approved assignments.
- [ ] `shoot` is always zero. Disable firing and motor output, or mechanically
      disconnect the actuator, before inspecting traffic.
- [ ] Implement firmware-local enable gating, a valid-observation watchdog,
      position/velocity/acceleration limits, and cache invalidation on stop.
- [ ] Relative-angle frames contain no session. Old frames already on the bus
      may arrive after a stop; firmware must reject them using local gating.

## Minimum firmware sequence

1. Start with motion prohibited, generate a new uint32 session, and periodically
   send HELLO: `01 01 session_BE 00 00`.
2. Accept only an ACK echoing that session with status=0. Retry on NOT_READY;
   report an error on VERSION_MISMATCH.
3. Send CONTROL every 80 ms: `03 01 session_BE flags seq`. Increment `seq` on
   each transmission, wrapping 255→0. Flags: red stop=02, red enable=03,
   blue stop=04, blue enable=05. Color zero is invalid even when stopping.
4. On relative-angle reception, first validate standard-frame type, DLC, and
   reserved bits. Ignore valid-angle frames unless locally enabled and the
   session is ready. Eight zero bytes immediately invalidate the measurement.
   Decode valid yaw/pitch and pass them to the firmware's own limited controller.
5. Invalidate measurements on receive timeout; choose a deployment-specific
   value, approximately 100–120 ms as a starting point. Never repeatedly
   accumulate the previous frame. Re-handshake and explicitly re-enable after
   vision disconnect/restart, CONTROL timeout, or malformed control.

## Vision-side bring-up

Repository: `/home/irm/iRM-Vision-2026`. Complete software acceptance first.
Enter the approved IDs and set `vision_can_ids_confirmed: true` in
`/home/irm/iRM-Vision-2026/configs/relative_current.yaml` only after agreement.
For the first hardware connection, inspect traffic with motion prohibited.

```bash
cd /home/irm/iRM-Vision-2026
make -C /home/irm/iRM-Vision-2026/build relative_servo_test relative_aim -j4
/home/irm/iRM-Vision-2026/build/relative_servo_test
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current.yaml -csv=/tmp/board.csv -stream=8080
```

Verify in order: NOT_READY→READY→blue enable→stop→red→blue again→CONTROL
heartbeat loss→re-handshake→camera/communication disconnect→SIGINT. Capture
traffic independently in each phase and compare quantized angles with the CSV;
check all-zero stops and `shoot=0`. Confirm horizontal/vertical signs before
testing a low-speed, limited closed loop. Exit with Ctrl+C; do not force-stop
the camera with `timeout`'s default SIGTERM.

Troubleshooting: no ACK → check direction, IDs, DLC, version. Persistent
NOT_READY → check model, geometry, and fresh images. ACK but no valid angles →
check CONTROL session/flags/sequence, color, image age, and PnP. Observations in
CSV but no bus output → inspect `publish_ok`, generation, and `tx_fail`.
Do not disable timeout protection to make a failing integration appear to work.