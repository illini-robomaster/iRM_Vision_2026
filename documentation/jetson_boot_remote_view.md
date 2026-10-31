# Jetson boot service and SSH-only remote vision

## Quick operator entry point

The command `irm-vision` is installed at `~/.local/bin/irm-vision`, a symlink
to `scripts/visionctl.sh`. It works from any directory and does not store sudo
credentials. Administrative commands prompt through ordinary sudo if needed.

| Command | Purpose |
|---|---|
| `irm-vision start` / `stop` / `restart` | Control the live camera + CAN service |
| `irm-vision status` | Process status, not hardware readiness |
| `irm-vision logs` | Follow logs; Ctrl+C only stops viewing logs |
| `irm-vision check` | Inspect camera USB, CAN counters, engine, listener and snapshot |
| `irm-vision enable` / `disable` | Enable-and-start / disable-and-stop boot service |
| `irm-vision view` | Print SSH tunnel instructions and current Jetson addresses |
| `irm-vision preview [N]` | Camera-only recognition, **no CAN**, port 8080 |
| `irm-vision replay [N]` | Demo replay, **no CAN or industrial camera**, port 8081 |
| `irm-vision can-reset` | Explicit can0 reset at 1 Mbps; requires the vision service stopped |

Camera preview refuses to run while the boot service owns the camera. `N` is
an optional frame count (default 0 means run until Ctrl+C). Replay can be used
while no hardware is attached and does not modify the boot service. Both test
modes bind video only to loopback. Stop any other manual camera process before
starting preview or the live service; only the systemd owner is checked by the
shortcut.

### When the camera and CAN are connected later

1. Connect the camera and check CAN wiring/termination/board power. Keep the
   MCU aiming enable off initially.
2. Run `irm-vision check`. The boot service already retries camera opening;
   a manual restart is available with `irm-vision restart`.
3. Use `irm-vision view` to set up the SSH tunnel and open the browser.
4. Inspect `irm-vision logs` for first-frame READY, HELLO and CONTROL. Only
   after checking image geometry/signs and safe actuator conditions should the
   MCU request `enable=1`.
5. If CAN is BUS-OFF or needs reinitialization after wiring is corrected:
   `irm-vision stop`, `irm-vision can-reset`, `irm-vision start`. Resetting a
   bus does not repair wiring and is never done implicitly by `check`.

For camera-only bring-up:

```bash
irm-vision stop
irm-vision preview     # no CAN; Ctrl+C exits cleanly
irm-vision start      # restore the normal boot service afterwards
```

With no hardware at all: `irm-vision replay` and an SSH tunnel forwarding 8081
instead of 8080 provide a demo recognition stream. `irm-vision replay 30`
performs a finite compute-only smoke test.

Shortcut validation: `bash -n` passes; help/status/check/view work from the
installed symlink; mocked systemctl/sudo routing and argument/conflict guards
pass. The real 30-frame replay completed with TensorRT, 28 valid observations,
no CAN opened, clean exit. Camera-only acquisition and physical CAN reset were
not exercised with absent hardware.

## Deployment

The installed service is `irm-vision.service`, copied from
`deploy/irm-vision.service` to `/etc/systemd/system/irm-vision.service`.
It is enabled for boot and runs as user `irm`, not root. It requires the
existing `irm-cansetup.service` (Classical CAN `can0`, 1 Mbps), then starts:

```bash
./build/relative_aim -c=configs/relative_boot.yaml -s=current \
  -stream=8080 -stream-bind=127.0.0.1
```

This is real-camera MindVision + TensorRT + relative-angle CAN v1, not video
replay. It retries camera opening if the device is absent. The model engine
must already exist at `assets/yolov5_0526_fp16.engine` (not tracked in Git).
The service restarts on process exit with a ten-second delay, and uses SIGINT
for normal camera cleanup and a best-effort control=0 stop frame.

The deployment config enables the four IDs matching the current Embedded 2026
`eoh_infant` source: relative=`0x0FE`, HELLO=`0x310`, ACK=`0x311`,
CONTROL=`0x312`. This installation was requested by the operator. The allocation
is source-aligned, **not a physical-bus acceptance certificate**. Recheck the
complete bus allocation and flashed firmware before powering actuators.
The conservative `configs/relative_current.yaml` remains unchanged.

No valid angle is sent without a fresh image, a matching HELLO session and
CONTROL(enable=1). The current deployment intentionally selects the nearest
valid armor to image center **regardless of its detected color**:

```yaml
relative_filter_color: false
vision_use_control_color: false
```

`vision_use_control_color` defaults to `true` for other configurations, retaining
the existing v1 target-color override. With it disabled, the estimator's local
`relative_filter_color` setting controls filtering. Both switches are required
for color-agnostic operation; merely disabling the local filter is not enough
when remote color override is enabled. The wire CONTROL frame is still validated
and required for session/enable/heartbeat. HUD `ctl` shows accepted wire color;
`target=any` indicates the current color-agnostic selector. There is no own-team
color inversion. This test mode can select friendly armor; use it only in a
controlled test environment. Confidence, keypoint, geometry and freshness checks
remain active. Selection is per-image, not a persistent identity lock.

Color-agnostic update validation: `relative_aim` and `relative_servo_test` build;
134 checks, 0 failures, including nearest red/blue cross-color selection in
both input orders. A separate real-TensorRT vcan replay accepted valid angles
under both red and blue CONTROL; 43 blue-target observations were accepted while
CONTROL said red. No physical control frames were injected by this test.

Relative aiming does not require measured gimbal yaw/pitch. The independent board-state telemetry
client is not started by this entry point; its 21-byte/four-fragment codec has
been tested separately. No automatic firing is implemented.

## Viewing from your computer

Run this on **your computer**, not inside a Jetson shell:

```bash
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \
  -L 127.0.0.1:8080:127.0.0.1:8080 irm@192.168.137.20
```

Keep the SSH session running and open `http://127.0.0.1:8080/` in your computer's
browser. A single frame is at `http://127.0.0.1:8080/snapshot.jpg`.
Windows PowerShell's OpenSSH client accepts the same options (use one line).
If local port 8080 is busy, use `-L 127.0.0.1:18080:127.0.0.1:8080` and browse
to port 18080 instead.

For a shell and tunnel in the same window, omit `-N`.
For access away from the local network, if both devices are authorized on the
same Tailscale network, replace the host with this Jetson's current Tailscale
address `100.65.141.79`. LAN/DHCP addresses can change; verify with `ip -br addr`.
No router port-forwarding or public video listener is required.

The MJPEG endpoint is deliberately bound to loopback: unauthenticated HTTP
video is not exposed on Wi-Fi, Ethernet or Tailscale. SSH authentication is the
access boundary. Verify the SSH host key on your first connection. SSH was
already enabled; no SSH authentication settings, firewall rules or passwords
were changed by this deployment. Prefer SSH keys for routine access.

The display includes detected armor outlines, selected target, relative angle,
frame age, inference time, CAN session state and send statistics. When the MCU
has not enabled aiming, `reason=disabled` is expected; detections still run.
When no camera frame has arrived, there is no real image to display. A running
service or listening HTTP port alone does not prove camera readiness.

## Service operations (on Jetson over SSH)

```bash
systemctl status irm-vision --no-pager
journalctl -u irm-vision -f
sudo systemctl restart irm-vision
sudo systemctl stop irm-vision
sudo systemctl disable --now irm-vision  # stop and remove boot activation
```

Stop this service before opening the same industrial camera from another
program. Restart it after testing. Use SIGINT, not SIGTERM/kill -9, for manual
camera processes.

To install on the same user/path layout again:

```bash
make -C build relative_aim board_state_can_test minipc_test relative_servo_test -j4
sudo install -m 0644 deploy/irm-vision.service /etc/systemd/system/irm-vision.service
sudo systemctl daemon-reload
sudo systemctl enable --now irm-vision
mkdir -p ~/.local/bin
ln -s /home/irm/iRM-Vision-2026/scripts/visionctl.sh ~/.local/bin/irm-vision
```

The symlink command is for a fresh installation; the link already exists on
this Jetson. If your shell omits `~/.local/bin` from PATH, use the full path or
add that directory to your shell profile.

Other machines need the service user/path, camera calibration, engine, CAN
setup service and ID allocation adjusted. Merely copying this unit is not a
portable hardware installation.

## Validation performed on Jetson (2026-10-31)

- The four build targets above compile successfully on aarch64.
- CAN and UART offline suites pass; relative-servo suite: 131 checks, 0 failures.
- Board-state vcan on `vcan42`: normal streaming, 800 ms NOT_READY delay,
  and a 1000 ms telemetry stall all pass. The stall causes a fresh session and
  successful stream recovery. Final streams have zero rejected/dropped frames.
- Relative-control vcan on `vcan_test`: real TensorRT/video replay passes
  HELLO, pre-enable gating, enable/disable, color switch, heartbeat timeout,
  new session, malformed CONTROL fail-closed behavior and int16/CSV agreement
  (91 valid frames in the final run). Slow-image 1 fps test passes; normal
  SIGINT stop frame was observed in 55 ms, slow-image shutdown in 356 ms.
- Replay MJPEG: HTTP 200 JPEG, multipart stream, loopback-only socket and clean
  SIGINT exit verified. An authenticated SSH `direct-tcpip` channel through the
  real local sshd returned a valid 127750-byte JPEG. This tests server-side SSH
  forwarding, not reachability from the operator's computer.
- Installed service is enabled and active, loads TensorRT, opens SocketCAN,
  listens only on `127.0.0.1:8080`, and survives a manual service restart.
- No actual reboot was performed. Power-cycle acceptance remains pending.

### Hardware limitations observed

The MindVision USB device (`f622:0001`) was absent from `lsusb`; the service
logs `Failed to enumerate camera!` and keeps retrying. Physical live imagery
and hot-plug recovery cannot yet be certified.

After service startup/restart, `can0` showed ERROR-PASSIVE with RX error counter
127, two total received packets and no sustained valid traffic; a subsequent
five-second passive capture received zero frames. This is a **physical CAN
failure/pending bring-up**, not a successful MCU link. Check board power,
FDCAN1 wiring, CAN H/L, common ground, endpoint termination and matching bit
rate. Do not interpret vcan success as hardware success. A service stop/restart
may attempt a control=0 stop frame; successful execution by the MCU is not
proved by a local write.

The Embedded-side reconnect issue found in the previous review is unchanged:
after a Jetson restart or heartbeat outage, firmware may need a reset/new HELLO
because its current `vision_session_ready` flag never expires. Fixing that
state machine is a separate Embedded task.

Temporary test artifacts are under `/tmp/vcan-{normal,delayed,recovery}-*` and
`/tmp/vision-boot-vcan-final*`; they are not committed. The two vcan interfaces
are test-only and are not configured for boot. No credentials are stored in
this repository or the service unit.
