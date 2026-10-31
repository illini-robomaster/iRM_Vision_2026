#!/usr/bin/env bash
# Operator entry point; no stored credentials, no implicit physical CAN reset.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
SERVICE=irm-vision.service

usage() {
  cat <<'HELP'
Usage: irm-vision COMMAND [frames]
  start       Start camera recognition + session-gated relative CAN
  stop        Stop cleanly (SIGINT); boot activation is unchanged
  restart     Restart after connecting hardware / changing configuration
  status      Show systemd status (running does NOT mean hardware ready)
  logs        Follow service logs; Ctrl+C exits viewing, not the service
  check       Inspect camera, CAN, model, listener and current image readiness
  can-reset   Reinitialize can0 at 1 Mbps; stop the service first (requires sudo)
  enable      Enable boot startup and start now (requires sudo)
  disable     Disable boot startup and stop now (requires sudo)
  view        Show the SSH tunnel and browser instructions
  preview [N] Camera recognition only, no CAN; stop the service first
              Local-only video: port 8080; N=0/default runs until Ctrl+C
  replay [N]  Demo-video recognition only, no CAN; local-only port 8081
              Does not use the industrial camera or change the boot service
  help        Show this help

Live mode uses configs/relative_boot.yaml. No valid angles before a fresh
image + MCU HELLO + CONTROL(enable=1). No automatic firing.
HELP
}

admin() {
  if [[ $EUID -eq 0 ]]; then
    systemctl "$@"
  else
    sudo systemctl "$@"
  fi
}

check_hardware() {
  echo '=== Service (process state only) ==='
  systemctl show "$SERVICE" -p LoadState -p ActiveState -p SubState -p MainPID -p NRestarts
  printf 'Boot: '
  systemctl is-enabled "$SERVICE" || true
  echo '=== MindVision USB f622:0001 ==='
  local found=0 device
  for device in /sys/bus/usb/devices/*; do
    [[ -r "$device/idVendor" && -r "$device/idProduct" ]] || continue
    if [[ $(< "$device/idVendor") == f622 && $(< "$device/idProduct") == 0001 ]]; then
      echo "Detected: ${device##*/} (USB presence, not frame readiness)"
      found=1
    fi
  done
  if [[ $found -eq 0 ]]; then
    echo 'WAITING: camera not connected; boot service retries automatically.'
  fi
  echo '=== Physical CAN (no frames transmitted by this check) ==='
  ip -details -statistics link show dev can0 || true
  echo 'ERROR-PASSIVE/BUS-OFF or zero traffic requires hardware checks; vcan is not hardware acceptance.'
  echo '=== TensorRT engine ==='
  if [[ -r "$ROOT/assets/yolov5_0526_fp16.engine" ]]; then
    ls -lh "$ROOT/assets/yolov5_0526_fp16.engine"
  else
    echo 'MISSING: assets/yolov5_0526_fp16.engine'
  fi
  echo '=== Video listener ==='
  ss -lnt | awk 'NR == 1 || $4 ~ /:8080$/'
  echo '=== Current image availability ==='
  if command -v curl >/dev/null; then
    local code
    code="$(curl --noproxy '*' -s -o /dev/null -w '%{http_code}' --max-time 3 \
      http://127.0.0.1:8080/snapshot.jpg)" || true
    if [[ $code == 200 ]]; then
      echo 'Image available (HTTP 200); this does not certify MCU control.'
    else
      echo "WAITING: no current image (HTTP ${code:-unavailable}); inspect logs."
    fi
  else
    echo 'curl unavailable; use the browser via SSH forwarding.'
  fi
  echo 'This is an informational check, not an end-to-end acceptance result.'
}

view_help() {
  cat <<'VIEW'
On YOUR COMPUTER, keep this terminal open (replace <JETSON_IP>):
  ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -L 127.0.0.1:8080:127.0.0.1:8080 irm@<JETSON_IP>
Then browse: http://127.0.0.1:8080/
Snapshot:    http://127.0.0.1:8080/snapshot.jpg
For replay, forward port 8081 instead of 8080 on both sides.
No image is expected before a real camera frame; a running service alone is not enough.
Current Jetson IPv4 addresses:
VIEW
  ip -4 -br addr show
}

command="${1:-help}"
if [[ $# -gt 2 || ( $# -eq 2 && $command != preview && $command != replay ) ]]; then
  usage >&2
  exit 2
fi
case "$command" in
  start|stop|restart) admin "$command" "$SERVICE" ;;
  enable|disable) admin "$command" --now "$SERVICE" ;;
  status) systemctl status "$SERVICE" --no-pager -l ;;
  logs) exec journalctl -u "$SERVICE" -n 60 -f ;;
  check) check_hardware ;;
  can-reset)
    if systemctl is-active --quiet "$SERVICE"; then
      echo 'Stop vision before resetting the bus: irm-vision stop' >&2
      exit 1
    fi
    echo 'Reinitializing can0 with the installed irm-cansetup.service (1 Mbps).'
    admin restart irm-cansetup.service
    ;;
  view) view_help ;;
  preview|replay)
    frames="${2:-0}"
    if [[ ! $frames =~ ^[0-9]+$ ]]; then
      echo 'frames must be a nonnegative integer' >&2
      exit 2
    fi
    if [[ ! -x "$ROOT/build/relative_aim" ]]; then
      echo 'Build first: make -C build relative_aim -j4' >&2
      exit 1
    fi
    if [[ $command == preview ]]; then
      if systemctl is-active --quiet "$SERVICE"; then
        echo 'Camera is reserved by the boot service. Run: irm-vision stop' >&2
        echo 'After preview, restore it with: irm-vision start' >&2
        exit 1
      fi
      config=configs/relative_boot.yaml
      port=8080
    else
      config=configs/relative_vcan.yaml
      port=8081
    fi
    echo "COMPUTE ONLY: CAN is not opened. Stream: 127.0.0.1:$port"
    cd -- "$ROOT"
    exec ./build/relative_aim "-c=$config" -dry-run "-n=$frames" "-stream=$port" -stream-bind=127.0.0.1
    ;;
  help|-h|--help) usage ;;
  *) usage >&2; exit 2 ;;
esac
