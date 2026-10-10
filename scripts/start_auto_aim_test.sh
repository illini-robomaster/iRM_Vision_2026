#!/usr/bin/env bash
# Camera + armor detection preview only; no gimbal, CBoard or firing commands.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${ROOT}/configs/mv_sua133gc.yaml"
PORT=8080
FRAMES=0
BUILD=0
DRY=0
EXTRA=()
fail() { printf 'Error: %s\n' "$*" >&2; exit 2; }
for arg in "$@"; do
  case "$arg" in
    --config=*) CONFIG="${arg#*=}" ;;
    --port=*) PORT="${arg#*=}" ;;
    --frames=*) FRAMES="${arg#*=}" ;;
    --no-yolo) EXTRA+=(-no-yolo) ;;
    --build) BUILD=1 ;;
    --dry-run) DRY=1 ;;
    -h|--help)
      cat <<'EOF'
Camera + armor detection test (NOT tracking/planning/firing).
Options: --config=PATH --port=8080 --frames=0 --build --dry-run --no-yolo
Relative config paths are resolved from the repository root.
Browser: http://JETSON_IP:8080/ (trusted LAN only, no authentication).
The stream binds all interfaces. Do not expose it to the Internet.
Stop with Ctrl+C; close calibration/other camera processes first.
If inference fails, the existing test warns and continues camera-only.
--no-yolo explicitly disables inference; it is NOT an auto-aim test.
EOF
      exit 0 ;;
    *) fail "Unknown option: $arg" ;;
  esac
done
[[ -n "$CONFIG" ]] || fail 'Empty config path'
[[ "$CONFIG" == /* ]] || CONFIG="${ROOT}/${CONFIG}"
[[ -f "$CONFIG" ]] || fail "Configuration not found: $CONFIG"
[[ "$PORT" =~ ^[0-9]{1,5}$ ]] || fail 'Invalid port'
PORT=$((10#$PORT))
(( PORT >= 1 && PORT <= 65535 )) || fail 'Port must be 1..65535'
[[ "$FRAMES" =~ ^[0-9]{1,8}$ ]] || fail 'Frames must be 0..99999999'
FRAMES=$((10#$FRAMES))
BIN="${ROOT}/build/detect_freq_visual_test"
COMMAND=("$BIN" "-c=$CONFIG" -m=live "-stream=$PORT" "-n=$FRAMES" "${EXTRA[@]}")
printf 'Detection preview only: no CBoard/gimbal/firing.\nBrowser: http://JETSON_IP:%s/\n' "$PORT"
printf 'Trusted LAN only. Close other camera programs. Stop with Ctrl+C.\nCommand:'
printf ' %q' "${COMMAND[@]}"
printf '\n'
(( DRY == 0 )) || exit 0
cd "$ROOT"
if (( BUILD )); then
  [[ -f "${ROOT}/build/Makefile" ]] || cmake -S "$ROOT" -B "${ROOT}/build"
  make -C "${ROOT}/build" detect_freq_visual_test -j2
fi
[[ -x "$BIN" ]] || fail 'Executable missing; use --build'
exec "${COMMAND[@]}"