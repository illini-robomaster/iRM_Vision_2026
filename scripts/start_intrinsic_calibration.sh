#!/usr/bin/env bash
# Camera-only calibration launcher. No CBoard or IMU connection.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${REPO_ROOT}/configs/mv_sua133gc.yaml"
BIND="127.0.0.1"
PORT="8081"
OUTPUT="${REPO_ROOT}/records/calibration"
IMAGES=""
BUILD=0
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage: bash scripts/start_intrinsic_calibration.sh [options]

  --config=PATH   Camera YAML (default: configs/mv_sua133gc.yaml)
  --bind=IPv4     Bind address (default: 127.0.0.1)
  --port=NUMBER   HTTP port (default: 8081)
  --output=PATH   Session output root (default: records/calibration)
  --images=PATH   Offline image directory; do not open a camera
  --lan           Bind to 0.0.0.0; trusted LAN only, no authentication
  --build         Configure and incrementally build before starting
  --dry-run       Validate paths and print command; do not build/start
  -h, --help      Show help

Relative paths are resolved from the repository root, not the calling directory.
Stop with Ctrl+C to release the camera normally. Do not use sudo or force-kill.
EOF
}

fail() { printf 'Error: %s\n' "$*" >&2; exit 2; }
absolute_path() {
  case "$1" in
    /*) printf '%s\n' "$1" ;;
    *) printf '%s/%s\n' "${REPO_ROOT}" "$1" ;;
  esac
}

for argument in "$@"; do
  case "${argument}" in
    --config=*) CONFIG="${argument#*=}" ;;
    --bind=*) BIND="${argument#*=}" ;;
    --port=*) PORT="${argument#*=}" ;;
    --output=*) OUTPUT="${argument#*=}" ;;
    --images=*) IMAGES="${argument#*=}"; [[ -n "${IMAGES}" ]] || fail 'Empty images path' ;;
    --lan) BIND="0.0.0.0" ;;
    --build) BUILD=1 ;;
    --dry-run) DRY_RUN=1 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown option: ${argument}. Use --help." ;;
  esac
done

[[ -n "${CONFIG}" && -n "${OUTPUT}" ]] || fail 'Config/output paths must not be empty'
[[ "${PORT}" =~ ^[0-9]{1,5}$ ]] || fail 'Port must be an integer in 1..65535'
PORT=$((10#${PORT}))
(( PORT >= 1 && PORT <= 65535 )) || fail 'Port must be in 1..65535'
IFS='.' read -r -a OCTETS <<< "${BIND}"
[[ "${BIND}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ && ${#OCTETS[@]} -eq 4 ]] || fail 'Bind must be an IPv4 address'
for octet in "${OCTETS[@]}"; do
  [[ ${#octet} -le 3 ]] && (( 10#${octet} <= 255 )) || fail 'Invalid IPv4 octet'
done
CONFIG="$(absolute_path "${CONFIG}")"
OUTPUT="$(absolute_path "${OUTPUT}")"
[[ -f "${CONFIG}" ]] || fail "Configuration not found: ${CONFIG}"
if [[ -n "${IMAGES}" ]]; then
  IMAGES="$(absolute_path "${IMAGES}")"
  [[ -d "${IMAGES}" ]] || fail "Image directory not found: ${IMAGES}"
fi

BIN="${REPO_ROOT}/build/intrinsic_calibration_gui"
COMMAND=("${BIN}" "-c=${CONFIG}" "-b=${BIND}" "-p=${PORT}" "-o=${OUTPUT}")
[[ -z "${IMAGES}" ]] || COMMAND+=("-i=${IMAGES}")
printf 'Camera intrinsic calibration — CBoard/IMU disabled\n'
printf 'Config: %s\nOutput: %s\n' "${CONFIG}" "${OUTPUT}"
if [[ "${BIND}" == '0.0.0.0' ]]; then
  printf 'Browser: http://<JETSON_IP>:%s/ (trusted LAN only; no authentication)\n' "${PORT}"
else
  printf 'Browser: http://%s:%s/\n' "${BIND}" "${PORT}"
fi
if [[ "${BIND}" == '127.0.0.1' ]]; then
  printf 'On your laptop: ssh -L %s:127.0.0.1:%s irm@JETSON_IP\n' "${PORT}" "${PORT}"
  printf 'Then open http://127.0.0.1:%s/ on your laptop.\n' "${PORT}"
fi
printf 'Stop: Ctrl+C. Close other camera programs before starting.\nCommand:'
printf ' %q' "${COMMAND[@]}"
printf '\n'
if (( DRY_RUN )); then exit 0; fi

cd "${REPO_ROOT}"
if (( BUILD )); then
  cmake -S "${REPO_ROOT}" -B "${REPO_ROOT}/build"
  make -C "${REPO_ROOT}/build" intrinsic_calibration_gui -j2
fi
[[ -x "${BIN}" ]] || fail 'Executable missing. Run this script with --build first.'
# Replace the shell so Ctrl+C reaches the program and triggers normal camera cleanup.
exec "${COMMAND[@]}"