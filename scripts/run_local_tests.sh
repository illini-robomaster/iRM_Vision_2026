#!/usr/bin/env bash
# 本地离线回归（x86_64 / WSL2）：不需要相机 / IMU / CAN / 真实硬件
#
# 覆盖用例：
#   1) planner_test_offline  纯规划器离线跑（configs/demo.yaml）
#   2) camera_test           视频文件回放取流（configs/offline.yaml）
#   3) dm_test               无 IMU 回放模式自检（-p=none）
#   4) auto_aim_test         端到端回放：检测（ONNX Runtime / cv::dnn）+ 跟踪 + 规划（需显示环境）
#
# 用法：
#   bash scripts/run_local_tests.sh          # 跑全部
#   bash scripts/run_local_tests.sh 2 4      # 只跑第 2、4 个用例
#   LOG_DIR=/tmp/mylogs bash scripts/run_local_tests.sh
#
# 退出码：0 = 没有失败（SKIP 不影响退出码）；1 = 至少一个用例 FAIL；2 = 环境不满足
#
# 判定：这些程序多数是死循环，用 timeout 限时；返回码 ∈ {0,124,130,143} 且日志里出现期望
#       标记才算 PASS（标记取自 AGENTS.md §8.4 / tests 实测输出）。日志留在 LOG_DIR 里便于排查。
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

LOG_DIR="${LOG_DIR:-/tmp/sp_vision_local_tests}"
rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"

if [ -t 1 ]; then
  C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'
  C_BLU=$'\033[34m'; C_DIM=$'\033[2m'; C_RST=$'\033[0m'
else
  C_RED=""; C_GRN=""; C_YEL=""; C_BLU=""; C_DIM=""; C_RST=""
fi

PASS=0
FAIL=0
SKIP=0
SUMMARY=()
ONLY=("$@")

printf '%s==== 本地离线回归（%s）====%s\n' "${C_BLU}" "${REPO_ROOT}" "${C_RST}"

if [ ! -d build ]; then
  printf '%sbuild/ 不存在：先执行 bash scripts/setup_x86_dev.sh -y%s\n' "${C_RED}" "${C_RST}"
  exit 2
fi

skip() {  # skip <序号> <名称> <原因>
  if [ "${#ONLY[@]}" -gt 0 ]; then
    local hit=1
    for want in "${ONLY[@]}"; do
      [ "${want}" = "$1" ] && hit=0
    done
    [ "${hit}" -ne 0 ] && return 0
  fi
  SKIP=$((SKIP + 1))
  printf '  %sSKIP%s  %s（%s）\n' "${C_YEL}" "${C_RST}" "$2" "$3"
  SUMMARY+=("${C_YEL}SKIP${C_RST} [$1] $2 —— $3")
}

# run_case <序号> <名称> <限时秒> <期望标记(ERE)> <可执行文件> <命令...>
run_case() {
  local idx="$1" name="$2" limit="$3" marker="$4" bin="$5"
  shift 5

  if [ "${#ONLY[@]}" -gt 0 ]; then
    local hit=1
    for want in "${ONLY[@]}"; do
      [ "${want}" = "${idx}" ] && hit=0
    done
    [ "${hit}" -ne 0 ] && return 0
  fi

  local log="${LOG_DIR}/${idx}_$(echo "${name}" | tr -c '[:alnum:]' '_').log"
  printf '\n%s[%s]%s %s %s(限时 %ss)%s\n' \
    "${C_BLU}" "${idx}" "${C_RST}" "${name}" "${C_DIM}" "${limit}" "${C_RST}"

  if [ ! -x "${bin}" ]; then
    skip "${idx}" "${name}" "缺少 ${bin}，先跑 bash scripts/setup_x86_dev.sh -y"
    return 0
  fi

  timeout -s INT "${limit}" "$@" >"${log}" 2>&1
  local rc=$?

  local rc_ok=1
  case "${rc}" in
    0 | 124 | 130 | 143) rc_ok=0 ;;  # 0=自行退出, 124=被 timeout 正常终止, 130/143=收到信号退出
  esac
  local marker_ok=1
  if grep -Eq "${marker}" "${log}"; then marker_ok=0; fi

  if [ "${rc_ok}" -eq 0 ] && [ "${marker_ok}" -eq 0 ]; then
    PASS=$((PASS + 1))
    printf '  %sPASS%s  rc=%s，命中标记 /%s/  %s(log: %s)%s\n' \
      "${C_GRN}" "${C_RST}" "${rc}" "${marker}" "${C_DIM}" "${log}" "${C_RST}"
    SUMMARY+=("${C_GRN}PASS${C_RST} [${idx}] ${name} (rc=${rc})")
  else
    FAIL=$((FAIL + 1))
    local why=""
    [ "${rc_ok}" -ne 0 ] && why="返回码 rc=${rc} 不在 {0,124,130,143}"
    [ "${marker_ok}" -ne 0 ] && why="${why:+${why}；}日志未命中标记 /${marker}/"
    printf '  %sFAIL%s  %s  %s(log: %s)%s\n' "${C_RED}" "${C_RST}" "${why}" "${C_DIM}" "${log}" "${C_RST}"
    printf '  %s日志末尾 15 行：%s\n' "${C_DIM}" "${C_RST}"
    tail -n 15 "${log}" | sed 's/^/    /'
    SUMMARY+=("${C_RED}FAIL${C_RST} [${idx}] ${name} —— ${why}")
  fi
}

# ---------------------------------------------------------------- 用例定义

run_case 1 "planner_test_offline：纯规划（configs/demo.yaml）" 8 'Plan Yaw' \
  ./build/planner_test_offline ./build/planner_test_offline configs/demo.yaml

run_case 2 "camera_test：视频文件回放取流（configs/offline.yaml）" 10 'demo\.avi' \
  ./build/camera_test ./build/camera_test -c=configs/offline.yaml

run_case 3 "dm_test：无 IMU 回放模式（-p=none）" 5 'z0\.00 y0\.00 x0\.00' \
  ./build/dm_test ./build/dm_test -p=none

# 4) 端到端：检测（ORT/cv::dnn）+ 跟踪 + 规划。只跑 60 帧（-e=60），几十秒内结束。
#    程序里有 cv::imshow，需要显示环境；没有 DISPLAY 但有 xvfb-run 时自动套用。
AA_BIN=./build/auto_aim_test
AA_CMD=("${AA_BIN}" -e=60 -c=configs/offline.yaml)
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ] && command -v xvfb-run >/dev/null 2>&1; then
  AA_CMD=(xvfb-run -a "${AA_CMD[@]}")
fi
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ] && ! command -v xvfb-run >/dev/null 2>&1; then
  skip 4 "auto_aim_test：端到端回放" \
    "没有 DISPLAY / WAYLAND_DISPLAY，也没有 xvfb-run（cv::imshow 需要显示环境）"
else
  run_case 4 "auto_aim_test：端到端回放（检测+跟踪+规划，60 帧）" 120 'yolo: [0-9.]+ms' \
    "${AA_BIN}" "${AA_CMD[@]}"
fi

# ---------------------------------------------------------------- 汇总

printf '\n%s==== 汇总 ====%s\n' "${C_BLU}" "${C_RST}"
for line in "${SUMMARY[@]}"; do
  printf '  %s\n' "${line}"
done
printf '\nPASS=%s%d%s  FAIL=%s%d%s  SKIP=%s%d%s   （日志：%s）\n' \
  "${C_GRN}" "${PASS}" "${C_RST}" "${C_RED}" "${FAIL}" "${C_RST}" "${C_YEL}" "${SKIP}" "${C_RST}" \
  "${LOG_DIR}"

if [ "${FAIL}" -gt 0 ]; then
  printf '%s结果：FAIL（%d 个用例失败，见上面的日志末尾）%s\n' "${C_RED}" "${FAIL}" "${C_RST}"
  exit 1
fi

printf '%s结果：PASS%s\n' "${C_GRN}" "${C_RST}"
exit 0
