#!/usr/bin/env bash
# 本地离线回归（x86_64 / WSL2）：不需要相机 / IMU / CAN / 真实硬件
#
# 覆盖用例：
#   1) planner_test_offline  纯规划器离线跑（configs/demo.yaml）
#   2) camera_test           视频文件回放取流（configs/offline.yaml）
#   3) dm_test               无 IMU 回放模式自检（-p=none）
#   4) auto_aim_test         端到端回放：检测（ONNX Runtime / cv::dnn）+ 跟踪 + 规划（需显示环境）
#   5) detect_freq_visual_test  检测链路的阶段性耗时 / 频率（cap/pre/detect/draw，bench 模式不需显示）
#   6) detect_freq_visual_test  传统方法二次矫正（use_traditional=true，Detector+Classifier）走通
#   7) replay_ab.sh             同录像 A/B 一致性基线（ORT vs cv::dnn，检测结果逐帧对比）
#
# 用法：
#   bash scripts/run_local_tests.sh          # 跑全部 7 个用例
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

# 5) 检测链路的阶段性耗时：cap（取流）/ pre（letterbox）/ detect（模型全链路）/ draw，
#    -m=bench 不开窗口，所以连显示环境都不需要。需要任一可用推理后端
#    （TensorRT / ONNX Runtime / OpenCV >= 4.9 的 DNN，见 AGENTS.md §8.5）：三个都没有时
#    程序会降级成「只测取流」（这是它的设计，不是失败），这里按 SKIP 处理并说明原因。
DF_BIN=./build/detect_freq_visual_test
if [ ! -x "${DF_BIN}" ]; then
  skip 5 "detect_freq_visual_test：检测频率与阶段耗时（bench）" \
    "缺少 ${DF_BIN}，先跑 bash scripts/setup_x86_dev.sh -y"
else
  # 先探一帧：只有真的选中了后端（日志出现 backend=）才认为本机可测
  if ! timeout -s INT 30 "${DF_BIN}" -c=configs/detect_freq.yaml -m=bench -n=1 \
      >"${LOG_DIR}/5_probe.log" 2>&1 || ! grep -Eq 'backend=' "${LOG_DIR}/5_probe.log"; then
    skip 5 "detect_freq_visual_test：检测频率与阶段耗时（bench）" \
      "本机没有可用推理后端（TensorRT / ONNX Runtime / OpenCV >= 4.9 都没有），见 AGENTS.md §8.5"
  else
    run_case 5 "detect_freq_visual_test：检测频率与阶段耗时（bench 40 帧）" 120 'detect.*avg' \
      "${DF_BIN}" "${DF_BIN}" -c=configs/detect_freq.yaml -m=bench -n=40
  fi
fi

# 6) 传统方法二次矫正（yaml 的 use_traditional=true）：YOLO 后端里再挂一个 Detector+Classifier，
#    对每个检出做传统方法矫正。覆盖的是「classifier/detector 去 OpenVINO 之后是否真的能在
#    推理后端里跑起来」这条链路（AGENTS.md §4.1），配置用第 5 条改一行派生（不新增配置文件）。
if [ ! -x "${DF_BIN}" ]; then
  skip 6 "detect_freq_visual_test：传统方法二次矫正（use_traditional=true）" \
    "缺少 ${DF_BIN}，先跑 bash scripts/setup_x86_dev.sh -y"
else
  TRAD_YAML="${LOG_DIR}/6_detect_traditional.yaml"
  sed 's/^use_traditional: false/use_traditional: true/' configs/detect_freq.yaml >"${TRAD_YAML}"
  if ! grep -q '^use_traditional: true' "${TRAD_YAML}"; then
    skip 6 "detect_freq_visual_test：传统方法二次矫正（use_traditional=true）" \
      "configs/detect_freq.yaml 里没有再出现 use_traditional: false，派生配置失败"
  elif ! timeout -s INT 30 "${DF_BIN}" -c="${TRAD_YAML}" -m=bench -n=1 \
      >"${LOG_DIR}/6_probe.log" 2>&1 || ! grep -Eq 'backend=' "${LOG_DIR}/6_probe.log"; then
    skip 6 "detect_freq_visual_test：传统方法二次矫正（use_traditional=true）" \
      "本机没有可用推理后端（TensorRT / ONNX Runtime / OpenCV >= 4.9 都没有），见 AGENTS.md §8.5"
  else
    run_case 6 "detect_freq_visual_test：传统方法二次矫正（Detector+Classifier，bench 20 帧）" 120 \
      'use_traditional=true' "${DF_BIN}" "${DF_BIN}" -c="${TRAD_YAML}" -m=bench -n=20
  fi
fi

#   7) A/B 一致性基线：同一段录像上跑两组 yaml 变体，逐帧对比检测结果（数量 / 标签 / 置信度 /
#      角点像素差，比较逻辑在 scripts/csv_ab.py）。这里用 ORT vs cv::dnn 两个后端，对应
#      AGENTS.md §8.5 的人工实测基线（conf 差 5.8e-5 / 角点差 0.07px）。replay_ab.sh 自己会
#      探针：可用变体少于 2 个（本机没有可用推理后端）返回 2 -> 按 SKIP 处理；超差返回 1 -> FAIL。
AB_BIN=./scripts/replay_ab.sh
if [ ! -x "${AB_BIN}" ]; then
  skip 7 "replay_ab.sh：同录像 A/B 一致性（ORT vs cv::dnn）" "缺少 ${AB_BIN}"
else
  AB_NAME="replay_ab.sh：同录像 A/B 一致性（ORT vs cv::dnn，120 帧）"
  AB_LOG="${LOG_DIR}/7_replay_ab.log"
  LOG_DIR="${LOG_DIR}/7_replay_ab" timeout -s INT 240 bash "${AB_BIN}" assets/demo/demo.avi -n=120 \
    -e ort=yolov5_backend:ort -e dnn=yolov5_backend:dnn >"${AB_LOG}" 2>&1
  AB_RC=$?
  if [ "${AB_RC}" -eq 2 ]; then
    skip 7 "replay_ab.sh：同录像 A/B 一致性（ORT vs cv::dnn）" \
      "可用推理后端少于 2 个（脚本返回 2），见 ${AB_LOG}"
  else
    AB_SEL=0
    if [ "${#ONLY[@]}" -gt 0 ]; then
      AB_SEL=1
      for want in "${ONLY[@]}"; do
        [ "${want}" = "7" ] && AB_SEL=0
      done
    fi
    if [ "${AB_SEL}" -eq 0 ]; then
      printf '\n%s[7]%s %s %s(限时 240s)%s\n' \
        "${C_BLU}" "${C_RST}" "${AB_NAME}" "${C_DIM}" "${C_RST}"
      if [ "${AB_RC}" -eq 0 ] && grep -q 'AB PASS' "${AB_LOG}"; then
        PASS=$((PASS + 1))
        printf '  %sPASS%s  rc=0，命中标记 /AB PASS/  %s(log: %s)%s\n' \
          "${C_GRN}" "${C_RST}" "${C_DIM}" "${AB_LOG}" "${C_RST}"
        SUMMARY+=("${C_GRN}PASS${C_RST} [7] ${AB_NAME} —— 两后端逐帧一致")
      else
        FAIL=$((FAIL + 1))
        printf '  %sFAIL%s  rc=%s，A/B 未通过（详见日志）  %s(log: %s)%s\n' \
          "${C_RED}" "${C_RST}" "${AB_RC}" "${C_DIM}" "${AB_LOG}" "${C_RST}"
        printf '  %s日志末尾 15 行：%s\n' "${C_DIM}" "${C_RST}"
        tail -n 15 "${AB_LOG}" | sed 's/^/    /'
        SUMMARY+=("${C_RED}FAIL${C_RST} [7] ${AB_NAME} —— rc=${AB_RC}")
      fi
    fi
  fi
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
