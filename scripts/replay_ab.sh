#!/usr/bin/env bash
# 同一段录像上跑多组配置，自动逐帧对比检测结果（A/B 一致性基线，AGENTS.md §4.1.4 / §8.7）
#
# 为什么要它：换推理后端 / 改一个阈值键之后，「识别结果到底有没有变」以前靠自己盯 -dump 的
# 数字，容易漏、不可复跑。这里把口径固定下来：同一条视频、同一个程序的 bench 模式、只改 yaml
# 里的一个键，逐帧对比装甲板数量 / 颜色编号类型标签 / 置信度 / 角点像素差，超差即 FAIL
# （比较逻辑在 scripts/csv_ab.py，纯标准库，不需要 numpy / pandas）。
#
# 用法：
#   bash scripts/replay_ab.sh [video.avi] [-c=<基配置>] [-n=<帧数>] [-t=<每变体限时秒>] \
#                             -e <名字>=<键:值[,键:值...]> [-e ...]
#
# 例（两条真正跑过的）：
#   # 后端一致性：ORT vs cv::dnn，期望 AB PASS（基线见 AGENTS.md §8.5：conf 差 5.8e-5 / 0.07px）
#   bash scripts/replay_ab.sh assets/demo/demo.avi -n=120 \
#        -e ort=yolov5_backend:ort -e dnn=yolov5_backend:dnn
#   # 反例自证：改了阈值就该被抓住，期望 AB FAIL
#   bash scripts/replay_ab.sh assets/demo/demo.avi -n=120 \
#        -e lo=min_confidence:0.8 -e hi=min_confidence:0.95
#
# 真机片段：先在有相机的机器上录（yaml 里把 record_video 设为 true）：
#   ./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=bench -n=300   # -> records/xxx.avi
# 再把 records/xxx.avi 传给本脚本即可（脚本会强制 camera_name: "video" + video_loop: false，
# 基配置里的相机键被忽略，所以真机配置也能直接复用）。
# 注意 -n 不能超过视频帧数：video_loop=false 时读到尾会抛「reached end of」（报错里会提示）。
#
# 变体语法：<名字>=<键:值>，多个键用逗号分隔，例如 -e v=yolov5_backend:ort,use_traditional:true
#   只改「顶层标量键」；键不在基配置里时追加到派生 yaml 末尾。值按原样写进 yaml，
#   字符串请自己带引号（如 -e x=video_path:\"a b.avi\"）。
#
# 退出码：0 = 全部对比通过（日志里有 AB PASS）
#         1 = 有变体跑失败或对比超差（AB FAIL）
#         2 = 环境/用法不满足（可用变体少于 2 个，例如本机没有可用推理后端）-> 调用方可当 SKIP
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

VIDEO="assets/demo/demo.avi"
BASE_CONFIG="configs/detect_freq.yaml"
FRAMES=120
LIMIT=300
BIN="./build/detect_freq_visual_test"
LOG_DIR="${LOG_DIR:-/tmp/sp_vision_replay_ab}"

usage() { awk 'NR > 1 && /^set -/ { exit } NR > 1' "$0" | sed 's/^# \{0,1\}//'; }

NAME_LIST=()
SPEC_LIST=()
while [ $# -gt 0 ]; do
  opt="$1"
  val=""
  case "${opt}" in
    *=*) val="${opt#*=}" ;;
    -e | -c | -n | -t | -v)
      # 也接受「-e 名字=键:值」这种空格写法（值的开头是 '-' 时请改用 -e=...）
      if [ $# -ge 2 ] && [ "${2#-}" = "$2" ]; then
        val="$2"
        shift
      fi
      ;;
  esac
  case "${opt%%=*}" in
    -h | --help)
      usage
      exit 0
      ;;
    -c) BASE_CONFIG="${val}" ;;
    -n) FRAMES="${val}" ;;
    -t) LIMIT="${val}" ;;
    -v) VIDEO="${val}" ;;
    -e)
      NAME_LIST+=("${val%%=*}")
      SPEC_LIST+=("${val#*=}")
      ;;
    -*) echo "未知参数：${opt}" >&2; usage; exit 2 ;;
    *) VIDEO="${opt}" ;;
  esac
  shift
done

if [ -t 1 ]; then
  C_GRN=$'\033[32m'; C_RED=$'\033[31m'; C_YEL=$'\033[33m'; C_DIM=$'\033[2m'; C_RST=$'\033[0m'
else
  C_GRN=""; C_RED=""; C_YEL=""; C_DIM=""; C_RST=""
fi
say() { printf '%s\n' "$*"; }

if [ "${#NAME_LIST[@]}" -eq 0 ]; then
  echo "至少要有一个 -e <名字>=<键:值> 变体" >&2
  usage
  exit 2
fi
for i in "${!NAME_LIST[@]}"; do
  if [ -z "${NAME_LIST[$i]}" ] || [ -z "${SPEC_LIST[$i]}" ]; then
    echo "-e 需要写成 <名字>=<键:值>（收到 name='${NAME_LIST[$i]}' spec='${SPEC_LIST[$i]}'）" >&2
    exit 2
  fi
done
command -v python3 >/dev/null 2>&1 || { echo "需要 python3（scripts/csv_ab.py）" >&2; exit 2; }
[ -x "${BIN}" ] || { echo "缺少 ${BIN}：bash scripts/setup_x86_dev.sh -y" >&2; exit 2; }
[ -f "${BASE_CONFIG}" ] || { echo "基配置不存在：${BASE_CONFIG}" >&2; exit 2; }
[ -f "${VIDEO}" ] || { echo "视频不存在：${VIDEO}" >&2; exit 2; }

rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"

# yaml_set <file> <key> <value>：顶层标量键「有则整行替换、无则追加」。
# 不用 sed，是因为键可能不存在（追加）且要避免把值里的字符当正则处理。
yaml_set() {
  local file="$1" key="$2" value="$3"
  awk -v k="${key}" -v v="${value}" '
    BEGIN { hits = 0 }
    $0 ~ "^" k ":" { print k ": " v; hits++; next }
    { print }
    END { if (hits == 0) print k ": " v }
  ' "${file}" >"${file}.tmp" && mv "${file}.tmp" "${file}"
}

# derive <out.yaml> <name=key:value[,key:value...]>：从基配置派生一份「回放 + 变体」配置
derive() {
  local out="$1" spec="$2" kv k v
  cp "${BASE_CONFIG}" "${out}"
  # 强制走视频回放：基配置里可能是真机相机（mindvision），这里一律覆盖成 video
  yaml_set "${out}" camera_name '"video"'
  yaml_set "${out}" video_path "\"${VIDEO}\""
  yaml_set "${out}" video_loop "false"
  IFS=',' read -r -a pairs <<<"${spec}"
  for kv in "${pairs[@]}"; do
    [ -z "${kv}" ] && continue
    k="${kv%%:*}"
    v="${kv#*:}"
    if [ -z "${k}" ] || [ "${k}" = "${kv}" ]; then
      echo "变体键格式错误（应为 键:值）：${kv}"
      return 1
    fi
    yaml_set "${out}" "${k}" "${v}"
  done
  grep -q '^camera_name: *"video"' "${out}" || { echo "派生配置缺少 camera_name: video"; return 1; }
  grep -q "^video_path: *\"" "${out}" || { echo "派生配置缺少 video_path"; return 1; }
  return 0
}

say "${C_DIM}录像：${VIDEO}   基配置：${BASE_CONFIG}   帧数：${FRAMES}   日志：${LOG_DIR}${C_RST}"

OK_NAMES=()
OK_CSV=()
SKIP_NAMES=()
RUN_FAIL=0

for i in "${!NAME_LIST[@]}"; do
  name="${NAME_LIST[$i]}"
  spec="${SPEC_LIST[$i]}"
  yaml="${LOG_DIR}/${name}.yaml"
  csv="${LOG_DIR}/${name}.csv"
  log="${LOG_DIR}/${name}.log"
  say ""
  say "${C_DIM}---- 变体 ${name}：${spec} ----${C_RST}"

  if ! derive "${yaml}" "${spec}" >"${LOG_DIR}/${name}_derive.log" 2>&1; then
    say "  ${C_RED}FAIL${C_RST} 派生配置失败：$(tail -1 "${LOG_DIR}/${name}_derive.log")"
    RUN_FAIL=1
    continue
  fi

  # 探针：先跑 1 帧确认真的选中了推理后端。本机没 TensorRT / 没 ORT / OpenCV < 4.9 时程序会
  # 降级成「只测取流」（设计如此，见 AGENTS.md §8.5），这种情况按 SKIP 处理而不是 FAIL。
  rm -f "${csv}"
  timeout -s INT 90 "${BIN}" -c="${yaml}" -m=bench -n=1 -dump="${csv}" >"${log}" 2>&1
  if ! grep -Eq 'backend=' "${log}"; then
    reason="$(grep -E '检测模型加载失败|\[(YOLO|YOLOV5|DNN|ORT|TRT)[^]]*\]' "${log}" | tail -1 |
      sed 's/^.*\] *//')"
    say "  ${C_YEL}SKIP${C_RST} 没有可用推理后端（${reason:-见 ${log}}）"
    SKIP_NAMES+=("${name}")
    continue
  fi

  timeout -s INT "${LIMIT}" "${BIN}" -c="${yaml}" -m=bench -n="${FRAMES}" -dump="${csv}" >"${log}" 2>&1
  rc=$?
  if [ ! -s "${csv}" ] || ! grep -Eq 'detect.*avg' "${log}"; then
    say "  ${C_RED}FAIL${C_RST} 变体 ${name} 没跑完（rc=${rc}，见 ${log}）"
    grep -q 'reached end of' "${log}" &&
      say "         原因：-n=${FRAMES} 超过视频帧数（video_loop=false 读到尾会抛错），把 -n 调小"
    RUN_FAIL=1
    continue
  fi

  stat="$(grep -E 'detect.*avg' "${log}" | head -1 | sed 's/^.*\] *//')"
  say "  ${C_GRN} OK ${C_RST} 变体 ${name}：${stat}（CSV ${csv}）"
  OK_NAMES+=("${name}")
  OK_CSV+=("${csv}")
done

# ---------------------------------------------------------------- 对比与汇总
if [ "${#OK_NAMES[@]}" -lt 2 ]; then
  say ""
  say "可用变体 ${#OK_NAMES[@]} 个（${OK_NAMES[*]:-无}），少于 2 个，无法做 A/B"
  [ "${#SKIP_NAMES[@]}" -gt 0 ] && say "（SKIP：${SKIP_NAMES[*]}，原因见各自日志）"
  say "AB SKIP"
  exit 2
fi

BASE_NAME="${OK_NAMES[0]}"
BASE_CSV="${OK_CSV[0]}"
say ""
say "==== A/B 对比（基准变体：${BASE_NAME}）===="
CMP_FAIL=0
for idx in "${!OK_NAMES[@]}"; do
  [ "${idx}" -eq 0 ] && continue
  other="${OK_NAMES[$idx]}"
  out="${LOG_DIR}/ab_${BASE_NAME}_vs_${other}.txt"
  python3 scripts/csv_ab.py "${BASE_CSV}" "${OK_CSV[$idx]}" \
    --name-a="${BASE_NAME}" --name-b="${other}" >"${out}" 2>&1
  crc=$?
  sed 's/^/  /' "${out}"
  if [ "${crc}" -eq 0 ]; then
    say "  ${C_GRN}[PASS]${C_RST} ${BASE_NAME} vs ${other}（详见 ${out}）"
  else
    say "  ${C_RED}[FAIL]${C_RST} ${BASE_NAME} vs ${other}（详见 ${out}）"
    CMP_FAIL=1
  fi
done

say ""
if [ "${RUN_FAIL}" -eq 0 ] && [ "${CMP_FAIL}" -eq 0 ]; then
  say "${C_GRN}AB PASS${C_RST}：${#OK_NAMES[@]} 个可用变体全部一致（SKIP ${#SKIP_NAMES[@]} 个）"
  say "日志与 CSV：${LOG_DIR}"
  exit 0
fi
say "${C_RED}AB FAIL${C_RST}：见上面的 FAIL 行（SKIP ${#SKIP_NAMES[@]} 个），日志与 CSV：${LOG_DIR}"
exit 1
