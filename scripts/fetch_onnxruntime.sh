#!/usr/bin/env bash
# 下载 / 解压官方预编译的 ONNX Runtime 到 third_party/onnxruntime（幂等，可重复执行）
#
# 用途：给 auto_aim 的 ONNX Runtime 推理后端（tasks/auto_aim/yolos/yolov5_ort.*）提供
#       include/ 与 lib/，见 AGENTS.md §8.5。该目录不入库（.gitignore 忽略 third_party/）。
#
# 用法：
#   bash scripts/fetch_onnxruntime.sh              # 按当前架构自动选择 x64 / aarch64 包
#   ORT_VERSION=1.17.3 bash scripts/fetch_onnxruntime.sh
#   ORT_TARBALL=/path/to/onnxruntime-linux-x64-1.17.3.tgz bash scripts/fetch_onnxruntime.sh  # 离线
#   ORT_DIR=/opt/onnxruntime bash scripts/fetch_onnxruntime.sh   # 换安装目录（CMake 侧用 ORT_DIR 覆盖）
set -euo pipefail

ORT_VERSION="${ORT_VERSION:-1.17.3}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ORT_DIR:-${REPO_ROOT}/third_party/onnxruntime}"

case "$(uname -m)" in
  x86_64 | amd64) PKG_ARCH="x64" ;;
  aarch64 | arm64) PKG_ARCH="aarch64" ;;
  *)
    echo "[onnxruntime] 不支持的架构: $(uname -m)（只提供 x64 / aarch64 预编译包）" >&2
    exit 1
    ;;
esac

if [ -f "${DEST}/include/onnxruntime_cxx_api.h" ] && ls "${DEST}"/lib/libonnxruntime.so* >/dev/null 2>&1; then
  echo "[onnxruntime] 已就位: ${DEST} (v$(cat "${DEST}/VERSION_NUMBER" 2>/dev/null || echo "?"))，跳过下载"
  exit 0
fi

NAME="onnxruntime-linux-${PKG_ARCH}-${ORT_VERSION}"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${NAME}.tgz"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

TARBALL="${ORT_TARBALL:-}"
if [ -n "${TARBALL}" ]; then
  echo "[onnxruntime] 使用本地包: ${TARBALL}"
else
  TARBALL="${TMP_DIR}/${NAME}.tgz"
  echo "[onnxruntime] 下载 ${URL}"
  if ! curl -fL --retry 3 --connect-timeout 15 -o "${TARBALL}" "${URL}"; then
    echo "[onnxruntime] 下载失败。若本机不能访问网络，请手动下载 ${NAME}.tgz 后用" >&2
    echo "               ORT_TARBALL=<文件> bash scripts/fetch_onnxruntime.sh 重试" >&2
    echo "               （没有 ORT 也能编译，检测后端会回退 OpenCV DNN，见 AGENTS.md §8.5）" >&2
    exit 1
  fi
fi

tar xzf "${TARBALL}" -C "${TMP_DIR}"
mkdir -p "${DEST}"
# 只拷运行时需要的三样：头文件、库、版本号（tarball 里顶层目录名随版本变化，用通配匹配）
EXTRACTED="$(find "${TMP_DIR}" -maxdepth 1 -type d -name 'onnxruntime-linux-*' | head -n 1)"
if [ -z "${EXTRACTED}" ]; then
  echo "[onnxruntime] 压缩包结构不符合预期，找不到 onnxruntime-linux-*/ 目录" >&2
  exit 1
fi

cp -r "${EXTRACTED}/include" "${EXTRACTED}/lib" "${DEST}/"
[ -f "${EXTRACTED}/VERSION_NUMBER" ] && cp "${EXTRACTED}/VERSION_NUMBER" "${DEST}/"

echo "[onnxruntime] 已解压到 ${DEST}"
ls -1 "${DEST}/lib"
echo "[onnxruntime] 提示：重新执行 cmake -B build 让 CMake 找到它（tasks/auto_aim 会定义 SPVISION_HAS_ORT）"
