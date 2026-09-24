#!/usr/bin/env bash
# x86_64 开发机（笔记本 / WSL2）一键准备：装依赖 -> 生成 .vscode/eigen_fix.h -> 编译可移植子集
#
# 用途：Jetson 不在手上时，在笔记本上编译并做离线/回放调试。
# 仅支持 x86_64 Linux（含 WSL2）；macOS / Windows 原生不支持，原因见 AGENTS.md §8。
#
# 用法：bash scripts/setup_x86_dev.sh [-y]
#   -y  跳过 apt 的交互确认
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

APT_ARGS=(-y --no-install-recommends)
if [ "${1:-}" != "-y" ]; then
  APT_ARGS=()
fi

echo "[1/3] 安装 apt 依赖（需要 sudo）"
sudo apt-get update
sudo apt-get install "${APT_ARGS[@]}" \
  build-essential cmake git \
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev \
  libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev libceres-dev \
  can-utils

echo "[2/3] 生成 .vscode/eigen_fix.h"
# 该文件被 .gitignore 忽略（AGENTS.md §2.3 要求不提交），每台开发机都要本地生成：
# IntelliSense 下 __ARM_NEON 会导致 Eigen 报错。
mkdir -p "${REPO_ROOT}/.vscode"
cat > "${REPO_ROOT}/.vscode/eigen_fix.h" <<'EOF'
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
EOF

echo "[3/3] configure + 编译可移植目标子集"
# 不要复用别的机器的 build/（CMakeCache.txt 里记录了编译器与库路径）
cmake -B build
make -C build -j"$(nproc)" \
  serial tools io auto_aim tinympcstatic omniperception \
  planner_test planner_test_offline \
  camera_test usbcamera_test multi_usbcamera_test cboard_test dm_test \
  fire_test gimbal_test gimbal_response_test handeye_test \
  capture split_video \
  calibrate_camera calibrate_handeye calibrate_robotworld_handeye

cat <<'EOF'

完成。可以直接用的离线调试入口：
  ./build/planner_test_offline configs/demo.yaml     # 纯规划（无需相机/IMU/CAN）
  ./build/camera_test -c configs/camera.yaml -d      # 相机取流（需接相机）
  ./build/usbcamera_test      -c configs/uav.yaml     # USB 相机
  ./build/cboard_test -c configs/uav.yaml             # CAN 通信（需 can0）

注意：需要推理后端的目标（uav / minimum_vision_system / auto_buff* / *_detect_test /
detector_video_test / auto_aim_test）在 TensorRT 迁移完成前无法编译，见 AGENTS.md §4、§8。
EOF
