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

echo "[0/4] 环境自检"
if grep -qi microsoft /proc/version 2>/dev/null; then
  echo "  - 检测到 WSL2"
  case "${REPO_ROOT}" in
    /mnt/*)
      echo "  !! 仓库位于 /mnt/...（Windows 驱动器）。跨 9p 文件系统编译极慢且文件事件异常，"
      echo "     建议移到 WSL 自己的 ext4 上重新 clone，例如："
      echo "       git clone <url> ~/iRM-Vision-2026 && cd ~/iRM-Vision-2026 && bash scripts/setup_x86_dev.sh -y"
      ;;
  esac
  echo "  - 提示：WSL 内默认没有 USB 设备（工业相机 / UVC 相机 / 达妙 IMU / USB2CAN）。"
  echo "     需实测硬件时在 Windows 侧装 usbipd-win 并 attach；纯调试请用 configs/offline.yaml 回放。"
  echo "  - 提示：cv::imshow 需要 WSLg（Win11/已更新 Win10）；没有就加 -d display 时不要用图形界面。"
else
  echo "  - 非 WSL（原生 Linux）"
fi

echo "[1/4] 安装 apt 依赖（需要 sudo）"
sudo apt-get update
sudo apt-get install "${APT_ARGS[@]}" \
  build-essential cmake git \
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev \
  libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev libceres-dev \
  can-utils

echo "[2/4] 生成 .vscode/eigen_fix.h"
# 该文件被 .gitignore 忽略（AGENTS.md §2.3 要求不提交），每台开发机都要本地生成：
# IntelliSense 下 __ARM_NEON 会导致 Eigen 报错。
mkdir -p "${REPO_ROOT}/.vscode"
cat > "${REPO_ROOT}/.vscode/eigen_fix.h" <<'EOF'
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
EOF

echo "[3/4] 获取 ONNX Runtime（检测后端，可选但强烈建议）"
# auto_aim 的 ONNX Runtime 后端需要 third_party/onnxruntime（不入库，见 AGENTS.md §8.5）。
# 没有它也能编译，但检测会回退 OpenCV DNN，而 OpenCV < 4.9 跑不了本模型。
if bash "${REPO_ROOT}/scripts/fetch_onnxruntime.sh"; then
  echo "  - ONNX Runtime 就位"
else
  echo "  !! ONNX Runtime 未就位：检测后端将回退 OpenCV DNN（OpenCV < 4.9 时不可用）"
  echo "     稍后可重试：bash scripts/fetch_onnxruntime.sh"
fi

echo "[4/4] configure + 编译可移植目标子集"
# 不要复用别的机器的 build/（CMakeCache.txt 里记录了编译器与库路径）
cmake -B build
make -C build -j"$(nproc)" \
  serial tools io auto_aim tinympcstatic omniperception \
  planner_test planner_test_offline \
  camera_test usbcamera_test multi_usbcamera_test cboard_test dm_test \
  fire_test gimbal_test gimbal_response_test handeye_test \
  capture split_video \
  calibrate_camera calibrate_handeye calibrate_robotworld_handeye \
  minimum_vision_system auto_aim_test camera_thread_test \
  detect_freq_visual_test

cat <<'EOF'

完成。可以直接用的调试入口（⚠️ 短选项必须写 -c=<path>，写成 "-c <path>" 会被解析成空值）：
  bash scripts/run_local_tests.sh                    # 一键离线回归（下面几条 + 端到端）
  ./build/planner_test_offline configs/demo.yaml     # 纯规划（无需相机/IMU/CAN）
  ./build/camera_test -c=configs/offline.yaml        # 视频文件回放取流（实测 29.97 fps）
  ./build/dm_test -p=none                            # 无 IMU 回放模式自检
  ./build/auto_aim_test -e=200 -c=configs/offline.yaml  # 端到端回放：检测+跟踪+规划（需显示）
  ./build/camera_test -c=configs/camera.yaml -d      # 真实相机取流（需接相机）
  ./build/usbcamera_test -c=configs/uav.yaml         # USB 相机（需 /dev/videoN）
  ./build/cboard_test -c=configs/uav.yaml            # CAN 通信（需 can0）
  ./build/detect_freq_visual_test -c=configs/detect_freq.yaml -m=bench -n=60   # 检测管线频率（无显示）
  #   ↑ OpenCV < 4.9 时模型会 forward 失败，程序会打印原因并降级为只测取流/显示（不静默、不退出）

检测后端优先级：TensorRT（未接入）> ONNX Runtime（third_party/onnxruntime 就位时启用）>
OpenCV DNN（兜底，需 OpenCV >= 4.9），可用 yolo_name / yolov5_backend 键切换，见 AGENTS.md §8.5。

注意：uav / uav_debug / auto_buff* / camera_detect_test / detector_video_test /
usbcamera_detect_test 仍无法编译——它们依赖尚未迁移的 auto_aim::Detector/Classifier 与
auto_buff 的推理代码（AGENTS.md §4.1）。
EOF
