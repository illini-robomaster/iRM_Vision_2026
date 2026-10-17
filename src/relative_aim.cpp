#include <fmt/core.h>

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/visual_servo.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// ---------------------------------------------------------------------------
// 相对角差（视觉伺服）入口：相机 → 检测 → 跟踪 → 视觉伺服 → CAN
//
// 与 src/standard.cpp 的区别：不需要 IMU/编码器姿态反馈，命令是「目标相对相机
// 光轴的角差」，由电控按增量执行。要求 config 里 can_command_mode: "relative"，
// 否则直接拒绝启动（避免把很小的角差当成绝对角发出去）。
// 协议见 documentation/relative_can_protocol.md
// ---------------------------------------------------------------------------
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/standard4.yaml | yaml 配置文件路径（需 can_command_mode: relative）}"
  "{frames n       | 0                      | 最多处理的帧数，0 = 一直跑到 Ctrl+C }"
  "{dry-run        |                        | 只算不发：不打开 CAN，打印相对角差 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>("config-path");
  auto max_frames = cli.get<int>("frames");
  auto dry_run = cli.has("dry-run");

  tools::Exiter exiter;

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::VisualServo servo(config_path);

  // dry-run 不构造 CBoard：没有 CAN 硬件也能验证「检测→跟踪→角差」这条链路
  std::unique_ptr<io::CBoard> cboard;
  if (dry_run) {
    tools::logger()->warn("[relative_aim] dry-run：只计算相对角差，不打开 CAN");
  } else {
    cboard = std::make_unique<io::CBoard>(config_path);
    if (!cboard->relative()) {
      tools::logger()->error(
        "[relative_aim] can_command_mode 不是 \"relative\"，拒绝发送绝对角命令；"
        "请在 {} 里设置 can_command_mode: \"relative\"",
        config_path);
      return 1;
    }
  }

  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  // 相对角差只用到 xyz_in_gimbal（与姿态无关），世界系固定为单位阵即可：
  // 这样即使板上不发四元数，Solver/Tracker 也能正常工作
  solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());

  auto last_log = std::chrono::steady_clock::now();
  int frame_count = 0;

  while (!exiter.exit()) {
    camera.read(img, t);

    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);
    auto command = servo.aim(armors, targets);

    if (cboard) cboard->send(command);

    frame_count++;

    if (tools::delta_time(std::chrono::steady_clock::now(), last_log) >= 1.0) {
      last_log = std::chrono::steady_clock::now();
      if (command.control)
        tools::logger()->info(
          "[relative_aim] dyaw={:+.3f}deg dpitch={:+.3f}deg tracker={} armors={}",
          command.yaw * 57.3, command.pitch * 57.3, tracker.state(), armors.size());
      else
        tools::logger()->info(
          "[relative_aim] no command (tracker={}, armors={})", tracker.state(), armors.size());
    }

    if (max_frames > 0 && frame_count >= max_frames) break;
  }

  tools::logger()->info("[relative_aim] 共处理 {} 帧，退出", frame_count);
  return 0;
}
