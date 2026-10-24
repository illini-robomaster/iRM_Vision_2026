#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/current_relative.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/visual_servo.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/mjpeg_server.hpp"
#include "tools/yaml.hpp"

// ---------------------------------------------------------------------------
// 相对角差（视觉伺服）入口：相机 → 检测 → 角差 → CAN
//
// 两条角差来源（-source=，默认 current）：
//   current：**当前帧观测**。每帧独立地过滤 → 确定性选目标 → 用这一块自己的角点做 PnP
//            → 参考系角差 → 整形 → 量化 → 发送。不依赖 Tracker（不需要连续命中和锁定），
//            也不做预测，没有「换目标首帧不发」，第一帧就能给出角差。
//   tracker：旧的跟踪路径（Tracker + VisualServo，EKF 锁定 + 换目标首帧不发），
//            保留兼容、行为不变；需要 Solver 的手眼外参键。
//
// 发送语义（见 documentation/relative_can_protocol.md）：
//   - 有效帧：control=1，带 Δyaw/Δpitch（量化后的值 = 总线上的字节）；
//   - 无效帧（无目标 / 过期 / 几何不可信 / 颜色不符）：control=0，角差与 shoot 清零，
//     让电控立刻停（不把上一帧的旧角度继续累加）；
//   - shoot 恒为 0；
//   - 退出时补发一帧 control=0（best effort），比等电控自己的接收超时更早。
//
// 配置要求（**在构造 CBoard 之前**校验，避免阻塞在 CAN/IMU 上）：
//   can_command_mode: "relative"   ← 否则拒绝启动（防止把角差当绝对角发出去）
//   cboard_wait_imu: false         ← 相对角差不依赖姿态；true 会让构造永久阻塞等四元数
// ---------------------------------------------------------------------------
const std::string keys =
  "{help h usage ? |                           | 输出命令行参数说明 }"
  "{config-path c  | configs/relative_current.yaml | yaml 配置文件路径（can_command_mode: "
  "relative）}"
  "{frames n       | 0                         | 最多处理的帧数，0 = 一直跑到 Ctrl+C }"
  "{dry-run        |                           | 只算不发：不打开 CAN，也不要求 can_command_mode }"
  "{source s       | current                   | 角差来源：current（当前帧观测）/ "
  "tracker（旧跟踪路径）}"
  "{csv            |                           | 逐帧 CSV 输出路径（留空 = 不写）}"
  "{stream         | 0                         | MJPEG 直播端口，0 = 关（无显示器时用它看 HUD）}"
  "{d display      |                           | 本地窗口显示（没有 X 时自动退化）}";

namespace
{
double percentile(std::vector<double> values, double p)
{
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(p * (values.size() - 1));
  return values[index];
}

std::string csv_header()
{
  return "frame,frame_t_ms,send_t_ms,age_ms,candidates,target_id,valid,reason,"
         "raw_yaw_deg,raw_pitch_deg,tx_yaw_deg,tx_pitch_deg,tx_yaw_1e4,tx_pitch_1e4,"
         "deadband,clamped,saturated,reproj_px,control,shoot,send_attempt,send_ok,send_ms,"
         "detect_ms";
}

// 逐帧 CSV：帧号 / 图像时间戳 / 发送时间戳 / 观测年龄 / 目标标识 / raw / TX / 有效性 /
// 无效原因 / 发送尝试与结果。CSV 里的 TX 就是总线上的 int16×1e4（见 current_relative.cpp），
// 所以「CSV 与 candump 对不上」只可能是发送链路问题，不是精度问题。
//   frame_t_ms / send_t_ms 都是**相对进程启动**的 steady_clock 毫秒；图像时间戳的来源是
//   「取流函数返回时刻」（MindVision SDK 的 CameraGetImageBuffer 返回时间），**不是曝光时刻**。
std::string csv_row(
  int frame, double frame_t_ms, double send_t_ms, double age_ms,
  const auto_aim::RelativeObservation & obs, bool attempt, bool ok, double send_ms,
  double detect_ms)
{
  return fmt::format(
    "{},{:.2f},{:.2f},{:.2f},{},{},{},{},{:.4f},{:.4f},{:.4f},{:.4f},{},{},{},{},{},{:.3f},{},"
    "{},{},{},{:.3f},{:.2f}",
    frame, frame_t_ms, send_t_ms, age_ms, obs.candidates, obs.target_id, obs.valid ? 1 : 0,
    obs.reason, obs.raw_yaw * 57.3, obs.raw_pitch * 57.3, obs.tx_yaw * 57.3, obs.tx_pitch * 57.3,
    io::quantize_angle_1e4(obs.tx_yaw), io::quantize_angle_1e4(obs.tx_pitch), obs.deadband ? 1 : 0,
    obs.clamped ? 1 : 0, obs.saturated ? 1 : 0, obs.reprojection, obs.command.control ? 1 : 0,
    obs.command.shoot ? 1 : 0, attempt ? 1 : 0, ok ? 1 : 0, send_ms, detect_ms);
}

// 旧跟踪路径 → 统一的观测结构。跟踪路径的整形发生在 VisualServo::aim() 内部，
// 所以这里 raw 与 TX 相同（不再二次整形），只补一次量化让 CSV 与总线一致。
auto_aim::RelativeObservation observe_via_tracker(
  auto_aim::VisualServo & servo, auto_aim::Tracker & tracker,
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  auto_aim::RelativeObservation obs;
  auto command = servo.aim(armors, targets);

  obs.candidates = static_cast<int>(armors.size());
  obs.valid = command.control;
  obs.reason = command.control ? "ok" : ("tracker_" + tracker.state());

  if (!targets.empty()) {
    obs.target_id = fmt::format(
      "tracker:{}:{}", auto_aim::ARMOR_NAMES[targets.front().name],
      auto_aim::ARMOR_TYPES[targets.front().armor_type]);
  }

  if (command.control) {
    command.yaw = static_cast<double>(io::quantize_angle_1e4(command.yaw)) / 1e4;
    command.pitch = static_cast<double>(io::quantize_angle_1e4(command.pitch)) / 1e4;
    command.shoot = false;
  } else {
    command = io::Command{false, false, 0.0, 0.0};
  }

  obs.raw_yaw = command.yaw;
  obs.raw_pitch = command.pitch;
  obs.tx_yaw = command.yaw;
  obs.tx_pitch = command.pitch;
  obs.command = command;
  return obs;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>("config-path");
  const auto max_frames = cli.get<int>("frames");
  const auto dry_run = cli.has("dry-run");
  const auto source = cli.get<std::string>("source");
  const auto csv_path = cli.get<std::string>("csv");
  const auto stream_port = cli.get<int>("stream");
  bool display = cli.has("display");

  if (source != "current" && source != "tracker") {
    tools::logger()->error("[relative_aim] -source 只能是 current 或 tracker，当前 '{}'", source);
    return 1;
  }
  const bool use_tracker = (source == "tracker");

  tools::Exiter exiter;

  // ===== 1) 配置校验：全部在构造 CBoard / Camera 之前完成 =====
  // 顺序很重要：CBoard 构造可能永久阻塞（等四元数），不能「先构造再检查配置」。
  auto yaml = tools::load(config_path);
  const auto command_mode = tools::optional_string(yaml, "can_command_mode", "absolute");
  if (!dry_run && command_mode != "relative") {
    tools::logger()->error(
      "[relative_aim] can_command_mode = \"{}\"：拒绝发送（防止把相对角差当绝对角发出去）。"
      "请在 {} 里设置 can_command_mode: \"relative\"，或加 -dry-run 只算不发。",
      command_mode, config_path);
    return 1;
  }
  if (!dry_run && tools::optional_bool(yaml, "cboard_wait_imu", true)) {
    tools::logger()->error(
      "[relative_aim] cboard_wait_imu 不是 false：相对角差不依赖姿态，但 CBoard 构造会阻塞等"
      "0x100 四元数（板上不发就永远停在 [Cboard] Waiting for q...）。"
      "请在 {} 里设置 cboard_wait_imu: false。",
      config_path);
    return 1;
  }
  const int relative_canid = yaml["relative_canid"] ? yaml["relative_canid"].as<int>() : 0xFE;
  if (!dry_run && (relative_canid <= 0 || relative_canid > 0x7FF)) {
    tools::logger()->error(
      "[relative_aim] relative_canid = {} 不是标准帧 ID（1..0x7FF），拒绝启动", relative_canid);
    return 1;
  }
  const double send_interval_ms = tools::optional_double(yaml, "relative_send_interval_ms", 0.0);

  tools::logger()->info("[relative_aim] 配置：{}", config_path);
  tools::logger()->info(
    "[relative_aim] 模式：{}，角差来源={}，相对帧 ID=0x{:X}，发送间隔={:.0f}ms（0 = 每帧都发）",
    dry_run ? "dry-run（不打开 CAN）" : "实发", source, relative_canid, send_interval_ms);

  // ===== 2) 角差来源 =====
  std::unique_ptr<auto_aim::CurrentRelativeEstimator> estimator;
  std::unique_ptr<auto_aim::Solver> solver;
  std::unique_ptr<auto_aim::Tracker> tracker;
  std::unique_ptr<auto_aim::VisualServo> servo;

  if (use_tracker) {
    solver = std::make_unique<auto_aim::Solver>(config_path);
    tracker = std::make_unique<auto_aim::Tracker>(config_path, *solver);
    servo = std::make_unique<auto_aim::VisualServo>(config_path);
    // 相对角差只用云台系（与姿态无关），世界系固定单位阵即可
    solver->set_R_gimbal2world(Eigen::Quaterniond::Identity());
    tools::logger()->warn(
      "[relative_aim] tracker 路径：raw 与 TX 相同（整形发生在 VisualServo 内部）；"
      "换目标首帧不发、要连续命中才会锁定（这是旧行为，不是本入口的默认路径）");
  } else {
    estimator = std::make_unique<auto_aim::CurrentRelativeEstimator>(config_path);
    tools::logger()->info("{}", estimator->describe());
    if (estimator->reference() == auto_aim::RelativeReference::camera_optical) {
      tools::logger()->info(
        "[relative_aim] camera_optical：角差相对**相机光轴**（参考点=相机光心），不需要手眼外参；"
        "与云台系参考轴的角差不是同一个量，别和别的程序混用");
    } else {
      tools::logger()->info(
        "[relative_aim] gimbal：角差相对**云台系**参考轴（已含 R_camera2gimbal / "
        "t_camera2gimbal）");
    }
  }

  auto_aim::YOLO detector(config_path, false);

  std::unique_ptr<io::CBoard> cboard;
  if (dry_run) {
    tools::logger()->warn("[relative_aim] dry-run：只计算相对角差，不打开 CAN");
  } else {
    cboard = std::make_unique<io::CBoard>(config_path);
    if (!cboard->relative()) {
      tools::logger()->error("[relative_aim] CBoard 不在 relative 模式，拒绝发送绝对角命令");
      return 1;
    }
  }

  io::Camera camera(config_path);

  // ===== 3) 观测输出：逐帧 CSV + HUD（imshow / MJPEG 直播）=====
  std::ofstream csv;
  if (!csv_path.empty()) {
    csv.open(csv_path);
    if (!csv) {
      tools::logger()->error("[relative_aim] 打不开 CSV 文件：{}", csv_path);
      return 1;
    }
    csv << csv_header() << '\n';
    csv.flush();
    tools::logger()->info("[relative_aim] 逐帧 CSV：{}", csv_path);
  }

  tools::MjpegServer stream(stream_port);
  if (display) tools::logger()->info("[relative_aim] 本地显示：按 q 退出");

  // ===== 4) 主循环 =====
  const std::string ref_text =
    use_tracker ? "gimbal(tracker)" : auto_aim::to_string(estimator->reference());

  const auto t_start = std::chrono::steady_clock::now();
  auto last_frame_time = std::chrono::steady_clock::time_point{};
  auto last_valid_send = t_start;
  bool has_sent_valid = false;

  int frames = 0;
  int valid_frames = 0;
  int sent = 0;
  int send_ok = 0;
  int send_fail = 0;
  int rate_limited = 0;
  double detect_ms_sum = 0.0;
  std::vector<double> ages_ms;

  cv::Mat img;
  std::chrono::steady_clock::time_point frame_time;

  while (!exiter.exit()) {
    camera.read(img, frame_time);
    ++frames;

    // 同一帧时间戳不重复处理：不把一张旧图当成新观测再发一次有效值
    if (frame_time == last_frame_time) continue;
    last_frame_time = frame_time;

    const auto t_capture = std::chrono::steady_clock::now();
    auto armors = detector.detect(img, frames);
    const auto t_detect = std::chrono::steady_clock::now();
    const double detect_ms = tools::delta_time(t_detect, t_capture) * 1000.0;
    detect_ms_sum += detect_ms;

    auto_aim::RelativeObservation obs;
    if (use_tracker) {
      auto targets = tracker->track(armors, frame_time);
      obs = observe_via_tracker(*servo, *tracker, armors, targets);
    } else {
      obs = estimator->observe(armors, img.size(), frame_time, t_detect);
    }

    const double frame_t_ms = tools::delta_time(frame_time, t_start) * 1000.0;
    const double age_ms = tools::delta_time(t_detect, frame_time) * 1000.0;
    ages_ms.push_back(age_ms);
    if (obs.valid) ++valid_frames;

    // ---- 发送：有效帧可按 relative_send_interval_ms 限频；无效帧立刻发（让电控马上停）----
    bool attempt = false;
    bool ok = false;
    double send_ms = 0.0;
    if (cboard) {
      const bool throttled =
        obs.valid && send_interval_ms > 0.0 && has_sent_valid &&
        tools::delta_time(t_detect, last_valid_send) * 1000.0 < send_interval_ms;

      if (throttled) {
        ++rate_limited;
      } else {
        attempt = true;
        const auto t_send = std::chrono::steady_clock::now();
        ok = cboard->send(obs.command);
        send_ms = tools::delta_time(std::chrono::steady_clock::now(), t_send) * 1000.0;
        ++sent;
        if (ok) {
          ++send_ok;
        } else {
          ++send_fail;
        }
        if (obs.valid) {
          has_sent_valid = true;
          last_valid_send = t_send;
        }
      }
    }

    // ---- 逐帧 CSV ----
    if (csv.is_open()) {
      const double send_t_ms =
        tools::delta_time(std::chrono::steady_clock::now(), t_start) * 1000.0;
      csv << csv_row(frames, frame_t_ms, send_t_ms, age_ms, obs, attempt, ok, send_ms, detect_ms)
          << '\n';
    }

    // ---- HUD（ASCII；中文在 Hershey 字体会变 ????）----
    if (display || stream_port > 0) {
      auto canvas = img.clone();
      for (const auto & armor : armors) {
        const bool selected = !obs.target_id.empty() && auto_aim::target_id(armor) == obs.target_id;
        tools::draw_points(
          canvas, armor.points, selected ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128),
          selected ? 2 : 1);
      }

      const auto color = obs.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
      tools::draw_text(
        canvas, fmt::format("src={} ref={} frame={}", source, ref_text, frames), {10, 30},
        {0, 255, 255}, 0.7, 2);
      tools::draw_text(
        canvas,
        fmt::format("raw  yaw={:+.2f} pitch={:+.2f} deg", obs.raw_yaw * 57.3, obs.raw_pitch * 57.3),
        {10, 60}, color, 0.7, 2);
      tools::draw_text(
        canvas,
        fmt::format("tx   yaw={:+.2f} pitch={:+.2f} deg", obs.tx_yaw * 57.3, obs.tx_pitch * 57.3),
        {10, 90}, color, 0.7, 2);
      tools::draw_text(
        canvas,
        fmt::format("valid={} reason={} cand={}", obs.valid ? 1 : 0, obs.reason, obs.candidates),
        {10, 120}, color, 0.7, 2);
      tools::draw_text(
        canvas, fmt::format("tgt={}", obs.target_id.empty() ? "-" : obs.target_id), {10, 150},
        {0, 255, 255}, 0.7, 2);
      tools::draw_text(
        canvas,
        fmt::format(
          "age={:.0f}ms detect={:.0f}ms sent={} ok={} fail={}", age_ms, detect_ms, sent, send_ok,
          send_fail),
        {10, 180}, {0, 255, 255}, 0.7, 2);

      if (stream_port > 0) stream.publish(canvas);
      if (display) {
        try {
          cv::imshow("relative_aim", canvas);
          if (cv::waitKey(1) == 'q') break;
        } catch (const cv::Exception & e) {
          // 无 X（headless Jetson）时 imshow 会抛异常：退化成只算/只发，程序继续跑
          tools::logger()->warn("[relative_aim] 本地显示不可用（{}），后续不再 imshow", e.what());
          display = false;
        }
      }
    }

    if (max_frames > 0 && frames >= max_frames) break;
  }

  // ===== 5) 退出：先把 CSV 落盘，再补一帧 control=0 =====
  if (csv.is_open()) csv.flush();

  if (cboard) {
    const auto t_send = std::chrono::steady_clock::now();
    const bool ok = cboard->send(io::Command{false, false, 0.0, 0.0});
    tools::logger()->info(
      "[relative_aim] 退出前补发一帧 control=0：{}（{:.2f} ms）", ok ? "成功" : "失败",
      tools::delta_time(std::chrono::steady_clock::now(), t_send) * 1000.0);
  }

  if (frames > 0) {
    tools::logger()->info(
      "[relative_aim] 共 {} 帧：有效 {}（{:.1f}%）；发送 {}（成功 {} / 失败 {}），限频跳过 {}",
      frames, valid_frames, 100.0 * valid_frames / frames, sent, send_ok, send_fail, rate_limited);
    tools::logger()->info(
      "[relative_aim] 检测 {:.1f} ms/帧；观测年龄 p50={:.1f} ms p95={:.1f} ms（图像时间戳 = "
      "取流返回时刻）",
      detect_ms_sum / frames, percentile(ages_ms, 0.5), percentile(ages_ms, 0.95));
  } else {
    tools::logger()->warn("[relative_aim] 没有处理任何帧");
  }

  return 0;
}
