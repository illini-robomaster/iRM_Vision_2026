#include "cboard.hpp"

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000),
  can_(read_yaml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1))
// 注意: callback的运行会早于Cboard构造函数的完成
{
  if (wait_imu_) {
    tools::logger()->info("[Cboard] Waiting for q...");
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);
  } else {
    // 板上不发四元数（读不到电机/云台姿态）时用这个分支：构造立即返回，
    // imu_at() 恒返回单位四元数，视觉链路不依赖姿态反馈。
    tools::logger()->warn("[Cboard] cboard_wait_imu=false：不等四元数，imu_at() 返回单位四元数");
    data_ahead_.q = Eigen::Quaterniond::Identity();
    data_behind_.q = Eigen::Quaterniond::Identity();
    data_ahead_.timestamp = std::chrono::steady_clock::time_point{};
    data_behind_.timestamp = std::chrono::steady_clock::time_point{};
  }

  if (command_mode_ == CommandMode::relative) {
    tools::logger()->info(
      "[Cboard] 相对角差模式：0x{:X} 上发 Δyaw/Δpitch（int16 BE ×1e4 rad）", relative_canid_);
  }

  tools::logger()->info("[Cboard] Opened.");
}

bool CBoard::relative() const { return command_mode_ == CommandMode::relative; }

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (!wait_imu_) return Eigen::Quaterniond::Identity();

  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

namespace
{
// 角度 → int16（单位 1e-4 rad），饱和而不是回绕：超范围时按 ±3.2767 rad 截断，
// 避免 int16 溢出把「往右转」变成「往左转」。
int16_t encode_angle_1e4(double rad)
{
  const double scaled = rad * 1e4;
  if (scaled >= 32767.0) return 32767;
  if (scaled <= -32768.0) return -32768;
  return static_cast<int16_t>(std::lround(scaled));
}
}  // namespace

void encode_relative_command(const Command & command, uint8_t data[8])
{
  data[0] = static_cast<uint8_t>((command.control ? 0x01 : 0x00) | (command.shoot ? 0x02 : 0x00));
  data[1] = 0;

  const int16_t dyaw = encode_angle_1e4(command.yaw);
  const int16_t dpitch = encode_angle_1e4(command.pitch);
  data[2] = static_cast<uint8_t>((dyaw >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(dyaw & 0xFF);
  data[4] = static_cast<uint8_t>((dpitch >> 8) & 0xFF);
  data[5] = static_cast<uint8_t>(dpitch & 0xFF);
  data[6] = 0;
  data[7] = 0;
}

void CBoard::send(Command command) const
{
  can_frame frame;
  frame.can_dlc = 8;

  // 相对角差模式：只发 Δyaw/Δpitch，不发绝对角（见 documentation/relative_can_protocol.md）
  if (command_mode_ == CommandMode::relative) {
    frame.can_id = relative_canid_;
    encode_relative_command(command, frame.data);

    try {
      can_.write(&frame);
    } catch (const std::exception & e) {
      tools::logger()->warn("{}", e.what());
    }
    return;
  }

  frame.can_id = send_canid_;
  frame.data[0] = (command.control) ? 1 : 0;
  frame.data[1] = (command.shoot) ? 1 : 0;
  frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;
  frame.data[3] = (int16_t)(command.yaw * 1e4);
  frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
  frame.data[5] = (int16_t)(command.pitch * 1e4);
  frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
  frame.data[7] = (int16_t)(command.horizon_distance * 1e4);

  try {
    can_.write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void CBoard::callback(const can_frame & frame)
{
  auto timestamp = std::chrono::steady_clock::now();

  if (frame.can_id == quaternion_canid_) {
    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp});
  }

  else if (frame.can_id == bullet_speed_canid_) {
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  // 新增可选键（缺省 = 旧行为）：can_command_mode / relative_canid / cboard_wait_imu
  const auto command_mode = tools::optional_string(yaml, "can_command_mode", "absolute");
  if (command_mode == "relative") {
    command_mode_ = CommandMode::relative;
  } else if (command_mode == "absolute") {
    command_mode_ = CommandMode::absolute;
  } else {
    throw std::runtime_error(
      "Unknown can_command_mode: '" + command_mode + "' (expected \"absolute\" or \"relative\")");
  }

  relative_canid_ = yaml["relative_canid"] ? yaml["relative_canid"].as<int>() : 0xFE;
  wait_imu_ = tools::optional_bool(yaml, "cboard_wait_imu", true);

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }

  return yaml["can_interface"].as<std::string>();
}

}  // namespace io