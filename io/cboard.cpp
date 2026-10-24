#include "cboard.hpp"

#include <algorithm>
#include <mutex>

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

  // 相对角差模式的 ID 校验与生效值日志在 read_yaml() 里（构造 can_ 之前完成，
  // 配置写错时不会先把 SocketCAN 拉起来）
  tools::logger()->info("[Cboard] Opened.");
}

bool CBoard::relative() const { return command_mode_ == CommandMode::relative; }

void CBoard::clear_frame_handlers()
{
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  handlers_.clear();
}

int CBoard::relative_canid() const { return relative_canid_; }

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
// 发送失败日志限流：30Hz 下每帧都失败会把日志刷爆（spdlog 还是 flush_on(info)），
// 1Hz 一条 + 累计计数足够定位问题，又不会反过来拖慢控制循环。
bool should_log_failure()
{
  static std::mutex mtx;
  static auto last = std::chrono::steady_clock::time_point{};
  static int suppressed = 0;
  std::lock_guard<std::mutex> lock(mtx);
  auto now = std::chrono::steady_clock::now();
  if (tools::delta_time(now, last) >= 1.0) {
    if (suppressed > 0)
      tools::logger()->debug("[Cboard] 最近 1s 内还有 {} 次发送失败未逐条打印", suppressed);
    last = now;
    suppressed = 0;
    return true;
  }
  ++suppressed;
  return false;
}
}  // namespace

void encode_relative_command(const Command & command, uint8_t data[8])
{
  // 非有限角度 = 无效观测：control 清零、角差写 0（见 cboard.hpp 的说明）
  const bool finite = std::isfinite(command.yaw) && std::isfinite(command.pitch);
  const bool control = command.control && finite;
  // shoot 恒为 0：相对角差帧表达的是「这一帧测到的角差」，
  // 单看角差无法判断是否对准，开火必须由电控/后续逻辑决定（v1 协议只传角差）。
  // 即使调用方写了 shoot=true 也不置位 —— 编码层强制，避免上层误发。
  const bool shoot = false;

  data[0] = static_cast<uint8_t>((control ? 0x01 : 0x00) | (shoot ? 0x02 : 0x00));
  data[1] = 0;

  const int16_t dyaw = control ? io::quantize_angle_1e4(command.yaw) : 0;
  const int16_t dpitch = control ? io::quantize_angle_1e4(command.pitch) : 0;
  data[2] = static_cast<uint8_t>((dyaw >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(dyaw & 0xFF);
  data[4] = static_cast<uint8_t>((dpitch >> 8) & 0xFF);
  data[5] = static_cast<uint8_t>(dpitch & 0xFF);
  data[6] = 0;
  data[7] = 0;
}

bool CBoard::send(Command command) const
{
  can_frame frame;
  frame.can_dlc = 8;

  // 相对角差模式：只发 Δyaw/Δpitch，不发绝对角（见 documentation/relative_can_protocol.md）
  if (command_mode_ == CommandMode::relative) {
    frame.can_id = static_cast<canid_t>(relative_canid_);
    encode_relative_command(command, frame.data);

    try {
      can_.write(&frame);
      return true;
    } catch (const std::exception & e) {
      if (should_log_failure()) tools::logger()->warn("[Cboard] 相对角差帧发送失败：{}", e.what());
      return false;
    }
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
    return true;
  } catch (const std::exception & e) {
    if (should_log_failure()) tools::logger()->warn("[Cboard] 绝对角帧发送失败：{}", e.what());
    return false;
  }
}

bool CBoard::send_raw(uint32_t can_id, const uint8_t data[8]) const
{
  if (can_id > CAN_SFF_MASK || data == nullptr) return false;
  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = static_cast<canid_t>(can_id);
  frame.can_dlc = 8;
  std::memcpy(frame.data, data, 8);

  try {
    can_.write(&frame);
    return true;
  } catch (const std::exception & e) {
    if (should_log_failure())
      tools::logger()->warn("[Cboard] 原始帧 0x{:X} 发送失败：{}", can_id, e.what());
    return false;
  }
}

CBoard::FrameHandlerId CBoard::add_frame_handler(FrameHandler handler)
{
  if (!handler) return 0;
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  const FrameHandlerId id = next_handler_id_++;
  handlers_.push_back({id, std::move(handler)});
  return id;
}

void CBoard::remove_frame_handler(FrameHandlerId handler_id)
{
  if (handler_id == 0) return;
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  const auto it = std::find_if(
    handlers_.begin(), handlers_.end(),
    [handler_id](const HandlerEntry & entry) { return entry.id == handler_id; });
  if (it != handlers_.end()) handlers_.erase(it);
}

void CBoard::callback(const can_frame & frame)
{
  if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) return;
  auto timestamp = std::chrono::steady_clock::now();

  // 先给 v1 协议的观察者（vision_cboard）看一遍原始帧，再走旧的解析分支：
  // 旧分支不认识 0x1xx 之外的 ID，顺序不影响已有行为。
  {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    for (const auto & entry : handlers_) entry.handler(frame.can_id, frame.data, frame.can_dlc);
  }

  // 协议观察者需要看畸形 DLC 以 fail-closed；旧遥测解析则不能读缺失字节。
  if (frame.can_dlc != 8) return;

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

  // 相对角差帧的 ID 校验（只校验标准帧 11 位范围）：写错成 0x800+ 会变成扩展帧标志位，
  // 或者与四元数/弹速/绝对角命令撞车 —— 两者都会让电控侧解析出错，启动时就拒绝。
  if (command_mode_ == CommandMode::relative) {
    if (relative_canid_ < 0 || relative_canid_ > 0x7FF) {
      throw std::runtime_error(
        "relative_canid 必须在 0..0x7FF（标准帧 11 位）之间，当前值 = " +
        std::to_string(relative_canid_));
    }
    if (
      relative_canid_ == quaternion_canid_ || relative_canid_ == bullet_speed_canid_ ||
      relative_canid_ == send_canid_) {
      throw std::runtime_error(
        "relative_canid 与 quaternion_canid / bullet_speed_canid / send_canid 冲突（当前 " +
        std::to_string(relative_canid_) + "），换一个未占用的 ID");
    }

    tools::logger()->info(
      "[Cboard] 相对角差模式：0x{:X} 上发 Δyaw/Δpitch（int16 BE ×1e4 rad），"
      "control=0 时角差与 shoot 一律写 0",
      relative_canid_);
  }

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }

  can_interface_ = yaml["can_interface"].as<std::string>();
  return can_interface_;
}

}  // namespace io
