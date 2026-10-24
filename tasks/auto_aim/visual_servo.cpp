#include "visual_servo.hpp"

#include <cmath>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace auto_aim
{
ServoLimits load_servo_limits(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  // 全部是新增可选键：缺键时保持默认值，已有 configs/*.yaml 不用改
  ServoLimits limits;
  limits.gain = tools::optional_double(yaml, "servo_gain", 1.0);
  limits.deadband = tools::optional_double(yaml, "servo_deadband_deg", 0.1) / 57.3;
  limits.max_step = tools::optional_double(yaml, "servo_max_step_deg", 20.0) / 57.3;
  limits.yaw_sign = tools::optional_double(yaml, "servo_yaw_sign", 1.0);
  limits.pitch_sign = tools::optional_double(yaml, "servo_pitch_sign", 1.0);
  limits.yaw_offset = tools::optional_double(yaml, "servo_yaw_offset_deg", 0.0) / 57.3;
  limits.pitch_offset = tools::optional_double(yaml, "servo_pitch_offset_deg", 0.0) / 57.3;
  return limits;
}

VisualServo::VisualServo(const std::string & config_path) : limits_(load_servo_limits(config_path))
{
  tools::logger()->info(
    "[VisualServo] gain={:.2f} deadband={:.2f}deg max_step={:.2f}deg yaw_sign={:+.0f} "
    "pitch_sign={:+.0f}",
    limits_.gain, limits_.deadband * 57.3, limits_.max_step * 57.3, limits_.yaw_sign,
    limits_.pitch_sign);
}

void VisualServo::angles(const Eigen::Vector3d & xyz_in_gimbal, double & yaw, double & pitch)
{
  const double x = xyz_in_gimbal[0];
  const double y = xyz_in_gimbal[1];
  const double z = xyz_in_gimbal[2];

  yaw = std::atan2(y, x);
  // 与 Aimer 一致（aimer.cpp：pitch = -(trajectory.pitch)，抬头为正）：
  // CBoard 的 pitch 约定是「向上为负」
  pitch = -std::atan2(z, std::sqrt(x * x + y * y));
}

io::Command VisualServo::shape(const ServoLimits & limits, double yaw, double pitch)
{
  yaw += limits.yaw_offset;
  pitch += limits.pitch_offset;

  if (std::abs(yaw) < limits.deadband) yaw = 0;
  if (std::abs(pitch) < limits.deadband) pitch = 0;

  yaw = tools::limit_min_max(yaw, -limits.max_step, limits.max_step);
  pitch = tools::limit_min_max(pitch, -limits.max_step, limits.max_step);

  io::Command command{false, false, 0, 0};
  command.yaw = limits.gain * limits.yaw_sign * yaw;
  command.pitch = limits.gain * limits.pitch_sign * pitch;
  command.control = true;
  command.shoot = false;  // 相对角差模式下射击由电控/后续逻辑决定，这里不置位
  return command;
}

io::Command VisualServo::aim(const std::list<Armor> & armors, const std::list<Target> & targets)
{
  if (targets.empty()) {
    has_last_target_ = false;
    return {false, false, 0, 0};
  }

  const auto & target = targets.front();

  // 换目标后的第一帧不发（原因见 visual_servo.hpp 里 has_last_target_ 的注释）
  if (!has_last_target_ || target.name != last_name_ || target.armor_type != last_type_) {
    has_last_target_ = true;
    last_name_ = target.name;
    last_type_ = target.armor_type;
    return {false, false, 0, 0};
  }

  // 同一目标可能有多块同型号装甲板：选最靠近画面中心的那块（与 Tracker 初选规则一致）
  const Armor * nearest = nullptr;
  double nearest_distance = 1e10;
  for (const auto & armor : armors) {
    if (armor.name != target.name || armor.type != target.armor_type) continue;
    const double distance =
      std::hypot(armor.center_norm.x - 0.5f, armor.center_norm.y - 0.5f);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = &armor;
    }
  }

  if (nearest == nullptr) return {false, false, 0, 0};

  // 目标在相机背后（x <= 0）或数据非法时角度没有意义，直接不发
  if (!nearest->xyz_in_gimbal.allFinite() || nearest->xyz_in_gimbal[0] <= 0) {
    return {false, false, 0, 0};
  }

  double yaw, pitch;
  angles(nearest->xyz_in_gimbal, yaw, pitch);
  return shape(limits_, yaw, pitch);
}

}  // namespace auto_aim
