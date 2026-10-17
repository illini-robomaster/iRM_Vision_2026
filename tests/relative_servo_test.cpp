// ---------------------------------------------------------------------------
// 相对角差（视觉伺服）离线自检：不需要相机 / CAN / 电机
//   1) VisualServo::angles：云台系坐标 → Δyaw / Δpitch（符号、坐标系）
//   2) VisualServo::shape ：死区 / 限幅 / 增益 / 符号 / 偏置
//   3) encode_relative_command：CAN 帧字节布局（int16 大端 ×1e4 rad、饱和）
//
// 用法：./build/relative_servo_test        # 全部通过返回 0，有 FAIL 返回 1
// ---------------------------------------------------------------------------
#include <fmt/core.h>

#include <cmath>
#include <cstdint>
#include <string>

#include "io/cboard.hpp"
#include "tasks/auto_aim/visual_servo.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

int checks = 0;
int failures = 0;

void expect(bool ok, const std::string & name)
{
  ++checks;
  if (ok) {
    fmt::print("ok    {}\n", name);
  } else {
    ++failures;
    fmt::print("FAIL  {}\n", name);
  }
}

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

// 与固件侧一致的解码方式（大端、有符号）
int decode_i16(const uint8_t * p)
{
  const int raw = (p[0] << 8) | p[1];
  return raw >= 0x8000 ? raw - 0x10000 : raw;
}

void test_angles()
{
  using auto_aim::VisualServo;
  double yaw = 1, pitch = 1;

  VisualServo::angles({1, 0, 0}, yaw, pitch);
  expect(near(yaw, 0) && near(pitch, 0), "angles: 目标在正前方 -> (0, 0)");

  VisualServo::angles({0, 1, 0}, yaw, pitch);
  expect(near(yaw, kPi / 2) && near(pitch, 0), "angles: 目标在正左方 -> dYaw=+90deg");

  VisualServo::angles({0, -1, 0}, yaw, pitch);
  expect(near(yaw, -kPi / 2), "angles: 目标在正右方 -> dYaw=-90deg");

  VisualServo::angles({1, 0, 1}, yaw, pitch);
  expect(near(pitch, -kPi / 4), "angles: 目标在上方 -> dPitch 为负（向上为负）");

  VisualServo::angles({1, 0, -1}, yaw, pitch);
  expect(near(pitch, kPi / 4), "angles: 目标在下方 -> dPitch 为正");
}

void test_shape()
{
  using auto_aim::ServoLimits;
  using auto_aim::VisualServo;

  ServoLimits limits;
  limits.gain = 1.0;
  limits.deadband = 0.1 * kDeg;
  limits.max_step = 20.0 * kDeg;

  auto inside = VisualServo::shape(limits, 0.05 * kDeg, -0.05 * kDeg);
  expect(
    inside.control && near(inside.yaw, 0) && near(inside.pitch, 0), "shape: 死区内不发角差");

  auto outside = VisualServo::shape(limits, 0.2 * kDeg, -0.2 * kDeg);
  expect(
    outside.control && near(outside.yaw, 0.2 * kDeg) && near(outside.pitch, -0.2 * kDeg),
    "shape: 死区外原样下发（gain=1, sign=+1）");

  auto clamped = VisualServo::shape(limits, 100 * kDeg, -100 * kDeg);
  expect(
    near(clamped.yaw, limits.max_step) && near(clamped.pitch, -limits.max_step),
    "shape: 单帧角差超过 max_step 时饱和到 +/-20deg");

  ServoLimits signd;
  signd.gain = 0.5;
  signd.yaw_sign = -1;
  signd.pitch_sign = -1;
  auto flipped = VisualServo::shape(signd, 2 * kDeg, 4 * kDeg);
  expect(
    near(flipped.yaw, -1 * kDeg) && near(flipped.pitch, -2 * kDeg),
    "shape: gain=0.5 且 sign=-1 -> 角度减半并反向");

  ServoLimits offset;
  offset.yaw_offset = 1 * kDeg;
  offset.pitch_offset = -1 * kDeg;
  auto biased = VisualServo::shape(offset, 0, 0);
  expect(
    near(biased.yaw, 1 * kDeg) && near(biased.pitch, -1 * kDeg),
    "shape: 静态偏置总是叠加（servo_*_offset_deg）");

  auto combined = VisualServo::shape(limits, -30 * kDeg, 30 * kDeg);
  expect(
    near(combined.yaw, -limits.max_step) && near(combined.pitch, limits.max_step),
    "shape: 负方向同样饱和");
}

void test_encode()
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  io::Command command{true, true, 0.1234, -0.0567, 0.0};
  io::encode_relative_command(command, data);
  expect(data[0] == 0x03, "encode: data[0] bit0=control bit1=shoot");
  expect(data[1] == 0 && data[6] == 0 && data[7] == 0, "encode: 保留字节为 0");
  expect(decode_i16(data + 2) == 1234, "encode: dYaw 大端 int16，单位 1e-4 rad");
  expect(decode_i16(data + 4) == -567, "encode: dPitch 大端 int16，负值");

  io::Command idle{false, false, 0, 0, 0};
  io::encode_relative_command(idle, data);
  expect(
    data[0] == 0x00 && decode_i16(data + 2) == 0 && decode_i16(data + 4) == 0,
    "encode: 无目标（control=0）时角差为 0，不会跳变");

  io::Command saturated{true, false, 10.0, -10.0, 0.0};
  io::encode_relative_command(saturated, data);
  expect(
    decode_i16(data + 2) == 32767 && decode_i16(data + 4) == -32768,
    "encode: 超出 int16 范围时饱和（不回绕）");

  io::Command roundtrip{true, false, -0.3, 0.3, 0.0};
  io::encode_relative_command(roundtrip, data);
  expect(
    near(decode_i16(data + 2) / 1e4, -0.3) && near(decode_i16(data + 4) / 1e4, 0.3),
    "encode: 编解码往返误差 <= 1e-4 rad");
}

void test_pipeline()
{
  using auto_aim::ServoLimits;
  using auto_aim::VisualServo;

  ServoLimits limits;
  limits.gain = 1.0;
  limits.deadband = 0.1 * kDeg;
  limits.max_step = 20.0 * kDeg;

  // 目标在前方 2m、右侧 0.3m（云台系 +y 向左）、上方 0.4m，角度都在 max_step 之内
  double yaw = 0, pitch = 0;
  VisualServo::angles({2, -0.3, 0.4}, yaw, pitch);
  auto command = VisualServo::shape(limits, yaw, pitch);

  expect(
    command.control && command.yaw < 0 && command.pitch < 0,
    "pipeline: 右前上方目标 -> dYaw<0（右转）且 dPitch<0（向上为负）");

  expect(
    near(command.yaw, std::atan2(-0.3, 2.0)) &&
      near(command.pitch, -std::atan2(0.4, std::sqrt(2.0 * 2.0 + 0.3 * 0.3))),
    "pipeline: 幅度与 atan2 解析解一致");

  // 更偏右（yaw≈-26.6deg）时必须被 max_step 截断，防止首帧猛甩
  VisualServo::angles({2, -1, 0.4}, yaw, pitch);
  expect(
    near(VisualServo::shape(limits, yaw, pitch).yaw, -limits.max_step),
    "pipeline: 超出 max_step 的 dYaw 被截断");

  // 极近且几乎在正侧方（≈90deg）：必须被 max_step 截断，防止首帧猛甩
  VisualServo::angles({0.001, 1, 0}, yaw, pitch);
  auto extreme = VisualServo::shape(limits, yaw, pitch);
  expect(near(extreme.yaw, limits.max_step), "pipeline: 极端角差被 max_step 截断");
}

}  // namespace

int main()
{
  test_angles();
  test_shape();
  test_encode();
  test_pipeline();

  fmt::print("\n{} checks, {} failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
