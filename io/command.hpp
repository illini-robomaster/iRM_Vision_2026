#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

#include <cmath>
#include <cstdint>

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
};

// 角度量化：rad → int16（单位 1e-4 rad），与固件解码宽度一致。
//   - 饱和而不是回绕：超出 int16 范围截断到 +32767 / -32768（≈ ±3.2767 rad），
//     避免 int16 溢出把「往右转」变成「往左转」；
//   - NaN / Inf 返回 0（调用方必须同时清 control，见 encode_relative_command），
//     不能让非有限值走进 lround（那是未定义行为）。
// 单一实现：io::encode_relative_command() 与 auto_aim::CurrentRelativeEstimator 的 TX 值
// 都用它，保证 CSV 里记录的 TX 角差与总线上真正发出去的字节完全一致。
inline int16_t quantize_angle_1e4(double rad)
{
  if (!std::isfinite(rad)) return 0;
  const double scaled = rad * 1e4;
  if (scaled >= 32767.0) return 32767;
  if (scaled <= -32768.0) return -32768;
  return static_cast<int16_t>(std::lround(scaled));
}

}  // namespace io

#endif  // IO__COMMAND_HPP