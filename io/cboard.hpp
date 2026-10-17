#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

// 上行命令语义（新增，缺键时默认 absolute，已有 configs 行为完全不变）
//   absolute：在 send_canid 上发「绝对」yaw/pitch（旧协议，需要电控有姿态反馈）
//   relative：在 relative_canid 上发「相对角差」Δyaw/Δpitch（视觉伺服，不需要电机位姿）
// 帧格式见 documentation/relative_can_protocol.md
enum class CommandMode
{
  absolute,
  relative
};

// 相对角差帧编码（纯函数，便于离线测试，见 tests/relative_servo_test.cpp）
//   data[0] bit0 = control, bit1 = shoot
//   data[1] = 0（保留）
//   data[2..3] = Δyaw   int16 大端，单位 1e-4 rad
//   data[4..5] = Δpitch int16 大端，单位 1e-4 rad
//   data[6..7] = 0（保留）
void encode_relative_command(const Command & command, uint8_t data[8]);

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

  // can_command_mode: "relative" → send() 按相对角差编码（见 encode_relative_command）
  bool relative() const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能

  // 新增选项：声明在 can_ 之前，保证 read_yaml()（在 can_ 构造期间执行）能安全写入。
  // 缺键时取下面的默认值，已有 configs 的行为完全不变。
  CommandMode command_mode_ = CommandMode::absolute;
  int relative_canid_ = 0xFE;
  bool wait_imu_ = true;  // false：不阻塞等四元数，imu_at() 恒返回单位四元数

  SocketCAN can_;
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  void callback(const can_frame & frame);

  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP