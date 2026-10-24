#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
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
//
// 安全性（编码层强制，不依赖调用方）：
//   - control=0 时角差与 shoot 一律写 0：不让「无目标」的旧角度值留在帧里被电控累加；
//   - yaw/pitch 出现 NaN / Inf 时视为无效观测：control 清零、角差写 0；
//   - 超范围饱和到 int16 边界（不回绕），见 io::quantize_angle_1e4()。
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

  // 返回值 = 这一帧是否**完整写进了 SocketCAN**。
  // 旧调用方忽略返回值即可（签名从 void 改成 bool 不影响已有代码），
  // 但 relative_aim 这类需要「发送结果观测」的入口必须检查它：
  // 打不开 CAN / 帧被拒绝 / 写了一半都不算发送成功。
  bool send(Command command) const;

  // can_command_mode: "relative" → send() 按相对角差编码（见 encode_relative_command）
  bool relative() const;

  // 相对角差帧的 CAN ID（can_command_mode: relative 时才有意义）
  int relative_canid() const;

  // 实际 SocketCAN 接口；共享此 CBoard 的协议用来核对配置没有指向另一总线。
  const std::string & interface() const { return can_interface_; }

  // 原始 8 字节帧发送（vision↔CBoard v1 协议用：HELLO_ACK / 角差帧都要指定 ID）。
  // 返回「整帧是否完整写进 SocketCAN」；失败已按 1Hz 限流打印 errno。
  bool send_raw(uint32_t can_id, const uint8_t data[8]) const;

  // 追加一个「所有上行帧」的观察者。回调运行在 SocketCAN 接收线程里：
  // 只做解析/入队，不要在里面发帧或做耗时操作（否则会拖慢全部接收）。
  using FrameHandler = std::function<void(uint32_t can_id, const uint8_t * data, uint8_t dlc)>;
  using FrameHandlerId = uint64_t;
  // 返回非零订阅 ID；调用方销毁捕获对象前必须用这个 ID 注销。多个协议
  // 可以共享一个 SocketCAN，不再需要用 clear_frame_handlers() 误删彼此。
  FrameHandlerId add_frame_handler(FrameHandler handler);
  void remove_frame_handler(FrameHandlerId handler_id);
  // 仅保留给需要清空全部订阅的既有调用方；共享协议请用 remove_frame_handler(id)。
  [[deprecated("clearing every handler can disconnect unrelated CAN links")]] void
  clear_frame_handlers();

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

  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  // 上行帧观察者：callback 在 SocketCAN 接收线程里遍历，所以要加锁。
  struct HandlerEntry
  {
    FrameHandlerId id;
    FrameHandler handler;
  };
  mutable std::mutex handlers_mutex_;
  std::vector<HandlerEntry> handlers_;
  FrameHandlerId next_handler_id_ = 1;
  std::string can_interface_;
  // 最后构造、最先析构：接收线程不能访问尚未构造/已经析构的成员。
  SocketCAN can_;

  void callback(const can_frame & frame);

  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP