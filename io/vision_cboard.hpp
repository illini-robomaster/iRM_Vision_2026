#ifndef IO__VISION_CBOARD_HPP
#define IO__VISION_CBOARD_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "io/command.hpp"

namespace io
{
class CBoard;

// ===========================================================================
// vision ↔ CBoard 应用层协议 v1（握手 / 会话 / 启停 / 颜色）
//
// 完整字节表与电控侧参考实现见 documentation/vision_cboard_protocol.md。
// 本文件只做两件事：
//   1) 纯编解码（无 CAN、无时间、无锁）：encode_* / decode_*
//   2) 可注入时间的会话状态机 VisionLinkCore（可在单测里直接驱动）
// 线程模型：VisionLinkCore 内部一把可重入互斥锁；生产代码用 transmit()
// 串行化选择/写入/记账。VisionCboardLink 的 CAN 回调仅入有界队列，
// 独立线程处理队列和发送，视觉线程只 publish/snapshot。
//
// 约束（与 documentation 一致，改代码时同步改文档）：
//   - Classical CAN、标准 11-bit 帧、DLC = 8；不用 FD / 扩展帧 / RTR；
//   - HELLO 由 CBoard 发起（视觉侧只做应答）；HELLO 里带 session_id；
//   - 视觉侧只接受**当前会话**的控制帧：旧 session、畸形帧、越界颜色一律拒绝，
//     并且这些帧**不刷新**心跳（不能靠发垃圾帧续命）；
//   - 同 session 的重复 HELLO 幂等：重发同一份 READY，不复位启停/颜色/generation；
//   - 控制帧超时 → 立刻失效会话并回到 control_timeout，必须重新握手才能再发角差。
// ===========================================================================

constexpr uint8_t kVisionProtocolVersion = 1;

// 消息类型字节（data[0]）
constexpr uint8_t kVisionMsgHandshakeRequest = 0x01;   // CBoard → 视觉
constexpr uint8_t kVisionMsgHandshakeResponse = 0x02;  // 视觉 → CBoard
constexpr uint8_t kVisionMsgControl = 0x03;            // CBoard → 视觉
// 相对角差帧没有类型字节（沿用 8 字节角差布局，见 io/command.hpp）：
// 它的「类型」就是 CAN ID 本身。

// HELLO_ACK 的 status（data[6]）
enum class HandshakeStatus : uint8_t
{
  ready = 0,
  not_ready = 1,
  version_mismatch = 2
};
std::string to_string(HandshakeStatus status);

// 控制帧里允许的颜色（协议值，不是 auto_aim::Color 的枚举值）
//   1 = red，2 = blue。0 / 3..255 一律视为畸形控制帧。
enum class ControlColor : uint8_t
{
  red = 1,
  blue = 2
};
bool valid_control_color(uint8_t value);
std::string to_string(ControlColor color);

// ---------------- 编解码（纯函数，便于离线测试）----------------

// HELLO：data[0]=0x01, data[1]=version, data[2..5]=session_id(大端), data[6..7]=0
void encode_handshake_request(uint32_t session_id, uint8_t data[8]);

// HELLO_ACK：data[0]=0x02, data[1]=version, data[2..5]=回显 session_id, data[6]=status, data[7]=0
void encode_handshake_response(uint32_t session_id, HandshakeStatus status, uint8_t data[8]);

// CONTROL：data[0]=0x03, data[1]=version, data[2..5]=session_id, data[6]=enable|color<<1,
//          data[7]=sequence
void encode_control(
  uint32_t session_id, bool enable, ControlColor color, uint8_t sequence, uint8_t data[8]);

// 结构合法（DLC==8 且类型字节/版本字节对得上）才返回 true；version 单独返回，
// 便于对「版本不兼容」回 VERSION_MISMATCH 而不是当成畸形帧丢弃。
bool decode_handshake_request(
  const uint8_t * data, uint8_t dlc, uint8_t & version, uint32_t & session_id);
bool decode_control(
  const uint8_t * data, uint8_t dlc, uint32_t & session_id, bool & enable, uint8_t & color,
  uint8_t & sequence);

// ---------------- 配置 ----------------

// relative_aim 专用新增键；实发必需键显式校验，旧入口不加载此配置：
//   vision_protocol              "v1"（默认）| "legacy"（必须显式选择）
//   relative_output_frame        坐标系 camera_optical | gimbal（不参与协议选择）
//   relative_canid               v1 下视觉 → CBoard 的角差帧 ID
//   vision_handshake_request_canid   CBoard → 视觉（HELLO）
//   vision_handshake_response_canid  视觉 → CBoard（HELLO_ACK）
//   vision_control_canid         CBoard → 视觉（CONTROL）
//   vision_control_timeout_ms    控制帧超时（默认 500）
//   relative_frame_timeout_ms    观测时效：超过就补一帧 control=0（默认 100）
//   relative_tx_rate_hz          角差帧发送上限（默认 0 = 不限，改用
//                                relative_send_interval_ms；两者都 0 = 有新观测就发）
//   vision_can_ids_confirmed     非核实 vcan 接口必须显式 true（legacy 同样适用）
struct VisionProtocolConfig
{
  bool v1 = false;
  uint32_t relative_canid = 0xFE;
  uint32_t handshake_request_canid = 0;
  uint32_t handshake_response_canid = 0;
  uint32_t control_canid = 0;
  double control_timeout_ms = 500.0;
  double observation_timeout_ms = 100.0;
  double tx_min_interval_ms = 0.0;
  bool ids_confirmed = false;
  std::string can_interface;
};

// 读 yaml 并做全部校验；非法配置抛 std::runtime_error（不静默退化）
VisionProtocolConfig load_vision_protocol_config(const std::string & config_path);

// 通过 rtnetlink 的 IFLA_INFO_KIND 精确核实 vcan；其它/未知接口不能豁免 ID 确认。
bool can_interface_is_virtual(const std::string & interface, bool & known);

// 会话状态（写进日志 / CSV，别改名：文档和 CSV 列都引用它）
enum class VisionLinkState
{
  wait_handshake,
  wait_control,
  active,
  stopped,
  control_timeout
};
std::string to_string(VisionLinkState state);

struct VisionLinkSnapshot
{
  VisionLinkState state = VisionLinkState::wait_handshake;
  bool has_session = false;
  uint32_t session_id = 0;
  bool enable = false;
  uint8_t color = 0;  // 协议颜色：0 = 未指定，1 = red，2 = blue
  uint8_t sequence = 0;
  uint32_t generation = 0;  // 每次「会话 / 使能 / 颜色」变化 +1
  double control_age_ms = 0.0;
  bool ready = false;
};

struct VisionLinkStats
{
  uint64_t rx_handshake = 0;
  uint64_t rx_control = 0;
  uint64_t rejected_version = 0;
  uint64_t rejected_session = 0;
  uint64_t rejected_malformed = 0;
  uint64_t rejected_color = 0;
  uint64_t tx_handshake_response = 0;
  uint64_t tx_relative = 0;
  uint64_t tx_stop = 0;
  uint64_t tx_fail = 0;
  uint64_t tx_skipped_rate = 0;
  uint64_t tx_skipped_empty = 0;
  uint64_t tx_skipped_stale = 0;
  uint64_t tx_skipped_generation = 0;
  uint64_t timeouts = 0;
};

// ---------------- 会话状态机 ----------------

class VisionLinkCore
{
public:
  enum class FrameKind
  {
    none,
    handshake_response,
    relative
  };

  explicit VisionLinkCore(const VisionProtocolConfig & config);

  const VisionProtocolConfig & config() const { return config_; }

  // READY 门限：模型 + 几何配置 + 首张合格图像都就绪后才置 true。
  // false 时 HELLO 一律回 NOT_READY，且不会建立会话。
  void set_ready(bool ready, const std::string & reason);

  // 队列消费线程调用（任意 ID，内部只认配置里的 HELLO / CONTROL）
  void on_frame(
    uint32_t can_id, const uint8_t * data, uint8_t dlc, std::chrono::steady_clock::time_point now);

  // 发送线程周期调用：只做超时判定（接收静默时也能失效会话）
  void poll(std::chrono::steady_clock::time_point now);

  // 视觉线程发布「本帧观测」。generation 必须来自取图/检测之前的 snapshot()：
  // now 参数为图像采集时间，不能用检测结束时间。
  // 期间若收到新 CONTROL，publish 的样本会被丢弃（不让变色前的图像生效）。
  bool publish(
    const Command & command, uint32_t generation, std::chrono::steady_clock::time_point now);

  // 发送线程取下一帧。返回 false = 这一轮不发（不是错误）。
  bool next_frame(
    std::chrono::steady_clock::time_point now, uint32_t & can_id, uint8_t data[8],
    FrameKind & kind);

  void note_send(bool ok, FrameKind kind);

  // 生产发送入口：选择、资格检查、写入与发送结果在同一事务内完成。
  bool transmit(
    std::chrono::steady_clock::time_point now,
    const std::function<bool(uint32_t, const uint8_t *)> & sender);

  // 关闭前请求补一帧 control=0（best effort）
  void request_stop_frame();

  VisionLinkSnapshot snapshot(std::chrono::steady_clock::time_point now) const;
  VisionLinkStats stats() const;

private:
  struct Sample
  {
    bool has = false;
    bool valid = false;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t generation = 0;
    bool sent = false;
    std::chrono::steady_clock::time_point time;
  };

  void poll_locked(std::chrono::steady_clock::time_point now);
  void transition_locked(VisionLinkState state, const std::string & reason);
  void expire_session_locked(const std::string & reason);
  void queue_handshake_response_locked(uint32_t session_id, HandshakeStatus status);

  VisionProtocolConfig config_;

  mutable std::recursive_mutex mutex_;
  VisionLinkState state_ = VisionLinkState::wait_handshake;
  bool ready_ = false;
  bool has_session_ = false;
  bool has_sequence_ = false;
  uint32_t session_id_ = 0;
  bool enable_ = false;
  uint8_t color_ = 0;
  uint8_t sequence_ = 0;
  uint32_t generation_ = 0;
  std::chrono::steady_clock::time_point generation_since_{};
  std::chrono::steady_clock::time_point last_control_time_{};

  bool has_pending_response_ = false;
  uint8_t pending_response_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t response_serial_ = 0;
  uint32_t sent_response_serial_ = 0;

  Sample sample_;
  bool pending_stop_ = false;
  bool stop_sent_for_sample_ = false;
  bool has_sent_valid_ = false;
  bool last_frame_was_stop_ = false;
  std::chrono::steady_clock::time_point last_tx_time_{};
  bool has_tx_time_ = false;

  VisionLinkStats stats_;
};

// ---------------- 挂到 CBoard 上的实现（CAN + 独立发送线程）----------------
//
// 发送节拍与 camera.read() / 检测完全解耦：取流阻塞、检测变慢、程序收到停止命令时，
// control=0 / 超时停止都不会等下一帧图像。
class VisionCboardLink
{
public:
  VisionCboardLink(const VisionProtocolConfig & config, CBoard & board);
  ~VisionCboardLink();

  VisionCboardLink(const VisionCboardLink &) = delete;
  VisionCboardLink & operator=(const VisionCboardLink &) = delete;

  void start();
  // 请求补发 control=0 并等发送线程收尾（必须在析构 CBoard 之前调用）
  void stop();

  void set_ready(bool ready, const std::string & reason) { core_.set_ready(ready, reason); }
  bool publish(
    const Command & command, uint32_t generation, std::chrono::steady_clock::time_point now)
  {
    return core_.publish(command, generation, now);
  }
  VisionLinkSnapshot snapshot() const { return core_.snapshot(std::chrono::steady_clock::now()); }
  VisionLinkStats stats() const { return core_.stats(); }
  const VisionProtocolConfig & config() const { return core_.config(); }

  // CAN 接收线程回调（由 CBoard::add_frame_handler 转发进来）：只复制/入有界队列
  void on_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc);

  // 一行汇总（退出时打印）
  std::string stats_text() const;

private:
  void run();

  CBoard & board_;
  uint64_t handler_id_ = 0;  // CBoard::FrameHandlerId; CBoard is forward-declared here.
  VisionLinkCore core_;
  struct ReceivedFrame
  {
    uint32_t id;
    uint8_t dlc;
    std::array<uint8_t, 8> data;
    std::chrono::steady_clock::time_point time;
  };
  std::mutex receive_mutex_;
  std::deque<ReceivedFrame> received_;
  bool receive_overflow_ = false;
  std::thread thread_;
  std::atomic<bool> quit_{false};
  std::atomic<bool> running_{false};
};

}  // namespace io

#endif  // IO__VISION_CBOARD_HPP
