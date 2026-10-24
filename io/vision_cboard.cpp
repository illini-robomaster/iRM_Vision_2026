#include "vision_cboard.hpp"

#include <fmt/format.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include "io/cboard.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
using Clock = std::chrono::steady_clock;

// 两个时间点之间的毫秒数（负数按 0 处理：steady_clock 不应倒退，但别让日志出现 -0.0）
double age_ms(Clock::time_point now, Clock::time_point then)
{
  const double ms =
    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - then).count();
  return ms > 0.0 ? ms : 0.0;
}

uint32_t read_u32(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

void write_u32(uint32_t value, uint8_t * p)
{
  p[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  p[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  p[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  p[3] = static_cast<uint8_t>(value & 0xFF);
}
}  // namespace

std::string to_string(HandshakeStatus status)
{
  switch (status) {
    case HandshakeStatus::ready:
      return "READY";
    case HandshakeStatus::not_ready:
      return "NOT_READY";
    case HandshakeStatus::version_mismatch:
      return "VERSION_MISMATCH";
  }
  return "UNKNOWN";
}

bool valid_control_color(uint8_t value)
{
  return value == static_cast<uint8_t>(ControlColor::red) ||
         value == static_cast<uint8_t>(ControlColor::blue);
}

std::string to_string(ControlColor color) { return color == ControlColor::red ? "red" : "blue"; }

std::string to_string(VisionLinkState state)
{
  switch (state) {
    case VisionLinkState::wait_handshake:
      return "wait_handshake";
    case VisionLinkState::wait_control:
      return "wait_control";
    case VisionLinkState::active:
      return "active";
    case VisionLinkState::stopped:
      return "stopped";
    case VisionLinkState::control_timeout:
      return "control_timeout";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// 编解码：一张表对应 documentation/vision_cboard_protocol.md 的第 2 节，
// 改任一侧时必须同时改文档（电控照文档实现，不照这份源码）。
// ---------------------------------------------------------------------------
void encode_handshake_request(uint32_t session_id, uint8_t data[8])
{
  data[0] = kVisionMsgHandshakeRequest;
  data[1] = kVisionProtocolVersion;
  write_u32(session_id, data + 2);
  data[6] = 0;
  data[7] = 0;
}

void encode_handshake_response(uint32_t session_id, HandshakeStatus status, uint8_t data[8])
{
  data[0] = kVisionMsgHandshakeResponse;
  data[1] = kVisionProtocolVersion;
  write_u32(session_id, data + 2);
  data[6] = static_cast<uint8_t>(status);
  data[7] = 0;
}

void encode_control(
  uint32_t session_id, bool enable, ControlColor color, uint8_t sequence, uint8_t data[8])
{
  data[0] = kVisionMsgControl;
  data[1] = kVisionProtocolVersion;
  write_u32(session_id, data + 2);
  const uint8_t flags =
    static_cast<uint8_t>((enable ? 0x01 : 0x00) | ((static_cast<uint8_t>(color) << 1) & 0x0E));
  data[6] = flags;
  data[7] = sequence;
}

bool decode_handshake_request(
  const uint8_t * data, uint8_t dlc, uint8_t & version, uint32_t & session_id)
{
  if (data == nullptr || dlc != 8) return false;
  if (data[0] != kVisionMsgHandshakeRequest || data[6] != 0 || data[7] != 0) return false;
  version = data[1];
  session_id = read_u32(data + 2);
  return true;
}

bool decode_control(
  const uint8_t * data, uint8_t dlc, uint32_t & session_id, bool & enable, uint8_t & color,
  uint8_t & sequence)
{
  if (data == nullptr || dlc != 8) return false;
  if (data[0] != kVisionMsgControl) return false;
  if (data[1] != kVisionProtocolVersion) return false;
  // data[6] 高 4 位保留必须为 0：非 0 说明对端在用一个我们还没实现的扩展
  if ((data[6] & 0xF0) != 0) return false;
  session_id = read_u32(data + 2);
  enable = (data[6] & 0x01) != 0;
  color = static_cast<uint8_t>((data[6] >> 1) & 0x07);
  sequence = data[7];
  return true;
}

bool can_interface_is_virtual(const std::string & interface, bool & known)
{
  known = false;
  const unsigned index = if_nametoindex(interface.c_str());
  if (index == 0) return false;
  const int fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
  if (fd < 0) return false;
  timeval timeout{0, 100000};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  struct Request
  {
    nlmsghdr header;
    ifinfomsg info;
  } request{};
  request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
  request.header.nlmsg_type = RTM_GETLINK;
  request.header.nlmsg_flags = NLM_F_REQUEST;
  request.info.ifi_family = AF_UNSPEC;
  request.info.ifi_index = static_cast<int>(index);
  sockaddr_nl address{};
  address.nl_family = AF_NETLINK;
  const auto sent = sendto(
    fd, &request, request.header.nlmsg_len, 0, reinterpret_cast<sockaddr *>(&address),
    sizeof(address));
  alignas(nlmsghdr) char buffer[8192];
  int length = sent > 0 ? static_cast<int>(recv(fd, buffer, sizeof(buffer), 0)) : -1;
  ::close(fd);
  for (auto * h = reinterpret_cast<nlmsghdr *>(buffer); NLMSG_OK(h, length);
       h = NLMSG_NEXT(h, length)) {
    if (h->nlmsg_type != RTM_NEWLINK) continue;
    auto * info = reinterpret_cast<ifinfomsg *>(NLMSG_DATA(h));
    if (info->ifi_index != static_cast<int>(index)) continue;
    known = true;
    int size = IFLA_PAYLOAD(h);
    for (auto * attr = IFLA_RTA(info); RTA_OK(attr, size); attr = RTA_NEXT(attr, size)) {
      if (attr->rta_type != IFLA_LINKINFO) continue;
      int nested_size = RTA_PAYLOAD(attr);
      for (auto * nested = reinterpret_cast<rtattr *>(RTA_DATA(attr)); RTA_OK(nested, nested_size);
           nested = RTA_NEXT(nested, nested_size)) {
        if (
          nested->rta_type == IFLA_INFO_KIND && RTA_PAYLOAD(nested) == 5 &&
          std::memcmp(RTA_DATA(nested), "vcan", 5) == 0)
          return true;
      }
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// relative_aim 专用配置：默认 v1，显式 ID；不改变旧入口的 CBoard 默认行为。
// ---------------------------------------------------------------------------
VisionProtocolConfig load_vision_protocol_config(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  VisionProtocolConfig config;

  const auto mode = tools::optional_string(yaml, "can_command_mode", "absolute");
  if (mode != "relative") {
    throw std::runtime_error(
      "can_command_mode 必须是 \"relative\"（当前 \"" + mode + "\"）：" + config_path +
      "。相对角差入口不接受 absolute 模式，避免把角差当绝对角发出去");
  }

  if (!yaml["can_interface"]) throw std::runtime_error("配置缺少 can_interface：" + config_path);
  config.can_interface = yaml["can_interface"].as<std::string>();

  const auto frame = tools::optional_string(yaml, "vision_protocol", "v1");
  if (frame == "v1") {
    config.v1 = true;
  } else if (frame == "legacy") {
    config.v1 = false;
  } else {
    throw std::runtime_error(
      "vision_protocol 取值非法：'" + frame + "'（只能是 \"v1\" 或 \"legacy\"，" + config_path +
      "）");
  }

  auto read_id = [&yaml, &config_path](const std::string & key, bool required) -> uint32_t {
    if (!yaml[key]) {
      if (required) {
        throw std::runtime_error("协议需要显式配置键 '" + key + "'（" + config_path + "）");
      }
      return 0;
    }
    const int value = yaml[key].as<int>();
    if (value < 0 || value > 0x7FF) {
      throw std::runtime_error(
        "'" + key + "' = " + std::to_string(value) + " 不是标准帧 ID（0..0x7FF，" + config_path +
        "）");
    }
    return static_cast<uint32_t>(value);
  };

  config.relative_canid = read_id("relative_canid", true);

  config.control_timeout_ms = tools::optional_double(yaml, "vision_control_timeout_ms", 500.0);
  config.observation_timeout_ms = tools::optional_double(yaml, "relative_frame_timeout_ms", 100.0);
  if (
    !std::isfinite(config.control_timeout_ms) || !std::isfinite(config.observation_timeout_ms) ||
    config.control_timeout_ms <= 0.0 || config.observation_timeout_ms <= 0.0) {
    throw std::runtime_error(
      "vision_control_timeout_ms / relative_frame_timeout_ms 必须 > 0（" + config_path + "）");
  }

  // 发送上限：relative_tx_rate_hz 优先；都为 0 = 有新观测就发
  const double rate_hz = tools::optional_double(yaml, "relative_tx_rate_hz", 0.0);
  const double interval_ms = tools::optional_double(yaml, "relative_send_interval_ms", 0.0);
  if (
    !std::isfinite(rate_hz) || !std::isfinite(interval_ms) || rate_hz < 0.0 || interval_ms < 0.0) {
    throw std::runtime_error(
      "relative_tx_rate_hz / relative_send_interval_ms 不能为负（" + config_path + "）");
  }
  config.tx_min_interval_ms = rate_hz > 0.0 ? 1000.0 / rate_hz : interval_ms;

  if (config.v1) {
    config.handshake_request_canid = read_id("vision_handshake_request_canid", true);
    config.handshake_response_canid = read_id("vision_handshake_response_canid", true);
    config.control_canid = read_id("vision_control_canid", true);

    // 四个 ID 互异（同 ID 收发会自己收到自己，行为完全不可预测）
    const std::vector<std::pair<std::string, uint32_t>> ids = {
      {"relative_canid", config.relative_canid},
      {"vision_handshake_request_canid", config.handshake_request_canid},
      {"vision_handshake_response_canid", config.handshake_response_canid},
      {"vision_control_canid", config.control_canid}};
    for (std::size_t i = 0; i < ids.size(); ++i) {
      for (std::size_t j = i + 1; j < ids.size(); ++j) {
        if (ids[i].second == ids[j].second) {
          throw std::runtime_error(
            "CAN ID 冲突：" + ids[i].first + " 与 " + ids[j].first + " 都是 0x" +
            fmt::format("{:X}", ids[i].second) + "（" + config_path + "）");
        }
      }
    }

    // 也不能占用旧协议里已知的上行 ID（四元数 / 弹速 / 绝对角命令）
    const std::vector<std::string> legacy_keys = {
      "quaternion_canid", "bullet_speed_canid", "send_canid"};
    for (const auto & key : legacy_keys) {
      if (!yaml[key]) continue;
      const int legacy_id = yaml[key].as<int>();
      for (const auto & entry : ids) {
        if (static_cast<int>(entry.second) == legacy_id) {
          throw std::runtime_error(
            entry.first + " 与旧协议的 " + key + " 撞车（都是 0x" + fmt::format("{:X}", legacy_id) +
            "，" + config_path + "）");
        }
      }
    }

    // 实机 ID 必须由电控确认后才可置位：这道门不是形式主义，
    // 写错一个 ID 会往别人的控制帧里灌角差。
    config.ids_confirmed = tools::optional_bool(yaml, "vision_can_ids_confirmed", false);
    bool known = false;
    const bool virtual_link = can_interface_is_virtual(config.can_interface, known);
    if (!config.ids_confirmed && !virtual_link) {
      throw std::runtime_error(
        "非已核实 vcan 接口需要 vision_can_ids_confirmed: true（" + config_path +
        "）：四个 ID 必须先与电控确认；只是本地验证的话用 configs/relative_vcan.yaml 的测试 ID");
    }

    tools::logger()->info(
      "[VisionLink] 接口 {}：{}（可判定={}）", config.can_interface,
      virtual_link ? "虚拟总线（测试）" : "物理总线（实机）", known ? "是" : "否");
  }

  tools::logger()->info(
    "[VisionLink] 模式={} 角差帧=0x{:X} HELLO=0x{:X} HELLO_ACK=0x{:X} CONTROL=0x{:X}；"
    "控制超时={:.0f}ms 观测超时={:.0f}ms 发送最小间隔={:.1f}ms",
    config.v1 ? "v1（握手 + 会话）" : "legacy（无握手，与旧版逐字节一致）", config.relative_canid,
    config.handshake_request_canid, config.handshake_response_canid, config.control_canid,
    config.control_timeout_ms, config.observation_timeout_ms, config.tx_min_interval_ms);
  if (!config.v1) {
    bool known = false;
    if (
      !can_interface_is_virtual(config.can_interface, known) &&
      !tools::optional_bool(yaml, "vision_can_ids_confirmed", false)) {
      throw std::runtime_error("legacy 实发同样需要 vision_can_ids_confirmed: true");
    }
    tools::logger()->warn(
      "[VisionLink] legacy 模式没有握手/启停/颜色协商：只要本帧有目标就往 0x{:X} 发角差。"
      "电控支持 v1 后请改成 vision_protocol: \"v1\"",
      config.relative_canid);
  }

  return config;
}

// ---------------------------------------------------------------------------
// 会话状态机
// ---------------------------------------------------------------------------
VisionLinkCore::VisionLinkCore(const VisionProtocolConfig & config) : config_(config)
{
  // legacy 没有会话概念：状态固定 active，颜色/使能由 config 决定（旧行为）
  if (!config_.v1) {
    state_ = VisionLinkState::active;
    ready_ = true;
  }
  tools::logger()->info(
    "[VisionLink] 初始状态={}（legacy 无握手；v1 需要 CBoard 先发 HELLO）", to_string(state_));
}

void VisionLinkCore::set_ready(bool ready, const std::string & reason)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (ready_ == ready) return;
  ready_ = ready;
  if (!ready && has_session_) expire_session_locked("not_ready");
  tools::logger()->info("[VisionLink] READY={}（{}）", ready ? "true" : "false", reason);
}

void VisionLinkCore::queue_handshake_response_locked(uint32_t session_id, HandshakeStatus status)
{
  encode_handshake_response(session_id, status, pending_response_);
  has_pending_response_ = true;
  ++response_serial_;

  // 只在 status / session 变化时打日志：CBoard 每 500ms 重试一次 HELLO，别刷满日志
  tools::logger()->debug(
    "[VisionLink] HELLO session=0x{:08X} -> {}", session_id, to_string(status));
}

void VisionLinkCore::transition_locked(VisionLinkState state, const std::string & reason)
{
  if (state_ == state) return;

  const VisionLinkState from = state_;
  state_ = state;

  // 离开 active 时立刻补一帧 control=0：不让电控按最后一条有效角差继续走
  if (from == VisionLinkState::active && state != VisionLinkState::active && has_sent_valid_) {
    pending_stop_ = true;
  }

  tools::logger()->info(
    "[VisionLink] {} -> {}（{}）：session={} enable={} color={} seq={} gen={}", to_string(from),
    to_string(state), reason,
    has_session_ ? fmt::format("0x{:08X}", session_id_) : std::string("-"), enable_ ? "1" : "0",
    static_cast<int>(color_), static_cast<int>(sequence_), generation_);
}

void VisionLinkCore::expire_session_locked(const std::string & reason)
{
  const VisionLinkState from = state_;
  if (from == VisionLinkState::active && has_sent_valid_) pending_stop_ = true;

  has_session_ = false;
  enable_ = false;
  color_ = 0;
  sample_.has = false;
  sample_.sent = true;
  stop_sent_for_sample_ = false;
  ++generation_;

  state_ = VisionLinkState::control_timeout;
  tools::logger()->warn(
    "[VisionLink] {} -> control_timeout（{}）：会话 0x{:08X} 作废，必须重新握手才能再发角差",
    to_string(from), reason, session_id_);
}

void VisionLinkCore::poll_locked(Clock::time_point now)
{
  if (config_.v1 && has_session_ && age_ms(now, last_control_time_) > config_.control_timeout_ms) {
    ++stats_.timeouts;
    expire_session_locked("control_timeout");
  }

  // 观测时效：有效样本过期后必须补一帧 control=0（即使视觉线程卡住 / 没有下一帧）
  if (sample_.has && sample_.valid && age_ms(now, sample_.time) > config_.observation_timeout_ms) {
    if (!sample_.sent) {
      sample_.sent = true;
      ++stats_.tx_skipped_stale;
    }
    if (!stop_sent_for_sample_) {
      pending_stop_ = true;
      stop_sent_for_sample_ = true;
    }
  }
}

void VisionLinkCore::on_frame(
  uint32_t can_id, const uint8_t * data, uint8_t dlc, Clock::time_point now)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!config_.v1) return;  // legacy：不解析任何上行帧

  poll_locked(now);

  if (can_id == config_.handshake_request_canid) {
    ++stats_.rx_handshake;

    uint8_t version = 0;
    uint32_t session_id = 0;
    if (!decode_handshake_request(data, dlc, version, session_id)) {
      ++stats_.rejected_malformed;
      return;  // 畸形 HELLO 不回包：回了就是「对垃圾帧的反射」，可能被当成放大器
    }

    if (version != kVisionProtocolVersion) {
      ++stats_.rejected_version;
      // 版本不兼容：回 VERSION_MISMATCH 并回显 session，方便电控打日志
      queue_handshake_response_locked(session_id, HandshakeStatus::version_mismatch);
      return;
    }

    if (!ready_) {
      queue_handshake_response_locked(session_id, HandshakeStatus::not_ready);
      return;
    }

    if (has_session_ && session_id == session_id_) {
      // 幂等：同 session 重复 HELLO 只重发 READY，不复位启停/颜色/generation，
      // 也不刷新心跳（心跳只由 CONTROL 续）
    } else {
      if (has_sent_valid_) pending_stop_ = true;
      if (has_session_) {
        tools::logger()->warn(
          "[VisionLink] CBoard 换会话：0x{:08X} -> 0x{:08X}，旧会话的观测与使能全部作废",
          session_id_, session_id);
      }
      has_session_ = true;
      session_id_ = session_id;
      enable_ = false;
      color_ = 0;
      sequence_ = 0;
      has_sequence_ = false;
      // 从握手时刻起算超时：刚建会话还没收到 CONTROL 时不应立刻判超时
      last_control_time_ = now;
      sample_.has = false;
      ++generation_;  // 让「握手前算出来的观测」失效
      generation_since_ = now;
      state_ = VisionLinkState::wait_handshake;
      transition_locked(VisionLinkState::wait_control, "hello");
    }

    queue_handshake_response_locked(session_id, HandshakeStatus::ready);
    return;
  }

  if (can_id == config_.control_canid) {
    ++stats_.rx_control;

    uint32_t session_id = 0;
    bool enable = false;
    uint8_t color = 0;
    uint8_t sequence = 0;
    if (!decode_control(data, dlc, session_id, enable, color, sequence)) {
      ++stats_.rejected_malformed;
      if (has_session_) expire_session_locked("malformed_control");
      return;
    }

    if (!has_session_ || session_id != session_id_) {
      ++stats_.rejected_session;
      return;
    }

    if (!valid_control_color(color)) {
      ++stats_.rejected_color;
      expire_session_locked("invalid_color");
      return;
    }

    // 模 256 前向窗口：允许 255→0，重复/倒序不续心跳；重启端必须换 session。
    const uint8_t advance = static_cast<uint8_t>(sequence - sequence_);
    if (has_sequence_ && (advance == 0 || advance >= 128)) return;
    has_sequence_ = true;

    last_control_time_ = now;
    sequence_ = sequence;

    const bool changed = (enable != enable_) || (color != color_);
    enable_ = enable;
    color_ = color;
    if (changed) {
      if (has_sent_valid_) pending_stop_ = true;
      ++generation_;
      generation_since_ = now;
      sample_.has = false;  // 变色/启停之前算出来的观测不允许生效
    }

    transition_locked(enable ? VisionLinkState::active : VisionLinkState::stopped, "control");
    return;
  }

  // 其它 ID 与本协议无关：不改任何状态（不做「收到什么就信什么」）
}

void VisionLinkCore::poll(Clock::time_point now)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  poll_locked(now);
}

bool VisionLinkCore::publish(const Command & command, uint32_t generation, Clock::time_point now)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (generation != generation_ || (config_.v1 && now < generation_since_)) {
    // 观测是在旧的会话/颜色/启停下算出来的：直接丢，不入样
    ++stats_.tx_skipped_generation;
    return false;
  }

  sample_.has = true;
  sample_.valid = command.control && std::isfinite(command.yaw) && std::isfinite(command.pitch);
  sample_.generation = generation;
  sample_.sent = false;
  sample_.time = now;
  stop_sent_for_sample_ = false;
  encode_relative_command(command, sample_.data);
  if (!sample_.valid) {
    pending_stop_ = true;
    sample_.sent = true;
  }

  return command.control && state_ == VisionLinkState::active;
}

bool VisionLinkCore::next_frame(
  Clock::time_point now, uint32_t & can_id, uint8_t data[8], FrameKind & kind)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  poll_locked(now);
  kind = FrameKind::none;

  // 1) 握手应答；待发送的安全停止帧优先于 ACK。
  if (has_pending_response_ && !pending_stop_) {
    can_id = config_.handshake_response_canid;
    std::memcpy(data, pending_response_, sizeof(pending_response_));
    sent_response_serial_ = response_serial_;
    kind = FrameKind::handshake_response;
    last_frame_was_stop_ = false;
    return true;
  }

  // 2) 停止帧最优先、并且不限频（安全动作不能被限频挡住）
  if (pending_stop_) {
    const Command idle{false, false, 0.0, 0.0};
    encode_relative_command(idle, data);
    can_id = config_.relative_canid;
    kind = FrameKind::relative;
    last_frame_was_stop_ = true;
    return true;
  }

  if (!sample_.has) {
    ++stats_.tx_skipped_empty;
    return false;
  }

  if (config_.v1) {
    if (state_ != VisionLinkState::active) return false;
    if (sample_.generation != generation_) {
      ++stats_.tx_skipped_generation;
      return false;
    }
  }

  if (sample_.sent) return false;  // 同一个观测只发一次，不重复灌角差

  if (
    config_.tx_min_interval_ms > 0.0 && has_tx_time_ &&
    age_ms(now, last_tx_time_) < config_.tx_min_interval_ms) {
    ++stats_.tx_skipped_rate;
    return false;
  }

  std::memcpy(data, sample_.data, sizeof(sample_.data));
  can_id = config_.relative_canid;
  sample_.sent = true;
  last_tx_time_ = now;
  has_tx_time_ = true;
  kind = FrameKind::relative;
  last_frame_was_stop_ = !sample_.valid;
  if (sample_.valid) has_sent_valid_ = true;
  return true;
}

void VisionLinkCore::note_send(bool ok, FrameKind kind)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (!ok) {
    ++stats_.tx_fail;
    return;  // pending_response_ 保留：下一个 tick 重试
  }

  if (kind == FrameKind::handshake_response) {
    ++stats_.tx_handshake_response;
    // 期间若又收到一个新 HELLO（serial 变了），不要把新的那份一起清掉
    if (response_serial_ == sent_response_serial_) has_pending_response_ = false;
    return;
  }

  if (kind == FrameKind::relative) {
    if (last_frame_was_stop_) {
      ++stats_.tx_stop;
      pending_stop_ = false;
      has_sent_valid_ = false;
    } else {
      ++stats_.tx_relative;
    }
  }
}

void VisionLinkCore::request_stop_frame()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  pending_stop_ = true;
  sample_.sent = true;  // 收尾时不再补发有效角差
}

VisionLinkSnapshot VisionLinkCore::snapshot(Clock::time_point now) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  VisionLinkSnapshot snapshot;
  snapshot.state = state_;
  snapshot.has_session = has_session_;
  snapshot.session_id = session_id_;
  snapshot.enable = enable_;
  snapshot.color = color_;
  snapshot.sequence = sequence_;
  snapshot.generation = generation_;
  snapshot.ready = ready_;
  snapshot.control_age_ms = has_session_ ? age_ms(now, last_control_time_) : 0.0;
  return snapshot;
}

VisionLinkStats VisionLinkCore::stats() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stats_;
}

bool VisionLinkCore::transmit(
  Clock::time_point now, const std::function<bool(uint32_t, const uint8_t *)> & sender)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint32_t id = 0;
  uint8_t data[8]{};
  FrameKind kind = FrameKind::none;
  if (!next_frame(now, id, data, kind)) return false;
  note_send(sender(id, data), kind);
  return true;
}

// ---------------------------------------------------------------------------
// VisionCboardLink：把 core 接到 CBoard + 一条独立发送线程
// ---------------------------------------------------------------------------
VisionCboardLink::VisionCboardLink(const VisionProtocolConfig & config, CBoard & board)
: board_(board), core_(config)
{
}

VisionCboardLink::~VisionCboardLink()
{
  board_.clear_frame_handlers();
  stop();
}

void VisionCboardLink::on_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (can_id != core_.config().handshake_request_canid && can_id != core_.config().control_canid)
    return;
  std::lock_guard<std::mutex> lock(receive_mutex_);
  if (received_.size() >= 64) {
    receive_overflow_ = true;
    return;
  }
  ReceivedFrame frame{can_id, dlc, {}, Clock::now()};
  if (data) std::memcpy(frame.data.data(), data, std::min<unsigned>(dlc, 8));
  received_.push_back(frame);
}

void VisionCboardLink::start()
{
  if (running_) return;
  quit_ = false;
  running_ = true;
  thread_ = std::thread([this] { run(); });
  tools::logger()->info("[VisionLink] 发送线程已启动（5ms tick，与取流/检测解耦）");
}

void VisionCboardLink::stop()
{
  if (!running_) return;
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  running_ = false;
}

void VisionCboardLink::run()
{
  using namespace std::chrono_literals;

  while (!quit_) {
    // 接收队列与发送资格检查串行化：入队成功的停止不会被旧有效帧越过。
    {
      std::lock_guard<std::mutex> lock(receive_mutex_);
      if (receive_overflow_) {
        core_.on_frame(core_.config().control_canid, nullptr, 0, Clock::now());
        received_.clear();
        receive_overflow_ = false;
      }
      for (const auto & frame : received_) {
        core_.on_frame(frame.id, frame.data.data(), frame.dlc, frame.time);
      }
      received_.clear();
      core_.transmit(Clock::now(), [this](uint32_t id, const uint8_t * bytes) {
        return board_.send_raw(id, bytes);
      });
    }

    std::this_thread::sleep_for(5ms);
  }

  // 收尾：补一帧 control=0（best effort）。这一步不依赖取流/检测是否还在跑。
  core_.request_stop_frame();
  const auto before = core_.stats().tx_stop;
  for (int attempt = 0; attempt < 5 && core_.stats().tx_stop == before; ++attempt) {
    core_.transmit(Clock::now(), [this](uint32_t id, const uint8_t * bytes) {
      return board_.send_raw(id, bytes);
    });
    std::this_thread::sleep_for(5ms);
  }
  tools::logger()->info(
    "[VisionLink] 退出停止帧：{}",
    core_.stats().tx_stop > before ? "已写入 SocketCAN" : "发送失败");
}

std::string VisionCboardLink::stats_text() const
{
  const auto s = stats();
  return fmt::format(
    "rx(HELLO={} CONTROL={}) 拒绝(版本={} 会话={} 畸形={} 颜色={}) "
    "tx(ACK={} 角差={} 停止={} 失败={}) 跳过(限频={} 空={} 过期={} 旧代={}) 超时={}",
    s.rx_handshake, s.rx_control, s.rejected_version, s.rejected_session, s.rejected_malformed,
    s.rejected_color, s.tx_handshake_response, s.tx_relative, s.tx_stop, s.tx_fail,
    s.tx_skipped_rate, s.tx_skipped_empty, s.tx_skipped_stale, s.tx_skipped_generation, s.timeouts);
}

}  // namespace io
