#include "board_state_can.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
using Clock = std::chrono::steady_clock;

double age_ms(Clock::time_point now, Clock::time_point then)
{
  const double ms =
    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - then).count();
  return ms > 0 ? ms : 0;
}

Clock::duration duration_ms(double milliseconds)
{
  return std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double, std::milli>(milliseconds));
}

uint32_t read_u32(const uint8_t * data)
{
  return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

void write_u32(uint32_t value, uint8_t * data)
{
  data[0] = static_cast<uint8_t>(value >> 24);
  data[1] = static_cast<uint8_t>(value >> 16);
  data[2] = static_cast<uint8_t>(value >> 8);
  data[3] = static_cast<uint8_t>(value);
}

void encode_handshake(
  BoardStateCanMessage type, uint32_t session_id, uint8_t status, uint8_t color, uint8_t * data)
{
  if (data == nullptr) return;
  data[0] = static_cast<uint8_t>(type);
  data[1] = BOARD_STATE_CAN_VERSION;
  write_u32(session_id, data + 2);
  data[6] = status;
  data[7] = color;
}

bool decode_handshake(
  BoardStateCanMessage expected, const uint8_t * data, uint8_t dlc, uint8_t & version,
  uint32_t & session_id)
{
  if (data == nullptr || dlc != 8 || data[0] != static_cast<uint8_t>(expected)) return false;
  if (data[6] != 0 || data[7] != 0) return false;
  version = data[1];
  session_id = read_u32(data + 2);
  return true;
}

bool valid_color(BoardStateCanColor color)
{
  return color == BoardStateCanColor::red || color == BoardStateCanColor::blue;
}

void validate_config(const BoardStateCanConfig & config)
{
  const uint32_t ids[] = {config.control_canid, config.response_canid, config.data_canid};
  for (const uint32_t id : ids) {
    if (id > 0x7FF) throw std::runtime_error("board-state CAN ID exceeds standard 11-bit range");
    if (id == 0x1FF || id == 0x2FF || (id >= 0x200 && id <= 0x20C))
      throw std::runtime_error("board-state CAN ID collides with motor command/feedback IDs");
    if (
      id == 0xFF || id == 0x100 || id == 0x101 || (id >= 0x300 && id <= 0x303) ||
      (id >= 0x310 && id <= 0x312))
      throw std::runtime_error("board-state CAN ID collides with a legacy/vision protocol ID");
  }
  if (
    config.control_canid == config.response_canid || config.control_canid == config.data_canid ||
    config.response_canid == config.data_canid)
    throw std::runtime_error("board-state CAN control, response, and data IDs must be distinct");
  if (
    !std::isfinite(config.retry_ms) || config.retry_ms <= 0 ||
    !std::isfinite(config.fragment_timeout_ms) || config.fragment_timeout_ms <= 0 ||
    !std::isfinite(config.stream_timeout_ms) || config.stream_timeout_ms <= 0)
    throw std::runtime_error("board-state CAN timeouts must be finite and positive");
  if (config.fragment_timeout_ms >= config.stream_timeout_ms)
    throw std::runtime_error("fragment timeout must be shorter than stream timeout");
}

void reject_collisions_with_existing_ids(
  const YAML::Node & yaml, const BoardStateCanConfig & config)
{
  const char * keys[] = {
    "quaternion_canid",
    "bullet_speed_canid",
    "send_canid",
    "relative_canid",
    "vision_handshake_request_canid",
    "vision_handshake_response_canid",
    "vision_control_canid"};
  const uint32_t new_ids[] = {config.control_canid, config.response_canid, config.data_canid};
  for (const char * key : keys) {
    if (!yaml[key]) continue;
    const int old_id = yaml[key].as<int>();
    for (const uint32_t new_id : new_ids) {
      if (old_id >= 0 && static_cast<uint32_t>(old_id) == new_id)
        throw std::runtime_error(std::string("board-state CAN ID collides with ") + key);
    }
  }
}

}  // namespace

std::string to_string(BoardStateCanColor color)
{
  switch (color) {
    case BoardStateCanColor::red:
      return "red";
    case BoardStateCanColor::blue:
      return "blue";
    case BoardStateCanColor::unspecified:
      return "unspecified";
  }
  return "invalid";
}

uint8_t board_state_can_session_tag(uint32_t session_id)
{
  return static_cast<uint8_t>(session_id & 0xFF);
}

void encode_board_state_can_start(uint32_t session_id, uint8_t data[8])
{
  encode_handshake(BoardStateCanMessage::start, session_id, 0, 0, data);
}

void encode_board_state_can_final_ack(uint32_t session_id, uint8_t data[8])
{
  encode_handshake(BoardStateCanMessage::final_ack, session_id, 0, 0, data);
}

void encode_board_state_can_ack(
  uint32_t session_id, BoardStateCanStatus status, BoardStateCanColor color, uint8_t data[8])
{
  encode_handshake(
    BoardStateCanMessage::ack_color, session_id, static_cast<uint8_t>(status),
    static_cast<uint8_t>(color), data);
}

bool decode_board_state_can_start(
  const uint8_t * data, uint8_t dlc, uint8_t & version, uint32_t & session_id)
{
  return decode_handshake(BoardStateCanMessage::start, data, dlc, version, session_id);
}

bool decode_board_state_can_final_ack(
  const uint8_t * data, uint8_t dlc, uint8_t & version, uint32_t & session_id)
{
  return decode_handshake(BoardStateCanMessage::final_ack, data, dlc, version, session_id);
}

bool decode_board_state_can_ack(const uint8_t * data, uint8_t dlc, BoardStateCanAck & ack)
{
  if (
    data == nullptr || dlc != 8 || data[0] != static_cast<uint8_t>(BoardStateCanMessage::ack_color))
    return false;
  ack.version = data[1];
  ack.session_id = read_u32(data + 2);
  ack.status = static_cast<BoardStateCanStatus>(data[6]);
  ack.color = static_cast<BoardStateCanColor>(data[7]);
  return true;
}

bool encode_board_state_can_fragment(
  const BoardState & state, uint32_t session_id, uint8_t sequence, uint8_t fragment,
  uint8_t data[8])
{
  if (data == nullptr || fragment >= BOARD_STATE_CAN_FRAGMENT_COUNT) return false;
  std::memset(data, 0, 8);
  data[0] = static_cast<uint8_t>((fragment << 5) | (sequence & BOARD_STATE_CAN_SEQUENCE_MASK));
  data[1] = board_state_can_session_tag(session_id);
  const size_t offset = fragment * BOARD_STATE_CAN_FRAGMENT_PAYLOAD;
  const size_t length = std::min(BOARD_STATE_CAN_FRAGMENT_PAYLOAD, sizeof(BoardState) - offset);
  std::memcpy(data + 2, reinterpret_cast<const uint8_t *>(&state) + offset, length);
  return true;
}

bool decode_board_state_can_fragment_header(
  const uint8_t * data, uint8_t dlc, uint8_t & sequence, uint8_t & session_tag, uint8_t & fragment)
{
  if (data == nullptr || dlc != 8) return false;
  fragment = static_cast<uint8_t>(data[0] >> 5);
  if (fragment >= BOARD_STATE_CAN_FRAGMENT_COUNT) return false;
  if (fragment == BOARD_STATE_CAN_FRAGMENT_COUNT - 1 && data[7] != 0) return false;
  sequence = static_cast<uint8_t>(data[0] & BOARD_STATE_CAN_SEQUENCE_MASK);
  session_tag = data[1];
  return true;
}

BoardStateCanConfig load_board_state_can_config(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  BoardStateCanConfig config;
  const auto read_id = [&yaml](const char * key, uint32_t fallback) {
    return yaml[key] ? yaml[key].as<uint32_t>() : fallback;
  };
  config.control_canid = read_id("board_state_can_control_canid", config.control_canid);
  config.response_canid = read_id("board_state_can_response_canid", config.response_canid);
  config.data_canid = read_id("board_state_can_data_canid", config.data_canid);
  config.retry_ms = tools::optional_double(yaml, "board_state_can_retry_ms", config.retry_ms);
  config.fragment_timeout_ms =
    tools::optional_double(yaml, "board_state_can_fragment_timeout_ms", config.fragment_timeout_ms);
  config.stream_timeout_ms =
    tools::optional_double(yaml, "board_state_can_stream_timeout_ms", config.stream_timeout_ms);
  config.ids_confirmed = tools::optional_bool(yaml, "board_state_can_ids_confirmed", false);
  config.can_interface = tools::optional_string(yaml, "can_interface", "");
  validate_config(config);
  reject_collisions_with_existing_ids(yaml, config);
  return config;
}

BoardStateCanCore::BoardStateCanCore(const BoardStateCanConfig & config) : config_(config)
{
  validate_config(config_);
}

void BoardStateCanCore::start(uint32_t session_id, Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = true;
  begin_session_locked(session_id, now);
}

void BoardStateCanCore::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = false;
  link_state_ = LinkState::idle;
  start_pending_ = false;
  final_pending_ = false;
  clear_sample_locked();
  has_completed_sequence_ = false;
  completed_sequence_ = 0;
  imu_history_.clear();
  has_state_ = false;
  state_ = MinipcState{};
}

uint32_t BoardStateCanCore::next_session(uint32_t session_id)
{
  ++session_id;
  return session_id == 0 ? 1 : session_id;
}

void BoardStateCanCore::begin_session_locked(uint32_t session_id, Clock::time_point now)
{
  session_id_ = session_id == 0 ? 1 : session_id;
  color_ = BoardStateCanColor::unspecified;
  link_state_ = LinkState::wait_response;
  start_pending_ = true;
  final_pending_ = false;
  next_start_tx_ = now;
  next_final_tx_ = Clock::time_point{};
  final_ack_since_ = Clock::time_point{};
  streaming_since_ = Clock::time_point{};
  last_complete_ = Clock::time_point{};
  clear_sample_locked();
  has_completed_sequence_ = false;
  completed_sequence_ = 0;
  imu_history_.clear();
  has_state_ = false;
  state_ = MinipcState{};
}

void BoardStateCanCore::clear_sample_locked()
{
  partial_.active = false;
  partial_.sequence = 0;
  partial_.mask = 0;
  partial_.bytes.fill(0);
  partial_.since = Clock::time_point{};
}

void BoardStateCanCore::poll(Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  poll_locked(now);
}

void BoardStateCanCore::poll_locked(Clock::time_point now)
{
  if (!running_) return;
  if (partial_.active && age_ms(now, partial_.since) >= config_.fragment_timeout_ms) {
    ++stats_.partial_timeouts;
    clear_sample_locked();
  }
  if (
    link_state_ == LinkState::wait_final_ack &&
    age_ms(now, final_ack_since_) >= config_.stream_timeout_ms) {
    // The final ACK may have been lost together with the CBoard session. A new
    // START is required; retrying only the old final ACK can never recover.
    ++stats_.stream_timeouts;
    begin_session_locked(next_session(session_id_), now);
    return;
  }
  if (link_state_ != LinkState::streaming) return;
  const auto since = has_state_ ? last_complete_ : streaming_since_;
  if (age_ms(now, since) >= config_.stream_timeout_ms) {
    ++stats_.stream_timeouts;
    begin_session_locked(next_session(session_id_), now);
  }
}

void BoardStateCanCore::on_frame(
  uint32_t can_id, const uint8_t * data, uint8_t dlc, Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_) return;
  poll_locked(now);
  if (can_id == config_.response_canid) {
    on_ack_locked(data, dlc, now);
  } else if (can_id == config_.data_canid) {
    on_fragment_locked(data, dlc, now);
  }
}

void BoardStateCanCore::on_ack_locked(const uint8_t * data, uint8_t dlc, Clock::time_point now)
{
  ++stats_.rx_ack;
  BoardStateCanAck ack;
  if (!decode_board_state_can_ack(data, dlc, ack)) {
    ++stats_.rejected_malformed;
    return;
  }
  if (ack.version != BOARD_STATE_CAN_VERSION) {
    ++stats_.rejected_version;
    return;
  }
  if (ack.session_id != session_id_) {
    ++stats_.rejected_session;
    return;
  }

  if (ack.status == BoardStateCanStatus::ready) {
    if (!valid_color(ack.color)) {
      ++stats_.rejected_color;
      return;
    }
    if (link_state_ == LinkState::streaming) {
      if (ack.color != color_) {
        ++stats_.rejected_color;
        return;
      }
      // The previous final ACK may have been lost. Repeat it without resetting
      // the stream, IMU history, or the negotiated color.
      final_pending_ = true;
      next_final_tx_ = now;
      return;
    }
    color_ = ack.color;
    if (link_state_ != LinkState::wait_final_ack) final_ack_since_ = now;
    link_state_ = LinkState::wait_final_ack;
    start_pending_ = false;
    final_pending_ = true;
    next_final_tx_ = now;
    return;
  }

  if (ack.status == BoardStateCanStatus::version_mismatch) ++stats_.rejected_version;
  if (
    ack.status != BoardStateCanStatus::not_ready &&
    ack.status != BoardStateCanStatus::version_mismatch) {
    ++stats_.rejected_malformed;
    return;
  }
  if (link_state_ != LinkState::streaming) {
    const bool retry_already_scheduled = start_pending_;
    link_state_ = LinkState::wait_response;
    start_pending_ = true;
    final_pending_ = false;
    if (!retry_already_scheduled) next_start_tx_ = now + duration_ms(config_.retry_ms);
  }
}

void BoardStateCanCore::on_fragment_locked(const uint8_t * data, uint8_t dlc, Clock::time_point now)
{
  ++stats_.rx_data;
  if (link_state_ != LinkState::streaming) return;

  uint8_t sequence = 0;
  uint8_t session_tag = 0;
  uint8_t fragment = 0;
  if (!decode_board_state_can_fragment_header(data, dlc, sequence, session_tag, fragment)) {
    ++stats_.rejected_fragment;
    return;
  }
  if (session_tag != board_state_can_session_tag(session_id_)) {
    ++stats_.rejected_session;
    return;
  }

  const auto not_forward = [](uint8_t candidate, uint8_t reference) {
    const uint8_t advance =
      static_cast<uint8_t>((candidate - reference) & BOARD_STATE_CAN_SEQUENCE_MASK);
    return advance == 0 || advance >= 16;
  };
  if (
    partial_.active && partial_.sequence != sequence && not_forward(sequence, partial_.sequence)) {
    ++stats_.rejected_fragment;
    return;
  }
  if (!partial_.active && has_completed_sequence_ && not_forward(sequence, completed_sequence_)) {
    ++stats_.rejected_fragment;
    return;
  }
  if (!partial_.active || partial_.sequence != sequence) {
    clear_sample_locked();
    partial_.active = true;
    partial_.sequence = sequence;
    partial_.since = now;
  }

  const size_t offset = fragment * BOARD_STATE_CAN_FRAGMENT_PAYLOAD;
  const size_t length = std::min(BOARD_STATE_CAN_FRAGMENT_PAYLOAD, sizeof(BoardState) - offset);
  const uint8_t bit = static_cast<uint8_t>(1U << fragment);
  if ((partial_.mask & bit) != 0) {
    if (std::memcmp(partial_.bytes.data() + offset, data + 2, length) == 0) return;
    ++stats_.rejected_fragment;
    clear_sample_locked();
    return;
  }

  std::memcpy(partial_.bytes.data() + offset, data + 2, length);
  partial_.mask = static_cast<uint8_t>(partial_.mask | bit);
  if (partial_.mask == 0x1F) complete_sample_locked(now);
}

void BoardStateCanCore::complete_sample_locked(Clock::time_point now)
{
  BoardState message{};
  std::memcpy(&message, partial_.bytes.data(), sizeof(message));
  const auto finite = [](const float * values, size_t count) {
    for (size_t i = 0; i < count; ++i) {
      if (!std::isfinite(values[i])) return false;
    }
    return true;
  };
  Eigen::Quaterniond q(message.q[0], message.q[1], message.q[2], message.q[3]);
  const float scalars[] = {message.yaw, message.pitch, message.bullet_speed};
  const bool finite_quaternion = std::isfinite(message.q[0]) && std::isfinite(message.q[1]) &&
                                 std::isfinite(message.q[2]) && std::isfinite(message.q[3]);
  if (!finite_quaternion || !finite(scalars, 3) || !std::isfinite(q.norm()) || !(q.norm() > 1e-6)) {
    ++stats_.rejected_fragment;
    clear_sample_locked();
    return;
  }
  q.normalize();

  state_.q = q;
  state_.yaw = message.yaw;
  state_.pitch = message.pitch;
  state_.bullet_speed = message.bullet_speed;
  std::memcpy(state_.mode.data(), message.mode, MODE_LEN);
  state_.timestamp = now;
  has_state_ = true;
  last_complete_ = now;
  imu_history_.push_back({q, now});
  if (imu_history_.size() > 1000) imu_history_.pop_front();
  ++stats_.valid_packets;
  has_completed_sequence_ = true;
  completed_sequence_ = partial_.sequence;
  clear_sample_locked();
}

bool BoardStateCanCore::transmit(Clock::time_point now, const Sender & sender)
{
  std::lock_guard<std::mutex> lock(mutex_);
  poll_locked(now);
  uint32_t can_id = 0;
  uint8_t data[8]{};
  FrameKind kind = FrameKind::none;
  if (!next_frame_locked(now, can_id, data, kind)) return false;
  note_send_locked(sender(can_id, data), kind, now);
  return true;
}

bool BoardStateCanCore::next_frame_locked(
  Clock::time_point now, uint32_t & can_id, uint8_t data[8], FrameKind & kind)
{
  kind = FrameKind::none;
  if (!running_) return false;
  const auto retry = duration_ms(config_.retry_ms);
  if (final_pending_ && now >= next_final_tx_) {
    encode_board_state_can_final_ack(session_id_, data);
    can_id = config_.control_canid;
    kind = FrameKind::final_ack;
    next_final_tx_ = now + retry;
    return true;
  }
  if (start_pending_ && now >= next_start_tx_) {
    encode_board_state_can_start(session_id_, data);
    can_id = config_.control_canid;
    kind = FrameKind::start;
    next_start_tx_ = now + retry;
    return true;
  }
  return false;
}

void BoardStateCanCore::note_send_locked(bool ok, FrameKind kind, Clock::time_point now)
{
  if (!ok) {
    ++stats_.tx_fail;
    return;
  }
  if (kind == FrameKind::start) {
    ++stats_.tx_start;
    return;
  }
  if (kind != FrameKind::final_ack) return;
  ++stats_.tx_final_ack;
  if (link_state_ == LinkState::wait_final_ack) {
    link_state_ = LinkState::streaming;
    final_pending_ = false;
    streaming_since_ = now;
  } else if (link_state_ == LinkState::streaming) {
    final_pending_ = false;
  }
}

MinipcState BoardStateCanCore::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

Eigen::Quaterniond BoardStateCanCore::imu_at(Clock::time_point timestamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (imu_history_.empty()) return state_.q;

  while (imu_history_.size() > 1 && imu_history_[1].timestamp <= timestamp)
    imu_history_.pop_front();
  const auto ahead = imu_history_.front();
  if (timestamp <= ahead.timestamp || imu_history_.size() == 1) return ahead.q;

  const auto behind = imu_history_[1];
  const double span = std::chrono::duration<double>(behind.timestamp - ahead.timestamp).count();
  if (!(span > 0)) return behind.q;
  const double k = std::chrono::duration<double>(timestamp - ahead.timestamp).count() / span;
  return ahead.q.slerp(k, behind.q).normalized();
}

BoardStateCanCore::Snapshot BoardStateCanCore::snapshot(Clock::time_point now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  Snapshot result;
  result.running = running_;
  result.streaming = link_state_ == LinkState::streaming;
  result.session_id = session_id_;
  result.color = color_;
  result.state_age_ms = has_state_ ? age_ms(now, last_complete_) : 0;
  result.state_valid =
    result.streaming && has_state_ && result.state_age_ms < config_.stream_timeout_ms;
  return result;
}

BoardStateCanCore::Stats BoardStateCanCore::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

BoardStateCanLink::BoardStateCanLink(const BoardStateCanConfig & config, CBoard & board)
: board_(board), core_(config)
{
}

BoardStateCanLink::~BoardStateCanLink() { stop(); }

uint32_t BoardStateCanLink::make_session_id()
{
  std::random_device random;
  uint32_t session = random() ^ static_cast<uint32_t>(Clock::now().time_since_epoch().count());
  return session == 0 ? 1 : session;
}

void BoardStateCanLink::start()
{
  if (running_) return;
  if (!core_.config().ids_confirmed)
    throw std::runtime_error(
      "board-state CAN IDs are candidates; set board_state_can_ids_confirmed: true after "
      "checking the physical bus allocation");
  if (!core_.config().can_interface.empty() && core_.config().can_interface != board_.interface())
    throw std::runtime_error(
      "board-state CAN interface does not match the CBoard SocketCAN interface: " +
      core_.config().can_interface + " vs " + board_.interface());

  quit_ = false;
  running_ = true;
  try {
    handler_id_ = board_.add_frame_handler(
      [this](uint32_t can_id, const uint8_t * data, uint8_t dlc) { on_frame(can_id, data, dlc); });
    if (handler_id_ == 0) throw std::runtime_error("failed to subscribe to CBoard frames");
    core_.start(make_session_id(), Clock::now());
    thread_ = std::thread([this] { run(); });
  } catch (...) {
    core_.stop();
    if (handler_id_ != 0) board_.remove_frame_handler(handler_id_);
    handler_id_ = 0;
    running_ = false;
    throw;
  }
  tools::logger()->info(
    "[BoardStateCAN] START=0x{:X} ACK+color=0x{:X} data=0x{:X} on {}", core_.config().control_canid,
    core_.config().response_canid, core_.config().data_canid, core_.config().can_interface);
}

void BoardStateCanLink::stop()
{
  if (!running_) return;
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (handler_id_ != 0) board_.remove_frame_handler(handler_id_);
  handler_id_ = 0;
  {
    std::lock_guard<std::mutex> lock(receive_mutex_);
    responses_.clear();
    telemetry_.clear();
  }
  core_.stop();
  running_ = false;
}

void BoardStateCanLink::on_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (!running_) return;
  ReceivedFrame frame;
  frame.id = can_id;
  frame.dlc = dlc;
  frame.time = Clock::now();
  if (data != nullptr) std::memcpy(frame.data.data(), data, std::min<unsigned>(dlc, 8));

  std::lock_guard<std::mutex> lock(receive_mutex_);
  if (can_id == core_.config().response_canid) {
    if (responses_.size() >= 16) {
      ++dropped_frames_;
      return;
    }
    responses_.push_back(frame);
  } else if (can_id == core_.config().data_canid) {
    if (telemetry_.size() >= 128) {
      telemetry_.pop_front();
      ++dropped_frames_;
    }
    telemetry_.push_back(frame);
  }
}

void BoardStateCanLink::run()
{
  using namespace std::chrono_literals;
  while (!quit_) {
    std::deque<ReceivedFrame> responses;
    std::deque<ReceivedFrame> telemetry;
    {
      std::lock_guard<std::mutex> lock(receive_mutex_);
      responses.swap(responses_);
      telemetry.swap(telemetry_);
    }
    for (const auto & frame : responses)
      core_.on_frame(frame.id, frame.data.data(), frame.dlc, frame.time);
    for (const auto & frame : telemetry)
      core_.on_frame(frame.id, frame.data.data(), frame.dlc, frame.time);
    const auto now = Clock::now();
    core_.poll(now);
    core_.transmit(
      now, [this](uint32_t id, const uint8_t * data) { return board_.send_raw(id, data); });
    std::this_thread::sleep_for(1ms);
  }
}

}  // namespace io
