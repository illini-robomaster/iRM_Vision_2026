#ifndef IO__BOARD_STATE_CAN_HPP
#define IO__BOARD_STATE_CAN_HPP

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "io/minipc/minipc.hpp"

namespace io
{

// Wire definitions for documentation/board_state_can_protocol.md. The existing
// vision-control session is intentionally independent; these IDs must not
// overlap its IDs, legacy telemetry IDs, or motor traffic.
constexpr uint8_t BOARD_STATE_CAN_VERSION = 1;
constexpr size_t BOARD_STATE_CAN_FRAME_LEN = 8;
constexpr size_t BOARD_STATE_CAN_FRAGMENT_COUNT = 5;
constexpr size_t BOARD_STATE_CAN_FRAGMENT_PAYLOAD = 6;
constexpr uint8_t BOARD_STATE_CAN_SEQUENCE_MASK = 0x1F;

enum class BoardStateCanMessage : uint8_t
{
  start = 0x01,
  ack_color = 0x02,
  final_ack = 0x03,
};

enum class BoardStateCanStatus : uint8_t
{
  ready = 0,
  not_ready = 1,
  version_mismatch = 2,
};

enum class BoardStateCanColor : uint8_t
{
  unspecified = 0,
  red = 1,
  blue = 2,
};
std::string to_string(BoardStateCanColor color);

struct BoardStateCanAck
{
  uint8_t version = 0;
  uint32_t session_id = 0;
  BoardStateCanStatus status = BoardStateCanStatus::not_ready;
  BoardStateCanColor color = BoardStateCanColor::unspecified;
};

uint8_t board_state_can_session_tag(uint32_t session_id);
void encode_board_state_can_start(uint32_t session_id, uint8_t data[8]);
void encode_board_state_can_final_ack(uint32_t session_id, uint8_t data[8]);
void encode_board_state_can_ack(
  uint32_t session_id, BoardStateCanStatus status, BoardStateCanColor color, uint8_t data[8]);
bool decode_board_state_can_start(
  const uint8_t * data, uint8_t dlc, uint8_t & version, uint32_t & session_id);
bool decode_board_state_can_final_ack(
  const uint8_t * data, uint8_t dlc, uint8_t & version, uint32_t & session_id);
bool decode_board_state_can_ack(const uint8_t * data, uint8_t dlc, BoardStateCanAck & ack);
bool encode_board_state_can_fragment(
  const BoardState & state, uint32_t session_id, uint8_t sequence, uint8_t fragment,
  uint8_t data[8]);
bool decode_board_state_can_fragment_header(
  const uint8_t * data, uint8_t dlc, uint8_t & sequence, uint8_t & session_tag, uint8_t & fragment);

struct BoardStateCanConfig
{
  uint32_t control_canid = 0x320;
  uint32_t response_canid = 0x321;
  uint32_t data_canid = 0x322;
  double retry_ms = 100.0;
  double fragment_timeout_ms = 50.0;
  double stream_timeout_ms = 500.0;
  bool ids_confirmed = false;
  std::string can_interface;
};

BoardStateCanConfig load_board_state_can_config(const std::string & config_path);

// Pure session/fragmentation core. Time and frame transmission are injected so
// loss, timeout, reboot, and reordered-fragment cases are testable without CAN.
class BoardStateCanCore
{
public:
  using Clock = std::chrono::steady_clock;
  using Sender = std::function<bool(uint32_t, const uint8_t *)>;

  explicit BoardStateCanCore(const BoardStateCanConfig & config);

  void start(uint32_t session_id, Clock::time_point now);
  void stop();
  void on_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc, Clock::time_point now);
  void poll(Clock::time_point now);

  // Attempts at most one due handshake frame. Returns false when none is due.
  bool transmit(Clock::time_point now, const Sender & sender);

  const BoardStateCanConfig & config() const { return config_; }
  MinipcState state() const;
  Eigen::Quaterniond imu_at(Clock::time_point timestamp);

  struct Snapshot
  {
    bool running = false;
    bool streaming = false;
    bool state_valid = false;
    uint32_t session_id = 0;
    BoardStateCanColor color = BoardStateCanColor::unspecified;
    double state_age_ms = 0;
  };
  Snapshot snapshot(Clock::time_point now) const;

  struct Stats
  {
    uint64_t rx_ack = 0;
    uint64_t rx_data = 0;
    uint64_t rejected_malformed = 0;
    uint64_t rejected_session = 0;
    uint64_t rejected_version = 0;
    uint64_t rejected_color = 0;
    uint64_t rejected_fragment = 0;
    uint64_t partial_timeouts = 0;
    uint64_t stream_timeouts = 0;
    uint64_t tx_start = 0;
    uint64_t tx_final_ack = 0;
    uint64_t tx_fail = 0;
    uint64_t valid_packets = 0;
  };
  Stats stats() const;

private:
  enum class LinkState
  {
    idle,
    wait_response,
    wait_final_ack,
    streaming,
  };
  enum class FrameKind
  {
    none,
    start,
    final_ack,
  };
  struct ImuFrame
  {
    Eigen::Quaterniond q;
    Clock::time_point timestamp;
  };
  struct Partial
  {
    bool active = false;
    uint8_t sequence = 0;
    uint8_t mask = 0;
    std::array<uint8_t, sizeof(BoardState)> bytes{};
    Clock::time_point since{};
  };

  static uint32_t next_session(uint32_t session_id);
  void begin_session_locked(uint32_t session_id, Clock::time_point now);
  void clear_sample_locked();
  void poll_locked(Clock::time_point now);
  void on_ack_locked(const uint8_t * data, uint8_t dlc, Clock::time_point now);
  void on_fragment_locked(const uint8_t * data, uint8_t dlc, Clock::time_point now);
  void complete_sample_locked(Clock::time_point now);
  bool next_frame_locked(
    Clock::time_point now, uint32_t & can_id, uint8_t data[8], FrameKind & kind);
  void note_send_locked(bool ok, FrameKind kind, Clock::time_point now);

  BoardStateCanConfig config_;
  mutable std::mutex mutex_;
  LinkState link_state_ = LinkState::idle;
  bool running_ = false;
  uint32_t session_id_ = 0;
  BoardStateCanColor color_ = BoardStateCanColor::unspecified;

  bool start_pending_ = false;
  bool final_pending_ = false;
  Clock::time_point next_start_tx_{};
  Clock::time_point next_final_tx_{};
  Clock::time_point final_ack_since_{};
  Clock::time_point streaming_since_{};
  Clock::time_point last_complete_{};

  Partial partial_;
  bool has_completed_sequence_ = false;
  uint8_t completed_sequence_ = 0;
  MinipcState state_;
  bool has_state_ = false;
  std::deque<ImuFrame> imu_history_;
  Stats stats_;
};

// SocketCAN adapter. CBoard remains the sole socket owner; this class only
// subscribes to its two IDs and sends through CBoard::send_raw().
class BoardStateCanLink
{
public:
  BoardStateCanLink(const BoardStateCanConfig & config, CBoard & board);
  ~BoardStateCanLink();

  BoardStateCanLink(const BoardStateCanLink &) = delete;
  BoardStateCanLink & operator=(const BoardStateCanLink &) = delete;

  // Physical-bus use requires board_state_can_ids_confirmed=true. start() may
  // be called again after stop(); CBoard must outlive this object.
  void start();
  void stop();

  void on_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc);
  MinipcState state() const { return core_.state(); }
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp)
  {
    return core_.imu_at(timestamp);
  }
  BoardStateCanCore::Snapshot snapshot() const
  {
    return core_.snapshot(std::chrono::steady_clock::now());
  }
  BoardStateCanCore::Stats stats() const { return core_.stats(); }
  uint64_t valid_packet_count() const { return core_.stats().valid_packets; }
  uint64_t dropped_frame_count() const { return dropped_frames_.load(); }
  bool send(const Command & command) const { return board_.send(command); }

private:
  struct ReceivedFrame
  {
    uint32_t id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
    std::chrono::steady_clock::time_point time{};
  };

  void run();
  static uint32_t make_session_id();

  CBoard & board_;
  BoardStateCanCore core_;
  CBoard::FrameHandlerId handler_id_ = 0;
  std::mutex receive_mutex_;
  std::deque<ReceivedFrame> responses_;
  std::deque<ReceivedFrame> telemetry_;
  std::thread thread_;
  std::atomic<bool> quit_{false};
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> dropped_frames_{0};
};

}  // namespace io

#endif  // IO__BOARD_STATE_CAN_HPP
