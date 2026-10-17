#ifndef IO__MINIPC__MINIPC_HPP
#define IO__MINIPC__MINIPC_HPP

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "io/minipc/minipc_protocol.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

// Decoded board_state packet: the single message the Jetson receives over this
// link. yaw/pitch are measured motor angles (offset-corrected), not attitude,
// so they are not derivable from q. mode is opaque to the protocol.
struct MinipcState
{
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  float yaw = 0;
  float pitch = 0;
  float bullet_speed = 0;
  std::array<uint8_t, MODE_LEN> mode{};
  std::chrono::steady_clock::time_point timestamp;
};

// UART link to the STM32 control board, reachable through the flasher's
// USB-serial adapter (ST-Link VCP). The port is discovered under
// /dev/serial/by-id/ because ttyACM/ttyUSB numbering is not stable.
class Minipc
{
public:
  explicit Minipc(const std::string & config_path);
  ~Minipc();

  Minipc(const Minipc &) = delete;
  Minipc & operator=(const Minipc &) = delete;

  // STM32 IMU attitude interpolated to timestamp, matching CBoard::imu_at's
  // contract. Falls back to the latest sample when nothing newer is buffered.
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  // Latest decoded board state. Meaningful once at least one packet arrived.
  MinipcState state() const;

  // Jetson -> STM32. command.control picks move-yoke vs search-target mode and
  // command.shoot is carried in the debug byte.
  void send(const Command & command);

  // Jetson -> STM32 raw packet. data must be CMD_TO_LEN[cmd_id] bytes.
  void send(uint8_t cmd_id, const void * data);

  uint32_t valid_packet_count() const;

private:
  struct ImuFrame
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  static void on_packet(void * context, const uint8_t * packet, uint8_t cmd_id);

  void read_thread();
  void handle_packet(const uint8_t * packet, uint8_t cmd_id);
  void reconnect();
  static std::string resolve_port(const std::string & port, const std::string & prefix);

  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_{false};
  mutable std::mutex mutex_;
  std::mutex write_mutex_;

  MinipcParser parser_;
  MinipcState state_;
  std::atomic<uint32_t> valid_packet_count_{0};

  // PopWhenFull keeps the freshest samples if the consumer stalls.
  tools::ThreadSafeQueue<ImuFrame, /*PopWhenFull=*/true> queue_{1000};
};

}  // namespace io

#endif  // IO__MINIPC__MINIPC_HPP
