#ifndef IO__MINIPC__MINIPC_HPP
#define IO__MINIPC__MINIPC_HPP

#include <Eigen/Geometry>
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

// Latest state reported by the STM32. Mirrors the firmware's status_data_t:
// every packet updates only the fields it carries, so a caller must check
// cmd_id() before trusting a group of fields (SELFCHECK and GIMBAL both write
// mode/debug_int, for instance).
struct MinipcStatus
{
  uint8_t my_color = 0;  // 0: red, 1: blue
  float rel_yaw = 0;
  float rel_pitch = 0;
  uint8_t mode = 0;
  uint8_t debug_int = 0;
  float vx = 0;
  float vy = 0;
  float vw = 0;
  float floats[6] = {0, 0, 0, 0, 0, 0};
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

// UART link to the STM32 control board, reachable through the flasher's
// USB-serial adapter (ST-Link VCP). The port is discovered under
// /dev/serial/by-id/ because ttyACM/ttyUSB numbering is not stable. See
// iRM_Vision_2023/docs/comm_protocol.md for the packet format.
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

  // Jetson -> STM32. command.control picks move-yoke vs search-target mode and
  // command.shoot is carried in the debug byte.
  void send(const Command & command);

  // Jetson -> STM32 raw packet. data must be CMD_TO_LEN[cmd_id] bytes.
  void send(uint8_t cmd_id, const void * data);

  uint8_t cmd_id() const;
  uint16_t seqnum() const;
  uint32_t valid_packet_count() const;
  MinipcStatus status() const;

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
  MinipcStatus status_;
  uint8_t cmd_id_ = 0;
  uint16_t seqnum_ = 0;
  std::atomic<uint32_t> valid_packet_count_{0};

  // PopWhenFull keeps the freshest samples if the consumer stalls.
  tools::ThreadSafeQueue<ImuFrame, /*PopWhenFull=*/true> queue_{1000};
};

}  // namespace io

#endif  // IO__MINIPC__MINIPC_HPP
