#include "minipc.hpp"

#include <dirent.h>
#include <unistd.h>

#include <cstring>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{

namespace
{
constexpr char BY_ID_DIR[] = "/dev/serial/by-id";
constexpr char DEFAULT_PORT_PREFIX[] = "usb-STMicroelectronics_STM32_STLink_";
constexpr int DEFAULT_BAUD = 115200;
constexpr uint32_t READ_TIMEOUT_MS = 2;

template <typename T>
T read_or(const YAML::Node & yaml, const std::string & key, const T & fallback)
{
  if (yaml[key]) return yaml[key].as<T>();
  return fallback;
}

std::string find_by_id(const std::string & prefix)
{
  DIR * dir = opendir(BY_ID_DIR);
  if (dir == nullptr) return "";

  std::string found;
  while (const dirent * entry = readdir(dir)) {
    const std::string name = entry->d_name;
    if (name.rfind(prefix, 0) != 0) continue;  // not our adapter
    found = std::string(BY_ID_DIR) + "/" + name;
    break;
  }
  closedir(dir);
  return found;
}
}  // namespace

Minipc::Minipc(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  const auto configured_port = read_or<std::string>(yaml, "minipc_port", "");
  const auto prefix = read_or<std::string>(yaml, "minipc_port_prefix", DEFAULT_PORT_PREFIX);
  const auto baud = read_or<int>(yaml, "minipc_baud", DEFAULT_BAUD);

  const auto path = resolve_port(configured_port, prefix);
  if (path.empty()) {
    tools::logger()->error("[Minipc] no serial adapter matching '{}' under {}", prefix, BY_ID_DIR);
    exit(1);
  }

  try {
    serial_.setPort(path);
    serial_.setBaudrate(baud);
    // A short timeout makes read() return partial bursts promptly; the parser
    // is incremental, so chunk boundaries are irrelevant.
    serial::Timeout timeout(READ_TIMEOUT_MS, READ_TIMEOUT_MS, 0, READ_TIMEOUT_MS, 0);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Minipc] failed to open serial {}: {}", path, e.what());
    exit(1);
  }

  tools::logger()->info("[Minipc] listening on {} @ {}", path, baud);
  thread_ = std::thread(&Minipc::read_thread, this);
}

Minipc::~Minipc()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  try {
    serial_.close();
  } catch (...) {
  }
}

std::string Minipc::resolve_port(const std::string & port, const std::string & prefix)
{
  // An explicit, existing path wins; otherwise fall back to by-id discovery so
  // a different flasher's serial number does not break the link.
  if (!port.empty() && access(port.c_str(), F_OK) == 0) return port;
  return find_by_id(prefix);
}

uint32_t Minipc::valid_packet_count() const { return valid_packet_count_.load(); }

MinipcState Minipc::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

void Minipc::send(uint8_t cmd_id, const void * data)
{
  if (cmd_id >= TOTAL_CMD_ID) {
    tools::logger()->warn("[Minipc] refusing to send unknown cmd id {}", cmd_id);
    return;
  }

  uint8_t packet[MAX_PACKET_LENGTH];
  const uint8_t len = pack_packet(packet, data, cmd_id);
  if (len == 0) return;

  std::lock_guard<std::mutex> lock(write_mutex_);
  try {
    serial_.write(packet, len);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Minipc] failed to write serial: {}", e.what());
  }
}

void Minipc::send(const Command & command)
{
  GimbalData data{};
  data.rel_yaw = static_cast<float>(command.yaw);
  data.rel_pitch = static_cast<float>(command.pitch);
  data.mode = command.control ? 1 : 0;  // 1: move yoke, 0: search target
  data.debug_int = command.shoot ? 1 : 0;
  send(GIMBAL_CMD_ID, &data);
}

Eigen::Quaterniond Minipc::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (queue_.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.q;
  }

  ImuFrame ahead = queue_.pop();
  if (queue_.empty()) return ahead.q;

  while (true) {
    const ImuFrame behind = queue_.front();
    if (!(ahead.timestamp < timestamp)) return ahead.q;
    if (timestamp <= behind.timestamp) {
      const double span = tools::delta_time(ahead.timestamp, behind.timestamp);
      if (span <= 0) return behind.q;
      const double k = tools::delta_time(ahead.timestamp, timestamp) / span;
      return ahead.q.slerp(k, behind.q).normalized();
    }

    ahead = queue_.pop();
    if (queue_.empty()) return ahead.q;
  }
}

void Minipc::on_packet(void * context, const uint8_t * packet, uint8_t cmd_id)
{
  static_cast<Minipc *>(context)->handle_packet(packet, cmd_id);
}

void Minipc::handle_packet(const uint8_t * packet, uint8_t cmd_id)
{
  // The STM32 publishes exactly one packet type; anything else on the line is
  // not ours to interpret.
  if (cmd_id != BOARD_STATE_CMD_ID) return;

  BoardState data_msg;
  std::memcpy(&data_msg, packet + DATA_OFFSET, sizeof(data_msg));

  const auto timestamp = std::chrono::steady_clock::now();
  Eigen::Quaterniond q(data_msg.q[0], data_msg.q[1], data_msg.q[2], data_msg.q[3]);
  if (!(q.norm() > 1e-6)) {
    tools::logger()->warn("[Minipc] dropping degenerate quaternion");
    return;
  }
  q.normalize();

  std::lock_guard<std::mutex> lock(mutex_);
  state_.q = q;
  state_.yaw = data_msg.yaw;
  state_.pitch = data_msg.pitch;
  state_.bullet_speed = data_msg.bullet_speed;
  std::memcpy(state_.mode.data(), data_msg.mode, MODE_LEN);
  state_.timestamp = timestamp;
  queue_.push({q, timestamp});
  ++valid_packet_count_;
}

void Minipc::read_thread()
{
  tools::logger()->info("[Minipc] read_thread started.");
  uint8_t buffer[MAX_PACKET_LENGTH];
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Minipc] too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    size_t len = 0;
    try {
      len = serial_.read(buffer, sizeof(buffer));
    } catch (const std::exception & e) {
      ++error_count;
      continue;
    }

    if (len == 0) continue;  // read timeout, no bytes arrived

    error_count = 0;
    parser_.feed(buffer, len, &Minipc::on_packet, this);
  }

  tools::logger()->info("[Minipc] read_thread stopped.");
}

void Minipc::reconnect()
{
  const int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Minipc] reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
    } catch (...) {
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    try {
      serial_.open();
      parser_.reset();
      queue_.clear();
      tools::logger()->info("[Minipc] reconnected serial successfully.");
      return;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Minipc] reconnect failed: {}", e.what());
    }
  }
}

}  // namespace io
