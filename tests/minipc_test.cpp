// Offline self-test for the minipc UART protocol, plus an optional live run.
//
//   ./build/minipc_test                      # offline framing/crc checks only
//   ./build/minipc_test -c=configs/xxx.yaml  # also talk to the STM32 over UART
//
// The offline half needs no hardware, so it is safe to run on the laptop.
#include "io/minipc/minipc.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "io/minipc/minipc_protocol.hpp"
#include "tools/crc.hpp"
#include "tools/math_tools.hpp"

namespace
{

int failures = 0;

void check(bool ok, const std::string & what)
{
  std::cout << (ok ? "  ok   " : "  FAIL ") << what << std::endl;
  if (!ok) ++failures;
}

struct Collected
{
  std::vector<uint8_t> cmd_ids;
  std::vector<std::vector<uint8_t>> payloads;
};

void collect(void * context, const uint8_t * packet, uint8_t cmd_id)
{
  auto * out = static_cast<Collected *>(context);
  const uint8_t len = io::CMD_TO_LEN[cmd_id];
  out->cmd_ids.push_back(cmd_id);
  out->payloads.emplace_back(packet + io::DATA_OFFSET, packet + io::DATA_OFFSET + len);
}

std::vector<uint8_t> frame(uint8_t cmd_id, const void * data)
{
  uint8_t packet[io::MAX_PACKET_LENGTH] = {0};
  const uint8_t len = io::pack_packet(packet, data, cmd_id);
  return std::vector<uint8_t>(packet, packet + len);
}

void test_crc()
{
  std::cout << "[crc8]" << std::endl;
  const auto * check_str = reinterpret_cast<const uint8_t *>("123456789");
  // CRC-8/MAXIM-DOW has a published check value of 0xA1 at init 0x00; the
  // protocol depends on that init, not on tools::get_crc8's default.
  check(tools::get_crc8(check_str, 9, 0x00) == 0xA1, "init 0x00 matches CRC-8/MAXIM check value");
  check(tools::get_crc8(check_str, 9) == 0x0B, "default init 0xff differs (as expected)");
  check(
    tools::get_crc8(check_str, 9) != tools::get_crc8(check_str, 9, 0x00),
    "the two inits are not interchangeable");
}

void test_lengths()
{
  std::cout << "[lengths]" << std::endl;
  check(io::packet_length(io::GIMBAL_CMD_ID) == 19, "gimbal packet is 19 bytes");
  check(io::packet_length(io::COLOR_CMD_ID) == 10, "color packet is 10 bytes");
  check(io::packet_length(io::CHASSIS_CMD_ID) == 21, "chassis packet is 21 bytes");
  check(io::packet_length(io::SELFCHECK_CMD_ID) == 11, "selfcheck packet is 11 bytes");
  check(io::packet_length(io::ARM_CMD_ID) == 33, "arm packet is 33 bytes");
  check(io::packet_length(io::IMU_CMD_ID) == 25, "imu packet is 25 bytes");
  check(io::packet_length(io::TOTAL_CMD_ID) == 0, "unknown cmd id yields no length");
  check(io::packet_length(0xff) == 0, "out-of-range cmd id yields no length");
  check(io::packet_length(io::IMU_CMD_ID) <= io::MAX_PACKET_LENGTH, "imu fits MAX_PACKET_LENGTH");
  check(
    io::MAX_PACKET_LENGTH == io::packet_length(io::ARM_CMD_ID),
    "MAX_PACKET_LENGTH is the largest packet");
}

void test_framing()
{
  std::cout << "[framing]" << std::endl;
  const io::ImuData imu{1.0f, 0.0f, 0.0f, 0.0f};
  const auto f = frame(io::IMU_CMD_ID, &imu);

  check(f.size() == 25, "packed imu frame is 25 bytes");
  check(f[0] == 'S' && f[1] == 'T', "header is 'ST'");
  check(f[io::DATA_LENGTH_OFFSET] == sizeof(io::ImuData), "data_len matches the struct");
  check(f[io::CMD_ID_OFFSET] == io::IMU_CMD_ID, "cmd id survives packing");
  check(f[f.size() - 2] == 'E' && f[f.size() - 1] == 'D', "tail is 'ED'");
  check(
    f[f.size() - 3] == tools::get_crc8(f.data(), f.size() - 3, 0x00),
    "crc8 sits at total-3 and uses init 0x00");
}

void test_roundtrip()
{
  std::cout << "[roundtrip]" << std::endl;

  const io::GimbalData gimbal{1.5f, -2.5f, 1, 7};
  const io::ColorData color{1};
  const io::ChassisData chassis{3.5f, -4.5f, 0.25f};
  const io::SelfcheckData selfcheck{2, 129};
  const io::ArmData arm{{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f}};
  const io::ImuData imu{0.5f, 0.5f, 0.5f, 0.5f};

  struct Case
  {
    uint8_t cmd_id;
    const void * data;
  };
  const Case cases[] = {{io::GIMBAL_CMD_ID, &gimbal},   {io::COLOR_CMD_ID, &color},
                        {io::CHASSIS_CMD_ID, &chassis}, {io::SELFCHECK_CMD_ID, &selfcheck},
                        {io::ARM_CMD_ID, &arm},         {io::IMU_CMD_ID, &imu}};

  for (const auto & c : cases) {
    const auto f = frame(c.cmd_id, c.data);
    Collected out;
    io::MinipcParser parser;
    parser.feed(f.data(), f.size(), &collect, &out);

    const std::string name = std::to_string(c.cmd_id);
    check(out.cmd_ids.size() == 1 && out.cmd_ids[0] == c.cmd_id, "cmd " + name + " round-trips");
    check(
      out.payloads.size() == 1 &&
        std::memcmp(out.payloads[0].data(), c.data, io::CMD_TO_LEN[c.cmd_id]) == 0,
      "cmd " + name + " payload round-trips byte-for-byte");
  }
}

void test_quaternion()
{
  std::cout << "[quaternion]" << std::endl;
  // +90 degrees about z, in wxyz order.
  const float c = std::sqrt(0.5f);
  const io::ImuData imu{c, 0.0f, 0.0f, c};
  const auto f = frame(io::IMU_CMD_ID, &imu);

  io::ImuData decoded{};
  std::memcpy(&decoded, f.data() + io::DATA_OFFSET, sizeof(decoded));
  check(
    decoded.w == c && decoded.x == 0.0f && decoded.y == 0.0f && decoded.z == c,
    "wxyz payload round-trips bit-for-bit");

  const Eigen::Quaterniond q(decoded.w, decoded.x, decoded.y, decoded.z);
  const Eigen::Vector3d ypr = tools::eulers(q.normalized(), 2, 1, 0) * 57.3;
  // eulers() round-trips through a rotation matrix, so allow a little slack.
  check(
    std::abs(ypr[0] - 90.0) < 0.05,
    "wxyz payload decodes to +90 deg yaw (got " + std::to_string(ypr[0]) + ")");
  check(
    std::abs(ypr[1]) < 0.05 && std::abs(ypr[2]) < 0.05, "pitch and roll stay at zero (got " +
                                                          std::to_string(ypr[1]) + ", " +
                                                          std::to_string(ypr[2]) + ")");
}

void test_resync()
{
  std::cout << "[resync]" << std::endl;
  const io::ImuData imu{1.0f, 0.0f, 0.0f, 0.0f};
  const auto good = frame(io::IMU_CMD_ID, &imu);

  // garbage ahead of a good frame
  {
    std::vector<uint8_t> stream = {'x', 'S', 'Z', 0x00, 0x11, 0xff};
    stream.insert(stream.end(), good.begin(), good.end());
    Collected out;
    io::MinipcParser parser;
    parser.feed(stream.data(), stream.size(), &collect, &out);
    check(out.cmd_ids.size() == 1, "resyncs after leading garbage");
  }

  // corrupted checksum
  {
    auto bad = good;
    bad[bad.size() - 3] ^= 0xff;
    Collected out;
    io::MinipcParser parser;
    parser.feed(bad.data(), bad.size(), &collect, &out);
    check(out.cmd_ids.empty(), "corrupted crc is rejected");
  }

  // data_len disagreeing with the cmd id
  {
    auto bad = good;
    bad[io::DATA_LENGTH_OFFSET] = 11;  // imu really carries 16
    Collected out;
    io::MinipcParser parser;
    parser.feed(bad.data(), bad.size(), &collect, &out);
    check(out.cmd_ids.empty(), "lying data_len is rejected");
  }

  // unknown cmd id, followed by a good frame
  {
    auto bad = good;
    bad[io::CMD_ID_OFFSET] = 0x7f;
    std::vector<uint8_t> stream = bad;
    stream.insert(stream.end(), good.begin(), good.end());
    Collected out;
    io::MinipcParser parser;
    parser.feed(stream.data(), stream.size(), &collect, &out);
    check(
      out.cmd_ids.size() == 1 && out.cmd_ids[0] == io::IMU_CMD_ID,
      "unknown cmd id is skipped and the next frame still parses");
  }

  // split delivery
  {
    Collected out;
    io::MinipcParser parser;
    parser.feed(good.data(), good.size() - 1, &collect, &out);
    check(out.cmd_ids.empty(), "incomplete frame is buffered, not emitted");
    const uint8_t last = good.back();
    parser.feed(&last, 1, &collect, &out);
    check(out.cmd_ids.size() == 1, "frame completes when the final byte arrives");
  }

  {
    Collected out;
    io::MinipcParser parser;
    for (const uint8_t byte : good) parser.feed(&byte, 1, &collect, &out);
    check(out.cmd_ids.size() == 1, "parses when delivered one byte at a time");
  }

  // back-to-back frames in one chunk
  {
    std::vector<uint8_t> stream = good;
    stream.insert(stream.end(), good.begin(), good.end());
    Collected out;
    io::MinipcParser parser;
    parser.feed(stream.data(), stream.size(), &collect, &out);
    check(out.cmd_ids.size() == 2, "parses back-to-back frames");
  }
}

int run_live(const std::string & config_path)
{
  std::cout << "[live] opening " << config_path << std::endl;
  io::Minipc minipc(config_path);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  uint32_t last_count = 0;

  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const auto now = std::chrono::steady_clock::now();
    const Eigen::Vector3d ypr = tools::eulers(minipc.imu_at(now), 2, 1, 0) * 57.3;
    const uint32_t count = minipc.valid_packet_count();

    std::cout << "cmd=0x" << std::hex << static_cast<int>(minipc.cmd_id()) << std::dec
              << " packets=" << count << " (+" << (count - last_count) << ")"
              << " yaw=" << ypr[0] << " pitch=" << ypr[1] << " roll=" << ypr[2] << std::endl;
    last_count = count;
  }
  return 0;
}

}  // namespace

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ? | | 输出命令行参数说明}"
    "{c config-path | | yaml配置文件路径；给出则连实机，否则只跑离线自检 }";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto config_path = cli.get<std::string>("config-path");

  std::cout << "== minipc offline self-test ==" << std::endl;
  test_crc();
  test_lengths();
  test_framing();
  test_roundtrip();
  test_quaternion();
  test_resync();

  if (failures == 0) {
    std::cout << "all checks passed" << std::endl;
  } else {
    std::cout << "FAILURES: " << failures << std::endl;
  }

  if (config_path.empty()) return failures == 0 ? 0 : 1;

  if (failures != 0) {
    std::cout << "skipping the live run because the offline self-test failed" << std::endl;
    return 1;
  }
  return run_live(config_path);
}
