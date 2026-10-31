// Offline tests for documentation/board_state_can_protocol.md, plus an
// optional five-second live run: ./build/board_state_can_test -c=configs/board_state_can_vcan.yaml
#include "io/board_state_can.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "tools/math_tools.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

int failures = 0;

void check(bool ok, const std::string & what)
{
  std::cout << (ok ? "  ok   " : "  FAIL ") << what << std::endl;
  if (!ok) ++failures;
}

io::BoardStateCanConfig test_config()
{
  io::BoardStateCanConfig config;
  config.ids_confirmed = true;
  return config;
}

io::BoardStateCanPayload make_state()
{
  const float c = std::sqrt(0.5f);
  io::BoardStateCanPayload state{};
  state.q[0] = c;
  state.q[3] = c;
  state.bullet_speed = 15.0f;
  state.mode[0] = 0xAA;
  return state;
}

struct Sent
{
  uint32_t id = 0;
  std::vector<uint8_t> data;
};

Sent transmit(io::BoardStateCanCore & core, Clock::time_point now, bool ok = true)
{
  Sent sent;
  core.transmit(now, [&](uint32_t id, const uint8_t * data) {
    sent.id = id;
    sent.data.assign(data, data + 8);
    return ok;
  });
  return sent;
}

void send_ack(
  io::BoardStateCanCore & core, uint32_t session, io::BoardStateCanStatus status,
  io::BoardStateCanColor color, Clock::time_point now)
{
  uint8_t data[8]{};
  io::encode_board_state_can_ack(session, status, color, data);
  core.on_frame(core.config().response_canid, data, 8, now);
}

void complete_handshake(io::BoardStateCanCore & core, uint32_t session, Clock::time_point now)
{
  core.start(session, now);
  transmit(core, now);
  send_ack(core, session, io::BoardStateCanStatus::ready, io::BoardStateCanColor::blue, now);
  transmit(core, now);
}

void send_fragment(
  io::BoardStateCanCore & core, const io::BoardStateCanPayload & state, uint32_t session,
  uint8_t sequence, uint8_t fragment, Clock::time_point now)
{
  uint8_t data[8]{};
  check(
    io::encode_board_state_can_fragment(state, session, sequence, fragment, data),
    "encode fragment " + std::to_string(fragment));
  core.on_frame(core.config().data_canid, data, 8, now);
}

void send_sample(
  io::BoardStateCanCore & core, const io::BoardStateCanPayload & state, uint32_t session,
  uint8_t sequence, Clock::time_point now, const std::vector<uint8_t> & order = {0, 1, 2, 3})
{
  for (const uint8_t fragment : order) send_fragment(core, state, session, sequence, fragment, now);
}

void test_handshake_codec()
{
  std::cout << "[handshake codec]" << std::endl;
  uint8_t data[8]{};
  io::encode_board_state_can_start(0x12345678, data);
  const uint8_t expected_start[] = {0x01, 0x01, 0x12, 0x34, 0x56, 0x78, 0x00, 0x00};
  check(std::memcmp(data, expected_start, 8) == 0, "START golden vector");

  uint8_t version = 0;
  uint32_t session = 0;
  check(
    io::decode_board_state_can_start(data, 8, version, session) && version == 1 &&
      session == 0x12345678,
    "START decodes");
  data[7] = 1;
  check(
    !io::decode_board_state_can_start(data, 8, version, session), "START reserved byte checked");

  io::encode_board_state_can_ack(
    0x12345678, io::BoardStateCanStatus::ready, io::BoardStateCanColor::blue, data);
  const uint8_t expected_ack[] = {0x02, 0x01, 0x12, 0x34, 0x56, 0x78, 0x00, 0x02};
  check(std::memcmp(data, expected_ack, 8) == 0, "READY+blue golden vector");
  io::BoardStateCanAck ack;
  check(
    io::decode_board_state_can_ack(data, 8, ack) && ack.session_id == 0x12345678 &&
      ack.status == io::BoardStateCanStatus::ready && ack.color == io::BoardStateCanColor::blue,
    "ACK+color decodes");
  check(!io::decode_board_state_can_ack(data, 7, ack), "short ACK rejected");

  io::encode_board_state_can_final_ack(0x12345678, data);
  const uint8_t expected_final[] = {0x03, 0x01, 0x12, 0x34, 0x56, 0x78, 0x00, 0x00};
  check(std::memcmp(data, expected_final, 8) == 0, "final ACK golden vector");
}

void test_fragment_codec()
{
  std::cout << "[fragment codec]" << std::endl;
  const auto state = make_state();
  std::vector<uint8_t> image(sizeof(io::BoardStateCanPayload));
  for (uint8_t fragment = 0; fragment < 4; ++fragment) {
    uint8_t data[8]{};
    check(
      io::encode_board_state_can_fragment(state, 0x12345678, 7, fragment, data),
      "fragment encodes");
    check(data[0] == ((fragment << 5) | 7) && data[1] == 0x78, "fragment metadata");
    uint8_t sequence = 0;
    uint8_t tag = 0;
    uint8_t decoded_fragment = 0;
    check(
      io::decode_board_state_can_fragment_header(data, 8, sequence, tag, decoded_fragment) &&
        sequence == 7 && tag == 0x78 && decoded_fragment == fragment,
      "fragment header decodes");
    const size_t length = fragment == 3 ? 3 : 6;
    std::memcpy(image.data() + fragment * 6, data + 2, length);
    if (fragment == 3) check(data[5] == 0 && data[6] == 0 && data[7] == 0, "tail padding is zero");
  }
  check(
    std::memcmp(image.data(), &state, sizeof(state)) == 0,
    "four fragments reproduce all 21 payload bytes");

  uint8_t data[8]{};
  check(!io::encode_board_state_can_fragment(state, 1, 0, 4, data), "fifth fragment rejected");
  for (size_t padding = 5; padding < 8; ++padding) {
    io::encode_board_state_can_fragment(state, 1, 0, 3, data);
    data[padding] = 1;
    uint8_t sequence = 0;
    uint8_t tag = 0;
    uint8_t fragment = 0;
    check(
      !io::decode_board_state_can_fragment_header(data, 8, sequence, tag, fragment),
      "nonzero tail padding rejected at byte " + std::to_string(padding));
  }
}

void test_embedded_golden_sample()
{
  std::cout << "[Embedded 21-byte golden sample]" << std::endl;
  io::BoardStateCanCore core(test_config());
  const auto t0 = Clock::time_point{};
  complete_handshake(core, 0x12345678, t0);
  // Independent firmware wire vectors: q=(1,0,0,0), speed=15, mode=0xAA.
  // Do not generate input with the Jetson encoder: catch symmetric codec bugs.
  const uint8_t frames[4][8] = {
    {0x07, 0x78, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00},
    {0x27, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x47, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x67, 0x78, 0x70, 0x41, 0xAA, 0x00, 0x00, 0x00},
  };
  io::BoardStateCanPayload expected{};
  expected.q[0] = 1;
  expected.bullet_speed = 15;
  expected.mode[0] = 0xAA;
  for (uint8_t f = 0; f < 4; ++f) {
    uint8_t encoded[8]{};
    io::encode_board_state_can_fragment(expected, 0x12345678, 7, f, encoded);
    check(std::memcmp(encoded, frames[f], 8) == 0, "Embedded fragment golden vector");
    core.on_frame(core.config().data_canid, frames[f], 8, t0 + 1ms);
    check(core.stats().valid_packets == (f == 3 ? 1U : 0U), "publish only on fourth fragment");
  }
  const auto state = core.state();
  check(
    state.q.w() == 1 && state.q.vec().norm() == 0 && state.bullet_speed == 15 &&
      state.mode[0] == 0xAA && std::isnan(state.yaw) && std::isnan(state.pitch),
    "Embedded payload decoded; motor angles explicitly unavailable");
  check(sizeof(io::BoardState) == 29, "UART absolute-pose wire layout unchanged");
}

void test_handshake_recovery()
{
  std::cout << "[handshake recovery]" << std::endl;
  io::BoardStateCanCore core(test_config());
  const auto t0 = Clock::time_point{};
  core.start(0x12345678, t0);
  auto sent = transmit(core, t0);
  check(sent.id == core.config().control_canid && sent.data.size() == 8, "START sent immediately");
  check(
    !core.transmit(t0, [](uint32_t, const uint8_t *) { return true; }), "START is rate-limited");

  send_ack(
    core, 0x12345678, io::BoardStateCanStatus::not_ready, io::BoardStateCanColor::blue, t0 + 1ms);
  check(
    !core.transmit(t0 + 50ms, [](uint32_t, const uint8_t *) { return true; }),
    "NOT_READY waits for retry");
  check(
    core.transmit(t0 + 100ms, [](uint32_t, const uint8_t *) { return true; }),
    "START retries after 100 ms");

  send_ack(
    core, 0x12345679, io::BoardStateCanStatus::ready, io::BoardStateCanColor::blue, t0 + 100ms);
  check(
    !core.transmit(t0 + 100ms, [](uint32_t, const uint8_t *) { return true; }),
    "wrong session cannot trigger final ACK");
  send_ack(
    core, 0x12345678, io::BoardStateCanStatus::ready, io::BoardStateCanColor::unspecified,
    t0 + 100ms);
  check(
    !core.transmit(t0 + 100ms, [](uint32_t, const uint8_t *) { return true; }),
    "READY without color rejected");

  send_ack(
    core, 0x12345678, io::BoardStateCanStatus::ready, io::BoardStateCanColor::blue, t0 + 100ms);
  sent = transmit(core, t0 + 100ms, false);
  check(
    sent.data.size() == 8 && sent.data[0] == 0x03 && !core.snapshot(t0).streaming,
    "failed final ACK does not start streaming");
  send_ack(
    core, 0x12345678, io::BoardStateCanStatus::ready, io::BoardStateCanColor::blue, t0 + 200ms);
  sent = transmit(core, t0 + 200ms);
  check(
    sent.data.size() == 8 && sent.data[0] == 0x03 && core.snapshot(t0 + 200ms).streaming,
    "final ACK retry completes handshake");

  send_sample(core, make_state(), 0x12345678, 0, t0 + 201ms);
  check(
    core.stats().valid_packets == 1 && core.snapshot(t0 + 201ms).state_valid,
    "telemetry accepted after handshake");
  send_ack(
    core, 0x12345678, io::BoardStateCanStatus::ready, io::BoardStateCanColor::blue, t0 + 202ms);
  sent = transmit(core, t0 + 202ms);
  check(
    sent.data.size() == 8 && sent.data[0] == 0x03 && core.stats().valid_packets == 1,
    "duplicate READY only repeats final ACK");

  core.poll(t0 + 700ms);
  check(
    core.snapshot(t0 + 700ms).session_id == 0x12345678 && core.snapshot(t0 + 700ms).streaming,
    "stream remains valid at 499 ms");
  core.poll(t0 + 702ms);
  check(
    core.snapshot(t0 + 702ms).session_id == 0x12345679 && !core.snapshot(t0 + 702ms).streaming &&
      !core.snapshot(t0 + 702ms).state_valid && core.stats().stream_timeouts == 1,
    "500 ms silence starts a fresh session");
  sent = transmit(core, t0 + 702ms);
  check(sent.data.size() == 8 && sent.data[0] == 0x01, "timeout recovery sends START");
}

void test_final_ack_timeout()
{
  std::cout << "[final ACK timeout]" << std::endl;
  io::BoardStateCanCore core(test_config());
  const auto t0 = Clock::time_point{};
  core.start(0x10203040, t0);
  transmit(core, t0);
  send_ack(core, 0x10203040, io::BoardStateCanStatus::ready, io::BoardStateCanColor::blue, t0);
  transmit(core, t0, false);
  core.poll(t0 + 499ms);
  check(
    core.snapshot(t0 + 499ms).session_id == 0x10203040,
    "final ACK retry keeps session before its deadline");
  core.poll(t0 + 500ms);
  check(
    core.snapshot(t0 + 500ms).session_id == 0x10203041 && !core.snapshot(t0 + 500ms).streaming &&
      core.stats().stream_timeouts == 1,
    "failed final ACK phase restarts the session at 500 ms");
  const auto sent = transmit(core, t0 + 500ms);
  check(
    sent.data.size() == 8 && sent.data[0] == 0x01,
    "final ACK timeout sends START instead of retrying an orphaned ACK");
}

void test_reassembly()
{
  std::cout << "[reassembly]" << std::endl;
  io::BoardStateCanCore core(test_config());
  const auto t0 = Clock::time_point{};
  const uint32_t session = 0x11223344;
  complete_handshake(core, session, t0);
  const auto state = make_state();

  send_sample(core, state, session, 3, t0 + 1ms, {2, 0, 1});
  check(core.stats().valid_packets == 0, "missing fragment never publishes");
  send_fragment(core, state, session, 3, 3, t0 + 2ms);
  check(core.stats().valid_packets == 1, "out-of-order fragments complete one sample");
  const auto decoded = core.state();
  check(
    std::isnan(decoded.yaw) && std::isnan(decoded.pitch) &&
      decoded.bullet_speed == state.bullet_speed && decoded.mode[0] == state.mode[0],
    "decoded state fields match");
  send_sample(core, state, session, 3, t0 + 3ms);
  check(core.stats().valid_packets == 1, "completed sample cannot be duplicated");

  send_fragment(core, state, session, 4, 0, t0 + 4ms);
  auto changed = state;
  changed.q[0] = 0.1f;  // fragment 0 contains q[0]
  send_fragment(core, changed, session, 4, 0, t0 + 5ms);
  send_sample(core, state, session, 4, t0 + 6ms, {1, 2, 3});
  check(core.stats().valid_packets == 1, "conflicting duplicate invalidates partial sample");
  core.poll(t0 + 60ms);
  check(core.stats().partial_timeouts == 1, "partial sample expires at 50 ms");

  send_sample(core, state, session ^ 0xFF, 5, t0 + 61ms);
  check(core.stats().valid_packets == 1, "old session tag rejected");
  send_sample(core, state, session, 5, t0 + 62ms);
  check(core.stats().valid_packets == 2, "new sample after errors still works");
  send_sample(core, state, session, 4, t0 + 63ms);
  check(core.stats().valid_packets == 2, "older sequence rejected");

  io::BoardStateCanCore wrap_core(test_config());
  complete_handshake(wrap_core, session, t0);
  send_sample(wrap_core, state, session, 31, t0 + 64ms);
  send_sample(wrap_core, state, session, 0, t0 + 65ms);
  check(wrap_core.stats().valid_packets == 2, "sequence wraps 31 to zero");

  auto invalid = state;
  invalid.q[0] = std::numeric_limits<float>::quiet_NaN();
  send_sample(core, invalid, session, 6, t0 + 66ms);
  check(core.stats().valid_packets == 2, "NaN quaternion rejected");

  invalid = state;
  invalid.bullet_speed = std::numeric_limits<float>::infinity();
  send_sample(core, invalid, session, 7, t0 + 67ms);
  check(core.stats().valid_packets == 2, "nonfinite bullet speed rejected");

  uint8_t bad[8]{};
  io::encode_board_state_can_fragment(state, session, 8, 0, bad);
  bad[0] = static_cast<uint8_t>(4 << 5);
  const auto rejected_before = core.stats().rejected_fragment;
  core.on_frame(core.config().data_canid, bad, 8, t0 + 68ms);
  check(core.stats().rejected_fragment == rejected_before + 1, "old fifth fragment rejected");
  check(core.stats().valid_packets == 2, "invalid fragment index rejected");
}

void test_imu_interpolation()
{
  std::cout << "[IMU interpolation]" << std::endl;
  io::BoardStateCanCore core(test_config());
  const auto t0 = Clock::time_point{};
  const uint32_t session = 55;
  complete_handshake(core, session, t0);

  io::BoardStateCanPayload first{};
  first.q[0] = 1;
  first.bullet_speed = 15;
  io::BoardStateCanPayload second = make_state();
  send_sample(core, first, session, 0, t0);
  send_sample(core, second, session, 1, t0 + 10ms);

  const Eigen::Quaterniond q_first(first.q[0], first.q[1], first.q[2], first.q[3]);
  const Eigen::Quaterniond q_second(second.q[0], second.q[1], second.q[2], second.q[3]);
  const auto expected = q_first.slerp(0.5, q_second).normalized();
  const auto actual = core.imu_at(t0 + 5ms);
  check(std::abs(actual.dot(expected)) > 1 - 1e-6, "IMU interpolates at requested time");
  check(core.imu_at(t0 + 20ms).dot(q_second) > 1 - 1e-6, "future lookup returns latest sample");
}

void test_config_validation()
{
  std::cout << "[config validation]" << std::endl;
  bool threw = false;
  try {
    auto config = test_config();
    config.data_canid = config.control_canid;
    io::BoardStateCanCore core(config);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw, "duplicate CAN IDs rejected");

  threw = false;
  try {
    auto config = test_config();
    config.data_canid = 0x200;
    io::BoardStateCanCore core(config);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw, "motor CAN ID rejected");

  threw = false;
  try {
    auto config = test_config();
    config.data_canid = 0x310;
    io::BoardStateCanCore core(config);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw, "vision CAN ID rejected");

  threw = false;
  try {
    auto config = test_config();
    config.fragment_timeout_ms = config.stream_timeout_ms;
    io::BoardStateCanCore core(config);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw, "invalid timeout relationship rejected");
}

int run_live(const std::string & config_path)
{
  std::cout << "[live] opening " << config_path << std::endl;
  auto config = io::load_board_state_can_config(config_path);
  io::CBoard board(config_path);  // config uses cboard_wait_imu=false; this link owns IMU history.
  io::BoardStateCanLink link(config, board);
  link.start();

  const auto deadline = Clock::now() + 5s;
  uint64_t last_count = 0;
  while (Clock::now() < deadline) {
    std::this_thread::sleep_for(500ms);
    const auto snapshot = link.snapshot();
    const auto state = link.state();
    const auto stats = link.stats();
    const auto ypr = tools::eulers(link.imu_at(Clock::now()), 2, 1, 0) * 57.3;
    std::cout << "session=0x" << std::hex << snapshot.session_id << std::dec
              << " streaming=" << snapshot.streaming << " valid=" << snapshot.state_valid
              << " color=" << io::to_string(snapshot.color) << " packets=" << stats.valid_packets
              << " (+" << (stats.valid_packets - last_count) << ")"
              << " imu_yaw=" << ypr[0] << " bullet_speed=" << state.bullet_speed
              << " mode=" << int(state.mode[0]) << " rejected=" << stats.rejected_fragment
              << " dropped=" << link.dropped_frame_count() << std::endl;
    last_count = stats.valid_packets;
  }
  const auto final_snapshot = link.snapshot();
  link.stop();
  if (last_count == 0 || !final_snapshot.state_valid) {
    std::cerr << "live verification failed: no complete board-state samples or stale final stream"
              << std::endl;
    return 2;
  }
  std::cout << "live verification passed" << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ? | | print usage}"
    "{c config-path | | YAML path; supplied also runs the live CAN check }";
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  std::cout << "== board-state CAN offline tests ==" << std::endl;
  test_handshake_codec();
  test_fragment_codec();
  test_embedded_golden_sample();
  test_handshake_recovery();
  test_final_ack_timeout();
  test_reassembly();
  test_imu_interpolation();
  test_config_validation();
  if (failures == 0) {
    std::cout << "all checks passed" << std::endl;
  } else {
    std::cout << "FAILURES: " << failures << std::endl;
    return 1;
  }

  const auto config_path = cli.get<std::string>("config-path");
  if (config_path.empty()) return 0;
  try {
    return run_live(config_path);
  } catch (const std::exception & e) {
    std::cerr << "live verification error: " << e.what() << std::endl;
    return 2;
  }
}
