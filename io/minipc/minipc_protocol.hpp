#ifndef IO__MINIPC__MINIPC_PROTOCOL_HPP
#define IO__MINIPC__MINIPC_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>

namespace io
{

// Wiring of the STM32 <-> Jetson UART protocol, documented in
// iRM_Vision_2023/docs/comm_protocol.md. The definition is owned by the
// firmware side: iRM_Embedded_2023/shared/libraries/minipc_protocol.h.
//
//   'ST' | seq (u16 LE) | data_len (u8) | cmd_id (u8) | data | crc8 | 'ED'
//
// HT_LEN (9) covers head (2) + seq (2) + data_len (1) + cmd_id (1) + crc8 (1)
// + tail (2), so the total packet length is data_len + HT_LEN. The checksum is
// CRC-8/MAXIM-DOW with an initial value of 0x00 over every byte preceding the
// crc8 byte -- note this differs from tools::get_crc8's default 0xff init.
//
// Both ends are little-endian (Cortex-M / aarch64), so the floats below are
// copied straight out of the packet without byte swapping.
enum MinipcCmdId : uint8_t
{
  GIMBAL_CMD_ID = 0x00,
  COLOR_CMD_ID = 0x01,
  CHASSIS_CMD_ID = 0x02,
  SELFCHECK_CMD_ID = 0x03,
  ARM_CMD_ID = 0x04,
  BOARD_STATE_CMD_ID = 0x05,
  TOTAL_CMD_ID
};

struct __attribute__((packed)) GimbalData
{
  float rel_yaw;
  float rel_pitch;
  uint8_t mode;  // 0: search target, 1: move yoke
  uint8_t debug_int;
};

struct __attribute__((packed)) ColorData
{
  uint8_t my_color;  // 0: red, 1: blue
};

struct __attribute__((packed)) ChassisData
{
  float vx;
  float vy;
  float vw;
};

struct __attribute__((packed)) SelfcheckData
{
  uint8_t mode;  // 0: flush, 1: echo, 2: id
  uint8_t debug_int;
};

struct __attribute__((packed)) ArmData
{
  float floats[6];
};

// The opaque mode byte in BoardState. The protocol assigns no meaning to it; a
// consumer that wants flags keeps its own bitfield and copies the byte into it.
inline constexpr uint8_t MODE_LEN = 1;

// STM32 -> Jetson state, sent as one packet so the Jetson gets an atomic
// snapshot (attitude and gimbal state from the same instant).
//
// yaw/pitch are the MEASURED motor angles with the gimbal calibration offset
// removed, i.e. the same frame as the angles the Jetson commands. They are not
// derivable from q: the encoder carries gear backlash and a per-robot offset.
struct __attribute__((packed)) BoardState
{
  float q[4];   // IMU attitude, w x y z
  float yaw;    // measured motor angle, offset-corrected
  float pitch;  // measured motor angle, offset-corrected
  float bullet_speed;
  uint8_t mode[MODE_LEN];  // opaque; semantics belong to the consumer
};

// These pin the wire format: any change here is a protocol change and must be
// mirrored in the firmware and in irm_jetson/python/comm/codec.py.
static_assert(sizeof(GimbalData) == 10, "gimbal data length is part of the wire protocol");
static_assert(sizeof(ColorData) == 1, "color data length is part of the wire protocol");
static_assert(sizeof(ChassisData) == 12, "chassis data length is part of the wire protocol");
static_assert(sizeof(SelfcheckData) == 2, "selfcheck data length is part of the wire protocol");
static_assert(sizeof(ArmData) == 24, "arm data length is part of the wire protocol");
static_assert(sizeof(BoardState) == 29, "board state length is part of the wire protocol");
static_assert(offsetof(BoardState, yaw) == 16, "board state.yaw offset changed");
static_assert(offsetof(BoardState, pitch) == 20, "board state.pitch offset changed");
static_assert(offsetof(BoardState, bullet_speed) == 24, "board state.bullet_speed offset changed");
static_assert(offsetof(BoardState, mode) == 28, "board state.mode offset changed");

enum
{
  HT_LEN = 9,
  MINIPC_DATA_LEN_MAX = sizeof(BoardState),
  MAX_PACKET_LENGTH = MINIPC_DATA_LEN_MAX + HT_LEN,
  MIN_PACKET_LENGTH = sizeof(ColorData) + HT_LEN,
  SEQNUM_OFFSET = 2,
  DATA_LENGTH_OFFSET = SEQNUM_OFFSET + 2,
  CMD_ID_OFFSET = DATA_LENGTH_OFFSET + 1,
  DATA_OFFSET = CMD_ID_OFFSET + 1
};

// Data-section length per cmd id, indexed by MinipcCmdId.
inline constexpr uint8_t CMD_TO_LEN[TOTAL_CMD_ID] = {sizeof(GimbalData),  sizeof(ColorData),
                                                     sizeof(ChassisData), sizeof(SelfcheckData),
                                                     sizeof(ArmData),     sizeof(BoardState)};

// Total packet length in bytes for cmd_id, or 0 when the id is unknown.
uint8_t packet_length(uint8_t cmd_id);

// Frames data (CMD_TO_LEN[cmd_id] bytes) into packet, which must hold at least
// packet_length(cmd_id) bytes. Returns the number of bytes written, or 0 when
// cmd_id is unknown.
uint8_t pack_packet(uint8_t * packet, const void * data, uint8_t cmd_id);

// Incremental parser for the byte stream. Feed arbitrary chunks; every complete
// and validated frame is reported through the callback. Garbage and corrupted
// frames are skipped one byte at a time, so the parser resynchronises instead
// of discarding everything behind a bad header.
class MinipcParser
{
public:
  using PacketHandler = void (*)(void * context, const uint8_t * packet, uint8_t cmd_id);

  void feed(const uint8_t * data, size_t len, PacketHandler handler, void * context);

  void reset();

private:
  void drop_first();
  void try_parse(PacketHandler handler, void * context);

  uint8_t buffer_[MAX_PACKET_LENGTH] = {0};
  size_t size_ = 0;
};

}  // namespace io

#endif  // IO__MINIPC__MINIPC_PROTOCOL_HPP
