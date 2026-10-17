#include "minipc_protocol.hpp"

#include <cstring>

#include "tools/crc.hpp"

namespace io
{

namespace
{
// The protocol fixes crc8's initial value at 0x00 (CRC-8/MAXIM-DOW), which is
// deliberately not tools::get_crc8's default of 0xff.
constexpr uint8_t MINIPC_CRC8_INIT = 0x00;
}  // namespace

uint8_t packet_length(uint8_t cmd_id)
{
  if (cmd_id >= TOTAL_CMD_ID) return 0;
  return static_cast<uint8_t>(CMD_TO_LEN[cmd_id] + HT_LEN);
}

uint8_t pack_packet(uint8_t * packet, const void * data, uint8_t cmd_id)
{
  if (cmd_id >= TOTAL_CMD_ID) return 0;

  const uint8_t data_len = CMD_TO_LEN[cmd_id];
  const uint8_t total = static_cast<uint8_t>(data_len + HT_LEN);
  const uint8_t crc_offset = static_cast<uint8_t>(DATA_OFFSET + data_len);

  packet[0] = 'S';
  packet[1] = 'T';
  // The firmware writes a dummy seqnum and never reads it back.
  packet[SEQNUM_OFFSET] = 0;
  packet[SEQNUM_OFFSET + 1] = 0;
  packet[DATA_LENGTH_OFFSET] = data_len;
  packet[CMD_ID_OFFSET] = cmd_id;
  std::memcpy(packet + DATA_OFFSET, data, data_len);
  packet[crc_offset] = tools::get_crc8(packet, crc_offset, MINIPC_CRC8_INIT);
  packet[crc_offset + 1] = 'E';
  packet[crc_offset + 2] = 'D';

  return total;
}

void MinipcParser::reset() { size_ = 0; }

void MinipcParser::drop_first()
{
  if (size_ == 0) return;
  std::memmove(buffer_, buffer_ + 1, size_ - 1);
  --size_;
}

void MinipcParser::feed(const uint8_t * data, size_t len, PacketHandler handler, void * context)
{
  for (size_t i = 0; i < len; ++i) {
    // try_parse drains or shrinks the buffer on every byte, so this is only a
    // guard against a pathological caller.
    if (size_ >= MAX_PACKET_LENGTH) size_ = 0;
    buffer_[size_++] = data[i];
    try_parse(handler, context);
  }
}

void MinipcParser::try_parse(PacketHandler handler, void * context)
{
  while (size_ >= 2) {
    if (buffer_[0] != 'S' || buffer_[1] != 'T') {
      drop_first();
      continue;
    }

    if (size_ < HT_LEN) return;  // header fields still incomplete

    const uint8_t cmd_id = buffer_[CMD_ID_OFFSET];
    if (cmd_id >= TOTAL_CMD_ID) {
      drop_first();
      continue;
    }

    // The firmware does not validate data_len on receive (it trusts cmd_id);
    // we do, so a lying length can never make us read past the frame.
    if (buffer_[DATA_LENGTH_OFFSET] != CMD_TO_LEN[cmd_id]) {
      drop_first();
      continue;
    }

    const size_t total = static_cast<size_t>(CMD_TO_LEN[cmd_id]) + HT_LEN;
    if (size_ < total) return;  // frame still incomplete

    const size_t crc_offset = total - 3;
    if (
      buffer_[total - 2] != 'E' || buffer_[total - 1] != 'D' ||
      tools::get_crc8(buffer_, static_cast<uint16_t>(crc_offset), MINIPC_CRC8_INIT) !=
        buffer_[crc_offset]) {
      drop_first();
      continue;
    }

    if (handler != nullptr) handler(context, buffer_, cmd_id);
    size_ = 0;
    return;
  }
}

}  // namespace io
