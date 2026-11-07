#pragma once

#include <cstdint>

namespace robot {

// Wireless control packet, must match the transmitter byte-for-byte.
//
// Payload layout (12 bytes, little-endian int16_t):
//   [0]  joyMoveX  left stick X   (forward/backward)
//   [2]  joyMoveY  right stick Y  (strafe)
//   [4]  joyRotate right stick pot (rotation)
//   [6]  override
//   [8]  emergency
//   [10] gear      index 0..kGearCount-1
struct ControlPacket {
  int16_t joyMoveX;
  int16_t joyMoveY;
  int16_t joyRotate;
  int16_t override;
  int16_t emergency;
  int16_t gear;
};

static_assert(sizeof(ControlPacket) == 12, "ControlPacket must be 12 bytes");

}  // namespace robot
