#pragma once

#include <cstdint>

namespace robot::pins {

// -----------------------------------------------------------------------------
// Motor driver direction inputs (A/B per wheel)
// -----------------------------------------------------------------------------
constexpr uint8_t kLeftFrontA = 4;
constexpr uint8_t kLeftFrontB = 5;

constexpr uint8_t kRightFrontA = 8;
constexpr uint8_t kRightFrontB = 9;

constexpr uint8_t kLeftBackA = 17;
constexpr uint8_t kLeftBackB = 18;

constexpr uint8_t kRightBackA = 7;
constexpr uint8_t kRightBackB = 6;

// -----------------------------------------------------------------------------
// Quadrature encoders (A = interrupt, B = direction)
// -----------------------------------------------------------------------------
constexpr uint8_t kLeftFrontEncoderA = 41;
constexpr uint8_t kLeftFrontEncoderB = 42;

constexpr uint8_t kRightFrontEncoderA = 39;
constexpr uint8_t kRightFrontEncoderB = 40;

constexpr uint8_t kLeftBackEncoderA = 44;
constexpr uint8_t kLeftBackEncoderB = 43;

constexpr uint8_t kRightBackEncoderA = 1;
constexpr uint8_t kRightBackEncoderB = 2;

// -----------------------------------------------------------------------------
// nRF24L01 on the ESP32-S3 FSPI bus
// -----------------------------------------------------------------------------
constexpr uint8_t kRadioCsn = 10;
constexpr uint8_t kRadioMosi = 11;
constexpr uint8_t kRadioMiso = 12;
constexpr uint8_t kRadioSck = 13;
constexpr uint8_t kRadioCe = 14;

// Reserved for a future "chip left power" / enable line.
constexpr uint8_t kClp = 15;

// -----------------------------------------------------------------------------
// MPU6050 on Wire1
// -----------------------------------------------------------------------------
constexpr uint8_t kSda = 21;
constexpr uint8_t kScl = 47;

// -----------------------------------------------------------------------------
// VL53L0X XSHUT pins (ToF not implemented yet)
// -----------------------------------------------------------------------------
constexpr uint8_t kXshutToF = 37;
constexpr uint8_t kXshutToFRight = 48;
constexpr uint8_t kXshutToFLeft = 35;
constexpr uint8_t kXshutToFBack = 36;

// Reserved: SD card logging (not implemented).
// constexpr uint8_t kSdCs = 20;

}  // namespace robot::pins
