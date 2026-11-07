#pragma once

#include <cstdint>

#include "core/imu_sample.h"

class TwoWire;

namespace robot::sensors {

// Register-level MPU6050 driver (no external IMU library).
class Mpu6050 {
 public:
  explicit Mpu6050(TwoWire &wire) : wire_(wire) {}

  // Verifies WHO_AM_I, resets, configures +-16 g / +-2000 dps at 100 Hz.
  bool begin();

  bool readSample(ImuSample &sample);

 private:
  bool present();
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t *data, uint8_t length);

  TwoWire &wire_;
};

}  // namespace robot::sensors
