#pragma once

#include <cstdint>

#include <MadgwickAHRS.h>

#include "core/imu_sample.h"

namespace robot::sensors {

class Mpu6050;

// Madgwick fusion on top of the MPU6050 driver: gyro bias calibration,
// yaw zeroing, and tilt estimation.
class Imu {
 public:
  // Called repeatedly during the blocking startup calibration so the caller
  // can keep the radio serviced and the motors stopped.
  using BackgroundHook = void (*)();

  bool begin(Mpu6050 *mpu);

  bool ready() const { return ready_; }
  bool calibrated() const { return calibrated_; }

  // Blocks for ~5 s while the robot must stand still.
  bool calibrateAndWarmup(BackgroundHook service);

  // Samples at kSampleIntervalUs; cheap to call from loop().
  void update(uint32_t nowUs);

  float yawDeg() const { return yawDeg_; }
  float pitchDeg() const { return pitchDeg_; }
  float rollDeg() const { return rollDeg_; }
  float tiltDeg() const { return tiltDeg_; }
  float accX() const { return accX_; }
  float accY() const { return accY_; }
  float accZ() const { return accZ_; }

 private:
  bool readSample(ImuSample &sample);

  Mpu6050 *mpu_ = nullptr;
  Madgwick madgwick_;

  bool ready_ = false;
  bool calibrated_ = false;

  float gyroBiasX_ = 0.0f;
  float gyroBiasY_ = 0.0f;
  float gyroBiasZ_ = 0.0f;
  float yawZero_ = 0.0f;

  float yawDeg_ = 0.0f;
  float pitchDeg_ = 0.0f;
  float rollDeg_ = 0.0f;
  float tiltDeg_ = 0.0f;
  float accX_ = 0.0f;
  float accY_ = 0.0f;
  float accZ_ = 0.0f;

  uint8_t consecutiveFailures_ = 0;
  uint32_t lastSampleUs_ = 0;
};

}  // namespace robot::sensors
