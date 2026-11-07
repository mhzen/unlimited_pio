#include "sensors/imu.h"

#include <Arduino.h>

#include "config/imu_config.h"
#include "core/math_util.h"
#include "sensors/mpu6050.h"

namespace robot::sensors {

bool Imu::begin(Mpu6050 *mpu) {
  mpu_ = mpu;
  madgwick_.begin(imu_cfg::kSampleRateHz);
  ready_ = mpu_ != nullptr && mpu_->begin();
  calibrated_ = false;
  if (ready_) lastSampleUs_ = micros();
  return ready_;
}

bool Imu::readSample(ImuSample &sample) {
  return mpu_ != nullptr && mpu_->readSample(sample);
}

bool Imu::calibrateAndWarmup(BackgroundHook service) {
  if (!ready_) return false;

  Serial.println("Calibrating MPU. Keep robot stationary.");

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  uint16_t valid = 0;
  uint32_t started = millis();

  while (valid < imu_cfg::kCalibrationSamples) {
    if (service != nullptr) service();
    if (millis() - started > imu_cfg::kCalibrationTimeoutMs) {
      ready_ = false;
      calibrated_ = false;
      return false;
    }

    ImuSample sample{};
    if (readSample(sample)) {
      sumX += sample.gx;
      sumY += sample.gy;
      sumZ += sample.gz;
      ++valid;
    }
    delay(imu_cfg::kCalibrationSampleDelayMs);
  }

  gyroBiasX_ = sumX / valid;
  gyroBiasY_ = sumY / valid;
  gyroBiasZ_ = sumZ / valid;

  madgwick_.begin(imu_cfg::kSampleRateHz);
  valid = 0;
  started = millis();

  while (valid < imu_cfg::kWarmupSamples) {
    if (service != nullptr) service();
    if (millis() - started > imu_cfg::kWarmupTimeoutMs) {
      ready_ = false;
      calibrated_ = false;
      return false;
    }

    ImuSample sample{};
    if (readSample(sample)) {
      madgwick_.updateIMU(sample.gx - gyroBiasX_, sample.gy - gyroBiasY_,
                          sample.gz - gyroBiasZ_, sample.ax, sample.ay,
                          sample.az);
      ++valid;
    }
    delay(imu_cfg::kCalibrationSampleDelayMs);
  }

  yawZero_ = madgwick_.getYaw();
  calibrated_ = true;
  consecutiveFailures_ = 0;
  lastSampleUs_ = micros();
  return true;
}

void Imu::update(uint32_t nowUs) {
  if (!ready_ || !calibrated_) return;
  if (nowUs - lastSampleUs_ < imu_cfg::kSampleIntervalUs) return;
  lastSampleUs_ = nowUs;

  ImuSample sample{};
  if (!readSample(sample)) {
    if (++consecutiveFailures_ >= imu_cfg::kMaxReadFailures) {
      ready_ = false;
      calibrated_ = false;
    }
    return;
  }

  consecutiveFailures_ = 0;
  madgwick_.updateIMU(sample.gx - gyroBiasX_, sample.gy - gyroBiasY_,
                      sample.gz - gyroBiasZ_, sample.ax, sample.ay, sample.az);

  const float roll = madgwick_.getRoll();
  const float pitch = madgwick_.getPitch();
  const float yaw = math::wrapAngle180(madgwick_.getYaw() - yawZero_);
  const float tiltCos =
      math::clampFloat(cosf(roll * DEG_TO_RAD) * cosf(pitch * DEG_TO_RAD),
                       -1.0f, 1.0f);

  rollDeg_ = roll;
  pitchDeg_ = pitch;
  yawDeg_ = yaw;
  tiltDeg_ = acosf(tiltCos) * RAD_TO_DEG;
  accX_ = sample.ax;
  accY_ = sample.ay;
  accZ_ = sample.az;
}

}  // namespace robot::sensors
