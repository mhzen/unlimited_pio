#pragma once

#include <cstdint>

namespace robot::imu_cfg {

// -----------------------------------------------------------------------------
// MPU6050 on I2C
// -----------------------------------------------------------------------------
constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kWhoAmIValue = 0x68;  // value returned by WHO_AM_I
constexpr uint32_t kI2cClock = 100000;

constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegPwrMgmt2 = 0x6C;
constexpr uint8_t kRegWhoAmI = 0x75;

// AFS_SEL=3 (+-16 g) and FS_SEL=3 (+-2000 dps).
constexpr float kAccelLsbPerG = 8192.0f;
constexpr float kGyroLsbPerDps = 65.5f;
constexpr float kGravityMps2 = 9.80665f;

// DSMPLRT_DIV=9 with DLPF=3 -> 100 Hz.
constexpr float kSampleRateHz = 100.0f;
constexpr uint32_t kSampleIntervalUs = 10000;

// Give up on the IMU after this many consecutive read failures.
constexpr uint8_t kMaxReadFailures = 10;

// -----------------------------------------------------------------------------
// Startup calibration (bias + Madgwick warm-up)
// -----------------------------------------------------------------------------
constexpr uint16_t kCalibrationSamples = 300;
constexpr uint16_t kWarmupSamples = 200;
constexpr uint32_t kCalibrationTimeoutMs = 10000;
constexpr uint32_t kWarmupTimeoutMs = 8000;
constexpr uint32_t kCalibrationSampleDelayMs = 10;

}  // namespace robot::imu_cfg
