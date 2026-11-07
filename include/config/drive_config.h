#pragma once

#include <cstdint>

namespace robot::drive_cfg {

// -----------------------------------------------------------------------------
// Output limits and gears
// -----------------------------------------------------------------------------
// kMaxMotorPwm is the PWM ceiling for a single motor channel.
constexpr int kMaxMotorPwm = 250;  // kalo ganti ini jangan lupa ganti kMaxWheelCps !
constexpr int kMaxRotationPwm = 70;  // batas PWM rotasi; harus <= kMaxMotorPwm

constexpr int kGearPwm[3] = {127, 160, 250};
constexpr int kGearCount = 3;

constexpr bool kRotateOnlyWhenStopped = false;

// -----------------------------------------------------------------------------
// Ramp profile (PWM per second)
// -----------------------------------------------------------------------------
// 0 -> 255 in 255 / 250.5 = ~1.02 s
constexpr float kTranslationAccelPwmPerSec = 250.5f;
// 255 -> 0 in ~1.02 s
constexpr float kTranslationDecelPwmPerSec = 250.5f;

constexpr float kRotationAccelPwmPerSec = 510.0f;
constexpr float kRotationDecelPwmPerSec = 510.0f;

constexpr uint32_t kControlIntervalUs = 20000;  // 50 Hz

// Replace with 80-90% of the slowest wheel's measured maximum counts/second.
constexpr float kMaxWheelCps = 890.0f;

// Order: LF, RF, LB, RB. Flip an incorrect encoder direction to -1.
constexpr int8_t kEncoderSign[4] = {-1, -1, 1, 1};

// -----------------------------------------------------------------------------
// Optional per-wheel PID (feedforward + PID on encoder speed)
// -----------------------------------------------------------------------------
constexpr bool kEnableWheelPid = false;

// Order: LF, RF, LB, RB.
constexpr float kWheelKp[4] = {0.4411f, 0.4279f, 0.3719f, 0.3766f};
constexpr float kWheelKi[4] = {9.7995f, 10.0219f, 7.3390f, 9.3217f};
constexpr float kWheelKd[4] = {-0.0014f, -0.0013f, -0.0014f, -0.0011f};

constexpr float kPidIntegralPwmLimit = 80.0f;
constexpr float kSpeedFilterAlpha = 0.35f;
constexpr float kStopSpeedCps = 8.0f;

// -----------------------------------------------------------------------------
// Heading lock (keep the robot pointing the same way while translating)
// -----------------------------------------------------------------------------
constexpr float kHeadingLockKp = 0.0f;  // kalo disable di set ke 0
constexpr float kHeadingLockMaxOmega = 60.0f;

// -----------------------------------------------------------------------------
// Telemetry
// -----------------------------------------------------------------------------
constexpr uint32_t kTelemetryIntervalMs = 500;

}  // namespace robot::drive_cfg
