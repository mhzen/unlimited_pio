#pragma once

#include <cstdint>

#include "control/joystick.h"
#include "core/wheel.h"

namespace robot {

namespace hal {
class EncoderBank;
class MotorDriver;
}  // namespace hal

// Per-wheel runtime state (setpoint, measurement, PID memory).
struct WheelState {
  float targetPwm = 0.0f;
  float targetCps = 0.0f;
  float measuredCps = 0.0f;
  float filteredCps = 0.0f;
  float integral = 0.0f;
  float previousError = 0.0f;
  int outputPwm = 0;
  int32_t previousCount = 0;
};

// X-drive kinematics + asymmetric ramping + optional per-wheel PID + heading
// lock. Call update() at a fixed ~50 Hz cadence; it emits motor PWM itself.
class DriveController {
 public:
  void begin(hal::MotorDriver *motors, hal::EncoderBank *encoders);

  // Joystick-derived setpoints, in PWM units (not counts/s).
  void setTargets(int vx, int vy, int omega);
  void setEmergency(bool emergency);

  // Yaw feedback for heading lock. `valid` is false until the IMU is ready.
  void setHeading(float yawDeg, bool valid);

  // `hold` = emergency stop or radio link lost: stop now, skip ramping.
  void update(uint32_t nowUs, bool hold);

  void reset();
  void stop();

  // --- telemetry ---
  int targetVx() const { return targetVx_; }
  int targetVy() const { return targetVy_; }
  int targetOmega() const { return targetOmega_; }
  float smoothVx() const { return smoothVx_; }
  float smoothVy() const { return smoothVy_; }
  float smoothOmega() const { return smoothOmega_; }
  bool emergency() const { return emergency_; }
  joystick::Direction direction() const { return direction_; }
  const WheelState &wheel(Wheel wheel) const { return wheels_[index(wheel)]; }

 private:
  void mixWheels(float vx, float vy, float omega,
                 float out[kWheelCount]) const;
  int calculatePid(WheelState &wheel, size_t wheelIndex, float dt) const;
  void applyOutputs();
  float headingCorrection();

  hal::MotorDriver *motors_ = nullptr;
  hal::EncoderBank *encoders_ = nullptr;

  WheelState wheels_[kWheelCount]{};
  int targetVx_ = 0;
  int targetVy_ = 0;
  int targetOmega_ = 0;
  joystick::Direction direction_ = joystick::Direction::kStop;
  bool emergency_ = false;

  float smoothVx_ = 0.0f;
  float smoothVy_ = 0.0f;
  float smoothOmega_ = 0.0f;
  uint32_t lastUpdateUs_ = 0;

  float headingYawDeg_ = 0.0f;
  bool headingValid_ = false;
  float headingSetpointDeg_ = 0.0f;
  bool headingActive_ = false;
};

}  // namespace robot
