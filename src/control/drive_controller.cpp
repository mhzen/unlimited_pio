#include "control/drive_controller.h"

#include <Arduino.h>
#include <math.h>

#include "config/drive_config.h"
#include "control/mixer.h"
#include "control/wheel_pid.h"
#include "core/math_util.h"
#include "hal/encoder_bank.h"
#include "hal/motor_driver.h"

namespace robot {

void DriveController::begin(hal::MotorDriver *motors,
                            hal::EncoderBank *encoders) {
  motors_ = motors;
  encoders_ = encoders;

  int32_t counts[kWheelCount];
  encoders_->readCounts(counts);
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    wheels_[i] = WheelState{};
    wheels_[i].previousCount = counts[i];
  }

  lastUpdateUs_ = micros();
}

void DriveController::setTargets(int vx, int vy, int omega) {
  targetVx_ = vx;
  targetVy_ = vy;
  targetOmega_ = omega;
  direction_ = joystick::directionFrom(vx, vy);
}

void DriveController::setEmergency(bool emergency) { emergency_ = emergency; }

void DriveController::setHeading(float yawDeg, bool valid) {
  headingYawDeg_ = yawDeg;
  headingValid_ = valid;
}

void DriveController::update(uint32_t nowUs, bool hold) {
  if (nowUs - lastUpdateUs_ < drive_cfg::kControlIntervalUs) return;

  // Clamp the loop period so a stalled loop cannot produce a huge ramp step.
  const float dt = math::clampFloat((nowUs - lastUpdateUs_) * 1.0e-6f, 0.005f,
                                    0.100f);
  lastUpdateUs_ = nowUs;

  if (hold) {
    reset();
    stop();
    return;
  }

  smoothVx_ =
      math::applyAsymmetricSlew(smoothVx_, targetVx_,
                                drive_cfg::kTranslationAccelPwmPerSec,
                                drive_cfg::kTranslationDecelPwmPerSec, dt);
  smoothVy_ =
      math::applyAsymmetricSlew(smoothVy_, targetVy_,
                                drive_cfg::kTranslationAccelPwmPerSec,
                                drive_cfg::kTranslationDecelPwmPerSec, dt);
  smoothOmega_ =
      math::applyAsymmetricSlew(smoothOmega_, targetOmega_,
                                drive_cfg::kRotationAccelPwmPerSec,
                                drive_cfg::kRotationDecelPwmPerSec, dt);

  float wheelPwm[kWheelCount];
  mixWheels(smoothVx_, smoothVy_, smoothOmega_ + headingCorrection(), wheelPwm);

  int32_t counts[kWheelCount];
  encoders_->readCounts(counts);

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    WheelState &wheel = wheels_[i];
    wheel.targetPwm = wheelPwm[i];

    const int32_t delta =
        (counts[i] - wheel.previousCount) * drive_cfg::kEncoderSign[i];
    wheel.previousCount = counts[i];
    wheel.measuredCps = static_cast<float>(delta) / dt;
    wheel.filteredCps +=
        drive_cfg::kSpeedFilterAlpha * (wheel.measuredCps - wheel.filteredCps);
    wheel.targetCps =
        wheel.targetPwm / drive_cfg::kMaxMotorPwm * drive_cfg::kMaxWheelCps;

    if (drive_cfg::kEnableWheelPid) {
      wheel.outputPwm = calculateWheelPid(wheel, i, dt);    } else {
      wheel.integral = 0.0f;
      wheel.previousError = 0.0f;
      wheel.outputPwm = static_cast<int>(roundf(wheel.targetPwm));
    }
  }

  applyOutputs();
}

// PD heading hold. Returns an omega command added to the operator's omega.
float DriveController::headingCorrection() {
  if (targetOmega_ != 0 || !headingValid_) {
    headingActive_ = false;
    return 0.0f;
  }

  if (targetVx_ == 0 && targetVy_ == 0) {
    headingActive_ = false;
    return 0.0f;
  }

  if (!headingActive_) {
    headingSetpointDeg_ = headingYawDeg_;
    headingActive_ = true;
  }

  const float error =
      math::wrapAngle180(headingSetpointDeg_ - headingYawDeg_);
  return math::clampFloat(-drive_cfg::kHeadingLockKp * error,
                          -drive_cfg::kHeadingLockMaxOmega,
                          drive_cfg::kHeadingLockMaxOmega);
}

void DriveController::reset() {
  smoothVx_ = smoothVy_ = smoothOmega_ = 0.0f;
  headingActive_ = false;

  int32_t counts[kWheelCount];
  encoders_->readCounts(counts);

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    wheels_[i] = WheelState{};
    wheels_[i].previousCount = counts[i];
  }
}

void DriveController::stop() { motors_->stopAll(); }

void DriveController::applyOutputs() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    motors_->set(static_cast<Wheel>(i), wheels_[i].outputPwm);
  }
}

}  // namespace robot
