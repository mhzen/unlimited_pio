#include "control/wheel_pid.h"

#include <Arduino.h>
#include <math.h>

#include "config/drive_config.h"
#include "core/math_util.h"

namespace robot {

int calculateWheelPid(WheelState &wheel, size_t wheelIndex, float dt) {
  const float error = wheel.targetCps - wheel.filteredCps;
  const float feedforward = wheel.targetPwm;
  const float proportional = drive_cfg::kWheelKp[wheelIndex] * error;

  wheel.integral += error * dt;
  if (drive_cfg::kWheelKi[wheelIndex] > 0.0f) {
    const float stateLimit =
        drive_cfg::kPidIntegralPwmLimit / drive_cfg::kWheelKi[wheelIndex];
    wheel.integral = math::clampFloat(wheel.integral, -stateLimit, stateLimit);
  } else {
    wheel.integral = 0.0f;
  }

  const float integralTerm = drive_cfg::kWheelKi[wheelIndex] * wheel.integral;
  const float derivative = (error - wheel.previousError) / dt;
  wheel.previousError = error;

  float output = feedforward + proportional + integralTerm +
                 drive_cfg::kWheelKd[wheelIndex] * derivative;

  if (fabsf(wheel.targetCps) < drive_cfg::kStopSpeedCps &&
      fabsf(wheel.filteredCps) < drive_cfg::kStopSpeedCps) {
    wheel.integral = 0.0f;
    wheel.previousError = 0.0f;
    output = 0.0f;
  }

  return constrain(static_cast<int>(roundf(output)),
                   -drive_cfg::kMaxMotorPwm, drive_cfg::kMaxMotorPwm);
}

}  // namespace robot
