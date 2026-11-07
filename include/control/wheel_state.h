#pragma once

#include <cstdint>

namespace robot {

// Runtime state of one wheel: PWM setpoint, speed measurement and PID memory.
// Shared by the robot firmware (DriveController) and the sysid tool.
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

}  // namespace robot
