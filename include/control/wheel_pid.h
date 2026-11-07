#pragma once

#include <cstddef>

#include "control/wheel_state.h"

namespace robot {

// Feedforward + PID on encoder speed for one wheel.
// Returns the PWM command clamped to +-kMaxMotorPwm.
int calculateWheelPid(WheelState &wheel, size_t wheelIndex, float dt);

}  // namespace robot
