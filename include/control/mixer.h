#pragma once

#include "core/wheel.h"

namespace robot {

// X-drive mixing: translates vx/vy/omega into the four wheel PWM setpoints and
// scales all of them down equally when any wheel would exceed kMaxMotorPwm.
//
// This lives in one place on purpose: the robot firmware and the sysid tool
// must agree on the RB sign convention, or a model fitted from tool data ends
// up inverted when used by the robot.
void mixWheels(float vx, float vy, float omega, float out[kWheelCount]);

}  // namespace robot
