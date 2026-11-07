#include "control/mixer.h"

#include <math.h>

#include "config/drive_config.h"

namespace robot {

void mixWheels(float vx, float vy, float omega, float out[kWheelCount]) {
  out[index(Wheel::kLeftFront)] = vx + vy + omega;
  out[index(Wheel::kRightFront)] = -vx + vy + omega;
  out[index(Wheel::kLeftBack)] = vx - vy + omega;
  out[index(Wheel::kRightBack)] = -vx - vy + omega;

  float maximum = 0.0f;
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    const float magnitude = fabsf(out[i]);
    if (magnitude > maximum) maximum = magnitude;
  }

  if (maximum > drive_cfg::kMaxMotorPwm) {
    const float scale = drive_cfg::kMaxMotorPwm / maximum;
    for (uint8_t i = 0; i < kWheelCount; ++i) {
      out[i] *= scale;
    }
  }
}

}  // namespace robot
