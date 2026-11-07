#include "control/joystick.h"

#include <Arduino.h>

#include "config/joystick_config.h"

namespace robot::joystick {

int toPwm(int raw, int center, int maxPwm) {
  raw = constrain(raw, joystick_cfg::kAdcMin, joystick_cfg::kAdcMax);
  if (raw >= center - joystick_cfg::kDeadzone &&
      raw <= center + joystick_cfg::kDeadzone) {
    return 0;
  }

  const int output =
      raw < center
          ? map(raw, joystick_cfg::kAdcMin, center - joystick_cfg::kDeadzone,
                -maxPwm, 0)
          : map(raw, center + joystick_cfg::kDeadzone,
                joystick_cfg::kAdcMax, 0, maxPwm);

  return constrain(output, -maxPwm, maxPwm);
}

Direction directionFrom(int vx, int vy) {
  if (vx == 0 && vy == 0) return Direction::kStop;

  // Only a label: vx and vy can both be active (diagonal driving).
  if (abs(vx) >= abs(vy)) {
    return vx > 0 ? Direction::kRight : Direction::kLeft;  // right stick
  }
  return vy > 0 ? Direction::kForward : Direction::kBackward;  // left stick
}

const char *directionName(Direction direction) {
  switch (direction) {
    case Direction::kForward:
      return "FORWARD";
    case Direction::kRight:
      return "RIGHT";
    case Direction::kBackward:
      return "BACKWARD";
    case Direction::kLeft:
      return "LEFT";
    default:
      return "STOP";
  }
}

}  // namespace robot::joystick
