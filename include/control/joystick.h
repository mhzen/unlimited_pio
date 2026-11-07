#pragma once

#include <cstdint>

namespace robot::joystick {

// Coarse movement label, used for telemetry and for ROTATE_ONLY_WHEN_STOPPED.
enum class Direction : uint8_t {
  kStop = 0,
  kForward,
  kRight,
  kBackward,
  kLeft,
};

// Maps a raw ADC reading to a signed PWM command with a dead zone.
int toPwm(int raw, int center, int maxPwm);

// Diagonal moves are allowed; the label only reports the dominant axis.
Direction directionFrom(int vx, int vy);

const char *directionName(Direction direction);

}  // namespace robot::joystick
