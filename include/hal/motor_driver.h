#pragma once

#include <cstdint>

#include "core/wheel.h"

namespace robot::hal {

// Thin wrapper over the four DC motor channels (2 direction pins each).
// PWM is signed: positive = forward on the A pin, negative = reverse on B.
class MotorDriver {
 public:
  void begin();

  void set(Wheel wheel, int pwm);
  void stopAll();

 private:
  static void writePins(uint8_t pinA, uint8_t pinB, int pwm);
};

}  // namespace robot::hal
