#pragma once

#include <cstdint>

#include "core/wheel.h"

namespace robot::hal {

// Four quadrature encoders read through GPIO interrupts on the A channel,
// with the B channel giving direction. Counts are signed and monotonic.
class EncoderBank {
 public:
  void begin();

  // Atomic snapshot of all four counters.
  void readCounts(int32_t counts[kWheelCount]) const;

  // Atomically zero all four counters (start of a run).
  void reset();

 private:
  // IRAM_ATTR is applied on the definitions (src/hal/encoder_bank.cpp) so this
  // header stays free of Arduino.h.
  static void onLeftFront();
  static void onRightFront();
  static void onLeftBack();
  static void onRightBack();

  static volatile int32_t counts_[kWheelCount];
};

}  // namespace robot::hal
