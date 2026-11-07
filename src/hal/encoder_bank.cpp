#include "hal/encoder_bank.h"

#include <Arduino.h>

#include "config/pins.h"
#include "driver/gpio.h"

namespace robot::hal {

volatile int32_t EncoderBank::counts_[kWheelCount] = {0, 0, 0, 0};

namespace {

struct EncoderPins {
  uint8_t a;
  uint8_t b;
};

// Indexed by Wheel: LF, RF, LB, RB.
constexpr EncoderPins kEncoderPins[kWheelCount] = {
    {pins::kLeftFrontEncoderA, pins::kLeftFrontEncoderB},
    {pins::kRightFrontEncoderA, pins::kRightFrontEncoderB},
    {pins::kLeftBackEncoderA, pins::kLeftBackEncoderB},
    {pins::kRightBackEncoderA, pins::kRightBackEncoderB},
};

}  // namespace

void EncoderBank::begin() {
  for (const EncoderPins &encoder : kEncoderPins) {
    pinMode(encoder.a, INPUT_PULLUP);
    pinMode(encoder.b, INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(kEncoderPins[index(Wheel::kLeftFront)].a),
                  onLeftFront, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPins[index(Wheel::kRightFront)].a),
                  onRightFront, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPins[index(Wheel::kLeftBack)].a),
                  onLeftBack, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPins[index(Wheel::kRightBack)].a),
                  onRightBack, RISING);
}

void EncoderBank::readCounts(int32_t counts[kWheelCount]) const {
  noInterrupts();
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    counts[i] = counts_[i];
  }
  interrupts();
}

void EncoderBank::reset() {
  noInterrupts();
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    counts_[i] = 0;
  }
  interrupts();
}

// `count += 1 / -1` instead of `++`/`--`: those are deprecated on
// volatile-qualified integers in C++20 (-Wvolatile) and mean the same thing.
//
// gpio_get_level() instead of digitalRead(): it is IRAM-resident, so the ISR
// cannot stall on a disabled flash cache (NVS writes, Wi-Fi flash access).
void IRAM_ATTR EncoderBank::onLeftFront() {
  const int level =
      gpio_get_level(static_cast<gpio_num_t>(pins::kLeftFrontEncoderB));
  counts_[index(Wheel::kLeftFront)] += level ? 1 : -1;
}

void IRAM_ATTR EncoderBank::onRightFront() {
  const int level =
      gpio_get_level(static_cast<gpio_num_t>(pins::kRightFrontEncoderB));
  counts_[index(Wheel::kRightFront)] += level ? 1 : -1;
}

void IRAM_ATTR EncoderBank::onLeftBack() {
  const int level =
      gpio_get_level(static_cast<gpio_num_t>(pins::kLeftBackEncoderB));
  counts_[index(Wheel::kLeftBack)] += level ? 1 : -1;
}

void IRAM_ATTR EncoderBank::onRightBack() {
  const int level =
      gpio_get_level(static_cast<gpio_num_t>(pins::kRightBackEncoderB));
  counts_[index(Wheel::kRightBack)] += level ? 1 : -1;
}

}  // namespace robot::hal
