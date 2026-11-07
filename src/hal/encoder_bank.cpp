#include "hal/encoder_bank.h"

#include <Arduino.h>

#include "config/pins.h"

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

// `count += 1 / -1` instead of `++`/`--`: those are deprecated on
// volatile-qualified integers in C++20 (-Wvolatile) and mean the same thing.
void IRAM_ATTR EncoderBank::onLeftFront() {
  counts_[index(Wheel::kLeftFront)] +=
      digitalRead(pins::kLeftFrontEncoderB) ? 1 : -1;
}

void IRAM_ATTR EncoderBank::onRightFront() {
  counts_[index(Wheel::kRightFront)] +=
      digitalRead(pins::kRightFrontEncoderB) ? 1 : -1;
}

void IRAM_ATTR EncoderBank::onLeftBack() {
  counts_[index(Wheel::kLeftBack)] +=
      digitalRead(pins::kLeftBackEncoderB) ? 1 : -1;
}

void IRAM_ATTR EncoderBank::onRightBack() {
  counts_[index(Wheel::kRightBack)] +=
      digitalRead(pins::kRightBackEncoderB) ? 1 : -1;
}

}  // namespace robot::hal
