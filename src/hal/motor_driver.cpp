#include "hal/motor_driver.h"

#include <Arduino.h>

#include "config/drive_config.h"
#include "config/pins.h"

namespace robot::hal {
namespace {

struct MotorPins {
  uint8_t a;
  uint8_t b;
};

// Indexed by Wheel: LF, RF, LB, RB.
constexpr MotorPins kMotorPins[kWheelCount] = {
    {pins::kLeftFrontA, pins::kLeftFrontB},
    {pins::kRightFrontA, pins::kRightFrontB},
    {pins::kLeftBackA, pins::kLeftBackB},
    {pins::kRightBackA, pins::kRightBackB},
};

}  // namespace

void MotorDriver::begin() {
  for (const MotorPins &motor : kMotorPins) {
    pinMode(motor.a, OUTPUT);
    pinMode(motor.b, OUTPUT);
  }
}

void MotorDriver::set(Wheel wheel, int pwm) {
  const MotorPins &motor = kMotorPins[index(wheel)];
  writePins(motor.a, motor.b, pwm);
}

void MotorDriver::stopAll() {
  for (const MotorPins &motor : kMotorPins) {
    writePins(motor.a, motor.b, 0);
  }
}

void MotorDriver::writePins(uint8_t pinA, uint8_t pinB, int pwm) {
  pwm = constrain(pwm, -drive_cfg::kMaxMotorPwm, drive_cfg::kMaxMotorPwm);
  if (pwm > 0) {
    analogWrite(pinB, 0);
    analogWrite(pinA, pwm);
  } else if (pwm < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

}  // namespace robot::hal
