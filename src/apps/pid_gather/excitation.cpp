#include "excitation.h"

#include <Arduino.h>
#include <math.h>

#include "core/math_util.h"

namespace robot::pid_gather {

const char *modeName(Mode mode) {
  switch (mode) {
    case Mode::kStep:
      return "STEP";
    case Mode::kChirp:
      return "CHIRP";
    case Mode::kPrbs:
      return "PRBS";
    case Mode::kRamp:
      return "RAMP";
  }
  return "UNKNOWN";
}

const char *axisName(Axis axis) {
  switch (axis) {
    case Axis::kVx:
      return "VX";
    case Axis::kVy:
      return "VY";
    case Axis::kOmega:
      return "OMEGA";
  }
  return "UNKNOWN";
}

void Excitation::reset(uint32_t nowMs) {
  prbsState_ = cfg::kPrbsSeed;
  prbsLastToggleMs_ = nowMs;
  // The original sketch emitted the first PRBS bit at run start, not after the
  // first bit period.
  stepPrbs();
}

int Excitation::value(float tSeconds, uint32_t nowMs) {
  switch (kMode) {
    case Mode::kStep:
      return cfg::kAmplitudePwm;

    case Mode::kChirp: {
      const float sweep =
          (cfg::kChirpEndHz - cfg::kChirpStartHz) / cfg::kChirpDurationS;
      const float phase =
          2.0f * PI * (cfg::kChirpStartHz * tSeconds +
                       0.5f * sweep * tSeconds * tSeconds);
      return static_cast<int>(cfg::kAmplitudePwm * sinf(phase));
    }

    case Mode::kPrbs:
      if (nowMs - prbsLastToggleMs_ >= cfg::kPrbsBitPeriodMs) {
        prbsLastToggleMs_ = nowMs;
        stepPrbs();
      }
      return prbsOutput_;

    default:
      return 0;
  }
}

void Excitation::stepPrbs() {
  const uint8_t bit = ((prbsState_ >> 14) ^ (prbsState_ >> 13)) & 1;
  prbsState_ = static_cast<uint16_t>(((prbsState_ << 1) | bit) & 0x7FFF);

  const int magnitude =
      cfg::kAmplitudePwm > 30 ? cfg::kAmplitudePwm : 30;
  prbsOutput_ = (prbsState_ & 0x4000) ? magnitude : -magnitude;
}

void RampProfile::reset() {
  pwm_ = 0.0f;
  decelerating_ = false;
  finished_ = false;
}

int RampProfile::update(float dt, bool requestDecel) {
  if (finished_) return 0;

  if (requestDecel) decelerating_ = true;

  const float target =
      decelerating_ ? 0.0f : static_cast<float>(cfg::kAmplitudePwm);
  pwm_ = math::applyAsymmetricSlew(pwm_, target, cfg::kRampAccelPwmPerSec,
                                   cfg::kRampDecelPwmPerSec, dt);

  if (decelerating_ && fabsf(pwm_) < 1.0f) {
    pwm_ = 0.0f;
    finished_ = true;
  }

  return static_cast<int>(lroundf(pwm_));
}

}  // namespace robot::pid_gather
