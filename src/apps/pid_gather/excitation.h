#pragma once

#include <cstdint>

#include "app/pid_gather_config.h"

namespace robot::pid_gather {

enum class Mode : uint8_t { kStep = 0, kChirp = 1, kPrbs = 2, kRamp = 3 };
enum class Axis : uint8_t { kVx = 0, kVy = 1, kOmega = 2 };

constexpr Mode kMode = static_cast<Mode>(cfg::kMode);
constexpr Axis kAxis = static_cast<Axis>(cfg::kAxis);

const char *modeName(Mode mode);
const char *axisName(Axis axis);

// STEP / CHIRP / PRBS generator. Only PRBS is stateful.
class Excitation {
 public:
  void reset(uint32_t nowMs);

  // PWM command for `tSeconds` after the run started. PRBS timing is
  // wall-clock based, hence `nowMs`.
  int value(float tSeconds, uint32_t nowMs);

 private:
  void stepPrbs();

  uint16_t prbsState_ = cfg::kPrbsSeed;
  uint32_t prbsLastToggleMs_ = 0;
  int prbsOutput_ = 0;
};

// RAMP: ramp up to the amplitude, hold, then ramp down to zero.
class RampProfile {
 public:
  void reset();

  // Returns the current PWM command. `requestDecel` latches the down-ramp.
  int update(float dt, bool requestDecel);

  bool finished() const { return finished_; }
  bool decelerating() const { return decelerating_; }

 private:
  float pwm_ = 0.0f;
  bool decelerating_ = false;
  bool finished_ = false;
};

}  // namespace robot::pid_gather
