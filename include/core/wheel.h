#pragma once

#include <cstddef>
#include <cstdint>

namespace robot {

// Wheel order is fixed: it indexes every tuning table in drive_config.h.
enum class Wheel : uint8_t {
  kLeftFront = 0,
  kRightFront = 1,
  kLeftBack = 2,
  kRightBack = 3,
};

constexpr uint8_t kWheelCount = 4;

constexpr size_t index(Wheel wheel) noexcept {
  return static_cast<size_t>(wheel);
}

constexpr const char *wheelName(Wheel wheel) noexcept {
  switch (wheel) {
    case Wheel::kLeftFront:
      return "LF";
    case Wheel::kRightFront:
      return "RF";
    case Wheel::kLeftBack:
      return "LB";
    case Wheel::kRightBack:
      return "RB";
  }
  return "??";
}

}  // namespace robot
