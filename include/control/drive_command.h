#pragma once

#include "core/control_packet.h"

namespace robot::control {

// Signed PWM setpoints in the units DriveController expects.
struct DriveCommand {
  int vx;
  int vy;
  int omega;
};

// Applies joystick calibration (signs, dead zone), the current gear, the
// rotate-only-when-stopped rule and the emergency stop to a wireless packet.
DriveCommand commandFrom(const ControlPacket &packet, int gearPwm);

}  // namespace robot::control
