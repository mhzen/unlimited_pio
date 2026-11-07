#include "control/drive_command.h"

#include "config/drive_config.h"
#include "config/joystick_config.h"
#include "control/joystick.h"

namespace robot::control {

DriveCommand commandFrom(const ControlPacket &packet, int gearPwm) {
  DriveCommand command;
  command.vx = joystick_cfg::kYSign *
               joystick::toPwm(packet.joyMoveY, joystick_cfg::kYCenter, gearPwm);
  command.vy = joystick_cfg::kXSign *
               joystick::toPwm(packet.joyMoveX, joystick_cfg::kXCenter, gearPwm);
  command.omega =
      joystick_cfg::kRotateSign *
      joystick::toPwm(packet.joyRotate, joystick_cfg::kRotateCenter,
                      drive_cfg::kMaxRotationPwm);

  // Legacy "snap" rule: while translating, ignore the rotation stick.
  if (drive_cfg::kRotateOnlyWhenStopped &&
      joystick::directionFrom(command.vx, command.vy) !=
          joystick::Direction::kStop) {
    command.omega = 0;
  }

  if (packet.emergency != 0) {
    command.vx = command.vy = command.omega = 0;
  }

  return command;
}

}  // namespace robot::control
