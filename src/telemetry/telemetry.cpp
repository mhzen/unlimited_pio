#include "telemetry/telemetry.h"

#include <Arduino.h>

#include "comms/radio_link.h"
#include "config/drive_config.h"
#include "control/drive_controller.h"
#include "control/joystick.h"
#include "core/wheel.h"
#include "sensors/imu.h"

namespace robot::telemetry {

// kalo pakai sdCard, ganti Serial.print dengan logPrint
void printSeparator() {
  Serial.println("===================================================================");
}

void printImu(const sensors::Imu &imu) {
  Serial.print("IMU pitch=");
  Serial.print(imu.pitchDeg(), 1);
  Serial.print(" roll=");
  Serial.print(imu.rollDeg(), 1);
  Serial.print(" yaw=");
  Serial.print(imu.yawDeg(), 1);
  Serial.print(" tilt=");
  Serial.print(imu.tiltDeg(), 1);
  Serial.print(" acc=");
  Serial.print(imu.accX(), 2);
  Serial.print(',');
  Serial.print(imu.accY(), 2);
  Serial.print(',');
  Serial.println(imu.accZ(), 2);
}

void printDrive(const DriveController &drive, const comms::RadioLink &radio,
                bool linkLost) {
  Serial.print("dir=");
  Serial.print(joystick::directionName(drive.direction()));
  Serial.print(" target=");
  Serial.print(drive.targetVx());
  Serial.print(',');
  Serial.print(drive.targetVy());
  Serial.print(',');
  Serial.print(drive.targetOmega());
  Serial.print(" smooth=");
  Serial.print(drive.smoothVx(), 1);
  Serial.print(',');
  Serial.print(drive.smoothVy(), 1);
  Serial.print(',');
  Serial.print(drive.smoothOmega(), 1);
  Serial.print(" emergency=");
  Serial.print(drive.emergency());
  Serial.print(" gear=");
  Serial.print(radio.gearIndex() + 1);
  Serial.print(" linkLost=");
  Serial.print(linkLost);
  Serial.print(" pid=");
  Serial.println(drive_cfg::kEnableWheelPid);

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    const Wheel wheel = static_cast<Wheel>(i);
    const WheelState &state = drive.wheel(wheel);
    Serial.print(wheelName(wheel));
    Serial.print(" tgtPWM=");
    Serial.print(state.targetPwm, 1);
    Serial.print(" outPWM=");
    Serial.print(state.outputPwm);
    Serial.print(" tgtCps=");
    Serial.print(state.targetCps, 1);
    Serial.print(" measCps=");
    Serial.println(state.filteredCps, 1);
  }
}

}  // namespace robot::telemetry
