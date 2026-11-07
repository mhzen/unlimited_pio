#pragma once

namespace robot {

class DriveController;

namespace sensors {
class Imu;
}  // namespace sensors

namespace comms {
class RadioLink;
}  // namespace comms

namespace telemetry {

void printSeparator();
void printImu(const sensors::Imu &imu);
void printDrive(const DriveController &drive, const comms::RadioLink &radio,
                bool linkLost);

}  // namespace telemetry
}  // namespace robot
