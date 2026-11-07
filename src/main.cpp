#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "comms/radio_link.h"
#include "config/drive_config.h"
#include "config/imu_config.h"
#include "config/pins.h"
#include "control/drive_command.h"
#include "control/drive_controller.h"
#include "core/control_packet.h"
#include "hal/encoder_bank.h"
#include "hal/motor_driver.h"
#include "sensors/imu.h"
#include "sensors/mpu6050.h"
#include "telemetry/telemetry.h"

#include "esp_bt.h"
#include "esp_wifi.h"

namespace {

robot::hal::MotorDriver g_motors;
robot::hal::EncoderBank g_encoders;
robot::DriveController g_drive;

SPIClass g_radioSpi(FSPI);
robot::comms::RadioLink g_radio;

robot::sensors::Mpu6050 g_mpu(Wire1);
robot::sensors::Imu g_imu;

uint32_t g_lastTelemetryMs = 0;

// The forklift keeps the ESP32 Wi-Fi/Bluetooth radios off: they add noise and
// power draw next to the nRF24L01 link.
void disableWirelessStack() {
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
}

// Keeps the radio serviced and the wheels still while the IMU calibrates.
void serviceDuringCalibration() {
  g_radio.service(millis());
  g_radio.poll();
  g_motors.stopAll();
}

void initializeSensorsAndRadio() {
  // FSPI with the custom nRF24L01 pin mapping. The bus must be running before
  // RF24 is handed the SPIClass.
  g_radioSpi.begin(robot::pins::kRadioSck, robot::pins::kRadioMiso,
                   robot::pins::kRadioMosi);
  g_radio.begin(&g_radioSpi);

  Wire1.begin(robot::pins::kSda, robot::pins::kScl);
  Wire1.setClock(robot::imu_cfg::kI2cClock);
  delay(100);

  if (g_imu.begin(&g_mpu)) {
    g_imu.calibrateAndWarmup(serviceDuringCalibration);
  } else {
    Serial.println("MPU unavailable; drive remains operational.");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) {}

  Serial.println("\nX-drive smooth ramp + optional wheel PID");
  Serial.print("PID enabled: ");
  Serial.println(robot::drive_cfg::kEnableWheelPid ? "YES" : "NO");

  g_motors.begin();
  g_encoders.begin();
  g_motors.stopAll();

  disableWirelessStack();
  Serial.println("Wi-Fi and Bluetooth di nonaktifkan");

  initializeSensorsAndRadio();

  // Seeds the encoder baselines and starts the control timebase.
  g_drive.begin(&g_motors, &g_encoders);

  Serial.println("Main control loop started.");
}

void loop() {
  const uint32_t nowMs = millis();

  g_radio.service(nowMs);
  g_radio.poll();
  g_imu.update(micros());

  const robot::ControlPacket &packet = g_radio.packet();
  const bool emergency = packet.emergency != 0;
  const bool linkLost = g_radio.linkLost(nowMs);

  const int gearPwm = robot::drive_cfg::kGearPwm[g_radio.gearIndex()];
  const robot::control::DriveCommand command =
      robot::control::commandFrom(packet, gearPwm);

  g_drive.setEmergency(emergency);
  g_drive.setTargets(command.vx, command.vy, command.omega);
  g_drive.setHeading(g_imu.yawDeg(), g_imu.ready() && g_imu.calibrated());

  // Emergency stop and radio loss bypass the ramp and stop immediately.
  g_drive.update(micros(), emergency || linkLost);

  if (nowMs - g_lastTelemetryMs >= robot::drive_cfg::kTelemetryIntervalMs) {
    g_lastTelemetryMs = nowMs;
    robot::telemetry::printSeparator();
    robot::telemetry::printImu(g_imu);
    robot::telemetry::printDrive(g_drive, g_radio, linkLost);
  }
}
