#include "sensors/mpu6050.h"

#include <Arduino.h>
#include <Wire.h>

#include "config/imu_config.h"

namespace robot::sensors {
namespace {

int16_t toInt16(uint8_t highByte, uint8_t lowByte) {
  return static_cast<int16_t>((static_cast<uint16_t>(highByte) << 8) | lowByte);
}

}  // namespace

bool Mpu6050::begin() {
  Serial.println("Initializing MPU6050...");

  if (!present()) return false;

  uint8_t whoAmI = 0;
  if (!readRegisters(imu_cfg::kRegWhoAmI, &whoAmI, 1) ||
      whoAmI != imu_cfg::kWhoAmIValue) {
    return false;
  }

  // Device reset, wake from sleep with the X gyro PLL as clock source.
  if (!writeRegister(imu_cfg::kRegPwrMgmt1, 0x80)) return false;
  delay(150);
  if (!writeRegister(imu_cfg::kRegPwrMgmt1, 0x01)) return false;
  delay(20);

  // DLPF 3 (~44 Hz), +-2000 dps gyro, +-16 g accel, 100 Hz sample rate.
  const bool configured =
      writeRegister(imu_cfg::kRegPwrMgmt2, 0x00) &&
      writeRegister(imu_cfg::kRegSmplrtDiv, 9) &&
      writeRegister(imu_cfg::kRegConfig, 0x03) &&
      writeRegister(imu_cfg::kRegGyroConfig, 0x08) &&
      writeRegister(imu_cfg::kRegAccelConfig, 0x08);

  if (!configured) return false;
  delay(50);

  ImuSample probe{};
  return readSample(probe);
}

bool Mpu6050::present() {
  wire_.beginTransmission(imu_cfg::kMpuAddress);
  return wire_.endTransmission() == 0;
}

bool Mpu6050::writeRegister(uint8_t reg, uint8_t value) {
  wire_.beginTransmission(imu_cfg::kMpuAddress);
  wire_.write(reg);
  wire_.write(value);
  return wire_.endTransmission() == 0;
}

bool Mpu6050::readRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
  wire_.beginTransmission(imu_cfg::kMpuAddress);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;

  const uint8_t received = wire_.requestFrom(imu_cfg::kMpuAddress, length);
  if (received != length) {
    while (wire_.available()) wire_.read();
    return false;
  }

  for (uint8_t i = 0; i < length; ++i) {
    if (!wire_.available()) return false;
    data[i] = wire_.read();
  }
  return true;
}

bool Mpu6050::readSample(ImuSample &sample) {
  uint8_t data[14];
  if (!readRegisters(imu_cfg::kRegAccelXoutH, data, sizeof(data))) return false;

  const int16_t rawAx = toInt16(data[0], data[1]);
  const int16_t rawAy = toInt16(data[2], data[3]);
  const int16_t rawAz = toInt16(data[4], data[5]);
  const int16_t rawT = toInt16(data[6], data[7]);
  const int16_t rawGx = toInt16(data[8], data[9]);
  const int16_t rawGy = toInt16(data[10], data[11]);
  const int16_t rawGz = toInt16(data[12], data[13]);

  sample.ax = rawAx / imu_cfg::kAccelLsbPerG * imu_cfg::kGravityMps2;
  sample.ay = rawAy / imu_cfg::kAccelLsbPerG * imu_cfg::kGravityMps2;
  sample.az = rawAz / imu_cfg::kAccelLsbPerG * imu_cfg::kGravityMps2;
  sample.gx = rawGx / imu_cfg::kGyroLsbPerDps;
  sample.gy = rawGy / imu_cfg::kGyroLsbPerDps;
  sample.gz = rawGz / imu_cfg::kGyroLsbPerDps;
  sample.temperature = rawT / 333.87f + 21.0f;
  return true;
}

}  // namespace robot::sensors
