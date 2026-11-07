#pragma once

namespace robot {

// One MPU6050 sample in SI-ish units:
//   acceleration in m/s^2, angular rate in deg/s, temperature in degC.
struct ImuSample {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float temperature;
};

}  // namespace robot
