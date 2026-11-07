#pragma once

namespace robot::math {

inline float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

inline float moveToward(float current, float target, float maximumStep) {
  if (current < target) {
    const float next = current + maximumStep;
    return next < target ? next : target;
  }
  if (current > target) {
    const float next = current - maximumStep;
    return next > target ? next : target;
  }
  return target;
}

// Accelerates and decelerates at different rates. When the setpoint reverses
// sign the output is first brought back to zero at the deceleration rate.
inline float applyAsymmetricSlew(float current, float target,
                                 float accelerationRate, float decelerationRate,
                                 float dt) {
  if (current * target < 0.0f) {
    return moveToward(current, 0.0f, decelerationRate * dt);
  }

  const bool increasingMagnitude =
      (target < 0.0f ? -target : target) > (current < 0.0f ? -current : current);
  const float rate = increasingMagnitude ? accelerationRate : decelerationRate;
  return moveToward(current, target, rate * dt);
}

// Normalizes an angle to (-180, 180].
inline float wrapAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

}  // namespace robot::math
