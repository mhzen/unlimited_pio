// pid_gather tool application.
//
// Standalone system-identification firmware: excites one axis open-loop and
// logs per-wheel encoder speed at 50 Hz, then serves the CSV over a Wi-Fi
// SoftAP. No radio, no IMU, no operator input.
//
// Hardware and drive code are shared with the robot firmware (src/hal,
// src/control), so the RB sign/pin convention is exactly the robot's.

#include <Arduino.h>

#include "app/pid_gather_config.h"
#include "config/drive_config.h"
#include "control/mixer.h"
#include "control/wheel_pid.h"
#include "control/wheel_state.h"
#include "core/math_util.h"
#include "core/wheel.h"
#include "hal/encoder_bank.h"
#include "hal/motor_driver.h"

#include "csv_log.h"
#include "excitation.h"
#include "wifi_csv_server.h"

namespace pg = robot::pid_gather;
namespace cfg = robot::pid_gather::cfg;

namespace {

using robot::kWheelCount;
using robot::Wheel;
using robot::WheelState;
using pg::Axis;
using pg::CsvLog;
using pg::Excitation;
using pg::Mode;
using pg::RampProfile;
using pg::WifiCsvServer;
using pg::axisName;
using pg::modeName;

enum class Phase : uint8_t { kWait, kRun, kDone };

robot::hal::MotorDriver g_motors;
robot::hal::EncoderBank g_encoders;

CsvLog g_csv;
WifiCsvServer g_server;
Excitation g_excitation;
RampProfile g_ramp;
WheelState g_wheels[kWheelCount];

Phase g_phase = Phase::kWait;
uint32_t g_startMs = 0;
uint32_t g_lastControlUs = 0;
float g_excitationTime = 0.0f;
bool g_logReady = false;

const char *phaseName(Phase phase) {
  switch (phase) {
    case Phase::kWait:
      return "WAIT";
    case Phase::kRun:
      return "RUN";
    case Phase::kDone:
      return "DONE";
  }
  return "UNKNOWN";
}

void stopMotors() { g_motors.stopAll(); }

// Sends one excitation command through the shared mixer, then either straight
// through (open loop, used for sysid) or through the shared wheel PID.
void applyCommand(int command, float dt) {
  float vx = 0.0f;
  float vy = 0.0f;
  float omega = 0.0f;

  if constexpr (pg::kAxis == Axis::kVx) {
    vx = static_cast<float>(command);
  } else if constexpr (pg::kAxis == Axis::kVy) {
    vy = static_cast<float>(command);
  } else {
    omega = static_cast<float>(command);
  }

  float wheelPwm[kWheelCount];
  robot::mixWheels(vx, vy, omega, wheelPwm);

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    WheelState &wheel = g_wheels[i];
    wheel.targetPwm = wheelPwm[i];
    wheel.targetCps = wheel.targetPwm / robot::drive_cfg::kMaxMotorPwm *
                      robot::drive_cfg::kMaxWheelCps;

    if (cfg::kUsePid) {
      wheel.outputPwm = robot::calculateWheelPid(
          wheel, i, dt);
    } else {
      wheel.integral = 0.0f;
      wheel.previousError = 0.0f;
      wheel.outputPwm = static_cast<int>(roundf(wheel.targetPwm));
    }

    g_motors.set(static_cast<Wheel>(i), wheel.outputPwm);
  }
}

void startRun() {
  Serial.printf("TEST_START mode=%s axis=%s amplitude=%d\n", modeName(pg::kMode),
                axisName(pg::kAxis), cfg::kAmplitudePwm);

  // Zero the encoders and every speed filter so the CSV starts at rest.
  g_encoders.reset();
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    g_wheels[i] = WheelState{};
  }

  g_excitationTime = 0.0f;
  g_excitation.reset(millis());
  g_ramp.reset();

  g_lastControlUs = micros();
  g_phase = Phase::kRun;
}

bool durationReached() {
  switch (pg::kMode) {
    case Mode::kStep:
      return g_excitationTime >= cfg::kStepDurationS;
    case Mode::kChirp:
      return g_excitationTime >= cfg::kChirpDurationS;
    case Mode::kPrbs:
      return g_excitationTime >= cfg::kPrbsDurationS;
    default:
      return false;
  }
}

// Runs the fixed-rate control tick, if one is due.
void controlTick() {
  const uint32_t nowUs = micros();
  if (nowUs - g_lastControlUs < robot::drive_cfg::kControlIntervalUs) return;

  const float dt = robot::math::clampFloat(
      (nowUs - g_lastControlUs) * 1.0e-6f, 0.005f, 0.100f);
  g_lastControlUs = nowUs;

  int32_t counts[kWheelCount];
  g_encoders.readCounts(counts);

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    const int32_t delta =
        (counts[i] - g_wheels[i].previousCount) *
        robot::drive_cfg::kEncoderSign[i];
    g_wheels[i].previousCount = counts[i];
    g_wheels[i].measuredCps = static_cast<float>(delta) / dt;
    g_wheels[i].filteredCps +=
        robot::drive_cfg::kSpeedFilterAlpha *
        (g_wheels[i].measuredCps - g_wheels[i].filteredCps);
  }

  switch (g_phase) {
    case Phase::kWait:
      if (millis() - g_startMs >= cfg::kStartDelayMs) startRun();
      break;

    case Phase::kRun: {
      g_excitationTime += dt;

      if (pg::kMode == Mode::kRamp) {
        int32_t maxCount = 0;
        for (uint8_t i = 0; i < kWheelCount; ++i) {
          const int32_t magnitude = abs(counts[i]);
          if (magnitude > maxCount) maxCount = magnitude;
        }

        const bool requestDecel =
            !g_ramp.decelerating() &&
            (maxCount > cfg::kMaxEncoderDistance ||
             g_excitationTime >= cfg::kRampDurationS);
        applyCommand(g_ramp.update(dt, requestDecel), dt);

        if (g_ramp.finished()) {
          stopMotors();
          g_phase = Phase::kDone;
          Serial.printf("TEST_COMPLETE rows=%u\n",
                        static_cast<unsigned>(g_csv.rows()));
        }
      } else {
        applyCommand(g_excitation.value(g_excitationTime, millis()), dt);

        bool distanceExceeded = false;
        for (uint8_t i = 0; i < kWheelCount; ++i) {
          if (abs(counts[i]) > cfg::kMaxEncoderDistance) distanceExceeded = true;
        }

        if (distanceExceeded) {
          stopMotors();
          g_phase = Phase::kDone;
          Serial.println("SAFETY_STOP: max encoder distance exceeded");
        } else if (durationReached()) {
          stopMotors();
          g_phase = Phase::kDone;
          Serial.printf("TEST_COMPLETE rows=%u\n",
                        static_cast<unsigned>(g_csv.rows()));
        }
      }
      break;
    }

    case Phase::kDone:
      stopMotors();
      break;
  }

  // The wait phase is logged too: the tuning script uses that zero segment as
  // its baseline, and STEP runs need the 0 -> amplitude edge inside the file.
  if (g_phase == Phase::kWait || g_phase == Phase::kRun) {
    g_csv.append(dt * 1.0e3f, counts, g_wheels);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) {}

  Serial.println("\n=== Wireless PID Data Gathering (standalone, no radio) ===");
  Serial.printf("Test mode: %s\n", modeName(pg::kMode));
  Serial.printf("Excitation axis: %s\n", axisName(pg::kAxis));
  Serial.printf("Amplitude PWM: %d\n", cfg::kAmplitudePwm);
  Serial.printf("Start delay: %lu s\n",
                static_cast<unsigned long>(cfg::kStartDelayMs / 1000));
  Serial.printf("Max encoder distance: %ld counts\n",
                static_cast<long>(cfg::kMaxEncoderDistance));

  g_motors.begin();
  g_encoders.begin();
  stopMotors();

  int32_t counts[kWheelCount];
  g_encoders.readCounts(counts);
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    g_wheels[i].previousCount = counts[i];
  }

  // Allocate the CSV rows before Wi-Fi claims heap (and fragment it).
  g_logReady = g_csv.begin();
  if (!g_logReady) {
    Serial.println("ERROR: CSV buffer allocation failed; test disabled.");
    g_phase = Phase::kDone;
  }

  g_server.begin(&g_csv);
  Serial.printf("SoftAP: %s (pass %s)\n", cfg::kWifiSsid, cfg::kWifiPass);
  Serial.println("Open http://192.168.4.1/status or /csv from a phone/laptop.");

  g_lastControlUs = micros();
  g_startMs = millis();
  if (g_logReady) {
    g_phase = Phase::kWait;
    Serial.println("PHASE_WAIT: waiting for start delay...");
  }
}

void loop() {
  g_server.handleClient();
  controlTick();

  uint32_t startInS = 0;
  if (g_phase == Phase::kWait) {
    const uint32_t elapsedMs = millis() - g_startMs;
    startInS = elapsedMs >= cfg::kStartDelayMs
                   ? 0
                   : (cfg::kStartDelayMs - elapsedMs) / 1000;
  }
  g_server.setStatus(phaseName(g_phase), g_excitationTime, startInS);
}
