#pragma once

#include <cstddef>
#include <cstdint>

// Sysid tool defaults. Override per run with build flags, e.g.
//   build_flags = -DPID_GATHER_MODE=2 -DPID_GATHER_AXIS=1
#ifndef PID_GATHER_MODE
#define PID_GATHER_MODE 2  // 0=STEP 1=CHIRP 2=PRBS 3=RAMP
#endif
#ifndef PID_GATHER_AXIS
#define PID_GATHER_AXIS 1  // 0=VX 1=VY 2=OMEGA
#endif
#ifndef PID_GATHER_USE_PID
#define PID_GATHER_USE_PID 0  // 1 = log the closed loop (validation only)
#endif

namespace robot::pid_gather::cfg {

constexpr uint8_t kMode = PID_GATHER_MODE;
constexpr uint8_t kAxis = PID_GATHER_AXIS;
constexpr bool kUsePid = (PID_GATHER_USE_PID != 0);

static_assert(kMode <= 3, "PID_GATHER_MODE must be 0..3 (STEP/CHIRP/PRBS/RAMP)");
static_assert(kAxis <= 2, "PID_GATHER_AXIS must be 0..2 (VX/VY/OMEGA)");

// -----------------------------------------------------------------------------
// Run profile
// -----------------------------------------------------------------------------
constexpr uint32_t kStartDelayMs = 5000;
constexpr int kAmplitudePwm = 200;
constexpr int32_t kMaxEncoderDistance = 3000;

constexpr float kRampAccelPwmPerSec = 127.5f;
constexpr float kRampDecelPwmPerSec = 127.5f;
constexpr float kRampDurationS = 5.0f;

constexpr float kStepDurationS = 10.0f;

constexpr float kChirpStartHz = 0.5f;
constexpr float kChirpEndHz = 10.0f;
constexpr float kChirpDurationS = 15.0f;

constexpr float kPrbsDurationS = 15.0f;
constexpr uint32_t kPrbsBitPeriodMs = 200;
constexpr uint16_t kPrbsSeed = 0x5A5A;

// Rows cover the start delay plus the run: 5 s wait + 15 s test at 50 Hz is
// ~1000 rows, 1280 x 60 B = 77 kB leaves margin.
constexpr size_t kMaxCsvRows = 1280;

// -----------------------------------------------------------------------------
// Wi-Fi access point (the only data path; no USB needed during a run)
// -----------------------------------------------------------------------------
constexpr char kWifiSsid[] = "pidtune_forklift";
constexpr char kWifiPass[] = "pidtune123";
constexpr uint16_t kWifiPort = 80;

}  // namespace robot::pid_gather::cfg
