#pragma once

#include <cstdint>

namespace robot::radio_cfg {

constexpr uint8_t kChannel = 100;

// Must match the transmitter.
constexpr uint8_t kAddress[6] = {'0', '0', '0', '0', '0', '1'};

// A packet older than this is treated as a lost link (=> emergency stop).
constexpr uint32_t kLinkTimeoutMs = 500;

// How often to retry radio bring-up while it is not answering.
constexpr uint32_t kRetryIntervalMs = 2000;

}  // namespace robot::radio_cfg
