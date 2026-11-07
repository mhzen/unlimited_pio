#pragma once

#include <cstdint>

#include <RF24.h>
#include <SPI.h>

#include "config/pins.h"
#include "core/control_packet.h"

namespace robot::comms {

// nRF24L01 receiver: owns the RF24 instance, handles bring-up/retry, drains
// the RX FIFO and tracks link health.
class RadioLink {
 public:
  // `spi` must already be started with the custom pin mapping.
  void begin(SPIClass *spi);

  // Retries bring-up every kRetryIntervalMs while the radio is not answering.
  void service(uint32_t nowMs);

  // Drains everything in the RX FIFO. Returns true when a packet arrived.
  bool poll();

  const ControlPacket &packet() const { return packet_; }
  bool ready() const { return ready_; }

  // Clamped gear index from the last packet (0..kGearCount-1).
  uint8_t gearIndex() const;

  // True once packets have been seen and then went quiet for kLinkTimeoutMs.
  bool linkLost(uint32_t nowMs) const;

 private:
  bool initialize();

  RF24 radio_{pins::kRadioCe, pins::kRadioCsn};
  SPIClass *spi_ = nullptr;
  ControlPacket packet_{};
  bool ready_ = false;
  uint32_t lastPacketMs_ = 0;
  uint32_t lastRetryMs_ = 0;
};

}  // namespace robot::comms
