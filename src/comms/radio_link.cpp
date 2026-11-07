#include "comms/radio_link.h"

#include <Arduino.h>

#include "config/drive_config.h"
#include "config/radio_config.h"

namespace robot::comms {

void RadioLink::begin(SPIClass *spi) {
  spi_ = spi;
  ready_ = initialize();
}

bool RadioLink::initialize() {
  Serial.println("Initializing nRF24L01...");

  // RF24 1.6 does not start the bus when handed an SPIClass: the caller (see
  // main.cpp) applies the custom SCK/MISO/MOSI mapping first.
  if (!radio_.begin(spi_)) return false;

  radio_.stopListening();
  radio_.setChannel(radio_cfg::kChannel);
  radio_.setDataRate(RF24_1MBPS);
  radio_.setPALevel(RF24_PA_MAX);
  // radio_.setPayloadSize(sizeof(ControlPacket));  // must match the transmitter
  radio_.setAutoAck(true);
  radio_.openReadingPipe(1, radio_cfg::kAddress);
  radio_.flush_rx();
  radio_.startListening();
  return true;
}

void RadioLink::service(uint32_t nowMs) {
  if (ready_) return;
  if (nowMs - lastRetryMs_ < radio_cfg::kRetryIntervalMs) return;

  lastRetryMs_ = nowMs;
  ready_ = initialize();
}

bool RadioLink::poll() {
  if (!ready_) return false;

  bool received = false;
  while (radio_.available()) {
    radio_.read(&packet_, sizeof(packet_));
    received = true;
  }

  if (!received) return false;

  lastPacketMs_ = millis();
  return true;
}

uint8_t RadioLink::gearIndex() const {
  return static_cast<uint8_t>(
      constrain(packet_.gear, 0, drive_cfg::kGearCount - 1));
}

bool RadioLink::linkLost(uint32_t nowMs) const {
  return ready_ && lastPacketMs_ != 0 &&
         (nowMs - lastPacketMs_) > radio_cfg::kLinkTimeoutMs;
}

}  // namespace robot::comms
