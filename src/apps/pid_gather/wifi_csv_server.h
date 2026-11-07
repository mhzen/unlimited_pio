#pragma once

#include <WebServer.h>
#include <WiFi.h>

#include "app/pid_gather_config.h"
#include "csv_log.h"

namespace robot::pid_gather {

// SoftAP + HTTP endpoints:
//   /        index page
//   /status  phase / elapsed / sample count
//   /csv     the current run's CSV, streamed in chunks
class WifiCsvServer {
 public:
  void begin(CsvLog *log);

  // Snapshot used by /status; cheap to call every control tick.
  void setStatus(const char *phase, float elapsedS, uint32_t startInS);

  void handleClient();

 private:
  void sendIndex();
  void sendStatus();
  void sendCsv();

  WebServer server_{cfg::kWifiPort};
  CsvLog *log_ = nullptr;
  const char *phase_ = "WAIT";
  float elapsedS_ = 0.0f;
  uint32_t startInS_ = 0;
};

}  // namespace robot::pid_gather
