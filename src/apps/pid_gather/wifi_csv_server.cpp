#include "wifi_csv_server.h"

#include <cstring>

#include "excitation.h"

namespace robot::pid_gather {
namespace {

constexpr size_t kLineBufferSize = 192;

}  // namespace

void WifiCsvServer::begin(CsvLog *log) {
  log_ = log;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(cfg::kWifiSsid, cfg::kWifiPass);

  server_.on("/", HTTP_GET, [this]() { sendIndex(); });
  server_.on("/status", HTTP_GET, [this]() { sendStatus(); });
  server_.on("/csv", HTTP_GET, [this]() { sendCsv(); });
  server_.begin();
}

void WifiCsvServer::setStatus(const char *phase, float elapsedS,
                              uint32_t startInS) {
  phase_ = phase;
  elapsedS_ = elapsedS;
  startInS_ = startInS;
}

void WifiCsvServer::handleClient() { server_.handleClient(); }

void WifiCsvServer::sendIndex() {
  String page = "<html><body><h3>Wireless PID Data Gathering</h3>";
  page += "<p>mode=";
  page += modeName(kMode);
  page += " axis=";
  page += axisName(kAxis);
  page += "</p>";
  page += "<p><a href=\"/status\">/status</a> - current phase</p>";
  page += "<p><a href=\"/csv\">/csv</a> - download this run's CSV</p>";
  page += "</body></html>";
  server_.send(200, "text/html", page);
}

void WifiCsvServer::sendStatus() {
  String body;
  body += "phase=" + String(phase_) + "\n";
  body += "mode=" + String(modeName(kMode)) + "\n";
  body += "axis=" + String(axisName(kAxis)) + "\n";
  body += "amplitude_pwm=" + String(cfg::kAmplitudePwm) + "\n";
  body += "elapsed_s=" + String(elapsedS_, 1) + "\n";
  body += "csv_rows=" + String(log_ ? static_cast<unsigned>(log_->rows()) : 0U) + "\n";
  body += "csv_capacity=" +
          String(log_ ? static_cast<unsigned>(log_->capacity()) : 0U) + "\n";
  body += "truncated=" + String(log_ && log_->truncated() ? 1 : 0) + "\n";
  if (strcmp(phase_, "WAIT") == 0) {
    body += "start_in_s=" + String(startInS_) + "\n";
  }
  server_.send(200, "text/plain", body);
}

void WifiCsvServer::sendCsv() {
  if (log_ == nullptr) {
    server_.send(500, "text/plain", "no log buffer\n");
    return;
  }

  const String disposition =
      "attachment; filename=\"pid_data_" + String(modeName(kMode)) + "_" +
      String(axisName(kAxis)) + ".csv\"";
  server_.sendHeader("Content-Disposition", disposition);

  // Unknown length -> chunked transfer, so the CSV never has to exist in RAM
  // as one contiguous string.
  server_.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server_.send(200, "text/csv", "");

  char line[kLineBufferSize];
  if (log_->header(line, sizeof(line)) > 0) {
    server_.sendContent(line);
  }
  for (size_t i = 0; i < log_->rows(); ++i) {
    if (log_->row(i, line, sizeof(line)) > 0) {
      server_.sendContent(line);
    }
  }
  server_.sendContent("");  // terminate the chunked body
}

}  // namespace robot::pid_gather
