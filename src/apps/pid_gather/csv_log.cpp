#include "csv_log.h"

#include <cstdio>
#include <new>

#include "excitation.h"

namespace robot::pid_gather {

bool CsvLog::begin() {
  rows_ = new (std::nothrow) Row[cfg::kMaxCsvRows];
  if (rows_ == nullptr) {
    capacity_ = 0;
    return false;
  }

  capacity_ = cfg::kMaxCsvRows;
  reset();
  return true;
}

void CsvLog::reset() {
  count_ = 0;
  truncated_ = false;
}

void CsvLog::append(float dtMs, const int32_t counts[kWheelCount],
                    const WheelState wheels[kWheelCount]) {
  if (rows_ == nullptr) return;

  if (count_ >= capacity_) {
    truncated_ = true;
    return;
  }

  Row &row = rows_[count_++];
  row.dtMs = dtMs;
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    row.enc[i] = counts[i];
    row.pwm[i] = static_cast<int16_t>(wheels[i].outputPwm);
    row.tgtCps[i] = wheels[i].targetCps;
    row.measCps[i] = wheels[i].filteredCps;
  }
}

size_t CsvLog::header(char *out, size_t outSize) const {
  const int written = snprintf(
      out, outSize,
      "# mode=%s axis=%s amplitude=%d rows=%u truncated=%d\n"
      "dt_ms,encLF,encRF,encLB,encRB,pwmLF,pwmRF,pwmLB,pwmRB,"
      "tgtCpsLF,tgtCpsRF,tgtCpsLB,tgtCpsRB,"
      "measCpsLF,measCpsRF,measCpsLB,measCpsRB\n",
      modeName(kMode), axisName(kAxis), cfg::kAmplitudePwm,
      static_cast<unsigned>(count_), truncated_ ? 1 : 0);

  if (written <= 0 || static_cast<size_t>(written) >= outSize) return 0;
  return static_cast<size_t>(written);
}

size_t CsvLog::row(size_t index, char *out, size_t outSize) const {
  if (rows_ == nullptr || index >= count_) return 0;

  const Row &row = rows_[index];
  const int written = snprintf(
      out, outSize,
      "%.2f,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,"
      "%.1f,%.1f,%.1f,%.1f\n",
      row.dtMs,
      static_cast<long>(row.enc[robot::index(Wheel::kLeftFront)]),
      static_cast<long>(row.enc[robot::index(Wheel::kRightFront)]),
      static_cast<long>(row.enc[robot::index(Wheel::kLeftBack)]),
      static_cast<long>(row.enc[robot::index(Wheel::kRightBack)]),
      row.pwm[robot::index(Wheel::kLeftFront)], row.pwm[robot::index(Wheel::kRightFront)],
      row.pwm[robot::index(Wheel::kLeftBack)], row.pwm[robot::index(Wheel::kRightBack)],
      row.tgtCps[robot::index(Wheel::kLeftFront)],
      row.tgtCps[robot::index(Wheel::kRightFront)],
      row.tgtCps[robot::index(Wheel::kLeftBack)],
      row.tgtCps[robot::index(Wheel::kRightBack)],
      row.measCps[robot::index(Wheel::kLeftFront)],
      row.measCps[robot::index(Wheel::kRightFront)],
      row.measCps[robot::index(Wheel::kLeftBack)],
      row.measCps[robot::index(Wheel::kRightBack)]);

  if (written <= 0 || static_cast<size_t>(written) >= outSize) return 0;
  return static_cast<size_t>(written);
}

}  // namespace robot::pid_gather
