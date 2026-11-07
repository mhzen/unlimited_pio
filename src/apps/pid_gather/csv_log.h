#pragma once

#include <cstddef>
#include <cstdint>

#include "app/pid_gather_config.h"
#include "control/wheel_state.h"
#include "core/wheel.h"

namespace robot::pid_gather {

// Bounded CSV row store. Rows are captured in RAM at 50 Hz during a run and
// formatted on demand when /csv is requested, which keeps memory predictable
// (no growing Arduino String, no flash writes inside the control loop).
class CsvLog {
 public:
  // Allocates the row buffer. Call once before starting Wi-Fi.
  bool begin();

  void reset();

  void append(float dtMs, const int32_t counts[kWheelCount],
              const WheelState wheels[kWheelCount]);

  size_t rows() const { return count_; }
  size_t capacity() const { return capacity_; }
  bool truncated() const { return truncated_; }

  // "# mode=... axis=..." comment lines plus the column header.
  size_t header(char *out, size_t outSize) const;

  // Formats one row; returns bytes written, 0 when the index is out of range.
  size_t row(size_t index, char *out, size_t outSize) const;

 private:
  // Must stay small: 1024 rows x 60 B = 61 kB.
  struct Row {
    float dtMs;
    int32_t enc[kWheelCount];
    int16_t pwm[kWheelCount];
    float tgtCps[kWheelCount];
    float measCps[kWheelCount];
  };

  Row *rows_ = nullptr;
  size_t capacity_ = 0;
  size_t count_ = 0;
  bool truncated_ = false;
};

}  // namespace robot::pid_gather
