#pragma once
#include <array>
#include <cmath>
#include <limits>

namespace ur_vr_bridge {
// Observation validity only. This is NOT an optical-tracking confidence test.
class PoseGuard {
 public:
  bool accept(double source_time, const std::array<double, 3>& xyz,
              const std::array<double, 4>& wxyz, double steady_now,
              std::array<double, 4>& xyzw) {
    if (!std::isfinite(source_time) || source_time <= 0 ||
        !std::isfinite(steady_now) || source_time <= last_source_) return false;
    for (double value : xyz) if (!std::isfinite(value)) return false;
    double n = 0;
    for (double value : wxyz) {
      if (!std::isfinite(value)) return false;
      n = std::hypot(n, value);
    }
    // Reject uninitialized or implausibly non-unit orientations.
    if (n < 0.5 || n > 1.5) return false;
    xyzw = {wxyz[1]/n, wxyz[2]/n, wxyz[3]/n, wxyz[0]/n};
    last_source_ = source_time;
    last_arrival_ = steady_now;
    return true;
  }
  bool fresh(double steady_now, double timeout) const {
    return std::isfinite(last_arrival_) && steady_now >= last_arrival_ &&
           steady_now - last_arrival_ < timeout;
  }
 private:
  double last_source_ = -std::numeric_limits<double>::infinity();
  double last_arrival_ = -std::numeric_limits<double>::infinity();
};
}  // namespace ur_vr_bridge
