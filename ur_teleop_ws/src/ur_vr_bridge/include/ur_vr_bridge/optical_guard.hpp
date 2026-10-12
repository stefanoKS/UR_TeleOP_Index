#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>

namespace ur_vr_bridge {
// Recent accepted Gen2 optical observations, NOT pose accuracy/confidence.
// Caller serializes access between the libsurvive callback and ROS timer.
class OpticalGuard {
 public:
  static constexpr int sensors = 32;
  static constexpr double timeout = .25;
  void observe(int lighthouse, int sensor, int plane, uint64_t source, double now) {
    if (lighthouse < 0 || sensor < 0 || sensor >= sensors || plane < 0 || plane > 1 ||
        source == 0 || !std::isfinite(now)) return;
    auto& sample = samples_[lighthouse][sensor][plane];
    if (source <= sample.source) return;  // Never renew on duplicate/reordered light.
    sample = {source, now};
  }
  std::map<int, int> counts(double now) const {
    std::map<int, int> result;
    for (const auto& station : samples_) {
      int n = 0;
      for (const auto& sensor : station.second) {
        bool pair = true;
        for (const auto& sample : sensor) {
          pair = pair && sample.source > 0 && std::isfinite(now) &&
                 now >= sample.arrival && now - sample.arrival < timeout;
        }
        if (pair) ++n;
      }
      result[station.first] = n;
    }
    return result;
  }
  // Compatibility alias: station count is never a motion prerequisite.
  bool strict(double now) const { return fresh(now); }
  bool fresh(double now) const {
    int stations = 0;
    for (const auto& entry : counts(now)) if (entry.second >= 1) ++stations;
    return stations >= 1;
  }
 private:
  struct Sample { uint64_t source = 0; double arrival = -std::numeric_limits<double>::infinity(); };
  std::map<int, std::array<std::array<Sample, 2>, sensors>> samples_;
};
}
