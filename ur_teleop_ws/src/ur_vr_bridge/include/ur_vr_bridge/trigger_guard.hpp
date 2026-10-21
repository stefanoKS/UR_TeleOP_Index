#pragma once
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace ur_vr_bridge {
// One selected device, externally synchronized. No synthetic Joy values enter
// this cache. IMU activity is additional continuity evidence, not an optical bypass.
class TriggerGuard {
 public:
  static constexpr double timeout = .25;
  void imu(uint32_t clock, double now) {
    if (!imu_seq || now - imu_at >= timeout || now < imu_at) {
      invalidate(); continuity_since = now;
    }
    // Modular forward ordering accepts hardware-clock wrap, not duplicates/replay.
    if (imu_seq && static_cast<int32_t>(clock - imu_clock) <= 0) return;
    imu_clock = clock; imu_at = now; ++imu_seq;
  }
  void invalidate() { if (trusted) ++loss_seq; trusted = false; }
  // Accept the libsurvive button-event shape directly. Other axes never refresh trigger provenance.
  template<class Event>
  void button(const Event& event, double source_now, double now, int64_t receipt_ns) {
    for (unsigned i = 0; i < event.axis_count && i < 8; ++i)
      if (event.axis_ids[i] == 1) axis(event.axis_val[i], event.time, source_now, now, receipt_ns);
  }
  void axis(double value, double source_time, double source_now,
            double now, int64_t receipt_ns) {
    const double age = source_now - source_time;
    if (!std::isfinite(value) || value < 0 || value > 1 ||
        !std::isfinite(age) || !std::isfinite(source_time) || source_time <= event_time ||
        age < 0 || age >= timeout) {
      ++rejected; invalidate(); return;
    }
    raw = value; event_time = source_time; event_at = now - age;
    receipt = receipt_ns; ++event_seq;
    if (imu_seq && now - imu_at >= 0 && now - imu_at < timeout &&
        event_at >= continuity_since) trusted = true;
    else invalidate();
  }
  std::string message(double now, int64_t stamp, const std::string& session,
                      const std::string& identity, bool pose, bool optical, bool running) {
    if (!running || !imu_seq || now - imu_at >= timeout || now < imu_at) invalidate();
    const bool alive = running && imu_seq && now - imu_at >= 0 && now - imu_at < timeout;
    const bool valid = trusted && alive && pose && optical;
    if (last_valid && !valid) ++loss_seq;
    last_valid = valid;
    const char* state = !alive ? "DISCONNECTED_OR_STALE" : !valid ? "TEMPORARILY_UNVERIFIED" :
        event_seq != published_event ? "GENUINE_FRESH_INPUT" : "VALID_HELD_INPUT";
    std::ostringstream raw_json;
    if (event_seq) raw_json << std::setprecision(17) << raw;
    else raw_json << "null";
    std::ostringstream s;
    s << std::setprecision(17) << "{\"schema\":1,\"session\":\"" << hex(session)
      << "\",\"device\":\"" << hex(identity) << "\",\"stamp_ns\":" << stamp
      << ",\"observation_seq\":" << ++observation_seq
      << ",\"loss_seq\":" << loss_seq
      << ",\"event_seq\":" << event_seq << ",\"event_time\":" << event_time
      << ",\"receipt_ns\":" << receipt << ",\"event_age_s\":" << (event_seq ? now-event_at : -1)
      << ",\"raw\":" << raw_json.str()
      << ",\"imu_seq\":" << imu_seq << ",\"imu_age_s\":" << (imu_seq ? now-imu_at : -1)
      << ",\"pose\":" << (pose ? "true" : "false")
      << ",\"optical\":" << (optical ? "true" : "false")
      << ",\"state\":\"" << state << "\",\"rejected_events\":" << rejected << "}";
    published_event = event_seq;
    return s.str();
  }
 private:
  static std::string hex(const std::string& text) {
    static const char* digits = "0123456789abcdef";
    std::string result;
    for (unsigned char c : text) { result += digits[c >> 4]; result += digits[c & 15]; }
    return result;
  }
  double raw = 0, event_time = 0, event_at = 0, imu_at = 0, continuity_since = 0;
  int64_t receipt = 0;
  uint32_t imu_clock = 0;
  uint64_t event_seq = 0, imu_seq = 0, observation_seq = 0, published_event = 0, rejected = 0;
  bool trusted = false, last_valid = false;
  uint64_t loss_seq = 0;
};
}  // namespace ur_vr_bridge
