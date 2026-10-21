#include "ur_vr_bridge/trigger_guard.hpp"
#include <cassert>
#include <string>
using ur_vr_bridge::TriggerGuard;
int main() {
  TriggerGuard g;
  g.imu(100, 1.); g.axis(.7, 10., 10., 1., 1000000000);
  auto read = [&](double t, bool pose=true) { return g.message(t, int64_t(t*1e9), "test", "KN0:serial", pose, pose, true); };
  auto first = read(1.);
  assert(first.find("GENUINE_FRESH_INPUT") != std::string::npos);
  g.imu(101, 1.1);
  auto lost = read(1.1, false);
  assert(lost.find("TEMPORARILY_UNVERIFIED") != std::string::npos);
  assert(lost.find("\"raw\":0.69999999999999996") != std::string::npos);
  g.imu(102, 1.2);
  auto held = read(1.2);
  assert(held.find("VALID_HELD_INPUT") != std::string::npos);
  assert(held.find("\"event_seq\":1") != std::string::npos);
  assert(held.find("\"event_time\":10") != std::string::npos);
  assert(read(1.5).find("DISCONNECTED_OR_STALE") != std::string::npos);
  g.imu(103, 1.51);
  assert(read(1.51).find("TEMPORARILY_UNVERIFIED") != std::string::npos);
  g.axis(.3, 10.8, 10.9, 1.515, 1515000000); // queued before device continuity restarted
  assert(read(1.515).find("TEMPORARILY_UNVERIFIED") != std::string::npos);
  g.axis(.4, 11., 11., 1.52, 1520000000);
  assert(read(1.52).find("GENUINE_FRESH_INPUT") != std::string::npos);
  g.axis(.1, 10.9, 11., 1.53, 1530000000); // old event invalidates, never overwrites cache
  assert(read(1.53).find("TEMPORARILY_UNVERIFIED") != std::string::npos);
  g.axis(.0, 11.1, 11.1, 1.54, 1540000000);
  assert(read(1.54).find("GENUINE_FRESH_INPUT") != std::string::npos);
  g.imu(103, 1.7); // duplicate hardware time cannot renew activity
  assert(read(1.8).find("DISCONNECTED_OR_STALE") != std::string::npos);
}
