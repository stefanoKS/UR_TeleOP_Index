#pragma once
#include <array>
#include <algorithm>
#include <cmath>

namespace ur_teleop_controllers {
// Called once per newly received trajectory, never once per repeated control tick.
class TargetHistory {
public:
  using Vector = std::array<double,6>;
  Vector previous{};
  double received_at = 0.;
  bool ready = false;
  void reset() {ready=false;}
  bool observe(const Vector& q, double now, double max_velocity, Vector& velocity) {
    const double dt=now-received_at;
    const bool valid=ready && std::isfinite(dt) && dt>=.001 && dt<=.1;
    if(valid) for(size_t i=0;i<6;++i) velocity[i]=std::clamp((q[i]-previous[i])/dt,-max_velocity,max_velocity);
    previous=q; received_at=now; ready=true;
    return valid;
  }
};
}
