#include "ur_vr_bridge/pose_guard.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  using ur_vr_bridge::PoseGuard;
  PoseGuard guard;
  std::array<double, 4> q{};
  require(!guard.fresh(10, .25), "uninitialized stream must not be fresh");
  require(!guard.accept(1, {0,0,0}, {0,0,0,0}, 10, q), "zero quaternion accepted");
  const double s = std::sqrt(.5);
  require(guard.accept(1, {1,2,3}, {s,0,0,s}, 10, q), "valid 90-degree rotation rejected");
  require(std::abs(q[2]-s)<1e-12 && std::abs(q[3]-s)<1e-12 && q[0]==0,
          "wxyz -> ROS xyzw conversion failed");
  require(guard.fresh(10.1,.25), "recent pose considered stale");
  require(!guard.accept(1,{0,0,0},{1,0,0,0},10.2,q), "duplicate timestamp accepted");
  require(!guard.fresh(10.26,.25), "duplicate pose renewed freshness");
  require(!guard.accept(.9,{0,0,0},{1,0,0,0},10.3,q), "out-of-order timestamp accepted");
  require(!guard.accept(2,{NAN,0,0},{1,0,0,0},10.4,q), "NaN position accepted");
  require(!guard.accept(2,{0,0,0},{INFINITY,0,0,0},10.4,q), "infinite orientation accepted");
  require(guard.accept(2,{0,0,0},{1.01,0,0,0},10.5,q), "near-unit quaternion rejected");
  require(std::abs(q[3]-1)<1e-12, "normalization failed");
  require(!guard.fresh(10.4,.25), "backwards steady time accepted");
  require(guard.fresh(10.6,.25), "stream did not recover on new sample");
  std::cout << "PoseGuard checks passed\n";
}
