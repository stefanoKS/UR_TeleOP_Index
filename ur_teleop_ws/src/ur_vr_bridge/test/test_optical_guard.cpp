#include "ur_vr_bridge/optical_guard.hpp"
#include <iostream>
#include <stdexcept>
void require(bool b) { if (!b) throw std::runtime_error("optical guard assertion failed"); }
int main() {
  ur_vr_bridge::OpticalGuard g;
  require(!g.fresh(10));
  for (int sensor=0; sensor<4; ++sensor) for (int plane=0; plane<2; ++plane)
    g.observe(0,sensor,plane,100,10);
  require(!g.fresh(10.01)); // One station is insufficient.
  for (int sensor=0; sensor<4; ++sensor) g.observe(1,sensor,0,100,10);
  require(!g.fresh(10.01)); // Both planes are required.
  for (int sensor=0; sensor<4; ++sensor) g.observe(1,sensor,1,100,10);
  require(g.fresh(10.01));
  require(!g.fresh(9.99));
  require(g.fresh(10.16)); // Brief light gaps are tolerated.
  require(g.fresh(10.24));
  require(!g.fresh(10.26)); // IMU/pose arrivals cannot refresh light.
  for (int lh=0; lh<2; ++lh) for (int s=0; s<4; ++s) for (int p=0; p<2; ++p)
    g.observe(lh,s,p,100,10.3);
  require(!g.fresh(10.31)); // Repeated optical timestamps also cannot refresh.
  for (int lh=0; lh<2; ++lh) for (int s=0; s<4; ++s) for (int p=0; p<2; ++p)
    g.observe(lh,s,p,101,10.4);
  require(g.fresh(10.41));
  g.observe(-1,0,0,200,10.4); g.observe(0,32,0,200,10.4);
  require(g.counts(10.41).size()==2);
  std::cout << "OpticalGuard checks passed\n";
}
