#include "ur_vr_bridge/optical_guard.hpp"
#include <iostream>
#include <stdexcept>
void require(bool b) { if (!b) throw std::runtime_error("optical guard assertion failed"); }
int main() {
  for (int station : {0, 1}) {
    ur_vr_bridge::OpticalGuard only;
    only.observe(station, 0, 0, 1, 1.);
    only.observe(station, 0, 1, 1, 1.);
    require(only.fresh(1.01) && only.strict(1.01));
  }
  ur_vr_bridge::OpticalGuard remaining;
  for (int lh : {0, 1}) for (int plane : {0, 1}) remaining.observe(lh, 0, plane, 1, 1.);
  for (int plane : {0, 1}) remaining.observe(1, 0, plane, 2, 1.2);
  require(remaining.fresh(1.3)); // Station 0 expires; station 1 remains.
  require(remaining.counts(1.3).at(0) == 0);
  require(!remaining.fresh(1.5));
  ur_vr_bridge::OpticalGuard single;
  single.observe(1, 0, 0, 1, 1.);
  require(!single.fresh(1.01));
  single.observe(1, 0, 1, 1, 1.);
  require(single.fresh(1.01));
  require(single.counts(1.01).at(1) == 1);
  require(single.strict(1.01));
  single.observe(1, 0, 1, 1, 1.2);
  require(!single.fresh(1.26));
  ur_vr_bridge::OpticalGuard g;
  require(!g.fresh(10));
  for (int sensor=0; sensor<4; ++sensor) for (int plane=0; plane<2; ++plane)
    g.observe(0,sensor,plane,100,10);
  require(g.fresh(10.01)); // Signal from one station is sufficient.
  require(g.strict(10.01)); // Legacy policy also accepts either station.
  for (int sensor=0; sensor<4; ++sensor) g.observe(1,sensor,0,100,10);
  require(g.fresh(10.01)); // Station 0 remains visible.
  require(g.strict(10.01)); // Station 0 is sufficient.
  for (int sensor=0; sensor<4; ++sensor) g.observe(1,sensor,1,100,10);
  require(g.fresh(10.01));
  require(g.strict(10.01));
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
