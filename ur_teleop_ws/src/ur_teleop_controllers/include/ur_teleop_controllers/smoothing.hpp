#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <ruckig/ruckig.hpp>

namespace ur_teleop_controllers {
// No ROS, allocation, angle wrapping, or measurement re-anchoring in the update path.
// Restored streaming velocity ramp. A short endpoint with a large terminal
// velocity can make position-mode OTG move backward to attain that state.
// Keep bounded derivatives; physical sway still requires validation.
class Smoothing {
public:
  using Vector = std::array<double, 6>;
  ruckig::InputParameter<6> input;
  ruckig::OutputParameter<6> output;
  ruckig::Ruckig<6> solver{.002};
  bool ready = false;
  const char* last_error = "none";
  int error_joint = -1;
  int result_code = 0;
  double max_velocity = .5, max_acceleration = 1., max_jerk = 10.;

  bool configure(double velocity, double acceleration, double jerk) {
    if (!std::isfinite(velocity) || !std::isfinite(acceleration) || !std::isfinite(jerk) ||
        velocity <= 0 || acceleration <= 0 || jerk <= 0) return false;
    max_velocity = velocity; max_acceleration = acceleration; max_jerk = jerk;
    input.max_velocity.fill(velocity);
    input.max_acceleration.fill(acceleration);
    input.max_jerk.fill(jerk);
    input.target_velocity.fill(0.);
    input.target_acceleration.fill(0.);
    input.control_interface = ruckig::ControlInterface::Velocity;
    input.synchronization = ruckig::Synchronization::None;
    return true;
  }

  bool reset(const Vector& q, const Vector& v) {
    ready = false;
    for (size_t i=0; i<6; ++i)
      if (!std::isfinite(q[i]) || !std::isfinite(v[i]) || std::abs(v[i]) > max_velocity) return false;
    input.current_position = q;
    input.current_velocity = v;
    input.current_acceleration.fill(0.);
    input.target_position = q;
    solver.reset();
    ready = true;
    return true;
  }

  bool step(const Vector& target, double dt, const Vector& feedforward = {}, double scale = 1., bool velocity_supplied = false) {
    last_error = "none"; error_joint = -1; result_code = 0;
    if (!ready) {last_error="not initialized"; return false;}
    if (!std::isfinite(dt) || dt <= 0 || dt > .02) {last_error="control period outside (0, 20 ms]"; return false;}
    if (!std::isfinite(scale) || scale < 0 || scale > 1.) {last_error="invalid speed scaling"; return false;}
    for (size_t i=0; i<6; ++i) {
      if (!std::isfinite(target[i]) || !std::isfinite(feedforward[i])) {
        last_error="nonfinite trajectory input"; error_joint=static_cast<int>(i); return false;
      }
      // Servo already closes its position loop against measured feedback. Do
      // not add a competing correction against the internal ramp reference.
      // Position-only legacy commands retain their previous position fallback.
      input.target_velocity[i] = std::clamp(
        scale * (feedforward[i] + (velocity_supplied ? 0. : 5.0 * (target[i] - input.current_position[i]))),
        -max_velocity*scale, max_velocity*scale);
    }
    solver.delta_time = dt;
    const auto result = solver.update(input, output);
    result_code = static_cast<int>(result);
    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
      last_error="Ruckig rejected input"; return false;
    }
    for (size_t i=0; i<6; ++i) {
      if (!std::isfinite(output.new_position[i]) || !std::isfinite(output.new_velocity[i]) ||
          !std::isfinite(output.new_acceleration[i]) ||
          std::abs(output.new_velocity[i]) > max_velocity+1e-8 ||
          std::abs(output.new_acceleration[i]) > max_acceleration+1e-8 ||
          std::abs(output.new_acceleration[i]-input.current_acceleration[i]) > max_jerk*dt+1e-7)
        {last_error="output derivative bound or finite-value check failed"; error_joint=static_cast<int>(i); return false;}
    }
    output.pass_to_input(input);
    return true;
  }
};
}
