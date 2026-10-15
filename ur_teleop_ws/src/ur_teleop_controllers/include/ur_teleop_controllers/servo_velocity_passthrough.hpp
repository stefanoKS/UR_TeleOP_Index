#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>

namespace ur_teleop_controllers {
// Humble Servo computes dq = (filtered(q_measured + dq_requested*dt) -
// q_measured)/dt. Filtering those absolute positions introduces delayed measured
// feedback into dq. Preserve the increment here; native Ruckig owns final joint
// velocity/acceleration/jerk limits. Only use with the supervised native chain.
class ServoVelocityPassthrough : public online_signal_smoothing::SmoothingBaseClass {
  bool initialized_ = false;
  void validate(const std::vector<double>& positions) const {
    // Humble Servo ignores doSmoothing/reset's return value. Throw rather than
    // allowing an invalid target through after a false return.
    if (!initialized_ || positions.size() != 6 ||
        !std::all_of(positions.begin(), positions.end(), [](double q) { return std::isfinite(q); }))
      throw std::runtime_error("ServoVelocityPassthrough requires six finite joint positions after initialization");
  }
public:
  bool initialize(rclcpp::Node::SharedPtr, moveit::core::RobotModelConstPtr, size_t num_joints) override {
    initialized_ = num_joints == 6;
    return initialized_;
  }
  bool doSmoothing(std::vector<double>& positions) override {
    validate(positions);
    return true;
  }
  bool reset(const std::vector<double>& positions) override {
    validate(positions);
    return true;
  }
};
}  // namespace ur_teleop_controllers
