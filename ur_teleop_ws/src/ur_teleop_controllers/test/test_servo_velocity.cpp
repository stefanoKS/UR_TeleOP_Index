#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <ur_teleop_controllers/servo_velocity_passthrough.hpp>
#include <ur_teleop_controllers/smoothing.hpp>
#include <limits>

TEST(ServoVelocity, PluginLoadsAndRejectsInvalidState) {
  pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass> loader(
      "moveit_core", "online_signal_smoothing::SmoothingBaseClass");
  auto plugin = loader.createSharedInstance("ur_teleop_controllers::ServoVelocityPassthrough");
  std::vector<double> q(6, 0.);
  EXPECT_THROW(plugin->doSmoothing(q), std::runtime_error);
  EXPECT_FALSE(plugin->initialize(nullptr, nullptr, 5));
  EXPECT_THROW(plugin->reset(q), std::runtime_error);
  ASSERT_TRUE(plugin->initialize(nullptr, nullptr, 6));
  EXPECT_TRUE(plugin->reset(q));
  q.pop_back();
  EXPECT_THROW(plugin->doSmoothing(q), std::runtime_error);
  q.resize(6, std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(plugin->reset(q), std::runtime_error);
  q.back() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(plugin->doSmoothing(q), std::runtime_error);
  q.assign(6, 0.);
  EXPECT_TRUE(plugin->reset(q));
  EXPECT_TRUE(plugin->doSmoothing(q));
}

TEST(ServoVelocity, MovingFeedbackCannotReverseDemandAndNativeLimitsRemain) {
  ur_teleop_controllers::ServoVelocityPassthrough plugin;
  ASSERT_TRUE(plugin.initialize(nullptr, nullptr, 6));
  ur_teleop_controllers::Smoothing native;
  ASSERT_TRUE(native.configure(.5, 1., 10.));
  ASSERT_TRUE(native.reset({}, {}));
  std::vector<double> target(6, 0.);
  ASSERT_TRUE(plugin.reset(target));
  ur_teleop_controllers::Smoothing::Vector q{}, dq{};
  for (int k = 0; k < 3000; ++k) {
    if (k % 5 == 0) {
      // Moving feedback must not contaminate Servo's requested velocity.
      const double measured = .004 * std::sin(2.*3.141592653589793*2.6*k*.002);
      const double requested = k < 1500 ? .1 : 0.;
      target.assign(6, measured + requested*.01);
      ASSERT_TRUE(plugin.doSmoothing(target));
      for (size_t j = 0; j < 6; ++j) {
        q[j] = target[j];
        dq[j] = (target[j] - measured)/.01;
        EXPECT_NEAR(dq[j], requested, 1e-12);
      }
    }
    const auto previous_a = native.input.current_acceleration;
    ASSERT_TRUE(native.step(q, .002, dq, 1., true));
    for (size_t j = 0; j < 6; ++j) {
      EXPECT_GE(native.input.current_velocity[j], -1e-10);
      EXPECT_LE(std::abs(native.input.current_velocity[j]), .5 + 1e-10);
      EXPECT_LE(std::abs(native.input.current_acceleration[j]), 1. + 1e-10);
      EXPECT_LE(std::abs(native.input.current_acceleration[j]-previous_a[j]), .02 + 1e-10);
      if (k > 2000) EXPECT_NEAR(native.input.current_velocity[j], 0., 1e-10);
    }
  }
  // Re-engagement has no retained position filter history.
  target.assign(6, -1.);
  ASSERT_TRUE(plugin.reset(target));
  ASSERT_TRUE(plugin.doSmoothing(target));
  for (double v : target) EXPECT_DOUBLE_EQ(v, -1.);
}
