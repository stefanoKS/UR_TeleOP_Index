#include <gtest/gtest.h>
#include <ur_teleop_controllers/smoothing.hpp>
#include <limits>
#include <ur_teleop_controllers/target_history.hpp>
#include <random>
using ur_teleop_controllers::Smoothing;

TEST(Smoothing, ReversalsAndNoiseRespectCommandBounds) {
  Smoothing s;
  ASSERT_TRUE(s.configure(.5, 1., 10.));
  ASSERT_TRUE(s.reset({}, {}));
  std::mt19937 random(42);
  std::uniform_real_distribution<double> noise(-.001, .001);
  std::uniform_real_distribution<double> jitter(.0018, .0022);
  Smoothing::Vector previous_v{}, previous_a{}, target{};
  for (int k=0; k<15000; ++k) {
    const double dt=jitter(random);
    for (size_t i=0;i<6;++i) target[i] = ((k/250)%2 ? -.05:.05) + noise(random);
    ASSERT_TRUE(s.step(target, dt)) << k;
    for (size_t i=0;i<6;++i) {
      EXPECT_LE(std::abs(s.input.current_velocity[i]), .5+1e-8);
      EXPECT_LE(std::abs(s.input.current_acceleration[i]), 1.+1e-8);
      EXPECT_LE(std::abs(s.input.current_velocity[i]-previous_v[i]), dt+1e-7);
      EXPECT_LE(std::abs(s.input.current_acceleration[i]-previous_a[i]), 10.*dt+1e-7);
    }
    previous_v=s.input.current_velocity; previous_a=s.input.current_acceleration;
  }
}

TEST(Smoothing, StationaryTargetConvergesWithoutSteadyScaleError) {
  Smoothing s; s.configure(.5,1.,10.); s.reset({},{});
  Smoothing::Vector target{.1,-.1,.05,-.02,.03,.12};
  for (int k=0;k<5000;++k) ASSERT_TRUE(s.step(target,.002));
  for (size_t i=0;i<6;++i) {
    EXPECT_NEAR(s.input.current_position[i],target[i],1e-9);
    EXPECT_NEAR(s.input.current_velocity[i],0.,1e-9);
    EXPECT_NEAR(s.input.current_acceleration[i],0.,1e-9);
  }
}

TEST(Smoothing, ReferenceDoesNotJumpToNoisyFeedback) {
  Smoothing s; s.configure(.5,1.,10.); s.reset({},{});
  Smoothing::Vector target{}; target.fill(.1);
  ASSERT_TRUE(s.step(target,.002));
  for(double q:s.input.current_position) EXPECT_LE(std::abs(q), 10.*std::pow(.002,3)/6.+1e-10);
}

TEST(Smoothing, InvalidSamplesNeverChangeCommandState) {
  Smoothing s; s.configure(.5,1.,10.); s.reset({},{});
  Smoothing::Vector target{}; target.fill(.01);
  for(double dt:{0.,-.001,.021,std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(s.step(target,dt)); EXPECT_EQ(s.input.current_position,Smoothing::Vector{});
  }
  target[3]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(s.step(target,.002));
  EXPECT_EQ(s.input.current_position,Smoothing::Vector{});
}

TEST(Smoothing, ResetDoesNotRevivePreviousMotion) {
  Smoothing s; s.configure(.5,1.,10.); s.reset({},{});
  Smoothing::Vector target{}; target.fill(.1);
  for(int k=0;k<100;++k) ASSERT_TRUE(s.step(target,.002));
  Smoothing::Vector measured{}; measured.fill(-.3);
  ASSERT_TRUE(s.reset(measured,{}));
  ASSERT_TRUE(s.step(measured,.002));
  EXPECT_EQ(s.input.current_position,measured);
  EXPECT_EQ(s.input.current_velocity,Smoothing::Vector{});
}

TEST(Smoothing, PositiveStreamingTargetDoesNotReverseToMeetTerminalVelocity) {
  Smoothing s; s.configure(.5,1.,10.); s.reset({},{});
  Smoothing::Vector q{},dq{}; q.fill(.003); dq.fill(.3);
  for(int k=0;k<100;++k) {
    ASSERT_TRUE(s.step(q,.002,dq,1.,true));
    for(double position:s.input.current_position) EXPECT_GE(position,0.);
    for(double velocity:s.input.current_velocity) EXPECT_GE(velocity,0.);
  }
  for(int k=0;k<1000;++k) ASSERT_TRUE(s.step(q,.002,dq,0.,true));
  for(double v:s.input.current_velocity) EXPECT_NEAR(v,0.,1e-9);
}

TEST(TargetHistory, MissingVelocityUsesReceivedTargetsAndTimeNotInternalState) {
  ur_teleop_controllers::TargetHistory history;
  Smoothing::Vector q{},dq{};
  EXPECT_FALSE(history.observe(q,10.,.5,dq));
  q.fill(.002);
  EXPECT_TRUE(history.observe(q,10.02,.5,dq));
  for(double v:dq) EXPECT_NEAR(v,.1,1e-10);
  q.fill(.02);
  EXPECT_TRUE(history.observe(q,10.03,.5,dq));
  for(double v:dq) EXPECT_DOUBLE_EQ(v,.5);
  EXPECT_FALSE(history.observe(q,11.,.5,dq));
  history.reset();
  EXPECT_FALSE(history.observe(q,11.01,.5,dq));
}

TEST(Smoothing, RestoredHoldBrakesAndRemainsStationary) {
  Smoothing s; s.configure(.5,1.,10.); s.reset({},{});
  Smoothing::Vector q{},dq{}; q.fill(.03); dq.fill(.1);
  for(int k=0;k<100;++k) ASSERT_TRUE(s.step(q,.002,dq,1.,true));
  dq.fill(0.);
  for(int k=0;k<1000;++k) ASSERT_TRUE(s.step(s.input.current_position,.002,dq,1.,true));
  const auto settled=s.input.current_position;
  for(int k=0;k<100;++k) ASSERT_TRUE(s.step(s.input.current_position,.002,dq,1.,true));
  for(size_t i=0;i<6;++i) {
    EXPECT_NEAR(s.input.current_position[i],settled[i],1e-10);
    EXPECT_NEAR(s.input.current_velocity[i],0.,1e-10);
  }
}

TEST(Smoothing, ServoVelocityNeverGetsAnOpposingInternalPositionCorrection) {
  Smoothing s; s.configure(.5,1.,10.); s.reset({},{});
  Smoothing::Vector q{},dq{}; q.fill(-.03); dq.fill(.1);
  for(int k=0;k<100;++k) {
    ASSERT_TRUE(s.step(q,.002,dq,1.,true));
    EXPECT_EQ(s.input.control_interface,ruckig::ControlInterface::Velocity);
    EXPECT_DOUBLE_EQ(s.input.target_velocity[0],.1);
    EXPECT_GE(s.input.current_velocity[0],0.);
  }
  dq.fill(0.);
  for(int k=0;k<1000;++k) ASSERT_TRUE(s.step(q,.002,dq,1.,true));
  for(double v:s.input.current_velocity) EXPECT_NEAR(v,0.,1e-10);
}

TEST(Smoothing, ServoFeedbackLoopSettlesWithActuatorLag) {
  // Deterministic 100 Hz Servo position-difference velocity and 500 Hz native
  // reference, including first-order Butterworth target filtering and lagged
  // measurement. Test both the operational gain and a faster response.
  for(const auto& setup:std::vector<std::pair<double,double>>{{2.,.04},{2.,.1},{10.,.04}}) {
    Smoothing s; s.configure(.5,1.,10.); s.reset({},{});
    Smoothing::Vector q{},dq{};
    double measured=0.,filtered=0.,previous_raw=0.,late_peak=0.;
    for(int k=0;k<10000;++k) {
      if(k%5==0) {
        const double velocity=std::clamp(setup.first*(.1-measured),-.5,.5);
        const double raw=measured+velocity*.01;
        filtered=(raw+previous_raw+.5*filtered)/2.5; previous_raw=raw;
        q.fill(filtered); dq.fill((filtered-measured)/.01);
      }
      ASSERT_TRUE(s.step(q,.002,dq,1.,true));
      measured+=(s.input.current_position[0]-measured)*(.002/setup.second);
      if(k>7500) late_peak=std::max(late_peak,std::abs(s.input.current_velocity[0]));
    }
    EXPECT_NEAR(measured,.1,1e-5);
    EXPECT_LT(late_peak,1e-5);
  }
}
