// Real plugin + real UR/JTC interpolation with local doubles as hardware.
// Never loads a hardware plugin or opens a robot connection.
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <array>
#include <thread>
#include <cmath>

TEST(Controller, FinalInterfaceWritesAreBoundedAfterInterpolation) {
  setenv("ROS_DOMAIN_ID","147",1); setenv("ROS_LOCALHOST_ONLY","1",1);
  rclcpp::init(0,nullptr);
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader("controller_interface","controller_interface::ControllerInterface");
  auto c=loader.createSharedInstance("ur_teleop_controllers/SmoothScaledJointTrajectoryController");
  std::vector<std::string> names{"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"joints",names}, {"command_interfaces",std::vector<std::string>{"position"}},
    {"state_interfaces",std::vector<std::string>{"position","velocity"}},
    {"speed_scaling_interface_name","speed_scaling/speed_scaling_factor"},
    {"update_rate",500}, {"cmd_timeout",.2}, {"smoothing.max_acceleration",1.},
    {"smoothing.max_jerk",10.}, {"smoothing.max_velocity",.5}});
  ASSERT_EQ(c->init("test_smooth_controller","",options),controller_interface::return_type::OK);
  ASSERT_EQ(c->configure().id(),lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::array<double,6> q{},v{},command{};
  double scale=1.;
  std::vector<hardware_interface::CommandInterface> ci;
  std::vector<hardware_interface::StateInterface> si;
  ci.reserve(6); si.reserve(13);
  for(size_t i=0;i<6;++i) {
    ci.emplace_back(names[i],"position",&command[i]);
    si.emplace_back(names[i],"position",&q[i]); si.emplace_back(names[i],"velocity",&v[i]);
  }
  si.emplace_back("speed_scaling","speed_scaling_factor",&scale);
  std::vector<hardware_interface::LoanedCommandInterface> lc;
  std::vector<hardware_interface::LoanedStateInterface> ls;
  for(auto& x:ci) lc.emplace_back(x);
  for(auto& x:si) ls.emplace_back(x);
  c->assign_interfaces(std::move(lc),std::move(ls));
  ASSERT_EQ(c->get_node()->activate().id(),lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  auto n=std::make_shared<rclcpp::Node>("smoothing_test_publisher");
  auto pub=n->create_publisher<trajectory_msgs::msg::JointTrajectory>("/test_smooth_controller/joint_trajectory",10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(c->get_node()->get_node_base_interface()); executor.add_node(n);
  for(int k=0;k<100 && !pub->get_subscription_count();++k) {
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(pub->get_subscription_count(),0u);
  const auto dt=rclcpp::Duration::from_seconds(.002);
  auto now=n->now();
  std::array<double,6> previous_q{},previous_v{},previous_a{};
  double max_v=0.,max_a=0.,max_j=0.,span=0.,settled_peak_v=0.;
  std::array<double,6> stationary_target{};
  for(int k=0;k<4500;++k) {
    if(k==1500) stationary_target=q;
    if(k%5==0 && k<4250) {
      trajectory_msgs::msg::JointTrajectory msg; msg.joint_names=names; msg.points.resize(1);
      msg.points[0].positions.resize(6);
      for(size_t i=0;i<6;++i) msg.points[0].positions[i]=(k<1500 ? q[i]+((k/250)%2 ? -.003:.003) : stationary_target[i]);
      msg.points[0].time_from_start=rclcpp::Duration::from_seconds(.01);
      if(k<1500) msg.points[0].velocities.assign(6, (k/250)%2 ? -.3 : .3);
      if(k>=4000) {
        msg.points[0].velocities.assign(6,0.);
        msg.points[0].time_from_start=rclcpp::Duration::from_seconds(.1);
      }
      pub->publish(msg);
    }
    executor.spin_some();
    now=now+dt;
    ASSERT_EQ(c->update(now,dt),controller_interface::return_type::OK) << k;
    for(size_t i=0;i<6;++i) {
      const double cv=(command[i]-previous_q[i])/.002;
      const double ca=(cv-previous_v[i])/.002;
      const double cj=(ca-previous_a[i])/.002;
      max_v=std::max(max_v,std::abs(cv)); max_a=std::max(max_a,std::abs(ca));
      max_j=std::max(max_j,std::abs(cj)); span=std::max(span,std::abs(command[i]));
      EXPECT_LE(std::abs(cv),.500001); EXPECT_LE(std::abs(ca),1.00001); EXPECT_LE(std::abs(cj),10.001);
      if(k>=3500 && k<4000) settled_peak_v=std::max(settled_peak_v,std::abs(cv));
      previous_q[i]=command[i]; previous_v[i]=cv; previous_a[i]=ca;
      // Simulate a 40 ms actuator lag, not perfect command tracking.
      const double measured=q[i]+.05*(command[i]-q[i]);
      v[i]=(measured-q[i])/.002; q[i]=measured;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_GT(span,.001);
  EXPECT_LT(settled_peak_v,.001) << "Stationary streamed targets must settle without a hold message";
  for(double cv:previous_v) EXPECT_LT(std::abs(cv),.02);
  std::cout<<"Final hardware-interface finite differences: velocity="<<max_v<<" acceleration="<<max_a<<" jerk="<<max_j<<" settled_peak_v="<<settled_peak_v<<" span="<<span<<std::endl;
  // A missed control deadline must not leak Base's unfiltered command to write().
  const auto before_failure=command;
  EXPECT_EQ(c->update(now+rclcpp::Duration::from_seconds(.1),rclcpp::Duration::from_seconds(.1)),controller_interface::return_type::ERROR);
  EXPECT_EQ(command,before_failure);
  c->get_node()->deactivate(); c->release_interfaces();
  executor.remove_node(c->get_node()->get_node_base_interface()); executor.remove_node(n);
  c.reset(); n.reset(); rclcpp::shutdown();
}
