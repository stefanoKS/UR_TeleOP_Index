#include <ur_controllers/scaled_joint_trajectory_controller.hpp>
#include <ur_teleop_controllers/smoothing.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace ur_teleop_controllers {
class SmoothScaledJointTrajectoryController : public ur_controllers::ScaledJointTrajectoryController {
  using Base = ur_controllers::ScaledJointTrajectoryController;
  using Message = trajectory_msgs::msg::JointTrajectory;
  Smoothing smoother_;
  Smoothing::Vector previous_write_{};
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> diagnostics_;
  double follow_error_ = .10;
  bool initialized_ = false;
  size_t cycles_ = 0;
  std::string scaling_interface_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Message>> monitor_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
public:
  CallbackReturn on_init() override {
    const auto result = Base::on_init();
    if (result != CallbackReturn::SUCCESS) return result;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    for (const auto& item:std::vector<std::pair<std::string,double>>{
        {"smoothing.max_velocity",.5}, {"smoothing.max_acceleration",1.},
        {"smoothing.max_jerk",10.}, {"smoothing.max_following_error",.10}})
      if (!get_node()->has_parameter(item.first)) get_node()->declare_parameter(item.first,item.second,descriptor);
    parameter_guard_ = get_node()->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& parameters) {
      rcl_interfaces::msg::SetParametersResult result; result.successful=true;
      for(const auto& parameter:parameters) if(parameter.get_name().rfind("smoothing.",0)==0) {
        result.successful=false; result.reason="Smoothing limits are startup-only; restart the driver";
      }
      return result;
    });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override {
    const auto result = Base::on_configure(state);
    if (result != CallbackReturn::SUCCESS) return result;
    if (dof_ != 6 || !has_position_command_interface_ || !has_velocity_state_interface_ ||
        has_velocity_command_interface_ || has_effort_command_interface_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Smoothing requires six position command interfaces");
      return CallbackReturn::ERROR;
    }
    follow_error_ = get_node()->get_parameter("smoothing.max_following_error").as_double();
    scaling_interface_ = get_node()->get_parameter("speed_scaling_interface_name").as_string();
    if (!std::isfinite(follow_error_) || follow_error_ <= 0 || follow_error_ > .2 ||
        !smoother_.configure(get_node()->get_parameter("smoothing.max_velocity").as_double(),
          get_node()->get_parameter("smoothing.max_acceleration").as_double(),
          get_node()->get_parameter("smoothing.max_jerk").as_double())) return CallbackReturn::ERROR;
    auto publisher = get_node()->create_publisher<Message>("~/smoothed_command", rclcpp::SystemDefaultsQoS());
    monitor_ = std::make_unique<realtime_tools::RealtimePublisher<Message>>(publisher);
    monitor_->msg_.joint_names = params_.joints;
    monitor_->msg_.points.resize(1);
    monitor_->msg_.points[0].positions.resize(6);
    monitor_->msg_.points[0].velocities.resize(6);
    monitor_->msg_.points[0].accelerations.resize(6);
    auto diagnostics = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/control_diagnostics", 1);
    diagnostics_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(diagnostics);
    auto& message = diagnostics_->msg_;
    message.layout.dim.resize(2);
    message.layout.dim[0].label = "joints in smoothed_command.joint_names order";
    message.layout.dim[0].size = 6; message.layout.dim[0].stride = 48;
    message.layout.dim[1].label = "measured_q,measured_dq,servo_q,servo_dq,ruckig_q,ruckig_dq,servo_minus_measured,ruckig_minus_measured";
    message.layout.dim[1].size = 8; message.layout.dim[1].stride = 8;
    // data[0] is ROS time in seconds; data_offset skips it for the 6x8 array.
    message.layout.data_offset = 1;
    message.data.resize(49);
    RCLCPP_INFO(get_node()->get_logger(), "Final hardware-command smoothing: v=%.3f a=%.3f j=%.3f (rad units)",
                smoother_.max_velocity, smoother_.max_acceleration, smoother_.max_jerk);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
    initialized_ = false;
    cycles_ = 0;
    return Base::on_activate(state);
  }

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override {
    // Base owns scaling, trajectory interpolation, tolerance checks and cmd_timeout.
    // ros2_control calls hardware.write only after this complete update returns.
    if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return Base::update(time, period);
    for (size_t i=0; i<6; ++i) previous_write_[i]=joint_command_interface_[0][i].get().get_value();
    const auto result = Base::update(time, period);
    if (result != controller_interface::return_type::OK) return fail("UR trajectory controller update failed");
    Smoothing::Vector measured{}, velocity{}, requested{}, feedforward{};
    double scale = scaling_interface_.empty() ? 1. : std::numeric_limits<double>::quiet_NaN();
    for (const auto& interface:state_interfaces_)
      if (interface.get_name()==scaling_interface_) scale=interface.get_value();
    for (size_t i=0; i<6; ++i) {
      measured[i] = state_current_.positions[i];
      velocity[i] = state_current_.velocities[i];
      requested[i] = joint_command_interface_[0][i].get().get_value();
      if (last_commanded_state_.velocities.size()==6) feedforward[i] = last_commanded_state_.velocities[i];
    }
    if (!initialized_) {
      // Re-activation must start at standstill; never revive an old command state.
      for (double v:velocity) if (!std::isfinite(v) || std::abs(v) > .02) return fail("Activation requires stationary joints");
      if (!smoother_.reset(measured, velocity)) return fail("Invalid joint feedback at activation");
      initialized_ = true;
    }
    // Keep velocity-mode ramping. Supplied Servo velocities are the complete
    // request, not feed-forward for a second position feedback loop.
    bool stopping = rt_is_holding_.load();
    bool velocity_supplied = false;
    const auto trajectory = traj_external_point_ptr_->get_trajectory_msg();
    if (trajectory && trajectory->points.size()==1) {
      const auto& velocities=trajectory->points[0].velocities;
      feedforward.fill(0.);
      if (velocities.size()==6) {
        velocity_supplied = true;
        for (size_t i=0; i<6; ++i) {
          const auto found=std::find(trajectory->joint_names.begin(), trajectory->joint_names.end(), params_.joints[i]);
          if(found==trajectory->joint_names.end()) return fail("Missing joint in streaming velocity command");
          feedforward[i]=velocities[static_cast<size_t>(found-trajectory->joint_names.begin())];
        }
      }
      stopping = stopping || (velocities.size()==6 && std::all_of(velocities.begin(),velocities.end(),
                                                       [](double v){return v==0.;}));
    }
    const auto incoming_position=requested, incoming_velocity=feedforward;
    if (stopping) {
      requested=smoother_.input.current_position;
      feedforward.fill(0.);
      velocity_supplied = true;
    }
    for (size_t i=0; i<6; ++i) {
      if (!std::isfinite(measured[i]) || std::abs(smoother_.input.current_position[i]-measured[i]) > follow_error_)
        return fail("Smoothed reference is too far from measured joints");
      if (!std::isfinite(requested[i]) || std::abs(requested[i]-measured[i]) > follow_error_)
        return fail("Trajectory reference is too far from measured joints");
    }
    if (!smoother_.step(requested, period.seconds(), feedforward, scale, velocity_supplied)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Smoothing rejection: %s; dt=%.6f s scale=%.6f joint=%s result=%d",
        smoother_.last_error, period.seconds(), scale,
        smoother_.error_joint >= 0 ? params_.joints[smoother_.error_joint].c_str() : "all",
        smoother_.result_code);
      return fail("Invalid command timing or Ruckig smoothing result");
    }
    for(size_t i=0; i<6; ++i)
      if(std::abs(smoother_.input.current_position[i]-measured[i]) > follow_error_)
        return fail("Next smoothed reference exceeds joint following-error bound");
    for (size_t i=0; i<6; ++i) {
      joint_command_interface_[0][i].get().set_value(smoother_.input.current_position[i]);
      last_commanded_state_.positions[i] = smoother_.input.current_position[i];
      if (last_commanded_state_.velocities.size()==6) last_commanded_state_.velocities[i] = smoother_.input.current_velocity[i];
      if (last_commanded_state_.accelerations.size()==6) last_commanded_state_.accelerations[i] = smoother_.input.current_acceleration[i];
    }
    // 100 Hz telemetry of the reference actually written, not the pre-filter target.
    if (++cycles_ % 5 == 0 && monitor_->trylock()) {
      monitor_->msg_.header.stamp = time;
      auto& point = monitor_->msg_.points[0];
      for (size_t i=0; i<6; ++i) {
        point.positions[i] = smoother_.input.current_position[i];
        point.velocities[i] = smoother_.input.current_velocity[i];
        point.accelerations[i] = smoother_.input.current_acceleration[i];
      }
      monitor_->unlockAndPublish();
    }
    // Preallocated numeric telemetry at 10 Hz: no formatting/allocation in the
    // hardware update thread. ROS bag receipt time plus data[0] provide timing.
    if(cycles_ % 50 == 0 && diagnostics_->trylock()) {
      auto& data=diagnostics_->msg_.data;
      data[0]=time.seconds();
      for(size_t i=0;i<6;++i) {
        const size_t k=1+8*i;
        data[k]=measured[i]; data[k+1]=velocity[i];
        data[k+2]=incoming_position[i]; data[k+3]=incoming_velocity[i];
        data[k+4]=smoother_.input.current_position[i]; data[k+5]=smoother_.input.current_velocity[i];
        data[k+6]=incoming_position[i]-measured[i]; data[k+7]=smoother_.input.current_position[i]-measured[i];
      }
      diagnostics_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type fail(const char* reason) {
    // Never leave Base's unfiltered values on the interfaces after a rejection.
    // Controller manager will deactivate on ERROR; UR's own stop path remains intact.
    for (size_t i=0; i<joint_command_interface_[0].size(); ++i) {
      const double q = previous_write_[i];
      if (std::isfinite(q)) joint_command_interface_[0][i].get().set_value(q);
    }
    RCLCPP_ERROR(get_node()->get_logger(), "%s; controller stopped, explicit recovery required", reason);
    initialized_ = false;
    return controller_interface::return_type::ERROR;
  }
};
}
PLUGINLIB_EXPORT_CLASS(ur_teleop_controllers::SmoothScaledJointTrajectoryController, controller_interface::ControllerInterface)
