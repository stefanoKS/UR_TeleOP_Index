#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Humble rclpy does not expose MessageInfo in subscription callbacks. Preserve
// the DDS publisher timestamp in C++ before passing Servo targets to Python.
// This relay has no robot command publisher and performs no motion conversion.
class ServoStampRelay : public rclcpp::Node {
 public:
  ServoStampRelay() : Node("servo_stamp_relay", "/vr_real") {
    if (get_parameter("use_sim_time").as_bool()) throw std::runtime_error("Relay requires wall time");
    output_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("servo_joint_targets", rclcpp::QoS(1));
    input_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "servo_unstamped_targets", rclcpp::QoS(1),
      [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg, const rclcpp::MessageInfo& info) {
        int64_t source = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (source == 0) source = info.get_rmw_message_info().source_timestamp;
        const int64_t age = now().nanoseconds() - source;
        if (source <= 0 || age < 0 || age >= 100000000LL) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Dropping target: unavailable, future or stale publisher timestamp");
          return;
        }
        auto stamped = *msg;
        stamped.header.stamp = rclcpp::Time(source);
        output_->publish(stamped);
      });
  }
 private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr output_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr input_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoStampRelay>());
  rclcpp::shutdown();
}
