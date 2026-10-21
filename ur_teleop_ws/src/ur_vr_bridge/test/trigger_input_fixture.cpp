// Hardware-free fixture: fake selected-device axis events and advancing IMU
// enter the exact production cache/serializer. Never opens libsurvive or USB.
#include "ur_vr_bridge/trigger_guard.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
struct FakeButtonEvent {
  double time;
  unsigned axis_count = 1;
  unsigned axis_ids[8] = {1};
  double axis_val[8] = {};
};
class Fixture : public rclcpp::Node {
 public:
  Fixture() : Node("trigger_input_fixture") {
    pub = create_publisher<std_msgs::msg::String>("/vr/controller/gripper_input", 1);
    sub = create_subscription<std_msgs::msg::String>("/test/trigger_events", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        std::istringstream s(m->data); std::string command; double value = 0; if (!(s >> command >> value)) return;
        const double t = now().seconds();
        if(command == "axis") {
          FakeButtonEvent event{}; event.time = t; event.axis_val[0] = value;
          guard.button(event, t, t, now().nanoseconds());
        }
        else if(command == "pose") pose = value != 0;
        else if(command == "stale") stale = value;
        else if(command == "imu") imu = value != 0;
      });
    timer = create_wall_timer(std::chrono::milliseconds(10), [this]() {
      const double t = now().seconds();
      if(imu) guard.imu(++clock, t);
      std_msgs::msg::String msg;
      msg.data = guard.message(t, now().nanoseconds()-int64_t(stale*1e9), "fixture", "KN0:fake", pose, pose, true);
      pub->publish(msg);
    });
  }
 private:
  ur_vr_bridge::TriggerGuard guard;
  double stale = 0;
  uint32_t clock = 0;
  bool pose = true, imu = true;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  rclcpp::TimerBase::SharedPtr timer;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<Fixture>()); rclcpp::shutdown();
}
