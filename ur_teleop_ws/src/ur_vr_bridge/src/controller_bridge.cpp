#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#define SURVIVE_ENABLE_FULL_API
#include <survive_api.h>
#include <survive.h>
#include "ur_vr_bridge/pose_guard.hpp"
#include "ur_vr_bridge/optical_guard.hpp"
#include <mutex>
#include <sstream>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class ControllerBridge : public rclcpp::Node {
 public:
  ControllerBridge() : Node("vr_controller_bridge") {
    device_name_ = declare_parameter<std::string>("device_name", "KN0");
    requested_serial_ = declare_parameter<std::string>("device_serial", "");
    frame_ = declare_parameter<std::string>("frame_id", "lighthouse");
    child_ = declare_parameter<std::string>("child_frame_id", "vr_controller");
    timeout_ = declare_parameter<double>("pose_timeout", 0.35);
    const int verbosity = declare_parameter<int>("libsurvive_verbosity", 10);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || frame_.empty() ||
        child_.empty() || frame_ == child_ || get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("Use wall time, distinct nonempty frames, and a positive finite pose_timeout");
    }
    auto reliable = rclcpp::QoS(10).reliable();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vr/controller/pose", reliable);
    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("/vr/controller/joy", reliable);
    fresh_pub_ = create_publisher<std_msgs::msg::Bool>("/vr/controller/pose_fresh", reliable);
    optical_pub_ = create_publisher<std_msgs::msg::Bool>("/vr/controller/optical_fresh", rclcpp::QoS(1).reliable());
    optical_detail_pub_ = create_publisher<std_msgs::msg::String>("/vr/controller/optical_status", rclcpp::QoS(1).reliable());
    time_pub_ = create_publisher<std_msgs::msg::Float64>("/vr/controller/source_time", reliable);
    identity_pub_ = create_publisher<std_msgs::msg::String>(
        "/vr/controller/device", rclcpp::QoS(1).reliable().transient_local());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Keep libsurvive arguments separate from ROS arguments; never pair or recalibrate here.
    lib_args_ = {"vr_controller_bridge", "--v", std::to_string(verbosity)};
    for (auto& arg : lib_args_) lib_argv_.push_back(arg.data());
    lib_argv_.push_back(nullptr);
    ctx_ = survive_simple_init(3, lib_argv_.data());
    if (!ctx_) throw std::runtime_error("libsurvive initialization failed");
    // Install before starting the polling thread. Preserve the library's processing.
    survive_simple_set_user(ctx_, this);
    previous_raw_imu_ = survive_install_raw_imu_fn(survive_simple_get_ctx(ctx_), monitor_raw_imu);
    previous_sweep_ = survive_install_sweep_angle_fn(survive_simple_get_ctx(ctx_), optical_sweep);
    survive_simple_start_thread(ctx_);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Observation bridge v0.2.0 started (FLT=%zu bytes). Gen2 optical monitor: >=4 paired sensors from each of >=2 calibrated stations within 250 ms. No robot command publishers. Looking for %s%s%s",
                sizeof(FLT), device_name_.c_str(), requested_serial_.empty() ? "" : " serial=", requested_serial_.c_str());
    last_diagnostic_ = steady_time();
  }

  ~ControllerBridge() override {
    if (ctx_) survive_simple_close(ctx_);
  }

 private:
  bool matches_device(const SurviveObject* so) const {
    return requested_serial_.empty() ? device_name_ == so->codename :
                                      requested_serial_ == so->serial_number;
  }

  static void monitor_raw_imu(SurviveObject* so, int mask, const FLT* data,
                              survive_timecode tc, int id) {
    auto* simple = static_cast<SurviveSimpleContext*>(so->ctx->user_ptr);
    auto* self = static_cast<ControllerBridge*>(survive_simple_get_user(simple));
    if (self->matches_device(so)) {
      self->last_imu_arrival_.store(steady_time());
      ++self->imu_packets_;
    }
    // Observation only: preserve the complete original processing chain.
    if (self->previous_raw_imu_) self->previous_raw_imu_(so, mask, data, tc, id);
    else survive_default_raw_imu_process(so, mask, data, tc, id);
  }

  static void optical_sweep(SurviveObject* so, survive_channel channel, int sensor,
                            survive_timecode tc, int8_t plane, FLT angle) {
    auto* simple = static_cast<SurviveSimpleContext*>(so->ctx->user_ptr);
    auto* self = static_cast<ControllerBridge*>(survive_simple_get_user(simple));
    const bool matches = self->matches_device(so);
    if (matches) {
      self->last_sweep_arrival_.store(steady_time());
      ++self->sweep_packets_;
    }
    // Called in the polling thread: inspect optical data here, never race by
    // reading mutable activations from the ROS thread.
    const int lh = survive_get_bsd_idx(so->ctx, channel);
    const bool valid = lh >= 0 && lh < NUM_GEN2_LIGHTHOUSES && sensor >= 0 &&
                       sensor < so->sensor_ct && sensor < ur_vr_bridge::OpticalGuard::sensors &&
                       plane >= 0 && plane < 2 && std::isfinite(angle);
    const uint64_t before = valid ? so->activations.timecode[sensor][lh][plane] : 0;
    if (self->previous_sweep_) self->previous_sweep_(so, channel, sensor, tc, plane, angle);
    else survive_default_sweep_angle_process(so, channel, sensor, tc, plane, angle);
    if (!valid || !matches || !so->ctx->bsd[lh].PositionSet) return;
    const uint64_t accepted = so->activations.timecode[sensor][lh][plane];
    // Rejected angles do not advance the accepted table. Require a fresh device
    // timestamp relative to the latest light/IMU time too.
    const uint64_t latest = SurviveSensorActivations_last_time(&so->activations);
    if (accepted <= before || latest < accepted || so->timebase_hz <= 0 ||
        static_cast<double>(latest - accepted) / so->timebase_hz >= ur_vr_bridge::OpticalGuard::timeout) return;
    std::lock_guard<std::mutex> lock(self->optical_mutex_);
    self->optical_guard_.observe(lh, sensor, plane, accepted, steady_time());
  }

  static double steady_time() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool select(const SurviveSimpleObject* obj) {
    if (!obj || survive_simple_object_get_type(obj) != SurviveSimpleObject_OBJECT) return false;
    const char* raw_name = survive_simple_object_name(obj);
    const char* raw_serial = survive_simple_serial_number(obj);
    const std::string name = raw_name ? raw_name : "";
    const std::string serial = raw_serial ? raw_serial : "";
    if (requested_serial_.empty() ? name != device_name_ : serial != requested_serial_) return false;
    if (selected_) return obj == selected_;  // Do not silently switch devices after reconnection.
    selected_ = obj;
    std_msgs::msg::String message;
    message.data = "name=" + name + " serial=" + serial +
      " subtype=" + SurviveObjectSubtypeStr(survive_simple_object_get_subtype(obj));
    identity_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
    return true;
  }

  void accept_pose(const SurviveSimplePoseUpdatedEvent& event) {
    if (!select(event.object)) return;
    ++pose_samples_;
    const auto& p = event.pose;
    last_raw_source_ = event.time;
    last_raw_norm_ = 0;
    for (double component : p.Rot) last_raw_norm_ = std::hypot(last_raw_norm_, component);
    last_raw_position_finite_ = std::isfinite(p.Pos[0]) && std::isfinite(p.Pos[1]) && std::isfinite(p.Pos[2]);
    if (!std::isfinite(event.time) || event.time <= 0) rejection_ = "invalid source timestamp";
    else if (!last_raw_position_finite_) rejection_ = "nonfinite position";
    else if (!std::isfinite(last_raw_norm_) || last_raw_norm_ < .5 || last_raw_norm_ > 1.5)
      rejection_ = "uninitialized or invalid quaternion";
    else rejection_ = "duplicate or non-increasing source timestamp";
    std::array<double, 4> q;
    if (!guard_.accept(event.time, {p.Pos[0], p.Pos[1], p.Pos[2]},
                       {p.Rot[0], p.Rot[1], p.Rot[2], p.Rot[3]}, steady_time(), q)) return;
    ++accepted_samples_;
    rejection_ = "none";
    latest_pose_.header.stamp = now();  // ROS receipt time; original source time published separately.
    latest_pose_.header.frame_id = frame_;
    latest_pose_.pose.position.x = p.Pos[0];
    latest_pose_.pose.position.y = p.Pos[1];
    latest_pose_.pose.position.z = p.Pos[2];
    latest_pose_.pose.orientation.x = q[0];
    latest_pose_.pose.orientation.y = q[1];
    latest_pose_.pose.orientation.z = q[2];
    latest_pose_.pose.orientation.w = q[3];
    latest_source_ = event.time;
    new_pose_ = true;
  }

  void tick() {
    SurviveSimpleEvent event{};
    new_pose_ = false;
    bool shutdown = false;
    // Drain the event queue (including input events) so old events cannot accumulate indefinitely.
    int count = 0;
    for (; count < 2048; ++count) {
      const auto type = survive_simple_next_event(ctx_, &event);
      if (type == SurviveSimpleEventType_None) break;
      if (type == SurviveSimpleEventType_Shutdown) { shutdown = true; break; }
      if (type == SurviveSimpleEventType_PoseUpdateEvent) {
        const auto* pose_event = survive_simple_get_pose_updated_event(&event);
        if (pose_event) accept_pose(*pose_event);
      } else if (type == SurviveSimpleEventType_ButtonEvent) {
        const auto* button_event = survive_simple_get_button_event(&event);
        if (button_event && select(button_event->object)) {
          ++button_events_;
          const auto button = static_cast<unsigned int>(button_event->button_id);
          if (button < buttons_.size()) {
            if (button_event->event_type == SURVIVE_INPUT_EVENT_BUTTON_DOWN) buttons_[button] = 1;
            if (button_event->event_type == SURVIVE_INPUT_EVENT_BUTTON_UP) buttons_[button] = 0;
          }
          for (unsigned int i = 0; i < button_event->axis_count && i < SURVIVE_MAX_AXIS_COUNT; ++i) {
            const auto axis = static_cast<unsigned int>(button_event->axis_ids[i]);
            const auto value = button_event->axis_val[i];
            if (axis < axes_.size()) axes_[axis] = std::isfinite(value) ? static_cast<float>(value) : 0.0f;
          }
        }
      }
    }
    const bool fresh = !shutdown && count < 2048 &&
        survive_simple_is_running(ctx_) && guard_.fresh(steady_time(), timeout_);
    std_msgs::msg::Bool status;
    status.data = fresh;
    fresh_pub_->publish(status);
    {
      std::lock_guard<std::mutex> lock(optical_mutex_);
      const double t = steady_time();
      std_msgs::msg::Bool optical;
      optical.data = fresh && optical_guard_.fresh(t);
      optical_pub_->publish(optical);
      std::ostringstream detail;
      detail << "Gen2 accepted paired sensors (250 ms):";
      for (const auto& entry : optical_guard_.counts(t)) detail << " LH" << entry.first << "=" << entry.second;
      detail << "; require >=4 on each of >=2 stations; optical_fresh=" << (optical.data ? "true" : "false");
      std_msgs::msg::String message;
      message.data = detail.str();
      optical_detail_pub_->publish(message);
    }

    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.header.frame_id = child_;
    joy.buttons.assign(16, 0);
    joy.axes.assign(12, 0.0f);
    if (fresh && selected_) {
      joy.buttons.assign(buttons_.begin(), buttons_.end());
      joy.axes.assign(axes_.begin(), axes_.end());
    } else {
      buttons_.fill(0);
      axes_.fill(0);
    }
    joy_pub_->publish(joy);

    if (fresh && new_pose_) {
      pose_pub_->publish(latest_pose_);
      std_msgs::msg::Float64 source;
      source.data = latest_source_;
      time_pub_->publish(source);
      geometry_msgs::msg::TransformStamped transform;
      transform.header = latest_pose_.header;
      transform.child_frame_id = child_;
      transform.transform.translation.x = latest_pose_.pose.position.x;
      transform.transform.translation.y = latest_pose_.pose.position.y;
      transform.transform.translation.z = latest_pose_.pose.position.z;
      transform.transform.rotation = latest_pose_.pose.orientation;
      tf_->sendTransform(transform);
    }
    if (fresh != last_fresh_) {
      RCLCPP_INFO(get_logger(), "Pose stream %s (freshness does not establish optical tracking quality)",
                  fresh ? "fresh" : "stale");
      last_fresh_ = fresh;
    }
    if (!fresh && steady_time() - last_diagnostic_ >= 2.0) {
      const double t = steady_time();
      const double imu_arrival = last_imu_arrival_.load();
      const double sweep_arrival = last_sweep_arrival_.load();
      const double imu_age = imu_arrival > 0 ? t - imu_arrival : -1;
      const double sweep_age = sweep_arrival > 0 ? t - sweep_arrival : -1;
      RCLCPP_WARN(get_logger(),
        "Waiting for pose: selected=%s running=%s pose_samples=%llu accepted=%llu input_events=%llu "
        "last_source=%.9f quaternion_norm=%.6g position_finite=%s last_rejection=%s drain_limit=%s",
        selected_ ? "yes" : "no", survive_simple_is_running(ctx_) ? "yes" : "no",
        static_cast<unsigned long long>(pose_samples_), static_cast<unsigned long long>(accepted_samples_),
        static_cast<unsigned long long>(button_events_), last_raw_source_, last_raw_norm_,
        last_raw_position_finite_ ? "yes" : "no", rejection_.c_str(), count >= 2048 ? "yes" : "no");
      RCLCPP_WARN(get_logger(),
        "Tracking inputs: imu_packets=%llu imu_age_s=%.3f sweep_angles=%llu sweep_age_s=%.3f "
        "(-1 means never received). Pose samples can include button-triggered updates; "
        "running=yes only means the library thread is running. %s",
        static_cast<unsigned long long>(imu_packets_.load()), imu_age,
        static_cast<unsigned long long>(sweep_packets_.load()), sweep_age,
        imu_age < 0 || imu_age > 1.0 ?
          "No recent IMU: check controller power/charge, receiver connection and wake/restart the controller." :
          (sweep_age < 0 || sweep_age > 1.0 ?
            "IMU arriving but no recent sweep angles: check base-station visibility/tracking." :
            "IMU and sweep angles arriving: inspect libsurvive pose initialization/solver logs."));
      last_diagnostic_ = steady_time();
    }
    if (shutdown) {
      RCLCPP_ERROR(get_logger(), "libsurvive stopped; restart the bridge after checking the device");
      rclcpp::shutdown();
    }
  }

  SurviveSimpleContext* ctx_ = nullptr;
  raw_imu_process_func previous_raw_imu_ = nullptr;
  std::atomic<uint64_t> imu_packets_{0}, sweep_packets_{0};
  std::atomic<double> last_imu_arrival_{0}, last_sweep_arrival_{0};
  sweep_angle_process_func previous_sweep_ = nullptr;
  std::mutex optical_mutex_;
  ur_vr_bridge::OpticalGuard optical_guard_;
  std::vector<std::string> lib_args_;
  std::vector<char*> lib_argv_;
  const SurviveSimpleObject* selected_ = nullptr;
  std::string device_name_, requested_serial_, frame_, child_;
  double timeout_ = 0.25, latest_source_ = 0;
  double last_diagnostic_ = 0;
  double last_raw_source_ = std::numeric_limits<double>::quiet_NaN();
  double last_raw_norm_ = std::numeric_limits<double>::quiet_NaN();
  bool last_raw_position_finite_ = false;
  uint64_t pose_samples_ = 0, accepted_samples_ = 0, button_events_ = 0;
  std::string rejection_ = "no selected-device pose sample received";
  bool new_pose_ = false, last_fresh_ = false;
  ur_vr_bridge::PoseGuard guard_;
  std::array<int32_t, 16> buttons_{};
  std::array<float, 12> axes_{};
  geometry_msgs::msg::PoseStamped latest_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fresh_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr optical_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr optical_detail_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr identity_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<ControllerBridge>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    fprintf(stderr, "vr_controller_bridge: %s\n", error.what());
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
