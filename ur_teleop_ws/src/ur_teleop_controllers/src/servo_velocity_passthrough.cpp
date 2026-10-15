#include <ur_teleop_controllers/servo_velocity_passthrough.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ur_teleop_controllers::ServoVelocityPassthrough,
                      online_signal_smoothing::SmoothingBaseClass)
