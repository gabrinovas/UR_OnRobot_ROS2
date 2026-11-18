// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ur_onrobot_control/ur_onrobot_safety_watchdog.hpp"

UrOnrobotSafetyWatchdog::UrOnrobotSafetyWatchdog(const rclcpp::NodeOptions & options)
: Node("ur_onrobot_safety_watchdog", options), updater_(this)
{
  onrobot_type_ = required<std::string>("onrobot_type");
  gripper_kind_ = required<std::string>("gripper_kind");
  joints_ = required<std::vector<std::string>>("joints");
  gripper_action_ = required<std::string>("gripper_action");
  reset_power_service_ = required<std::string>("reset_power_service");
  auto_recover_ = required<bool>("auto_recover");
  hold_current_position_ = required<bool>("hold_current_position");
  hold_position_ = required<double>("hold_position");
  hold_max_effort_ = required<double>("hold_max_effort");
  action_server_wait_s_ = required<double>("action_server_wait_s");
  reset_service_wait_s_ = required<double>("reset_service_wait_s");
  telemetry_warning_age_s_ = required<double>("telemetry_warning_age_s");
  const auto queue_depth = static_cast<size_t>(required<int64_t>("queue_depth"));
  const auto diagnostic_period =
    std::chrono::duration<double>(required<double>("diagnostic_period_s"));
  const auto safety_mode_topic = required<std::string>("safety_mode_topic");
  const auto robot_mode_topic = required<std::string>("robot_mode_topic");
  if (joints_.empty()) {
    throw std::runtime_error("[Fail-Fast] Parameter 'joints' of the safety watchdog is empty");
  }
  joint_position_.assign(joints_.size(), 0.0);

  RCLCPP_INFO(
    get_logger(), "Safety watchdog for %s: safety mode on %s, robot mode on %s, gripper action %s, "
    "auto recover %s", onrobot_type_.c_str(), safety_mode_topic.c_str(), robot_mode_topic.c_str(),
    gripper_action_.c_str(), auto_recover_ ? "on" : "off");

  updater_.setHardwareID("UR Safety Watchdog (" + onrobot_type_ + ")");
  updater_.add("Safety & Interlock Monitor", this, &UrOnrobotSafetyWatchdog::produceDiagnostics);

  safety_mode_subscription_ = create_subscription<SafetyMode>(
    safety_mode_topic, queue_depth,
    std::bind(&UrOnrobotSafetyWatchdog::onSafetyMode, this, std::placeholders::_1));
  robot_mode_subscription_ = create_subscription<RobotMode>(
    robot_mode_topic, queue_depth,
    std::bind(&UrOnrobotSafetyWatchdog::onRobotMode, this, std::placeholders::_1));
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    required<std::string>("joint_states_topic"), queue_depth,
    std::bind(&UrOnrobotSafetyWatchdog::onJointState, this, std::placeholders::_1));

  gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_);
  reset_power_client_ = create_client<Trigger>(reset_power_service_);

  recover_service_ = create_service<Trigger>(
    "~/recover_system",
    std::bind(
      &UrOnrobotSafetyWatchdog::onRecoverRequest, this, std::placeholders::_1,
      std::placeholders::_2));
  // Services that act as if the robot had changed its safety mode, to try the reactions without a
  // stop.
  inject_services_ = {
    injectService("~/inject_protective_stop", SafetyMode::PROTECTIVE_STOP),
    injectService("~/inject_emergency_stop", SafetyMode::ROBOT_EMERGENCY_STOP),
    injectService("~/inject_clear_stop", SafetyMode::NORMAL)};

  diagnostic_timer_ = create_wall_timer(diagnostic_period, [this]() {updater_.force_update();});
  last_telemetry_time_ = now();
}

rclcpp::Service<UrOnrobotSafetyWatchdog::Trigger>::SharedPtr UrOnrobotSafetyWatchdog::injectService(
  const std::string & name, uint8_t mode)
{
  return create_service<Trigger>(
    name,
    [this, mode](const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response) {
      triggerSafetyMode(mode);
      response->success = true;
      response->message = "Safety mode " + safetyModeToString(mode) + " injected.";
    });
}

void UrOnrobotSafetyWatchdog::triggerSafetyMode(uint8_t mode)
{
  SafetyMode message;
  message.mode = mode;
  onSafetyMode(std::make_shared<SafetyMode>(message));
}

void UrOnrobotSafetyWatchdog::triggerRobotMode(int8_t mode)
{
  RobotMode message;
  message.mode = mode;
  onRobotMode(std::make_shared<RobotMode>(message));
}

void UrOnrobotSafetyWatchdog::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    for (size_t joint = 0; joint < joints_.size(); ++joint) {
      if (msg->name[i] == joints_[joint]) {
        joint_position_[joint] = msg->position[i];
        has_telemetry_ = true;
      }
    }
  }
  if (has_telemetry_) {
    last_telemetry_time_ = now();
  }
}

void UrOnrobotSafetyWatchdog::onRobotMode(const RobotMode::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  robot_mode_ = msg->mode;
}

void UrOnrobotSafetyWatchdog::onSafetyMode(const SafetyMode::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const uint8_t previous = safety_mode_;
  safety_mode_ = msg->mode;
  if (safety_mode_ == previous) {
    return;
  }
  RCLCPP_WARN(
    get_logger(), "UR safety mode changed: %s -> %s", safetyModeToString(previous).c_str(),
    safetyModeToString(safety_mode_).c_str());

  const bool emergency_stop = safety_mode_ == SafetyMode::SYSTEM_EMERGENCY_STOP ||
    safety_mode_ == SafetyMode::ROBOT_EMERGENCY_STOP;
  const bool stopped = state_ == WatchdogState::PROTECTIVE_STOP_ACTIVE ||
    state_ == WatchdogState::EMERGENCY_STOP_ACTIVE;

  if (safety_mode_ == SafetyMode::PROTECTIVE_STOP) {
    state_ = WatchdogState::PROTECTIVE_STOP_ACTIVE;
    protective_stop_count_++;
    holdGripper();
  } else if (emergency_stop) {
    state_ = WatchdogState::EMERGENCY_STOP_ACTIVE;
    emergency_stop_count_++;
    haltOnEmergencyStop();
  } else if (safety_mode_ == SafetyMode::NORMAL && stopped) {
    if (auto_recover_) {
      RCLCPP_INFO(get_logger(), "The robot is back in its normal safety mode: re-arming.");
      recover();
    } else {
      RCLCPP_INFO(
        get_logger(), "The robot is back in its normal safety mode. The hold is kept: call "
        "~/recover_system to re-arm.");
    }
  }
}
