// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "ur_onrobot_control/ur_onrobot_safety_watchdog.hpp"

void UrOnrobotSafetyWatchdog::holdGripper()
{
  hold_engaged_ = true;
  RCLCPP_ERROR(
    get_logger(), "Protective stop: telling the %s to hold what it has.", onrobot_type_.c_str());

  const auto wait = std::chrono::duration<double>(action_server_wait_s_);
  if (!gripper_client_->wait_for_action_server(wait)) {
    RCLCPP_ERROR(
      get_logger(), "The gripper action '%s' is not available: the hold was not sent.",
      gripper_action_.c_str());
    return;
  }

  GripperCommand::Goal goal;
  goal.command.max_effort = hold_max_effort_;
  char description[96];
  if (hold_current_position_) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      goal.command.position = joint_position_.front();
    }
    snprintf(
      description, sizeof(description), "Held where it was: %.4f (effort %.0f)",
      goal.command.position, hold_max_effort_);
  } else {
    goal.command.position = hold_position_;
    snprintf(
      description, sizeof(description), "Held at %.2f (effort %.0f)", hold_position_,
      hold_max_effort_);
  }
  hold_description_ = description;
  RCLCPP_INFO(get_logger(), "Hold command sent. %s", description);
  gripper_client_->async_send_goal(goal);
}

void UrOnrobotSafetyWatchdog::haltOnEmergencyStop()
{
  hold_engaged_ = true;
  hold_description_ = "Emergency stop: nothing is sent to the gripper (safety circuit open)";
  RCLCPP_ERROR(
    get_logger(),
    "Emergency stop: no command is sent to the gripper while the safety circuit is open.");
}

bool UrOnrobotSafetyWatchdog::recover()
{
  state_ = WatchdogState::RECOVERING;
  const auto wait = std::chrono::duration<double>(reset_service_wait_s_);
  if (reset_power_client_->wait_for_service(wait)) {
    reset_power_client_->async_send_request(std::make_shared<Trigger::Request>());
    RCLCPP_INFO(get_logger(), "Tool power reset requested on %s.", reset_power_service_.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(), "The service %s is not available: the tool power is not reset.",
      reset_power_service_.c_str());
  }

  state_ = WatchdogState::NORMAL;
  hold_engaged_ = false;
  recovery_count_++;
  hold_description_ = "Normal operation";
  RCLCPP_INFO(get_logger(), "Re-armed: the watchdog is back in its normal state.");
  return true;
}

bool UrOnrobotSafetyWatchdog::executeRecovery()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (safety_mode_ != SafetyMode::NORMAL) {
    return false;
  }
  return recover();
}

void UrOnrobotSafetyWatchdog::onRecoverRequest(
  const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (safety_mode_ != SafetyMode::NORMAL) {
    response->success = false;
    response->message = "Cannot re-arm: the UR robot is not in its normal safety mode (it is in " +
      safetyModeToString(safety_mode_) + ")";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }
  response->success = recover();
  response->message = response->success ? "UR + OnRobot re-armed" : "The recovery failed";
}
