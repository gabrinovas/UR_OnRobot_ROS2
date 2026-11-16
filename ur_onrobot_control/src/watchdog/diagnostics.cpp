// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "ur_onrobot_control/ur_onrobot_safety_watchdog.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;

void UrOnrobotSafetyWatchdog::produceDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  std::lock_guard<std::mutex> data_lock(data_mutex_);

  status.add("Gripper Model", onrobot_type_);
  status.add("Watchdog State", watchdogStateToString(state_));
  status.add("UR Safety Mode", safetyModeToString(safety_mode_));
  status.add("UR Robot Mode", robotModeToString(robot_mode_));
  status.add("Fail-Safe Hold Engaged", hold_engaged_ ? "YES" : "NO");
  status.add("Interlock Status", hold_description_);
  status.add("Protective Stop Events", protective_stop_count_);
  status.add("Emergency Stop Events", emergency_stop_count_);
  status.add("Successful Recoveries", recovery_count_);

  double telemetry_age = 0.0;
  if (has_telemetry_) {
    telemetry_age = (now() - last_telemetry_time_).seconds();
    status.addf("Telemetry Age (s)", "%.3f", telemetry_age);
  } else {
    status.add("Telemetry Age (s)", "no telemetry yet");
  }
  for (size_t joint = 0; joint < joints_.size(); ++joint) {
    if (isVacuum()) {
      status.addf("Vacuum " + joints_[joint], "%.1f %%", joint_position_[joint] * 100.0);
    } else {
      status.addf("Position " + joints_[joint] + " (mm)", "%.2f", joint_position_[joint] * 1000.0);
    }
  }

  if (state_ == WatchdogState::EMERGENCY_STOP_ACTIVE) {
    status.summary(
      DiagnosticStatus::ERROR,
      "EMERGENCY STOP ACTIVE: safety circuit open, nothing is sent to the gripper.");
  } else if (state_ == WatchdogState::PROTECTIVE_STOP_ACTIVE) {
    status.summary(DiagnosticStatus::WARN, "PROTECTIVE STOP ACTIVE: the gripper is holding.");
  } else if (has_telemetry_ && telemetry_age > telemetry_warning_age_s_) {
    status.summaryf(
      DiagnosticStatus::WARN, "Joint telemetry delayed (> %.1f s)", telemetry_warning_age_s_);
  } else {
    status.summary(
      DiagnosticStatus::OK, "Watchdog operational: robot and gripper in normal safety conditions.");
  }
}
