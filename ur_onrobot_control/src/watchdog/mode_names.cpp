// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <string>

#include "ur_onrobot_control/ur_onrobot_safety_watchdog.hpp"

std::string UrOnrobotSafetyWatchdog::safetyModeToString(uint8_t mode)
{
  switch (mode) {
    case SafetyMode::NORMAL: return "NORMAL";
    case SafetyMode::REDUCED: return "REDUCED";
    case SafetyMode::PROTECTIVE_STOP: return "PROTECTIVE_STOP";
    case SafetyMode::RECOVERY: return "RECOVERY";
    case SafetyMode::SAFEGUARD_STOP: return "SAFEGUARD_STOP";
    case SafetyMode::SYSTEM_EMERGENCY_STOP: return "SYSTEM_EMERGENCY_STOP";
    case SafetyMode::ROBOT_EMERGENCY_STOP: return "ROBOT_EMERGENCY_STOP";
    case SafetyMode::VIOLATION: return "VIOLATION";
    case SafetyMode::FAULT: return "FAULT";
    case SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP: return "AUTO_SAFEGUARD_STOP";
    default: return "UNKNOWN (" + std::to_string(mode) + ")";
  }
}

std::string UrOnrobotSafetyWatchdog::robotModeToString(int8_t mode)
{
  switch (mode) {
    case RobotMode::NO_CONTROLLER: return "NO_CONTROLLER";
    case RobotMode::DISCONNECTED: return "DISCONNECTED";
    case RobotMode::CONFIRM_SAFETY: return "CONFIRM_SAFETY";
    case RobotMode::BOOTING: return "BOOTING";
    case RobotMode::POWER_OFF: return "POWER_OFF";
    case RobotMode::POWER_ON: return "POWER_ON";
    case RobotMode::IDLE: return "IDLE";
    case RobotMode::BACKDRIVE: return "BACKDRIVE";
    case RobotMode::RUNNING: return "RUNNING";
    case RobotMode::UPDATING_FIRMWARE: return "UPDATING_FIRMWARE";
    default: return "UNKNOWN (" + std::to_string(mode) + ")";
  }
}

std::string UrOnrobotSafetyWatchdog::watchdogStateToString(WatchdogState state)
{
  switch (state) {
    case WatchdogState::NORMAL: return "NORMAL";
    case WatchdogState::PROTECTIVE_STOP_ACTIVE: return "PROTECTIVE_STOP_ACTIVE";
    case WatchdogState::EMERGENCY_STOP_ACTIVE: return "EMERGENCY_STOP_ACTIVE";
    case WatchdogState::RECOVERING: return "RECOVERING";
    default: return "UNKNOWN";
  }
}
