// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ur_onrobot_control/ur_onrobot_safety_watchdog.hpp"

namespace
{
using diagnostic_msgs::msg::DiagnosticStatus;
using SafetyMode = ur_dashboard_msgs::msg::SafetyMode;
using RobotMode = ur_dashboard_msgs::msg::RobotMode;

/// Every parameter the watchdog requires, as the launch file gives them for a finger gripper.
rclcpp::NodeOptions watchdog_options()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("onrobot_type", "2fg7");
  options.append_parameter_override("gripper_kind", "finger");
  options.append_parameter_override("joints", std::vector<std::string>{"finger_width"});
  options.append_parameter_override(
    "gripper_action",
    "/onrobot/gripper_action_controller/gripper_cmd");
  options.append_parameter_override("reset_power_service", "/onrobot/reset_power");
  options.append_parameter_override("auto_recover", false);
  options.append_parameter_override("hold_current_position", true);
  options.append_parameter_override("hold_position", 0.0);
  options.append_parameter_override("hold_max_effort", 140.0);
  options.append_parameter_override("action_server_wait_s", 0.05);
  options.append_parameter_override("reset_service_wait_s", 0.05);
  options.append_parameter_override("telemetry_warning_age_s", 3.0);
  options.append_parameter_override("queue_depth", 10);
  options.append_parameter_override("diagnostic_period_s", 1.0);
  options.append_parameter_override("safety_mode_topic", "/io_and_status_controller/safety_mode");
  options.append_parameter_override("robot_mode_topic", "/io_and_status_controller/robot_mode");
  options.append_parameter_override("joint_states_topic", "/merged_joint_states");
  return options;
}

class SafetyWatchdogTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {node = std::make_shared<UrOnrobotSafetyWatchdog>(watchdog_options());}

  std::shared_ptr<UrOnrobotSafetyWatchdog> node;
};

TEST_F(SafetyWatchdogTest, AMissingParameterStopsTheNodeWithAFailFastError)
{
  rclcpp::NodeOptions incomplete;
  incomplete.append_parameter_override("onrobot_type", "2fg7");
  try {
    UrOnrobotSafetyWatchdog watchdog(incomplete);
    FAIL() << "the watchdog started without its parameters";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("[Fail-Fast]"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("gripper_kind"), std::string::npos);
  }
}

TEST_F(SafetyWatchdogTest, ModesAndStatesHaveNames)
{
  EXPECT_EQ(UrOnrobotSafetyWatchdog::safetyModeToString(SafetyMode::NORMAL), "NORMAL");
  EXPECT_EQ(
    UrOnrobotSafetyWatchdog::safetyModeToString(
      SafetyMode::PROTECTIVE_STOP), "PROTECTIVE_STOP");
  EXPECT_EQ(UrOnrobotSafetyWatchdog::safetyModeToString(99), "UNKNOWN (99)");
  EXPECT_EQ(UrOnrobotSafetyWatchdog::robotModeToString(RobotMode::RUNNING), "RUNNING");
  EXPECT_EQ(UrOnrobotSafetyWatchdog::robotModeToString(RobotMode::POWER_OFF), "POWER_OFF");
  EXPECT_EQ(
    UrOnrobotSafetyWatchdog::watchdogStateToString(WatchdogState::RECOVERING),
    "RECOVERING");
}

TEST_F(SafetyWatchdogTest, StartsInTheNormalState)
{
  EXPECT_EQ(node->getState(), WatchdogState::NORMAL);
  EXPECT_FALSE(node->isFailSafeHoldEngaged());
  EXPECT_EQ(node->getProtectiveStopCount(), 0u);
  EXPECT_EQ(node->getEstopCount(), 0u);
  EXPECT_EQ(node->getRecoveryCount(), 0u);
}

TEST_F(SafetyWatchdogTest, AProtectiveStopEngagesTheHoldAndBlocksTheRecovery)
{
  node->triggerSafetyMode(SafetyMode::PROTECTIVE_STOP);
  EXPECT_EQ(node->getState(), WatchdogState::PROTECTIVE_STOP_ACTIVE);
  EXPECT_TRUE(node->isFailSafeHoldEngaged());
  EXPECT_EQ(node->getProtectiveStopCount(), 1u);

  // The robot is still stopped: re-arming is refused.
  EXPECT_FALSE(node->executeRecovery());
  EXPECT_EQ(node->getState(), WatchdogState::PROTECTIVE_STOP_ACTIVE);
}

TEST_F(SafetyWatchdogTest, AnEmergencyStopIsCounted)
{
  node->triggerSafetyMode(SafetyMode::ROBOT_EMERGENCY_STOP);
  EXPECT_EQ(node->getState(), WatchdogState::EMERGENCY_STOP_ACTIVE);
  EXPECT_TRUE(node->isFailSafeHoldEngaged());
  EXPECT_EQ(node->getEstopCount(), 1u);
}

TEST_F(SafetyWatchdogTest, TheRecoveryGoesBackToNormalOnceTheRobotIsNormal)
{
  node->triggerSafetyMode(SafetyMode::PROTECTIVE_STOP);
  node->triggerSafetyMode(SafetyMode::NORMAL);
  EXPECT_EQ(node->getState(), WatchdogState::PROTECTIVE_STOP_ACTIVE);  // auto_recover is off

  EXPECT_TRUE(node->executeRecovery());
  EXPECT_EQ(node->getState(), WatchdogState::NORMAL);
  EXPECT_FALSE(node->isFailSafeHoldEngaged());
  EXPECT_EQ(node->getRecoveryCount(), 1u);
}

TEST_F(SafetyWatchdogTest, TheDiagnosticsFollowTheState)
{
  diagnostic_updater::DiagnosticStatusWrapper normal;
  node->produceDiagnostics(normal);
  EXPECT_EQ(normal.level, DiagnosticStatus::OK);
  EXPECT_NE(normal.message.find("Watchdog operational"), std::string::npos);

  node->triggerSafetyMode(SafetyMode::PROTECTIVE_STOP);
  diagnostic_updater::DiagnosticStatusWrapper protective_stop;
  node->produceDiagnostics(protective_stop);
  EXPECT_EQ(protective_stop.level, DiagnosticStatus::WARN);

  node->triggerSafetyMode(SafetyMode::SYSTEM_EMERGENCY_STOP);
  diagnostic_updater::DiagnosticStatusWrapper emergency_stop;
  node->produceDiagnostics(emergency_stop);
  EXPECT_EQ(emergency_stop.level, DiagnosticStatus::ERROR);
}

}  // namespace
