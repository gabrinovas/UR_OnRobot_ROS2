// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef UR_ONROBOT_CONTROL__UR_ONROBOT_SAFETY_WATCHDOG_HPP_
#define UR_ONROBOT_CONTROL__UR_ONROBOT_SAFETY_WATCHDOG_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"

enum class WatchdogState
{
  NORMAL = 0,
  PROTECTIVE_STOP_ACTIVE = 1,
  EMERGENCY_STOP_ACTIVE = 2,
  RECOVERING = 3
};

/// Watches the safety mode of the UR robot and reacts on the gripper:
/// - protective stop: the gripper is told to hold (fingers stay where they are, vacuum goes to
/// full), so
///   that the part is not dropped;
/// - emergency stop: nothing is sent to the gripper, its power is cut by the safety circuit;
/// - back to normal: the tool power is reset and the watchdog is armed again, by itself or on
/// request. Every value is a parameter (config/safety_watchdog.yaml); the node does not start
/// without all of them.
class UrOnrobotSafetyWatchdog : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using Trigger = std_srvs::srv::Trigger;
  using SafetyMode = ur_dashboard_msgs::msg::SafetyMode;
  using RobotMode = ur_dashboard_msgs::msg::RobotMode;

  explicit UrOnrobotSafetyWatchdog(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  WatchdogState getState() const {return state_;}
  uint8_t getSafetyMode() const {return safety_mode_;}
  bool isFailSafeHoldEngaged() const {return hold_engaged_;}
  size_t getProtectiveStopCount() const {return protective_stop_count_;}
  size_t getEstopCount() const {return emergency_stop_count_;}
  size_t getRecoveryCount() const {return recovery_count_;}

  /// Act as if the robot had reported this safety or robot mode (tests and the inject services).
  void triggerSafetyMode(uint8_t mode);
  void triggerRobotMode(int8_t mode);
  /// Re-arm; refused (false) while the robot is not in its normal safety mode.
  bool executeRecovery();

  static std::string safetyModeToString(uint8_t mode);
  static std::string robotModeToString(int8_t mode);
  static std::string watchdogStateToString(WatchdogState state);

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);

private:
  /// Value of a parameter that must be set; throws a [Fail-Fast] error naming it when it is not.
  template<typename T>
  T required(const std::string & name);

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onSafetyMode(const SafetyMode::SharedPtr msg);
  void onRobotMode(const RobotMode::SharedPtr msg);
  void holdGripper();
  void haltOnEmergencyStop();
  bool recover();
  void onRecoverRequest(
    const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
  rclcpp::Service<Trigger>::SharedPtr injectService(const std::string & name, uint8_t mode);
  bool isVacuum() const {return gripper_kind_ == "vacuum";}

  std::string onrobot_type_;
  std::string gripper_kind_;         // finger or vacuum
  std::vector<std::string> joints_;  // joints of the gripper, as they are in the joint states
  std::string gripper_action_;
  std::string reset_power_service_;
  bool auto_recover_;
  bool hold_current_position_;       // hold where the first joint is, instead of at hold_position_
  double hold_position_;
  double hold_max_effort_;
  double action_server_wait_s_;
  double reset_service_wait_s_;
  double telemetry_warning_age_s_;

  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  rclcpp::Subscription<SafetyMode>::SharedPtr safety_mode_subscription_;
  rclcpp::Subscription<RobotMode>::SharedPtr robot_mode_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Client<Trigger>::SharedPtr reset_power_client_;
  rclcpp::Service<Trigger>::SharedPtr recover_service_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> inject_services_;

  std::mutex state_mutex_;
  WatchdogState state_{WatchdogState::NORMAL};
  uint8_t safety_mode_{SafetyMode::NORMAL};
  int8_t robot_mode_{RobotMode::RUNNING};
  bool hold_engaged_{false};
  std::string hold_description_{"Normal operation"};
  size_t protective_stop_count_{0};
  size_t emergency_stop_count_{0};
  size_t recovery_count_{0};

  std::mutex data_mutex_;
  std::vector<double> joint_position_;  // last position of every joint of joints_
  bool has_telemetry_{false};
  rclcpp::Time last_telemetry_time_;
};

template<typename T>
T UrOnrobotSafetyWatchdog::required(const std::string & name)
{
  // A required parameter is one the launch file (or a parameter file) gives a value to.
  const auto & given = get_node_parameters_interface()->get_parameter_overrides();
  const auto found = given.find(name);
  if (found != given.end()) {
    if (!has_parameter(name)) {
      declare_parameter(name, found->second);
    }
    return get_parameter(name).get_value<T>();
  }
  throw std::runtime_error(
          "[Fail-Fast] Parameter '" + name + "' of node '" + get_fully_qualified_name() +
          "' is not set");
}

#endif  // UR_ONROBOT_CONTROL__UR_ONROBOT_SAFETY_WATCHDOG_HPP_
