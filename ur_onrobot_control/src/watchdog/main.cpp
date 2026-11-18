// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ur_onrobot_control/ur_onrobot_safety_watchdog.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UrOnrobotSafetyWatchdog>());
  } catch (const std::runtime_error & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ur_onrobot_safety_watchdog"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
