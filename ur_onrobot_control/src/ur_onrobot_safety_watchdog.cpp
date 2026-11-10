#include "ur_onrobot_control/ur_onrobot_safety_watchdog.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<UrOnrobotSafetyWatchdog>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
