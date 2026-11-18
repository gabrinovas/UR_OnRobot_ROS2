cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

include_directories(include)

set(WATCHDOG_DEPENDENCIES
  control_msgs
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_action
  sensor_msgs
  std_srvs
  ur_dashboard_msgs
)

# The safety watchdog, as a library (also linked by its test) and its node.
add_library(ur_onrobot_safety_watchdog_objects OBJECT
  src/watchdog/watchdog_node.cpp
  src/watchdog/safety_reaction.cpp
  src/watchdog/diagnostics.cpp
  src/watchdog/mode_names.cpp
)
ament_target_dependencies(ur_onrobot_safety_watchdog_objects ${WATCHDOG_DEPENDENCIES})

add_executable(ur_onrobot_safety_watchdog
  src/watchdog/main.cpp
  $<TARGET_OBJECTS:ur_onrobot_safety_watchdog_objects>
)
ament_target_dependencies(ur_onrobot_safety_watchdog ${WATCHDOG_DEPENDENCIES})

ament_python_install_package(${PROJECT_NAME})

install(TARGETS ur_onrobot_safety_watchdog DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/joint_state_merger.py
  scripts/selector_tui.py
  scripts/servo_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_safety_watchdog
    test/test_safety_watchdog.cpp
    $<TARGET_OBJECTS:ur_onrobot_safety_watchdog_objects>
  )
  ament_target_dependencies(test_safety_watchdog ${WATCHDOG_DEPENDENCIES})

  ament_add_pytest_test(test_configuration test/test_configuration.py)
  ament_add_pytest_test(test_source_rules test/test_source_rules.py)
  add_launch_test(test/test_cell_bringup.launch.py)
endif()

ament_package()
