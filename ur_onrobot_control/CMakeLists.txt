cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# Nodo C++ para Pick & Place con MoveIt Task Constructor
add_executable(ur_onrobot_mtc_node src/ur_onrobot_mtc_node.cpp)
ament_target_dependencies(ur_onrobot_mtc_node
  rclcpp
  moveit_core
  moveit_task_constructor_core
  moveit_ros_planning_interface
  geometry_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_eigen
)

# Nodo C++ para Watchdog y Enclavamientos de Seguridad Industrial UR + OnRobot
add_executable(ur_onrobot_safety_watchdog src/ur_onrobot_safety_watchdog.cpp)
ament_target_dependencies(ur_onrobot_safety_watchdog
  rclcpp
  rclcpp_action
  ur_dashboard_msgs
  diagnostic_updater
  diagnostic_msgs
  control_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS
  ur_onrobot_mtc_node
  ur_onrobot_safety_watchdog
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch urdf config scripts
  DESTINATION share/${PROJECT_NAME}
)

# Instalar scripts Python como EJECUTABLES
install(PROGRAMS
  scripts/joint_state_merger.py
  scripts/selector_tui.py
  scripts/servo_keyboard_teleop.py
  scripts/spawn_and_pick.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_safety_watchdog
    test/test_safety_watchdog.cpp
  )
  ament_target_dependencies(test_safety_watchdog
    rclcpp
    rclcpp_action
    ur_dashboard_msgs
    diagnostic_updater
    diagnostic_msgs
    control_msgs
    sensor_msgs
    std_msgs
    std_srvs
  )
  target_include_directories(test_safety_watchdog PRIVATE
    include
  )

  add_launch_test(test/test_cell_bringup.launch.py)
endif()

ament_package()

