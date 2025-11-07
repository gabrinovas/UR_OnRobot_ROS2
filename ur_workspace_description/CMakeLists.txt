cmake_minimum_required(VERSION 3.8)
project(ur_workspace_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(ur_onrobot_description REQUIRED)
find_package(ur_description REQUIRED)
find_package(onrobot_description REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ur_onrobot_description ur_description onrobot_description)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()