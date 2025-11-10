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

# Install all necessary directories
install(
  DIRECTORY 
    launch 
    rviz 
    urdf 
    meshes
    config
  DESTINATION share/${PROJECT_NAME}
)

# Install STL files properly
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_export_dependencies(
  ur_onrobot_description 
  ur_description 
  onrobot_description
  ur_robot_driver
  controller_manager
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()