cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_semantic_description test/test_semantic_description.py)
  ament_add_pytest_test(test_source_rules test/test_source_rules.py)
endif()

ament_package()
