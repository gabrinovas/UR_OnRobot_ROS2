cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

# Instalar scripts Python como EJECUTABLES
install(PROGRAMS
  scripts/joint_state_merger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
