cmake_minimum_required(VERSION 3.5)
project(ur_onrobot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Instalar directorios
install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

# Instalar paquete Python
ament_python_install_package(${PROJECT_NAME})

# Instalar módulos Python adicionales
install(PROGRAMS
  ${PROJECT_NAME}/launch_common.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
