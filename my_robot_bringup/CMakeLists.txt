cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_mrac_system.py
  scripts/debug_system.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
