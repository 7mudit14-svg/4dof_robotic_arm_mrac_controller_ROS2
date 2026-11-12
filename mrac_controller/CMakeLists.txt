cmake_minimum_required(VERSION 3.8)
project(mrac_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(rclcpp_action        REQUIRED)
find_package(rclcpp_lifecycle     REQUIRED)
find_package(control_msgs         REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(std_srvs             REQUIRED)
find_package(trajectory_msgs      REQUIRED)
find_package(builtin_interfaces   REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface   REQUIRED)
find_package(pluginlib            REQUIRED)
find_package(realtime_tools       REQUIRED)
find_package(yaml_cpp_vendor      REQUIRED)
find_package(yaml-cpp             REQUIRED)
find_package(Eigen3 3.3           REQUIRED NO_MODULE)

# ── ros2_control controller plugin (this is what the launch file uses) ──────
add_library(mrac_controller_plugin SHARED src/mrac_controller_plugin.cpp)
target_include_directories(mrac_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mrac_controller_plugin
  rclcpp rclcpp_action rclcpp_lifecycle control_msgs std_msgs std_srvs trajectory_msgs
  controller_interface hardware_interface pluginlib realtime_tools
)
target_link_libraries(mrac_controller_plugin Eigen3::Eigen yaml-cpp::yaml-cpp)
pluginlib_export_plugin_description_file(controller_interface mrac_plugin.xml)

# ── Legacy standalone node (previous implementation, kept as a fallback; ────
#    not started by the launch file)
add_executable(mrac_controller src/mrac_controller.cpp)
ament_target_dependencies(mrac_controller
  rclcpp rclcpp_action control_msgs
  sensor_msgs std_msgs trajectory_msgs builtin_interfaces
)
target_link_libraries(mrac_controller Eigen3::Eigen)

install(TARGETS mrac_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mrac_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

# Installed to the share root so the launch file finds it at
# <share>/mrac_controller/robust_direct_mrac_params.yaml
install(FILES src/robust_direct_mrac_params.yaml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mrac_core test/test_mrac_core.cpp)
  target_include_directories(test_mrac_core PRIVATE include)
  ament_target_dependencies(test_mrac_core trajectory_msgs)
  target_link_libraries(test_mrac_core Eigen3::Eigen yaml-cpp::yaml-cpp)
endif()

ament_package()
