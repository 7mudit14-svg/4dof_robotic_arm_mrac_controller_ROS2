cmake_minimum_required(VERSION 3.8)
project(mrac_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rclcpp_action     REQUIRED)
find_package(control_msgs      REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(trajectory_msgs   REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 3.3        REQUIRED NO_MODULE)

add_executable(mrac_controller src/mrac_controller.cpp)

ament_target_dependencies(mrac_controller
  rclcpp rclcpp_action control_msgs
  sensor_msgs std_msgs trajectory_msgs builtin_interfaces
)

target_link_libraries(mrac_controller Eigen3::Eigen)

install(TARGETS mrac_controller
  DESTINATION lib/${PROJECT_NAME}
)

# FIXED: install to share root (not share/src/) so launch file path works:
# os.path.join(pkg_mrac, "robust_direct_mrac_params.yaml")
install(FILES src/robust_direct_mrac_params.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
