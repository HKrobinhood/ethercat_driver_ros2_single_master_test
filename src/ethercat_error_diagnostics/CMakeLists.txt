cmake_minimum_required(VERSION 3.8)
project(ethercat_error_diagnostics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ethercat_msgs REQUIRED)

add_executable(ethercat_error_diagnostics_node src/ethercat_error_diagnostics.cpp)
ament_target_dependencies(ethercat_error_diagnostics_node
  rclcpp
  diagnostic_msgs
  ethercat_msgs
)

install(TARGETS ethercat_error_diagnostics_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
