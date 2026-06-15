cmake_minimum_required(VERSION 3.16)
project(omx_cyclo_safe_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cyclo_motion_controller_core REQUIRED)
find_package(cyclo_motion_controller_models REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(omx_safe_movel_controller_node
  src/omx_safe_movel_controller_node.cpp
)

ament_target_dependencies(omx_safe_movel_controller_node
  geometry_msgs
  rclcpp
  robotis_interfaces
  sensor_msgs
  std_msgs
  trajectory_msgs
)

target_link_libraries(omx_safe_movel_controller_node
  cyclo_motion_controller_core::cyclo_motion_controller_core
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS omx_safe_movel_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
