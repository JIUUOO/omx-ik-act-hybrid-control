cmake_minimum_required(VERSION 3.10)
project(omx_ik_act_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
