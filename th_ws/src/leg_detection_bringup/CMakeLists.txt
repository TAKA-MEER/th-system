cmake_minimum_required(VERSION 3.10)
project(leg_detection_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  param
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
