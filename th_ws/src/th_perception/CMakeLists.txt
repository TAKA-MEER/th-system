cmake_minimum_required(VERSION 3.8)
project(th_perception)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
install(PROGRAMS
  scripts/lidar_filter.py
  scripts/person_predictor.py
  scripts/person_tracker_stub.py
  scripts/gazebo_person_relay.py
  scripts/person_mover.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
