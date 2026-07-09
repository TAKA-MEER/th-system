cmake_minimum_required(VERSION 3.8)
project(th_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_planning/) をインストール
ament_python_install_package(th_planning)

install(PROGRAMS
  scripts/follow_planner.py
  scripts/follow_planner_mapless.py
  scripts/panel_navigator.py
  scripts/manual_command_handler.py
  scripts/crawler_teleop.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
