cmake_minimum_required(VERSION 3.8)
project(th_config_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_config_manager/) をインストール
ament_python_install_package(th_config_manager)

install(PROGRAMS
  scripts/config_manager.py
  scripts/slam_control.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
