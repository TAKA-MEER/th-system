cmake_minimum_required(VERSION 3.8)
project(th_maintenance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_maintenance/) をインストール
ament_python_install_package(th_maintenance)

install(PROGRAMS
  scripts/opcheck_runner.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()