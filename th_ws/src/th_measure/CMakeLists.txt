cmake_minimum_required(VERSION 3.8)
project(th_measure)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_measure/) をインストール。link_core.py は rclpy に
# 依存しない。scripts/ 配下の measure_link.py だけが rclpy を使う。
ament_python_install_package(th_measure)

install(PROGRAMS
  scripts/measure_link.py
  scripts/measure_clearance.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
