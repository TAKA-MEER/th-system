cmake_minimum_required(VERSION 3.8)
project(th_esp32_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_esp32_bridge/) をインストール
ament_python_install_package(th_esp32_bridge)

install(PROGRAMS
  scripts/esp32_bridge.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
