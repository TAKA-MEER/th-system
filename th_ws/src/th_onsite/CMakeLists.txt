cmake_minimum_required(VERSION 3.8)
project(th_onsite)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_onsite/) をインストール
ament_python_install_package(th_onsite)

install(PROGRAMS
  scripts/pin_registrar.py
  scripts/wait_clear_gate.py
  scripts/venue_navigator.py
  scripts/home_declarer.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
