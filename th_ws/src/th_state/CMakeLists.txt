cmake_minimum_required(VERSION 3.8)
project(th_state)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_state/) をインストール。state_core.py / guards.py / zones.py /
# connectivity_core.py は rclpy に依存しない（WP-STATE-01・WP-STATE-02・WP-STATE-03）。
# scripts/ 配下のノードだけが rclpy を使う。
ament_python_install_package(th_state)

install(PROGRAMS
  scripts/state_manager.py
  scripts/connectivity_checker.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
