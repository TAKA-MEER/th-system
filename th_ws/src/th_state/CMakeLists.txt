cmake_minimum_required(VERSION 3.8)
project(th_state)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python パッケージ (th_state/) をインストール。rclpy には依存しない（WP-STATE-01）。
ament_python_install_package(th_state)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
