cmake_minimum_required(VERSION 3.8)
project(th_params)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_params/) をインストール。rclpy には依存しない（WP-PARAM-01・P-1）。
# scripts/params_audit.py だけが rclpy を使う（WP-PARAM-02）。
ament_python_install_package(th_params)

install(PROGRAMS
  scripts/params_audit.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
