cmake_minimum_required(VERSION 3.8)
project(th_calibration)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
install(PROGRAMS
  scripts/linear_calib.py
  scripts/rotation_calib.py
  scripts/apply_calib.py
  scripts/imu_calib_check.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
