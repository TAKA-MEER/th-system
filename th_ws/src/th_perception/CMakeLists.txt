cmake_minimum_required(VERSION 3.8)
project(th_perception)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python パッケージ (th_perception/) をインストール（WP-CALIB-01a。
# scan_geometry.py は th_planning と同じ流儀で <pkg>/<pkg>/ に置く）
ament_python_install_package(th_perception)

install(PROGRAMS
  scripts/lidar_filter.py
  scripts/person_predictor.py
  scripts/person_tracker_stub.py
  scripts/person_tracker_bridge.py
  scripts/gazebo_person_relay.py
  scripts/person_mover.py
  scripts/obstacle_mover.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES
  scripts/person_tracker_bridge_core.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
