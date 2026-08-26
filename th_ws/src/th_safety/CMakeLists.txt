cmake_minimum_required(VERSION 3.8)
project(th_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(std_msgs       REQUIRED)
find_package(std_srvs       REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(th_system_msgs REQUIRED)

# link_quality_core（WP-SAFE-00）: ヘッダオンリーの純粋ロジック層。
add_library(link_quality_core INTERFACE)
target_include_directories(link_quality_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# scan_geometry_core（WP-CALIB-01a）: 角度→LaserScan添字変換のヘッダオンリー
# 純粋ロジック層。th_perception/th_perception/scan_geometry.py の C++移植
# （B-3。std::optional<std::size_t> を使うため C++17 が必要）。
add_library(scan_geometry_core INTERFACE)
target_include_directories(scan_geometry_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(scan_geometry_core INTERFACE cxx_std_17)

# safety_monitor_core（WP-SAFE-01）: ROS2 非依存の判定ロジック層。
add_library(safety_monitor_core src/safety_monitor_core.cpp)
target_include_directories(safety_monitor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(safety_monitor src/safety_monitor.cpp)
target_link_libraries(safety_monitor safety_monitor_core link_quality_core)
ament_target_dependencies(safety_monitor
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs th_system_msgs)

install(TARGETS safety_monitor DESTINATION lib/${PROJECT_NAME})
install(TARGETS safety_monitor_core link_quality_core scan_geometry_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_link_quality_core test/test_link_quality_core.cpp)
  target_link_libraries(test_link_quality_core link_quality_core)

  ament_add_gtest(test_safety_monitor_core test/test_safety_monitor_core.cpp)
  target_link_libraries(test_safety_monitor_core safety_monitor_core)
endif()

ament_package()
