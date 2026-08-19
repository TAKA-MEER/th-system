cmake_minimum_required(VERSION 3.8)
project(th_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(std_msgs       REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(th_system_msgs REQUIRED)

# link_quality_core — ROS2 非依存の純粋コア（ヘッダオンリー。WP-SAFE-00）
add_library(link_quality_core INTERFACE)
target_include_directories(link_quality_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(safety_monitor src/safety_monitor.cpp)
target_link_libraries(safety_monitor link_quality_core)
ament_target_dependencies(safety_monitor
  rclcpp std_msgs sensor_msgs th_system_msgs)

install(TARGETS safety_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES config/twist_mux.yaml config/safety_monitor.yaml
        DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_link_quality_core test/test_link_quality_core.cpp)
  target_link_libraries(test_link_quality_core link_quality_core)
endif()

ament_package()
