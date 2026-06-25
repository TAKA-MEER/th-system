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

add_executable(safety_monitor src/safety_monitor.cpp)
ament_target_dependencies(safety_monitor
  rclcpp std_msgs sensor_msgs th_system_msgs)

install(TARGETS safety_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES config/twist_mux.yaml config/safety_monitor.yaml
        DESTINATION share/${PROJECT_NAME}/config)

ament_package()
