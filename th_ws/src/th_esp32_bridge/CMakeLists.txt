cmake_minimum_required(VERSION 3.8)
project(th_esp32_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(std_msgs       REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(nav_msgs       REQUIRED)
find_package(tf2_ros        REQUIRED)
find_package(tf2            REQUIRED)
find_package(th_system_msgs REQUIRED)

add_executable(esp32_bridge src/esp32_bridge.cpp)
ament_target_dependencies(esp32_bridge
  rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2 th_system_msgs)

install(TARGETS esp32_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
