cmake_minimum_required(VERSION 3.10)
project(multiple_sensor_person_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pluginlib REQUIRED)
find_package(multiple_observation_kalman_filter REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FollowingPosition.msg"
  "srv/LegDetectionService.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

include_directories(
  include
)

# LiDAR-only build: only the PersonTracker component (leg tracking) is built.
# The camera-based sensor rotator / re-identification nodes are intentionally
# excluded from this minimal, robot-independent leg-detection package.
add_library(${PROJECT_NAME}_component SHARED
  src/multiple_sensor_person_tracker_component.cpp
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  vision_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  laser_geometry
  pluginlib
  multiple_observation_kalman_filter
)

target_link_libraries(${PROJECT_NAME}_component
  ${multiple_observation_kalman_filter_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../multiple_observation_kalman_filter/include
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_component "${cpp_typesupport_target}")

# Register nodes as components explicitly
rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "multiple_sensor_person_tracking::PersonTracker"
)

install(TARGETS ${PROJECT_NAME}_component
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  component_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rclcpp_components component_description.xml)

ament_package()
