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

# obstacle_limiter_core（DetailedDesign-safety.md §3）: ROS2 非依存の
# obstacle_limiter 判定ロジック層。scan_geometry.hpp（角度→添字変換）に
# 依存するため C++17 が必要。
add_library(obstacle_limiter_core src/obstacle_limiter_core.cpp)
target_include_directories(obstacle_limiter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(obstacle_limiter_core PUBLIC scan_geometry_core)
target_compile_features(obstacle_limiter_core PUBLIC cxx_std_17)

# obstacle_limiter_params（DetailedDesign-wp2.md WP-SAFE-03）: obstacle_limiter
# ノードが行う「ROS パラメータ表 → コア入力値」変換のうち、テストできる部分
# （speed_limit 名の解決・blind_angle_ranges の平坦配列→ペア変換）だけを
# 切り出したヘッダオンリーの純粋ロジック層。
add_library(obstacle_limiter_params INTERFACE)
target_include_directories(obstacle_limiter_params INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(safety_monitor src/safety_monitor.cpp)
target_link_libraries(safety_monitor safety_monitor_core link_quality_core)
ament_target_dependencies(safety_monitor
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs th_system_msgs)

install(TARGETS safety_monitor DESTINATION lib/${PROJECT_NAME})
install(TARGETS safety_monitor_core obstacle_limiter_core obstacle_limiter_params
                link_quality_core scan_geometry_core
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

  # scan_geometry の Python/C++ 等価性テスト（WP-CALIB-01a §7・B-3）。
  # テストベクタ (test/data/scan_geometry_vectors.json) は
  # th_testing/tools/gen_scan_geometry_vectors.py が Python 正本を実際に
  # 呼び出して生成したもの（pybind11 は使わず JSON 経由で突き合わせる）。
  ament_add_gtest(test_scan_geometry_equivalence test/test_scan_geometry_equivalence.cpp)
  target_link_libraries(test_scan_geometry_equivalence scan_geometry_core)
  target_compile_definitions(test_scan_geometry_equivalence PRIVATE
    TH_SAFETY_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

  # obstacle_limiter_core の property test（DetailedDesign-safety.md §3.5・
  # 不変条件 L1〜L7）。乱数シード固定・10,000 通り × 2（単発呼び出しの
  # 不変条件・ヒステリシスの時系列挙動）。
  ament_add_gtest(test_obstacle_limiter_core test/test_obstacle_limiter_core.cpp)
  target_link_libraries(test_obstacle_limiter_core obstacle_limiter_core)

  # observe_cone() の Python/C++ 等価性テスト（DetailedDesign-safety.md
  # §3.4.3。is_path_blocked() を最近傍距離を返す形に拡張した Python版
  # mapless_follow_core.observe_cone() と C++版 obstacle_limiter_core の
  # observe_cone() を突き合わせる）。テストベクタ
  # (test/data/obstacle_cone_vectors.json) は
  # th_testing/tools/gen_obstacle_cone_vectors.py が Python 正本を実際に
  # 呼び出して生成したもの（pybind11 は使わず JSON 経由で突き合わせる）。
  ament_add_gtest(test_obstacle_cone_equivalence test/test_obstacle_cone_equivalence.cpp)
  target_link_libraries(test_obstacle_cone_equivalence obstacle_limiter_core)
  target_compile_definitions(test_obstacle_cone_equivalence PRIVATE
    TH_SAFETY_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

  # obstacle_limiter_params.hpp の単体試験（WP-SAFE-03。obstacle_limiter ノード
  # が行う「ROS パラメータ表 → コア入力値」変換のうちテストできる部分）。
  ament_add_gtest(test_obstacle_limiter_params test/test_obstacle_limiter_params.cpp)
  target_link_libraries(test_obstacle_limiter_params obstacle_limiter_params)
endif()

ament_package()
