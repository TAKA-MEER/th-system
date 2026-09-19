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
# obstacle_limiter（WP-SAFE-03）: 起動時に 1 度だけ base_link<-laser_link を
# 取得するために要る（names.md §1.2。20Hz では TF を引かない）。
find_package(tf2               REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
# jog_gate（WP-SAFE-04）: attributes.yaml（th_state と同じファイル）を読むのに要る。
find_package(ament_index_cpp   REQUIRED)
find_package(yaml_cpp_vendor   REQUIRED)

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

# obstacle_limiter（WP-SAFE-03）: 判定ロジックは持たない。ROS の世界と
# obstacle_limiter_core.hpp の相互変換だけを行うノード（src/obstacle_limiter.cpp
# 冒頭コメント参照）。launch 配線・twist_mux.yaml の改名は別パケット。
add_executable(obstacle_limiter src/obstacle_limiter.cpp)
target_link_libraries(obstacle_limiter obstacle_limiter_core obstacle_limiter_params)
ament_target_dependencies(obstacle_limiter
  rclcpp std_msgs sensor_msgs geometry_msgs th_system_msgs tf2 tf2_ros tf2_geometry_msgs)

# jog_gate_core（DetailedDesign-wp2.md WP-SAFE-04 §4.1）: 手動ジョグ指令の
# ゲート判定ロジック。attributes.yaml の読込（yaml-cpp）を含む。
# attributes.yaml は th_state と同じファイルを読む（J-4）。
add_library(jog_gate_core src/jog_gate_core.cpp)
target_include_directories(jog_gate_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(jog_gate_core PUBLIC yaml-cpp)

# jog_gate（WP-SAFE-04）: /cmd_vel_manual_raw → /cmd_vel_manual のゲートノード。
# 判定は jog_gate_core に持ち、このノードは sub/pub と パラメータ/ファイル
# 解決だけを行う。通さないときは沈黙する（ゼロを撃たない。J-1）。
add_executable(jog_gate src/jog_gate.cpp)
target_link_libraries(jog_gate jog_gate_core)
ament_target_dependencies(jog_gate rclcpp geometry_msgs th_system_msgs ament_index_cpp)

install(TARGETS safety_monitor obstacle_limiter jog_gate DESTINATION lib/${PROJECT_NAME})
install(TARGETS safety_monitor_core obstacle_limiter_core obstacle_limiter_params
                link_quality_core scan_geometry_core jog_gate_core
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

  # WS-9P: 障害物とみなすのに必要な点数（ノイズ 1 点で止まらない）。
  ament_add_gtest(test_obstacle_min_points test/test_obstacle_min_points.cpp)
  target_link_libraries(test_obstacle_min_points obstacle_limiter_core)

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

  # jog_gate_core の単体試験（DetailedDesign-wp2.md WP-SAFE-04 §7。
  # 不変条件 J-1〜J-3・F-28・18 モード）。テスト名は設計書 §7 に固定。
  ament_add_gtest(test_jog_gate_core test/test_jog_gate_core.cpp)
  target_link_libraries(test_jog_gate_core jog_gate_core)
  # AllModesFromAttributes は th_state が持つ実ファイルを読む（J-4:
  # th_state の jog_allowed と同じ attributes.yaml を読む）。表を写した
  # テストにすると yaml を書き換えても緑のまま通ってしまう（実測済み）。
  # 流儀は test_scan_geometry_equivalence の TH_SAFETY_TEST_DATA_DIR と同じ。
  target_compile_definitions(test_jog_gate_core PRIVATE
    TH_STATE_ATTRIBUTES_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../th_state/config/attributes.yaml")
endif()

ament_package()
