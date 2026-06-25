cmake_minimum_required(VERSION 3.8)
project(th_testing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

if(BUILD_TESTING)
  # ── 純粋単体テスト (ROS2 不要・高速) ────────────────────
  ament_add_pytest_test(test_follow_logic
    test/test_follow_planner_logic.py
    TIMEOUT 30)

  # ── ROS2 ノード統合テスト ────────────────────────────────
  ament_add_pytest_test(test_mode_transitions
    test/test_mode_transitions.py
    TIMEOUT 120)
  ament_add_pytest_test(test_safety_monitor
    test/test_safety_monitor.py
    TIMEOUT 90)
  ament_add_pytest_test(test_twist_mux
    test/test_twist_mux_priority.py
    TIMEOUT 60)
  ament_add_pytest_test(test_fault_detection
    test/test_fault_detection.py
    TIMEOUT 90)

  # ── シミュレーション統合テスト (TH_SKIP_SIM=0 で有効化) ──
  ament_add_pytest_test(test_simulation
    test/test_simulation_scenarios.py
    TIMEOUT 120
    ENV TH_SKIP_SIM=1)   # CI ではデフォルトでスキップ
endif()

ament_package()
