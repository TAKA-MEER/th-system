cmake_minimum_required(VERSION 3.8)
project(th_testing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

if(BUILD_TESTING)
  # ── 純粋単体テスト (ROS2 不要・高速) ────────────────────
  ament_add_pytest_test(test_follow_logic
    test/test_follow_planner_logic.py
    TIMEOUT 30)
  ament_add_pytest_test(test_summon_logic
    test/test_summon_logic.py
    TIMEOUT 30)
  ament_add_pytest_test(test_esp32_ws_protocol
    test/test_esp32_ws_protocol.py
    TIMEOUT 60)
  ament_add_pytest_test(test_mapless_follow_logic
    test/test_mapless_follow_logic.py
    TIMEOUT 60)
  ament_add_pytest_test(test_person_tracker_bridge_logic
    test/test_person_tracker_bridge_logic.py
    TIMEOUT 60)
  ament_add_pytest_test(test_scenario_configs
    test/test_scenario_configs.py
    TIMEOUT 60)
  ament_add_pytest_test(test_yaml_writer
    test/test_yaml_writer.py
    TIMEOUT 60)
  # esp32_bridge の純粋コア (WP-SAFE-02。DEBT-4: /cmd_vel stale タイムアウト
  # ＋ オドメトリ積分・スタンプ再同期の抽出。ROS2 不要)
  ament_add_pytest_test(keepalive_core
    test/test_keepalive_core.py
    TIMEOUT 60)
  ament_add_pytest_test(test_odom_core
    test/test_odom_core.py
    TIMEOUT 60)
  # D-1: WHEEL_CMD 送信コアレッシング (実機計測 2026-08 の受信ギャップ
  # 切り分け作業。in-flight 中の送信要求を保留フレームの上書きで束ねる。
  # ROS2 不要)
  ament_add_pytest_test(test_send_coalescer
    test/test_send_coalescer.py
    TIMEOUT 60)
  # D-2: 受信キューの有界化 (実機計測 2026-08 の受信ギャップ切り分け作業。
  # maxsize 溢れは古い方を捨て、1回の drain の処理件数にも上限を設ける。
  # ROS2 不要)
  ament_add_pytest_test(test_rx_backpressure
    test/test_rx_backpressure.py
    TIMEOUT 60)
  # CMakeLists.txt へのテスト登録漏れを検出するガード（このファイル自身と
  # ament_add_gtest 登録も対象。上記5本が長期間未登録だった事故の再発防止）
  ament_add_pytest_test(test_cmake_test_registration
    test/test_cmake_test_registration.py
    TIMEOUT 60)

  # ── 型定義テスト (th_system_msgs の生成物が要る・ノード起動は不要) ──
  ament_add_pytest_test(test_msg_definitions
    test/test_msg_definitions.py
    TIMEOUT 60)

  # ── 状態機械コア (th_state。ROS2 不要・純粋単体テスト。WP-STATE-01) ──
  ament_add_pytest_test(test_state_core
    test/test_state_core.py
    TIMEOUT 60)
  ament_add_pytest_test(test_transition_table
    test/test_transition_table.py
    TIMEOUT 60)
  ament_add_pytest_test(connectivity_core
    test/test_connectivity_core.py
    TIMEOUT 60)

  # ── パラメータ registry コア (th_params。ROS2 不要・純粋単体テスト。WP-PARAM-01) ──
  ament_add_pytest_test(test_params_schema
    test/test_params_schema.py
    TIMEOUT 60)
  ament_add_pytest_test(test_params_derive
    test/test_params_derive.py
    TIMEOUT 60)
  ament_add_pytest_test(test_params_assertions
    test/test_params_assertions.py
    TIMEOUT 60)
  ament_add_pytest_test(test_params_export
    test/test_params_export.py
    TIMEOUT 60)
  ament_add_pytest_test(test_params_registry
    test/test_params_registry.py
    TIMEOUT 60)
  ament_add_pytest_test(test_params_launch
    test/test_params_launch.py
    TIMEOUT 60)

  # ── ROS2 ノード統合テスト ────────────────────────────────
  ament_add_pytest_test(test_mode_transitions
    test/test_mode_transitions.py
    TIMEOUT 120)
  ament_add_pytest_test(test_state_manager_node
    test/test_state_manager_node.py
    TIMEOUT 120)
  ament_add_pytest_test(connectivity_checker_node
    test/test_connectivity_checker_node.py
    TIMEOUT 120)
  ament_add_pytest_test(test_params_audit_node
    test/test_params_audit_node.py
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
  # esp32_bridge ノード統合テスト (WP-SAFE-02。故障注入12「/cmd_vel の途絶」)
  ament_add_pytest_test(esp32_bridge_node
    test/test_esp32_bridge_node.py
    TIMEOUT 90)

  # ── シミュレーション統合テスト (TH_SKIP_SIM=0 で有効化) ──
  ament_add_pytest_test(test_simulation
    test/test_simulation_scenarios.py
    TIMEOUT 120
    ENV TH_SKIP_SIM=1)   # CI ではデフォルトでスキップ
endif()

ament_package()
