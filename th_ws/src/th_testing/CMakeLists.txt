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
  ament_add_pytest_test(test_route_record_core
    test/test_route_record_core.py
    TIMEOUT 30)
  # WS-9H: 経路ファイル保存系（.wip 自動保存 / .prev 世代退避 / 原子的書き込み。
  # route_record_core の純関数を実ファイルで検証。ROS2 不要）
  ament_add_pytest_test(test_route_record_files
    test/test_route_record_files.py
    TIMEOUT 30)
  # WS-9K-B: 地図セッション ID。経路に記録セッションの印を付け、別セッションの
  # map 経路を再生に進ませない判定 (can_replay_route) を固定。ROS2 不要
  ament_add_pytest_test(test_route_map_session
    test/test_route_map_session.py
    TIMEOUT 30)
  # WS-9L: 地図の凍結・保存・再読込。Pause トグルの収束ロジック / /map_session/open
  # の引数検証・id 無害化（slam_control_logic 純関数）と配線の ast 静的検証。ROS2 不要
  ament_add_pytest_test(test_slam_control_logic
    test/test_slam_control_logic.py
    TIMEOUT 30)
  # WS-9U: call_and_wait が再入 spin をやめてポーリングになったことの ast 検証と、
  # fake client での戻り値・タイムアウト挙動。ROS2 不要
  ament_add_pytest_test(test_service_call
    test/test_service_call.py
    TIMEOUT 30)
  # WS-9W: WebUI 調整対象（tunable_targets）が参照先 YAML と SettingsPanel.jsx の
  # FIELDS 配列と名前一致していることの固定。ROS2 不要
  ament_add_pytest_test(test_tunable_targets
    test/test_tunable_targets.py
    TIMEOUT 30)
  # WS-9O: 重大フォルトの保持時間（単発の誤検知で ESTOP に落ちない）と、
  # しきい値を緩める方向へ流れていないことの静的検証。ROS2 不要
  ament_add_pytest_test(test_critical_fault_hold
    test/test_critical_fault_hold.py
    TIMEOUT 30)
  # WS-9P: 一時停止の理由（RouteStatus.arrived）と障害物とみなす点数の
  # ノード側配線。gtest / playwright が届かない範囲だけ。ROS2 不要
  ament_add_pytest_test(test_pause_reason_and_min_points
    test/test_pause_reason_and_min_points.py
    TIMEOUT 30)
  # WS-9Q: /route/status を出すのは自分のモードのノードだけ（交互配信で
  # route_loaded が往復し「再生」が拒否されるのを防ぐ）。ROS2 不要
  ament_add_pytest_test(test_route_status_ownership
    test/test_route_status_ownership.py
    TIMEOUT 30)
  # WS-9F: /route/preview の帯域対策の静的テスト（decimate_polyline 適用と
  # ハートビート配信の設計判断を固定。ROS2 不要）
  ament_add_pytest_test(test_route_preview_bandwidth
    test/test_route_preview_bandwidth.py
    TIMEOUT 30)
  # WS-9G: /map を表示用に間引く純関数 downsample_occupancy のテスト（ROS2 不要）
  ament_add_pytest_test(test_map_downsample
    test/test_map_downsample.py
    TIMEOUT 30)
  # install(PROGRAMS ...) に載せたスクリプトが実際に起動できる形か（実行ビット・
  # shebang）。実機で map_downsampler.py が「libexec に無い」と言われた事故の再発防止。
  # core.fileMode=false のため chmod だけでは index に反映されない点も含めて固定する
  ament_add_pytest_test(test_installed_scripts_executable
    test/test_installed_scripts_executable.py
    TIMEOUT 30)
  ament_add_pytest_test(test_route_replay_core
    test/test_route_replay_core.py
    TIMEOUT 30)
  # 自己位置源選択コア (WS-9D。EKF 出力 /odometry/filtered が新鮮ならそれ、
  # 古ければ生 /odom にフォールバック。ROS2 不要)
  ament_add_pytest_test(test_odom_source
    test/test_odom_source.py
    TIMEOUT 30)
  # 角度 → LaserScan 添字の変換規約（WP-CALIB-01a。DetailedDesign-wp2.md
  # WP-CALIB-01 §7。登録名は設計書どおり scan_geometry）
  ament_add_pytest_test(scan_geometry
    test/test_scan_geometry.py
    TIMEOUT 30)
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
  # D-3: 実行時計器 (実機計測 2026-08 の受信ギャップ切り分け作業。
  # drain タイマー間隔・drain 処理時間・WHEEL_FEEDBACK 到着間隔の最大値、
  # および frames/drop/invalid の累積カウントを1Hzサマリで出す。ROS2不要)
  ament_add_pytest_test(test_bridge_diagnostics
    test/test_bridge_diagnostics.py
    TIMEOUT 60)
  # WS-9J-A: 1Hz 診断サマリ (esp32_bridge_diag:) を既定で黙らせ、悪化した窓だけ
  # warn で出すゲート。diag_window_is_abnormal 純関数と esp32_bridge.py の
  # ast 静的検証 (diag_log_period_s の declare / 無条件 create_timer が無いこと)。
  # 実機ログ 2026-09-03 で 268 行中 178 行 (66%) がこのサマリだった。ROS2 不要
  ament_add_pytest_test(test_esp32_bridge_diag_gate
    test/test_esp32_bridge_diag_gate.py
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
  # 「再生を押しても機体が動かない」の回帰試験 (2026-09-02 実機。
  #  準備状態が PAUSE へ落ちて経路選択が飛ばされる不具合)
  ament_add_pytest_test(test_replay_route_selection
    test/test_replay_route_selection.py
    TIMEOUT 60)
  # EKF 設定の整合性 (2026-09-02: imu0 のトピック指定行が欠落していて
  #  ジャイロが無言で融合されていなかった件の回帰試験)
  ament_add_pytest_test(test_ekf_config
    test/test_ekf_config.py
    TIMEOUT 60)
  ament_add_pytest_test(connectivity_core
    test/test_connectivity_core.py
    TIMEOUT 60)
  # ゾーン合成コア (WP-STATE-02。derive_limits() の速度上限が §4.1 の規則
  # 「合成は各画面の最小値。ゾーンの強弱では合成しない」を満たすことの担保。N-13)
  ament_add_pytest_test(test_zones
    test/test_zones.py
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
  # th_state/zones.py の LIMIT_STRICTNESS と registry.yaml の
  # speed_limit_strictness_rank が一致することの機械的な照合（N-15 レビュー。
  # th_state と th_params の両方を import してよい唯一のテストコード）
  ament_add_pytest_test(test_state_registry_consistency
    test/test_state_registry_consistency.py
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
  # jog_gate ノード統合テスト（WP-SAFE-04 §7。J-1 の帰結:
  # jog_gate が黙っている間に /cmd_vel_behavior=priority20 が /cmd_vel_muxed に出る）
  # TIMEOUT: 3 本のテストがそれぞれ discovery 待ち 1s ＋ stale 待ち 2.2s 等を
  # 含むため 60 では足りない（実測で 40s 前後）。
  ament_add_pytest_test(jog_gate_node
    test/test_jog_gate_node.py
    TIMEOUT 150)
  # esp32_bridge ノード統合テスト (WP-SAFE-02。故障注入12「/cmd_vel の途絶」)
  ament_add_pytest_test(esp32_bridge_node
    test/test_esp32_bridge_node.py
    TIMEOUT 90)

  # ── 故障注入 13 項目 + 対照ケース (WP-TEST-01。fault_injection/) ──
  # DetailedDesign-safety.md §10 の 13 行が仕様。ファイル名は意図的に test_ で
  # 始めていない (test/fault_injection/conftest.py 冒頭の docstring 参照。ホストの
  # 通常 pytest 実行 (`pytest src/th_testing/test/`) に収集されないようにするための
  # 分離)。glob ではなく1本ずつ明示列挙する — ファイルを消しても colcon test が
  # 静かにスルーしないようにするため (test_cmake_test_registration.py と同じ考え方)。
  # 実機専用の 04・08・10 以外はこのパケットではプレースホルダ (中身は次パケット)。
  #
  # 01・02・11・control は Gazebo を実際に起動するため TIMEOUT 240
  # （既定の 60 では足りない。実測: gazebo.launch.py sim:=true の起動完了に
  # 40〜45 秒。内訳の見積り: sim_stack の準備完了ポーリング上限 90 秒
  # （実測値に安全側の余裕を載せた打ち切り値。合否のしきい値ではない）＋
  # ウォームアップ数秒 ＋ 停止観測窓 30 秒（_STOP_WINDOW_MS）＋ 故障注入11の
  # 追加アサーション類 約10秒 ＋ 後始末（SIGTERM の猶予。最大 30 秒＋
  # 最終手段のSIGKILL猶予10秒）。積み上げた粗い上限が約170秒なので、
  # 環境負荷のばらつきを見込んで240秒とした）。
  ament_add_pytest_test(fault_injection_01
    test/fault_injection/case_01_obstacle_limiter_forward.py
    TIMEOUT 240)
  ament_add_pytest_test(fault_injection_02
    test/fault_injection/case_02_obstacle_limiter_backward.py
    TIMEOUT 240)
  # 03 は 01/02 と同じく Gazebo を実際に起動して走行させるため TIMEOUT 240
  # (積算根拠は01/02と同じ。上のコメント参照)。
  ament_add_pytest_test(fault_injection_03
    test/fault_injection/case_03_estop_ui.py
    TIMEOUT 240)
  ament_add_pytest_test(fault_injection_04
    test/fault_injection/case_04_estop_physical.py
    TIMEOUT 60)
  # 05・06 は 01/02 と同じ Gazebo 起動コストに加え、`/scan` を止めるために
  # gzserver/gzclient を `_terminate_gazebo_processes()` で SIGINT→SIGTERM→
  # SIGKILL の順に段階的に終わらせる待ち（1段10秒×最大3段＝最悪30秒。通常は
  # SIGINT で数秒以内に終わる）が追加で乗る。01/02の見積り(約170秒)に
  # この最悪30秒と後始末での二重の gzserver 終了待ち分の余裕を足し、
  # 300秒とした（conftest.py の `cut_lidar_and_keep_clock_alive` 参照。
  # Docker 未検証のため実測に応じて見直しが要る可能性がある）。
  ament_add_pytest_test(fault_injection_05
    test/fault_injection/case_05_link_loss_fault.py
    TIMEOUT 300)
  ament_add_pytest_test(fault_injection_06
    test/fault_injection/case_06_fault_to_stop.py
    TIMEOUT 300)
  ament_add_pytest_test(fault_injection_07
    test/fault_injection/case_07_summon_retreat_wait.py
    TIMEOUT 60)
  ament_add_pytest_test(fault_injection_08
    test/fault_injection/case_08_esp32_watchdog.py
    TIMEOUT 60)
  ament_add_pytest_test(fault_injection_09
    test/fault_injection/case_09_person_detection_off.py
    TIMEOUT 60)
  ament_add_pytest_test(fault_injection_10
    test/fault_injection/case_10_physical_button_at_boot.py
    TIMEOUT 60)
  ament_add_pytest_test(fault_injection_11
    test/fault_injection/case_11_limiter_sigkill.py
    TIMEOUT 240)
  ament_add_pytest_test(fault_injection_12
    test/fault_injection/case_12_cmd_vel_stale.py
    TIMEOUT 60)
  ament_add_pytest_test(fault_injection_13
    test/fault_injection/case_13_localization_loss.py
    TIMEOUT 60)
  ament_add_pytest_test(fault_injection_control
    test/fault_injection/control.py
    TIMEOUT 240)

  # ── シミュレーション統合テスト (TH_SKIP_SIM=0 で有効化) ──
  ament_add_pytest_test(test_simulation
    test/test_simulation_scenarios.py
    TIMEOUT 120
    ENV TH_SKIP_SIM=1)   # CI ではデフォルトでスキップ
endif()

ament_package()
