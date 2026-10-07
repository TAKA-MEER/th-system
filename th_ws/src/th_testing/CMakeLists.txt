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
  ament_add_pytest_test(test_serial_framer
    test/test_serial_framer.py
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
  # 1b-7 SG-B18: 記録の連続性切れの純判定（pose_gap/jump_broken）と
  # しきい値宣言・配線の AST 検証。ROS2 不要
  ament_add_pytest_test(test_teach_record_broken
    test/test_teach_record_broken.py
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
  # install(PROGRAMS) に載せた th_onsite のノードが bringup.launch.py から
  # 実際に起動されることを固定する。home_declarer が「ビルドされるのに
  # 誰も起動しない」状態で実機に出た事故（2026-09-08）の再発防止
  ament_add_pytest_test(test_onsite_nodes_launched
    test/test_onsite_nodes_launched.py
    TIMEOUT 30)
  # モード→状態の表が state_core.py（Python）と safety_monitor_core.cpp（C++）で
  # ずれていないことを固定する。片方だけにモードを足すと、そのモードに入った瞬間に
  # STATE_INCONSISTENT(CRITICAL) → ESTOP になる（2026-09-08 の AT_HOME で実機発生）
  ament_add_pytest_test(test_mode_states_sync
    test/test_mode_states_sync.py
    TIMEOUT 30)
  ament_add_pytest_test(test_route_replay_core
    test/test_route_replay_core.py
    TIMEOUT 30)
  # 全域ローカライズの確度スコア s・m と粗→細探索（W-01 P1。th_planning/localize_core.py）。
  # numpy / scipy が要るのでホストの素の python3 では走らない（Docker で回す。
  # CLAUDE.md の除外一覧に載せてある）。合成地図の探索を含むので TIMEOUT は長め
  ament_add_pytest_test(test_localize_core
    test/test_localize_core.py
    TIMEOUT 300)
  # 始業点検（OPCHECK）の判定コア (WP-MAINT-01。th_maintenance/check_core.py。
  # 合否判定の純粋関数。registry.yaml の WP-MAINT-01 ブロックと値が一致することは
  # test_maintenance_registry_driven が別途検証する。ROS2 不要)
  ament_add_pytest_test(test_opcheck_core
    test/test_opcheck_core.py
    TIMEOUT 60)
  # 起動時の自動点検の判定コア (Spec-ops.md §2.6。th_maintenance/auto_check_core.py。
  # ESTOP/IMU/LIDAR の自動判定・総合・開発モードの抑制判定の純粋関数。ROS2 不要)
  ament_add_pytest_test(test_auto_check_core
    test/test_auto_check_core.py
    TIMEOUT 60)
  # 校正（CALIB）の計算・許容判定コアと履歴の永続化 (WP-MAINT-02。
  # th_maintenance/calib_core.py・calib_store.py。ROS2 不要)
  ament_add_pytest_test(test_calib_core
    test/test_calib_core.py
    TIMEOUT 60)
  ament_add_pytest_test(test_calib_store
    test/test_calib_store.py
    TIMEOUT 60)
  # 校正 BLIND（死角マスク）の純粋関数（th_maintenance/blind_core.py。ROS2 不要）
  ament_add_pytest_test(test_calib_blind_core
    test/test_calib_blind_core.py
    TIMEOUT 60)
  # 校正で確定した死角マスクの起動時上書き（params_generation。ROS2 不要）
  ament_add_pytest_test(test_params_blind_override
    test/test_params_blind_override.py
    TIMEOUT 120)
  # S-31 故障診断のチェック手順（repair_hints.yaml）と WebUI 用 json の一致
  # （WP-MAINT-03。ROS2 不要）
  ament_add_pytest_test(test_repair_hints_sync
    test/test_repair_hints_sync.py
    TIMEOUT 60)
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
  # esp32_bridge の wheel_radius_scale（車輪半径スケール）純粋コア
  # (WP-ESP32-02。O-d10: 指令 /k・実測 ×k・A10 判定。A10 は
  # th_params.assertions を再利用。ROS2 不要)
  ament_add_pytest_test(test_wheel_scale_core
    test/test_wheel_scale_core.py
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
  # 自己位置推定の健全性コア＋配線 (WP-SAFE-05。純粋コア・registry・生成 yaml・
  # launch AST・msg 定義。ROS2 不要)
  ament_add_pytest_test(test_localization_health_core
    test/test_localization_health_core.py
    TIMEOUT 60)
  ament_add_pytest_test(test_localization_health_launch
    test/test_localization_health_launch.py
    TIMEOUT 60)
  # B′ 閾値の実走回帰 (2026-09-21。registry 駆動・ROS2 不要。
  # 旧閾値に戻すと発火しない側が赤くなる)
  ament_add_pytest_test(test_jump_thresholds
    test/test_jump_thresholds.py
    TIMEOUT 60)
  # 開発モードの土台 (WP-DEV-01A。dev 分岐・gate 真理値表・launch AST。
  # th_state の純粋コア＋ソース検査。ROS2 不要)
  ament_add_pytest_test(test_dev_mode
    test/test_dev_mode.py
    TIMEOUT 60)
  # 開発モードのログ選択記録 (WP-DEV-01C。実効選択・state/fault 変化・
  # cmdvel 間引き・JSON 書式＋ノード配線の検査。ROS2 不要)
  ament_add_pytest_test(test_dev_log_core
    test/test_dev_log_core.py
    TIMEOUT 60)
  # ゾーン合成コア (WP-STATE-02。derive_limits() の速度上限が §4.1 の規則
  # 「合成は各画面の最小値。ゾーンの強弱では合成しない」を満たすことの担保。N-13)
  ament_add_pytest_test(test_zones
    test/test_zones.py
    TIMEOUT 60)
  # 2点指示コア (WP-ONSITE-01。方位・点間距離・対象妥当性・base_link→map 合成。
  # th_onsite の純ロジック。ROS2 不要)
  ament_add_pytest_test(test_two_point_core
    test/test_two_point_core.py
    TIMEOUT 30)

  # 退避待ちゲートコア (WP-ONSITE-03。hold 蓄積・タイムアウト・二重発火ガード。
  # th_onsite の純ロジック。ROS2 不要)
  ament_add_pytest_test(test_wait_clear_core
    test/test_wait_clear_core.py
    TIMEOUT 30)

  # venue_navigator コア (WP-ONSITE-02。旋回誤差・旋回指令クランプ・到着判定。
  # th_onsite の純ロジック。ROS2 不要)
  ament_add_pytest_test(test_venue_nav_core
    test/test_venue_nav_core.py
    TIMEOUT 30)

  # ピン壁近接チェックコア (WS-9AB。costmap から最寄り占有セル距離・退避先。
  # th_onsite の純ロジック。ROS2 不要)
  ament_add_pytest_test(test_pin_clearance_core
    test/test_pin_clearance_core.py
    TIMEOUT 30)

  # 待機場所宣言コア (Spec-onsite §4.0。姿勢オフセット・許容差判定。
  # th_onsite の純ロジック。ROS2 不要)
  ament_add_pytest_test(test_home_declare_core
    test/test_home_declare_core.py
    TIMEOUT 30)

  # 機体姿勢ピン登録コア (REG-2。HOME 置換 / PANEL 連番の pin dict 構築。
  # th_onsite の純ロジック。ROS2 不要)
  ament_add_pytest_test(test_robot_pose_register_core
    test/test_robot_pose_register_core.py
    TIMEOUT 30)

  # onsite コンテキスト派生値 (WP-ONSITE-F2。/person/targets・/onsite/pins →
  # Context の candidate_count / target_selected / target_confident / pin_kinds。
  # th_state の純ロジック。ROS2 不要)
  ament_add_pytest_test(test_onsite_context
    test/test_onsite_context.py
    TIMEOUT 30)

  # tracker_enabled の純ロジック (brief-tracker-default-off §3.1。Spec-modes.md
  # §9「動かさない」自動停止集合・§5.1 要 person（SUMMON 全状態+PREP/REGISTER）の
  # OFF 拒否。th_state の純ロジック。ROS2 不要)
  ament_add_pytest_test(test_tracker_policy
    test/test_tracker_policy.py
    TIMEOUT 30)

  # DR-SPAAM 起動/停止判断の純ロジック (brief-tracker-default-off §3.2。
  # tracker_enabled と lifecycle 状態からの transition_id 決定。ROS2 不要)
  ament_add_pytest_test(test_dr_spaam_lifecycle_controller_core
    test/test_dr_spaam_lifecycle_controller_core.py
    TIMEOUT 30)

  # DR-SPAAM 起動/停止アクチュエータの結合テスト (brief-tracker-default-off §3.2。
  # /system/state の tracker_enabled を見て DR-SPAAM の lifecycle サービスへ
  # activate/deactivate を飛ばす本番の経路。launch_testing。rclpy 要）
  ament_add_pytest_test(test_dr_spaam_lifecycle_controller_node
    test/test_dr_spaam_lifecycle_controller_node.py
    TIMEOUT 120)

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
  # SG-A14 (1b-10): 駆動系の天井と v_max の導出・生成物の一致（ROS2 不要）。
  # §1 が A13 ブロック中のため 2 件は赤のまま（報告参照）。登録自体は正規の手順。
  ament_add_pytest_test(test_drivetrain_ceiling
    test/test_drivetrain_ceiling.py
    TIMEOUT 60)
  ament_add_pytest_test(test_params_registry
    test/test_params_registry.py
    TIMEOUT 60)
  ament_add_pytest_test(test_params_launch
    test/test_params_launch.py
    TIMEOUT 60)
  # W-03: 教示・再生・間引きの registry 駆動（値一致・生成物・launch 配線。
  # ROS2 不要）
  ament_add_pytest_test(test_w03_registry_driven
    test/test_w03_registry_driven.py
    TIMEOUT 120)
  # W-13: 試験場内・人物追跡の registry 駆動（値一致・生成物・launch 配線。
  # ROS2 不要）
  ament_add_pytest_test(test_w13_registry_driven
    test/test_w13_registry_driven.py
    TIMEOUT 120)
  # W-06 の②: DRIVE_RUNAWAY の鮮度ゲート（値一致・生成物・整合・配線。
  # ROS2 不要）
  ament_add_pytest_test(test_w06_runaway_freshness
    test/test_w06_runaway_freshness.py
    TIMEOUT 120)
  ament_add_pytest_test(test_prelaunch_guard
    test/test_prelaunch_guard.py
    TIMEOUT 60)
  # OPCHECK の registry 駆動 (WP-MAINT-01。registry.yaml の行 ↔ check_core の
  # CheckParams ↔ 生成物 opcheck_runner.yaml の 3 点一致と、placeholder 巻き込み
  # 防止。ROS2 不要)
  ament_add_pytest_test(test_maintenance_registry_driven
    test/test_maintenance_registry_driven.py
    TIMEOUT 120)
  # 保守系ノード（opcheck_runner / opcheck_auto / calib_runner）の宣言型が生成 yaml の
  # 値の型と一致すること（rclpy は INTEGER↔DOUBLE を変換せず起動時に落ちる。
  # 2026-10-02 実機で opcheck_runner が起動しなかった。ROS2 不要)
  ament_add_pytest_test(test_maintenance_param_types
    test/test_maintenance_param_types.py
    TIMEOUT 120)
  # SG-B21（1b-10）: consumers に挙がっていたのに読み手がいない行の除去と、
  # 点検・校正の直書きタイムアウトの registry 化（値・型・生成物の 3 点一致と、
  # 外した consumers の否定試験。ROS2 不要）
  ament_add_pytest_test(test_sgb21_registry_wiring
    test/test_sgb21_registry_wiring.py
    TIMEOUT 120)
  # W-01 P5: 全域ローカライズ 6 値の registry 化（registry ↔ ノード既定値 ↔
  # 生成物の 3 点一致、placeholder の載らなさ、stage 5 の A8 armed、宣言型の
  # 一致。ROS2 不要）
  ament_add_pytest_test(test_w01p5_localize_registry
    test/test_w01p5_localize_registry.py
    TIMEOUT 120)
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
  # 1b-2（SG-A5）: INIT からの非常停止の往復を state_manager と connectivity_checker
  # の同時起動で縛る（C-09b-init → T-INIT-01。link_ok 再送の境界を含む）
  ament_add_pytest_test(test_estop_init_link_resume_node
    test/test_estop_init_link_resume_node.py
    TIMEOUT 120)
  # 1b-2（SG-A5 の境界）: ESTOP に入る前に gate が真でも、INIT に戻ったあと
  # link_ok が再送されて IDLE まで進む（別 launch。同一 launch では起動直後の
  # 前提が崩れるため分ける）
  ament_add_pytest_test(test_estop_init_link_reemit_node
    test/test_estop_init_link_reemit_node.py
    TIMEOUT 120)
  # opcheck_runner ノードの振る舞い試験（WP-MAINT-01 §4-2。ノードを実際に起動し、
  # /system/state を TRANSIENT_LOCAL で流して OPCHECK 限定動作・MOTOR デッドマン・
  # 符号逆 NG・OPCHECK 離脱時の即停止を確かめる。state_manager は起動しない）
  ament_add_pytest_test(test_opcheck_runner_node
    test/test_opcheck_runner_node.py
    TIMEOUT 120)
  # opcheck_auto ノードの振る舞い試験（起動時の自動点検。ノードを実際に起動し、
  # 正常時 OK・/scan 未受信 NG・IMU 未受信 NG・押下 NG・開発モードの抑制と
  # 鮮度・/cmd_vel_behavior に非ゼロが出ないことを確かめる）
  ament_add_pytest_test(test_opcheck_auto_node
    test/test_opcheck_auto_node.py
    TIMEOUT 180)
  # calib_runner ノードの振る舞い試験（WP-MAINT-02。FSM 無しで単体の安全を縛る:
  # CALIB 以外は動かない・CALIB を抜けたら止まる・/system/state や /odom の途絶・
  # 検証に合格していない値を確定しない・IMU）
  ament_add_pytest_test(test_calib_runner_node
    test/test_calib_runner_node.py
    TIMEOUT 180)
  # 校正を本番の経路で通す試験（WP-MAINT-02。state_manager と calib_runner を同時起動し、
  # 代役の esp32_bridge と物理モデルで 直進→実測→プレビュー→適用→検証→確定・検証NGで戻る・
  # A10・中断・フォルト/非常停止・ロールバックを通す）
  ament_add_pytest_test(test_calib_wiring_node
    test/test_calib_wiring_node.py
    TIMEOUT 300)
  # 校正 BLIND（死角マスク）を本物の 5 ノードで通す試験（state_manager・calib_runner・
  # lidar_filter・obstacle_limiter・opcheck_runner。3 ノードへの同時反映・上限・適用前へ戻す）
  ament_add_pytest_test(test_calib_blind_node
    test/test_calib_blind_node.py
    TIMEOUT 600)
  # obstacle_limiter の起動時 blind_angle_ranges が上限違反ならマスクなしで起動する（観測できる振る舞い）
  ament_add_pytest_test(test_blind_startup_limit_node
    test/test_blind_startup_limit_node.py
    TIMEOUT 180)
  ament_add_pytest_test(connectivity_checker_node
    test/test_connectivity_checker_node.py
    TIMEOUT 120)
  ament_add_pytest_test(test_params_audit_node
    test/test_params_audit_node.py
    TIMEOUT 120)
  # SG-B8: overrides → 生成 → 生成 yaml → ノード起動の本番経路
  # （launch_testing。ROS2 要）
  ament_add_pytest_test(test_params_overrides_node
    test/test_params_overrides_node.py
    TIMEOUT 120)
  ament_add_pytest_test(test_safety_monitor
    test/test_safety_monitor.py
    TIMEOUT 90)
  # SG-A8 フォルト集約の Docker launch テスト（1b-4。
  # 2 重フォルトで 1 つ解消しても active=true（残りの代表）・全解消で active=false、
  # state_manager 経由で「はい」が残フォルト中は拒否されること。rclpy 要＝ホスト除外）
  ament_add_pytest_test(test_fault_aggregate_node
    test/test_fault_aggregate_node.py
    TIMEOUT 300)
  # LOCALIZATION_LOST の Docker launch テスト（WP-SAFE-05 §完了条件1〜3。
  # 継続で発火・単発で発火せず・途絶で発火。CRITICAL の確認を含む）
  ament_add_pytest_test(test_localization_lost
    test/test_localization_lost.py
    TIMEOUT 180)
  # DRIVE_RUNAWAY の鮮度ゲートの Docker launch テスト（W-06 の② §完了条件。
  # 未受信で発火せず・新鮮な乖離で発火・発火後の途絶で解除されず。
  # CRITICAL の確認を含む）
  ament_add_pytest_test(test_runaway_freshness_node
    test/test_runaway_freshness_node.py
    TIMEOUT 180)
  # S-14「経路からのずれ」の Docker launch テスト（台帳 W-02 の一部。
  # 横にずらした odom で /route/status のずれが増え、load_route で 0 に戻る。
  # rclpy 要＝ホスト除外。TIMEOUT 180）
  ament_add_pytest_test(test_replay_cross_track_node
    test/test_replay_cross_track_node.py
    TIMEOUT 180)
  # 全域ローカライズの探索の実体（W-01 P2）の Docker launch テスト。
  # replay_runner を起動し、代役サービス・/scan・既知 pgm で
  # (a) 正位置で done＋探索姿勢 (b) 不一致で low (c) global 不成立で done 無し
  # を見る。rclpy 要＝ホスト除外。TIMEOUT 180）
  ament_add_pytest_test(test_replay_localize_node
    test/test_replay_localize_node.py
    TIMEOUT 180)
  # 経路の途中からの復帰（W-01 P4）の Docker launch テスト。
  # replay_runner を起動し、確定姿勢→再開 index→向き合わせ→RUN／遠すぎたら
  # 留まる・逆再生でも 1 本を見る。rclpy 要＝ホスト除外。TIMEOUT 300）
  ament_add_pytest_test(test_replay_from_index_node
    test/test_replay_from_index_node.py
    TIMEOUT 300)
  # 開発モードの項目（Spec-safety.md §10。2026-09-23 改定）の Docker launch テスト。
  # LiDAR 無しで scan_stop（MANUAL だけ v_reverse 以下で通す・AUTO は止める・
  # effective だけを読む・鮮度切れで戻る）と lidar_fault（LIDAR_LOST の抑止と再発）。
  ament_add_pytest_test(test_dev_mode_safety_node
    test/test_dev_mode_safety_node.py
    TIMEOUT 180)
  # 自動ブレーキの正本（1b-15 SG-B10）。state_manager を起動して、場外 OFF・場内 ON・
  # 切替要求・入り直しで既定へ戻る・自律系で拒否・画面途絶で ON を縛る。rclpy 要＝ホスト除外。
  ament_add_pytest_test(test_auto_brake_state_node
    test/test_auto_brake_state_node.py
    TIMEOUT 180)
  # 手動系の自動ブレーキ ON/OFF を obstacle_limiter 本体で縛る（1b-15 SG-B10。ON は減速・
  # OFF は減速せず接近警告・state 途絶で ON・estop／fault_lock／AUTO／NA は OFF でも止める）。
  ament_add_pytest_test(test_auto_brake_limiter_node
    test/test_auto_brake_limiter_node.py
    TIMEOUT 180)
  # PERSON_TRACKER_LOST のゲート・猶予・記録だけモードの Docker launch テスト
  # （DetailedDesign-safety.md §5.5 P-01＋P-02。
  # OFFでは出ない・ONで止めると出る（report_only=false）・
  # report_only=trueでは出ない・OFFで解除される）
  ament_add_pytest_test(test_person_tracker_lost_gate_node
    test/test_person_tracker_lost_gate_node.py
    TIMEOUT 180)
  # 計画的な再起動の保留の Docker launch テスト（O-e3 §完了条件3。
  # 知らせ無しで検知・通知中は保留・上限超過で故障・publisher 死亡でも超過で故障）
  ament_add_pytest_test(test_planned_restart_node
    test/test_planned_restart_node.py
    TIMEOUT 180)
  # 全域ローカライズの探索中の保留の Docker launch テスト（W-01 P5 §完了条件。
  # 知らせ無しで検知・探索通知中は global_localizing で保留・上限超過で故障・
  # 保留中は階段でも jump 無し・知らせ無しなら jump が出る対照）
  ament_add_pytest_test(test_explore_hold_node
    test/test_explore_hold_node.py
    TIMEOUT 180)
  # 全域ローカライズの数値が registry→生成 yaml→ノードへ届くことの Docker launch
  # テスト（W-01 P5 §完了条件。生成 yaml を先に渡して起動し、dict 上書きしない。
  # 6 値の読み戻し＋resume 上限が振る舞いに効くこと）
  ament_add_pytest_test(test_replay_localize_registry_node
    test/test_replay_localize_registry_node.py
    TIMEOUT 300)
  # 人物検出 OFF ゲートの Docker launch テスト（brief-tracker-default-off §3.3。
  # /system/state.tracker_enabled=false 中は /person/targets・/person/status が
  # is_lost/disabled を強制し、evt.target_lost と select サービス呼び出しを抑止
  # する本番の経路。変異②の標的）
  ament_add_pytest_test(test_person_tracker_bridge_node
    test/test_person_tracker_bridge_node.py
    TIMEOUT 180)
  # 退避待ちゲートの disabled 結合テスト（brief-tracker-default-off §3.3。
  # /person/status が is_lost=True / lost_reason="disabled" の間 clear_ok を
  # 出さない。見えている人が十分離れると発火する対照を含む）
  ament_add_pytest_test(test_wait_clear_gate_node
    test/test_wait_clear_gate_node.py
    TIMEOUT 180)
  # mode gate の Docker launch テスト（WP-SAFE-05修正 §2。
  # IDLE で inactive（生値のまま）・PANEL_NAV で1秒以内に ng・
  # PREP/MAPPING で inactive・PREP/RETURN で ng）
  ament_add_pytest_test(test_localization_mode_gate_node
    test/test_localization_mode_gate_node.py
    TIMEOUT 180)
  # slam_control の再起動通知のインプロセス振る舞い試験（O-e3。
  # 読み直し・破棄・例外・クラッシュ。rclpy 要＝ホスト除外。TIMEOUT 60）
  ament_add_pytest_test(test_slam_control_restart
    test/test_slam_control_restart.py
    TIMEOUT 60)
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
  # esp32_bridge の wheel_radius_scale 両方向適用 + A10 の強制 (WP-ESP32-02)
  # ノード統合テスト。偽 ESP32 で指令 /k・実測 ×k・起動時拒否・実行時拒否を確認
  ament_add_pytest_test(esp32_scale
    test/test_esp32_scale.py
    TIMEOUT 120)

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
  # 07 は Gazebo を立てず `state_manager` / `pin_registrar` /
  # `wait_clear_gate` / `jog_gate` を launch_testing で直接起こすだけなので
  # 起動コストは数秒。積算は 退避待ちの観測窓（`clear_timeout_ms` の
  # launch 上書き 6000ms ＋ 余裕 3 秒 ≒ 9 秒）＋ ジョグ遮断/陽性対照 各 10 秒
  # ＋ 退避の陽性対照（`clear_hold_ms` 1500ms 待ち）≒ 10 秒 ＋ 各 test の
  # 手順 1〜4（サービス待ちと遷移待ち 各 5 秒上限）≒ 40 秒 ＋ teardown
  # （rclpy.shutdown と launch_testing の SIGTERM 猶予 最大30秒）を見込み、
  # 環境負荷のばらつき分を加えた 240 秒とした（01/02/03 と同じ枠）。
  ament_add_pytest_test(fault_injection_07
    test/fault_injection/case_07_summon_retreat_wait.py
    TIMEOUT 240)
  ament_add_pytest_test(fault_injection_08
    test/fault_injection/case_08_esp32_watchdog.py
    TIMEOUT 60)
  ament_add_pytest_test(fault_injection_09
    test/fault_injection/case_09_person_detection_off.py
    TIMEOUT 240)
  ament_add_pytest_test(fault_injection_10
    test/fault_injection/case_10_physical_button_at_boot.py
    TIMEOUT 60)
  ament_add_pytest_test(fault_injection_11
    test/fault_injection/case_11_limiter_sigkill.py
    TIMEOUT 240)
  ament_add_pytest_test(fault_injection_12
    test/fault_injection/case_12_cmd_vel_stale.py
    TIMEOUT 60)
  # 13 は Gazebo を**2回**起動する（本体: REPLAY で SIGKILL→LOCALIZATION_LOST→
  # ESTOP の検証に加え、IDLE 対照: 監視が効かないことの検証も別起動で行う）。
  # 01/02 の1起動あたりの積算見積り（約170秒。上のコメント参照）を2本ぶん
  # 積むと約340秒。速度低下・後始末（gzserver の SIGINT→SIGTERM→SIGKILL の
  # 段階的終了待ち）の余裕を見て480秒とした。
  ament_add_pytest_test(fault_injection_13
    test/fault_injection/case_13_localization_loss.py
    TIMEOUT 480)
  ament_add_pytest_test(fault_injection_control
    test/fault_injection/control.py
    TIMEOUT 240)

  # ── シミュレーション統合テスト (TH_SKIP_SIM=0 で有効化) ──
  ament_add_pytest_test(test_simulation
    test/test_simulation_scenarios.py
    TIMEOUT 120
    ENV TH_SKIP_SIM=1)   # CI ではデフォルトでスキップ

  # 1b-7 SG-B1: 教示中の非常停止 → 解除 → 戻る、で「記録中」と実際の記録が
  # 一致すること（state_manager＋route_recorder の本番の経路。rclpy 要＝ホスト除外）
  ament_add_pytest_test(test_teach_estop_resume_node
    test/test_teach_estop_resume_node.py
    TIMEOUT 120)
  # 1b-7 SG-B18: 記録の連続性切れで evt.record_broken が出ること
  # （route_recorder 単体の本番の経路。rclpy 要＝ホスト除外）
  ament_add_pytest_test(test_teach_record_broken_node
    test/test_teach_record_broken_node.py
    TIMEOUT 120)
  # 盤前移動の一時停止→再開（#3）・BLOCKED タイムアウトなし（#4）
  # (DetailedDesign-onsite.md §9。Gazebo・Nav2 本体なしの launch_testing。
  # 積算: 起動 discovery 10 秒 ＋ 既存 7 件 ＋ SG-A1/A15 の 6 件追加（各 10〜15 秒）
  # ＋ teardown 余裕。fault_injection_07 と同じ 240 秒枠では足りないので 400 秒)
  ament_add_pytest_test(test_venue_nav_pause_blocked_node
    test/test_venue_nav_pause_blocked_node.py
    TIMEOUT 400)

  # 待機場所宣言の配線（DetailedDesign-onsite.md §9 #10。Gazebo なしの
  # launch_testing で pin_registrar＋home_declarer を起動し、TF をずらして
  # 照合・force・ピン変更時の解除・境界を検証。積算は venue_nav と同じ 240 秒枠）
  ament_add_pytest_test(test_home_declarer_node
    test/test_home_declarer_node.py
    TIMEOUT 240)
endif()

ament_package()
