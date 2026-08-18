# 既存資産の去就

[DetailedDesign.md](DetailedDesign.md) の詳細。**`th_ws/` の全ファイルに扱いを 1 行で与える。**

> **`DD-4`**: 「ほとんど作り直す」と言われた実装者は、迷えば作り直す。
> **捨ててはいけないもの（実機で潰した知見が入っている設定値）を名指しで守る。**

| 記号 | 意味 |
| --- | --- |
| **維持** | そのまま使う。触らない |
| **抽出** | 純粋コアとして取り出し、新しいノードから呼ぶ |
| **改修** | ファイルは残すが中身を変える |
| **土台** | 考え方と構造を引き継いで書き直す |
| **廃止** | 削除する |

---

## 1. 絶対に捨ててはいけないもの

**実機で問題を踏んで直した結果**が入っている。値だけ見ると「適当な数字」に見えるので、
**根拠のコメントごと**引き継ぐ。

| 対象 | なぜ捨ててはいけないか |
| --- | --- |
| `th_safety/config/twist_mux.yaml` の優先度とタイムアウト | `manual_joy.timeout: 1.0` は WiFi 受信ギャップ（0.5〜1.2 s）を踏まえて 0.5 → 1.0 に緩めた実績値。ロック 255/254 の設計も同様 |
| `esp32_bridge` の **独立ロック層**（`/safety/estop` `/safety/fault_lock` を直接購読・0.5 s の stale もロック扱い） | twist_mux がロック中に無出力になる実装であることが 2026-08-06 に判明して入れた層。**`obstacle_limiter` と重複ではない**（[safety](DetailedDesign-safety.md) §1.1） |
| `esp32_bridge` の **20 Hz キープアライブ** | WiFi ジッタでウォッチドッグが誤発動するのを防ぐ。**ただし `DEBT-4` の対処（stale タイムアウト）を同時に入れる** |
| `WHEEL_FEEDBACK` の `dt_sec`（13 byte 版）と 9 byte 旧形式の受理 | 到着時刻から積分区間を推測すると WiFi 遅延がそのまま yaw ドリフトになる。旧形式受理は「再書き込みは AP が落ちる作業」への配慮 |
| `ekf_params.yaml` の **`imu0` は `vyaw` のみ融合** | BNO055 の NDOF 絶対方位を `world_frame: odom` に入れると屋内の磁気擾乱で odom の連続性が壊れる |
| **`odom → base_link` の TF は `ekf_filter_node` だけが出す**（`esp32_bridge` は `publish_tf: false`） | 二重親で TF ツリーが壊れる |
| `leg_tracker_param.yaml` の **`require_explicit_target_selection: true`** | 机・椅子の脚へ乗り移る誤追跡（2026-07-11 実機で確認）の再発防止 |
| `PersonTracker` の **自機回転補償**（`compensateEgoMotion`） | DR-SPAAM が約 2 Hz なので、旋回中は見かけ位置が飛んで対象を取り落とす。**ゲート半径を広げて対処してはいけない** |
| `slam_control.py` の **「停止＝localization モードへ切替」** | `pause_new_measurements` は `map→odom` を凍結させる実害があり 2026-08-07 に廃止 |
| `mapGeometry.js` の等方スケール処理 | 地図が伸びるバグを直した結果 |
| `useRosbridge.js` の **TF 手組み** | `ROSLIB.TFClient` は `tf2_web_republisher` を要求するが、このリポジトリには無い |
| `Dockerfile` の DR-SPAAM `map_location` パッチ | 配布チェックポイントが CUDA 保存のため CPU 機で `torch.load` が落ちる |
| `Dockerfile` が `--symlink-install` を**使わない**こと | `./src` を bind mount で上書きするため、イメージ内 `src/` へのシンボリックリンクが dangling になる |

---

## 2. パッケージ別

### 2.1 `th_system_msgs`

| ファイル | 扱い | 備考 |
| --- | --- | --- |
| `RobotMode.msg` | **廃止** | 18 モードは文字列で持つ（[names](DetailedDesign-names.md) §2.1） |
| `PersonStatus.msg` | **廃止** | `PersonTargets.msg` へ統合 |
| `FollowStatus.msg` / `SearchStatus.msg` / `SummonStatus.msg` | **廃止** | `SystemState` ＋各機能の status へ |
| `PanelArrival.msg` | **廃止** | `evt.arrived` へ |
| `WheelCommand.msg` | **廃止** | 未使用 |
| `FaultStatus.msg` | **改修** | `severity` を追加 |
| `WheelFeedback.msg` | **維持** | 指令側にも再利用する流儀も維持 |
| `SetMode.srv` / `GoToPanel.srv` / `CompleteInspection.srv` / `SetTunableParams.srv` / `SaveTunableParams.srv` | **廃止** | 置き換え先は [names](DetailedDesign-names.md) §5.2 |
| （新設） | — | `SystemState` / `StateEvent` / `ActiveScreen` / `LimiterStatus` / `LinkQuality` / `ParamsStatus` / `PersonTargets` / `Pin` / `PinList` / `RouteInfo` / `RouteList` / `RouteStatus` / `MapSessionStatus` / `WaitClearStatus` / `CheckStatus` / `CalibStatus` ＋ srv 群 |

### 2.2 `th_mode_manager` → **パッケージごと廃止**

| ファイル | 扱い | 移設先 |
| --- | --- | --- |
| `src/mode_manager.cpp`（242 行） | **廃止** | `th_state`。遷移が `switch` で 18 モードに拡張不能。サブステートの概念が無い |

### 2.3 `th_safety`

| ファイル | 扱い | 備考 |
| --- | --- | --- |
| `src/safety_monitor.cpp`（236 行） | **改修** | `severity` 追加／監視対象に limiter と map_session を追加／`/person/targets` へ差し替え／タイムアウトを生成値から読む／リンク品質を publish |
| `config/twist_mux.yaml` | **改修（生成対象に）** | 値は維持。**`registry.yaml` から生成する**ようにする。ロック優先度・タイムアウトの根拠コメントは残す |
| `config/safety_monitor.yaml` | **廃止（生成へ）** | 値の根拠コメントは `registry.yaml` の `note` へ移す |
| （新設）`src/obstacle_limiter.cpp` ＋ `include/obstacle_limiter_core.hpp` | — | [safety](DetailedDesign-safety.md) §3 |
| （新設）`src/jog_gate.cpp` | — | 同 §3.4.1 |

### 2.4 `th_planning` → **パッケージごと廃止**（`th_transit` / `th_route` へ）

| ファイル | 扱い | 移設先 |
| --- | --- | --- |
| `th_planning/follow_planner_core.py`（422 行・28 テスト） | **抽出（一部）** | 幾何関数のみ `th_transit/geometry.py` へ: `to_absolute` / `to_relative` / `update_trail` / `get_trail_goal` / `pure_pursuit_control`。**`FollowPlannerCore` 本体・`PREPARE`／`EVADING`・`find_nearest_open_direction` / `compute_evade_goal` は廃止**（`Spec-transit.md` §1.2「近づいたら停止する」と食い違う） |
| `th_planning/mapless_follow_core.py`（301 行・27 テスト） | **抽出（主体）** | `th_transit/follow_core.py` の土台。`next_mapless_state` / `should_stop_for_lost` / `is_path_blocked` / `mapless_target_speed` / `rate_limit` をそのまま |
| `th_planning/summon_navigator_core.py`（51 行） | **抽出** | `th_onsite`。`compute_stop_goal` は 2 点指示に置き換わるので `is_target_valid` のみ |
| `scripts/follow_planner.py`（334 行） | **廃止** | Nav2 ゴール方式。新設計は速度直接 |
| `scripts/follow_planner_mapless.py`（316 行） | **土台** | `th_transit/scripts/follow_runner.py`。`add_on_set_parameters_callback` と `_POSITIVE_PARAMS` ガードの流儀は引き継ぐ |
| `scripts/summon_navigator.py`（220 行） | **土台** | `th_onsite/scripts/venue_navigator.py` に統合 |
| `scripts/panel_navigator.py`（192 行） | **土台** | 同上。**`panels.yaml` の静的読み込みは廃止**し、ランタイム登録へ |
| `scripts/manual_command_handler.py`（154 行） | **廃止** | heartbeat は `ui.jog.hold` のリースへ。`/manual/target_pose` は消費者なし |
| `scripts/crawler_teleop.py`（206 行） | **維持** | 開発用のキーボードテレオペ。`th_transit` へ移すだけ |

### 2.5 `th_perception`

| ファイル | 扱い | 備考 |
| --- | --- | --- |
| `scripts/lidar_filter.py`（98 行） | **維持** | `blind_angle_ranges` のライブ更新も維持 |
| `scripts/person_tracker_bridge.py` ＋ `_core.py` | **改修** | 出力を `/person/targets`（`PersonTargets`）に。候補一覧と `evt.auto_selected` を足す |
| `scripts/person_predictor.py`（203 行） | **廃止** | 捜索旋回（`/cmd_vel_retreat`）は新設計に無い |
| `scripts/person_tracker_stub.py`（104 行） | **維持** | シミュレーション用 |
| `scripts/gazebo_person_relay.py`（333 行） / `person_mover.py` / `obstacle_mover.py` | **維持** | シミュレーション専用 |

### 2.6 `th_esp32_bridge`

| ファイル | 扱い | 備考 |
| --- | --- | --- |
| `th_esp32_bridge/ws_protocol.py`（130 行） | **維持** | ファームと byte 互換。**`ws_link.h` と同時にしか変えない** |
| `scripts/esp32_bridge.py`（505 行） | **改修＋抽出** | ①オドメトリ積分・スタンプ再同期を `odom_core.py` へ抽出 ②**`/cmd_vel` の stale タイムアウトを追加**（`DEBT-4`） ③`wheel_radius_scale` の双方向適用（`O-d10`） ④ロック層とキープアライブは**維持** |
| `config/params.yaml` | **改修（生成へ）** | `publish_tf: false` は**絶対に変えない** |

### 2.7 `th_config_manager` → **パッケージごと廃止**

| ファイル | 扱い | 移設先 |
| --- | --- | --- |
| `scripts/config_manager.py`（199 行） | **土台** | `th_params/scripts/params_audit.py`。`MultiThreadedExecutor` ＋ `ReentrantCallbackGroup` の必要性（`SingleThreadedExecutor` だと rosbridge ごと停まる）は**引き継ぐ** |
| `th_config_manager/yaml_writer.py`（56 行） | **抽出** | `th_params`。コメント保持の round-trip |
| `th_config_manager/tunable_targets.py` | **廃止** | `registry.yaml` の `consumers` が代替 |
| `th_config_manager/service_call.py` | **抽出** | `th_params` |
| `scripts/slam_control.py`（489 行） | **土台** | `th_route/scripts/map_session.py`。**`_map_basename()` のハードコードを引数化する**のが主な改修 |

### 2.8 `th_calibration` → **パッケージごと廃止**（`th_maintenance` へ）

| ファイル | 扱い | 移設先 |
| --- | --- | --- |
| `scripts/linear_calib.py`（209 行） | **抽出** | 計算部を `calib_core.corrected_wheel_radius()` へ。**publish 先は `/cmd_vel_behavior`**（`jog_gate` が `CALIB` を塞ぐため `/cmd_vel_manual` は使えない）。中断は `/system/state` の購読で行う（`/robot/mode` は廃止） |
| `scripts/rotation_calib.py`（194 行） | **抽出** | `calib_core.corrected_wheel_base()` |
| `scripts/apply_calib.py`（105 行） | **廃止** | `/calib/apply` サービスへ |
| `scripts/imu_calib_check.py`（84 行） | **土台** | `calib_runner` の IMU 項目 |

### 2.9 `th_bringup`

| ファイル | 扱い | 備考 |
| --- | --- | --- |
| `launch/bringup.launch.py`（390 行） | **改修** | ノード構成を全面的に入れ替え。**`remappings=[('cmd_vel_out','/cmd_vel')]` を `/cmd_vel_muxed` に**（`DEBT-4` の対処と同一パケット）。パラメータ生成の `OpaqueFunction` を先頭に追加 |
| `launch/gazebo.launch.py`（606 行） | **改修** | 同上。シナリオプリセットの仕組みは**維持** |
| `launch/slam.launch.py` | **廃止** | `map_session` が担う |
| `launch/teleop.launch.py` / `esp32_keyboard_test.launch.py` | **維持** | 開発用 |
| `config/planning_params.yaml` | **廃止（生成へ）** | 値は `registry.yaml` へ移す。**根拠コメントごと移す** |
| `config/perception_params.yaml` | **廃止（生成へ）** | `blind_angle_ranges` は**幅ゼロのまま移さない**（`DEBT-2`） |
| `config/panels.yaml` | **廃止** | ランタイム登録（`/root/th_data/venue/pins.yaml`）へ |
| `config/ekf_params*.yaml` | **維持** | `imu0` の `vyaw` のみ融合は変えない |
| `config/nav2_params*.yaml` | **改修** | `FollowPath` を使う構成に。局所回避を切る |
| `config/slam_params*.yaml` | **改修** | スロット別のファイル名を受け取れるように |
| `config/safety_monitor_sim.yaml` | **廃止（生成へ）** | `sim` 用の上書きは `export.py --sim`（`allow_placeholder`）と launch 引数 `sim` で行う。**registry にモード別の上書き列は持たない**（[params](DetailedDesign-params.md) §1.3） |
| `config/scenarios/*.yaml`（5 本） | **改修** | `recommended_mode` を新しいモード名に。`planning_overrides` の対象ノード名を更新 |
| `worlds/*.world`（6 本） | **維持** | |
| `maps/th_map.yaml` | **廃止** | `/root/th_data/venue/` へ |
| `config/fastdds_profile.xml` | **維持** | `lidar_filter` にだけ per-node で適用する流儀も維持 |

### 2.10 `th_description` / vendored 3 パッケージ

| 対象 | 扱い |
| --- | --- |
| `th_description`（URDF） | **維持** |
| `leg_detection_bringup` / `multiple_sensor_person_tracking` / `multiple_observation_kalman_filter` | **維持。触らない。**upstream が private repo なので vendoring されている |

### 2.11 `th_testing`

| ファイル | 扱い | 備考 |
| --- | --- | --- |
| `test/test_follow_planner_logic.py`（395 行） | **一部維持** | 幾何関数のテストは残す。`FollowPlannerCore` のテストは削除 |
| `test/test_mapless_follow_logic.py`（459 行） | **維持** | `follow_core` のテストとして引き継ぐ |
| `test/test_esp32_ws_protocol.py`（176 行） | **維持** | |
| `test/test_summon_logic.py` / `test_person_tracker_bridge_logic.py` / `test_yaml_writer.py` | **一部維持** | |
| `test/test_mode_transitions.py`（376 行） | **廃止** | 新しい遷移表のテストに置き換わる |
| `test/test_safety_monitor.py` / `test_fault_detection.py` | **改修** | `severity` と新しいフォルト種別に追随 |
| `test/test_twist_mux_priority.py`（270 行） | **改修** | 出力トピックが `/cmd_vel_muxed` に変わる |
| `test/test_simulation_scenarios.py` | **改修** | 退避（`A1`〜`A3`）のテストは新設計に無いので置き換え |
| `test/test_scenario_configs.py` | **改修** | |
| `test/conftest.py` | **改修** | 新パッケージの `sys.path` を追加。**正本 `docs/plan/spec/` へのパスも渡す**（`spec_ref` の逆引きテスト用） |
| **CMakeLists.txt に未登録の 5 本** | **改修** | `test_mapless_follow_logic` / `test_esp32_ws_protocol` / `test_person_tracker_bridge_logic` / `test_yaml_writer` / `test_scenario_configs` は `pytest` でしか回っていない。**登録する** |

### 2.12 `web_ui`

[webui](DetailedDesign-webui.md) §3 が正。要約のみ。

| 対象 | 扱い |
| --- | --- |
| `src/App.jsx`（35 KB）の 3 タブシェル | **廃止** |
| `src/App.css`（890 行） | **廃止**（モックアップの CSS へ） |
| `VirtualStick` / `CandidateRadar` / `stickToCmd` / 二段階アーム式ボタン | **抽出** |
| `MapView.jsx` / `audience/WorldCanvas.jsx` の projector / `mapGeometry.js` | **土台／維持** |
| `WheelSpeedView.jsx` | **維持**（S-30 の項目 2 へ） |
| `SettingsPanel.jsx` | **土台**（`NumberField` は抽出、画面構成は S-50 へ作り直し） |
| `VoiceDevPanel.jsx` | **廃止**（S-50 の開発モードタブへ統合） |
| `voice/` 一式 | **維持** |
| `audience/` 一式 | **維持**（`main.jsx` で分岐する構造も維持） |
| `hooks/useRosbridge.js`（29 KB） | **土台**。TF 手組みとパラメータサービス呼び出しは**維持**、廃止サービスの呼び出しは差し替え |
| `public/roslib.min.js` | **維持**（CDN を使わない） |

### 2.13 `esp32/`（ファームウェア）

| ファイル | 扱い | 備考 |
| --- | --- | --- |
| `src/config.h` | **改修** | **`ESTOP_BENCH_TEST_BYPASS` を外す**（`DEBT-1`）。`WHEEL_RADIUS_M` は**公称値として固定**（`O-d10`） |
| `src/main.cpp` | **改修** | バイパス削除、`[DBG]` printf 削除（`DEBT-9`） |
| `src/ws_link.h` / `.cpp` | **改修** | ヘッダのフレーム表を実装に合わせる（`DEBT-7`）。`ESTOP_HW` にファーム構成フラグを追加（`DEBT-1` の検出） |
| `src/imu.cpp` | **改修** | BNO055 オフセットの NVS 保存・復元を追加（[maintenance](DetailedDesign-maintenance.md) §5） |
| `src/encoder.*` / `motor.*` / `pid.h` | **維持** | PID の非対称クランプ・アンチワインドアップはそのまま |
| `src/wifi_credentials.h` | **廃止** | `.gitignore` 化（`DEBT-6`） |
| `tools/ws_test_server.py` | **改修** | 3 要素返却に追随（`DEBT-8`） |
| `ESP32-test/`（ベンチ試験プロジェクト） | **維持** | ただし `pins.h` の値が本体とずれていることを README に明記 |

### 2.14 Docker / スクリプト

| ファイル | 扱い | 備考 |
| --- | --- | --- |
| `docker-compose.yml` | **改修** | **`./data:/root/th_data` の bind mount を追加**（`dr_spaam_weights` と同じ流儀） |
| `Dockerfile` | **維持** | DR-SPAAM のパッチと `--symlink-install` を使わない判断はそのまま |
| `setup.sh` | **改修** | WebUI の**本番ビルド**を含める |
| `scripts/run_tests.sh` | **改修** | 新パッケージを追加 |
| `scripts/diagnose.sh` / `verify_*` | **維持** | |
| `udev/99-th-robot.rules` | **維持** | `/dev/lidar` `/dev/esp32` のシンボリックリンク |

---

## 3. 移設の一覧（逆引き）

| 新パッケージ | 主な由来 |
| --- | --- |
| `th_state` | （新規。`th_mode_manager` の置換） |
| `th_params` | `th_config_manager`（`config_manager` / `yaml_writer` / `service_call`） |
| `th_transit` | `th_planning`（`mapless_follow_core` 主体 ＋ `follow_planner_core` の幾何） |
| `th_route` | `th_config_manager/slam_control.py` ＋ 新規 |
| `th_onsite` | `th_planning`（`panel_navigator` / `summon_navigator`） |
| `th_maintenance` | `th_calibration` 4 本の計算部 |
| `th_safety` | 既存＋新規 2 ノード |
