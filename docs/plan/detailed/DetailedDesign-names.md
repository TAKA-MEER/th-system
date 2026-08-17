# 名前辞書

[DetailedDesign.md](DetailedDesign.md) の詳細。**この文書に無い名前を実装で作ってはいけない。**

> **`DD-1`**: 名前を発明する余地をゼロにする。必要な名前が無いと分かったら、
> **実装する前にこのファイルへ行を足す。**

**この文書は機械可読にできる。**`tools/export_names.py` が §2 / §4 / §5 / §6 / §7 / §8 の
表から名前を抜き、`th_ws/web_ui/src/ros/names.json` を作る
（[webui](DetailedDesign-webui.md) **§9.1**）。**WebUI の受け入れ条件 10 がそれを使う。**

```bash
python3 docs/plan/detailed/tools/export_names.py .
```

**表の書式を変えるときは生成器も同時に直す。**列を入れ替えると黙って 0 件になる
——生成器は 0 件で非ゼロ終了するので、そこで気づける。

---

## 0. 命名規則

| 対象 | 規則 | 例 |
| --- | --- | --- |
| パッケージ | `th_<領域>` | `th_state` |
| ノード | 小文字スネーク。パッケージ名を繰り返さない | `state_manager` |
| トピック | 名前空間 `/<領域>/<名前>` | `/system/state` |
| サービス | `/<領域>/<動詞>_<目的語>` | `/onsite/register_pin` |
| モード | 大文字スネーク。**文字列**（数値定数は持たない → §2.1） | `TEACH_FOLLOW` |
| 状態 | 大文字スネーク。モード内で一意 | `WAIT_CLEAR` |
| トリガ／事象 | `<入口>.<名前>`。ドット区切り | `ui.stop` / `evt.arrived` |
| パラメータ | 小文字スネーク。**単位を接尾辞に付ける** | `clear_timeout_ms` |
| 画面 | `S-nn`。十の位が系統 | `S-21` |
| 作業パケット | `WP-<領域>-<番号>` | `WP-STATE-03` |

**速度指令トピックだけは名前空間を付けない**（`/cmd_vel_*`）。
twist_mux の設定と ROS2 の慣行がこの形であり、変えると既存の実機知見が読めなくなる。

---

## 1. パッケージ

| パッケージ | 種別 | 役割 | 現行からの由来 |
| --- | --- | --- | --- |
| `th_system_msgs` | `ament_cmake` (rosidl) | 型定義 | 既存・拡張 |
| **`th_params`** | `ament_cmake` + python | **パラメータ registry・導出・監査**（§7） | 新規（`th_config_manager` の一部を吸収） |
| **`th_state`** | `ament_cmake` + python | **状態機械**。遷移表が正本 | 新規（`th_mode_manager` を置換） |
| `th_safety` | `ament_cmake` (C++) | `safety_monitor` / **`obstacle_limiter`** / twist_mux 設定 | 既存・拡張 |
| **`th_transit`** | `ament_cmake` + python | 走行方式の挙動ノード（`FOLLOW` / `MANUAL` / `TEACH_*` / `LINE` / `LEASH`） | 新規（`th_planning` を置換） |
| **`th_route`** | `ament_cmake` + python | 教示の記録・再生・**地図セッション** | 新規（`slam_control.py` を吸収） |
| **`th_onsite`** | `ament_cmake` + python | 試験場内（`PREP` / `PANEL_NAV` / `AT_PANEL` / `SUMMON` / `HOME_NAV`） | 新規（`panel_navigator` / `summon_navigator` を置換） |
| **`th_maintenance`** | `ament_cmake` + python | 始業点検・校正・故障診断 | 新規（`th_calibration` を吸収） |
| `th_perception` | `ament_cmake` + python | `lidar_filter` / `person_tracker_bridge` | 既存・ほぼ維持 |
| `th_esp32_bridge` | `ament_cmake` + python | ESP32 リンク | 既存・改修 |
| `th_bringup` | `ament_cmake` | launch / config / maps / worlds | 既存・改修 |
| `th_description` | `ament_cmake` | URDF | 既存・維持 |
| `th_testing` | `ament_cmake` + pytest | 全テスト | 既存・拡張 |
| `leg_detection_bringup` ／ `multiple_sensor_person_tracking` ／ `multiple_observation_kalman_filter` | vendored | 人物追跡 | 既存・**触らない** |

**廃止するパッケージ**: `th_mode_manager` ／ `th_planning` ／ `th_calibration` ／ `th_config_manager`。
移設先は [DetailedDesign-reuse.md](DetailedDesign-reuse.md)。

### 1.1 ノード一覧（**launch が使う名前**）

| ノード名 | パッケージ | 実行ファイル | 担当 |
| --- | --- | --- | --- |
| `state_manager` | `th_state` | `scripts/state_manager.py` | 状態機械 |
| `params_audit` | `th_params` | `scripts/params_audit.py` | パラメータ監査 |
| `safety_monitor` | `th_safety` | `src/safety_monitor.cpp` | フォルト・非常停止の集約 |
| **`obstacle_limiter`** | `th_safety` | `src/obstacle_limiter.cpp` | 最終段のリミッタ |
| **`jog_gate`** | `th_safety` | `src/jog_gate.cpp` | 手動指令のゲート（§6.1） |
| `twist_mux` | （外部） | — | 多重化 |
| `connectivity_checker` | `th_state` | `scripts/connectivity_checker.py` | 起動時の疎通確認 |
| `follow_runner` | `th_transit` | `scripts/follow_runner.py` | `FOLLOW` / `TEACH_FOLLOW` / `PREP` の走行 |
| `line_runner` | `th_transit` | `scripts/line_runner.py` | `LINE`（後回し） |
| `leash_runner` | `th_transit` | `scripts/leash_runner.py` | `LEASH`（後回し） |
| `route_recorder` | `th_route` | `scripts/route_recorder.py` | 教示の記録 |
| `replay_runner` | `th_route` | `scripts/replay_runner.py` | 教示再生 |
| `map_session` | `th_route` | `scripts/map_session.py` | 地図スロット |
| `pin_registrar` | `th_onsite` | `scripts/pin_registrar.py` | 2 点指示・ピン |
| `venue_navigator` | `th_onsite` | `scripts/venue_navigator.py` | `PANEL_NAV` / `SUMMON` / `HOME_NAV` |
| `wait_clear_gate` | `th_onsite` | `scripts/wait_clear_gate.py` | 退避待ち |
| `opcheck_runner` | `th_maintenance` | `scripts/opcheck_runner.py` | 始業点検 |
| `calib_runner` | `th_maintenance` | `scripts/calib_runner.py` | 校正 |
| `lidar_filter` | `th_perception` | `scripts/lidar_filter.py` | 死角マスク（既存） |
| `person_tracker_bridge` | `th_perception` | `scripts/person_tracker_bridge.py` | 人物追跡の橋渡し（既存） |
| `esp32_bridge` | `th_esp32_bridge` | `scripts/esp32_bridge.py` | ESP32 リンク（既存） |

**`person_predictor` は廃止する。**`/cmd_vel_retreat` への捜索旋回の発行者であり、
`Spec-transit.md` §1.2「ロストしたら選択を解除して停止」と食い違う
（[DetailedDesign-transit.md](DetailedDesign-transit.md) §1.5）。

### 1.2 フレーム

| frame_id | 発行者 | 用途 |
| --- | --- | --- |
| `map` | `map_session`（slam_toolbox） | ピン・経路・Nav2 のグローバル |
| `odom` | `ekf_filter_node` | 局所的に連続。教示の経路記録 |
| `base_link` | URDF | 機体中心。人物位置・障害物判定の基準 |
| `laser_link` | URDF | `/scan` の原点。**`obstacle_limiter` は TF を引かず、URDF 由来の固定変換を起動時に 1 度取得して保持する**（20 Hz で TF を引かない） |
| `imu_link` | URDF | `/esp32/imu_data` |

**`odom → base_link` の TF を発行するのは `ekf_filter_node` だけ**（既存の不変ルールを維持）。

### 1.3 launch 引数

| 引数 | 既定 | 意味 |
| --- | --- | --- |
| `sim` | `false` | Gazebo か実機か |
| `dev_mode` | `false` | **開発モード。`safety_monitor` と `obstacle_limiter` には渡さない**（構造的な保証） |
| `lidar_source` | `network` | `local` / `network`（既存） |
| `imu_enabled` | `false` | 既存 |
| `scenario` | `''` | Gazebo のシナリオプリセット（既存） |
| `log_level` | `info` | 既存 |

**`use_stub` / `map_yaml` は廃止する。**地図は `map_session` が管理し、
人物追跡のスタブは `scenario` 側で指定する。

---

## 2. モード（18）

### 2.1 モードは文字列で持つ

**数値定数（`RobotMode.msg` の `uint8`）を廃止する。**

| 理由 | |
| --- | --- |
| 遷移表が文字列で書かれている | 数値と文字列の二重管理は必ずずれる（`DD-3`） |
| モードの増減で採番し直しになる | 18 モードは今後も増える（`LINE` / `LEASH` の後にも） |
| 既存の流儀と揃う | `FollowStatus` / `SearchStatus` / `SummonStatus` は既に `string state` を使い、CLAUDE.md も「文字列定義は .msg のコメントが正」としている |

**タイプミスは起動時に落ちる。**`th_state` はロード時に遷移表の全 `mode` / `state` が
下表の集合に含まれることを検証し、外れていれば起動に失敗する。

### 2.2 一覧

| 群 | モード | 表示名（`Spec-webui.md` §8.1 が正） | 画面 | ゾーン | 人物追跡 |
| --- | --- | --- | --- | --- | --- |
| 基盤 | `INIT` | 起動中 | S-00 | 非該当 | 任意 |
| | `IDLE` | 待機 | S-01 / S-21 / S-50 | 画面による | 任意 |
| | `ESTOP` | 非常停止 | 直前の画面 | 直前の画面 | 直前を継続 |
| | `CARRY` | 手押し | 直前の画面 | 直前の画面 | 直前を継続 |
| 走行方式 | `FOLLOW` | 追従走行 | S-10 | 場外 | **要** |
| | `MANUAL` | 手動走行 | S-11 | 場外 | 不要 |
| | `TEACH_FOLLOW` | 教示（追従） | S-12 | 場外 | **要** |
| | `TEACH_MANUAL` | 教示（手動） | S-13 | 場外 | 不要 |
| | `REPLAY` | 教示再生 | S-14 | 場外 | 不要 |
| | `LINE` | ライン誘導 | S-15 | 場外 | 不要 |
| | `LEASH` | 電子リード | S-16 | 場外 | 不要 |
| 試験場内 | `PREP` | 試験準備 | S-20 | **場内** | **要**（登録時のみ） |
| | `PANEL_NAV` | 配電盤へ移動 | S-21 | **場内** | 不要 |
| | `AT_PANEL` | 配電盤前 | S-21 | **場内** | 不要 |
| | `SUMMON` | 呼び寄せ | S-21 | **場内** | **要** |
| | `HOME_NAV` | 待機場所へ移動 | S-21 | **場内** | 不要 |
| 保守 | `OPCHECK` | 始業点検 | S-30 / S-31 | **非該当** | 不要 |
| | `CALIB` | 校正 | S-40 | **非該当** | 不要 |

**「ゾーン」列は参考。**正は §4 の画面表（`Spec-modes.md` §7 ／ `F-19`）。

---

## 3. 状態（サブステート）

**`NONE` は「そのモードに状態が無い」ことを表す予約語。**状態を持たないモードでも空文字にしない。

| モード | 状態 |
| --- | --- |
| `INIT` | `CHECK` |
| `IDLE` | `NONE` |
| `ESTOP` | `NONE` |
| `CARRY` | `NONE` |
| `FOLLOW` | `SELECT` / `CONFIRM` / `RUN` / `PAUSE` |
| `MANUAL` | `RUN` / `PAUSE` |
| `TEACH_FOLLOW` ／ `TEACH_MANUAL` | `ROUTE_SEL` / `REC` / `PAUSE` / `SAVED` |
| `REPLAY` | `ROUTE_SEL` / `LOCALIZE` / `READY` / `RUN` / `PAUSE` / `SAVED` |
| `LINE` | `SETUP` / `PLANNED` / `RUN` / `PAUSE` / `ARRIVED` |
| `LEASH` | `DEV_CHECK` / `READY` / `RUN` / `HOLD` / `PAUSE` |
| `PREP` | `MAPPING` / `REGISTER` / `RETURN` / `EDIT` / `PAUSE` / `SAVED` |
| `PANEL_NAV` | `NAV` / `BLOCKED` / `PAUSE` / `ALIGN` |
| `AT_PANEL` | `IDLE_P` / `WORKING` / `PAUSE` |
| `SUMMON` | `POINT` / `WAIT_CLEAR` / `NAV` / `BLOCKED` / `PAUSE` / `ALIGN` |
| `HOME_NAV` | `NAV` / `BLOCKED` / `PAUSE` |
| `OPCHECK` | `LIST` / `RUNNING_CHECK` / `REPAIR` |
| `CALIB` | `LIST` / `S1` / `S2` / `S3` / `S4` |

**`STOP` という状態名は存在しない**（`Spec-modes.md` §3.1.1）。「停止」は操作の名前である。

**`ARRIVED` は `LINE` にしか無い**（唯一の自動停止。`F-02`）。
`PANEL_NAV` / `SUMMON` / `HOME_NAV` の到着は**滞在する状態ではない**。
状態として持つと**到達不能状態になり `validate()` が落ちる**ので、状態集合から外す。
到着したことは `effects: [mark_arrived]` と `/system/state.last_event` で表す。

> **2026-08-16 反映済み**: 正本 `Spec-modes.md` §3・§3.1.2 の `ARRIVED` 表記を
> 「到着（`AT_PANEL` へ）」に直し、**§3.0-① に「滞在するのは `LINE` だけ」と明記した**
> （[DetailedDesign-open.md](DetailedDesign-open.md) §3 A-4）。

### 3.1 フラグ

モードと直交し、モードを変えない。`SystemState` に載る。

| フラグ | 型 | 意味 | 誰が立てるか |
| --- | --- | --- | --- |
| `jog_active` | bool | 手動ジョグ介入中（**リース式** → [DetailedDesign-state.md](DetailedDesign-state.md) §5） | `th_state`（UI のリース受領） |
| `estop_ui` | bool | WebUI の非常停止が押されている | `safety_monitor` |
| `estop_hw` | bool | 物理非常停止が押されている | `safety_monitor`（ESP32 由来） |
| `tracker_enabled` | bool | 人物追跡が動いている | `th_state`（S-50 の設定） |
| `auto_brake` | bool | 自動ブレーキが有効 | `th_state`（ゾーン既定＋UI トグル） |
| `working` | bool | 盤前の「作業中」ボタンが ON | `th_state` |
| `map_update` | bool | **地図の書き足しが ON。**`REPLAY` ＋ 試験場内 4 モード（`PANEL_NAV` / `SUMMON` / `HOME_NAV` / `AT_PANEL`。[onsite](DetailedDesign-onsite.md) §3.7.3） | `th_state` |
| `unsaved` | string[] | 未保存の種別（`venue_map` / `route` / `calib` / `map_patch`） | 各機能ノードが `evt.unsaved.*` で申告 |

---

## 4. 画面（15）とゾーン

**ゾーンは 3 値である**（`Spec-modes.md` §7）。真偽値にしてはいけない
（`OPCHECK` / `CALIB` に場外の `v_max` が適用されてしまう）。

| ID | 画面名 | ゾーン | 速度上限 |
| --- | --- | --- | --- |
| `S-00` | 接続確認 | `NA` | 停止 |
| `S-01` | メインメニュー | `OUT` | 停止 |
| `S-10` | 追従走行 | `OUT` | `v_max` |
| `S-11` | 手動走行 | `OUT` | `v_max` |
| `S-12` | 教示（追従） | `OUT` | `v_max` |
| `S-13` | 教示（手動） | `OUT` | `v_max` |
| `S-14` | 教示再生走行 | `OUT` | `v_max` |
| `S-15` | ライン誘導走行 | `OUT` | `v_max` |
| `S-16` | 電子リード走行 | `OUT` | `v_leash` |
| `S-20` | 試験準備 | **`IN`** | `v_slow` |
| `S-21` | 試験 | **`IN`** | `v_slow` |
| `S-30` | 始業点検 | **`NA`** | `v_check` |
| `S-31` | 故障診断 | **`NA`** | 停止 |
| `S-40` | 校正 | **`NA`** | `v_calib` |
| `S-50` | 設定 | `OUT` | 停止 |

> **2026-08-16 解消**: `mockup/index.html` の `SCREENS[].zone` は真偽値で `NA` を
> `false`（＝場外）に潰していたが、**3 値（`'IN'` / `'OUT'` / `'NA'`）＋画面ごとの `limit` に直した**
> （[DetailedDesign-open.md](DetailedDesign-open.md) §3 A-8）。`Spec-modes.md` §7 ／
> `Spec-webui.md` §2 の表と本表の 3 者が一致している状態が正。

### 4.1 ゾーンの合成は「速度上限の最小値」で行う

**ゾーンの強弱（`IN` > `NA` > `OUT`）で合成してはいけない。**
`NA` の画面は S-31＝停止、S-40＝`v_calib`、S-30＝`v_check` で、**`IN` の `v_slow` より低い**。
`IN` へ倒すと**上限が上がる**（S-40 で校正中に別端末が S-21 を開くと `v_calib` → `v_slow`）。

```python
def derive_limits(screens, now_ms, p):
    active = [s for s in screens.values()
              if s.interacting and now_ms - s.last_input_ms <= p.ui_active_window_s * 1000]
    if not active:
        return Limits(zone="NA", speed_limit="stop", auto_brake=True)   # 途絶は停止へ倒す
    zone  = "IN" if any(SCREEN_ZONE[s.screen_id] == "IN" for s in active) else ...
    limit = min(SCREEN_SPEED_LIMIT[s.screen_id] for s in active)        # ← 上限は最小値
    return Limits(zone=zone, speed_limit=limit, auto_brake=(zone == "IN") or default)
```

| 状況 | 速度上限 | ゾーン表示 |
| --- | --- | --- |
| 使用中の端末が 1 台 | その画面の上限 | その画面のゾーン |
| 使用中の端末が複数 | **各画面の上限の最小値** | いずれかが `IN` なら `IN` |
| **`/ui/active_screen` が途絶／使用中が 0 台** | **停止** | `NA`。自動ブレーキ ON |

**「使用中」の定義は `Spec-safety.md` §6.2 と同一**（`interacting` かつ `ui_active_window_s` 以内）。
見ているだけ・開いているだけは含めない。

### 4.2 ウィンドウ ID

| ID | 種別 |
| --- | --- |
| `W-1` | 異常ウィンドウ |
| `W-2` | 手押しウィンドウ |
| `W-3` | 案内ウィンドウ |
| `W-4` | 確認ウィンドウ |
| `W-5` | 経路ブロックウィンドウ |
| `W-6` | 手動操作パネル |

案内キー（`guide{key}` の引数）: `estop_held_at_boot` / `route_end` / `home_arrived` /
`leash_not_connected` / `target_lost` / `register_rejected` / `line_lost` / `wait_clear_timeout` /
`blind_mask_uncalibrated` / `params_placeholder`。**日本語は `i18n/guides.js` が持つ。**

---

## 5. メッセージ・サービス（`th_system_msgs`）

### 5.1 メッセージ

| ファイル | フィールド | 備考 |
| --- | --- | --- |
| **`SystemState.msg`** | `Header header` / `string mode` / `string state` / `string prev_mode` / `string prev_state` / `string zone`（`IN`/`OUT`/`NA`） / `bool jog_active` / `bool estop_ui` / `bool estop_hw` / `bool tracker_enabled` / `bool auto_brake` / `bool working` / `bool map_update` / `string[] unsaved` / `builtin_interfaces/Time since` / `string last_event` / `string last_reject_reason` | **状態の唯一の発行元。**`prev_*` は `ESTOP` / `CARRY` の復帰先ラッチ |
| **`StateEvent.msg`** | `Header header` / `string event`（`evt.*` のみ） / `string source_node` / `string arg_json` | 挙動ノード → `th_state` |
| **`ActiveScreen.msg`** | `Header header` / `string screen_id` / `string client_id` / `bool interacting` / **`builtin_interfaces/Time last_input`** | UI → `th_state`。**`header.stamp` は 2 Hz の定期発行時刻であって「最後の操作時刻」ではない。**`last_input` が無いと、画面を開いているだけの端末が永久に「使用中」になる |
| `FaultStatus.msg` | `Header header` / `bool active` / `string fault_type` / **`string severity`**（`RECOVERABLE` / `CRITICAL`） / `string description` | **`severity` を追加**（`F-20`） |
| **`LimiterStatus.msg`** | `Header header` / `bool alive` / `string action`（`PASS`/`CLAMP`/`STOP`/`ZERO_STALE`/**`BLOCKED_UNCALIBRATED`**） / `float32 in_linear` / `float32 out_linear` / `float32 nearest_obstacle_m` / `string source_class`（`MANUAL`/`AUTO`） / `float32 applied_limit_mps` | 監視と画面表示の両方に使う |
| **`WaitClearStatus.msg`** | `Header header` / `float32 distance_m` / `float32 remaining_sec` / `bool satisfied` / `string verdict`（`OK`/`WAITING`/`NOT_CLEAR`） | 退避待ちの表示 |
| **`RouteList.msg`** | `Header header` / `RouteInfo[] routes` | transient_local。トピック `/route/catalog` |
| **`LinkQuality.msg`** | `Header header` / `string link`（`esp32`/`lidar`/`ui`） / `float32 p50_ms` / `float32 p99_ms` / `float32 max_ms` / `uint32 window_sec` | タイムアウトの根拠を実測で持つ |
| **`ParamsStatus.msg`** | `Header header` / `uint16 placeholder_count` / `string[] placeholder_names` / `string digest` | ヘッダのバッジ源（§7） |
| **`PersonTargets.msg`** | `Header header` / `geometry_msgs/Point[] candidates` / `int32 selected_index` / `float32 confidence` / `bool is_lost` / `string lost_reason` | `PersonStatus` ＋候補一覧を 1 本に統合 |
| **`Pin.msg`** | `string id` / `string name` / `string kind`（`HOME`/`PANEL`） / `geometry_msgs/Pose pose` / `builtin_interfaces/Time registered_at` | 待機場所ピンと配電盤ピン |
| **`PinList.msg`** | `Header header` / `Pin[] pins` | transient_local |
| **`RouteInfo.msg`** | `string id` / `string name` / `uint32 generation` / `float32 length_m` / `uint32 point_count` / `float32 start_yaw` / `builtin_interfaces/Time recorded_at` | 教示経路のメタ |
| **`RouteStatus.msg`** | `Header header` / `string state` / `RouteInfo current` / `float32 recorded_m` / `float32 elapsed_sec` / `uint32 points` | S-12/S-13/S-14 の表示源 |
| **`MapSessionStatus.msg`** | `Header header` / `string slot`（`VENUE`/`ROUTE`） / `string session_id` / `string mode`（`UNLOADED`/`MAPPING`/`LOCALIZING`） / `bool dirty` | §6.4 |
| **`CheckStatus.msg`** | `Header header` / `string item` / `string result`（`OK`/`WARN`/`NG`/`UNKNOWN`） / `string detail` / `string next_screen` | 始業点検 4 項目 |
| **`CalibStatus.msg`** | `Header header` / `string item` / `string step` / `string result` / `string preview_before` / `string preview_after` / `string detail` | 校正ウィザード |
| `WheelFeedback.msg` | 既存のまま | 実測と指令の両方に使う |

**廃止する msg**: `RobotMode.msg`（§2.1）／`PersonStatus.msg`（`PersonTargets` へ統合）／
`FollowStatus.msg`・`SearchStatus.msg`・`SummonStatus.msg`（`SystemState` ＋ 各機能の status へ統合）／
`WheelCommand.msg`（未使用）／`PanelArrival.msg`（`evt.arrived` へ）。

### 5.2 サービス

| サービス | 型 | 提供者 |
| --- | --- | --- |
| **`/system/trigger`** | `UiTrigger`: `string trigger` / `string arg_json` / `string requester` → `bool accepted` / `string reject_reason_key` / `string mode` / `string state` | `th_state` |
| `/system/set_flag` | `SetFlag`: `string flag` / `bool value` → `bool accepted` / `string reject_reason_key` | `th_state` |
| `/route/list` | `ListRoutes`: — → `RouteInfo[] routes` | `th_route` |
| `/route/save` | `SaveRoute`: `string name` → `bool success` / `RouteInfo info` / `string message` | `th_route` |
| `/route/delete` | `DeleteRoute`: `string id` → `bool success` / `string message` | `th_route` |
| `/route/export` | `TransferRoute`: `string id` / `string path` → `bool success` / `string message` | `th_route` |
| `/route/import` | `TransferRoute` | `th_route` |
| `/map_session/open` | `OpenMapSession`: `string slot` / `string session_id` / `string mode` → `bool success` / `string message` | `th_route` |
| `/map_session/save` | `std_srvs/Trigger` | `th_route` |
| `/map_session/discard` | `std_srvs/Trigger` | `th_route` |
| `/onsite/two_point` | `TwoPointPress`: `string purpose`（`HOME`/`PANEL`/`SUMMON`/`LINE_POSE`） / `uint8 index`（1 or 2） → `bool accepted` / `string reject_reason_key` / `float32 yaw` | `th_onsite` |
| `/onsite/register_pin` | `RegisterPin`: `string kind` / `string name` / `string method`（`TWO_POINT`/`PLANE`） → `bool success` / `Pin pin` / `string message` | `th_onsite` |
| `/onsite/edit_pin` | `EditPin`: `string id` / `string new_name` / `bool delete` → `bool success` / `string message` | `th_onsite` |
| `/onsite/declare_home` | `DeclareHome`: `bool force` → `bool success` / `float32 offset_m` / `float32 offset_deg` / `string message` | `th_onsite` |
| `/onsite/map_erase` | `EraseMapRegion`: `float32 x0,y0,x1,y1` / `bool undo` → `bool success` | `th_route` |
| `/opcheck/run_item` | `RunCheck`: `string item` → `bool started` / `string message` | `th_maintenance` |
| `/opcheck/answer` | `AnswerCheck`: `string item` / `bool ok` → `bool accepted` | `th_maintenance` |
| `/calib/start` | `StartCalib`: `string item` → `bool started` / `string message` | `th_maintenance` |
| `/calib/submit` | `SubmitCalib`: `string item` / `float64 measured` / `string arg_json` → `bool success` / `string preview_before` / `string preview_after` | `th_maintenance` |
| `/calib/apply` | `ApplyCalib`: `string item` → `bool success` / `string message` | `th_maintenance` |
| `/calib/rollback` | `RollbackCalib`: `string item` / `uint8 generation` → `bool success` | `th_maintenance` |
| `/params/get` | `GetParams`: `string[] names` → `string json` | `th_params` |
| `/params/set` | `SetParams`: `string json` → `bool success` / `string message` | `th_params` |
| `/params/save` | `std_srvs/Trigger` | `th_params` |
| `/shutdown/prepare` | `std_srvs/Trigger` → 未保存一覧 | `th_state` |
| `/shutdown/execute` | `std_srvs/Trigger` | `th_state` |
| **`/safety/clear_estop_ui`** | `std_srvs/Trigger` | **`safety_monitor`**。**UI に依存しない解除経路**（[safety](DetailedDesign-safety.md) §6.3.1 ／ `N-4`）。`/safety/estop_hw` が `false` かつ重大フォルトが無いときだけ受理し、**必ずログに残す**。**通常運用では使わない**——WebUI の解除ボタンが正規の経路である |

**廃止・改名するサービス**:

| 現行 | 扱い |
| --- | --- |
| `/mode_manager/set_mode`（`SetMode.srv`） | **廃止** → `/system/trigger` |
| `/panel_navigator/go_to_panel`（`GoToPanel.srv`） | **廃止** → `ui.goto` |
| `/panel_navigator/complete_inspection`（`CompleteInspection.srv`） | **廃止** → `ui.working{on:false}` |
| `/config_manager/set_tunable_params`（`SetTunableParams.srv`） | **廃止** → `/params/set` |
| `/config_manager/save_tunable_params`（`SaveTunableParams.srv`） | **廃止** → `/params/save` |
| `/slam_control/*` | **廃止** → `/map_session/*` |
| `/safety/tablet_estop` | **改名** → `/safety/estop_ui`（端末はタブレットとは限らない） |
| `/summon_navigator/call` | **廃止** → `ui.goto{kind:SUMMON}` |

**これらは既存 WebUI（`useRosbridge.js`）が呼んでいる。**廃止と UI 改修は**同一の作業パケット**で行う
（片方だけ進めると設定パネルと地図作成が無言で壊れる）。

---

## 6. トピック

### 6.1 速度指令（**最重要**）

```
th_transit / th_onsite / th_route ──► /cmd_vel_behavior  (priority 20) ─┐
WebUI（ジョグ・手動走行）           ──► /cmd_vel_manual    (priority 30) ─┤
Nav2 controller_server              ──► /cmd_vel_nav       (priority 10) ─┤
                                                                          │
safety_monitor ──► /safety/estop      (lock 255) ────────────────────────┤ twist_mux
safety_monitor ──► /safety/fault_lock (lock 254) ────────────────────────┘
                                                                          │
                                                      /cmd_vel_muxed ─────┘
                                                            │
                                                   [ obstacle_limiter ]  ← /scan, /system/state, /cmd_vel_manual
                                                            │
                                                        /cmd_vel ──► esp32_bridge ──► ESP32
```

| トピック | 型 | 優先度 | タイムアウト | 発行者 |
| --- | --- | --- | --- | --- |
| **`/cmd_vel_manual_raw`** | `geometry_msgs/Twist` | — | — | **WebUI（rosbridge 直）。`jog_gate` の手前** |
| `/cmd_vel_manual` | `geometry_msgs/Twist` | **30** | `manual_joy_timeout`（現行 1.0 s） | **`jog_gate`**（通さないときは**沈黙する**。ゼロを撃たない） |
| `/cmd_vel_behavior` | `geometry_msgs/Twist` | **20** | `0.5 s` | 走行方式・試験場内の挙動ノード |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | **10** | `0.5 s` | Nav2 `controller_server` |
| **`/cmd_vel_muxed`** | `geometry_msgs/Twist` | — | — | **`twist_mux`（`cmd_vel_out` の remap 先を変更）** |
| `/cmd_vel` | `geometry_msgs/Twist` | — | `cmd_vel_stale_ms` | **`obstacle_limiter`（唯一の発行者）** |

> **`/cmd_vel_retreat` は廃止する。**近接退避（`follow_planner_core` の `PREPARE` / `EVADING`）は
> `Spec-transit.md` §1.2「対象に近づいたとき: **停止する**」と食い違う。
> 退避も捜索旋回も新設計には無い。

**不変ルール（現行から変更）**: `/cmd_vel` を publish してよいのは **`obstacle_limiter` だけ**。
`twist_mux` も直接は出さない。

### 6.2 状態・安全

| トピック | 型 | QoS | レート |
| --- | --- | --- | --- |
| `/system/state` | `SystemState` | transient_local, depth 1, reliable | 10 Hz ＋ 変化時即時 |
| `/system/event` | `StateEvent` | reliable, depth 10 | 事象時 |
| `/system/params_status` | `ParamsStatus` | transient_local, depth 1 | 変化時 |
| `/ui/active_screen` | `ActiveScreen` | reliable, depth 5 | 2 Hz（端末ごと） |
| `/safety/estop_hw` | `std_msgs/Bool` | reliable | 10 Hz |
| `/safety/estop_ui` | `std_msgs/Bool` | reliable | 押下・解除時＋2 Hz |
| `/safety/estop` | `std_msgs/Bool` | reliable | 10 Hz |
| `/safety/fault` | `FaultStatus` | reliable, depth 5 | 変化時 |
| `/safety/fault_lock` | `std_msgs/Bool` | reliable | 10 Hz |
| `/safety/limiter_status` | `LimiterStatus` | best_effort, depth 1 | **20 Hz**（heartbeat 兼用） |
| `/safety/link_quality` | `LinkQuality` | best_effort | 1 Hz |
| **`/safety/firmware_flags`** | `std_msgs/UInt8` | **transient_local, depth 1** | 変化時＋接続時（[hardware](DetailedDesign-hardware.md) §3.1） |
| **`/esp32/battery`** | `sensor_msgs/BatteryState` | reliable, depth 1 | **1 Hz**（[hardware](DetailedDesign-hardware.md) §3.3） |

### 6.3 知覚

| トピック | 型 | 備考 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | RaspberryPi4 から。**`obstacle_limiter` はこちらを使う**（§理由は [DetailedDesign-safety.md](DetailedDesign-safety.md) §4.3） |
| `/scan_filtered` | `sensor_msgs/LaserScan` | 死角マスク済み。SLAM・costmap・人物追跡用 |
| `/person/targets` | `PersonTargets` | 候補一覧＋選択中を 1 本に |
| `/esp32/wheel_feedback` ／ `/esp32/wheel_cmd_speed` ／ `/esp32/imu_data` ／ `/esp32/imu_calib_status` | 既存のまま | |
| `/odom` ／ `/odometry/filtered` ／ `/tf` ／ `/map` ／ `/plan` | 既存のまま | |

### 6.4 機能別

| トピック | 型 |
| --- | --- |
| トピック | 型 | QoS |
| --- | --- | --- |
| `/route/status` | `RouteStatus` | reliable, depth 1, 2 Hz |
| **`/route/catalog`** | `RouteList` | **transient_local**, depth 1（サービス `/route/list` と同内容） |
| `/map_session/status` | `MapSessionStatus` | transient_local, depth 1 |
| `/onsite/pins` | `PinList` | transient_local, depth 1 |
| `/onsite/wait_clear` | `WaitClearStatus` | reliable, depth 1, 5 Hz |
| `/opcheck/status` | `CheckStatus` | reliable, depth 5 |
| `/calib/status` | `CalibStatus` | reliable, depth 5 |
| `/leash/status` ／ `/line/status` | 各 status | reliable, depth 1 |
| **`/ui/jog_lease`** | `std_msgs/String`（`client_id`） | **best_effort**, depth 1, 5 Hz 以上 |

> **`/route/list` はサービス名。**トピック側は **`/route/catalog`** に改名した（名前の衝突を解いた）。

### 6.5 §6.1・§6.3 の QoS

| トピック | QoS |
| --- | --- |
| `/cmd_vel_manual` ／ `/cmd_vel_behavior` ／ `/cmd_vel_nav` ／ `/cmd_vel_muxed` ／ `/cmd_vel` | **reliable, depth 1**（twist_mux の既定に合わせる） |
| `/person/targets` | reliable, depth 1 |
| `/scan` ／ `/scan_filtered` | **SensorDataQoS**（best_effort） |
| `/map` | transient_local, depth 1 |

---

## 7. パラメータ

**数値は `th_params/registry.yaml` にしか書かない**（`DD-6` ／ [DetailedDesign-params.md](DetailedDesign-params.md)）。
ここでは**名前と分類だけ**を固定する。

### 7.1 速度

| 名前 | 単位 | 分類 | 適用先 |
| --- | --- | --- | --- |
| `v_max` | m/s | (b) | ゾーン `OUT` |
| `v_slow` | m/s | (b) | ゾーン `IN` |
| `v_leash` | m/s | (b) | `LEASH`（一定速） |
| `v_reverse` | m/s | (b) | 後退（全モード共通の上限） |
| `v_jog_panel` | m/s | (b) | `AT_PANEL` のジョグ |
| `v_check` | m/s | (b) | `OPCHECK` のモーター確認 |
| `v_calib` | m/s | (b) | `CALIB` の自律走行・旋回 |
| `w_max` | rad/s | (b) | 角速度上限 |
| **`w_align_max`** | rad/s | (b) | **`d_floor` を割っている間の角速度上限**（safety L3） |
| `speed_preset_low` ／ `_mid` ／ `_high` | 比率 | given | UI プリセット。**上限に対する割合**。`consumers: [web_ui]` |
| `drivetrain_ceiling_mps` | m/s | (b) | 出力上限 ÷ フィードフォワード係数 |
| **`v_max_headroom_ratio`** | — | given | `v_max` を天井の何割にするか（補正の余地を残す） |
| **`venue_clearance_m`** ／ **`blind_clearance_m`** ／ **`panel_clearance_m`** | m | (a)／方針値 | `v_slow` ／ `v_reverse` ／ `v_jog_panel` の逆算元 |

### 7.2 距離

| 名前 | 単位 | 分類 |
| --- | --- | --- |
| `brake_accel_mps2` | m/s² | **(c) 最優先の実測項目** |
| `brake_delay_s` | s | (b) |
| `obstacle_stop_distance_m` | m | (b) 導出（挙動ノード側 `d_behavior`） |
| `obstacle_floor_distance_m` | m | (b) 導出（リミッタ側 `d_floor`。**必ず `< obstacle_stop_distance_m`**） |
| `follow_stop_distance_m` | m | (b) 導出 |
| `clear_distance_m` | m | (b) |
| `two_point_spacing_m` ／ `two_point_min_spacing_m` | m | (b) |
| `intrusion_budget_m` | m | **(a)／方針値。**通信断で進んでよい距離の上限。**逆算元ではなく人が決める** |
| `safety_margin_m` ／ `floor_margin_m` ／ `person_margin_m` | m | (b)。§7.5 |
| `hysteresis_ratio` ／ `hysteresis_band_m` | — / m | (b) |
| `body_width_m` ／ `body_length_m` ／ **`body_half_length_m`** | m | given（機体外形） |
| **`clear_margin_m`** | m | (b) | 退避待ちゲートの余裕 |
| `corridor_width_m` | m | **(a)**（`O-a4`） |
| `deviation_budget_m` | m | (b) 導出 |
| `nav_tolerance_m` ／ `nav_tolerance_deg` | m / deg | given（Nav2 の現行設定値） |
| `person_position_sigma_m` | m | **(c)**（`O-c3`） |
| `blocked_lookahead_m` | m | (b) |
| `home_declare_tolerance_m` ／ `home_declare_tolerance_deg` | m / deg | (b) |
| `align_tolerance_rad` | rad | (b) |
| `route_sample_interval_m` ／ `route_jump_m` | m | (b) |
| `replay_drift_m_per_100m` | m | **(c)**（`O-c4`） |
| **`link_gap_p99_ms`** | ms | **(c)**。`value_by: [esp32, lidar, ui]`（`WP-MEAS-04`） |
| **`battery_endurance_min`** | 分 | **(c)**（`O-c7`。`WP-MEAS-05`） |
| **`battery_warn_v`** ／ **`battery_critical_v`** | V | given（[hardware](DetailedDesign-hardware.md) §3.3） |
| **`obstacle_cone_half_width_rad`** ／ **`obstacle_cone_half_width_reverse_rad`** | rad | (b)。**リミッタの判定コーン幅**（前方／後退で別値） |
| **`blind_angle_ranges`** | deg のペア列 | **(c)**。死角セクタ。`list[[a0,a1]]`（[wp2](DetailedDesign-wp2.md) `WP-CALIB-01` §5） |

### 7.5 マージン 3 種の使い分け

| 名前 | 何に足すか | 分類 | なぜ分けるか |
| --- | --- | --- | --- |
| `safety_margin_m` | 挙動ノードの `obstacle_stop_distance_m` | (b) | 方式ごとの意味論を含む余裕 |
| **`floor_margin_m`** | リミッタの `obstacle_floor_distance_m` | (b) | **`safety_margin_m` より小さい。**A2 の成否を単独で決める |
| `person_margin_m` | `follow_stop_distance_m` | (b) | 人に対する余裕。障害物より大きく取る |

### 7.3 時間

| 名前 | 単位 | 分類 |
| --- | --- | --- |
| `tracker_lost_grace_ms` | ms | (c) |
| `auto_select_hold_s` | s | (b)。**`E-8`** の「数秒」 |
| `clear_hold_ms` ／ `clear_timeout_ms` | ms | (b) |
| `lidar_timeout_ms` ／ `esp32_timeout_ms` ／ `person_timeout_ms` | ms | (b) 導出 |
| **`muxed_stale_ms`** | ms | (b)。**リミッタ**が `/cmd_vel_muxed` の途絶を判定する |
| **`cmd_vel_stale_ms`** | ms | (b)。**`esp32_bridge`** が `/cmd_vel` の途絶を判定する（`< esp32_watchdog_ms`） |
| **`scan_stale_ms`** | ms | (b)。**リミッタ**が `/scan` の途絶を判定する。超えたら `action = STOP` |
| `jog_lease_ms` | ms | (b)。**`≥ manual_joy_timeout`** |
| **`manual_joy_timeout`** | s | given。**`twist_mux.yaml` の `manual_joy.timeout` と同一の値。registry から生成する** |
| **`estop_ui_lease_ms`** | ms | (b)。UI 非常停止の押下継続を判定する |
| `screen_stale_ms` | ms | (b) |
| **`state_stale_ms`** | ms | (b)。リミッタ／`jog_gate` が `/system/state` の途絶を判定する |
| **`lock_stale_ms`** | ms | (b)。`/safety/estop` `/safety/fault_lock` の途絶をロック扱いにする閾値（現行 0.5 s） |
| **`estop_ui_repeat_hz`** | Hz | given。UI 非常停止の押下継続の送信頻度 |
| **`limiter_dead_ms`** | ms | (b)。`/safety/limiter_status` の途絶（20 Hz の 5 周期） |
| **`mux_dead_ms`** | ms | (b)。`MUX_DEAD` の判定（[wp2](DetailedDesign-wp2.md) `WP-SAFE-01` §4.1） |
| **`runaway_hold_ms`** | ms | (b)。`DRIVE_RUNAWAY` の保持時間 |
| **`link_quality_window_sec`** | s | given。分位点を取る窓（`WP-SAFE-00`） |
| **`behavior_cmd_timeout_s`** ／ **`nav_cmd_timeout_s`** | s | given。`twist_mux.yaml` の生成元（現行 0.5 s） |
| **`wheel_radius_scale`** | — | measured（校正の出力） |
| **`wheel_radius_scale_max_dev`** | — | given。A10 の閾値 |
| **`runaway_ratio`** ／ **`runaway_zero_threshold`** | — / m/s | (b)。`DRIVE_RUNAWAY` の乖離判定 |
| **`link_quality_regression_ratio`** | — | given。カメラ接続後の `/scan` p99 悪化の許容比（[hardware](DetailedDesign-hardware.md) §2.1） |
| `ui_active_window_s` | s | (b)。`Spec-safety.md` §6.2 の「使用中」（**旧 `tablet_active_window_s` から改名**） |
| `esp32_alive_timeout_ms` | ms | (b)。**起動時の疎通判定にだけ使う**（運用中は `esp32_timeout_ms`） |
| **`enabled_targets`** | string[] | given。`safety_monitor` の監視対象（**段階ごとに launch から渡す**。`O-7`） |
| **`scan_expected_points`** | — | given。起動時の疎通判定（`WP-STATE-03`） |
| **`required_nodes`** | string[] | given。同上 |
| **`esp32_watchdog_ms`** | ms | given。`esp32/src/config.h` の写し |
| **`link_wait_timeout_ms`** | ms | (b)。`sys.link_timeout` の契機 |
| `blocked_hold_ms` ／ `unblocked_hold_ms` | ms | (b) |
| `route_gap_timeout_ms` | ms | (b) |
| `leash_stop_latency_ms` | ms | (c) |
| `calib_interval_days` | 日 | (c) |

**`muxed_stale_ms` と `cmd_vel_stale_ms` は別物である。**
前者はリミッタの入力（20 Hz 周期より少し長い）、後者は ESP32 の手前（`esp32_watchdog_ms` より短い）。
1 つの名前にすると片方が誤発火する。

### 7.4 許容範囲（校正）

`calib_linear_tolerance_ratio` ／ `calib_rotation_tolerance_deg` ／ `calib_blind_tolerance_deg` — すべて **(c)**。

---

## 8. トリガと事象の名前

**入口が 3 本ある。名前空間の分離はそれ自体が安全境界である**
（`th_state` は挙動ノードからの `ui.*` を拒否し、UI からの `evt.*` を拒否する）。

| 入口 | 経路 | 誰が出すか |
| --- | --- | --- |
| `ui.*` | サービス `/system/trigger`。**ただし `ui.jog.hold` だけトピック `/ui/jog_lease`**（§8.4） | WebUI のみ |
| `evt.*` | トピック `/system/event` | 挙動ノードのみ |
| `fault.*` ／ `hw.*` | `th_state` が `/safety/fault` `/safety/estop*` を直接購読して内部生成 | `safety_monitor` のみ |
| **`sys.*`** | **`th_state` の内部タイマ。**外から入れられない | `th_state` 自身 |

`sys.*` は 2 つだけ。

| 事象 | 契機 |
| --- | --- |
| `sys.jog_lease_expired` | 最後の `ui.jog.hold` から `jog_lease_ms` 経過 |
| `sys.link_timeout` | `INIT/CHECK` で `link_wait_timeout_ms` 経過 |

### 8.1 `ui.*`（WebUI 操作）

| トリガ | 出す画面 | 対応するボタン |
| --- | --- | --- |
| `ui.enter_mode` | S-01 | 走行方式・試験準備・始業点検・校正の各ボタン。`arg_json` に `{"mode": "..."}` |
| `ui.stop` | 全画面 | **停止**（操作カード左上） |
| `ui.confirm` | S-10 / S-12 | **確認**（操作カード中上） |
| `ui.run` | S-10 / S-12 / S-14 / S-15 / S-16 | **走行 / 再生**（操作カード右上）。ラベルは状態で変わるがトリガは 1 つ |
| `ui.save` | S-12 / S-13 / S-14 / S-20 | **保存**（操作カード左下） |
| `ui.finish` | 全画面 | **終了**（画面左上の操作バー） |
| `ui.jog.hold` | 全走行方式 / S-20 / S-21 | 仮想スティック。**リースなので繰り返し送る**（§8.4） |
| `ui.select_target` | S-10 / S-12 / S-20 / S-21 | レーダーの候補タップ。`arg_json` に `{"index": n}` |
| `ui.route_select` | S-12 / S-13 / S-14 | 経路の選択。`{"id": "...", "reverse": bool}` |
| `ui.resume_yes` ／ `ui.resume_no` ／ `ui.resume_ack` | 異常ウィンドウ W-1 | はい／いいえ／確認 |
| `ui.abort` | S-21（退避待ち） / S-40（校正） | **中止** |
| `ui.working` | S-21 | 作業中ボタン。`{"on": bool}` |
| `ui.goto` | S-21 | 行き先の選択。`{"kind": "PANEL"|"HOME"|"SUMMON", "pin_id": "..."}` |
| `ui.register` | S-20 | 「待機場所を登録」「配電盤を登録」 |
| `ui.return_home` | S-20 | 「1 ボタンで待機場所に戻す」 |
| `ui.map_edit` | S-20 | 地図修正へ入る |
| `ui.check_item` | S-30 | 点検項目の選択。`{"item": "..."}` |
| `ui.calib_item` | S-40 | 校正項目の「開始」。`{"item": "..."}` |
| `ui.calib_next` | S-40 | ウィザードの次へ |
| `ui.reroute` | W-5 | 「再検索する」 |
| `ui.localize_global` | S-14 | 「完全グローバルで探す」 |
| `ui.screen` | 全画面 | 画面遷移の宣言（ゾーン導出用。遷移は起こさない） |
| **`ui.carry_resume`** | W-2 | **「再開」**（押下前のモードへ戻る） |
| **`ui.estop.press` / `ui.estop.release`** | ヘッダ | **`safety_monitor` 経由**で `th_state` に届く（UI から直接ではない） |

### 8.2 `evt.*`（挙動ノードの事象）

| 事象 | 出すノード |
| --- | --- |
| `evt.link_ok` | `connectivity_checker`（必須 3 者が疎通） |
| `evt.arrived` | `panel_navigator` / `home_navigator` / `summon_navigator` / `line_runner` |
| `evt.align_done` | 各 navigator |
| `evt.blocked` ／ `evt.unblocked` | 各 navigator |
| `evt.target_lost` | `person_tracker_bridge` |
| `evt.auto_selected` | `person_tracker_bridge`（`auto_select_hold_s` 成立） |
| `evt.clear_ok` ／ `evt.clear_timeout` | `wait_clear_gate` |
| `evt.localize_done` ／ `evt.localize_low` | `replay_runner` |
| `evt.leash_taut` ／ `evt.leash_slack` ／ `evt.leash_absent` | `leash_runner`（後回し） |
| `evt.line_lost` | `line_runner`（後回し） |
| `evt.plane_done` | `pin_registrar` |
| **`evt.two_point_done`** | `pin_registrar`（2 点指示の②が押された） |
| **`evt.setup_done`** | `line_runner`（現在地・向き・目的地の 3 指定が揃った） |
| **`evt.leash_present`** | `leash_runner`（デバイス接続確認 OK） |
| **`evt.unblocked`** | 各 navigator |
| `evt.register_ok` ／ `evt.register_rejected` | `pin_registrar` |
| `evt.record_broken` | `route_recorder`（記録の連続性が切れた → `IDLE` へ） |
| `evt.check_result` | `opcheck_runner`。`{"item":..., "result":...}` |
| `evt.calib_step_done` ／ `evt.calib_verify_ng` | `calib_runner` |
| `evt.unsaved.set` ／ `evt.unsaved.clear` | 記録を持つ全ノード |

> **`evt.target_reacquired` は作らない**（初版にあったが削除した）。
> 追跡が切れたら `T-FOLLOW-06` で `SELECT` へ落ち、**再開には人の選び直しが要る**。
> 自動で再捕捉して走行に戻す事象を用意すると、既存の
> `require_explicit_target_selection: true`（`leg_tracker_param.yaml`）と正面から衝突し、
> **別人を掴んだまま走り出す経路ができる。**
> 再捕捉したことの**表示**は `/person/targets` の中身で足りる（事象は要らない）。

### 8.3 `fault.*` ／ `hw.*`（`th_state` の内部生成）

| 事象 | 契機 |
| --- | --- |
| `fault.recoverable` ／ `fault.cleared` | `/safety/fault` の `severity == RECOVERABLE` |
| `fault.critical` | `severity == CRITICAL` |
| `hw.estop.press` ／ `hw.estop.release` | `/safety/estop_hw` の立ち上がり／立ち下がり |
| `ui.estop.press` ／ `ui.estop.release` | `/safety/estop_ui`。**`ui.*` だが UI の直接呼び出しではなく `safety_monitor` 経由**（安全チェーンを通す） |

### 8.4 ジョグは「押下／解放」ではなくリースである

**エッジ（`touch` / `release`）で表すと、WiFi の受信ギャップ（実測 0.5〜1.2 s）で
`release` が落ちたとき `jog_active` が永久にラッチする。**

| 項目 | 仕様 |
| --- | --- |
| UI | スティックに触れている間、`ui.jog.hold` を **5 Hz 以上**で送り続ける |
| `th_state` | 最後の受信から `jog_lease_ms` を超えたら `jog_active = false` |
| 制約 | **`jog_lease_ms ≥ /cmd_vel_manual` の twist_mux timeout（1.0 s）。**逆にすると「多重化はまだ手動を流しているのに FSM は解除済み」になる |

---

## 9. ファイル・ディレクトリ

### 9.1 リポジトリ内

| パス | 内容 |
| --- | --- |
| `th_ws/src/th_state/th_state/state_core.py` | **ROS2 非依存の純粋コア** |
| `th_ws/src/th_state/config/transitions.yaml` | **遷移表（`Spec-modes.md` §3.1 の転写）** |
| `th_ws/src/th_state/config/attributes.yaml` | モード別属性表（`Spec-modes.md` §6 の転写） |
| `th_ws/src/th_state/scripts/state_manager.py` | ROS2 ノード（コアを呼ぶだけ） |
| `th_ws/src/th_params/config/registry.yaml` | **全パラメータの唯一の置き場** |
| `th_ws/src/th_params/th_params/derive.py` | (b) の導出式（純粋関数） |
| `th_ws/src/th_params/scripts/params_audit.py` | 起動時監査ノード |
| `th_ws/src/th_safety/src/obstacle_limiter.cpp` | **新設** |
| `th_ws/src/th_safety/config/twist_mux.yaml` | 出力を `/cmd_vel_muxed` へ変更 |
| `th_ws/web_ui/src/screens/S00Connect.jsx` … `S50Settings.jsx` | 15 画面 |
| `th_ws/web_ui/src/shell/Header.jsx` ／ `OperationCard.jsx` ／ `Windows.jsx` | 共通シェル |
| `th_ws/src/th_testing/test/` | 全テスト（既存の置き場を踏襲） |

### 9.2 実行時データ（Docker ワークスペース内）

`Spec-params.md` §6 の「別の位置」を実体化する。**取り違え防止のため物理的に分ける。**

```
/root/th_data/
├── venue/                 試験場内地図（1 枚のみ保持）
│   ├── map.pgm / map.yaml / map.posegraph
│   └── pins.yaml          待機場所ピン・配電盤ピン
├── routes/                教示の経路と地図（経路ごと・新版＋旧版 1 世代）
│   └── <route_id>/
│       ├── current/       route.yaml / path.csv / map.pgm / map.yaml / map.posegraph
│       └── previous/      同じ構成
├── linemap/               ラインマップ（後回し）
├── calib/                 校正の補正値と履歴（3 世代）
│   ├── current.yaml
│   └── history/           001.yaml … 003.yaml
├── generated/             registry.yaml から生成した ROS2 パラメータ YAML
│   ├── <node>.yaml        起動のたび再生成
│   └── twist_mux.yaml     ← manual_joy.timeout の実体はここではなく registry
└── logs/                  開発モードで選択したログ
```

`docker-compose.yml` に `./data:/root/th_data` の bind mount を追加する
（`--rm` の使い捨てコンテナで消えないようにする。`dr_spaam_weights` と同じ流儀）。

---

## 10. 逆引き（完全設計書 → この文書）

| 完全設計書 | ここでの反映 |
| --- | --- |
| `Spec.md` §8（モード 18） | §2 |
| `Spec-modes.md` §2・§3（モードと状態） | §2・§3 |
| `Spec-modes.md` §7（ゾーン） / `F-19` | §4。**3 値化** |
| `Spec-modes.md` §8（ジョグ介入） / `U-4` | §3.1・§8.4。**リース式** |
| `Spec-webui.md` §2（画面 15） | §4 |
| `Spec-webui.md` §3.3（操作カード） / `U-15` | §8.1 |
| `Spec-webui.md` §8.1（表示名） / `U-18` | §2.2 |
| `Spec-safety.md` §1（安全チェーン） / `SD-6` | §6.1 |
| `Spec-safety.md` §3.5（フォルト 2 階級） / `F-20` | §5.1 `FaultStatus.severity` |
| `Spec-params.md` §2〜§5（パラメータ） | §7 |
| `Spec-params.md` §6（保存先） / `C-04`・`F-14` | §9.2 |
