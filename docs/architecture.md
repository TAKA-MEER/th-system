# 保守・拡張 技術詳細 (アーキテクチャ)

[← README に戻る](../README.md)


## アーキテクチャ設計方針

本システムは以下の 3 つの原則に基づいて設計されています。

**安全レイヤーの独立性**: `twist_mux` が `/cmd_vel` の最終出力を一元管理し、個々のノードの実装ミスが物理的な動きに影響しないことをアーキテクチャで保証します。`safety_monitor` が `mode_manager` のモード遷移を待たずに `twist_mux` をロックするため、フォルト検知から物理停止までにソフトウェア処理のレイテンシが介在しません。

**テスト可能なコアロジック**: 追従ロジック（`follow_planner_core.py`）を ROS2 非依存の純粋 Python モジュールとして実装しています。これにより ROS2 環境なしでロジックの単体テストが実行でき、アルゴリズムの変更を安全に検証できます。

**パラメータ外部化**: 全ノードの調整値を YAML ファイルで管理し、コード変更なしにチューニングできます。特に追従距離・PID ゲイン・フォルトタイムアウトは現場検証後に頻繁に変更される値であるため、すべてパラメータ化されています。

---

## 速度指令の排他制御（twist_mux）

`/cmd_vel` への速度指令は複数のノードから発行されますが、最終的な出力は常に `twist_mux` が一元管理します。これにより個々のノードが互いの状態を意識する必要がなくなります。

```txt
優先度（高い方が優先）:
  255: /safety/estop      lock   — E-Stop 発動中は全入力を無視してゼロ出力
  254: /safety/fault_lock lock   — LIDAR_LOST/ESP32_DISCONNECTED 検知時も同様
                                  （PERSON_TRACKER_LOST はここには含めない。
                                    走行の物理安全とは無関係なため。下記参照）
   20: /cmd_vel_retreat   topic  — follow_planner からの近接退避指令・person_predictor からの捜索旋回指令（Nav2 を迂回）
   10: /cmd_vel_nav       topic  — Nav2 controller_server の通常出力
```

`retreat` が `nav` より高い優先度を持つため、Nav2 がゴールへの経路を計算し続けていても、退避が必要な瞬間に `follow_planner` が直接 `/cmd_vel_retreat` を発行すれば即座に反映されます。`follow_planner` 側で Nav2 のゴールをキャンセルする必要はありません。

退避が終了した際は `/cmd_vel_retreat` の発行を止めるだけで、`twist_mux` のタイムアウト（0.5 秒）が経過すると自動的に `/cmd_vel_nav` に切り替わります。これが「退避解除」の実装です。

各入力トピックのタイムアウト値変更が必要な場合は `th_safety/config/twist_mux.yaml` を編集します。

---

## 状態管理（mode_manager FSM）

### 遷移ルールの実装

`mode_manager` は `isTransitionAllowed()` 関数で遷移の許可・拒否を判定します。新しいモードを追加する場合はこの関数に遷移元/遷移先のペアを追加するだけで対応できます。

```cpp
// mode_manager.cpp の isTransitionAllowed() を参照
// 例: 新モード "AUTO_PATROL" を追加する場合
case RobotMode::IDLE:
    return to == RobotMode::FOLLOWING ||
           to == RobotMode::MANUAL    ||
           to == RobotMode::AUTO_PATROL;  // ← 追加
```

### 安全設計上の制約（変更禁止）

以下の遷移ルールは安全上の要件であり、変更してはいけません。

- `IDLE → FOLLOWING` および `IDLE → MANUAL` は外部からのサービス呼び出し（明示操作）のみで発生し、自動では遷移しない。ロボットが操作者の意図しないタイミングで動き出すことを防ぐための方針。
- `ESTOP` へはどのモードからも即座に遷移できる。`ESTOP` からは `IDLE` にのみ遷移できる。
- フォルト（`/safety/fault`）は `LIDAR_LOST`・`ESP32_DISCONNECTED` の場合、`FOLLOWING`・`MOVING_TO_PANEL`・`MANUAL`・`AT_PANEL`・`FOLLOWING_MAPLESS`・`SUMMONING` のいずれからでも `IDLE` へ強制遷移させる。`PERSON_TRACKER_LOST` は試験員位置に依存するモード（`FOLLOWING`・`FOLLOWING_MAPLESS`・`SUMMONING`）のみを対象とし、`MANUAL`・`MOVING_TO_PANEL`・`AT_PANEL` は人物データが無関係なため強制遷移させない（VISION.md §5, 2026-07-24 決定）。`IDLE` 中のフォルトはモード変化を起こさない。

### heartbeat によるMANUAL自動解除

`manual_command_handler` はタブレット UI からの `/manual/heartbeat`（`std_msgs/Empty`、2 Hz）を監視します。既定 2.5 秒間（`heartbeat_timeout_sec`）受信がない場合は通信断と判断し、`/mode_manager/set_mode` を呼び出して `IDLE` へ遷移させます。これは想定外の切断（Wi-Fi 瞬断・ブラウザクラッシュ等）に対するフェイルセーフです。タブレットから意図的に `MANUAL` を終了した場合（「追従再開」ボタン押下）は、`FOLLOWING` へ直接遷移します。

（2026-07-24: 実機の WiFi(ESP32 AP)経由では `/scan`・`wheel_feedback`・`/person/status` と同様に heartbeat も 0.5〜1.2 秒程度の受信ギャップが時折発生することが判明したため、1.0 秒から 2.5 秒に緩和した。`safety_monitor.yaml` の同種タイムアウトと揃えている）

---

## 試験員追従ロジック（follow_planner_core / mapless_follow_core）

追従ロジックは2つの独立した実装があり、モードによって使い分けられます。

| | `follow_planner_core.py` | `mapless_follow_core.py` |
| --- | --- | --- |
| 使用モード | `FOLLOWING` / `MOVING_TO_PANEL` | `FOLLOWING_MAPLESS` |
| 目標地点の送出先 | Nav2 (`NavigateToPose`) | 直接 `(v, ω)` を `/cmd_vel_retreat` へ |
| 地図・costmap | 必要（`/local_costmap/costmap` + TF） | 不要（`/odom` TF と `/scan_filtered` のみ） |
| 近接時の挙動 | 地図上の最空きスペース方向へ退避 | 退避せずその場停止・離れたら再開 |

### 内部状態と優先順位（follow_planner_core）

`FollowPlannerCore.update()` は毎制御周期（10 Hz）に呼ばれ、試験員との距離のみに基づいて次の3状態を切り替えます（`followLogic.md` v2 設計）。

```txt
TRACKING（軌跡追従）   d ≥ d_prepare(既定3.0m)
      ↓ d < d_prepare
PREPARE（退避準備）     d_evade(既定2.0m) ≤ d < d_prepare
      ↓ d < d_evade
EVADING（退避）         d < d_evade
```

各状態の詳細は `follow_planner_core.py` の `FollowPlannerCore.update()` を参照してください。

### 状態遷移のヒステリシス（ハンチング防止）

`next_follow_state()` は復帰判定に `distance_hysteresis_m`（既定 0.2m）分の余裕を要求します。例えば `d_prepare=3.0m` の場合、距離が 3.0m を下回ると PREPARE に入りますが、TRACKING へ戻るには `d_prepare + distance_hysteresis_m = 3.2m` 以上必要です（EVADING⇔PREPARE も同様に `d_evade + distance_hysteresis_m` を要求）。現場でハンチングが観測された場合は `distance_hysteresis_m` を大きくしてください。

### 軌跡追従（TRACKING）

試験員の位置履歴（`trail`。ロボットが移動しても意味が変わらないよう絶対座標系=odom系で保持）を `lookback_distance`（既定 1.0m）だけ遡った点をそのまま Nav2 のゴールとして送ります。旧設計にあった通路幅判定・角度オフセット・LiDAR視野角制約は廃止されています。

### 退避方向探索（PREPARE）

退避方向は試験員の歩行方向に依存しません。`find_nearest_open_direction()` が `/local_costmap/costmap` を放射状（`evade_scan_directions`、既定16方向、`evade_scan_max_dist` まで走査）に調べ、`retreat_check_clearance`（既定0.5m）以上の自由空間を確保できる中で最も開けた方向を選びます。この方向は PREPARE 突入時に一度だけ計算され、EVADING に移るまで保持されます。

### 退避走行（EVADING）

PREPARE で決めた方向に `evade_route_length_m`（既定2.0m）先の点をゴールとし、Pure Pursuit（`pure_pursuit_control()`）で `retreat_speed` を上限速度として走行します。

---

## MAP不要軌跡追従ロジック（mapless_follow_core）

`FOLLOWING_MAPLESS` モードでは `MaplessFollowCore.update()` が Nav2 を使わず毎周期直接 `(v, ω)` を計算します。状態は TRACKING/STOPPED の2つのみで、近接時も退避行動は取りません。

```txt
TRACKING（追従中）
      ↓ d < stop_distance(既定1.0m)
STOPPED（停止中・退避せずその場停止）
      ↓ d ≥ resume_distance(既定1.3m)
TRACKING へ復帰
```

- **軌跡追従**: `follow_planner_core.py` の `update_trail`/`get_trail_goal`/`pure_pursuit_control`（絶対座標系での軌跡保持を含む）をそのまま再利用します。
- **進路上障害物チェック**: costmap ではなく `/scan_filtered` の生レンジ値を `is_path_blocked()` で直接走査します。軌跡ゴールへの進行方位を中心に `obstacle_check_half_width_deg`（既定20°）の範囲・`obstacle_check_distance_m`（既定1.0m）未満に有効なレンジ値があれば、試験員との距離に関わらず停止します。
- **フェイルセーフ**: TF（odom→base_link）が未確立、または `/scan_filtered` を一度も受信していない場合は安全側に倒して停止します（costmap 方式の「未受信時は自由とみなす」というフェイルオープンとは逆の設計です。他に障害物安全層が無いためです）。

---

## 安全設計の実装詳細

### フォルト検知のタイムライン

```txt
フォルト発生
  ↓ （check_period_ms 以内、デフォルト 100ms）
safety_monitor が途絶を検知
  ↓ （即時）
/safety/fault を発行
  ↓ （twist_mux がトピックを受信した次の制御周期、通常 10ms 以内）
twist_mux が /cmd_vel をゼロに固定（物理的な動きを停止）
  ↓ （mode_manager が /safety/fault を受信・処理）
mode_manager が IDLE へ遷移
  ↓ （タブレット UI が /robot/mode を受信）
UI にフォルト表示・操作要求
```

`twist_mux` によるモーター停止は `mode_manager` のモード遷移処理を待ちません。これにより、フォルト検知から物理停止までの時間は `check_period_ms`（100ms）+ twist_mux の処理時間（数 ms）のみです。

### ESP32 側の二重フェイルセーフ

ROS2 側の `safety_monitor` に加え、ESP32 ファームウェアにもウォッチドッグが実装されています。

```txt
ROS2 クラッシュ・USB 切断発生
  ↓ （同時に独立して動作）
[ESP32 側] wheel_cmd 受信が WATCHDOG_MS(300ms) 途絶
           → モーター強制停止（ハードウェアレベル）
[ROS2 側]  safety_monitor が wheel_feedback 途絶を esp32_timeout_ms(500ms) で検知
           → /safety/fault 発行 → twist_mux ロック
```

ESP32 ウォッチドッグ（300ms）の方が safety_monitor（500ms）より先に作動する設計です。これにより ROS2 側の処理が間に合わなくても物理停止が保証されます。

**WHEEL_CMD キープアライブ（2026-08-05 追加）**: `esp32_bridge` は `/cmd_vel` のコールバック
受信時だけでなく、最新値を 20Hz のタイマーで ESP32 へ再送し続ける（`esp32_bridge.py` の
`_cb_cmd_vel_keepalive`）。実機ログで、ESP32-PC 間の WiFi ジッタ（`docs/network.md` 記載:
平常時でも 0.5〜1.2 秒）により WHEEL_CMD の到達間隔が 300ms を超え、直進走行中に
ウォッチドッグが誤発動して一瞬停止する事象を確認したため。ROS2/`esp32_bridge` が本当に
クラッシュすればこの再送タイマーごと停止するため、300ms 以内に物理停止する最終保証（上記）
は変わらない。

### E-Stop の集約

物理ボタンとタブレット UI の両方の E-Stop を `safety_monitor` が集約します。

```txt
[物理スイッチ] → GPIO34 → ESP32 → /safety/estop_hw → safety_monitor
                                                              ↓（OR）
[タブレット UI] → WebSocket → /safety/tablet_estop → safety_monitor
                                                              ↓
                                                    /safety/estop
                                                              ↓
                                              twist_mux lock（ゼロ強制）
                                              mode_manager → ESTOP
```

物理スイッチは電気的にモータードライバ電源を遮断するため、ROS2 の処理に依存しないハードウェア的な停止も同時に発動します。

---

## オドメトリとセンサフュージョン

### 差動駆動の運動学

`esp32_bridge` が実装するオドメトリ計算式です。

```txt
入力:
  v_L, v_R = 左右ホイールの実速度 [m/s]（ESP32 から wheel_feedback で取得）
  dt        = 前回更新からの経過時間 [s]

更新:
  v_center = (v_L + v_R) / 2
  omega    = (v_R - v_L) / wheel_base
  x   += v_center * cos(θ + omega * dt / 2) * dt
  y   += v_center * sin(θ + omega * dt / 2) * dt
  θ   += omega * dt
```

クローラーは超信地旋回時にスリップが発生しやすく、エンコーダ単体のオドメトリは誤差が蓄積します。SLAM Toolbox による自己位置補正（スキャンマッチング）がこのドリフトを抑制します。

### キャリブレーション手順の詳細

```txt
目標精度: 直進 2m で誤差 ±2cm 以内、旋回 360° で誤差 ±5° 以内

1. linear_calib.py を 3 回実行して平均を取る
   → 補正後の WHEEL_RADIUS_M(ESP32 ファームウェアのコンパイル時定数)が算出される。
     config.h を書き換えて esptool で再書き込みし、再起動後に再計測して検算する
     (wheel_radius は ROS 側パラメータではなくファーム定数が真の情報源のため)。
2. rotation_calib.py を 3 回実行して平均を取る（CCW/CW 両方向で行うと精度が上がる）
3. apply_calib.py で wheel_base を反映
   （esp32_bridge のランタイムパラメータへ即時反映 + th_bringup/config/calib.yaml に保存）
4. 改めて linear_calib.py / rotation_calib.py で確認 → 精度が目標に達するまで繰り返す

注意:
  wheel_radius(ファーム再書き込みが必要)と wheel_base(ROS 側で完結)は
  互いに影響するため、必ず直進 → 旋回の順でキャリブレーションすること。
  linear_calib.py / rotation_calib.py は /cmd_vel_manual(twist_mux 経由)へ publish し、
  /robot/mode が IDLE/MANUAL でない場合は安全のため自動的に中断する。
```

### IMU (DSR1603 / BNO055) 追加

超信地旋回時のクローラースリップによる yaw ドリフトを抑えるため、ダイセン電子工業製 9軸デジタルコンパス **DSR1603**（センサIC: Bosch **BNO055**、3軸加速度+3軸ジャイロ+3軸地磁気のオンチップセンサフュージョン）を ESP32 に追加する。

**ハードウェア**

| 項目 | 値 |
| --- | --- |
| センサ | BNO055（I2C, 400kHz, address = 0x28） |
| 接続先 | ESP32 GPIO21 (SDA) / GPIO22 (SCL)（`config.h` の `IMU_SDA`/`IMU_SCL`） |
| 電源 | **3.3V** で駆動する（DSR1603 は 3.3〜5.0V 対応だが、外部 I2C バスのプルアップがモジュール供給電圧にプルアップされる回路のため、5V 駆動だと ESP32/ラズパイの非5V-トレラントな GPIO を破損しうる。必ず 3.3V 系統から給電する） |
| コネクタ | XH-4pin（VDD/SCL/SDA/GND）。付属ケーブルから ESP32 の該当 GPIO + 3.3V + GND に配線 |

**プロトコル**

ESP32 ⇔ esp32_bridge は WebSocket バイナリフレーム通信（`th_ws/esp32/src/ws_link.h` ⇔ `th_esp32_bridge/th_esp32_bridge/ws_protocol.py`）。`IMU_DATA (0x04)` フレームを ESP32 → bridge 方向に追加し、クォータニオン(qw,qx,qy,qz)・角速度(wx,wy,wz)・線形加速度(ax,ay,az、重力除去済み)・キャリブレーション状態(sys/gyro/accel/magを2bitずつパックした1byte)を毎制御周期(100ms, 10Hz)送信する。BNO055 自体は最大100Hzサンプリングだが、EKF・オドメトリ更新も10Hzのため既存の制御ループに相乗りさせており、独立タイマーは追加していない。

bridge 側は `/esp32/imu_data`（`sensor_msgs/Imu`, frame_id=`imu_link`）と `/esp32/imu_calib_status`（`std_msgs/UInt8`）を発行する。

**キャリブレーション**

BNO055 の地磁気センサはロボットごとに8の字運動でのキャリブレーションが必要（DSR1603 マニュアル記載: 上下左右に8の字を描くように2回転）。`ros2 run th_calibration imu_calib_check.py` で `/esp32/imu_calib_status` を監視し、sys/gyro/accel/mag が全て 3(Fully calibrated) になるまで操作案内を表示する。

**有効化手順**

```bash
# 1. ESP32 ファームウェアを IMU 対応でビルド・書き込み（config.h の IMU_SDA/IMU_SCL は既定で有効）
#    DSR1603 未接続の個体でも Imu::init() が失敗を検出し、IMU_DATA 送信をスキップして起動する

# 2. ESP32 が /esp32/imu_data を発行することを確認
ros2 topic echo /esp32/imu_data

# 3. 8の字キャリブレーションを実施
ros2 run th_calibration imu_calib_check.py

# 4. EKF を IMU 入力有効に切替えて起動
#    imu_enabled:=true で ekf_params.yaml（imu0込み）、false（既定）で
#    ekf_params_no_imu.yaml（エンコーダのみ）を選択する
ros2 launch th_bringup bringup.launch.py imu_enabled:=true

# 5. EKF のチューニング
#    robot_localization のドキュメントを参照し
#    ekf_params.yaml の process_noise_covariance を実機データで調整
```

### 直進ドリフト補正 (2026-07-25 追加)

直進指令中(左右目標速度が等しい)に一方向へ逸れるクセ(タイヤ径公差・摩擦差等)を補正する機能。`th_ws/esp32/src/main.cpp` の `computeDriftCorrection()` が制御周期ごとに左右輪の目標速度へ小さな差動バイアスを加える(`rampLeft`/`rampRight` を PID へ渡す直前に加算するため、前進速度の合計には影響しない)。

- **基本方式(IMU検出時)**: 実測ヨーレート(`wz`, 目標=0)を使った PI 制御。定数は `config.h` の `DRIFT_KP_YAW`/`DRIFT_KI_YAW`/`DRIFT_ITERM_MAX_MPS`/`DRIFT_CORRECTION_MAX_MPS`。
- **フォールバック(IMU未検出時)**: `DRIFT_TRIM_MPS` の固定トリム値(既定 0.0 = 補正なし)。実機で直進走行させズレを見ながら調整する。
- **適用条件**: `|targetLeft - targetRight| < DRIFT_STRAIGHT_THRESHOLD_MPS` の間のみ適用。旋回指令中は無効化し積分もリセットする(意図した旋回に補正が干渉しないようにするため)。E-Stop/ウォッチドッグ発動時・WS切断時も積分をリセットする。

**⚠️ 符号の実機検証が必須**: BNO055 の `wz` の符号は実装向き・軸割り当てに依存し、コードだけからは断定できない。実機で直進コマンドを送り、ズレが改善するかを目視確認すること。悪化する場合は `config.h` の `DRIFT_IMU_SIGN` を反転(`+1.0f` ⇔ `-1.0f`)して再検証する。

### 車輪速度の指令/実測比較 (`/esp32/wheel_cmd_speed`, WebUI速度表示カード, 2026-07-25 追加)

`esp32_bridge` は `/cmd_vel` を差動駆動変換した左右目標速度(ESP32 へ `WHEEL_CMD` で送る値と同じ)を `/esp32/wheel_cmd_speed`(`th_system_msgs/WheelFeedback` 型を指令値側に再利用)として発行する。WebUI の「車輪速度」カード(`web_ui/src/WheelSpeedView.jsx`)がこれと実測値 `/esp32/wheel_feedback` を左右輪ごとに直近15秒の時系列グラフで重畳表示し、PID の追従遅れ・定常偏差・振動を目視で確認できるようにしている。PID ゲイン自体(`config.h` の `PID_KP_*`/`PID_KI_*`/`PID_KD_*`)は現状コンパイル時定数のままで、WebUI からのライブ調整は未対応(将来検討)。

`imu_enabled` は DSR1603 未装着の機体を壊さないようデフォルト `false`（エンコーダのみ）。装着・キャリブレーション済みの機体でのみ `true` を指定すること。

---

## LiDAR 死角の実測と更新

20mm 角アルミパイプ 4 本による死角は取り付け位置・角度によって変わります。現場環境で正確に実測し、パラメータを更新してください。

```bash
# 1. ロボットを静止させ RViz2 で /scan を表示
ros2 run rviz2 rviz2
# LaserScan を追加し Fixed Frame を "laser_link" に設定

# 2. アルミ角柱が映る角度帯を読み取る
#    RViz2 の "Measurement" ツールで角度を計測
#    または:
ros2 run tf2_ros tf2_echo base_link laser_link  # フレームオフセット確認

# 3. perception_params.yaml を更新
#    例: 角柱が 43°〜47°, 133°〜137°, 223°〜227°, 313°〜317° の場合
```

```yaml
# src/th_bringup/config/perception_params.yaml
lidar_filter:
  ros__parameters:
    blind_angle_ranges:
      - 43.0   # 右前 開始
      - 47.0   # 右前 終了
      - 133.0  # 右後 開始
      - 137.0  # 右後 終了
      - 223.0  # 左後 開始
      - 227.0  # 左後 終了
      - 313.0  # 左前 開始
      - 317.0  # 左前 終了
```

```bash
# 4. 反映（ランタイムで変更も可能）
ros2 param set /lidar_filter blind_angle_ranges "[43.0, 47.0, 133.0, 137.0, 223.0, 227.0, 313.0, 317.0]"
```

---

## 配電盤座標の登録

地図作成後に RViz2 で各配電盤の前面座標（ロボットが停止する位置）を取得し、`panels.yaml` に登録します。

```bash
# RViz2 で "2D Pose Estimate" ツールを使い配電盤前に位置を指定
# → TF ツールで map 座標系の値を読み取る
ros2 topic echo /amcl_pose   # または
ros2 run tf2_ros tf2_echo map base_link
```

```yaml
# src/th_bringup/config/panels.yaml
panels:
  - id: "panel_01"
    name: "第1配電盤"
    x: 3.42        # 実測値を入力
    y: -1.15
    yaw: 1.5708    # 配電盤に向く方向 (rad)
```

`yaw` の値: 配電盤が北壁なら 0、東壁なら -1.5708（-π/2）、南壁なら 3.1416（π）、西壁なら 1.5708（π/2）。

---

## person_tracker 本番実装（human_kenchi ベース）

`person_tracker` の本番実装には
[`TAKA-MEER/human_kenchi`](https://github.com/TAKA-MEER/human_kenchi)（`sobits_follower` から
2D-LiDAR のみの脚検出・追跡部分を抜き出したワークスペース）を採用している。
`use_stub:=false` で以下のパイプラインが起動する。

```txt
/scan_filtered (死角マスク済み)
  → dr_spaam_ros (DR-SPAAM 脚検出, human_kenchi/2d_lidar_person_detection)
  → PersonTracker (leg モード, human_kenchi/multiple_sensor_person_tracking)
  → sobits_follower/multiple_sensor_person_tracking/following_position
  → person_tracker_bridge.py (th_perception)
  → /person/status (th_system_msgs/PersonStatus)
```

`person_tracker_bridge.py` が `following_position.status`
（`0=NO_EXISTS` / `1=EXISTS_LEG`）を `PersonStatus.is_lost` に変換する薄い変換ノード。
`th_ws/src/th_perception/scripts/person_tracker_bridge.py` を参照。

human_kenchi 自体の3パッケージ（`multiple_observation_kalman_filter` /
`multiple_sensor_person_tracking` / `leg_detection_bringup`）は
`th_ws/src/` に直接コミットしている（vendoring）。**human_kenchi はプライベートリポジトリ**
のため Dockerfile から `git clone` できず（認証情報が必要）、`sllidar_ros2` と同じ
外部依存パターンが使えなかったための対応。upstream の更新を取り込む場合は、
アクセス権のあるアカウントで手動 clone し、該当3パッケージのディレクトリを
上書きコミットすること。

DR-SPAAM（`TeamSOBITS/2d_lidar_person_detection`, 公開リポジトリ）は引き続き Dockerfile で
`git clone` して colcon ビルドに含めている（`sllidar_ros2` と同じパターン）。

### DR-SPAAM 重みファイルの配置（初回のみ）

DR-SPAAM の学習済み重み（`ckpt_jrdb_ann_ft_dr_spaam_e20.pth`, 実測 約 30 MB。配布元
フォルダには他モデルの重みも含めて複数ファイルがあるが、`weight_file` パラメータの
既定値であるこのファイルだけあれば動く）は配布元（Google Drive）の都合上 Docker
イメージには含めていない。以下の手順で手動配置する。

1. [重みファイルをダウンロード](https://drive.google.com/drive/folders/1Wl2nC8lJ6s9NI1xtWwmxeAUnuxDiiM4W)
2. リポジトリルートの `th_ws/dr_spaam_weights/` に置く（`.gitignore` 対象、コミット不要）
3. `docker-compose.yml` の bind mount によりコンテナ内
   `/root/th_ws/install/dr_spaam_ros/share/dr_spaam_ros/weights/` に自動反映される
   （`docker compose run --rm` の使い捨てコンテナでも消えない。実機検証で
   このマウント先パスが正しいことを確認済み）

> **既知の問題（対応済み）**: 配布されているチェックポイントは GPU (CUDA) 保存の
> ため、CPU 専用機（`use_gpu:false`）では素の `torch.load` が
> `Attempting to deserialize object on a CUDA device` で失敗する。Dockerfile 内で
> `dr_spaam/detector.py` に `sed` で `map_location` を明示するパッチを当てて回避済み
> （upstream 未対応のため、DR-SPAAM を再取得・再ビルドする際は要再適用）。

### 動作確認

```bash
ros2 launch th_bringup bringup.launch.py \
  map_yaml:=<地図パス> \
  use_stub:=false

# パイプライン各段の確認
ros2 topic hz /dr_spaam/dr_spaam_detections
ros2 topic echo sobits_follower/multiple_sensor_person_tracking/following_position
ros2 topic echo /person/status    # is_lost が適切に切り替わるか
ros2 topic hz /person/status      # 10 Hz 以上発行されているか
```

`use_stub:=true` の場合は上記パイプラインは一切起動せず、`person_tracker_stub.py` のみが
`/person/status` を発行する。

### person_tracker（脚検知）だけを素早く動作確認する

`bringup.launch.py` は Nav2 や ESP32 ブリッジ等も一括起動するため、脚検知パイプライン
だけを素早く確認したい場合は以下を個別に起動する（コンテナ内・実機 LiDAR 直結時の例。
ネットワーク LiDAR の場合は 1. を省略しラズパイ側の /scan を使う）:

```bash
source /opt/ros/humble/setup.bash && source /root/th_ws/install/setup.bash

# 1. LiDAR (直結時のみ)
ros2 run sllidar_ros2 sllidar_node --ros-args   -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000   -p frame_id:=laser_link -p angle_compensate:=true -p scan_mode:=Standard &

# 2. 死角フィルタ
ros2 run th_perception lidar_filter.py &

# 3. DR-SPAAM + PersonTracker（単一LiDARなのでTF不要、target_frame=laser_link）
ros2 launch leg_detection_bringup leg_detection.launch.py   scan_topic:=/scan_filtered target_frame:=laser_link   scan_frame:=laser_link odom_frame:=laser_link   use_rviz:=false autostart:=true &

# 4. /person/status へのブリッジ
ros2 run th_perception person_tracker_bridge.py &

# 5. 確認
ros2 topic hz /dr_spaam/dr_spaam_detections
ros2 topic echo /person/status --once

# 6. 可視化: rviz2 で LaserScan(/scan_filtered)・PoseArray(/dr_spaam/dr_spaam_detections)
#    を追加、Fixed Frame は laser_link
```

`target_frame:=base_link`（本番と同じ設定）で確認したい場合は `robot_state_publisher` 等で
`base_link → laser_link` の TF を先に流しておくこと。

### チューニング

障害物の陰に隠れた際の再取得挙動は `th_ws/src/leg_detection_bringup/param/leg_tracker_param.yaml`
で調整する。主要パラメータは human_kenchi の README を参照。LiDAR 死角（アルミ角柱）による誤検出が
出る場合は `perception_params.yaml` の `blind_angle_ranges` を先に見直すこと
（DR-SPAAM の入力は `/scan_filtered` のため死角マスクが有効）。

---

## カメラ昇降システムとの連携（TBD）

現状は `AT_PANEL` 到着通知（`/panel_navigator/arrived`）のみ実装しています。カメラ昇降システム側の設計が確定したら以下を追加実装します。

| 追加が必要な実装 | 対応ノード | 優先度 |
| --- | --- | --- |
| `/panel_navigator/arrived` のメッセージ内容確定 | `panel_navigator.py` | 高 |
| 昇降完了通知の受信 → `complete_inspection` サービス呼び出し | `panel_navigator.py` | 高 |
| `AT_PANEL → MANUAL` 中断時の同期確認（遷移前 OR 非同期通知の選択） | `mode_manager.cpp` + `panel_navigator.py` | 中 |
| `AT_PANEL` 中の近接警告（近接検知のみ、移動なし） | `safety_monitor.cpp` または新規ノード | 中 |

---

## 新しいモードの追加方法

実例: `FOLLOWING_MAPLESS`（MAP不要の軌跡追従モード）を追加した際の手順です。同じ手順で「自動巡回モード（AUTO_PATROL）」等を追加できます。

```txt
1. th_system_msgs/msg/RobotMode.msg に定数を追加
   uint8 AUTO_PATROL = 8   # FOLLOWING_MAPLESS(7) の次の空き番号を使う

2. mode_manager.cpp の isTransitionAllowed() に遷移ルールを追加
   （FOLLOWING_MAPLESS の実装例: IDLE/MANUAL からのみ遷移可、
    MOVING_TO_PANEL 等の地図前提の遷移は含めない）
   case RobotMode::IDLE:
     return to == RobotMode::FOLLOWING ||
            to == RobotMode::AUTO_PATROL;
   sub_fault_ のフォルト強制 IDLE 判定にも新モードを追加すること（安全上必須）。

3. 新ノード auto_patrol.py を th_planning/scripts/ に追加
   /robot/mode を購読し AUTO_PATROL 中のみ動作する
   （既存ノードは無条件起動のまま常時併存させ、RobotMode の排他性で
    衝突を防ぐ。follow_planner.py / follow_planner_mapless.py が同じ
    パターン）

4. th_planning/CMakeLists.txt の install(PROGRAMS ...) に追加
   th_bringup/launch/bringup.launch.py と gazebo.launch.py に
   ノードエントリを追加

5. th_testing/test/test_mode_transitions.py に遷移テストを追加

6. web_ui/src/App.jsx（MODE 定数・ボタン・modeColor）と
   web_ui/src/hooks/useRosbridge.js（MODE_NAMES）に追加
```

新しいモードは常に「ESTOP からは IDLE のみ経由で復帰」「IDLE への安全側遷移を持つ」「フォルト発生時は IDLE へ強制遷移する」という設計方針に従ってください。

---

## WebUI 設定パネル（パラメータ調整）

VISION.md §6 の完成形を実装したもの。タブレット WebUI（`web_ui/src/SettingsPanel.jsx`）から
`follow_planner_mapless` の数値パラメータと `lidar_filter.blind_angle_ranges` を確認・変更できる。

### 構成

```
[SettingsPanel.jsx]
  │ getTunableParams()  ─── rcl_interfaces/GetParameters を対象ノードへ直接呼び出し（読み取り専用）
  │                          /follow_planner_mapless/get_parameters, /lidar_filter/get_parameters
  │
  │ applyTunableParam() ─── th_system_msgs/SetTunableParams
  │ saveTunableParams() ─── th_system_msgs/SaveTunableParams
  └───────────────────────► /config_manager/{set,save}_tunable_params
                                       │
                              [config_manager ノード]  (th_config_manager パッケージ)
                              - /robot/mode を購読。IDLE/MANUAL 以外は拒否
                              - set: 対象ノードの標準 set_parameters へフォワード
                              - save: 対象ノードの get_parameters で現在値取得
                                → yaml_writer.update_ros_params_yaml() で YAML へ書き戻し
```

- rosbridge は `rosbridge_websocket` 単体起動で **rosapi は起動していない**ため、
  `ROSLIB.Param`（rosapi 依存）ではなく `rcl_interfaces/srv/{Get,Set}Parameters` を
  素の `ROSLIB.Service` で直接呼んでいる（`web_ui/src/hooks/useRosbridge.js` の
  `getTunableParams`/`applyTunableParam`/`saveTunableParams`）。
- **実行時反映と YAML 保存は別操作**。`applyTunableParam` は対象ノードへ即座に反映するが
  再起動で失われる。`saveTunableParams` は対象ノードの現在値を取得して YAML に書き戻す
  （設定パネルの「YAML に保存」ボタン）。
- `follow_planner_mapless.py` と `lidar_filter.py` は起動時に一度だけパラメータを読み、
  内部状態（`MaplessFollowParams` データクラス / `_blind_ranges`）にキャッシュしている。
  そのため両ノードには `add_on_set_parameters_callback` を追加し、`set_parameters` が
  呼ばれた際に内部状態を再構築するようにしてある。**新しいノードをチューニング対象に
  追加する場合、同様のコールバックが無いとライブ反映が機能しない**点に注意。
- YAML への書き戻しは `ruamel.yaml` の round-trip モードを使い、既存のコメント・
  キー順序を保持する（`th_config_manager/th_config_manager/yaml_writer.py`）。
  インデント設定 `yaml.indent(mapping=2, sequence=4, offset=2)` は
  `planning_params.yaml` / `perception_params.yaml` の実際の書式に合わせて検証済み。
- 変更は `IDLE` / `MANUAL` モード中のみ許可する。`SettingsPanel.jsx` は該当モード以外で
  入力を disabled にするが、`config_manager` ノード側でも `/robot/mode` を見て同じ判定を
  行う（UI の見た目だけに頼らないサーバー側の安全ガード）。

### 新しいチューニング可能パラメータを追加する手順

```txt
1. th_config_manager/th_config_manager/tunable_targets.py の TUNABLE_TARGETS に追記
   （対象ノード名・YAML パッケージ/パス・ros__parameters のブロックキー・パラメータ名）

2. 対象ノードに add_on_set_parameters_callback が無ければ追加する
   （follow_planner_mapless.py / lidar_filter.py の実装を参照。パラメータが
    起動時に一度だけ内部状態へコピーされている場合は必須）

3. web_ui/src/SettingsPanel.jsx にフォーム項目を追加
   （MAPLESS_FIELDS 等のフィールド定義配列にラベル・単位・入力レンジを追記）
```

対象拡大（`follow_planner`・`person_predictor`・Nav2 パラメータ・`panels.yaml` 等）は
VISION.md §7 の未確定事項を参照。

---

## パラメータチューニングガイド

上記の WebUI 設定パネルで調整できるパラメータ（follow_planner_mapless の数値パラメータ全数、
lidar_filter.blind_angle_ranges）は、以下の CLI 手順の代わりにタブレットから直接変更できる。

### 追従ロジック — FOLLOWING（planning_params.yaml の `follow_planner`）

```txt
lookback_distance: 1.0m
  → 追従がロボットに近づきすぎる場合は大きく（1.5m 等）
  → 追従の反応が鈍い場合は小さく（0.7m 等）

d_prepare: 3.0m
d_evade: 2.0m
distance_hysteresis_m: 0.2m
  → d_prepare/d_evade の差が PREPARE 状態の距離帯の広さ
  → distance_hysteresis_m がハンチング防止幅。ハンチングが観測されたら大きく（0.4m 等）
  → 退避が早すぎる/遅すぎる場合は d_evade を調整

evade_scan_directions: 16
evade_scan_max_dist: 3.0m
retreat_check_clearance: 0.5m
  → 退避方向が不自然な場合は evade_scan_directions を増やして分解能を上げる
  → 狭所で退避不可（stop）が頻発する場合は retreat_check_clearance を小さく

retreat_speed: 0.15 m/s
  → 退避が遅すぎる/速すぎる場合に調整（通常追従速度の50〜70%が目安）
```

### MAP不要追従ロジック — FOLLOWING_MAPLESS（planning_params.yaml の `follow_planner_mapless`）

```txt
lookback_distance: 1.0m
  → follow_planner と同様の目安

stop_distance: 1.0m
resume_distance: 1.3m
  → 差（0.3m）がハンチング防止幅。頻繁に停止/再開を繰り返す場合は差を大きく
  → 停止が遅すぎる場合は stop_distance を大きく

obstacle_check_distance_m: 1.0m
obstacle_check_half_width_deg: 20.0°
  → 障害物での停止が頻発する場合（誤検知）は距離・角度幅を小さく
  → 停止が遅い（間に合わない）場合は obstacle_check_distance_m を大きく

v_max: 0.3 m/s
  → 走行速度の上限。現場の安全要件に応じて調整

max_linear_accel_mps2 / max_linear_decel_mps2
max_angular_accel_rad_s2
  → 急加減速・急旋回で機体が揺れる場合は小さく(滑らかになるが反応は遅くなる)
  → 追従が遅れて感じる場合は大きく(ただし機体の実際の加減速性能を超えないこと)
```

### フォルト検知タイムアウト（safety_monitor.yaml）

```txt
lidar_timeout_ms: 500ms
esp32_timeout_ms: 500ms
  → 電磁ノイズ・ジッタによる誤検知が多い場合は大きく（800ms 等）
  → 故障への反応が遅い場合は小さく（300ms 等）
  → ESP32 ウォッチドッグ(300ms)よりは必ず大きく設定すること

startup_grace_sec: 3s（シミュレーションでは safety_monitor_sim.yaml で 7s に上書き）
  → 起動直後のタイムアウト誤検知を抑制する猶予時間
  → Gazebo の spawn_delay(4.5s) + 初期化時間を考慮してシミュレーションでは大きく設定する
```

### ESP32 PID ゲイン（config.h）

```txt
初期値: Kp=80-100, Ki=30, Kd=8
チューニング手順:
  1. Ki=0, Kd=0 で Kp のみ調整 → 振動しない最大値を探す
  2. Ki を少しずつ増やして定常偏差を減らす
  3. Kd を増やして過渡応答を改善する
  4. 左右で別々に調整（旋回精度に影響する）
```

---

## トラブルシューティング

### ロボットが動かない

```bash
# 1. E-Stop が解除されているか確認
ros2 topic echo /safety/estop   # false であるべき

# 2. twist_mux の出力を確認
ros2 topic echo /cmd_vel        # ゼロでないか

# 3. ESP32 との通信を確認
ros2 topic hz /esp32/wheel_feedback  # 10 Hz 程度で来ているか

# 4. モードを確認
ros2 topic echo /robot/mode     # FOLLOWING/MANUAL であるべき
```

### 追従がぎこちない（ハンチング）

```bash
# デッドゾーンを広くする（FOLLOWING）
ros2 param set /follow_planner goal_deadzone_m 0.5

# 状態遷移のヒステリシス幅を広くする（FOLLOWING）
ros2 param set /follow_planner distance_hysteresis_m 0.4

# 停止/再開のヒステリシス幅を広くする（FOLLOWING_MAPLESS）
ros2 param set /follow_planner_mapless resume_distance 1.6
```

### LiDAR が誤認識する

```bash
# 死角フィルターのパラメータをランタイムで調整
ros2 param set /lidar_filter blind_angle_ranges \
  "[43.0, 47.0, 133.0, 137.0, 223.0, 227.0, 313.0, 317.0]"

# /scan と /scan_filtered を同時に RViz2 で比較
```

### ESP32 が頻繁に再接続する

```bash
# esp32_bridge の WebSocket 接続ログを確認 (接続/切断イベントが出力される)
ros2 topic echo /rosout | grep esp32_bridge

# ESP32 側のシリアルモニタで WiFi/WS 状態を確認
# (現行構成では ESP32 はラズパイに USB 接続されているため、ラズパイ側で確認する)
pio device monitor    # または ラズパイ上で /dev/ttyUSB1 を 115200 で読む

# ★5 分周期の切断→即再接続は「定期リフレッシュ」で正常動作(仕様)

# AP 構成の場合: PC の固定IP (192.168.4.50) と portproxy (8766→8765) が
# wifi_credentials.h の WS_SERVER_HOST/PORT と一致しているか確認
# STA 構成の場合: ホットスポットの SSID/パスワードが一致しているか確認
# PC 側ファイアウォールが ws_port をブロックしていないか確認

# ウォッチドッグタイムアウトを確認
# config.h: WATCHDOG_MS が通信周期より十分大きいか確認
```

### Nav2 が経路を計画できない

```bash
# costmap を RViz2 で確認
ros2 run rviz2 rviz2
# /local_costmap/costmap と /global_costmap/costmap を表示

# ロボットの位置推定を確認
ros2 topic echo /odom | head -20

# SLAM のマッチング状態確認
ros2 topic echo /slam_toolbox/scan_matched
```
