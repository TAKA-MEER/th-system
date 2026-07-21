# 日常運用ガイド

[← README に戻る](../README.md)

毎回の起動チートシートは [README](../README.md#毎回の起動手順) にある。
本書はその詳細と、地図作成・キャリブレーション・追従の使い方・トラブル対処をまとめる。

## 目次

1. [起動手順の詳細](#起動手順の詳細)
2. [SLAM で地図を作る](#slam-で地図を作る)
3. [オドメトリキャリブレーション](#オドメトリキャリブレーション)
4. [追従を使う (FOLLOWING / FOLLOWING_MAPLESS)](#追従を使う)
5. [Web UI (タブレット)](#web-ui-タブレット)
6. [モード早見表 / フォルト対応](#モード早見表--フォルト対応)
7. [トラブルシューティング](#トラブルシューティング)
8. [TBD (今後確定が必要な事項)](#tbd-今後確定が必要な事項)

---

## 起動手順の詳細

前提: 初回セットアップ([setup.md](setup.md))済み。ラズパイの rplidar は systemd で自動起動。

```bash
# 0. (推奨) WSL をクリーンに — 長時間セッション後の DDS 劣化を避ける
wsl --shutdown

# 1. WSL2 (Ubuntu) ターミナルで、th_ws/ にて
docker start th_robot        # 常駐コンテナの起動 (compose 設定を変えた時だけ docker compose up -d)

# 2. コンテナに入って bringup (実機構成: LiDAR はラズパイ配信)
docker exec -it th_robot bash
cd /root/th_ws && source install/setup.bash
ros2 launch th_bringup bringup.launch.py lidar_source:=network use_stub:=false \
  map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml   # 地図なし(SLAMモード)なら map_yaml を省略

# 3. 健全性確認 (フォルトが全て出ていない/CLEARED になっていること)
#    起動直後の [FAULT] は正常 (grace 3 秒 + ESP32 再接続待ち)。1〜2 分で CLEARED になる
grep -aE "FAULT" <ログ>   # launch を別ログに向けている場合
ros2 topic echo /robot/mode --once     # mode: 1 (IDLE) が初期状態
```

その他の起動オプション:

```bash
# 駆動系だけのキーボード操作テスト (LiDAR・安全監視なしの最小構成)
ros2 launch th_bringup esp32_keyboard_test.launch.py

# キーボードテレオペ (bringup/slam と併用)
ros2 launch th_bringup teleop.launch.py            # /cmd_vel_nav 経由 (通常)
ros2 launch th_bringup teleop.launch.py direct:=true   # /cmd_vel 直接 (SLAM 地図作成用)

# 試験員追従スタブを使う場合 (脚検知パイプラインを起動しない)
ros2 launch th_bringup bringup.launch.py lidar_source:=network use_stub:=true
```

> **コンテナを再作成した場合**(`docker compose up -d` で設定変更を反映した後など)は
> `colcon build --symlink-install` のやり直しが必要(setup.md 5 の注意参照)。

---

## SLAM で地図を作る

```bash
# 1. SLAM モードで起動 (bringup の map_yaml なし、または slam.launch.py)
ros2 launch th_bringup slam.launch.py lidar_source:=network

# 2. 別ターミナルでテレオペ (xterm ウィンドウが開く)
ros2 launch th_bringup teleop.launch.py direct:=true

# 3. RViz2 で /map を見ながら部屋全体を走る
#    コツ: 旋回はゆっくり少なめに・直進主体で壁沿いを周回する
#    (クローラーの超信地旋回はスリップでオドメトリ誤差が大きい)

# 4. 保存
ros2 run nav2_map_server map_saver_cli -f /root/th_ws/src/th_bringup/maps/th_map
```

テレオペのキー: `w/s` 直進前後退, `q/e` 超信地旋回, `a/d` 緩旋回前進, `z/c` 緩旋回後退,
`x`/Space 停止, `+/-` 並進速度, `[/]` 旋回速度。

> 地図が「放射状のノイズだらけ」になる場合: 旋回しすぎ(スリップ)か、
> **PC の時刻ズレ**(setup.md 4)か、旋回キャリブレーション未実施が原因。

---

## オドメトリキャリブレーション

目標精度: 直進 2m で ±2cm、旋回 360° で ±5°。

前提: bringup が実機構成(`use_stub:=false`)で起動済みで、`/odom` が配信されていること。
また `linear_calib.py` / `rotation_calib.py` は `/robot/mode` が **IDLE または MANUAL** で
なければ自動的に中断する(FOLLOWING 中などに誤って走行させる事故を防ぐガード)。
必要なら先にモードを切り替える:

```bash
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 5, requester: 'calib'}"   # 5 = MANUAL
```

直進・旋回とも `/cmd_vel_manual` トピック(twist_mux 経由、priority 30)へ publish するため、
estop/fault_lock の安全チェーンの対象になる(直接 `/cmd_vel` へは publish しない — `/cmd_vel` は
twist_mux の出力であり、直接 publish してはいけない不変ルールに従う)。

wheel_radius(車輪半径)と wheel_base(輪距)は補正の反映方法が異なるので注意:

- **wheel_base**: `apply_calib.py` で ROS パラメータとして反映。**再起動不要で即座に**
  cmd_vel 変換・オドメトリ計算に反映され、`th_bringup/config/calib.yaml` にも保存されるため
  次回 bringup 起動時にも自動で読み込まれる。
- **wheel_radius**: ROS 側にパラメータは存在せず、ESP32 ファームウェアのコンパイル時定数
  (`esp32/src/config.h` の `WHEEL_RADIUS_M`)が実際の距離換算を行っている。
  `linear_calib.py` は補正後の定数値を算出・表示するだけで、反映には
  **config.h の書き換え + esptool での再書き込み**が必要(手順は setup.md / 実機メモ参照)。

```bash
# 直進 (床に 2m マーキングを準備)
ros2 run th_calibration linear_calib.py --ros-args -p distance:=2.0
# → 補正後 WHEEL_RADIUS_M が表示される。config.h を書き換えて再書き込みし、
#   再起動後に再計測して検算する。

# 旋回
ros2 run th_calibration rotation_calib.py --ros-args -p turns:=1

# 反映 (wheel_base のみ。th_bringup/config/calib.yaml に保存され、以後 bringup が自動で読む)
ros2 run th_calibration apply_calib.py --ros-args -p wheel_base:=<補正後>
```

wheel_radius(ファーム再書き込みが必要)と wheel_base(ROS 側で完結)は相互に影響するため
**必ず直進→旋回の順**。3 回平均を推奨。

> `rotation_calib.py` の既定 `speed:=0.3`(rad/s)は超信地旋回の摩擦に負けて
> ほぼ回転しない場合がある(その場合オドメトリが目標角度に達せず終了しない)。
> 回らない場合は `-p speed:=0.5` 等に上げてみる(teleop の超信地旋回 spin_speed=0.8 が参考値)。

---

## 追従を使う

モード切替はタブレット UI のボタン、または CLI:

```bash
# FOLLOWING (地図 + Nav2 使用。mode 2)
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 2, requester: 'cli'}"

# FOLLOWING_MAPLESS (地図・Nav2 不要。mode 7)
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 7, requester: 'cli'}"

# IDLE に戻す (mode 1)
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 1, requester: 'cli'}"
```

FOLLOWING_MAPLESS 実機運用の要点(2026-07-11 の走行検証より):

- 追従対象はロボット正面 0.7m 以上に立ってから切り替える
  (stop_distance 未満だと停止のまま、進路上 0.45m 以内に物があると obstacle_ahead で停止)
- 人を見失うと捜索旋回 → 再取得。長時間ロストで PERSON_TRACKER_LOST → 強制 IDLE(安全設計)
- **机・椅子の脚に追跡が乗り移ることがある**(既知の課題。
  `leg_detection_bringup/param/leg_tracker_param.yaml` の再取得パラメータで調整)
- 距離パラメータは `th_bringup/config/planning_params.yaml` の `follow_planner_mapless`。
  現在は狭所向け(stop 0.5 / resume 0.7 / obstacle 0.45)。広い現場では 1.0/1.3/1.0 が目安
- 検知レートの都合で LiDAR は Standard モード推奨(setup.md 6-1)

---

## Web UI (タブレット)

```bash
cd th_ws/web_ui && npm run dev   # → http://<PCのIP>:5173
```

| 操作 | 説明 |
| --- | --- |
| 追従開始 | IDLE → FOLLOWING |
| 軌跡追従(マップ不要) | IDLE/MANUAL → FOLLOWING_MAPLESS |
| 手動操作 | MANUAL + 仮想ジョイスティック |
| 配電盤移動 | MOVING_TO_PANEL + 目的地選択 |
| 緊急停止 | ESTOP へ即時遷移 |

rosbridge(9090)は bringup が自動起動。接続先は既定でページ配信ホスト
(`web_ui/src/hooks/useRosbridge.js`)。MANUAL はハートビート(2Hz)が 1 秒途絶えると
自動で IDLE に落ちる(タブレット切断へのフェイルセーフ)。

---

## モード早見表 / フォルト対応

| モード | 番号 | 状態 |
| --- | --- | --- |
| IDLE | 1 | 静止待機(起動時の初期状態) |
| FOLLOWING | 2 | 追従(地図・Nav2 使用) |
| MOVING_TO_PANEL | 3 | 配電盤へ移動中 |
| AT_PANEL | 4 | 配電盤前作業中 |
| MANUAL | 5 | 手動操作 |
| ESTOP | 6 | 緊急停止(復帰は IDLE 経由のみ) |
| FOLLOWING_MAPLESS | 7 | 追従(地図・Nav2 不要) |

| フォルト | 原因 | 対処 |
| --- | --- | --- |
| `LIDAR_LOST` | /scan 途絶 | ラズパイ側ノード・WiFi・時刻ズレを確認 → [network.md](network.md) |
| `ESP32_DISCONNECTED` | wheel_feedback 途絶 (WiFi/WS) | AP への WiFi 接続・固定IP・portproxy 残骸を確認 → [network.md](network.md) |
| `PERSON_TRACKER_LOST` | /person/status 途絶 | 脚検知パイプライン確認(下記トラブル参照) |

フォルト発生時は twist_mux が即座にモーター出力をゼロにし、mode_manager が IDLE に遷移する。
復帰には明示的なモード切替操作が必要。タイムアウト値は
`th_safety/config/safety_monitor.yaml`(WiFi 実測ジッタ込みで調整済み)。

---

## トラブルシューティング

ネットワーク系(WiFi/AP/DDS/scan 不達)は **[network.md](network.md) の復旧手順**を参照。

### ロボットが動かない

```bash
ros2 topic echo /safety/estop --once      # false であるべき
ros2 topic echo /robot/mode --once        # 動くモード (2/5/7) にいるか
ros2 topic hz /esp32/wheel_feedback       # 10 Hz 前後で来ているか
ros2 topic echo /cmd_vel                  # 指令が出ているか
```

FOLLOWING_MAPLESS で止まったままの場合は follow_planner_mapless のログを見る
(`停止中（理由: person_close/obstacle_ahead/no_pose/no_scan）` が出る)。

### 追従がぎこちない (ハンチング)

```bash
ros2 param set /follow_planner goal_deadzone_m 0.5             # FOLLOWING
ros2 param set /follow_planner distance_hysteresis_m 0.4       # FOLLOWING
ros2 param set /follow_planner_mapless resume_distance 0.9     # FOLLOWING_MAPLESS
```

### LiDAR が誤認識する / 自分のフレームが映る

```bash
# 死角フィルターをランタイム調整 (実測方法は architecture.md)
ros2 param set /lidar_filter blind_angle_ranges \
  "[43.0, 47.0, 133.0, 137.0, 223.0, 227.0, 313.0, 317.0]"
```

### 脚検知 (/person/status) が来ない

```bash
ros2 topic hz /scan_filtered                     # 入力があるか
ros2 topic hz /dr_spaam/dr_spaam_detections      # 検出器 (CPU 推論で 2〜5Hz 程度)
ros2 topic echo /person/status --once            # ブリッジ出力
```

- dr_spaam_ros は稀にセグフォで落ちるが自動再起動する(respawn 設定済み)
- 検出が遅い場合は LiDAR を Standard モードに(setup.md 6-1)
- 脚検知だけを単体で素早く確認したい場合は [architecture.md の person_tracker 節](architecture.md#person_tracker-本番実装human_kenchi-ベース) 参照

### ESP32 が頻繁に再接続する

- **5 分周期の切断→即再接続は「定期リフレッシュ」で正常(仕様)**
- それより頻繁な場合は [network.md](network.md) の「ESP32 が WS に繋がらない」を参照

### Nav2 が経路を計画できない

```bash
# costmap と自己位置を RViz2 で確認
ros2 topic echo /odom | head -20
ros2 run tf2_ros tf2_echo map base_link   # map フレームが無ければ SLAM/AMCL が動いていない
```

map フレームが存在しない場合、まず時刻ズレ(setup.md 4)を疑う。

---

## TBD (今後確定が必要な事項)

- [ ] LiDAR 死角角度の実測値 (`perception_params.yaml`)
- [ ] 配電盤座標の登録 (`panels.yaml`)
- [x] DR-SPAAM 重みファイルの実機への配置・単体動作確認
- [ ] 机・椅子への追跡乗り移り対策 (`leg_tracker_param.yaml` の調整、長時間・実運用環境での検証)
- [ ] IMU 調達後の EKF 切替 (`imu_enabled:=true`)
- [ ] カメラ昇降システムとのインターフェース確定
- [ ] タブレット機種確定後の UI レイアウト調整
- [ ] 広い現場での追従パラメータ再調整 (現在は狭所向け設定)
