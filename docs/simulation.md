# Gazebo シミュレーション

[← README に戻る](../README.md)


## 概要

実機（ESP32・RPLIDAR S1）がなくても、Gazebo Classic 上で全追従ロジックを視覚的に確認できます。
`sim:=true/false` の引数一つで実機とシミュレーションを切り替えられます。

```txt
実機モード:   sllidar_node + esp32_bridge(WebSocketサーバー) が起動
シミュレーション: Gazebo が /odom・/scan を発行し上記3ノードは不要
共通:         mode_manager・follow_planner・Nav2・rosbridge 等は同一ノードが動く
```

---

## 起動コマンド早見表

```bash
# 1. SLAM で地図を作りながらシミュレーション（最初はこれ）
ros2 launch th_bringup gazebo.launch.py

# 2. 既存地図でナビゲーション＋追従シミュレーション
ros2 launch th_bringup gazebo.launch.py \
  slam:=false map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml

# 3. walk.dae がない環境（cylinder モデルで代替）
ros2 launch th_bringup gazebo.launch.py \
  world:=$(ros2 pkg prefix th_bringup)/share/th_bringup/worlds/panel_room_no_actor.world

# 4. URDF だけ RViz2 で確認
ros2 launch th_description display.launch.py

# 5. キーボード手動操作（別ターミナルで実行）
ros2 launch th_bringup teleop.launch.py           # /cmd_vel_nav 経由
ros2 launch th_bringup teleop.launch.py direct:=true  # /cmd_vel 直接（SLAM 用）

# 6. 実機モード（Gazebo なし）
ros2 launch th_bringup gazebo.launch.py sim:=false \
  map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml
```

---

## Step by Step：初回シミュレーション実行手順

### Step 1: Docker コンテナで起動

**Linux ホストの場合:**

```bash
xhost +local:docker            # X11 転送を許可
docker compose run --rm th_robot bash
```

**Windows (WSL2) の場合:**

```bash
# VcXsrv や X410 などの X サーバーを起動してから
export DISPLAY=:0
docker compose run --rm th_robot bash
```

> **補足:** 現行構成では LiDAR はラズパイ側に接続するため、`docker-compose.yml` の
> `devices:` (`/dev/lidar`) はコメントアウト済み。PC に LiDAR を USB 直結する場合のみ
> コメントを解除する。

### Step 2: ビルド

```bash
# コンテナ内
cd /root/th_ws
colcon build --symlink-install   # 初回または C++ 変更時
source install/setup.bash
```

> Dockerfile 内でビルド済みのため、通常は不要。Python スクリプトの変更はシンボリックリンクで即時反映される。C++ パッケージ (`th_safety` など) を変更した場合のみ再ビルドが必要。

### Step 3: SLAM で地図を作成

```bash
# ターミナル 1: シミュレーション起動（初回は SLAM モード）
ros2 launch th_bringup gazebo.launch.py

# ターミナル 2: クローラーテレオペ（別 xterm ウィンドウが開く）
ros2 launch th_bringup teleop.launch.py direct:=true
```

xterm ウィンドウをクリックしてフォーカスを与えてからキー操作:

| キー | 動作 |
| --- | --- |
| `w` | 直進前進 |
| `s` | 直進後退 |
| `q` / `e` | 超信地旋回 左 / 右（その場旋回） |
| `a` / `d` | 緩旋回前進 左 / 右（円弧走行） |
| `z` / `c` | 緩旋回後退 左 / 右（円弧走行） |
| `x` / Space | 停止 |
| `+` / `-` | 並進速度を ±0.05 m/s |
| `[` / `]` | 旋回速度を ±0.1 rad/s |

RViz2 で `/map` に地図が表示されたら部屋全体を走り回り、地図を保存:

```bash
# ターミナル 3
ros2 run nav2_map_server map_saver_cli \
  -f /root/th_ws/src/th_bringup/maps/th_map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10000.0
```

### Step 4: 追従動作の確認

```bash
# ターミナル 1: 地図ありで起動（AMCL 自己位置推定）
ros2 launch th_bringup gazebo.launch.py \
  slam:=false map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml
```

起動から約 10 秒後（Nav2 が active になってから）、モードを FOLLOWING に切替:

```bash
# ターミナル 2
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 2, requester: 'cli'}"
```

Gazebo ウィンドウで Inspector（試験員役）が移動し、ロボットが追従することを確認する。Inspector は Gazebo 起動 1 秒後から配電盤前を往復し始める。

> FOLLOWING モード中に手動操作したい場合は `requested_mode: 5` (MANUAL) に切替え、別ターミナルで `ros2 launch th_bringup teleop.launch.py` を実行する。

### Step 5: MAP不要の軌跡追従モード（FOLLOWING_MAPLESS）の確認

`follow_planner_mapless.py` は Nav2・SLAM 占有格子を一切使わず、`/odom`（ホイールオドメトリ）と `/scan_filtered`（死角マスク済み生 LiDAR スキャン）だけで動く追従モード。地図読み込み・AMCL・Nav2 のゴール計画が不要なため、**地図を作る前でも検証できる**。

`RobotMode.msg` に `FOLLOWING_MAPLESS` 定数を追加しているため、検証前に **`th_system_msgs` を含むフルビルドが必須**:

```bash
cd /root/th_ws
colcon build --symlink-install
source install/setup.bash
```

起動（SLAM/地図の有無に関わらず動作確認できる。Nav2 自体はこの起動コマンドで引き続き立ち上がるが、`follow_planner_mapless` はそれを一切参照しない）:

```bash
ros2 launch th_bringup gazebo.launch.py
```

FOLLOWING_MAPLESS へ切替（CLI から）:

```bash
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 7, requester: 'cli'}"
```

タブレット UI から切替える場合は「軌跡追従(マップ不要)」ボタンを押す。

確認項目:

| 確認項目 | 方法 | 期待動作 |
| --- | --- | --- |
| 軌跡追従 | Inspector（または `person_mover.py --pattern:=patrol`）を移動させる | Nav2 にゴールを送らず、`/cmd_vel_retreat` に直接速度指令が出て試験員の軌跡を遡って追従する |
| 接近時は退避せず停止 | `person_mover.py --ros-args -p pattern:=approach -p approach_dist:=0.6` 等で `stop_distance`（既定 1.0m）以内に接近させる | 後退・旋回せず、その場で停止する（`/cmd_vel_retreat` がゼロ） |
| 停止後の追従再開 | 接近させた後、`resume_distance`（既定 1.3m）以上離す | 自動的に追従を再開する（オペレータ操作不要） |
| 進路上障害物での停止 | ロボットの進行方位上（軌跡ゴールへのベアリング方向）に障害物モデルを置く | 試験員との距離に関わらずその場で停止する |
| フェイルセーフ | `/scan_filtered` を止める（`lidar_filter` ノードを kill 等） | スキャン未受信になった時点で停止する（安全側） |

停止理由はログで確認できる:

```bash
ros2 topic echo /rosout | grep follow_planner_mapless
# "停止中（理由: person_close/obstacle_ahead/no_pose/no_scan）" または "追従再開" が出力される
```

`/cmd_vel_retreat` を直接確認する場合:

```bash
ros2 topic echo /cmd_vel_retreat
```

実機での検証も同じ手順で行える（`bringup.launch.py` にも `follow_planner_mapless` が同様に組み込まれている）。地図・Nav2・AMCL を一切起動していない状態でも `FOLLOWING_MAPLESS` 単体で動作することを確認するとよい。

---

## ワールド環境の構成

```txt
panel_room.world（デフォルト）
  部屋サイズ: 10m × 8m
  配電盤:     北壁沿いに 3 台（青い箱）
  通路:       幅 1.4m の仕切り壁（狭路追従テスト用）
  試験員:     Gazebo Actor（walk.dae アニメーション付き人物）
              → walk.dae がない場合は panel_room_no_actor.world を使用

panel_room_no_actor.world（代替）
  試験員:     cylinder モデル（person_mover.py で自動移動）
```

---

## 試験員（Actor/Cylinder）の制御

### Actor 版（panel_room.world）

Actor はウェイポイントに沿って自動で動きます。経路の変更は world ファイルの
`<trajectory>` セクションを編集してください。

```xml
<!-- 配電盤間を移動するパス例 -->
<waypoint><time>0.0</time><pose>-3.5 2.5 0 0 0 0</pose></waypoint>
<waypoint><time>5.0</time><pose> 3.5 2.5 0 0 0 0</pose></waypoint>
```

### Cylinder 版（panel_room_no_actor.world）

`person_mover.py` でシナリオを制御します。

```bash
# 巡回パターン（配電盤前を往復）
ros2 run th_perception person_mover.py --ros-args -p pattern:=patrol

# 接近パターン（退避ロジックのテスト）
ros2 run th_perception person_mover.py --ros-args \
  -p pattern:=approach -p approach_dist:=0.6

# 静止パターン（静止時再配置のテスト）
ros2 run th_perception person_mover.py --ros-args -p pattern:=static
```

---

## 確認できるシナリオ

| シナリオ | 設定 | RViz2 で見るもの |
| --- | --- | --- |
| 通常追従 | `pattern:=patrol` | ロボットが試験員の後ろ 1.5m を追従 |
| 近接退避 | `pattern:=approach` | 0.8m 以内で `/cmd_vel_retreat` が発行され後退 |
| 狭路真後ろ追従 | 仕切り壁の通路を通る | 角度オフセットが 0° になる |
| 静止再配置 | `pattern:=static` | 2 秒後に試験員の側面〜背面に移動 |
| 配電盤移動 | `ros2 service call /panel_navigator/go_to_panel ...` | Nav2 がパネル前まで誘導 |
| E-Stop | タブレット UI の緊急停止 | ロボットが即時停止し ESTOP モードへ |
| MAP不要軌跡追従 | `FOLLOWING_MAPLESS` へ切替 + `pattern:=approach` | 地図・Nav2 なしで追従、接近時は退避せず停止(詳細は Step 5 参照) |

---

## RViz2 の見方

起動時に表示される `th_sim.rviz` レイアウト:

| 表示 | 説明 |
| --- | --- |
| 赤い点群 | `/scan`（生スキャン、アルミ角柱の死角含む） |
| 緑の点群 | `/scan_filtered`（死角マスク済み、Nav2 が使用するもの） |
| グレーの地図 | SLAM で作成中の `/map` |
| 薄い色のオーバーレイ | ローカル costmap（Nav2 の障害物回避に使用） |
| 青い線 | Nav2 の計画経路 |
| 黄色い球 | 試験員の予測位置（`/person/predicted_position`） |
| ロボットモデル | URDF から生成された 3D モデル |

---

## よくある問題と対処

### Gazebo が起動しない（画面が出ない）

```bash
# X11 転送を確認
echo $DISPLAY           # :0 や :1 が表示されるべき
xhost +local:docker     # ホスト側で実行

# Docker 内で確認
glxinfo | head -5       # OpenGL が使えるか確認
```

### ロボットが Gazebo でスポーンされない

```bash
# spawn を手動で再実行
ros2 run gazebo_ros spawn_entity.py \
  -topic /robot_description -entity th_robot -x -4 -y -3 -Y 0
```

### walk.dae が見つからない（Actor エラー）

`panel_room_no_actor.world` に切り替えてください:

```bash
ros2 launch th_bringup gazebo.launch.py \
  world:=$(ros2 pkg prefix th_bringup)/share/th_bringup/worlds/panel_room_no_actor.world
```

### /scan が来ない（LiDAR プラグインが動かない）

```bash
# Gazebo プラグインのロードを確認
ros2 topic list | grep scan
# → /scan が表示されなければ URDF の gazebo_plugins.xacro を確認

# LiDAR の可視化を ON にして Gazebo で確認
# (gazebo_plugins.xacro の <visualize>false</visualize> を true に変更)
```

### 試験員の位置が /person/status に来ない

```bash
# Gazebo のモデル状態を確認
ros2 topic echo /gazebo/model_states | grep -A5 inspector

# リレーノードのログを確認
ros2 node info /gazebo_person_relay
```
