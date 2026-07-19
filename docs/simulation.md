# Gazebo シミュレーション

[← README に戻る](../README.md)


## 概要

実機（ESP32・RPLIDAR S1）がなくても、Gazebo Classic 上で全追従ロジックを視覚的に確認できます。
`sim:=true/false` の引数一つで実機とシミュレーションを切り替えられます。
さらに `scenario:=<name>` でワールド・地図・試験員の動き・スポーン位置・障害物・
プランナパラメータを一括切替できます（下記「シナリオプリセット」参照）。

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

# 7. シナリオプリセットで起動（推奨）
ros2 launch th_bringup gazebo.launch.py scenario:=narrow_room
```

---

## シナリオプリセット

`config/scenarios/<name>.yaml` に定義されたプリセットを `scenario:=<name>` で
一括ロードする。**優先順位: CLI 明示指定 > シナリオプリセット > 従来デフォルト**
（`scenario` 未指定なら従来どおり panel_room + patrol + wanderer）。

| シナリオ | 目的 | 起動 / モード切替 | 期待動作・合格基準 |
| --- | --- | --- | --- |
| `narrow_room` | 狭所追従（実機の現行狭所パラメータ検証） | `scenario:=narrow_room` → モード 7 | 幅 1.2m の L 字通路 + 0.9m 狭窄部を壁接触なしで追従。0.5m 未満で停止・0.7m 超で再開 |
| `wide_area` | 広所向けパラメータ (stop 1.0 / resume 1.3 / obstacle 1.0) の先行検証 | `scenario:=wide_area` → モード 7 | 約 1.0m で停止・約 1.3m で再開。`ros2 param get /follow_planner_mapless stop_distance` が 1.0 |
| `cluttered` | 什器（机脚・椅子脚・柱）環境での追従観察 | `scenario:=cluttered` → モード 7 → 2 | 机脚の間を追従。脚と人物の混同挙動を観察・記録（診断用途）。モード 2 で wanderer を回避 |
| `lost_reacquire` | 人物ロスト → 捜索 → 再捕捉 | `scenario:=lost_reacquire` → モード 7 | 遮蔽板通過 約 1 秒で `/person/status` の `is_lost: true` → 捜索 → 再出現で再捕捉 |
| `panel_shuttle` | 試験場での一連の運用フロー（[VISION.md](../VISION.md) §1-3 準拠） | `scenario:=panel_shuttle`（下記の手順参照） | mapless追従→IDLE待機(捕捉継続・速度ゼロ)→配電盤巡回→mapless帰還が一通り成立する |

モード切替コマンド（起動 10 秒後・Nav2 active 後）:

```bash
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 7, requester: 'cli'}"     # 7=FOLLOWING_MAPLESS, 2=FOLLOWING
```

### panel_shuttle: VISION.md 準拠の一連フロー検証

[VISION.md](../VISION.md) の完成形運用（保管場所⇔試験場は mapless 追従、試験場内は
IDLE 待機、配電盤へは要請時のみ移動）を一通りなぞる手順。`panel_shuttle` は
試験員(inspector)が起動直後にスポーン地点(保管場所想定)から通路を抜けて
配電盤前エリア(試験場想定)まで自動で歩くよう経路を組んである。

```bash
# 0. 起動（起動直後は IDLE。試験員は保管場所→試験場へ歩き始める）
ros2 launch th_bringup gazebo.launch.py scenario:=panel_shuttle

# 1. mapless 追従で試験場へ移動（試験員のあとを追う）
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 7, requester: 'cli'}"

# 2. 試験員が配電盤エリアで静止したら IDLE へ（試験場内の既定状態）
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 1, requester: 'cli'}"
# → この間 /person/status が更新され続け、/cmd_vel が常にゼロであることを確認
#   (VISION.md §4: 捕捉継続と移動不可の両立)
ros2 topic echo /person/status
ros2 topic echo /cmd_vel

# 3. 配電盤ごとに「移動要請 → 作業完了 → IDLE」を panel_01/02/03 で繰り返す
#    (panel_navigator が MOVING_TO_PANEL → AT_PANEL → IDLE を自動遷移させる)
ros2 service call /panel_navigator/go_to_panel \
  th_system_msgs/srv/GoToPanel "{panel_id: 'panel_01'}"
# 到着(AT_PANEL)を確認したら:
ros2 service call /panel_navigator/complete_inspection \
  th_system_msgs/srv/CompleteInspection "{panel_id: 'panel_01'}"
# panel_02, panel_03 も同様に繰り返す

# 4. 全配電盤の点検完了後、再び mapless 追従で保管場所へ帰還
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 7, requester: 'cli'}"
```

合格基準: 各ステップでモード遷移が `mode_manager` に拒否されないこと、ステップ2で
`/cmd_vel` がゼロのまま `/person/status` が更新され続けること、各配電盤に
`arrival_threshold_m`(0.3m)以内で到達すること、最後に試験員に追従して
スポーン地点付近まで戻れること。

### プリセット YAML スキーマ

```yaml
scenario:
  description: "..."
  recommended_mode: 7            # 起動ログに set_mode コマンドを表示
  world: narrow_room.world       # worlds/ 相対 (必須)
  slam: true                     # false のとき map_yaml が必須
  map_yaml: ""                   # maps/ 相対
  robot_spawn: {x: 0.0, y: 0.0, yaw: 0.0}
  person:
    pattern: waypoints           # waypoints | patrol | approach | static
    waypoints: [x1, y1, pause1, x2, y2, pause2, ...]   # flat リスト
    move_speed: 0.4
  relay:
    max_detect_range: 8.0
    occlusion_check: false       # true で遮蔽(LOS)判定を有効化
    occlusion_segments: []       # flat [x1,y1,x2,y2,...] world 座標の壁線分
  obstacle:
    enabled: false
    bounds: {x_min: -4.0, x_max: 4.0, y_min: -3.0, y_max: 3.0}
    move_speed: 0.5
  planning_overrides:            # planning_params.yaml の後に dict で上書き
    follow_planner_mapless: {stop_distance: 1.0, ...}
```

整合性は純粋 pytest で検証できる（ワールド・地図の参照切れ、配列長など）:

```bash
python3 -m pytest src/th_testing/test/test_scenario_configs.py -v
```

### 遮蔽（LOS）チェックの仕組み

`lost_reacquire` では `gazebo_person_relay.py` がロボット→人物の視線と
`occlusion_segments`（遮蔽板の壁線分）の交差を判定し、遮られている間は位置更新を
止める。`lost_timeout_sec`（1.0 秒）経過で自然に `is_lost: true` となり、
人物が再び見えると自動で再捕捉する。
**ワールドの遮蔽板を動かしたら、シナリオ YAML の `occlusion_segments` も必ず同期**
させること（`lost_reacquire.world` のヘッダコメントに端点座標一覧あり）。

### wide_area 用地図の生成

`wide_area` プリセットは `maps/wide_area_map.yaml` を参照する。未生成の場合は:

```bash
# 1. SLAM モードで起動し、テレオペで柱を全て通る一周を走行
ros2 launch th_bringup gazebo.launch.py scenario:=wide_area slam:=true
ros2 launch th_bringup teleop.launch.py direct:=true   # 別ターミナル

# 2. 地図を保存してコミット
ros2 run nav2_map_server map_saver_cli \
  -f /root/th_ws/src/th_bringup/maps/wide_area_map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10000.0
```

narrow_room / cluttered / lost_reacquire は地図をコミットしない
（モード 7 は mapless、モード 2 の検証は `slam:=true` のライブ地図で行い、
ワールド編集と地図の乖離を防ぐ）。

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
| 接近時は退避せず停止 | `person_mover.py --ros-args -p pattern:=approach -p approach_dist:=0.6` 等で `stop_distance`（現行設定 0.5m・狭所向け）以内に接近させる | 後退・旋回せず、その場で停止する（`/cmd_vel_retreat` がゼロ） |
| 停止後の追従再開 | 接近させた後、`resume_distance`（現行設定 0.7m）以上離す | 自動的に追従を再開する（オペレータ操作不要） |
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

> `planning_params.yaml` の `follow_planner_mapless` は現在 **狭所向け設定
> (stop 0.5 / resume 0.7 / obstacle 0.45 / lookback 0.5)**。広所向け設定
> (stop 1.0 / resume 1.3 / obstacle 1.0 / lookback 1.0) は `scenario:=wide_area`
> の `planning_overrides` として適用される。

実機での検証も同じ手順で行える（`bringup.launch.py` にも `follow_planner_mapless` が同様に組み込まれている）。地図・Nav2・AMCL を一切起動していない状態でも `FOLLOWING_MAPLESS` 単体で動作することを確認するとよい。

---

## ワールド環境の構成

```txt
panel_room_no_actor.world（デフォルト / panel_shuttle）
  部屋サイズ: 10m × 8m、配電盤 3 台（北壁・青い箱）、幅 1.4m の仕切り通路
  試験員:     cylinder モデル（person_mover.py で自動移動）

narrow_room.world（narrow_room）
  8×6m。幅 1.2m の L 字通路 + 幅 0.9m の狭窄部

wide_area.world（wide_area）
  20×16m ホール。非対称配置の柱 7 本 + 壁アルコーブ（SLAM/AMCL の特徴用）

cluttered.world（cluttered）
  10×8m。机 3（脚のみ collision）・椅子 4・柱 2。LiDAR には細い脚だけ映る

lost_reacquire.world（lost_reacquire）
  12×8m。自立遮蔽板 3 枚（occlusion_segments と同期必須）

panel_room.world（旧 actor 版・保守のみ）
  試験員:     Gazebo Actor（walk.dae アニメーション付き人物）
```

ワールドを追加する際の必須構成（`gazebo_ros_state` プラグイン等）は
`th_bringup/worlds/README.md` を参照。

---

## 試験員（Actor/Cylinder）の制御

試験員は cylinder モデル `inspector` を `person_mover.py` が動かす方式が標準
（シナリオプリセットの `person:` セクションで自動設定される）。
手動で動かし直す場合:

```bash
# 巡回パターン（配電盤前を往復）
ros2 run th_perception person_mover.py --ros-args -p pattern:=patrol

# 接近パターン（接近停止ロジックのテスト）
ros2 run th_perception person_mover.py --ros-args \
  -p pattern:=approach -p approach_dist:=0.6

# 静止パターン（静止時再配置のテスト）
ros2 run th_perception person_mover.py --ros-args -p pattern:=static

# 任意経路（flat [x, y, pause_sec, ...]。pattern より優先される）
ros2 run th_perception person_mover.py --ros-args \
  -p waypoints:="[1.0, -1.0, 2.0, 4.0, -1.0, 3.0]"
```

旧 actor 版（`panel_room.world`）はウェイポイントを world ファイルの
`<trajectory>` セクションで編集する。新規シナリオでは使用しない。

---

## 確認できるシナリオ

| シナリオ | 設定 | RViz2 で見るもの |
| --- | --- | --- |
| 通常追従 | `pattern:=patrol` | ロボットが試験員の後ろ 1.5m を追従 |
| 近接退避 | `pattern:=approach` | 0.8m 以内で `/cmd_vel_retreat` が発行され後退 |
| 狭路真後ろ追従 | 仕切り壁の通路を通る | 角度オフセットが 0° になる |
| 静止再配置 | `pattern:=static` | 2 秒後に試験員の側面〜背面に移動 |
| 試験場の一連フロー | `scenario:=panel_shuttle`(手順は上記参照) | mapless追従→IDLE待機→配電盤巡回→mapless帰還(VISION.md準拠) |
| E-Stop | タブレット UI の緊急停止 | ロボットが即時停止し ESTOP モードへ |
| MAP不要軌跡追従 | `FOLLOWING_MAPLESS` へ切替 + `pattern:=approach` | 地図・Nav2 なしで追従、接近時は退避せず停止(詳細は Step 5 参照) |
| 狭所追従 | `scenario:=narrow_room` + モード 7 | 幅 1.2m 通路を壁接触なしで追従 |
| 広所追従 | `scenario:=wide_area` + モード 7 | 広所パラメータ (1.0/1.3) で停止・再開 |
| 什器環境 | `scenario:=cluttered` + モード 7→2 | 机脚の間を追従、wanderer 回避 |
| ロスト再捕捉 | `scenario:=lost_reacquire` + モード 7 | 遮蔽で is_lost → 捜索 → 再捕捉 |

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
