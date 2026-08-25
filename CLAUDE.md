# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 作業開始前のルール

**実装作業に入る前に必ず `git status` を確認し、作業前から存在する未コミットの変更がないか調べること。**

- 未コミットの変更があれば、着手前にユーザーへ提示して扱いを確認する（先にコミットするか、そのまま残すか）。ユーザーが別のターミナルや別セッションで進行中の作業であることが多い。
- 勝手にコミットも破棄もしない。確認せずに作業を始めると、こちらの変更と混ざって切り分けられなくなる。
- コミット時は自分が変更したファイルだけをステージする。`git add -A` / `git add .` は作業前からあった変更を巻き込むため使わない。
- この環境では `git add -p` が使えない（対話的フラグ非対応）ため、1つのファイルに複数の関心事の変更を混ぜると後からコミットを分割できない。無関係な変更を同じファイルに同時に入れないよう作業順序を組む。

## 方針変更時のルール

このリポジトリでは `VISION.md`(README.md と同じ階層)に、ユーザーが目指す「完成形」(最終的なシステム像・挙動・要件)を記述している。

**追従ロジック・モード遷移・安全設計・アーキテクチャ全般について、ユーザーから方針変更の指示があった場合は、コードを修正する前に必ず `VISION.md` を該当箇所から更新すること。** コード修正はその後に行う。VISION.md とコードの内容に矛盾が生じた場合は、ユーザーに確認しどちらが正か明確にしてから作業を進める。

`docs/architecture.md` は現状実装の保守・拡張ガイド(as-built)であり、`VISION.md` とは役割が異なる(目指す姿 vs 今の実装)。両者が食い違う場合は VISION.md 側を優先して実装を追いつかせる。

`docs/plan/` は**未確定の検討メモ**を置く場所で、VISION.md を上書きしない。書き方のルール（本体は結論と表だけにして一目で読める分量を保ち、根拠・詳細は `<テーマ>-<側面>.md` に分ける）は `docs/plan/README.md` に定義してある。plan 配下を編集する前に必ず読むこと。

## このファイル自体の保守ルール

- 作業中に判明したこのプロジェクト固有の環境の癖・落とし穴（コマンドの意外な挙動、ツールの制約など）は、ユーザーに確認せず「環境の癖」セクションに追記してよい。
- **CLAUDE.md を更新するたびに、ファイル全体を読み直し、陳腐化した記述・重複・冗長な説明がないか見直すこと。** コンテキストを圧迫しないよう、価値の下がった記述は削除するか簡潔にまとめる。肥大化を優先して情報を積み増すだけにしない。

## 環境の癖・注意点

- `ros2 node list` はデーモンキャッシュの影響で新規ノードが反映されないことがある。`ros2 node list --no-daemon`（または `ros2 daemon stop` 後に再実行）で確実に最新状態を取得する。
- **`th_robot` コンテナはユーザーが実機作業中のセッションであることがある。** デバッグ用にノードを起動・停止する前に必ず `docker exec th_robot ps -eo pid,etimes,args` で稼働中のプロセスを確認し、自分が起動したものだけを PID 指定で止めること（実際に `rotation_calib.py` が 50 分間走っている最中に遭遇した）。
- `docker exec th_robot bash -lc '... pkill -f <pattern> ...'` は、パターンがこのシェル自身のコマンドライン（`-lc` の引数文字列全体）にマッチして**自分を殺す**。出力が一切出ず exit 143 になったらこれを疑う。スクリプトをファイルに書いてから実行するか、PID 指定で止める。
- 長時間動くノード（`component_container_mt` 等）を `docker exec` から `&` で起動すると、シェル終了時に道連れになる。`setsid ... > log 2>&1 < /dev/null &` で切り離す。
- **`docker compose run --rm th_robot` は毎回新しいコンテナを作り、`build/` と `install/` はバインドマウントされていない**（マウントは `src` / `esp32` / `scripts` / `data` / `dr_spaam_weights` のみ）。そのため `colcon build` と `colcon test` を別々の `docker compose run` で実行すると、テスト側からビルド成果が見えず `colcon test-result` が「0 tests」になる。**ビルドからテストまでを 1 回の `bash -lc` の中で通すこと。**
- `test_simulation_scenarios.py` は Gazebo + Nav2 のフル環境が前提。環境なしでもスキップ判定が効かずに実行され、`test_scenario_A1_retreat_on_approach` が「後退指令が来なかった」で落ちる。ヘッドレスでの `colcon test` ではこの 1 件の失敗は想定内。
- **テストの大半は Docker 不要でホストの `python3` から直接走る。**`th_ws/src/th_testing/test/` のうち ROS2 環境（`rclpy` / ビルド済み `th_system_msgs`）が要るのは次の 9 ファイルだけで、他は素の pytest で緑赤を判定できる（2026-08-24 時点で 385 passed）。`colcon build` は数分かかるので、まずホストで回して最後に Docker で 1 回通すのが速い。
  除外する 9 ファイル: `test_connectivity_checker_node.py` / `test_fault_detection.py` / `test_mode_transitions.py` / `test_params_audit_node.py` / `test_safety_monitor.py` / `test_state_manager_node.py` / `test_twist_mux_priority.py` / `test_simulation_scenarios.py` / `test_msg_definitions.py`
- `th_ws/esp32/.vscode/extensions.json` は **`.gitignore` に載っているのに tracked** という状態で、内容もモードも index と一致しているのに `git status` に `M` が出続けることがある（index の stat キャッシュが NTFS 時代の古いサイズを持っているため）。`git diff` が空なのに `M` が消えないときはこれ。`git add -f <path>` で解消でき、内容が同じなので差分はステージされない。
- **ノードを `kill -9` で落とすことを繰り返すと、コンテナ内の DDS discovery が壊れる。** 症状は「ノードは起動しログも出ているのに、他プロセスからサービス/トピックが一切見つからない」。`ls /dev/shm | wc -l` で `fastrtps_*` の残骸が溜まっているか確認する（ROS プロセスが 0 なのに大量にあれば該当）。`/dev/shm` の掃除だけでは直らないことがあり、その場合はコンテナ再起動が必要。デバッグ用ノードは `kill -TERM` で落とすこと。

## 開発環境

すべての ROS2 コマンドは Docker コンテナ内（または ROS2 Humble がインストールされた環境）で実行する。

Windows では Docker Desktop ではなく **WSL2 内の Docker Engine** でコンテナを起動する。`docker compose` はこの WSL2 側の Docker Engine に接続されるため、コマンドは WSL2 のシェル（または WSL2 統合が有効なターミナル）から実行すること。

```bash
# Linux
xhost +local:docker
docker compose run --rm th_robot bash

# Windows (WSL2)
export DISPLAY=:0
docker compose run --rm th_robot bash
```

コンテナ内では `/root/th_ws` がワークスペースルート。

## ビルドとテスト

```bash
# フルビルド（初回・C++ 変更時）
cd /root/th_ws
colcon build --symlink-install
source install/setup.bash

# 特定パッケージのみ
colcon build --symlink-install --packages-select th_safety th_mode_manager

# 純粋単体テスト（ROS2 不要・最速）
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -v

# 特定テストクラスのみ
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -v -k "TestRetreatHysteresis"

# ROS2 統合テスト
colcon test --packages-select th_testing --event-handlers console_direct+
colcon test-result --verbose

# 一括実行スクリプト
bash scripts/run_tests.sh           # 単体テストのみ
bash scripts/run_tests.sh --all     # 単体 + 統合
bash scripts/run_tests.sh --all --sim  # + シナリオテスト
```

## シミュレーション起動

```bash
# SLAM で地図作成（初回）
ros2 launch th_bringup gazebo.launch.py

# 既存地図でナビゲーション
ros2 launch th_bringup gazebo.launch.py \
  slam:=false map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml

# シナリオプリセットで起動（narrow_room / wide_area / cluttered /
# lost_reacquire / panel_shuttle。th_bringup/config/scenarios/ 参照）
ros2 launch th_bringup gazebo.launch.py scenario:=narrow_room

# キーボードテレオペ（別ターミナル）
ros2 launch th_bringup teleop.launch.py           # /cmd_vel_nav 経由（通常）
ros2 launch th_bringup teleop.launch.py direct:=true  # /cmd_vel 直接（SLAM 用）

# FOLLOWING モードに切替（起動 10 秒後）
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 2, requester: 'cli'}"
```

## アーキテクチャ

### 速度指令の流れ（最重要）

```
follow_planner.py ─→ /cmd_vel_retreat (priority 20) ─┐
person_predictor.py ─→ /cmd_vel_retreat (priority 20) ─┤
Nav2 controller_server ─→ /cmd_vel_nav (priority 10) ───┤ twist_mux ─→ /cmd_vel ─→ ESP32
                                                          │
safety_monitor ─→ /safety/estop     (lock 255) ──────────┤
safety_monitor ─→ /safety/fault_lock (lock 254) ─────────┘
```

**不変ルール**: `/cmd_vel` に直接 publish するノードを追加してはいけない。すべての速度指令は twist_mux 経由。退避・捜索旋回は `/cmd_vel_retreat`（priority 20）、Nav2 経由の移動は `/cmd_vel_nav`（priority 10）を使う。

### 追従ロジックの二層構造

追従ロジックは意図的に二層に分けられている:

- `th_planning/th_planning/follow_planner_core.py` — **ROS2 非依存の純粋 Python**。`FollowPlannerCore.update()` がコアアルゴリズム。このファイルは ROS2 を import しないことで `pytest` で直接テスト可能。
- `th_planning/scripts/follow_planner.py` — ROS2 ノード。`follow_planner_core.py` を import して `/person/status` → Nav2 ゴール / `/cmd_vel_retreat` に接続するだけ。

新しい追従ロジックを追加する際は必ず `follow_planner_core.py` に純粋関数として実装し、`test_follow_planner_logic.py` にテストを追加してから `follow_planner.py` から呼ぶ。

### 安全チェーンの設計

`safety_monitor`（C++）が `/safety/estop` と `/safety/fault_lock` を twist_mux に送る。`mode_manager` の処理を待たずに twist_mux がモーターをゼロにする（フォルト検知 → 物理停止は 100ms 以内）。

ESP32 には独立したウォッチドッグ（600ms、`config.h` の `WATCHDOG_MS`）があり、ROS2 がクラッシュしても停止できる。esp32_bridge は `/cmd_vel` を20Hzキープアライブで再送しており、WiFi ジッタによる誤発動を避けるため2026-08-05に300ms→600msへ緩和した（詳細: `docs/architecture.md`「ESP32側の二重フェイルセーフ」）。

### オドメトリと TF

```
ESP32 (WHEEL_FEEDBACK: 左右速度 + dt) ─→ esp32_bridge ─→ /odom (publish_tf は false)
                                                            ↓
ESP32 (IMU_DATA: BNO055) ─→ /esp32/imu_data ─→ ekf_filter_node ─→ odom→base_link TF
```

**不変ルール**: `odom → base_link` の TF を発行するのは `ekf_filter_node` だけ。`esp32_bridge` の `publish_tf` を true に戻してはいけない（TF ツリーが二重親になる）。

- EKF が融合する IMU 入力は**ジャイロの `vyaw` のみ**。BNO055 は NDOF モードで絶対方位（地磁気参照）を返すため、屋内の磁気擾乱でヨーが飛ぶ。`world_frame: odom` に絶対方位を入れてはいけない。
- オドメトリの積分区間は ESP32 が `WHEEL_FEEDBACK` に載せてくる `dt`。到着時刻から推測してはいけない（WiFi 遅延がそのまま yaw ドリフトになる）。旧形式の 9 byte フレームも受理する。

### モード FSM

`mode_manager.cpp` の `isTransitionAllowed()` で遷移を制御:

```
INIT → IDLE のみ
IDLE → FOLLOWING, FOLLOWING_MAPLESS, SUMMONING, MANUAL
FOLLOWING → MANUAL, MOVING_TO_PANEL, IDLE
FOLLOWING_MAPLESS → MANUAL, IDLE
SUMMONING → MANUAL, IDLE
MOVING_TO_PANEL → AT_PANEL, MANUAL, IDLE
AT_PANEL → FOLLOWING, MANUAL, IDLE
MANUAL → FOLLOWING, FOLLOWING_MAPLESS, IDLE
any → ESTOP
ESTOP → IDLE のみ
```

フォルト発生時は動作系モード（FOLLOWING / FOLLOWING_MAPLESS / SUMMONING / MOVING_TO_PANEL / AT_PANEL / MANUAL）から IDLE へ強制遷移。IDLE 中のフォルトはモード変化なし。

ただし `PERSON_TRACKER_LOST` だけは例外で、試験員データを使うモード（FOLLOWING / FOLLOWING_MAPLESS / SUMMONING）からのみ強制遷移する。MANUAL ジョグや配電盤移動は人物データを使わないため継続できる（VISION.md §5）。

### カスタムメッセージ型（th_system_msgs）

- `RobotMode.msg` — mode フィールド (uint8) と定数 INIT=0, IDLE=1, FOLLOWING=2, MOVING_TO_PANEL=3, AT_PANEL=4, MANUAL=5, ESTOP=6, FOLLOWING_MAPLESS=7, SUMMONING=8
- `PersonStatus.msg` — `position`（ロボット base_link 基準の相対座標 m）, `confidence`, `is_lost`, `lost_reason`
- `FaultStatus.msg` — `active`, `fault_type` ("LIDAR_LOST" / "ESP32_DISCONNECTED" / "PERSON_TRACKER_LOST")
- `WheelFeedback.msg` — ESP32 から届く左右ホイール実速度(`/esp32/wheel_feedback`)。指令値側にも同型を再利用し `/esp32/wheel_cmd_speed`(esp32_bridge が `/cmd_vel` から計算)として発行、WebUI の速度表示カードで指令vs実測を比較する
- 状態 publish 3種（`FollowStatus` = `/follow/status`、`SearchStatus` = `/person/search_status`、`SummonStatus` = `/summon_navigator/status`）— 追従・捜索・呼び寄せの内部状態。音声アナウンスと WebUI 表示のトリガ源。**state / reason / phase の文字列定義は各 .msg のコメントが正**なので、写像を書くときは必ずそちらを見る

### シミュレーション固有のノード

`gazebo.launch.py` が起動するが `bringup.launch.py` には含まれないノード:

- `gazebo_person_relay.py` — Gazebo の Actor/モデル位置を `/person/status` に変換。`GetEntityState` サービス → `/gazebo/model_states` → `/gazebo/link_states` の順にフォールバック。TF は使わずロボット相対座標を直接計算（`robot_name` パラメータで参照）。
- `person_mover.py` — cylinder モデルをシナリオ制御（patrol/approach/static パターン）

### パラメータファイルの場所

| 対象 | ファイル |
|------|---------|
| 追従ロジック全般 | `th_bringup/config/planning_params.yaml` |
| オドメトリ融合（EKF） | `th_bringup/config/ekf_params.yaml`（IMU有効・既定） / `ekf_params_no_imu.yaml` |
| 人物トラッカー | `leg_detection_bringup/param/leg_tracker_param.yaml` |
| 安全タイムアウト（実機） | `th_safety/config/safety_monitor.yaml` |
| 安全タイムアウト（シミュ） | `th_bringup/config/safety_monitor_sim.yaml` |
| twist_mux 優先度 | `th_safety/config/twist_mux.yaml` |
| Nav2（実機） | `th_bringup/config/nav2_params.yaml` |
| Nav2（シミュ） | `th_bringup/config/nav2_params_sim.yaml` |
| LiDAR 死角 | `th_bringup/config/perception_params.yaml` |
| 配電盤座標 | `th_bringup/config/panels.yaml` |
| ESP32 ブリッジ | `th_esp32_bridge/config/params.yaml` |

Python スクリプトは `--symlink-install` によりシンボリックリンクで即時反映される。C++ パッケージを変更した場合は `colcon build` が必要。
