# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 方針変更時のルール

このリポジトリでは `VISION.md`(README.md と同じ階層)に、ユーザーが目指す「完成形」(最終的なシステム像・挙動・要件)を記述している。

**追従ロジック・モード遷移・安全設計・アーキテクチャ全般について、ユーザーから方針変更の指示があった場合は、コードを修正する前に必ず `VISION.md` を該当箇所から更新すること。** コード修正はその後に行う。VISION.md とコードの内容に矛盾が生じた場合は、ユーザーに確認しどちらが正か明確にしてから作業を進める。

`docs/architecture.md` は現状実装の保守・拡張ガイド(as-built)であり、`VISION.md` とは役割が異なる(目指す姿 vs 今の実装)。両者が食い違う場合は VISION.md 側を優先して実装を追いつかせる。

## 開発環境

すべての ROS2 コマンドは Docker コンテナ内（または ROS2 Humble がインストールされた環境）で実行する。

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

ESP32 には独立したウォッチドッグ（300ms）があり、ROS2 がクラッシュしても停止できる。

### モード FSM

`mode_manager.cpp` の `isTransitionAllowed()` で遷移を制御:

```
IDLE → FOLLOWING, MANUAL
FOLLOWING → MANUAL, MOVING_TO_PANEL, IDLE
MOVING_TO_PANEL → AT_PANEL, MANUAL, IDLE
AT_PANEL → MANUAL, IDLE
MANUAL → FOLLOWING, IDLE
any → ESTOP
ESTOP → IDLE のみ
```

フォルト発生時は FOLLOWING / MOVING_TO_PANEL / MANUAL / AT_PANEL → IDLE へ強制遷移。IDLE 中のフォルトはモード変化なし。

### カスタムメッセージ型（th_system_msgs）

- `RobotMode.msg` — mode フィールド (uint8) と定数 IDLE=1, FOLLOWING=2, MOVING_TO_PANEL=3, AT_PANEL=4, MANUAL=5, ESTOP=6
- `PersonStatus.msg` — `position`（ロボット base_link 基準の相対座標 m）, `confidence`, `is_lost`, `lost_reason`
- `FaultStatus.msg` — `active`, `fault_type` ("LIDAR_LOST" / "ESP32_DISCONNECTED" / "PERSON_TRACKER_LOST")
- `WheelFeedback.msg` — ESP32 から届く左右ホイール実速度

### シミュレーション固有のノード

`gazebo.launch.py` が起動するが `bringup.launch.py` には含まれないノード:

- `gazebo_person_relay.py` — Gazebo の Actor/モデル位置を `/person/status` に変換。`GetEntityState` サービス → `/gazebo/model_states` → `/gazebo/link_states` の順にフォールバック。TF は使わずロボット相対座標を直接計算（`robot_name` パラメータで参照）。
- `person_mover.py` — cylinder モデルをシナリオ制御（patrol/approach/static パターン）

### パラメータファイルの場所

| 対象 | ファイル |
|------|---------|
| 追従ロジック全般 | `th_bringup/config/planning_params.yaml` |
| 安全タイムアウト（実機） | `th_safety/config/safety_monitor.yaml` |
| 安全タイムアウト（シミュ） | `th_bringup/config/safety_monitor_sim.yaml` |
| twist_mux 優先度 | `th_safety/config/twist_mux.yaml` |
| Nav2（実機） | `th_bringup/config/nav2_params.yaml` |
| Nav2（シミュ） | `th_bringup/config/nav2_params_sim.yaml` |
| LiDAR 死角 | `th_bringup/config/perception_params.yaml` |
| 配電盤座標 | `th_bringup/config/panels.yaml` |
| ESP32 ブリッジ | `th_esp32_bridge/config/params.yaml` |

Python スクリプトは `--symlink-install` によりシンボリックリンクで即時反映される。C++ パッケージを変更した場合は `colcon build` が必要。
