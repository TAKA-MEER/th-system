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

- **`sudo` はこのセッションからは実行できない。**パスワードが要るうえ TTY が無く、`!` プレフィックス経由でも `a terminal is required to read the password` になる。管理者権限が要る作業はスクリプトに書き出し、ユーザーに別ターミナルで実行してもらう。`usermod -aG` の反映にはセッションの再起動が要る（グループはログイン時に確定するため、Claude Code を再起動しないとツール側から見えない）。
- `ros2 node list` はデーモンキャッシュの影響で新規ノードが反映されないことがある。`ros2 node list --no-daemon`（または `ros2 daemon stop` 後に再実行）で確実に最新状態を取得する。
- **`th_robot` コンテナはユーザーが実機作業中のセッションであることがある。** デバッグ用にノードを起動・停止する前に必ず `docker exec th_robot ps -eo pid,etimes,args` で稼働中のプロセスを確認し、自分が起動したものだけを PID 指定で止めること（実際に `rotation_calib.py` が 50 分間走っている最中に遭遇した）。
- `docker exec th_robot bash -lc '... pkill -f <pattern> ...'` は、パターンがこのシェル自身のコマンドライン（`-lc` の引数文字列全体）にマッチして**自分を殺す**。出力が一切出ず exit 143 になったらこれを疑う。スクリプトをファイルに書いてから実行するか、PID 指定で止める。
- 長時間動くノード（`component_container_mt` 等）を `docker exec` から `&` で起動すると、シェル終了時に道連れになる。`setsid ... > log 2>&1 < /dev/null &` で切り離す。
- **`docker compose run --rm` は毎回まっさらなコンテナを作る。**`docker-compose.yml` がバインドマウントするのは `./src` `./esp32` `./scripts` `../docs`（読み取り専用）の4つだけで、`th_ws/install` `th_ws/build` はコンテナの書き込み層にしかない。そのため `docker compose run --rm ... colcon build` の後に**別の** `docker compose run --rm` を実行すると `install/` が消えており、`source install/setup.bash` や `python3 -m pytest`（`ModuleNotFoundError: No module named 'th_system_msgs'` 等）が失敗する。ビルドと、それを使うコマンド（テスト実行・ノード起動確認など）は**同一の `bash -lc '...'` 内で `&&` や改行で連結し、1回の `docker compose run` で完結させる**こと。
- **`ros2 test`（`ament_add_pytest_test` で `@pytest.mark.launch_test` を持つファイル）は、モジュール直下に置いた素の `pytest` 関数（`def test_xxx(): ...`）を収集しない。** `launch_testing` の pytest プラグインがファイル全体を1つの pytest アイテムとして扱い、その中で `unittest.TestCase` のメソッドだけを実行する（`pytest --collect-only` で確認できる）。同一ファイルで `launch_test` と素の関数テストを混ぜると、素の関数は**エラーも出さずに黙って実行されない**。ROS を使わない検査（AST 検査など）も含め、全部 `unittest.TestCase` のメソッドとして書くこと。
- **`ros2 run` を `&` で起動して `kill -TERM <pid>` しても、ノードの実体は生き残る。**`ros2 run` はラッパープロセスで、実際のノードは別プロセスとして起動されるため。残骸がポート（`esp32_bridge` の 8766）やトピックを掴んだままになり、次の起動が `address already in use` で落ちる。さらに `/cmd_vel` の publisher が生き残ると**新旧の速度指令が交互に届いてモータが断続的に回る**（2026-08-18 に実機で発生）。`setsid` で独立プロセスグループにして `kill -TERM -<pid>`（負号＝グループ宛）で落とすか、**用途ごとにコンテナを分けて終了で片づける**。
- **実機が動いている状態で統合テストを回すと落ちる。**実機（ラズパイの `/scan`、ESP32 ブリッジ）は `ROS_DOMAIN_ID=10` で動いており、テストが立てるスタブと**同じドメインで混ざる**。`test_fault_detection` / `test_safety_monitor` は「LiDAR が途絶したらフォルトを出す」ことを確かめるが、実機の `/scan` が生きていると途絶しないので**必ず失敗する**（2026-08-18 に遭遇）。実機作業と並行してテストを回すときは `docker compose run --rm -e ROS_DOMAIN_ID=42 ...` のようにドメインを分ける。
- **実機のリンク品質を測定している最中に ROS2 ノードを起動しない。**DDS のディスカバリはすべてのインターフェースにマルチキャストを流すため、計測対象の Wi-Fi リンクに自分で負荷を掛けることになる。測定中に進める作業は、ビルドと ROS2 非依存の純粋テストまでに限る。
- **ノードを `kill -9` で落とすことを繰り返すと、コンテナ内の DDS discovery が壊れる。** 症状は「ノードは起動しログも出ているのに、他プロセスからサービス/トピックが一切見つからない」。`ls /dev/shm | wc -l` で `fastrtps_*` の残骸が溜まっているか確認する（ROS プロセスが 0 なのに大量にあれば該当）。`/dev/shm` の掃除だけでは直らないことがあり、その場合はコンテナ再起動が必要。デバッグ用ノードは `kill -TERM` で落とすこと。

## 開発環境

**開発機は Ubuntu 22.04 実機 ＋ ネイティブ Docker Engine**（Docker Desktop は使わない）。ホストに ROS2 は入れず、すべての ROS2 コマンドはコンテナ内で実行する。

```bash
xhost +local:docker
docker compose run --rm th_robot bash
```

Windows で作業する場合のみ WSL2 内の Docker Engine を使う（`export DISPLAY=:0` が要る）。初回構築は `docs/setup.md`、Docker 導入後は `th_ws/setup.sh` がイメージビルドと npm 導入をまとめて行う。

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
