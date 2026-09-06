# テスト

[← README に戻る](../README.md)


## テスト構成一覧

**`th_ws/src/th_testing/test/` に 55 ファイル。うち 43 ファイル・783 件は ROS2 なしで
ホストの `python3 -m pytest` から直接走る**（2026-09-06 時点。`783 passed / 1 skipped`）。
`colcon build` は数分かかるので、**まずホストで回して最後に Docker で 1 回通す**のが速い。

| 区分 | ファイル数 | 件数 | 実行方法 |
| --- | --- | --- | --- |
| ROS2 不要（純粋コア・設定整合・静的検査） | 43 | 783 | ホストで `python3 -m pytest`（下記） |
| ROS2 必須（`rclpy` / ビルド済み `th_system_msgs`） | 11 | — | コンテナで `colcon test`（下記） |
| その他（`fault_injection/` のケース等） | 1 | — | `colcon test` 経由 |

**ROS2 が必要な 11 ファイル**（これ以外はホストで走る）:

```txt
test_connectivity_checker_node.py  test_fault_detection.py     test_mode_transitions.py
test_params_audit_node.py          test_safety_monitor.py      test_state_manager_node.py
test_twist_mux_priority.py         test_simulation_scenarios.py test_msg_definitions.py
test_esp32_bridge_node.py          test_jog_gate_node.py
```

件数の多い主なファイル:

| ファイル | 件数 | 対象 |
| --- | --- | --- |
| `test_transition_table.py` | 137 | 現行 FSM（`th_state`）の遷移表 |
| `test_mapless_follow_logic.py` | 53 | 地図なし追従コア（旧設計） |
| `test_follow_planner_logic.py` | 45 | 人物追従コア（旧設計） |
| `test_scenario_configs.py` / `test_esp32_ws_protocol.py` | 36 / 36 | シナリオ整合 / ESP32 フレーム定義 |
| `test_serial_framer.py` | 30 | シリアル区間のエンベロープ（sync+len+CRC8） |
| `test_slam_control_logic.py` / `test_zones.py` | 29 / 28 | 地図セッション / 速度ゾーン |
| `test_route_record_files.py` / `test_route_replay_core.py` | 28 / 25 | 教示の保存 / 再生の pure-pursuit |
| `test_params_*.py`（6 ファイル） | 87 | `registry.yaml` からの生成・導出・監査 |

> **`test_simulation_scenarios.py` は Gazebo を起動せず、既定でスキップされる**
> （`TH_SKIP_SIM=1`）。検証しているのは新設計で廃止済みの挙動（近接退避・捜索旋回）で、
> `WP-TRANSIT-01` で `follow_planner` ごと削除される見込み。復活させる前に必ず
> ファイル冒頭の docstring を読むこと。

---

## ROS2 なしのテスト実行（ホスト・最速）

`*_core.py` 系のコアロジックは ROS2 に依存しない純粋な Python として実装してあるため、
ROS2 環境がなくてもテストできます（`follow_planner_core.py` / `route_replay_core.py` /
`serial_framer.py` / `state` 系など）。

```bash
# リポジトリルートから、ROS2 が要る 11 ファイルを除いて一括実行
python3 -m pytest th_ws/src/th_testing/test/ -q \
  --ignore=th_ws/src/th_testing/test/test_connectivity_checker_node.py \
  --ignore=th_ws/src/th_testing/test/test_fault_detection.py \
  --ignore=th_ws/src/th_testing/test/test_mode_transitions.py \
  --ignore=th_ws/src/th_testing/test/test_params_audit_node.py \
  --ignore=th_ws/src/th_testing/test/test_safety_monitor.py \
  --ignore=th_ws/src/th_testing/test/test_state_manager_node.py \
  --ignore=th_ws/src/th_testing/test/test_twist_mux_priority.py \
  --ignore=th_ws/src/th_testing/test/test_simulation_scenarios.py \
  --ignore=th_ws/src/th_testing/test/test_msg_definitions.py \
  --ignore=th_ws/src/th_testing/test/test_esp32_bridge_node.py \
  --ignore=th_ws/src/th_testing/test/test_jog_gate_node.py
# → 783 passed, 1 skipped

# 個別ファイル / 特定クラスだけ
cd th_ws
python3 -m pytest src/th_testing/test/test_route_replay_core.py -v
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -v -k "TestNextFollowState"
python3 -m pytest src/th_testing/test/test_scenario_configs.py -v   # シナリオプリセット整合性
```

> **`pip3 install platformio`（ESP32 ファームのビルド用）を同じ Python 環境に入れると、
> 依存の `anyio` が pytest プラグインとして自動登録され、この環境の pytest 6.2.5 と
> 非互換で全滅する**（`ModuleNotFoundError: No module named '_pytest.scope'`）。
> `python3 -m pytest -p no:anyio ...` で回避できる（2026-09-05）。

> Gazebo シナリオ (`scenario:=narrow_room` 等) 自体は手動・目視で検証する
> （手順と合格基準は [simulation.md](simulation.md) のシナリオ表を参照）。
> `test_simulation_scenarios.py` の Gazebo 不要ハーネスは従来どおり。

`test_follow_planner_logic.py` のテスト対象クラスと確認内容:

```txt
TestFrameConversion         ── base_link相対⇔絶対座標(odom系)変換の往復・回転整合性
TestTrail                   ── 軌跡点の最小移動距離フィルタ・lookback距離での遡り
TestNextFollowState         ── TRACKING/PREPARE/EVADING の境界値・ハンチング防止
TestFindNearestOpenDirection ── 全方向自由/一方向のみ自由/最小自由空間未達での不採用
TestComputeEvadeGoal        ── 退避方向・距離からの退避ゴール点計算
TestOrientToEvadeRoute      ── 許容誤差内での停止・誤差角に比例した旋回
TestPurePursuitControl      ── 停止半径内での停止・距離に応じた速度クランプ
TestFollowPlannerCore       ── 状態遷移・trail追従ゴール・PREPARE/EVADING出力・reset/clear_trail
```

`test_mapless_follow_logic.py` のテスト対象クラスと確認内容:

```txt
TestNextMaplessState  ── stop_distance/resume_distance の境界・ハンチング防止
TestIsPathBlocked     ── 進路上障害物の有無・角度範囲外/inf/nanの無視
TestMaplessFollowCore ── 追従駆動・接近時停止と再開・障害物停止・no_pose/no_scanフェイルセーフ
```

---

## ROS2 統合テストのビルドと実行

```bash
# ROS2 環境をソース（Docker 内では不要）
source /opt/ros/humble/setup.bash

# ビルド（テスト対象パッケージを含めて）
cd th_ws
colcon build --symlink-install \
  --packages-select \
    th_system_msgs th_safety th_mode_manager th_state \
    th_planning th_perception th_config_manager th_params th_testing

source install/setup.bash

# 全統合テストを実行
colcon test --packages-select th_testing \
  --event-handlers console_direct+

# 結果確認
colcon test-result --verbose
```

> **`docker compose run --rm th_robot` は毎回新しいコンテナを作り、`build/` と
> `install/` はバインドマウントされていない**（マウントは `src` / `esp32` /
> `scripts` / `data` / `dr_spaam_weights` のみ）。そのため `colcon build` と
> `colcon test` を別々の `docker compose run` で実行すると、テスト側からビルド成果が
> 見えず `colcon test-result` が「0 tests」になる。**ビルドからテストまでを 1 回の
> `bash -lc` の中で通すこと。**

### 個別テストの単独実行

```bash
# mode_manager 遷移テストのみ
python3 -m pytest src/th_testing/test/test_mode_transitions.py \
  -v --timeout=120

# safety_monitor フォルト検知テストのみ
python3 -m pytest src/th_testing/test/test_safety_monitor.py \
  -v --timeout=90

# twist_mux 優先度テストのみ
python3 -m pytest src/th_testing/test/test_twist_mux_priority.py \
  -v --timeout=60

# フォルト→モード遷移の順序テストのみ
python3 -m pytest src/th_testing/test/test_fault_detection.py \
  -v --timeout=90
```

---

## シナリオテストの実行

シナリオテストは `person_tracker_stub` を使って Gazebo なしで実行できます。
`TH_SKIP_SIM` 環境変数で制御します（デフォルト=スキップ）。

```bash
# シナリオテストを有効化して実行
TH_SKIP_SIM=0 python3 -m pytest \
  src/th_testing/test/test_simulation_scenarios.py -v --timeout=120
```

確認されるシナリオ:

- **Scenario A-1**: 試験員が近接距離以内に接近 → `/cmd_vel_retreat` に退避指令が発行される
- **Scenario A-2**: 壁際（costmap でブロック）での接近 → その場停止
- **Scenario A-3**: 試験員が離れると退避解除 → 通常追従に戻りハンチングしない
- **Scenario C**: ロスト後の予測外挿 → `max_predict_sec` 後に `/person/search_mode=true`
- **Scenario D**: MANUAL 中は退避が作動しない（試験員が近くにいても後退しない）

> **注意**: 退避方向は試験員の位置とは無関係に `find_nearest_open_direction()` による「地図上の最空きスペース方向」で決まる。costmap を配信しない stub 環境では試験員自身が障害物として地図に現れないため、Scenario A-1 の「試験員を避けて後退した」という確認は Gazebo + 実際の costmap がある環境でのみ意味を持つ。詳細は `test_simulation_scenarios.py` 内のコメントを参照。

---

## 一括テスト実行スクリプト

```bash
# 単体テストのみ（デフォルト・高速）
bash scripts/run_tests.sh

# 単体 + ROS2 統合テスト
bash scripts/run_tests.sh --all

# 統合テスト + シナリオテスト
bash scripts/run_tests.sh --all --sim
```

---

## テスト追加のガイドライン

新しいロジックを追加する際は以下の構成に従ってください（**二層構造**）。

```txt
ロジック追加
  → <パッケージ>/<パッケージ>/<name>_core.py に ROS2 非依存の純粋関数/クラスとして実装
  → src/th_testing/test/test_<name>_core.py にテストを追加
  → src/th_testing/CMakeLists.txt の if(BUILD_TESTING) に ament_add_pytest_test で登録
  → ROS2 ノード(scripts/<name>.py)はコアロジックを呼び出して配線するだけにする
```

実例: `route_replay_core.py`（再生の pure-pursuit）／`serial_framer.py`（シリアル
エンベロープ）／`follow_planner_core.py`（人物追従・旧設計）／`state` 系。
この構造により、ROS2 環境なしでホストから高速にテストできます。

> **CMakeLists.txt への登録を忘れると `test_cmake_test_registration.py` が落ちます**
> （テストファイルを足したのに `colcon test` で走らない事故を防ぐためのガード）。

> **`launch_testing` を使うテスト（`generate_test_description()` を持つファイル）は、
> 本体が `unittest.TestCase` なので pytest のフィクスチャを一切受け取れません。**
> `conftest.py` が提供する値が要るときは、同じ解決ロジックをモジュールレベルで
> 呼ぶこと。CMake 側は `add_launch_test` ではなく `ament_add_pytest_test` で登録できます
> （`test_esp32_bridge_node.py` / `fault_injection` が実例）。

---

---
