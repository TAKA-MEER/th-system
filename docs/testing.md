# テスト

[← README に戻る](../README.md)


## テスト構成一覧

| ファイル | 分類 | ROS2 要否 | テスト件数 | 設計書対応 |
| --- | --- | --- | --- | --- |
| `test_follow_planner_logic.py` | 純粋単体テスト | 不要 | 43 件 | 10.1 §3 |
| `test_mapless_follow_logic.py` | 純粋単体テスト(MAP不要モード) | 不要 | 20 件 | - |
| `test_scenario_configs.py` | 純粋単体テスト(シナリオプリセット整合性) | 不要 | 36 件 | - |
| `test_mode_transitions.py` | ROS2 統合テスト | 必要 | 19 件 | 10.1 §1 |
| `test_safety_monitor.py` | ROS2 統合テスト | 必要 | 10 件 | 10.1 §5 |
| `test_twist_mux_priority.py` | ROS2 統合テスト | 必要 | 7 件 | 10.1 §4 |
| `test_fault_detection.py` | ROS2 統合テスト | 必要 | 4 件 | 10.1 §5+ |
| `test_simulation_scenarios.py` | シナリオテスト | 必要 | 5 件 | 10.3 |

---

## 純粋単体テストの実行（ROS2 なし）

`follow_planner_core.py` のコアロジックは ROS2 に依存しない純粋な Python モジュールとして実装されており、ROS2 環境がなくてもテストできます。

```bash
# pytest を直接実行
cd th_ws
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -v

# 特定のテストクラスだけ実行
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py \
  -v -k "TestNextFollowState"

# 失敗時に即座に停止
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -x

# シナリオプリセット (config/scenarios/*.yaml) の整合性検証
python3 -m pytest src/th_testing/test/test_scenario_configs.py -v
```

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
    th_system_msgs th_safety th_mode_manager \
    th_planning th_perception th_testing

source install/setup.bash

# 全統合テストを実行
colcon test --packages-select th_testing \
  --event-handlers console_direct+

# 結果確認
colcon test-result --verbose
```

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

新しいロジックを追加する際は以下の構成に従ってください。

```txt
ロジック追加
  → th_planning/th_planning/follow_planner_core.py に純粋関数/クラスとして実装
  → src/th_testing/test/test_follow_planner_logic.py にテストクラスを追加
  → ROS2 ノード(follow_planner.py)はコアロジックを呼び出すだけにする
```

この構造により、ROS2 環境なしで CI/CD パイプラインの高速テストが可能になります。

---

---
