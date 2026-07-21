# 地図なし追従走行（FOLLOWING_MAPLESS）試験手順書

[← README に戻る](../README.md)

## 目的・対象範囲

[VISION.md](../VISION.md) §3 の `FOLLOWING_MAPLESS`（モード 7、保管場所⇔試験場の移動専用、
地図・Nav2 に依存せず LiDAR 脚検知の相対位置のみで追従）が、完成形の挙動要件を満たすことを
実センサ・実際の人の動き・安全チェーン連携込みで確認する。

対象外: `test_mapless_follow_logic.py` が既に境界値まで網羅している純粋ロジック（状態遷移の
閾値、Pure Pursuit 計算式など）はここでは扱わない。本手順書はロジック単体では検出できない
「実センサのノイズ」「実際の人の動き」「他ノードとの連携」「安全チェーンとの相互作用」に絞る。

対象パッケージ: `th_planning`（`follow_planner_mapless.py` / `mapless_follow_core.py`）、
`th_safety`（`safety_monitor`）、`th_mode_manager`。

## 事前準備

1. 使用するパラメータ値を記録用ファイルに転記する（現場で `planning_params.yaml` を調整した
   場合は既定値ではなく実際の値を記録すること）。
   ```bash
   ros2 param get /follow_planner_mapless stop_distance
   ros2 param get /follow_planner_mapless resume_distance
   ros2 param get /follow_planner_mapless obstacle_check_distance_m
   ros2 param get /follow_planner_mapless v_max
   ros2 param get /follow_planner_mapless lookback_distance
   ```
2. 対象コミットハッシュを記録する: `git rev-parse --short HEAD`
3. シム実施の場合: `ros2 launch th_bringup gazebo.launch.py scenario:=<name>`
   （項目ごとの推奨シナリオは各項目に記載）
4. 実機実施の場合: [docs/operation.md](operation.md) の起動手順に従い、`/safety/estop` が
   解除済み・フォルトなしの状態から開始する。
5. モード切替:
   ```bash
   ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
     "{requested_mode: 7, requester: 'cli'}"
   ```

## 実施順序

シムで 1・2・4・5・8 を先に確認し、ロジック起因の不具合を安価に洗い出してから実機で
全項目（3・6・7・9・10 はシムでは十分に再現できないため実機必須）を実施する。

## 試験項目

### 1. 基本追従（直進・カーブ・停止発進）

- **推奨環境**: 実機、または `wide_area` シナリオ
- **手順**: 試験員が直進 → カーブ → 停止 → 発進を含む経路を通常歩行速度で歩く。ロボットは
  1〜2m 程度後方から追従させる。
- **観察**: `ros2 topic echo /cmd_vel` で速度指令、目視で追従挙動を確認。
- **合格基準**: 追従遅れ（試験員とロボットの距離）が発散せず一定範囲に収束する。カーブで
  ロボットが軌跡をショートカットせず trail を辿る。

### 2. 接近時の停止・再開（ヒステリシス）

- **推奨環境**: シム `wide_area`（stop 1.0m / resume 1.3m の検証用シナリオ）または実機
- **手順**: 試験員が振り返ってロボットに近づく → 停止確認 → 離れる → 再追従開始、を 3 回以上
  繰り返す。
- **合格基準**: `stop_distance` ± 0.2m 程度で停止、`resume_distance` 到達で再開。3 回とも
  ハンチング（停止/発進を細かく繰り返す）が発生しない。

### 3. 急加減速抑制

- **推奨環境**: 実機
- **手順**: 試験員が急に立ち止まる／急に歩き出す動作を数回行う。
- **合格基準**: 指令速度の変化が `max_linear_accel_mps2` / `max_linear_decel_mps2` の
  レートに収まり、機体の急停止・急発進による転倒・スリップが発生しない。

### 4. 進路上障害物での停止

- **推奨環境**: シム `cluttered` または実機（台車・什器を進路上に設置）
- **手順**: 追従経路上に人以外の障害物を置く。あわせて試験員が障害物近くを歩き、試験員自身の
  脚が障害物として誤検知されないことも確認する。
- **合格基準**: `obstacle_check_distance_m` 未満で停止し接触しない。試験員の脚だけがある
  区間では停止しない（脚除外ロジックが機能）。

### 5. ロスト・再捕捉

- **推奨環境**: シム `lost_reacquire` または実機（柱の陰・すれ違いで遮蔽）
- **手順**: 試験員が遮蔽物の裏に一時的に隠れる → 再出現。
- **合格基準**: ロスト中は指令が 0（探索走行で動き回らない）。再捕捉後、trail の不連続に
  よる急な進路変更や飛び出しがなく、0 から滑らかに再加速する。

### 6. 長時間ロスト時のフォルト連携

- **推奨環境**: 実機推奨（`person_timeout_ms` 2500ms の実測込みで確認するため）
- **手順**: 試験員が `person_timeout_ms`（既定 2500ms）を超えて検知範囲外に出る。
- **合格基準**: `PERSON_TRACKER_LOST` フォルトが発生し `mode_manager` が強制的に `IDLE` へ
  遷移する。遷移時の停止が安全（急停止で機体が不安定にならない）。

### 7. 誤切替防止

- **推奨環境**: 実機（第三者に協力を依頼）
- **手順**: 試験員の追従中、別の人物がロボットの近くを横切る・並んで歩く。
- **合格基準**: 追従対象が別人に乗り移らない。

### 8. 姿勢・スキャン未確立時のフェイルセーフ

- **推奨環境**: シムまたは実機
- **手順**: モード 7 への切替直後（TF・`/scan_filtered` が確立する前）の挙動を観察する。
- **合格基準**: 指令を出さず停止したままである（`no_pose` / `no_scan` reason で待機）。

### 9. 安全系との連携

- **推奨環境**: 実機必須
- **手順**: 追従中に以下を個別に発生させる。
  1. E-Stop（物理ボタン）
  2. E-Stop（タブレット UI）
  3. LiDAR 切断（ケーブル抜線など安全な方法で）
  4. ESP32 通信断
- **合格基準**: いずれも `twist_mux` によるゼロ固定が実測で 100ms 級（`check_period_ms`
  100ms + 数 ms）で発生する。mapless 固有ロジックが安全チェーンを迂回していない
  （`/cmd_vel` へ直接 publish するノードがない、という不変ルールの再確認）。

### 10. 実運用シナリオ通し試験

- **推奨環境**: 実機（VISION.md 想定の保管場所⇔試験場の実距離）
- **手順**: [docs/simulation.md](simulation.md) の `panel_shuttle` 手順に準じ、往路（保管場所→
  試験場）・復路（試験場→保管場所）を通しで実施する。
- **合格基準**: 往復を通じて試験員による異常介入（MANUAL 切替・E-Stop）が 0 回。追従対象の
  誤切替が 0 件。

## 記録

各実施ごとに [docs/mapless_follow_test_record.md](mapless_follow_test_record.md) を
コピーして日付入りのファイル名（例: `test_records/mapless_follow_2026-07-21.md`）で保存し、
結果を記入する。
