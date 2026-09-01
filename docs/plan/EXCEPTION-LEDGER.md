# EXCEPTION-LEDGER — 特例運用で省略・バイパスした事項の台帳

**発効 2026-09-01（ユーザー指示）。**根拠と方針は [VISION.md](../../VISION.md) §2.5。

実現可能性デモ（手動教示・教示再生の一気通貫）を最優先するため、実装・試験・安全システムの
一部を期間限定で省略・バイパスしている。**ここに記録の無い省略・バイパスをしてはいけない。**

## 運用ルール

- 省略・バイパスを 1 件行うごとに、下表へ 1 行追加する。
- コードには grep 可能なタグを残す — Python `# WAIVER(demo): <ID>`、C++ `// WAIVER(demo): <ID>`、
  JS/JSX `// WAIVER(demo): <ID>`。ID は表の左端（`W-01` …）。
- **特例解除時、全件を「正規実装で閉じる」まで `feat/demo-teach-replay` を `main` にマージしない。**
- G1（接触 0・意図しない挙動 0）／物理非常停止ボタン／ESP32 ウォッチドッグ／WebUI 専用／
  モード切替は明示操作 — これらは特例でも**バイパス禁止**（VISION.md §2.5）。台帳に載せる対象外。

## 台帳

| ID | 対象 | 省略・バイパスの内容 | 理由 | リスク / 影響 | 解除時にやること | タグの場所 | 状態 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| W-01 | replay_runner の初期姿勢推定 | LiDAR 推定・グローバルローカライズを省略し、load_route 受信時に即 evt.localize_done を発行。widen_search/global_localize は no-op。**代わりに「今いる場所を記録の始点とみなす」— 記録経路を現在 odom へ 2D 剛体変換で移す**（`align_path_to_current`。2026-09-01 追加。実機で経路が横に流れる問題への対処） | 期限までに実現可能性を示すため（VISION §2.5）。LiDAR ローカライザは未実装 | 現在地を始点と仮定するので、記録時と現在の絶対位置がずれていても経路の"形"は辿れるが、地図・障害物との整合は取れない。先導者同行・保管場所起点の運用で使う | Spec-transit §4.1 手順3〜4 の LiDAR 初期姿勢推定（探索範囲拡大・完全グローバルローカライズ）を実装し、evt.localize_low/evt.localize_done を推定結果から出す。`align_path_to_current` は撤去 | replay_runner.py / route_replay_core.py `# WAIVER(demo): W-01` | OPEN |
| W-02 | 再生中のドリフト補正なし | pure-pursuit はオドメトリ＋ジャイロの /odom だけで経路を辿り、走行中の自己位置補正をしない（既知の課題 CL-T-4） | 同上。走行中ローカライゼーションは未実装 | 長距離再生で終点誤差が累積する。1 経路の長さを実測で制限して運用 | Spec-transit §4.5 のとおり走行中ドリフト補正手段を設計・実装する | replay_runner.py `# WAIVER(demo): W-02`（該当箇所にコメント） | OPEN |
| W-03 | 教示・再生ノードの数値パラメータ | `route_recorder` / `replay_runner` の間引き距離・pure-pursuit の先読み距離・巡航速度等をノード内リテラル既定値で持つ。`registry.yaml`（Spec-params が定める唯一の置き場）経由でない。値の出どころ（誰が・いつ・何を根拠に）が構造的に残らない | デモ最優先。registry への登録と params_generation の拡張は範囲外 | 現場調整が WebUI からできず、値の履歴が残らない（VISION §2 の前提に反する） | `route_sample_interval_m` / `replay_drift_m_per_100m` 等（既に names.json にある宣言）を `registry.yaml` に足し、両ノードを registry 駆動にする。launch で YAML を渡す | route_recorder.py / replay_runner.py の `declare_parameter` 群（`# WAIVER(demo): W-03`） | OPEN |
| W-04 | 教示経路の再保存が上書き | `route_recorder` は同じ `route_id` で `finalize` されると `<id>.json` を上書きする。Spec-transit §3.5 は「新版として保存し旧版を 1 世代残す」を定める | デモは 1 経路 1 回記録で足りる。世代管理は範囲外 | 記録し直しで旧版が消える。記録が途中で失敗すると復旧できない | `finalize` 時に既存ファイルがあれば `generation` を増やして別名保存し、旧版を 1 世代残す | route_recorder.py `_on_effect` の `finalize_route` 分岐（`# WAIVER(demo): W-04`） | OPEN |
| W-05 | `v_reverse` の既定値を 0.25 に | `registry.yaml` の `v_reverse` は `blind_clearance_m`（LiDAR 死角。placeholder）由来の `derived` → null → 生成 yaml に出ない → `obstacle_limiter.cpp` の `declare_parameter("v_reverse", 0.0)` が効き**後退が 0 にクランプ**（実機で後退不可が発覚）。registry を触ると `obstacle_stop_distance_m[v_reverse]` が計算され A2a/A11 違反で生成が止まるため、**C++ の宣言既定値だけ 0.25 m/s（場内低速相当）にした**。生成 yaml に `v_reverse` が入れば上書きされる | 実機の LiDAR は 360° で死角なし（`blind_angle_ranges` 空・`blind_calibrated: true`）。`blind_clearance_m` の実測は O-a2 待ちで未計画 | 後退の減速根拠が実測でない。死角のある構成に変えたら過大な後退速度になりうる | `blind_clearance_m` を実測 → `v_reverse` を `derived` のまま解決させ、A2a を満たすよう `obstacle_floor_distance_m` を含めて詰める。C++ 既定は 0.0 に戻す | `obstacle_limiter.cpp` の `declare_parameter("v_reverse", ...)`（`// WAIVER(demo): W-05`） | OPEN |
| W-06 | safety_monitor の `runaway` ターゲット | `bringup.launch.py` の `SAFETY_ENABLED_TARGETS` から `runaway` を除外し、`DRIVE_RUNAWAY`（指令 `/cmd_vel` と実測 `/esp32/wheel_feedback` の乖離が比 1.5 を 500ms 超で継続 → CRITICAL → ESTOP）の検知を無効化 | WiFi 経由の `/esp32/wheel_feedback` は受信ギャップ（500ms 超〜数秒）が起きる。`/cmd_vel` は obstacle_limiter が 20Hz で常時出すため、ギャップ中は「新鮮な指令 vs 走り出し直後の古い実測(≈0)」を比べ続けて走行のたびに誤発火し、教示・再生デモが 1 回も通らない（実機 2026-09-01）。他候補（回頭フェーズの `linear.x=0` + 非対称車輪／指令速度に未到達／`wheel_radius_scale=1.0` 未校正）も切り分け前 | PID 発振・エンコーダ断線・指令ゼロ時の惰走を safety_monitor が検知しなくなる。物理非常停止ボタンと ESP32 ウォッチドッグ（600ms, `config.h`）は有効なので、真の暴走の物理停止はできる。`is_runaway_condition` のロジック自体はコードに残す | ①次の実機日に `/cmd_vel` と `/esp32/wheel_feedback` を 10 秒トレースして主因を特定 ②`safety_monitor` に wheel_feedback の鮮度ゲート（実測が古いときは runaway 判定をスキップ）を実装 ③回頭フェーズ（意図的な `linear.x=0` + 旋回）を Case A から除外 ④`wheel_radius_scale` を実測較正 ⑤上記を踏まえ `runaway_hold_ms` / `runaway_ratio` を実測で右サイズ化 ⑥`SAFETY_ENABLED_TARGETS` に `runaway` を戻す | `bringup.launch.py` の `SAFETY_ENABLED_TARGETS`（`# WAIVER(demo): W-06`）/ `registry.yaml` の `runaway_*` 3 行の note | OPEN |

## 状態の凡例

- `OPEN` — バイパス中。正規実装は未着手
- `PLANNED` — 正規実装の設計 or パケットが決まった（リンクを貼る）
- `CLOSED` — 正規実装で置き換え済み（置き換えたコミットを書く）
