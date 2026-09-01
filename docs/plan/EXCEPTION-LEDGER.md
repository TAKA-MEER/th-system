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
| W-01 | replay の**全域**ローカライズ・経路途中復帰 | WS-8B で縮小: 教示・再生とも slam_toolbox を mapping モードで連続起動し、経路は map フレームで記録・追従する（`align_path_to_current` は odom フォールバック経路のみに残存）。ただし**保存地図のロード＋全域（グローバル）ローカライズは未実装**。ロボットは毎回、教示始点とほぼ同じ物理位置（床の始点マーク）から再生を始める必要がある。`widen_search`/`global_localize`/`evt.localize_low` は依然 no-op、`ui.localize_global` の WebUI ボタンも無い。LOCALIZE は「map→base_link TF が来るまで最大 5s 待つ」実待ちになった | 期限までに実現可能性を示すため（VISION §2.5）。保存地図データベース・全域ローカライザ・READY 確認画面は範囲が大きくデモ期限に間に合わない | 始点マーク運用が必須（fallback でなく前提条件）。経路 JSON の map 座標は「地図原点＝始点マーク」基準でのみ有効。別日・別セットアップ・他ロボットへの持ち出しは不可。走行中にローカライズを失ったら始点からやり直し | WS-8C: Spec-transit §4.1 手順3〜4 の保存地図ロード＋信頼度つき初期姿勢推定（AMCL `reinitialize_global_localization` 収束判定 or slam_toolbox localization モード）、`widen_search`/`global_localize` 実装、`ui.localize_global` ボタン、READY 確認画面。`align_path_to_current` を完全撤去 | replay_runner.py / route_replay_core.py `# WAIVER(demo): W-01` | OPEN |
| W-02 | 長距離再生の終点誤差（縮小） | WS-8B で slam_toolbox が map→odom を 50Hz で連続補正するため、map フレーム経路の走行中ドリフトは実質解消。ただし**専用のドリフト補正手段（走行中の横ずれ表示・スキャンマッチ補正の設計）は未実装**で、slam_toolbox が死ぬ/発散すると `map→odom` が凍って残り区間デッドレコニングに無音で劣化する（`LOCALIZATION_LOST` フォルトは未追加）。odom フォールバック経路（slam 未起動）は従来どおり補正なし | 同上。`LOCALIZATION_LOST` 重大フォルトと横ずれ表示は WS-8C | 短経路（~3m）なら誤差はスキャンマッチ精度でほぼ一定。slam クラッシュ時の劣化に気づけない。開けた会場ではスキャンマッチ自体が劣化 | WS-8C: `LOCALIZATION_LOST` を `safety_monitor` に追加、`route_recorder` の `evt.record_broken`（slam 死）、走行中の経路横ずれ表示（Spec-transit §4.7）。実機で `replay_drift_m_per_100m` を実測し 1 経路長上限を決める | replay_runner.py `# WAIVER(demo): W-02` | OPEN |
| W-03 | 教示・再生ノードの数値パラメータ | `route_recorder` / `replay_runner` の間引き距離・pure-pursuit の先読み距離・巡航速度等をノード内リテラル既定値で持つ。`registry.yaml`（Spec-params が定める唯一の置き場）経由でない。値の出どころ（誰が・いつ・何を根拠に）が構造的に残らない | デモ最優先。registry への登録と params_generation の拡張は範囲外 | 現場調整が WebUI からできず、値の履歴が残らない（VISION §2 の前提に反する） | `route_sample_interval_m` / `replay_drift_m_per_100m` 等（既に names.json にある宣言）を `registry.yaml` に足し、両ノードを registry 駆動にする。launch で YAML を渡す | route_recorder.py / replay_runner.py の `declare_parameter` 群（`# WAIVER(demo): W-03`） | OPEN |
| W-04 | 教示経路の再保存が上書き | `route_recorder` は同じ `route_id` で `finalize` されると `<id>.json` を上書きする。Spec-transit §3.5 は「新版として保存し旧版を 1 世代残す」を定める | デモは 1 経路 1 回記録で足りる。世代管理は範囲外 | 記録し直しで旧版が消える。記録が途中で失敗すると復旧できない | `finalize` 時に既存ファイルがあれば `generation` を増やして別名保存し、旧版を 1 世代残す | route_recorder.py `_on_effect` の `finalize_route` 分岐（`# WAIVER(demo): W-04`） | OPEN |
| W-05 | `v_reverse` の既定値を 0.25 に | `registry.yaml` の `v_reverse` は `blind_clearance_m`（LiDAR 死角。placeholder）由来の `derived` → null → 生成 yaml に出ない → `obstacle_limiter.cpp` の `declare_parameter("v_reverse", 0.0)` が効き**後退が 0 にクランプ**（実機で後退不可が発覚）。registry を触ると `obstacle_stop_distance_m[v_reverse]` が計算され A2a/A11 違反で生成が止まるため、**C++ の宣言既定値だけ 0.25 m/s（場内低速相当）にした**。生成 yaml に `v_reverse` が入れば上書きされる | 実機の LiDAR は 360° で死角なし（`blind_angle_ranges` 空・`blind_calibrated: true`）。`blind_clearance_m` の実測は O-a2 待ちで未計画 | 後退の減速根拠が実測でない。死角のある構成に変えたら過大な後退速度になりうる | `blind_clearance_m` を実測 → `v_reverse` を `derived` のまま解決させ、A2a を満たすよう `obstacle_floor_distance_m` を含めて詰める。C++ 既定は 0.0 に戻す | `obstacle_limiter.cpp` の `declare_parameter("v_reverse", ...)`（`// WAIVER(demo): W-05`） | OPEN |
| W-06 | safety_monitor の `runaway` ターゲット | `bringup.launch.py` の `SAFETY_ENABLED_TARGETS` から `runaway` を除外し、`DRIVE_RUNAWAY`（指令 `/cmd_vel` と実測 `/esp32/wheel_feedback` の乖離が比 1.5 を 500ms 超で継続 → CRITICAL → ESTOP）の検知を無効化 | WiFi 経由の `/esp32/wheel_feedback` は受信ギャップ（500ms 超〜数秒）が起きる。`/cmd_vel` は obstacle_limiter が 20Hz で常時出すため、ギャップ中は「新鮮な指令 vs 走り出し直後の古い実測(≈0)」を比べ続けて走行のたびに誤発火し、教示・再生デモが 1 回も通らない（実機 2026-09-01）。他候補（回頭フェーズの `linear.x=0` + 非対称車輪／指令速度に未到達／`wheel_radius_scale=1.0` 未校正）も切り分け前 | PID 発振・エンコーダ断線・指令ゼロ時の惰走を safety_monitor が検知しなくなる。物理非常停止ボタンと ESP32 ウォッチドッグ（600ms, `config.h`）は有効なので、真の暴走の物理停止はできる。`is_runaway_condition` のロジック自体はコードに残す | ①次の実機日に `/cmd_vel` と `/esp32/wheel_feedback` を 10 秒トレースして主因を特定 ②`safety_monitor` に wheel_feedback の鮮度ゲート（実測が古いときは runaway 判定をスキップ）を実装 ③回頭フェーズ（意図的な `linear.x=0` + 旋回）を Case A から除外 ④`wheel_radius_scale` を実測較正 ⑤上記を踏まえ `runaway_hold_ms` / `runaway_ratio` を実測で右サイズ化 ⑥`SAFETY_ENABLED_TARGETS` に `runaway` を戻す | `bringup.launch.py` の `SAFETY_ENABLED_TARGETS`（`# WAIVER(demo): W-06`）/ `registry.yaml` の `runaway_*` 3 行の note | OPEN |

## 状態の凡例

- `OPEN` — バイパス中。正規実装は未着手
- `PLANNED` — 正規実装の設計 or パケットが決まった（リンクを貼る）
- `CLOSED` — 正規実装で置き換え済み（置き換えたコミットを書く）
