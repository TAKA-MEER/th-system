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
| W-01 | replay_runner の初期姿勢推定 | LiDAR 推定・グローバルローカライズを省略し、load_route 受信時に即 evt.localize_done を発行。widen_search/global_localize は no-op。記録始点にロボットがいる前提 | 期限までに実現可能性を示すため（VISION §2.5）。LiDAR ローカライザは未実装 | 記録始点から大きくずれた位置で再生開始すると経路を正しく辿れない。先導者同行・保管場所起点の運用で回避 | Spec-transit §4.1 手順3〜4 の LiDAR 初期姿勢推定（探索範囲拡大・完全グローバルローカライズ）を実装し、evt.localize_low/evt.localize_done を推定結果から出す | replay_runner.py `# WAIVER(demo): W-01` | OPEN |
| W-02 | 再生中のドリフト補正なし | pure-pursuit はオドメトリ＋ジャイロの /odom だけで経路を辿り、走行中の自己位置補正をしない（既知の課題 CL-T-4） | 同上。走行中ローカライゼーションは未実装 | 長距離再生で終点誤差が累積する。1 経路の長さを実測で制限して運用 | Spec-transit §4.5 のとおり走行中ドリフト補正手段を設計・実装する | replay_runner.py `# WAIVER(demo): W-02`（該当箇所にコメント） | OPEN |
| W-03 | 教示・再生ノードの数値パラメータ | `route_recorder` / `replay_runner` の間引き距離・pure-pursuit の先読み距離・巡航速度等をノード内リテラル既定値で持つ。`registry.yaml`（Spec-params が定める唯一の置き場）経由でない。値の出どころ（誰が・いつ・何を根拠に）が構造的に残らない | デモ最優先。registry への登録と params_generation の拡張は範囲外 | 現場調整が WebUI からできず、値の履歴が残らない（VISION §2 の前提に反する） | `route_sample_interval_m` / `replay_drift_m_per_100m` 等（既に names.json にある宣言）を `registry.yaml` に足し、両ノードを registry 駆動にする。launch で YAML を渡す | route_recorder.py / replay_runner.py の `declare_parameter` 群（`# WAIVER(demo): W-03`） | OPEN |
| W-04 | 教示経路の再保存が上書き | `route_recorder` は同じ `route_id` で `finalize` されると `<id>.json` を上書きする。Spec-transit §3.5 は「新版として保存し旧版を 1 世代残す」を定める | デモは 1 経路 1 回記録で足りる。世代管理は範囲外 | 記録し直しで旧版が消える。記録が途中で失敗すると復旧できない | `finalize` 時に既存ファイルがあれば `generation` を増やして別名保存し、旧版を 1 世代残す | route_recorder.py `_on_effect` の `finalize_route` 分岐（`# WAIVER(demo): W-04`） | OPEN |

## 状態の凡例

- `OPEN` — バイパス中。正規実装は未着手
- `PLANNED` — 正規実装の設計 or パケットが決まった（リンクを貼る）
- `CLOSED` — 正規実装で置き換え済み（置き換えたコミットを書く）
