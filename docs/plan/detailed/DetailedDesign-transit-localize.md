# 教示再生の全域ローカライズ・経路途中からの復帰

**結論**: slam_toolbox の localization モードのまま、保存地図に対する二段構え
（自前スキャンマッチの粗探索 → 上位 1 候補だけ `deserialize_map` で確定）で
全域ローカライズを行う。AMCL への置換はしない。確度は自前一致度スコアで測る。
FSM の行は足りているが、発火元（`evt.localize_low` の発行者・画面ボタン・探索の実体）
が無いので足す。詳細の根拠は [§根拠](DetailedDesign-transit-localize-options.md)。

**正本**: `Spec-transit.md` §4.1 手順 3〜5・§4.2・§4.5、`Spec-webui.md` §3.7、
台帳 W-01。**spec は書き換えない。**食い違いは報告（`.briefs/tmp/report.md`）に書く。

## 1. 方式の結論

| 項目 | 決定 |
| --- | --- |
| 推奨 | **案 A: slam_toolbox のまま二段構え**（粗探索 → `LOCALIZE_AT_POSE` で確定） |
| 捨てる | 案 B: AMCL 置換（地図形式・TF 競合・工数で非推奨。将来の置換候補として記録のみ） |
| 捨てる | 案 C: 周辺拡大のみ（spec §4.1 手順 4「最初から完全グローバルも選べる」を満たさない） |
| 確度 | **自前一致度スコア**（`/scan` と凍結地図の一致率。共分散は使えないことが実測済み） |
| 途中復帰 | 確定姿勢に最も近い**前向きの**経路点を再開 index とする（向きで前後を解く） |
| 安全 | 探索中は既存の「計画的再起動の保留」を拡張（上限付き）。READY 中も監視は効かせたまま |

比較の軸（CPU・時間・失敗の見分け）は [§1](DetailedDesign-transit-localize-options.md)。

## 2. FSM: 行は足りるが発火元が無い

| 既存の行 | 状態 | 足りないもの |
| --- | --- | --- |
| `T-REPLAY-02`（`evt.localize_low` → `widen_search`） | 待ち受けだけある | **発行者がいない。**確度評価（replay_runner 拡張）が `arg_json{score}` 付きで出す |
| `T-REPLAY-03`（`ui.localize_global` → `global_localize`） | 同上 | **画面ボタンが無い**（S-14 に足す） |
| `widen_search` / `global_localize` effect | no-op（`WAIVER W-01`） | 探索の実体を実装する（中身は [§2](DetailedDesign-transit-localize-options.md)） |
| 経路途中からの再開 | 行が無い | `load_route` に `from_index` 任意引数（effect 転送が passthrough なら行追加は不要。要確認） |

## 3. 作業パケットへの分割（1 パケット＝1 ブランチ）

| # | 内容 | 触るファイル | 完了条件（緑になる試験） | Gazebo / 実機 |
| --- | --- | --- | --- | --- |
| P1 | 自前スコアの純関数＋単体試験 | `route_replay_core.py`（追加のみ）、`test_route_replay_core.py` | 追加したテストが host pytest で緑 | Gazebo 不要 |
| P2 | `widen_search` / `global_localize` 実装＋`evt.localize_low` 発行 | `replay_runner.py`、`slam_control.py`（探索 API）、`transitions.yaml` は不変 | `test_route_replay_core.py`＋新規ノード試験。故障注入 13 が緑のまま | Gazebo 可（新シナリオ `replay_localize`）／所要時間の実値は実機 |
| P3 | S-14: 「完全グローバルで探す」ボタン・確度表示・READY 確認 | `S14Replay.jsx` 系、`DetailedDesign-webui.md` 追記 | 画面試験（W-19 の④と統合可） | Gazebo 可（表示確認） |
| P4 | 途中復帰（`from_index` 再開＋向き合わせ再利用） | `replay_runner.py`、`route_replay_core.py` | P1 の延長＋Gazebo 通し試験 | Gazebo 可／ずれ量の実値は実機 |
| P5 | registry placeholder＋保留拡張 | `registry.yaml`、`params_generation.py`、`localization_health*`、試験 | `test_params_assertions.py`、`test_planned_restart_node.py` 系が緑 | Gazebo 可 |
| P6 | 実機で閾値・時間・CPU を実測し placeholder を詰める | 測定のみ（コード変更なし） | 測定記録（`data/meas*`）＋ registry 更新は別パケット | **実機のみ** |

Gazebo で確かめられること／実機でしか分からないことの線引きは
[§5](DetailedDesign-transit-localize-options.md)。

## 4. 台帳 W-01 との対応

| W-01「解除時にやること」（WS-8C） | 本設計 |
| --- | --- |
| 保存地図ロード＋信頼度つき初期姿勢推定 | 対応（§1・P1/P2。`reinitialize_global_localization` ではなく slam 二段構えに読み替え） |
| `widen_search` / `global_localize` 実装 | 対応（P2） |
| `ui.localize_global` ボタン | 対応（P3） |
| READY 確認画面 | 対応（P3。表示内容は [§4](DetailedDesign-transit-localize-options.md)） |
| `align_path_to_current` を完全撤去 | **残す。**odom フォールバック経路（slam 未起動時の保険）を消す理由が無い。W-01 は条件付きクローズとし、残件として報告に記録 |

## 5. 逆引き

| 正本 | ここでの反映 |
| --- | --- |
| `Spec-transit.md` §4.1 手順 3〜5・§4.2・§4.2.1〜4.2.3・§4.5 | §1・[§2〜§4](DetailedDesign-transit-localize-options.md) |
| `Spec-webui.md` §3.7 S-14 | [§4](DetailedDesign-transit-localize-options.md)（spec に無い詳細は「設計で決めた」と明記） |
| `Spec-safety.md` §3.5・§3.5.0（A/C/B′・計画的再起動の保留） | [§6](DetailedDesign-transit-localize-options.md) |
| `Spec-modes.md` §9（人物追跡は REPLAY で動かさない） | 探索中に人検出を起動しない（CPU 予算の前提） |
| 台帳 W-01・W-02（`replay_drift`）・W-19（逆再生） | §4・[§7](DetailedDesign-transit-localize-options.md) |
