# VISION.md — 役目を終えた文書（2026-09-20）

**この文書の内容はすべて他所へ移した。新しく書き足さないこと。**

完成形（何が・どう振る舞うべきか）の正本は **[`docs/plan/spec/`](docs/plan/spec/README.md)** である。
どこに何が書いてあるかは [spec/README.md](docs/plan/spec/README.md) §1.2 の索引を見る。

## なぜ畳んだか

本書は 1240 行まで肥大し、毎回の作業で読む文書としては過剰になっていた
（うち 905 行が日付つきの実機フィードバックの記録）。調べたところ**本書にしかない要素は
2 つだけ**で、残りは他文書と重複しているか、`spec/` へ反映されるべきなのに溜まっていた設計だった。
後者を `spec/` へ流し込み終えたので、本書を畳む（ユーザー決定 2026-09-20）。

## 移設先の対応

| 旧 VISION.md の節 | 移設先 |
| --- | --- |
| §0 索引「どこに何が書いてあるか」 | [spec/README.md](docs/plan/spec/README.md) §1・§1.2 |
| §0.1 旧版（〜2026-08-17）からの差分表 | 削除（git 履歴に残る） |
| §1 目的とユースケース | [Spec.md](docs/plan/spec/Spec.md) §1・§9 |
| §2 揺るがない前提 | [Spec.md](docs/plan/spec/Spec.md) §3（G1）・§5（SD-1〜SD-9）／[Spec-safety.md](docs/plan/spec/Spec-safety.md) §10。**本書にしか無かった 2 項目は SD-8（モード切替は明示操作）と SD-9（数値は停止中だけ変更・出どころが残る）として移した** |
| §2.5 特例運用（統治・フェーズ定義・許可範囲） | [EXCEPTION-LEDGER.md](docs/plan/EXCEPTION-LEDGER.md) 冒頭 |
| §2.5「実機フィードバックによる設計変更」24 件 | [Spec-safety.md](docs/plan/spec/Spec-safety.md) §2.5・§3・§3.5.1・§3.5.2・§6.2 ／ [Spec-transit.md](docs/plan/spec/Spec-transit.md) §2.2・§4.2〜§4.5 ／ [Spec-webui.md](docs/plan/spec/Spec-webui.md) §3.7・§3.15 ／ [Spec-onsite.md](docs/plan/spec/Spec-onsite.md) §2.1・§2.1.1・§3.5・§6.1〜§6.2 ／ [Spec-modes.md](docs/plan/spec/Spec-modes.md) §3.1.1 ／ [Spec.md](docs/plan/spec/Spec.md) §6.1 |
| §3 音声アナウンス・観客向け表示 | [docs/voice-and-audience.md](docs/voice-and-audience.md)（旧 §6.3 → §1、旧 §7.x → §2.x） |
| §4 実装状況 | [ImplementationPlan.md](docs/plan/ImplementationPlan.md) |

**経緯そのもの**（症状 → 原因 → 決定に至る議論）は git 履歴に残っている。
`git log -p --follow VISION.md` で追える。

## 文書の役割分担

| 文書 | 役割 |
| --- | --- |
| [`docs/plan/spec/`](docs/plan/spec/README.md) | **完成形の正本。**何が・どう振る舞うべきか |
| [`docs/plan/detailed/`](docs/plan/detailed/README.md) | 詳細設計。ノード名・トピック名・作業パケット |
| [`CLAUDE.md`](CLAUDE.md) | 作業のしかた・環境の癖・コードの不変ルール |
| [`docs/architecture.md`](docs/architecture.md) | 現状実装の保守・拡張ガイド（as-built） |
| [`docs/plan/EXCEPTION-LEDGER.md`](docs/plan/EXCEPTION-LEDGER.md) | デモ特例で省略・バイパスしたままの事項。**未クローズが何かはこれが唯一の正** |
