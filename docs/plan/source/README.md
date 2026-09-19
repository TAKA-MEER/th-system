# docs/plan/source — 原典（統合前の検討メモ）

`docs/plan/spec/` を作る前に書かれた、統合前の生の検討メモ・意思決定の経緯。
**すでに [`docs/plan/spec/`](../spec/README.md) に統合済み。** 新しく読むならまず `spec/` へ。

ここは**上書きしない**。`spec/` の記述と食い違う場合の優先順位・扱いは
[`spec/README.md` §4](../spec/README.md#4-原典との関係) を参照。

## ファイル構成

| ファイル | 内容 |
| --- | --- |
| `NewDesign.md` | 設計の原典本体（ユーザー指示による正） |
| `NewDesign-transit.md` / `-onsite.md` / `-kpi.md` / `-calib.md` / `-operationalCheck.md` | 原典の詳細ファイル |
| `NewDesign-checklist.md` | 指摘管理表（仕様ソースではない） |
| `NewVISION.md` | 意思決定の経緯（本体） |
| `onsite-*.md` / `transit-*.md` | 意思決定の経緯（走行方式・盤前動作の検討） |
| `questions-venue.md` | 導入先への確認事項 |
| `教示再生画面例.png` / `追従走行画面例.png` / `高見舎ロゴ.png` | `NewDesign.md` が参照する画面例・ロゴ画像 |

[`docs/plan/restate/`](../restate/README.md) はこのうち一部の文書を「再表現による点検」にかけた記録で、
別フォルダのまま残してある。
