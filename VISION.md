# VISION.md — TH システム完成形

このドキュメントは、TH システム（配電盤上部確認ロボット移動機構）の**目指す完成形**を記述する。
読者は開発者本人と、コーディングを行う AI。

**運用ルール**: 方針を変更する際は、コードを修正する前に必ず本書（または本書が委譲した先の設計書）を更新する。
コードと文書が食い違う場合は文書が正であり、実装を追いつかせる（CLAUDE.md「方針変更時のルール」）。
`docs/architecture.md` は現状実装の記述（as-built）であり、本書は目指す姿（to-be）である。

---

## 0. 完成形の定義は完全設計書・詳細設計書に従う

**2026-08-18 決定。**完成形の全体像は `docs/plan/spec/`（**完全設計書**）と
`docs/plan/detailed/`（**詳細設計書**）に書き切ってある。
**本書はその内容を要約も再掲もしない。**両者は `docs/plan/` 配下にあるが、
**確定した方針としてここで採用する**（したがって「未確定の検討メモ」ではない）。

| 知りたいこと | 見る文書 |
| --- | --- |
| **何が・どう振る舞うべきか。どういう値が要るか** | [完全設計書](docs/plan/spec/README.md) — 本体は [Spec.md](docs/plan/spec/Spec.md) |
| 目標 G1〜G5・設計思想 SD-1〜SD-7・走行方式 7・モード 18・1 日の流れ | [Spec.md](docs/plan/spec/Spec.md) |
| モード定義と遷移表（状態モデルの正本） | [Spec-modes.md](docs/plan/spec/Spec-modes.md) §3.1 |
| 安全チェーン 4 層・障害物・非常停止・通信断 | [Spec-safety.md](docs/plan/spec/Spec-safety.md) |
| 画面一覧（15）と見た目 | [Spec-webui.md](docs/plan/spec/Spec-webui.md) ＋ [mockup/index.html](docs/plan/spec/mockup/index.html) |
| **ノード名・トピック名・ファイル構成・アルゴリズム・作業パケット** | [詳細設計書](docs/plan/detailed/README.md) — 本体は [DetailedDesign.md](docs/plan/detailed/DetailedDesign.md) |
| 名前（発明してはいけない） | [DetailedDesign-names.md](docs/plan/detailed/DetailedDesign-names.md) |
| 既存コードを消してよいか | [DetailedDesign-reuse.md](docs/plan/detailed/DetailedDesign-reuse.md) §1 |
| 何が決まっていないか | [Spec-open.md](docs/plan/spec/Spec-open.md) §5 ／ [DetailedDesign-open.md](docs/plan/detailed/DetailedDesign-open.md) §4 |
| 実装の順番・進捗 | [ImplementationPlan.md](docs/plan/ImplementationPlan.md) |

**本書に残すのは次の 3 つだけ**: 目的とユースケース（§1）、揺るがない前提（§2）、
両設計書が扱っていない事項（§3）。

**矛盾したときの扱い**: §2 が最優先。それ以外で本書と両設計書が食い違ったら**両設計書が正**とし、
本書側の記述を消す（要約を残すと二重管理になる）。

### 0.1 旧版（〜2026-08-17）からの主な変更

旧 VISION.md の §2〜§6・§8・§9 は両設計書に置き換わった。差分の概要のみ記録する。

| 項目 | 旧 VISION.md | 現在（両設計書） |
| --- | --- | --- |
| モード | 9（`FOLLOWING_MAPLESS` 中心） | **18**。`IDLE` ハブ＆スポーク（[Spec-modes.md](docs/plan/spec/Spec-modes.md)） |
| 保管場所⇔試験場の移動 | 地図なし追従の 1 本 | **走行方式 7 つを全部作って社内試験で絞る**（[Spec-transit.md](docs/plan/spec/Spec-transit.md)） |
| フォルト時の扱い | 全フォルトで `IDLE` へ強制遷移 | **2 階級**。回復 → `PAUSE` ／ 重大 → `ESTOP`。**`IDLE` へは落とさない**（[Spec-safety.md](docs/plan/spec/Spec-safety.md) §3.5） |
| 安全チェーン | twist_mux ＋ ESP32 ウォッチドッグ | **4 層**（物理 E-Stop / ESP32 WD 600ms / 安全監視 100ms / 状態遷移）＋ **障害物リミッタ新設**（[Spec-safety.md](docs/plan/spec/Spec-safety.md) §1） |
| 手動介入 | `MANUAL` モードへ切り替える | モードを変えずに**ジョグ介入**（リース式・`jog_gate` で構造的に塞ぐ） |
| WebUI | 3 タブ（運用/準備/診断） | **15 画面**（[Spec-webui.md](docs/plan/spec/Spec-webui.md)）。3 タブ構成は廃止 |
| 配電盤の位置 | `panels.yaml` を起動時に読む | **2 点指示によるランタイム登録**（[Spec-onsite.md](docs/plan/spec/Spec-onsite.md)） |
| 始業点検・校正 | CLI 4 本 | **サービス駆動のウィザード**（[Spec-checks.md](docs/plan/spec/Spec-checks.md)） |
| 数値パラメータ | 各 YAML に直書き | **`registry.yaml` が唯一の置き場。**未確定値 (c) は起動を止める（[Spec-params.md](docs/plan/spec/Spec-params.md)） |
| 実装状況トラッキング | 本書 §9 の表 | [ImplementationPlan.md](docs/plan/ImplementationPlan.md) ＋ [DetailedDesign-packets.md](docs/plan/detailed/DetailedDesign-packets.md)（48 パケット） |

---

## 1. 目的とユースケース

配電盤の上部確認試験において、カメラ昇降機構を搭載したクローラーロボットが試験員に同行し、
試験員の指示で配電盤前に移動して撮影作業を支援する。**原則 1 人で全作業が完結する**ことを目標にする。

1. 保管場所で電源を入れ、疎通確認のうえ**走行方式を 1 つ選んで**試験場まで移動する
2. 試験場では**前日に**地図を作り、待機場所と配電盤を登録して保存する
3. **当日**は待機場所で待ち、試験員の要請があったとき**のみ**盤前へ移動する
   （登録済みのピンを選ぶ／その場で呼び寄せる）
4. 盤前で撮影作業（カメラ昇降は別担当）。完了したら待機場所へ戻る
5. 全作業終了後、**往路と同じ手順・同じ UI で**保管場所へ戻る

**設計方針の要点**: 試験場内で常に追従すると試験員の作業の邪魔になり、狭い場所での衝突リスクも増える。
このため**追従するのは保管場所⇔試験場の移動時だけ**とし、試験場内は待機 + 要請時のみ移動とする。

手順の詳細は [Spec.md](docs/plan/spec/Spec.md) §9、試験場内は [Spec-onsite.md](docs/plan/spec/Spec-onsite.md)、
起動・終了は [Spec-ops.md](docs/plan/spec/Spec-ops.md)。

---

## 2. 揺るがない前提

**ここだけは両設計書より優先する。**両設計書がこれに反する記述を持っていたら、そちらを直す。

- **G1（人・物への接触 0 件、意図しない挙動 0 件）は他のすべてに優先する。**
  G1 を犠牲にして G2〜G5 を満たしてはいけない（[Spec.md](docs/plan/spec/Spec.md) §3）
- **速度指令は必ず優先度つきの多重化を経由する。**駆動へ直接指令するノードを作ってはいけない
  （SD-6。詳細設計では `/cmd_vel` を publish してよいのは `obstacle_limiter` だけ）
- **安全チェーンの上位層は下位層の故障に依存しない。**
  物理非常停止ボタンと ESP32 ウォッチドッグは**開発モードでも無効化できない**
  （[Spec-safety.md](docs/plan/spec/Spec-safety.md) §1・§10）
- **操作系は WebUI のみ。**専用アプリ・コマンドライン操作を運用手順に含めない（SD-1）。
  非常停止はどんなときでも押せ、何にも覆われない（SD-3）
- **ロボットが試験員の意図しないタイミングで動き出さないこと。**モードの切替はすべて明示操作による
- **測らないと分からない値に事前の目標を置かない**（SD-7。[Spec-params.md](docs/plan/spec/Spec-params.md)）
- **数値は現場で何度も変わる前提で作る。**機体寸法・速度上限・追従距離は実機の動きを見ながら詰める。
  **WebUI から調整でき、値の出どころ（誰が・いつ・何を根拠に決めたか）が構造的に残る**こと。
  **ただし調整できるのは停止中（`IDLE` / `MANUAL`）だけ**——走行中に安全チェーンの閾値が変わる経路を作らない
  （2026-08-18 決定）

---

## 3. 両設計書が扱っていない事項

**[docs/voice-and-audience.md](docs/voice-and-audience.md) が正。**

音声アナウンスと観客向け表示（`?view=audience`）は、詳細設計書が
[DetailedDesign-reuse.md](docs/plan/detailed/DetailedDesign-reuse.md) §2.12 で
`voice/` 一式・`audience/` 一式とも**「維持」**とだけ決めており、中身の設計をどちらも持っていない。
旧 VISION.md §6.3・§7 の内容をそのまま上記へ移した。

**移行時に引き直しが要る**: 音声の発火トリガは旧 9 モード体系を前提にしている。
特にフォルトが 2 階級になり `IDLE` へ落とさなくなったため、安全通知の文案が現状のままでは意味が合わない。

---

## 4. 実装状況

[ImplementationPlan.md](docs/plan/ImplementationPlan.md) が正。
作業パケット単位の進捗は [DetailedDesign-packets.md](docs/plan/detailed/DetailedDesign-packets.md)。
