# docs/plan/spec — 完全設計書

`docs/plan/source/NewDesign.md` と `docs/plan/restate/NewDesign-restated.md` を**両方とも正**として統合し、
散在した記述をまとめて矛盾を解消した設計書。

**本体は [Spec.md](Spec.md)。** そこから各詳細ファイルへ辿れる。

---

## 1. 位置づけ

| 文書 | 役割 |
| --- | --- |
| `VISION.md`（リポジトリ直下） | ユーザーが目指す**完成形**。**確定した方針はここが正** |
| `docs/architecture.md` | **現状実装**の保守・拡張ガイド（as-built） |
| `docs/plan/` | **未確定の検討メモ。**`VISION.md` を上書きしない |
| **`docs/plan/spec/`（ここ）** | **完全設計書。**検討メモを統合し矛盾を解消したもの。**まだ `VISION.md` ではない** |

**この設計書は `docs/plan/` 配下の検討段階の文書である。**
方針として確定した時点で、`CLAUDE.md`「方針変更時のルール」に従い、
**コードを触る前に `VISION.md` を更新する。**

### 1.1 次の工程

本設計書は**詳細設計の入力**である。したがって次のものは**意図的に書いていない**。

* ROS2 のノード名・トピック名・サービス名
* パラメータファイルのパス・ファイル構成
* rosbridge の配線、WebUI の実装技術
* アルゴリズムの実装方法

書いてあるのは「**何が・どう振る舞うべきか**」と「**どういう値が要るか**」だけである。

---

## 2. ファイル構成

| ファイル | 内容 | 行数の目安 |
| --- | --- | --- |
| **[Spec.md](Spec.md)** | **本体。**対象・用語・目標・設計思想・全体構成・走行方式一覧・モード一覧・目次 | 200 行以内を維持 |
| [Spec-modes.md](Spec-modes.md) | **モード定義と遷移表。状態モデルの正本（§3.1）。**ゾーン・ジョグ介入・人物追跡の要否 | — |
| [Spec-transit.md](Spec-transit.md) | 7 走行方式の完全仕様と比較 | — |
| [Spec-onsite.md](Spec-onsite.md) | 試験場内（前日の地図作成・登録／当日の盤前移動・呼び寄せ）・配電盤前 | — |
| [Spec-safety.md](Spec-safety.md) | 安全チェーン・障害物・非常停止・異常ウィンドウ・通信断 | — |
| [Spec-ops.md](Spec-ops.md) | 導入・起動・疎通確認・終了手順 | — |
| [Spec-checks.md](Spec-checks.md) | 始業点検 4 項目・校正 4 項目 | — |
| [Spec-webui.md](Spec-webui.md) | 画面一覧（15）・遷移図・ウィンドウ・共通部品 | — |
| [Spec-params.md](Spec-params.md) | パラメータ一覧・逆算の連鎖・保存先 | — |
| **[Spec-open.md](Spec-open.md)** | **矛盾の解決記録・未確定事項・既存 ID の処理状況** | — |
| [mockup/index.html](mockup/index.html) | **WebUI の見た目。**ブラウザで開ける単一ファイル | — |

`docs/plan/README.md` の運用ルール（本体は結論と表だけ・200 行超で分割）に従っている。

---

## 3. 読む順番

| 目的 | 読む順 |
| --- | --- |
| 全体を掴みたい | [Spec.md](Spec.md) だけ |
| **何が決まっていないか知りたい** | [Spec-open.md](Spec-open.md) §5 |
| **なぜこう決めたのか知りたい** | [Spec-open.md](Spec-open.md) §1〜§3 |
| 実装に入りたい（詳細設計） | [Spec-modes.md](Spec-modes.md) → 担当する領域のファイル → [Spec-params.md](Spec-params.md) |
| WebUI を作りたい | [Spec-webui.md](Spec-webui.md) ＋ [mockup/index.html](mockup/index.html) |
| 何を測ればいいか知りたい | [Spec-params.md](Spec-params.md) §3・§8 |

---

## 4. 原典との関係

### 4.1 原典

| 文書 | 扱い |
| --- | --- |
| `docs/plan/source/NewDesign.md` | **正**（ユーザー指示） |
| `docs/plan/restate/NewDesign-restated.md` | **正**（ユーザー指示・今回限り） |
| `docs/plan/source/NewDesign-transit.md` / `-onsite.md` / `-kpi.md` / `-calib.md` / `-operationalCheck.md` | 詳細ファイル。原典に準じる |
| `docs/plan/source/NewVISION.md` / `onsite-*.md` / `transit-*.md` | 意思決定の経緯 |
| `docs/plan/source/NewDesign-checklist.md` | 指摘管理表（**仕様ソースではない**） |
| `docs/plan/restate/*-findings.md` | 点検結果（**正ではないと自ら宣言している**） |

**優先順位**: ユーザー確定事項 > `NewDesign.md` > 詳細ファイル > 経緯ファイル > `restate/` 配下。
`NewDesign.md` と `restate/NewDesign-restated.md` が食い違う場合のみ個別に判断した
（[Spec-open.md](Spec-open.md) §2）。

### 4.2 原典が更新されたとき

1. **[Spec-open.md](Spec-open.md) §2 の矛盾表と §4 の陳腐化リストを見直す**
2. 原典が直った指摘は §4（陳腐化）へ移す
3. 新しい矛盾が生じたら §2 に行を足し、採否と根拠を書く
4. そのうえで該当する詳細ファイルを直す

**原典を本設計書に合わせて書き換えることはしない。**
本設計書は原典の統合結果であり、原典が上流である。

### 4.3 本設計書が原典と違えている箇所

**すべて本文中に `> **原典との差異**:` の引用ブロックで明示してある。**

| 種別 | 箇所 | 内容 |
| --- | --- | --- |
| **ユーザー確定による上書き** | `NewDesign-onsite.md` 手順 6〜7 | **2 点指示の押下順とゴール点**が入れ替わった（[Spec-open.md](Spec-open.md) U-3） |
| 書き分け不足の統一 | `NewDesign-onsite.md` 当日手順 5・9 | (a) のあとと (b) のあとで戻り先の選択肢が違っていたのを**統一**（[Spec-onsite.md](Spec-onsite.md) §4.2） |
| 書き分け不足の統一 | `NewDesign-transit.md` 手動走行 | 手動走行だけ「終了」しか無かったのを**全方式で「停止」「終了」に統一**（[Spec-transit.md](Spec-transit.md) §0.1） |

「書き分け不足の統一」は原典の意図を変えるものではなく、
一方にしか書かれていない規則を全体に揃えたものである。
**意図そのものを変えたのは U-3 の 1 件だけ。**

---

## 5. モックアップについて

[mockup/index.html](mockup/index.html) は**見た目と操作感を確かめるための静的 HTML**である。

| 項目 | 内容 |
| --- | --- |
| 開き方 | ファイルをブラウザで開くだけ。ビルド不要 |
| 依存 | **なし。**外部のフォント・アイコン・スクリプトを読み込まない（機体の AP 配下にインターネットが無いため） |
| 実装との関係 | **実装ではない。**`th_ws/web_ui/` とは無関係で、そちらのコードには一切触れていない |
| 保守 | [Spec-webui.md](Spec-webui.md) と**同じ画面一覧・同じウィンドウ種別**を持つ。片方を直したらもう片方も直す |
