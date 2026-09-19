# docs/plan/detailed — 詳細設計書

[完全設計書](../spec/README.md)（`docs/plan/spec/`）を入力とし、**実装できる形に落とした設計書。**

**本体は [DetailedDesign.md](DetailedDesign.md)。** そこから各詳細ファイルへ辿れる。

---

## 1. 位置づけ

| 文書 | 役割 |
| --- | --- |
| `VISION.md`（リポジトリ直下） | ユーザーが目指す**完成形**。**確定した方針はここが正** |
| `docs/architecture.md` | **現状実装**の保守・拡張ガイド（as-built） |
| `docs/plan/spec/` | **完全設計書。**「何が・どう振る舞うべきか」と「どういう値が要るか」 |
| **`docs/plan/detailed/`（ここ）** | **詳細設計書。**ノード名・トピック名・ファイル構成・アルゴリズム・作業パケット |

**この設計書も `docs/plan/` 配下の検討段階の文書である。**
実装に入る前に `CLAUDE.md`「方針変更時のルール」に従い、**先に `VISION.md` を更新する。**

### 1.1 完全設計書との境界

完全設計書が**意図的に書いていない**もの（`spec/README.md` §1.1）が、ここの担当範囲である。

| 完全設計書 | 詳細設計書 |
| --- | --- |
| 「対象に近づいたら停止する」 | `follow_stop_distance_m` を制動距離から導出し、`follow_core.py` の `update()` で判定する |
| 「速度指令は優先度つきの多重化を経由する」 | `/cmd_vel_behavior`(20) / `/cmd_vel_manual`(30) / `/cmd_vel_nav`(10) → `twist_mux` → `obstacle_limiter` → `/cmd_vel` |
| 「モードは 18」 | `SystemState.msg` の `string mode`。数値定数は持たない |
| 「(c) に事前の目標値を置いてはいけない」 | `registry.yaml` の `class`／`status` 組合せ規則 S1〜S5 と起動時アサーション |

---

## 2. ファイル構成

| ファイル | 内容 | 誰が読むか |
| --- | --- | --- |
| **[DetailedDesign.md](DetailedDesign.md)** | **本体。**設計方針・全体構成・パッケージ・段階・目次 | 全員（まずここ） |
| [DetailedDesign-packets.md](DetailedDesign-packets.md) | **段階と作業パケット 48 件の一覧。**実装規約・順序制約・依存グラフ | **実装者（次にここ）** |
| [DetailedDesign-wp0.md](DetailedDesign-wp0.md) ／ [-wp1.md](DetailedDesign-wp1.md) ／ [-wp2.md](DetailedDesign-wp2.md) | **段階 0〜2 の作業パケット 23 件の実体**（§0〜§12） | **実装者（担当パケットだけ）** |
| [DetailedDesign-names.md](DetailedDesign-names.md) | **名前辞書** | 全員（常時参照） |
| [DetailedDesign-state.md](DetailedDesign-state.md) | 状態機械・遷移表・ガード・effect | 状態機械／UI の担当 |
| [DetailedDesign-safety.md](DetailedDesign-safety.md) | 安全チェーン・リミッタ・フォルト・**安全負債の台帳** | **全員（着手前に必読）** |
| [DetailedDesign-params.md](DetailedDesign-params.md) | パラメータの実体化 | 全員（数値を書く前に） |
| [DetailedDesign-transit.md](DetailedDesign-transit.md) | 走行方式 | 走行の担当 |
| [DetailedDesign-onsite.md](DetailedDesign-onsite.md) | 試験場内 | 試験場内の担当 |
| [DetailedDesign-maintenance.md](DetailedDesign-maintenance.md) | 始業点検・校正・故障診断 | 保守の担当 |
| [DetailedDesign-webui.md](DetailedDesign-webui.md) | 15 画面 | UI の担当 |
| [DetailedDesign-hardware.md](DetailedDesign-hardware.md) | **機器構成・ESP32 フレーム・手配** | **部品を買う前／ファームを触る前に必読** |
| [DetailedDesign-reuse.md](DetailedDesign-reuse.md) | **既存資産の去就** | **既存コードを消す前に必読** |
| [DetailedDesign-open.md](DetailedDesign-open.md) | 未確定・申し送り・レビュー指摘管理表 | 全員 |

`docs/plan/README.md` の運用ルール（本体は結論と表だけ・200 行超で分割）に従っている。

---

## 3. 読む順番

| 目的 | 読む順 |
| --- | --- |
| 全体を掴みたい | [DetailedDesign.md](DetailedDesign.md) だけ |
| **実装に入る** | [DetailedDesign.md](DetailedDesign.md) → [-packets.md](DetailedDesign-packets.md) の §0・§1 → **`-wp<段階>.md` の担当パケット 1 つだけ**（その §2 に挙がった節も読む） |
| **既存コードを消してよいか知りたい** | [-reuse.md](DetailedDesign-reuse.md) §1（絶対に捨ててはいけないもの） |
| 名前を決めたい | [-names.md](DetailedDesign-names.md)。**無ければ実装前にここへ足す** |
| 数値を書きたい | [-params.md](DetailedDesign-params.md) §9。**裸の数値は書かない** |
| **何が危ないか知りたい** | [-safety.md](DetailedDesign-safety.md) §0（安全負債 5 件） |
| **部品を買う／ファームのフレームを足す** | [-hardware.md](DetailedDesign-hardware.md) §3・§5 |
| 何が決まっていないか知りたい | [-open.md](DetailedDesign-open.md) §4 |

---

## 4. 実装者への 3 つの約束

**実装は Sonnet 5-high が行う。**この設計書は次を保証する。

| # | 約束 |
| --- | --- |
| **1** | **担当パケットの §2 に挙がった節だけ読めば実装できる。**完全設計書を読む必要はない |
| **2** | **名前を発明する必要がない。**必要な名前はすべて [-names.md](DetailedDesign-names.md) にある |
| **3** | **完了したかどうかを自分で判定できる。**受け入れ条件はすべて実行可能なコマンドになっている |

**この 3 つが崩れている箇所を見つけたら、実装を止めて設計書側を直す。**
黙って推測で埋めない（推測は次のセッションで別の推測になる）。

### 4.1 約束 1 の例外 —— 完全設計書を直接読ませる 4 箇所

**次の 4 パケットだけは完全設計書の節を直接指している**（`WP-MEAS-03` / `-05` は §2 の表に、
`WP-MEAS-01` / `-02` は本文の根拠として）。
詳細設計に写すと**原典より劣化した引き写しが 1 つ増えるだけ**の性質のもの
（測定の根拠・目標値・`n=1` の理由）で、写す価値が無いと判断した。

| パケット | 読む節（フルパス） | なぜ写さないか |
| --- | --- | --- |
| `WP-MEAS-01` | `docs/plan/spec/Spec-params.md` §8 ／ `docs/plan/spec/Spec-safety.md` §2.2 | **制動距離の式そのもの。**写すと式が 2 箇所に増える |
| `WP-MEAS-02` | `docs/plan/spec/Spec-safety.md` §3 | 「重心が高くなることが予想される」という**予想の出どころ**。要約すると根拠が消える |
| `WP-MEAS-03` | `docs/plan/spec/Spec.md` §2（目標 G4） | **目標値の定義。**詳細設計は目標を定義しない |
| `WP-MEAS-05` | `docs/plan/spec/Spec-ops.md` §5 | **`n=1` で足りる根拠。**判断そのものが原典にある |

**4 件とも段階 0 の測定パケットである。**実装コードを書くパケットは 1 件も含まない
——**コードを書く実装者は完全設計書を読まなくてよい**という約束 1 は保たれている。

**これ以外の場所に完全設計書への参照を増やさない。**増やすなら、この表に行を足してから。

---

## 5. 完全設計書が更新されたとき

1. [-open.md](DetailedDesign-open.md) §3 の申し送り表を見直す（直った項目は行を消す）
2. 変更が遷移に関わるなら、[-state.md](DetailedDesign-state.md) §4 の該当行と `spec_ref` を直す
3. 変更が数値に関わるなら、[-params.md](DetailedDesign-params.md) §6 の索引と `registry.yaml` を直す
4. **完全設計書を詳細設計書に合わせて書き換えることはしない。**完全設計書が上流である

---

## 6. レビュー

| 回 | 時期 | 観点 | 結果 |
| --- | --- | --- | --- |
| **1 回目** | 基盤 4 ファイル完成時 | 安全・整合 ／ 実装可能性 | **指摘 87 件。**[-open.md](DetailedDesign-open.md) §5 |
| **2 回目** | 全 13 ファイル完成時 | 網羅性 ／ 安全・整合 ／ 実装可能性 | **指摘 84 件。**[-open.md](DetailedDesign-open.md) §5.3。うち 13 件が持ち越し → **10 件を段階 0〜2 のパケット実体化で処理**。残り A〜D |
| **3 回目** | **A〜C 処理済み（2026-08-17）。いつでも実施できる** | 同左 | **未実施** |

指摘は [-open.md](DetailedDesign-open.md) §5 に記録し、
**「対応済み」か「対応しない＋理由」に落とし切るまで完了としない。**
