# 詳細設計書 — 配電盤上部確認用ロボット 移動システム

[完全設計書](../spec/Spec.md)（`docs/plan/spec/`）を入力とし、**実装できる形に落とす。**
読み方・位置づけは [README.md](README.md)。

**実装は Sonnet 5-high が行う。**この文書の作りはすべてそこから来ている（§1）。

---

## 1. 設計方針（`DD-1`〜`DD-8`）

完全設計書 4,000 行と既存コードを同時に保持できないので、
**「読まなくても実装できる形」に設計書の側を作り替える。**

| # | 原則 | なぜ |
| --- | --- | --- |
| **DD-1** | **名前は辞書が正本。**ノード・トピック・サービス・msg・パラメータ・ファイルパス・画面 ID・モード ID・状態 ID・トリガ名をすべて 1 つの表に集める | 名前を発明する余地をゼロにする。セッションをまたぐと必ず食い違う |
| **DD-2** | **作業パケット単位で書く。**1 パケット＝1 セッションで完結。各パケットに「読むべき節（それだけ）」「触るファイル」「受け入れ条件（実行コマンドまで）」「依存」「禁止事項」 | 実装者は完全設計書を読まない。自己完結が実装可能性の条件 |
| **DD-3** | **状態遷移は表からコードを生成できる粒度で書く。**実装は表を**データとして**読む | `Spec-modes.md` §3.1 の「表に無い遷移は起こらない」をコードの性質として担保する |
| **DD-4** | **既存資産の去就を 1 ファイル 1 行で書く**（維持／抽出／改修／土台／廃止） | 迷えば作り直す実装者から、**実機で潰した知見が入っている設定値**を名指しで守る |
| **DD-5** | **トレーサビリティ。**各項目に完全設計書の節・ID を併記し、逆引き表で網羅を示す | `Spec-open.md` §6 と同じ流儀 |
| **DD-6** | **未確定値 (c) には数値を書かない。**パラメータ名・分類・起動時の止め方だけ書く | `Spec-params.md` §0。放置すると測定が「目標の確認」になる |
| **DD-7** | **段階に割り、各段階の終わりに動くものが残る** | `ImplementationPlan.md` の「実装→単体試験→Gazebo→実機→次の段階」が成立する切り方 |
| **DD-8** | **完全設計書を上書きしない。**矛盾は申し送りとして記録する | 確定したら `VISION.md` → `spec/` の順で直す（`CLAUDE.md`） |

---

## 2. 全体構成

```
[ WebUI (React) ] ──rosbridge──┐
                                │  ui.* トリガ送信 ／ /system/state 購読
                          [ th_state ]  ← 遷移表が唯一の正
                                │  /system/state (mode, state, flags, zone)
   ┌────────────────────────────┼────────────────────────────┐
[ th_transit ]              [ th_safety ]              [ th_onsite / th_route ]
 FOLLOW / MANUAL /          safety_monitor              PREP / PANEL_NAV /
 TEACH_* / LINE / LEASH     （フォルト2階級）             AT_PANEL / SUMMON /
                                                        HOME_NAV / 教示・再生
   │ /cmd_vel_behavior (20)                                  │
   │                    WebUI ──► /cmd_vel_manual_raw ──► [ jog_gate ] ──► (30)
   │                    Nav2  ──► /cmd_vel_nav (10)
   └──────────► [ twist_mux ] ──► /cmd_vel_muxed ──► [ obstacle_limiter ] ──► /cmd_vel
                  ロック 255/254                      速度上限・障害物停止
                                                              │
                                                     [ esp32_bridge ] ──► ESP32
                                                     （独立ロック層は維持）
```

**不変ルール**: `/cmd_vel` を publish してよいのは **`obstacle_limiter` だけ**。

---

## 3. パッケージ

| パッケージ | 扱い | 役割 |
| --- | --- | --- |
| `th_system_msgs` | 拡張 | 型定義 |
| **`th_params`** | 新規 | パラメータ registry・導出・監査 |
| **`th_state`** | 新規 | 状態機械（`th_mode_manager` を置換） |
| `th_safety` | 拡張 | `safety_monitor` ＋ **`obstacle_limiter`** ＋ **`jog_gate`** |
| **`th_transit`** | 新規 | 走行方式（`th_planning` を置換） |
| **`th_route`** | 新規 | 教示の記録・再生・地図セッション |
| **`th_onsite`** | 新規 | 試験場内 |
| **`th_maintenance`** | 新規 | 始業点検・校正・故障診断 |
| `th_perception` | ほぼ維持 | `lidar_filter` / `person_tracker_bridge` |
| `th_esp32_bridge` | 改修 | ESP32 リンク |
| `th_bringup` / `th_description` / `th_testing` | 改修／維持／拡張 | |
| `web_ui` | 作り直し | 既存 React を土台に 15 画面へ |

**廃止**: `th_mode_manager` / `th_planning` / `th_calibration` / `th_config_manager`。

---

## 4. 現行実装との主な差

| 完全設計書 | 現行 | 詳細設計での扱い |
| --- | --- | --- |
| 18 モード＋サブステート＋フラグ | 9 モード・`switch` にハードコード | **Python・表駆動**（[state](DetailedDesign-state.md)） |
| フォルト 2 階級（回復→`PAUSE`／重大→`ESTOP`） | 全フォルトで `IDLE` へ強制遷移 | `FaultStatus.severity` ＋ **重大は `fault_lock` にも載せる** |
| 障害物リミッタ | **存在しない** | **後段に新設。ただし挙動側にも判定を残す二層**（`d_floor < d_behavior`） |
| ゾーンは画面が決める | 概念が無い | **3 値**（`IN`/`OUT`/`NA`）。合成は**速度上限の最小値** |
| ジョグ介入 | 概念が無い | **リース式**（`ui.jog.hold` を送り続ける）＋ **`jog_gate` で構造的に塞ぐ** |
| 走行方式 7 | 追従 2 種と手動のみ | 教示・再生を新設。ライン・リードは後回し |
| 盤のランタイム登録 | `panels.yaml` を起動時に読むだけ | **2 点指示による登録**（`th_onsite`） |
| 始業点検・校正・故障診断 | CLI 4 本 | **サービス駆動のウィザード**（`th_maintenance`） |
| 数値の 3 分類 | 各 YAML に直書き | **`registry.yaml` が唯一の置き場**。(c) は起動を止める |

---

## 5. 段階（段階 0〜7 で 48 パケット）

| 段階 | 内容 | 終わりに残るもの |
| --- | --- | --- |
| **0** | 測定・ネットワーク・契約 | 実測値が入った registry と、pytest が通る純粋コア |
| **1** | 状態機械と UI 骨格 | 実機で `INIT→IDLE→MANUAL`。非常停止が常に押せる |
| **2** | **安全層** | 故障注入 13 項目が自動で回る。**安全負債 `DEBT-1`〜`DEBT-4` が解除される** |
| **3** | 手動系と共通 UI 部品 | 手動走行が完動（＝フォールバックが成立） |
| **4** | 追従系 | 追従走行・教示（追従）。KPI 計測開始 |
| **5** | 再生 | 教示 → 再生の往復が閉じる |
| **6** | 試験場内 | 前日準備＋当日運用が一通り |
| **7** | 保守 | 1 日の流れが全部 WebUI で完結 |
| **8** | ライン誘導・電子リード | — |

**絶対に守る順序制約 5 件**は [packets](DetailedDesign-packets.md) §1。
特に **`/cmd_vel` の stale タイムアウト（`DEBT-4`）はリミッタ新設より前**に行う。

---

## 6. 先に潰す

`Spec.md` §11 の 4 つに、探索で見つかった**安全負債 4 件**が加わる。

| 順 | 項目 | なぜ先か |
| --- | --- | --- |
| **1** | **`DEBT-4`** `/cmd_vel` 途絶時もキープアライブが最後の非ゼロ指令を送り続ける | 塞がないうちに後段へノードを足すと**単一障害点を足すことになる** |
| **2** | **`DEBT-1`** 物理 E-Stop がファームのバイパスで無効 | 安全チェーン層 1 が不在のまま走らせない |
| **3** | **`DEBT-2`** LiDAR 死角マスクが幅ゼロ | この状態でリミッタを有効にすると**自分の角柱で永久停止する** |
| **4** | **実測の制動加速度**（`O-c1`） | 最も多くのパラメータを従属させる |
| **5** | **`DEBT-3`** タイムアウト 2000 ms（0.7 m/s で 1.4 m 走る） | ネットワーク改善（`task.md` 1）と同時に片づける |
| 6 | カメラ昇降の許容差（`O-a1`） | 聞くより先に「どこまで緩められるか」を交渉する |
| 7 | 人が歩いている部屋で地図を作る（`O-c2`） | 最も安いテスト。人物マスクの要否が決まる |
| 8 | 前日準備・手押しのベースライン（`O-a3` / `O-a7`） | 無いと G3・G4 を検証できない |

---

## 7. 詳細ファイル

| ファイル | 内容 |
| --- | --- |
| [DetailedDesign-names.md](DetailedDesign-names.md) | **名前辞書。**この文書に無い名前を実装で作ってはいけない |
| [DetailedDesign-state.md](DetailedDesign-state.md) | 状態機械。正規化した遷移表・ガード 28 件・effect 一覧・純粋コアの API |
| [DetailedDesign-safety.md](DetailedDesign-safety.md) | 安全チェーン 4 層・`obstacle_limiter`・`jog_gate`・フォルト 2 階級・**安全負債の台帳** |
| [DetailedDesign-params.md](DetailedDesign-params.md) | パラメータの実体化。registry・導出式・起動時アサーション |
| [DetailedDesign-transit.md](DetailedDesign-transit.md) | 追従／手動／教示 2 種／教示再生（ライン・リードは節のみ） |
| [DetailedDesign-onsite.md](DetailedDesign-onsite.md) | 試験場内 5 モード・2 点指示・退避待ちゲート |
| [DetailedDesign-maintenance.md](DetailedDesign-maintenance.md) | 始業点検・校正・故障診断 |
| [DetailedDesign-webui.md](DetailedDesign-webui.md) | 15 画面の実装 |
| [DetailedDesign-hardware.md](DetailedDesign-hardware.md) | **機器 → ノード → トピック。**USB 帯域・ESP32 フレーム・カメラの選定・手配 |
| [DetailedDesign-reuse.md](DetailedDesign-reuse.md) | **既存資産の去就。**捨ててはいけないものを名指しで守る |
| [DetailedDesign-packets.md](DetailedDesign-packets.md) | **段階と作業パケット 48 件の一覧・順序制約・検証。**実装はここから読む |
| [DetailedDesign-wp0.md](DetailedDesign-wp0.md) | **段階 0 の作業パケット 10 件の実体**（測定・ネットワーク・契約） |
| [DetailedDesign-wp1.md](DetailedDesign-wp1.md) | **段階 1 の作業パケット 6 件の実体**（状態機械と UI 骨格） |
| [DetailedDesign-wp2.md](DetailedDesign-wp2.md) | **段階 2 の作業パケット 7 件の実体**（安全層） |
| [DetailedDesign-open.md](DetailedDesign-open.md) | 未確定・申し送り・**レビュー指摘管理表** |

**段階 3 以降のパケットは実体が無い。**着手前に `DetailedDesign-wp<n>.md` を
[packets](DetailedDesign-packets.md) §0.1 のテンプレートで書く。

---

## 8. 実装に入る前に

**この文書は `docs/plan/` 配下の検討段階の文書である**（完全設計書と同じ位置づけ）。

1. **`VISION.md` を更新する。**`CLAUDE.md`「方針変更時のルール」により、
   追従ロジック・モード遷移・安全設計・アーキテクチャの変更は**コードより先に** `VISION.md` を直す
2. ~~[DetailedDesign-open.md](DetailedDesign-open.md) §3 の申し送り 11 件を完全設計書へ戻す~~
   → **2026-08-16 完了**（A-1〜A-11。`Spec-modes.md` §3.1 に ID 列と欠けていた遷移が入った）
3. ~~同 §5.3 の未対応 A・B・C を処理する~~
   → **2026-08-17 完了**（A＝[state](DetailedDesign-state.md) §4.4 の `spec_ref` 全 124 行 ／
   B＝[onsite](DetailedDesign-onsite.md) §3.7 の地図再利用 3 段 ／
   C＝[hardware](DetailedDesign-hardware.md) 新設）。**残るは D（段階 3〜8 のパケット実体）だけ**
4. **3 回目のレビューを行う**（観点 3 つ。[README.md](README.md) §6）
5. `git status` を確認し、未コミットの変更があればユーザーへ提示する
