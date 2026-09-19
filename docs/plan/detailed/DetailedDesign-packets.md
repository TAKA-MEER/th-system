# 段階と作業パケット

[DetailedDesign.md](DetailedDesign.md) の詳細。**実装の単位と順序。**

> **`DD-2`**: 1 パケット＝1 セッションで完結する。実装者は完全設計書を読まない。
> **`DD-7`**: 各段階の終わりに「実機（モータ電源断）で確認できる動くもの」が残る。

---

## 0. 実装規約（**全パケット共通。先に読む**）

| # | 規約 | 理由 |
| --- | --- | --- |
| **R1** | **パケットの §2「参照する設計書の節」に無い仕様は実装しない。**必要になったら**先に設計書へ行を足す** | `Spec-modes.md` §3.1 の運用をそのまま引き継ぐ |
| **R2** | **数値リテラルをコードに書かない。**すべて `registry.yaml` 経由 | [params](DetailedDesign-params.md) §9 |
| **R3** | **ROS2 非依存の純粋コア＋薄いノード。**テストは `th_ws/src/th_testing/test/` に置く | 既存資産の流儀 |
| **R4** | **日本語の文字列は WebUI の `i18n/` にだけ置く。**ノードはキーを返す | [webui](DetailedDesign-webui.md) §7 |
| **R5** | **入力途絶時の既定を必ず明記する。**沈黙禁止・安全側へ倒す | [safety](DetailedDesign-safety.md) §3.4.2 |
| **R6** | **1 パケット ≒ 1 ノードまたは 1 画面。**実装＋単体試験が 1 セッションで閉じる大きさ | — |
| **R7** | **作業前に `git status` を確認する。**未コミットの変更があれば着手前にユーザーへ確認する | `CLAUDE.md` |
| **R8** | **自分が変更したファイルだけをステージする。**`git add -A` / `git add .` は使わない | 同上 |

### 0.1 パケットのテンプレート

```
## WP-<領域>-<番号> <名前>

### 0. 一行要旨
### 1. 対象と非対象          ← 「これは作らない」を明示
### 2. 参照する設計書の節      ← ここに挙げた節だけ読めば実装できること
### 3. インターフェース契約
      3.1 トピック（名前・型・QoS・レート・pub/sub）
      3.2 サービス／アクション（要求・応答・拒否理由キー）
      3.3 パラメータ（名前・単位・class・status・registry 行）※裸の数値を書かない
      3.4 フレーム（frame_id・TF 前提）
### 4. 内部設計
      4.1 純粋コアの関数シグネチャ一覧
      4.2 ノードの責務（「コアを呼ぶだけ」と明記）
      4.3 不変条件
### 5. 表駆動データ（該当時）   ← 設計書からの転写。加工しない
### 6. 安全要件
      6.1 触れる安全チェーンの層
      6.2 フェイルセーフ既定（入力途絶・上流沈黙時に何を出すか）
      6.3 このノード単独が壊れたときに何が起きるか（FMEA 3 行）
### 7. 単体試験              ← 「テスト名 → 満たす仕様行」の対応表
### 8. Gazebo シナリオ
### 9. 実機での確認手順       ← モータ電源断でできること／通電が要ることを分ける
### 10. 完了条件             ← すべて機械判定可能なチェックリスト
### 11. 既知の負債・未確定 (c) と、その扱い
### 12. 依存 WP / 被依存 WP
```

**§6.3 の FMEA と §9 の「電源断／通電」の分離は、この案件に固有の必須項目である。**
前者は `DEBT-4` のような穴を書く時点で発見させるため、後者は
`ImplementationPlan.md` の実機条件（モータの電源のみ切断して PC と接続）と直結するため。

### 0.2 パケットの実体はどこにあるか

**このファイルは一覧と順序制約だけを持つ。**各パケットの中身（§0〜§12）は別ファイルにある。

| 段階 | 実体 | 状態 |
| --- | --- | --- |
| **0** | [DetailedDesign-wp0.md](DetailedDesign-wp0.md) | **10 パケットすべて記述済み** |
| **1** | [DetailedDesign-wp1.md](DetailedDesign-wp1.md) | **6 パケットすべて記述済み** |
| **2** | [DetailedDesign-wp2.md](DetailedDesign-wp2.md) | **7 パケットすべて記述済み** |
| **3** | [DetailedDesign-wp3.md](DetailedDesign-wp3.md) | **`WP-UI-03` / `WP-TRANSIT-01` の 2 件のみ記述済み。**`WP-ROUTE-01` / `WP-TRANSIT-02` は着手時に追記する |
| 4〜8 | — | **未記述。**着手前に §0.1 のテンプレートで書く |

**段階 3 以降に着手するときは、まず該当段階の `DetailedDesign-wp<n>.md` を書く。**
一覧表の 4 列（WP / 名前 / 依存 / 完了条件）だけでは `DD-2`（1 パケット＝1 セッションで完結）が成立しない
——実装者は結局、設計書を全部読むことになる。

### 0.3 §10「完了条件」の書き方（**3 回目レビューで 31 件が動かないと判明した**）

**「機械判定可能」は、コマンドが書いてあることではない。**
実際に走って、**合格と不合格で違う終了コードになる**ことである。

| # | 規約 | 破ったときに起きること |
| --- | --- | --- |
| **V1** | **ブロック冒頭に実行場所を書く。**`# コンテナ内 (/root/th_ws)` ／ `# ホスト（リポジトリルート・Git Bash）` | `docker-compose.yml` は `./src` `./esp32` `./scripts` しかマウントしないので、**`web_ui/` も `docs/` もコンテナ内に無い。**パスが解決しない |
| **V2** | **`grep` の否定（`! grep ...`）は、対象ファイルが存在することを先に確かめる。**`test -f <path> && ! grep ...` | `grep` はファイルが無いと exit 2 を返し、**`!` が反転して合格になる**——検査が無効化される |
| **V3** | **`grep` は必ず期待文字列そのものを書く。**`grep -q '<期待>'`。`-E` を使うなら明示する | `grep "ESTOP_HW.*3"` は既存の `ESTOP_HW (0x03)` に当たって**改修前でも合格する** |
| **V4** | **`ros2 topic echo` / `hz` は必ず `timeout N` で包む。**`--field` はドット区切り（`linear.x`） | `hz` は Ctrl-C まで戻らない。「来ないこと」が期待値の検査は**合格時に永久ブロックする** |
| **V5** | **`colcon test` の後に必ず `colcon test-result --verbose` を置く** | `colcon test` は**テストが落ちても exit 0**。さらに `th_safety` は現在テスト登録が 0 件なので**無条件で合格する** |
| **V6** | **`-R` に渡す ctest 名を §7 の表に書く**（`ament_add_pytest_test(<名前> ...)` の第 1 引数） | 名前を発明されると `-R` が空振りし、**テスト 0 件で exit 0 ＝ 無条件合格** |
| **V7** | **バックグラウンド起動は `PID=$!` で PID を取り、`kill -TERM $PID` で止める。**`kill %1` を使わない | 非対話シェル（`docker exec bash -lc`）ではジョブ制御が無効で `%1: no such job`。**publisher が残って次の検査を汚染する。**`kill -9` は DDS discovery を壊す（`CLAUDE.md`） |
| **V8** | **Python モジュール実行の前に `colcon build --symlink-install --packages-select <pkg> && source install/setup.bash` を置く** | `python3 -m th_params.export` は実体が `src/th_params/th_params/` にあるので **ImportError** |
| **V9** | **件数は数える。**`test $(ls ... \| wc -l) -eq N` | 「3 条件 × 5 回」とコメントに書いても検査していない |
| **V10** | **中身がコメントだけのヒアドキュメントを置かない** | 何も実行せず常に exit 0。約束 3 が形式だけになる |

**`docker-compose.yml` のマウントは 3 つだけ**（`./src` `./esp32` `./scripts`）。
**ホスト側でしか実行できないもの**: `web_ui/` の npm・Playwright、`docs/` 配下の測定データ、
`docker-compose.yml` 自身、`git ls-files`。

---

## 1. 絶対に守る順序制約

**この 8 件は、破ると「起動しない中間状態」または「安全装置が無い状態」を作る。**

| # | 制約 | 理由 |
| --- | --- | --- |
| **O-1** | **`WP-SAFE-02`（`/cmd_vel` の stale タイムアウト）は `WP-SAFE-03`（リミッタ）より前** | 塞がないうちに後段へノードを足すと、「安全装置を足した」ではなく**単一障害点を足した**ことになる（`DEBT-4`） |
| **O-2** | **twist_mux の remap 変更・`obstacle_limiter` の新設・`test_twist_mux_priority.py` の更新は `WP-SAFE-03` の中で同時に行う** | 先に remap だけ変えると `/cmd_vel` の publisher が消えて機体が動かない。launch は **`bringup` と `gazebo` の 2 本**ある |
| **O-3** | **`RobotMode.msg` などの旧 msg 削除は `WP-CLEAN-01` まで行わない。**`WP-MSG-01` は**追加だけ**する | 削除は `th_mode_manager` / `th_planning` / `web_ui` / 既存テストを同時に壊す。新旧が併存する期間を作る |
| **O-4** | **`WP-CALIB-01`（死角マスク校正）は `WP-SAFE-03` より前** | 幅ゼロのマスクのままリミッタを有効にすると、**自分のアルミ角柱で永久停止する**（`DEBT-2`） |
| **O-5** | **`WP-ESP32-01`（バイパス解除）は実機で走らせる全パケットより前**（例外なし。§1.1） | 物理非常停止が効かないまま走らせない（`DEBT-1`） |
| **O-6** | **`jog_gate` の新設（`WP-SAFE-04`）と、WebUI の publish 先を `/cmd_vel_manual_raw` に変える作業は同一パケット** | 分けると旧 UI と `jog_gate` が同じ `/cmd_vel_manual` に publish し、**手動ジョグが断続的に潰される** |
| **O-7** | **`safety_monitor` の監視対象は、その publisher を作るパケットが済むまで有効にしない** | `/person/targets`（段階 4）・`/safety/limiter_status`（段階 2）・`/map_session/status`（段階 5）は段階 2 時点で存在しない。`enabled_targets` パラメータで段階ごとに有効化する |
| **O-8** | **`WP-PARAM-02` は launch の `parameters=` を `generated/` へ差し替えるところまで含む** | 生成するだけでは誰も読まない。`bringup` / `gazebo` の該当行をすべて直す |

### 1.1 `O-5` に例外を作らず、`WP-ESP32-01` を段階 0 へ移した

**段階 0 の測定のうち `WP-MEAS-01`（制動加速度）と `WP-MEAS-02`（人が歩く部屋の地図）は実機で走る。**
`WP-ESP32-01` が段階 2 にあると `O-5` と段階順が矛盾する。

| パケット | 実機に何をするか | `O-5` に当たるか |
| --- | --- | --- |
| `WP-MEAS-01` 制動加速度 | **走る**（`v_max` から急停止） | **当たる** |
| `WP-MEAS-02` 人が歩く部屋の地図 | **走る**（手動走行のみ） | **当たる** |
| `WP-MEAS-03` 手押し搬送 | **未通電・手押し**。ROS2 も使わない | 当たらない（駆動系が生きていない） |
| `WP-MEAS-04` リンク品質 | **通電・静止**（機体を動かさない） | 当たらない（走らない） |

**例外を作るのではなく `WP-ESP32-01` を段階 0 へ移した。**理由は 3 つ。

| # | 理由 |
| --- | --- |
| **1** | **`WP-MEAS-01` §6.1 が「`DEBT-1` が有効な状態で走らせない」と既に書いている。**例外を作ると、`O-5` と `WP-MEAS-01` §6 のどちらかを嘘にすることになる |
| **2** | **制動距離を「止められない機体」で測るのは、測定として筋が悪い。**急停止の測定中に急停止できないのは、代替手段の有無以前の問題である |
| **3** | **移せる。**`WP-ESP32-01` の依存は `WP-NET-01`（同じ段階 0・望ましい程度）だけで、`std_msgs/UInt8` しか使わないので `WP-MSG-01` にも依存しない |

**これで `O-5` は例外なしの制約になった。**段階 0 の入口で `WP-ESP32-01` を済ませ、
以降どの段階のどのパケットも「物理非常停止が効く機体」で走らせる。

---

## 2. 段階の一覧

| 段階 | 内容 | 終わりに残る「動くもの」 |
| --- | --- | --- |
| **0** | 測定・ネットワーク・契約（msg／registry／`state_core`） | pytest が通る純粋コアと、実測値が入った registry |
| **1** | 状態機械と UI 骨格 | 実機で `INIT→IDLE→MANUAL`。非常停止が常に押せる |
| **2** | **安全層**（リミッタ・ジョグゲート・フォルト 2 階級・安全負債 4 件） | Gazebo で故障注入 13 項目が自動で回る |
| **3** | 手動系と共通 UI 部品 | **手動走行が新アーキで完動**（＝他方式のフォールバックが成立）。教示（手動）で経路が保存できる |
| **4** | 追従系 | 追従走行・教示（追従）。KPI 計測開始 |
| **5** | 再生 | 教示 → 再生の往復が閉じる |
| **6** | 試験場内 | 前日準備＋当日運用が一通り |
| **7** | 保守 | **1 日の流れが全部 WebUI で完結**（**`SD-1`** 達成） |
| **8** | ライン誘導・電子リード | — |

**手動系を段階 3 に置く理由**: ① 他方式が使えないときのフォールバック（`Spec-transit.md` §2）
② 保守モードの開始条件が `IDLE` / `MANUAL`（`Spec-checks.md` §1）。

---

## 3. 段階 0 — 測定・ネットワーク・契約

**実体 → [DetailedDesign-wp0.md](DetailedDesign-wp0.md)。**下表は索引である。

| WP | 名前 | 依存 | 完了条件 |
| --- | --- | --- | --- |
| **`WP-MEAS-01`** | **実測の制動加速度** | — | `brake_accel_mps2` が `status: measured` になる。前進・後退・場内速度域の 3 条件で各 5 回 |
| `WP-MEAS-02` | 人が歩いている部屋で地図を作る | — | 写り込みの有無を記録。**人物マスク実装の要否が決まる**（`O-c2`。最も安いテスト） |
| `WP-MEAS-03` | 手押し搬送のベースライン | — | 片道 200 m の所要時間・人数・負担（`O-a7`。目標 G4 の比較対象） |
| **`WP-MEAS-05`** | **1 日の運用を通した完走試験**（目標 G5） | 段階 6 完了後 | 往復＋準備＋試験を**再起動なし・バッテリー交換なし**で完走。§2.4 の自動再起動が走ったら未達。**n=1 で足りる**（`Spec-ops.md` §5）。`battery_endurance_min` が `measured` になる |
| **`WP-SAFE-00`** | **リンク品質計測だけを先に作る**（`/safety/link_quality` の publish） | `WP-MSG-01` | 受信間隔の p50/p99/max が出る。**`WP-MEAS-04` の前提** |
| `WP-MEAS-04` | リンク品質の実測 | `WP-NET-01`, **`WP-SAFE-00`** | `link_gap_p99_ms` が ESP32／LiDAR／UI 別に `measured` になる |
| **`WP-NET-01`** | **ネットワークの改善** | — | `task.md` 1。`DEBT-6`（秘匿情報）も同時に片づける。受信ギャップの改善を `WP-MEAS-04` で確認 |
| **`WP-MSG-01`** | `th_system_msgs` の新設（**追加のみ**） | — | 新 msg／srv がビルドできる。**旧 msg は消さない**（`O-3`） |
| **`WP-PARAM-01`** | registry ＋ `derive` ＋ `assertions` ＋ `export`（**純粋コアのみ**） | `WP-MEAS-01` | `pytest` が通る。`export.py` が YAML を吐く。**S1〜S5 と A1〜A10 のテストが全部ある** |
| **`WP-STATE-01`** | `state_core` ＋ `transitions.yaml` ＋ `guards.py`（**ROS2 なし**） | — | `pytest` が通る。`validate()` が空を返す。**遷移表の全行に対応テストがある** |
| **`WP-ESP32-01`** | **バイパス解除 ＋ `ESTOP_HW` にファーム構成フラグ**（`DEBT-1`） | `WP-NET-01`（望ましい） | 物理非常停止が実際に効く。`/safety/firmware_flags` が出て `safety_monitor` が検出できる形になる。**`WP-MEAS-01` / `-02` より前**（`O-5`・§1.1） |

### 3.1 `WP-MEAS-01` は実装ではなく測定である

**`Spec-params.md` §8 が「先に潰す 4 つ」の 1 番に挙げているもの。**
これが決まるまで `obstacle_*_distance_m` / `follow_stop_distance_m` / タイムアウトが
すべて `placeholder` になり、実機では起動できない（`blocking: true`）。

| 手順 | 内容 |
| --- | --- |
| 1 | 一定速度で走らせ、`/cmd_vel` をゼロにしてから停止するまでの距離を測る |
| 2 | `/odometry/filtered` の積分と、巻尺の実測の両方を記録する |
| 3 | 前進・後退・場内速度域の 3 条件 × 5 回 |
| 4 | **最も悪い値**を採る（平均ではない） |
| 5 | `registry.yaml` に `status: measured` / `measured_at` / `source` を入れる |

**実機・通電が必要。**`ImplementationPlan.md` の「管理者権限が必要な操作と実機での動作確認のみ人が関与」に該当。

---

## 4. 段階 1 — 状態機械と UI 骨格

**実体 → [DetailedDesign-wp1.md](DetailedDesign-wp1.md)。**下表は索引である。

| WP | 名前 | 依存 | 完了条件 |
| --- | --- | --- | --- |
| `WP-STATE-02` | `state_manager` ノード | `WP-STATE-01`, `WP-MSG-01` | `/system/state` が出る。`/system/trigger` が受理・拒否を返す。**`ui.*` を `/system/event` から入れたら拒否される** |
| `WP-STATE-03` | `connectivity_checker` | `WP-STATE-02` | 必須 3 者の疎通で `evt.link_ok`。一定時間で `sys.link_timeout` |
| `WP-PARAM-02` | `params_audit` ノード ＋ launch の `OpaqueFunction` | `WP-PARAM-01` | 起動時に `generated/` が出来る。アサーション違反で launch が止まる。`/system/params_status` が出る |
| `WP-UI-01` | シェル（ヘッダ・操作カード・ウィンドウ・i18n） | `WP-STATE-02` | §9 の受け入れ条件 1〜7 が通る |
| `WP-UI-02` | S-00 接続確認 ／ S-01 メインメニュー | `WP-UI-01`, `WP-STATE-03` | 必須 3 者が揃うまで S-01 へ進めない。「運用の終了」区画が動く |
| **`WP-CARRY-01`** | **手押しモード（`CARRY`）一式** | `WP-STATE-02`, `WP-UI-01` | 物理 E-Stop 押下で `CARRY` へ／W-2 が解除方法を案内／解除で「再開」が出る／**`CARRY` 中の UI 非常停止が `estop_disabled_in_carry` で拒否され、その理由が W-2 に出る**／再開で押下前のモードへ戻る。**通電した実機でのみ最終確認できる** |

**段階 1 の出口**: 実機（モータ電源断）で `INIT → IDLE → MANUAL` が確認でき、
**異常ウィンドウを開いた状態で非常停止ボタンが押せる**こと。

---

## 5. 段階 2 — 安全層（**最重要**）

**実体 → [DetailedDesign-wp2.md](DetailedDesign-wp2.md)。**下表は索引である。

| WP | 名前 | 依存 | 完了条件 |
| --- | --- | --- | --- |
| **`WP-SAFE-02`** | **`esp32_bridge` の `/cmd_vel` stale タイムアウト**（`DEBT-4`） | `WP-PARAM-02` | 故障注入 §10-12 が通る。**ロック層とキープアライブは残したまま** |
| **`WP-CALIB-01`** | **LiDAR 死角マスク校正（前倒し・最小範囲）**（`DEBT-2`） | `WP-UI-01` | 実機の角柱に合った `blind_angle_ranges` が `registry.yaml` に入る |
| `WP-SAFE-01` | `safety_monitor` 改修（**`FaultStatus.severity` の追加を含む**・監視対象の `enabled_targets` 化・タイムアウトの導出値化） | `WP-MSG-01`, `WP-MEAS-04` | 回復／重大の 2 階級が出る。`fault_lock` に `CRITICAL` が載る。**`test_safety_monitor.py` / `test_fault_detection.py` を同時に更新**（`severity` 追加で両方壊れる） |
| **`WP-SAFE-03`** | **`obstacle_limiter` 新設 ＋ twist_mux remap ＋ テスト更新** | `WP-SAFE-02`, `WP-CALIB-01`, `WP-PARAM-02` | L1〜L7 の property test が通る。故障注入 §10-1/2/11 が通る |
| `WP-SAFE-04` | `jog_gate` 新設 | `WP-SAFE-03`, `WP-STATE-02` | `WAIT_CLEAR` / `OPCHECK` / `CALIB` / `IDLE` でジョグ指令が**構造的に**ゼロになる |
| `WP-TEST-01` | 故障注入 13 項目の自動化 | 上記すべて | `colcon test --ctest-args -R fault_injection` が通る |

### 5.1 `WP-CALIB-01` の最小範囲

**保守機能の骨格を丸ごと作らせない。**この段階で作るのは次だけ。

| 作る | 作らない |
| --- | --- |
| S-40 の「LiDAR 死角」ペイン（ライブスキャン＋ドラッグ選択） | `CALIB` の他 3 項目・ウィザードの 4 ステップ骨格 |
| `/calib/submit` の `BLIND` 項目だけ | 履歴 3 世代・ロールバック |
| 結果を `registry.yaml` へ書く | `/root/th_data/calib/` の永続化構造 |
| `OPCHECK` の項目 4（LiDAR）の自動判定 | `OPCHECK` の他 3 項目・故障診断 |

**段階 7 で本格版に作り直す。**ここは「リミッタを動かすために必要な最小限」と割り切る。

### 5.2 `WP-SAFE-03` は 1 パケットで 4 つを同時に変える

**分けると「起動しない中間状態」ができる**（`O-2`）。

1. `th_safety/src/obstacle_limiter.cpp` ＋ `obstacle_limiter_core.hpp` を新設
2. `th_bringup/launch/bringup.launch.py` の `remappings=[('cmd_vel_out','/cmd_vel')]` → `/cmd_vel_muxed`
3. `th_bringup/launch/gazebo.launch.py` の同じ行 → `/cmd_vel_muxed`
4. **`twist_mux.yaml` の入力トピック改名**: `retreat: /cmd_vel_retreat` → `behavior: /cmd_vel_behavior`（priority 20 は維持）
5. `th_testing/test/test_twist_mux_priority.py` の**出力トピック名と、テスト内にインラインで持っているパラメータ辞書（L39-51）の両方**を更新
6. `th_safety/package.xml` / `CMakeLists.txt` に依存を追加: **`geometry_msgs` / `tf2_ros` / `tf2_geometry_msgs` / `ament_cmake_gtest`**（現状 `rclcpp` / `std_msgs` / `sensor_msgs` / `th_system_msgs` のみ）

> **4 と 5 を落とすと矛盾が検出できない。**テストは `twist_mux.yaml` を読まずに
> インライン辞書を使っているので、`/cmd_vel_retreat` を publish し続けたまま**通ってしまう**。
> 実機では `/cmd_vel_behavior` に誰も反応しない。

**段階 2 の出口**: Gazebo で故障注入 13 項目が自動で回り、
実機（モータ電源断）で `/cmd_vel` がゼロになることをトピックで確認できること。
**`DEBT-2`〜`DEBT-4` が解除され、`WP-SAFE-01` がバイパスを重大フォルトとして扱うこと**
（**検出そのものは段階 0 の `WP-ESP32-01` が用意する**。§1.1）。

> **`DEBT-1` の正式な解除は段階 7 である。**解除条件は「始業点検 項目 1 が OK」であり、
> それを実行できるのは `WP-MAINT-01`（段階 7）だから。
> 段階 2 で行うのは**検出できるようにすること**までで、解除ではない。

---

## 6. 段階 3 — 手動系と共通 UI 部品

| WP | 名前 | 依存 | 完了条件 |
| --- | --- | --- | --- |
| `WP-UI-03` | 走行タブ（`VirtualStick` / `SpeedPreset` / W-6） | `WP-UI-01` | `/ui/jog_lease` が 5 Hz 以上で出る。**`ui.jog.release` を送っていない**。W-6 が本文レイアウトを動かさない |
| `WP-TRANSIT-01` | `MANUAL`（S-11） | `WP-UI-03`, `WP-SAFE-04` | スティックで `PAUSE ⇄ RUN`。リース満了で停止。速度上限がリミッタ側で効く |
| `WP-ROUTE-01` | `route_recorder` ＋ 保存（新版＋旧版 1 世代） | `WP-STATE-02`, `WP-PARAM-02` | `REC` 中だけ記録。`ui.save` で終点確定。`previous/` へ退避 |
| `WP-TRANSIT-02` | `TEACH_MANUAL`（S-13） | `WP-TRANSIT-01`, `WP-ROUTE-01` | S-11 と**同一の走行タブ**＋教示タブ 1 枚。操作カードに「走行」が**無い** |

**段階 3 の出口**: 手動走行が新アーキで完動する。
**これで他方式が使えないときのフォールバックが成立し、保守モードの開始条件（`IDLE`/`MANUAL`）も満たせる。**
`WP-MEAS-03`（手押しのベースライン）と比較する測定を開始できる。

---

## 7. 段階 4 — 追従系

| WP | 名前 | 依存 | 完了条件 |
| --- | --- | --- | --- |
| `WP-PERC-01` | `person_tracker_bridge` → `PersonTargets`（候補一覧＋自動選択） | `WP-MSG-01` | `/person/targets` が出る。`auto_select_hold_s` で `evt.auto_selected` |
| `WP-TRANSIT-03` | `follow_runner`（`FOLLOW`） | `WP-PERC-01`, `WP-SAFE-03` | `CONFIRM` で角速度のみ／`RUN` で軌跡追従。近づいたら**停止**（退避しない） |
| `WP-UI-04` | S-10（レーダー・対象選択・「起動中」表示） | `WP-UI-03`, `WP-PERC-01` | 候補が出たら「起動中」が隅へ退く |
| `WP-TRANSIT-04` | `TEACH_FOLLOW`（S-12） | `WP-TRANSIT-03`, `WP-ROUTE-01` | S-10 と同一の走行タブ＋教示タブ。記録開始前に対象を 1 度提示 |
| `WP-STATE-04` | `tracker_enabled` の分離（意図的 OFF とフォルトの区別） | `WP-PERC-01` | 故障注入 §10-9 が通る |

**段階 4 の出口**: 追従走行の KPI（ロスト回数/100 m・誤追従回数）の計測を開始できる。

---

## 8. 段階 5 — 再生

| WP | 名前 | 依存 | 完了条件 |
| --- | --- | --- | --- |
| `WP-ROUTE-02` | `map_session`（`VENUE` / `ROUTE` の 2 スロット） | `WP-ROUTE-01` | スロット切替が SIGTERM ＋ respawn で行われる。`deserialize` を使わない |
| `WP-SAFE-05` | 自己位置喪失を重大フォルトに（`DEBT-5`） | `WP-ROUTE-02`, `WP-SAFE-01` | 故障注入 §10-13 が通る |
| `WP-ROUTE-03` | `replay_runner`（`LOCALIZE` → `READY` → `RUN`。**Nav2 を使わない**） | `WP-ROUTE-02` | 逆再生が同じコードで動く。終端で `PAUSE`＋案内 |
| `WP-UI-05` | S-14（経路選択・順/逆・初期姿勢・地図更新） | `WP-UI-03`, `WP-ROUTE-03` | 地図更新 ON のときだけ「保存」が出る |

**段階 5 の出口**: 教示 → 再生の往復が閉じ、`replay_drift_m_per_100m` が測れる。

---

## 9. 段階 6 — 試験場内

| WP | 名前 | 依存 | 完了条件 |
| --- | --- | --- | --- |
| `WP-ONSITE-01` | `two_point_core` ＋ `pin_registrar`（ランタイム登録） | `WP-PERC-01`, `WP-ROUTE-02` | **①がゴール**（`U-3`）。ロスト中は猶予なしで拒否。ピンが再起動後も残る |
| `WP-ONSITE-02` | `venue_navigator`（**`FollowPath` 方式**） | `WP-ONSITE-01` | `PAUSE → ui.run` で**再プランが走らない**。`BLOCKED` がタイムアウトしない |
| `WP-ONSITE-03` | `wait_clear_gate` | `WP-ONSITE-02`, `WP-SAFE-04` | 故障注入 §10-7 が通る。`WAIT_CLEAR` 中は手動 UI が非活性 |
| `WP-ONSITE-04` | `PREP` の地図修正（人の映り込み消去）＋ `RETURN` | `WP-ROUTE-02`, `WP-MEAS-02` | 消したセルが**未知（-1）**になる。元に戻せる |
| `WP-UI-06` | S-20 試験準備（地図／対象選択タブ・登録・ピン編集） | `WP-ONSITE-01`, `WP-UI-04` | 対象未選択なら登録ボタンが非活性 |
| `WP-UI-07` | S-21 試験（待機場所の宣言・行き先・呼び寄せ・作業中） | `WP-ONSITE-03`, `WP-UI-06` | `working == true` で行き先が拒否される |

**`WP-MEAS-02` の結果次第で `WP-ONSITE-04` の範囲が変わる**（人物マスクが要るかどうか）。

**段階 6 の出口**: 目標 G2（停止精度）と G3（前日準備）の測定を開始できる。

---

## 10. 段階 7 — 保守

| WP | 名前 | 依存 | 完了条件 |
| --- | --- | --- | --- |
| `WP-MAINT-01` | `opcheck_runner` 4 項目 | `WP-UI-01`, `WP-SAFE-01` | **項目 1（ESTOP）が OK になる**＝`DEBT-1` の解除証明 |
| `WP-MAINT-02` | `calib_runner`（直進・旋回・IMU）＋ 履歴 3 世代 | `WP-CALIB-01`, `WP-MAINT-01` | 検証で許容範囲を超えたら適用前に戻る。ロールバックが効く |
| `WP-ESP32-02` | `wheel_radius_scale` の双方向適用 ＋ IMU オフセットの NVS | `WP-MAINT-02` | 往復で元に戻る。A10（`\|k−1\| ≤ 0.1`）が効く。再起動でオフセットが復元される |
| `WP-MAINT-03` | 故障診断 S-31 | `WP-MAINT-01` | NG の内容に応じたチェック手順が出る |
| `WP-UI-08` | S-30 / S-31 / S-40 / S-50 | `WP-MAINT-01`〜`03` | S-50 に人物追跡 ON/OFF と開発モードがある |
| **`WP-CLEAN-01`** | **旧 msg・旧サービス・旧パッケージの削除**（`O-3`） | 上記すべて | `RobotMode.msg` 等を削除しても全テストが通る。`th_mode_manager` / `th_planning` / `th_calibration` / `th_config_manager` を削除 |

**段階 7 の出口**: 1 日の流れが全部 WebUI で完結する（**`SD-1`**「操作系は WebUI のみ」が達成される）。
**`DEBT-1`（物理 E-Stop バイパス）の解除もここ**（`WP-MAINT-01` の始業点検 項目 1）。**両者は別物である。**

---

## 11. 段階 8 — ライン誘導・電子リード（**後回し**）

| WP | 名前 | 依存 | 備考 |
| --- | --- | --- | --- |
| **`WP-LINE-00`** | **カメラの選定と帯域検証** | — | **実装より前に行う。**要求と接続方式は [hardware](DetailedDesign-hardware.md) §4（**CSI・型番は決めない**）。測定手順と合格条件は同 §2.1 |
| `WP-LINE-01`〜 | `line_runner` ＋ S-15 | `WP-LINE-00` | 内部設計は未着手。受け入れ条件は [transit](DetailedDesign-transit.md) §6.4 |
| `WP-LEASH-01`〜 | `leash_runner` ＋ S-16 | — | 同 §7.4。**`PAUSE` 中に張力をかけても発進しない**が最重要 |

**ラインマップの編集画面は設計しない**（`O-d1`。原典が明示的に後回しを宣言）。

---

## 12. 各段階の検証

| 段階 | 単体 | 統合 | シミュレーション | 実機 |
| --- | --- | --- | --- | --- |
| 0 | `pytest`（純粋コア） | — | — | **測定 4 件**（通電） |
| 1 | 〃 | `colcon test` | `gazebo.launch.py` | 電源断で `INIT→IDLE→MANUAL` |
| 2 | 〃＋property test | 〃 | **故障注入 13 項目** | 電源断で `/cmd_vel` ゼロ／**通電で物理 E-Stop・ウォッチドッグ** |
| 3 | 〃 | 〃 | `narrow_room` | 通電で手動走行 |
| 4 | 〃 | 〃 | `cluttered` / `lost_reacquire` | 通電で追従走行 |
| 5 | 〃 | 〃 | `wide_area` | 通電で教示→再生 |
| 6 | 〃 | 〃 | `panel_shuttle` | 通電で前日準備＋当日運用 |
| 7 | 〃 | 〃 | — | 通電で始業点検・校正 |

```bash
# 単体（ROS2 不要・最速）
python3 -m pytest src/th_testing/test/test_state_core.py -v

# 統合
colcon test --packages-select th_testing --event-handlers console_direct+
colcon test-result --verbose

# 故障注入だけ
colcon test --packages-select th_testing --ctest-args -R fault_injection

# 一括
bash scripts/run_tests.sh --all --sim
```

### 12.1 実機での「電源断でできること」と「通電が要ること」

`ImplementationPlan.md` は「実装時、実機をモータの電源のみ切断し PC と接続する」と定めている。

| 電源断でできる | 通電が要る |
| --- | --- |
| 疎通確認・モード遷移・UI の全操作 | 走行を伴う確認すべて |
| `/cmd_vel` の内容の観測（ゼロになるか） | 制動距離の測定 |
| フォルト検知のタイミング測定 | 物理非常停止の動作確認 |
| エンコーダの手回し確認 | ESP32 ウォッチドッグの停止確認 |
| IMU のデータ・校正状態 | 直進・旋回校正 |

**通電が要るものだけ人が関与する**（`ImplementationPlan.md`）。

---

## 13. パケット一覧（依存グラフ）

```
段階0  NET-01 ─► ESP32-01 ─► MEAS-01 ─► PARAM-01 ─► PARAM-02 ─┐
          │                    │  (O-5)                        │
          │                    └─► MEAS-02                     │
          └─► SAFE-00 ─► MEAS-04 ────────────────────────────┤
       MSG-01 ──────────────────────────────────┬────────────┤
       STATE-01 ────────────────────────────────┴─► STATE-02 ─► STATE-03
段階1                                                 │
       UI-01 ◄─────────────────────────────────────┘─► UI-02
段階2  SAFE-02 ─┐                 CALIB-01
                ├──────────────────────► SAFE-03 ─► SAFE-04 ─► TEST-01
       SAFE-01 ─┘   ▲
                    └── ESP32-01（段階 0）が firmware_flags を供給
段階3  UI-03 ─► TRANSIT-01 ─┐
       ROUTE-01 ────────────┴─► TRANSIT-02
段階4  PERC-01 ─► TRANSIT-03 ─► TRANSIT-04
                 └─► UI-04      STATE-04
段階5  ROUTE-02 ─┬─► SAFE-05
                 └─► ROUTE-03 ─► UI-05
段階6  ONSITE-01 ─► ONSITE-02 ─► ONSITE-03
                 └─► UI-06 ─► UI-07      ONSITE-04
段階7  MAINT-01 ─► MAINT-02 ─► ESP32-02
                 └─► MAINT-03 ─► UI-08 ─► CLEAN-01
段階8  LINE-00 ─► LINE-01…        LEASH-01…
```

**段階 0〜7 で 48 パケット**（0:**11** / 1:6 / 2:**6** / 3:4 / 4:5 / 5:4 / 6:6 / 7:6）。
**総数は変わっていない**——`WP-ESP32-01` が段階 2 から段階 0 へ移った（§1.1）。
これが `Spec-modes.md` の 18 モードのうち、`LINE` / `LEASH` を除く **16 モード**を覆う。
段階 8 は `WP-LINE-00`（カメラ選定）だけを確定させ、残りは未着手。

> **この数は主張ではなく機械的に数える。**[open](DetailedDesign-open.md) §6 の検査 4
> （`LINE` / `LEASH` 以外のすべてのモードに少なくとも 1 つの作業パケットが対応している）で突き合わせる。

---

## 14. 逆引き

| 完全設計書 | ここでの反映 |
| --- | --- |
| `Spec.md` §4（開発の進め方） | §2（PoC では削らずに全部作る／MVP で削る） |
| `Spec.md` §11（先に潰す 4 つ） | §3（`WP-MEAS-01`〜`04`） |
| `Spec-params.md` §8 | 同上 |
| `Spec-safety.md` §9（故障注入） | §5（`WP-TEST-01`） |
| `Spec-ops.md` §1（導入） | §10（`WP-MAINT-01` / `WP-MAINT-02`） |
| `Spec-transit.md` §7.3（KPI の測り方） | §7・§9 の出口 |
| `docs/plan/ImplementationPlan.md` | §0（規約）・§12.1（実機の条件） |
| `docs/plan/task.md` 1（ネットワーク改善） | §3（`WP-NET-01`） |
| `docs/plan/task.md` 2・3（速度改善と試験） | §5（`WP-SAFE-03`）＋ §7 |
| `docs/plan/task.md` 5（教示再生） | §8 |
| `docs/plan/task.md` 6・7（区画線・電子リード） | §11 |
