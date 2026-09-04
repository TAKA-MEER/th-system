# VISION.md — TH システム完成形

このドキュメントは、TH システム（配電盤上部確認ロボット移動機構）の**目指す完成形**を記述する。
読者は開発者本人と、コーディングを行う AI。

**運用ルール**: 方針を変更する際は、コードを修正する前に必ず本書（または本書が委譲した先の設計書）を更新する。
コードと文書が食い違う場合は文書が正であり、実装を追いつかせる（CLAUDE.md「方針変更時のルール」）。
`docs/architecture.md` は現状実装の記述（as-built）であり、本書は目指す姿（to-be）である。

> ⚠️ **2026-09-01 〜 特例運用中。**実現可能性デモのため、安全システムのソフト層バイパス等を
> 期間限定で許可している。範囲・遵守事項・記録義務は **§2.5** を見ること。

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

## 2.5 特例運用中 — 実現可能性デモ優先（2026-09-01 〜 特例解除まで）

**2026-09-01 決定（ユーザー指示）。**期限までに現行の実装計画では実現可能性（動くデモ）を
示せない見込みのため、**期間限定の特例**を発動する。

### 目的

**手動教示（手動運転で経路を記録）と教示再生（`REPLAY`）の一気通貫を動かすこと**、これだけを
最優先にする。最小ループ ＝ 記録開始 → 手動運転 → 保存 → 一覧から選択 → 初期姿勢合わせ → 順再生。
逆再生・地図書き足し・先導同行の作法・KPI 計測は特例期間の対象外。

### 特例で許可すること

- 一部の実装・試験の**省略**（変異テスト・統合テスト・実機試験項目を含む）
- 安全システムの**ソフト層のバイパス・省略**（下表）
- モード FSM（新設計の `TEACH_MANUAL` / `REPLAY`）を、上記最小ループに必要な経路だけ通す暫定実装

| バイパス可 | 内容 |
| --- | --- |
| `obstacle_limiter` の LiDAR 減速・停止 | 素通し可（`/cmd_vel_muxed` → `/cmd_vel` パススルー） |
| `safety_monitor` のフォルトタイムアウト | 判定を緩める／無効化してよい |
| モード FSM の遷移ガード | 最小ループに必要な遷移だけ通す暫定実装でよい |
| params `registry.yaml` の未確定値による起動停止 | 仮値で起動してよい |
| 変異・統合・実機の試験項目 | 省略してよい（記録は必須） |

### 特例でも無効化しないこと（§2 のうち解除不可）

- **G1（人・物への接触 0 件、意図しない挙動 0 件）** — 特例の上位。バイパスの結果 G1 を脅かすなら止める
- **物理非常停止ボタン** と **ESP32 ウォッチドッグ（600ms）** — 開発モードでも無効化しない（§2）
- **モードの切替はすべて明示操作**（意図しないタイミングで動き出さない）
- **操作系は WebUI のみ**

### 記録義務と復元

- 省略・バイパスは 1 件 1 行で [docs/plan/EXCEPTION-LEDGER.md](docs/plan/EXCEPTION-LEDGER.md) に記録する
- コード側には grep 可能なタグ **`# WAIVER(demo):`**（C++ は `// WAIVER(demo):`）を付ける
- **特例解除時、台帳の全件を正規実装でクローズする**まで `feat/demo-teach-replay` を `main` にマージしない
- **完成形（`docs/plan/spec/`）は変更しない。**このデモは spec を置き換えない。デモ後の正規実装は
  spec に従って作り直す（デモコードの流用可否は台帳で個別に判断）

### 作業ブランチ

`feat/demo-teach-replay`（`main` の PR #20 マージ後の状態から分岐）。

### 実機フィードバックによる設計変更（バイパスではない。spec を更新した）

デモを実機で試して判明した使い勝手の問題のうち、方針変更に当たるものは
CLAUDE.md「方針変更時のルール」に従い **spec を先に更新**した（WAIVER ではない）。

- **2026-09-01 — UI 非常停止の解除後の復帰**: WebUI の非常停止ボタンは手動走行中に
  誤って触れやすく、そのたびにメインメニュー経由で入り直すのが実運用の障害になった。
  **UI ボタン起因の `ESTOP`（重大フォルトを伴わないもの）に限り**、解除後に
  「元のモードに戻る／メインメニューへ」を選べるよう変更（`CARRY` と同型）。
  重大フォルト起因の `ESTOP` は従来どおり `IDLE` のみ。
  更新: [Spec-modes.md](docs/plan/spec/Spec-modes.md) §3.1.1 `SM-3.1.1-11`・§4.1、
  [Spec-safety.md](docs/plan/spec/Spec-safety.md) §4.1。

- **2026-09-02 — `PAUSE` は「走っていたものを止めた」状態に限る**: 実機で
  「再生を押しても機体が動かない」が再現した。原因は共通遷移 `C-01`（ジョグ介入）
  と `C-03`（回復可能フォルト）が **`mode=* state=*`** で `PAUSE` へ落とすため、
  経路をまだ積んでいない準備状態（`REPLAY` の `ROUTE_SEL` / `LOCALIZE` / `READY`）
  からも `PAUSE` に入ってしまうこと。`PAUSE` からの `ui.run`（`T-REPLAY-07`）は
  「経路は既に積んである」前提で `resume_path` を出すだけなので、**経路選択が
  丸ごと飛ばされ、再生は受理されるのに追従する対象が無い**状態になる。
  実機再現: `REPLAY` に入り（`ROUTE_SEL`）→ ジョグを握る → `PAUSE` →
  再生が受理され `RUN` へ。`load_route` は一度も発火しない。
  起点合わせのためにジョグを触る／`ESP32_DISCONNECTED` が一度出るだけで発生する。

  **変更**: 走っていないものは止められない、という原則を明示する。
  1. `C-01` / `C-03` は、現在の状態がそのモードの**準備状態**（走行前。`REPLAY`
     では `ROUTE_SEL` / `LOCALIZE` / `READY`）のときは `PAUSE` へ落とさず
     **状態を保つ**。停止すべき動作がそもそも無く、落とすと準備の進捗だけが
     失われるため。ジョグ介入の記録（`jog_active`）とフォルト表示は従来どおり行う。
  2. 安全網として `PAUSE` / `SAVED` からの `ui.run`（`T-REPLAY-07` / `T-REPLAY-10`）に
     **経路が読み込まれていること**を要求するガードを課す。満たさない場合は
     拒否理由を UI に返す。あわせて `REPLAY` の `PAUSE` から `ui.route_select` で
     経路を選び直せるようにし、詰まったときの復帰路を用意する。

  更新: [Spec-modes.md](docs/plan/spec/Spec-modes.md) §3.1.2（`SM-3.1.2-020` 周辺）。

- **2026-09-03 — 再生時の地図凍結と自己位置推定継続（WS-9N）**: 実機で
  再生の `READY` 時に自己位置が経路始点から **1.52 m / 41.8°** ずれ、
  走り出すと一気に発散した。原因は再生中も slam_toolbox が **地図作成モード**で
  動いていたこと。ずれた自己位置に新しいスキャンを描き足して地図が汚れ、
  次のスキャンマッチがさらにずれる悪循環になっていた。

  **変更**: SLAM ノードを `async_slam_toolbox_node` から
  `map_and_localization_slam_toolbox_node` へ差し替える（`bringup.launch.py`）。
  教示中は mapping（`set_localization_mode(false)`）、再生中は localization
  （`set_localization_mode(true)` ＝ **地図を凍結したまま自己位置推定だけ継続**）。
  保存は mapping モードのまま serialize（localization では書き出されない。
  2026-09-03 実機確認）。再生の読み直しは localization に入ってから
  `deserialize_map(match_type=3, initial_pose=経路始点)`（毎回 slam_toolbox を
  作り直してから。WS-9S）。
  旧 §8 の「地図作成停止 ＝ 自己位置推定停止ではない」要求に実装を追いつかせた。

- **2026-09-04 — 一瞬の重大フォルトで止まり、元のモードへ戻れない（WS-9O）**:
  実機で「一瞬でもエラーが出ると停止し、そこから復帰できない」。原因は独立した 2 つ。

  1. **重大フォルトはタイムアウトを 1 回超えた瞬間に発火する。**猶予を持つのは
     `DRIVE_RUNAWAY` だけで、それは W-06 でバイパス中。`safety_monitor` の監視ループ
     （100ms）自身が一時的に遅れると、複数の入力が同じ判定周期で同時にタイムアウト
     超過に見える。実機ログ: `ESP32_DISCONNECTED` と `LIMITER_DEAD` が同じ 1ms に
     発火し、**26ms 後に両方とも解除**された。実体の途絶ではなく検知側の取りこぼし。
  2. **フォルトは edge で publish されるため、26ms のフォルトでも `active=true` は
     必ず `state_manager` に届く。**`C-06a`（`mode=* state=*` → ESTOP、ガード無し）で
     必ず ESTOP に落ちる。`fault.cleared` には ESTOP からの出口が無い。唯一の出口
     `C-09f`（`ui.resume_ack`）の行き先は IDLE 固定で、`C-09c`（元モードへ復帰）の
     ガード `estop_resume_prev` は `ctx.estop_from_ui` を要求する。**フォルト起因の
     ESTOP は構造的に元のモードへ戻れない。**再生中なら経路選択からやり直しになり、
     そこで再び 6.9MB の地図を読むので落ちやすさが再発する。

  **変更**（ユーザー決定 2026-09-04）:
  1. **重大フォルトの発火に保持時間を課す。**`LIMITER_DEAD` / `MUX_DEAD` /
     `STATE_INCONSISTENT` は、条件が `critical_fault_hold_ms` 継続したときだけ
     報告する。監視ループ 1 周期分の遅れで生じる単発の誤検知を消す。タイムアウト値
     そのもの（`limiter_dead_ms` 等）は変えない。回復可能フォルトは一時停止から
     正常に再開できるため対象外。
  2. **フォルト起因の ESTOP からも元のモードへ戻れるようにする。**
     `estop_resume_prev` から `estop_from_ui` の要求を外す。**フォルトの種類で
     除外は設けない**（ユーザー決定）。復帰先は `$prev_mode` の **PAUSE**（停止状態）
     で、走り出すには操作者がもう一度「再生」を押す必要がある。フォルトが実際に
     消えていること・物理ボタンが解放されていることは従来どおり要求する。
     WebUI の非常停止ウィンドウも、フォルト起因のとき「確認」だけでなく
     「元のモードに戻る／メインメニューへ」を出す。

  物理非常停止と ESP32 のウォッチドッグ（600ms）はどちらも変更しない。

- **2026-09-04 — 一時停止の理由が伝わらない／ノイズ 1 点で止まる（WS-9P）**:
  実機で 2 件。どちらも「単発の信号で決め打ちする」同じ形。

  1. **ESP32 が一瞬途切れて止まったのに、画面が「記録の終端に達しました。終了を
     押してください」と出す。**`S14Replay.poseText` が状態名 `PAUSE` だけを見て
     文言を決めていた。`PAUSE` には終端到達（`T-REPLAY-09`）・フォルト（`C-03`）・
     ジョグ介入（`C-01`）・停止ボタン（`T-REPLAY-06`）が**すべて**落ちてくるので、
     状態名だけでは区別できない。FSM には `T-REPLAY-07`（`PAUSE --ui.run
     [route_loaded]--> RUN` ＋ `resume_path`）があり、`_from_index` は保持される
     ので**途中から再開できるはず**だったのに、画面が終了を促していた。

  1b. **その「再生」を押しても半分の確率で無反応（WS-9Q。2026-09-04 追加報告）。**
     `/route/status` には `route_recorder` と `replay_runner` の**両方**が
     publisher を持ち、どちらも無条件に 2Hz で出していた。再生中も記録側が
     `points=0` を流すので、`state_manager._on_route_status` が更新する
     `route_loaded` が **4Hz で真偽を往復**する。`T-REPLAY-07` のガードはこれを
     見るため、「再生」を押した瞬間がどちらのメッセージの直後かで受理／拒否が
     変わっていた。実測: `/route/status` は publisher 2 個・**4.002 Hz**。
     `/route/preview` は WS-6.4 で同じ理由（表示の点滅）から記録側をゲート済み
     だったが、`/route/status` だけ残っていた。
     → **自分のモードのときだけ publish する**（`owns_route_status`。教示系は
     記録側、`REPLAY` は再生側、それ以外は誰も出さない）。
  2. **何も無いところで止まり、しばらくして自分で走り出す。**
     `obstacle_limiter_core.observe_cone` が円錐内の**最小値をそのまま**採用して
     いた。スキャン 1 枚のノイズ 1 点で停止距離を割り、次のスキャンで消えるので
     独りでに再開する。実測（2026-09-04）: `/scan_filtered` は 720 点・分解能
     **0.499°**、前方円錐（±28.6°）に **105 点**。停止距離 0.425 m では 1 ビームの
     弧長が 3.7 mm なので、**幅 2 cm の実体は 5 点以上に映る**。1 点で止めるのは
     過敏すぎる。

  **変更**:
  1. `replay_runner` が `RouteStatus.arrived`（終端まで走り切ったか）を publish し、
     UI は `PAUSE` の文言をそれで出し分ける。終端なら従来どおり終了を促し、
     それ以外は「**「再生」を押すと止まったところから続きを走ります**」と案内する。
     あわせて `resume_path` で到着ラッチを解除する（解除しないと終端で再生を押した
     ときに `evt.arrived` を出し直せず `RUN` のまま固まる）。
  2. `observe_cone` は円錐内の **k 番目に小さい距離**を採用する
     （`obstacle_min_points`、既定 3）。有効点が k 未満のときだけ最小値へ退避する
     （LiDAR が壊れている場合に安全側へ倒すため）。**円錐の角度・停止距離・
     スキャン途絶判定・死角マスクは一切変えない。**変えるのは「何点あれば障害物と
     みなすか」だけ。

- **2026-09-04 — 画面を触らないと 30 秒で黙って止まる（WS-9R）**: 実機で
  「謎の一時停止が発生する。画面をスクロールしたりすると復帰する」。

  バグではなく**設計された在席確認**（Spec-safety §6.2）が効いていた。
  `zones.derive_limits()` は「使用中の端末」を
  **`interacting` かつ最後の**操作**から `ui_active_window_s`（30 秒）以内**と
  定義し、0 台なら `zone=NA` / `speed_limit=stop` / `auto_brake=True` に倒す。
  `speed_limit=stop` は `obstacle_limiter` の `screen_limit_mps=0` になるので
  機体が止まる。`last_input` を更新するのは `pointerdown` / `keydown` /
  `touchstart` だけなので、**画面を見ているだけでは在席とみなされない**。
  スクロールで復帰するのは `touchstart` が発火するから。
  実機確認: 端末未接続時に `zone: NA` / `speed_limit: stop` /
  `applied_limit_mps: 0.0`。

  問題は 2 つある。**再生中、操作者が見るのはロボットであってタブレットではない**
  ので 30 秒ごとに触れという要求が用途に合っていない。そして**止まったことが
  画面にもログにも一切出ない**（フォルトでも一時停止でもなく速度上限が静かに
  0 になるだけ）ので、操作者に理由が分からない。

  **変更**（ユーザー決定 2026-09-04）:
  1. **在席の根拠を「操作したか」から「見ているか」に変える。**
     `derive_limits()` の窓は `last_input`（最後のタッチ）ではなく
     `last_seen`（その端末から最後にメッセージが届いた時刻）で測る。
     `/ui/active_screen` は 2Hz で出続けるので、**アプリが前面にあって接続が
     生きている限り在席**とみなす。画面を消す・別アプリに切り替える
     （`interacting=false`）、端末が落ちる・回線が切れる（メッセージが途絶）は
     従来どおり `stop` に倒れる。窓の長さ（`ui_active_window_s`）は変えない。
  2. **止まった理由を画面に出す。**原因（在席未確認／障害物／死角未校正／フォルト）を
     画面に表示する。黙って止まるのをやめる。

     追修正（同日・実機「赤い帯が基本的に常に出ていて邪魔。一部表示を隠して
     しまっている」）: **機体が走るはずのときだけ**出す
     （`attributes.yaml` の `run_state`。REPLAY→RUN / TEACH_MANUAL→REC）。
     走らせていない間は速度指令が無いのが normal で、リミッタは常に
     `ZERO_STALE` を返す。それを「止まっている理由」として出すと待機中ずっと
     帯が出たままになる。`ZERO_STALE` は理由から外した（操作者に打つ手が無く、
     RUN 直後に一瞬出て点滅する）。帯は 1 行に切り詰め、隠す面積を最小にする。

  物理非常停止と ESP32 のウォッチドッグ（600ms）は変更しない。

- **2026-09-04 — 再生で経路を選ぶたび地図が変わる・自己位置が飛ぶ（WS-9S）**:
  実機で「再生走行で地図を上書きしている。元の地図を参照して自己位置推定して
  いないかも」。切り分けで確定（memory `replay-deserialize-accumulates-bug`）:
  経路選択のたび `slam_control` が長寿命の `slam_toolbox` へ `deserialize_map` を
  投げているが、このノードは deserialize で既存ポーズグラフをクリアせず**上乗せ**
  する。同じ `test.posegraph`（ファイルは不変）を 3 回読み直させたら結果が毎回
  変わり（`/map` が 298×167 → 333×262 → 345×213、原点も移動）、`map→odom` TF が
  読み直しのたびに数 m・100〜200° 飛んだ（2 回目の後は yaw +192°＝ほぼ逆向き。
  逆再生が 1.4 m で PAUSE）。1 回目（教示直後）だけ誤差が小さく前進再生は完走
  していた。ノードは終始 localization モードなので**ディスクの `.posegraph` は
  無傷**——壊れるのは稼働中ノードのグラフと配信中の `/map`（→ WebUI 表示）。

  方針（§8 / WS-9N：再生中は地図凍結・保存地図で自己位置推定）は不変。実装バグ。

  **変更**（ユーザー決定 2026-09-04・案A）: 再生の地図読み直し（`/map_session/open`
  reload）は、毎回 `slam_toolbox` を SIGTERM して `bringup.launch.py` の
  `respawn=True` で作り直し、**まっさらなノードに対して `deserialize_map` を
  1 回だけ**呼ぶ。`set_localization_mode(true)` →
  `deserialize_map(match_type=3, initial_pose=経路始点)` の中身は不変で、その手前に
  respawn を挟むだけ。`discard_map`（`docs/使い方.md` §4-5）が既に使っている
  「SIGTERM → respawn 待ち」を reload でも同期でやりきる形にする。

  **トレードオフ**: 経路選択に respawn ぶんの待ちが増える（短経路で数秒、長距離
  20 MB のポーズグラフで ~30–45 秒）。`docs/使い方.md` §4-4 / §4-6 に明記する。

  更新: `slam_control.py` / `replay_runner.py` / `slam_control_logic.py`。

- **2026-09-04 — 教示再生が遅く変える手段がない／手動ジョグの旋回が遅い（WS-9T）**:
  実機（ユーザー）。

  1. 再生の巡航速度は `replay_runner` のノード内リテラル既定
     （`cruise_speed_mps=0.18` m/s・`max_yaw_rate_rps=0.5` rad/s）固定で、launch 引数も
     画面コントロールも無い。
  2. 手動ジョグのその場旋回が遅い。旋回上限 `w_max=0.6` rad/s（≈34°/s）に加え、
     WebUI スティックが旋回指令を前進速度プリセットで絞る（`wz = wn * speedPct`）ため、
     低速設定では 0.15 rad/s（≈9°/s）。

  **変更**（ユーザー決定 2026-09-04）:
  1. 再生速度を **WebUI（S-14）から可変**にする。新トピック `/replay/speed_scale`
     （`std_msgs/Float32`、比率 0..1、latched）。S-14 の「再生速度」低速/中速/高速
     ボタンが publish、`replay_runner` が subscribe して pure-pursuit パラメータを
     作り直す（`route_replay_core.scale_replay_params`。`cruise` は
     `replay_cruise_min_mps`〜`cruise_speed_mps` の線形補間）。**走行中も切替可。**
     既定は中速相当（比率 0.65 → cruise ≈ 0.33 m/s、従来より速い）。実 m/s 換算は
     `replay_runner` が持ち、ブラウザは比率だけを送る（jog の speed_preset と同じ方針）。
  2. `w_max` を **0.6 → 1.2** rad/s（`registry.yaml`）。かつ WebUI スティックの
     **その場旋回（前進成分ゼロ）を速度プリセットから切り離す**
     （`stickGeometry.scaleJogCmd`：pure turn は `wz = wn`、arc・前進は従来どおり
     `* speedPct`）。前進速度は据え置き、教示の低速走行はそのまま。

  **不変**: 速度指令の最終クランプ（`obstacle_limiter`）・twist_mux 優先度・ESP32
  ウォッチドッグ（600ms）は一切触らない。速度コントロールは上限を握らず、
  障害物・在席未確認時は従来どおり `obstacle_limiter` が最終的に絞る。

  更新: `route_replay_core.py` / `replay_runner.py` / `registry.yaml` /
  `stickGeometry.js` / `JogConsole.jsx` / 新 `ReplaySpeedControl.jsx` ほか WebUI。

- **2026-09-04 — 長距離再生で replay_runner が落ちる／別セッション経路を再生できない（WS-9U）**:
  実機。長距離経路（1150 点・現セッション）を再生しようとすると画面に「初期姿勢を
  推定できません。別の起動セッションで記録した経路の可能性があります」。短距離は正常。
  ユーザー指摘: WS-9S で reload のたび slam_toolbox を作り直して経路の地図を読み直す
  以上、別セッションでも再生できてよいはず。

  実機ログで確定した原因は **2 つ、別物**:

  1. **replay_runner のクラッシュ（本命）**。`_reload_map` が `/map_session/open` を
     `call_and_wait`（= `rclpy.spin_until_future_complete(node, future, executor=node._executor)`）
     で待っていた。これは自分の実行スレッドの中から、`main()` で既に回している
     MultiThreadedExecutor を**再入 spin** する。`_spin_once_impl` の共有ジェネレータ
     （`wait_for_ready_callbacks` の `_cb_iter`）と共有リスト `_futures` を 2 スレッドが
     同時に壊し、`future.result()` が無関係なコールバックの例外まで再送出してノードごと
     落ちる。WS-9S 以前は reload が数秒で終わり窓が小さく潜在化していたが、respawn
     （5〜45 秒）＋ slam_toolbox 再起動の TF 嵐で、20/10/2Hz タイマ＋TF リスナを回す
     replay_runner では衝突がほぼ確実になった。**画面の「別セッション…」は LOCALIZE が
     進まないことからの UI の推測で真因ではない。**
  2. **セッション制限（`can_replay_route`）が WS-9S で不要に**。WS-9K-B は
     「別セッションの map 座標は現在の in-memory 地図と合わない」ため
     `route_session == current_session` を要求した。WS-9S 以降は経路選択で
     slam_toolbox を作り直しその経路の `.posegraph` を deserialize するので、地図は
     経路自身のもの。セッション一致は不要。

  **変更**:
  1. `call_and_wait`（`th_config_manager/service_call.py`）を **spin せず
     `future.done()` をポーリング**する実装に差し替え（`_wait_for_slam_restart` と同じ
     思想）。応答は呼び出し元ノードの `main()` の `executor.spin()` スレッドが処理する。
     呼び出し元 6 箇所は無変更で直る。各ノードの `MultiThreadedExecutor` は
     `num_threads=4` を明示（ポーリング中に 1 worker を塞ぐため下限保証。実機は 8 コア）。
  2. `can_replay_route` の map 経路判定を `bool(route_session)` に緩和（保存地図を
     持つ map 経路は別セッションでも可。posegraph が無い経路だけ弾く。実ファイルの
     有無は `slam_control._handle_map_reload` が確認）。S-14 の LOCALIZE 長引き
     しきい値を 20s → 60s（respawn ぶんの余裕）、文言を原因非依存に。

  **不変**: 安全チェーン・FSM・WS-9S の respawn 方式は変更なし。

  更新: `service_call.py` / `route_record_core.py` / `replay_runner.py` /
  `config_manager.py` / `slam_control.py` / `route_recorder.py` / S-14 WebUI。

- **2026-09-04 — 特徴の少ない廊下の再生で地図とずれ、補正されない（WS-9V）**:
  実機の長距離再生で、L 字コーナーの先の廊下で `map→base_link` が実機から
  〜0.4m ずれ、scan が壁からはみ出て obstacle_limiter が停止。

  **なぜ localization が補正しないか**: `map_and_localization_slam_toolbox_node` の
  localization は「odom で予測した pose のまわり **±0.25m**（`correlation_search_space_dimension`
  0.5 の半分）を相関スキャンマッチで探索し、`link_match_minimum_response_fine`(0.1) を
  超えたら採用」。ずれが 0.25m を超えると**真の pose が探索窓の外**で、マッチャーは
  それを一度も評価しない（「ずれを見ているのに直さない」のではなく、正しい pose に
  到達できない）。さらに特徴の無い廊下は廊下軸方向に相関の勾配が無いので、窓の中でも
  引き戻せない。復帰用のパーティクルフィルタ（AMCL 相当）も無い。`minimum_travel_distance`
  0.3 も相まって、補正頻度 < ドリフト蓄積速度になり負ける。

  **変更**（ユーザー決定 2026-09-04）: **`imu_enabled` の既定を `false` → `true`**
  （`bringup.launch.py`）。EKF が融合するのはジャイロ `vyaw` のみ（絶対方位は不使用。
  屋内磁気擾乱対策）。有効化の前提条件は揃っている: dps→rad/s 修正（2026-08-06）、
  `ekf_params.yaml` の `imu0` 欠落修正（2026-09-02）、実機で gyro fully calibrated・
  `/esp32/imu_data` 10Hz。エンコーダのみで欠けるコーナーの yaw 変化量をジャイロが
  埋める → ドリフトの上流を断つ。ジャイロ単位未修正ファームの個体でだけ
  `imu_enabled:=false`。

  **限界**: IMU は上流のオドメトリを直すが、特徴の無い長い廊下の廊下軸方向ドリフトは
  原理的に残る。SLAM localization のチューニング（探索窓拡大・補正頻度）は別途。

  更新: `bringup.launch.py` / `docs/使い方.md` §1-2 / `docs/architecture.md`。

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
