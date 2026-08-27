# 未確定事項・申し送り・レビュー指摘管理

[DetailedDesign.md](DetailedDesign.md) の詳細。**この文書の「未対応」が 0 になることが完成の条件。**

---

## 1. 詳細設計で解決した未確定事項

完全設計書が「詳細設計で決める」と明示的に送っていたもの。

| 元 ID | 内容 | 決定 | 場所 |
| --- | --- | --- | --- |
| **`O-d7`** | UI 非常停止ボタンの内部動作と物理 E-Stop との二重化 | **UI ボタンは `safety_monitor` を経由する**（直結しない）。押下は継続送信＋押下側にラッチ。集約は `estop_hw \|\| estop_ui` の 1 か所 | [safety](DetailedDesign-safety.md) §6 |
| **`O-d10`** | 直進校正の補正値が ESP32 のコンパイル時定数である問題 | **PC 側のスケール係数 `wheel_radius_scale` で解決。**ファーム変更・再書き込み不要。`\|k−1\| ≤ 0.1` を起動時にアサート | [maintenance](DetailedDesign-maintenance.md) §4 |
| `Spec.md` §6.1 #8 | ライン誘導走行用カメラの選定 | **RaspberryPi4 の CSI ポートを推奨**（LiDAR と USB 帯域を共有しないため）。実装は後回しだが**機材選定は先に行う** | [transit](DetailedDesign-transit.md) §6.3 |
| `Spec-safety.md` §2.4 | 障害物リミッタの置き場所 | **多重化の後段。**ただし挙動ノード側にも判定を残す二層構造（`d_floor < d_behavior`） | [safety](DetailedDesign-safety.md) §3 |
| `Spec-modes.md` §3.1 | 遷移表の実装形態 | **YAML データ＋純粋コア。**スキーマ・トークン・ガード 24 件・effect 一覧を確定 | [state](DetailedDesign-state.md) §3 |
| `Spec-params.md` §0 | (c) に目標値を置かない仕掛け | **registry ＋ `class`／`status` 組合せ規則 S1〜S5 ＋ 起動時アサーション。**sentinel は使わない | [params](DetailedDesign-params.md) §1.3 |

**`O-d7` / `O-d10` は正本へ戻した**（2026-08-16）。`Spec-open.md` §5.3 から §3 へ移り、
**`F-39`（UI 非常停止の内部動作と二重化）／ `F-40`（スケール係数方式）**になっている。
残りの 4 行は正本側が「詳細設計で決める」と書いたままでよい
（決定の所在が詳細設計にあることを正本が明示しているため、二重管理にならない）。

---

## 2. 安全負債の台帳（**着手前提条件**）

**「いつか直す TODO」ではない。**各行が特定の段階の blocking precondition である。

| # | 現象 | 根拠 | 影響 | 解除条件（検証手段） | 解除する段階 |
| --- | --- | --- | --- | --- | --- |
| **DEBT-1** | `ESTOP_BENCH_TEST_BYPASS` が有効で物理 E-Stop が効かない | `esp32/src/config.h:130` / `main.cpp:119-121` | **安全チェーン層 1 が不在。**`Spec-safety.md` §10 に反する | **始業点検 項目 1（ESTOP）が OK になること。**バイパス有効では構造的に通らない | 段階 2（出口）／段階 3（入口） |
| **DEBT-2** | `blind_angle_ranges` が幅ゼロ（40/40, 130/130, 220/220, 310/310） | `th_bringup/config/perception_params.yaml:15-23` | 死角マスクが無効。**この状態でリミッタを有効にすると自分の角柱で永久停止する** | 校正 項目 4（BLIND）を実施し、始業点検 項目 4 が OK | 段階 2 |
| **DEBT-3** | `lidar_timeout_ms` / `esp32_timeout_ms` = 2000 ms | `th_safety/config/safety_monitor.yaml:16-19` | `v_max` = 0.7 m/s で**タイムアウトまでに 1.4 m 走る** | A1 が満たされること（タイムアウトを導出値にし、破れるなら `v_max` を下げる） | 段階 2 |
| **DEBT-4** | `/cmd_vel` 途絶時も `esp32_bridge` がキープアライブで最後の非ゼロ指令を送り続ける | `esp32_bridge.py:237-238` | **上流が死んでも誰も止めない。**後段リミッタの前提が崩れる | 故障注入 §10-12（`cmd_vel_stale_ms` 以内にゼロ） | **段階 2 の最初** |
| **DEBT-5** | 自己位置喪失が `/safety/fault` に上がらない | `slam_control.py:210-250` | `map→odom` が凍結しても走り続ける | 故障注入 §10-13 | 段階 5 |
| DEBT-6 | `wifi_credentials.h` が実 SSID・実パスワードを含んでコミットされている。ポートも 8765 で `params.yaml`（8766）と不一致 | `esp32/src/wifi_credentials.h` | 秘匿情報の混入 | `.gitignore` 化＋履歴からの除去 | 段階 0（`task.md` 1 と同時） |
| DEBT-7 | `ws_link.h` のフレーム表が古い（`WHEEL_FEEDBACK` を 9 byte と書くが実装は 13 byte） | `esp32/src/ws_link.h` | プロトコル改訂時の事故源 | 表を実装に合わせる | プロトコルを触る最初のパケット |
| DEBT-8 | `esp32/tools/ws_test_server.py:86` が `unpack_wheel_feedback` の 3 要素返却に追従しておらず `ValueError` | 同左 | ベンチ試験ツールが動かない | 修正または削除 | 同上 |
| DEBT-9 | `main.cpp:196-202` の `[DBG]` printf が「原因切り分け後に削除」のまま残存 | 同左 | シリアル帯域の浪費 | 削除または開発モードのログ選択へ | 段階 3 |
| **DEBT-10** | **`th_safety/CMakeLists.txt` にテスト登録が 1 件も無い** | `th_safety/CMakeLists.txt`（`if(BUILD_TESTING)` ブロックに `ament_add_gtest` / `ament_add_pytest_test` が 0 件） | **`colcon test --packages-select th_safety` が現在この瞬間でも exit 0 で合格する。**安全の中核パッケージの完了条件が**すべて無条件合格**になる | `WP-SAFE-00` §7 の `ament_add_gtest` を登録し、**`colcon test-result --verbose` が「1 件以上実行された」と出る**こと | **段階 0（`WP-SAFE-00`）** |

---

## 3. 完全設計書へ戻すべきもの（申し送り）— **現在 0 件**

**詳細設計で埋めたが、正本を直さないと `Spec-modes.md` §3.1 の「表に無い遷移は起こらない」が破れる。**
`Spec-open.md` §7.1 手順 6 が定めている手続きに従う。

**A-1〜A-11 は 2026-08-16 に、A-12 / A-13 は 2026-08-17 に正本へ反映した。**
以下は反映内容の記録であり、**残っている作業ではない**
（§6 の保守規則 2 により、未処理の行はここに置かない）。

| # | 対象 | 反映した内容 |
| --- | --- | --- |
| A-1 | `Spec-modes.md` §3.1.1 / §3.1.2 | **ID 列を追加**（`SM-3.1.1-01`〜`-17` ／ `SM-3.1.2-001`〜`-098`）。「行番号で参照しない・欠番を再利用しない」も明記 |
| A-2 | `Spec-modes.md` §3.1.1 / §3.1.2 | 詳細設計にしか無かった遷移を全部戻した（下の内訳表） |
| A-3 | `Spec-modes.md` §4.1 | 物理押下が `CARRY` へ行かない例外を **2 つの表**にした（始業点検の非常停止確認中 ＋ **`INIT/CHECK`**・`CL-B-6`） |
| A-4 | `Spec-modes.md` §3・§3.0-①・§3.1.2 | `PANEL_NAV` / `SUMMON` / `HOME_NAV` の到着を「**通過**（`AT_PANEL` / `IDLE` へ）」表記に。**滞在するのは `LINE` だけ**と明記 |
| A-5 | `Spec-safety.md` §3.5 | 重大フォルトに **`LOCALIZATION_LOST`** を追加（理由つき） |
| A-6 | `Spec-safety.md` §1・§6.1 | 「必須通信系統の断 → `IDLE` へ強制遷移」を **`PAUSE`** に修正。**層 4 の説明（§1）も同じ誤りだったので併せて直した** |
| A-7 | `Spec-open.md` C-15 | `SUMMON` の `PERSON_TRACKER_LOST` を「`IDLE` へ強制遷移」から **`PAUSE`** へ。改訂の理由も併記 |
| A-8 | `Spec-modes.md` §7 ／ `Spec-webui.md` §2 ／ `mockup/index.html` | **ゾーンを 3 値**（`IN`/`OUT`/`NA`）に。モックアップの `SCREENS[].zone` を文字列化し、**画面ごとの `limit` も持たせた** |
| A-9 | `Spec-modes.md` §7.1 | 合成を **「速度上限の最小値」**に。途絶・使用中 0 台は停止。**画面由来とモード由来の最小値**であることも明記 |
| A-10 | `Spec-modes.md` §3.0-②・§3.1.1・§6 | **`IDLE` / `OPCHECK` / `CALIB` は `PAUSE` を持たない**を 3 か所に明記。`SM-3.1.1-03` の適用外であることも |
| A-11 | `Spec-checks.md` §3.1・§3.6・§3.8 ／ `Spec-open.md` | `O-d10` を解決済み（スケール係数方式）に。**`Spec-open.md` では `O-d10`→`F-40`・`O-d7`→`F-39` として §5.3 から §3 へ移し、§6 の参照も直した** |
| **A-13** | `Spec-open.md` §3.3・§5.3・§6.2 ／ `Spec-modes.md` §3.0・§3.1.2 ／ `Spec-onsite.md` §4.0.1 ／ `Spec-webui.md` §2・§3.11 ／ `Spec-transit.md` §0.1 | **地図の書き足し（`map_update`）を試験場内 4 モードへ広げた。**正本に **`F-41`** を新設（§3.3「詳細設計から戻ってきた決定」も同時に新設）し、遷移 4 行を **`SM-3.1.2-099`〜`-102`** として追加。**`SAVED` を作らない**ことを §3.0-③ に、3 段（A/B/C）の使い分けを `Spec-onsite.md` §4.0.1 に、S-21 の地図更新トグルと「保存」を `Spec-webui.md` §3.11 に置いた。**`O-d5` は解決していない**——決めたのは選べる形だけで、既定の選択は会場条件が要る（`Spec-open.md` §5.3 に明記） |
| **A-12** | `Spec-open.md` §7 | **指摘が誤りだった。**「§3 の表に `F-17` の行が無い」としたが、**取り消し線つきの行（`~~F-17~~` → 「§2 の C-14 へ移動」）は初めからあった**（`git log -S` で確認。3 回目レビューより前のコミット）。レビューが `**F-17**` の形だけを探して取りこぼしたもの。**再発防止として §7 に「移動した項目の行は消さず、取り消し線で移動先を残す」を明文化した** |

### 3.0 未処理 — **無し**

**A-12 の教訓**: 「正本に無い」という指摘は、**書式違いで grep が外れただけ**のことがある。
`~~ID~~`（取り消し線）・`**ID**`（強調）・素の `ID` は同じ ID である。
**ID の有無を数えるときは書式を剥がしてから数える**（`tools/check_id_coverage.py` はそうしている）。

### 3.1 A-2 の内訳（正本に追加した遷移）

| 追加先 | 正本 ID | 対応する詳細設計の行 |
| --- | --- | --- |
| §3.1.1（共通） | `-08` / `-12` / `-15` / `-16` / `-17` | `C-06r`（`CARRY` 中の UI 非常停止を拒否）／ `C-09f`（重大フォルト起因の `ESTOP` からの出口）／ `C-13`（モード選択）／ `C-15`（試験画面からの行き先選択）／ `C-14`（画面切替） |
| §3.1.2 | `-003` / `-019` / `-022` / `-034` / `-035` / `-037` / `-044` | `T-INIT-03` / `T-TEACH-06` / `T-REPLAY-03` / `T-LINE-05` / `T-LINE-06` / `T-LEASH-07` / `T-PREP-04` |
| §3.1.2 | `-060` / `-066` / `-072`〜`-075` / `-082`〜`-084` | `T-PNAV-07` / `T-ATP-05` / `T-SUM-08`〜`10`・`T-SUM-13` / `T-HNAV-05`〜`07`（**§5.3-A で「列挙から漏れている」とされた 6 件を含む**） |
| §3.1.2 | `-086` / `-090` / `-091` / `-092` / `-097` / `-098` | `T-OPC-04` / `T-OPC-06` / `T-OPC-08` / `T-OPC-07` / `T-CAL-08` / `T-CAL-09` |
| §3.1.2（**行の修正**） | `-010` / `-015` | `T-MANUAL-01` / `T-TEACH-03M` の**自己ループ**（`RUN` / `REC` 発）を遷移元に追加。§3.1.1 のリース表に**「リースの更新は遷移の採否と独立」**を追記 |
| §3.1.2（**行の修正**） | `-096` / `-097` | 旧「`CALIB` 任意 → 中断／非常停止／フォルト → `LIST`」の 1 行を、**「中断」**と**回復フォルト**に割った。**非常停止・重大フォルトは共通行で `ESTOP`/`CARRY` へ行く**（校正を安全チェーンの 2 つ目の例外にしない）ことを表の下に明記 |
| §3.1.1 除外表・§8 | — | ジョグ介入の不可モードに **`IDLE`** を追加（無いと `IDLE/PAUSE` という未定義状態が生まれる） |

> **`T-PREP-08` / `T-PREP-09` は既に正本にあった**（A-2 の列挙が古かった）。**追加していない。**

**A-1〜A-11 の作業中に見つけて併せて直したもの**（同じ「詳細設計にしかない」の類）:

| 対象 | 内容 |
| --- | --- |
| `Spec-modes.md` §6 `AT_PANEL` の復帰列 | 「復帰不要」→ **「確認」1 択 → `IDLE_P`**。`AT_PANEL` は `PAUSE` を持つので回復フォルトで落ちるが、選択肢を出さないと**出口の無い状態**になる（レビュー指摘 安全-14。`attributes.yaml` 側は既に `ack_only` / `IDLE_P` だった） |
| `Spec-safety.md` §1 の層 4 | 「動作系モードから `IDLE` へ落とす」→ **回復フォルトは `PAUSE`／重大フォルトは `ESTOP`**（A-6 と同じ誤りが §1 にも残っていた） |
| `Spec-webui.md` §2 の画面一覧 | ゾーン列を 3 値にし、**画面ごとの速度上限の列を新設**（A-8 / A-9 と同じ表を 3 か所で一致させるため） |

---

## 4. なお残る未確定（完全設計書から引き継ぐ）

**詳細設計でも決められない。**外部の情報か実測が要る。

| # | 項目 | これが決まると決まるもの |
| --- | --- | --- |
| `O-a1` | カメラ昇降の位置決め許容差 | 到着後の盤面正対を作るか（G2 の合否基準）／校正の許容範囲 |
| `O-a2` | 盤の数・間隔・盤前の空間・盤面高さ | ピン方式の価値・平面認識の成立性 |
| `O-a3` | 前日準備の現状所要時間 | 目標 G3 の検証 |
| `O-a4` | 通路幅・扉・段差・勾配・床の材質 | 許容逸脱・ライン誘導の成立性 |
| `O-a5` | 教示再生・ライン誘導の無人走行の許可 | 先導付きで測るか無人で測るか |
| `O-a6` | 昇降・撮影側との正式インターフェース | 「作業中ボタン」を置き換えられるか |
| `O-a7` | 手押し搬送の現状ベースライン | 目標 G4 の検証 |
| `O-c1` | **実測の制動加速度** | 最も多くを従属させる 1 つ |
| `O-c2` | 人が歩いている部屋の地図の写り込み | 人物マスク実装の要否（**最も安いテスト**） |
| `O-c3` | 2 点指示の実際の角度精度 | LiDAR 盤面正対が要るか |
| `O-c4` | 教示再生の終点位置誤差 | 1 経路の長さの上限 |
| `O-c5` | 校正の許容範囲・要再校正のしきい値 | F-13 の枠に入れる数値 |
| `O-c6` | WiFi 受信ギャップの改善後の値 | タイムアウトの下限（§7） |
| `O-c7` | バッテリーが 1 日持つか | 目標 G5 |
| `O-d1` | ライン誘導のマップ編集画面 | **設計しない**（原典が明示的に後回しを宣言） |
| `O-d3` | 盤前の低速並進 | 到着ずれのリカバリ手段 |
| `O-d4` | 起動チェック失敗時の再起動回数・待ち時間 | マニュアル側で決める |
| `O-d5` | **前日の地図を当日どう扱うかの既定**（そのまま／書き足す／作り直す） | **選べる形は用意した**（`F-41` ／ [onsite](DetailedDesign-onsite.md) §3.7）。**残るは「どれを既定にするか」だけ**で、会場条件が要る |
| `O-d6` | 開発モードの権限 | 実運用の体制が決まっていない |
| `O-d8` | `NewVISION.md` 案 2「SLAM 自律」を候補に戻すか | 未決 |
| `O-d9` | 手押しを選択可能な移動手段として並べるか | 未決 |

### 4.1 詳細設計で新たに生じた未確定

| # | 項目 | 状態 |
| --- | --- | --- |
| **N-1** | **教示再生の走行中ドリフト補正。**地図があるので scan matching で補正できるが、「走行開始時に決定した経路を逸脱しない」との関係を詰めていない | 保留（[transit](DetailedDesign-transit.md) §4.7） |
| ~~N-2~~ | **`intrusion_budget_m`（通信断で進んでよい距離）を誰が決めるか。**逆算元ではなく方針値なので、人が決める必要がある | **解決**（2026-08-24）。`intrusion_budget_m = 0.50 m`（人が決めた方針値。`registry.yaml` `status: given`）。`v_max(1.12) × esp32_timeout_ms(322ms)/1000 = 0.361 m ≤ 0.50 m` で余裕を持って成立する |
| **N-3** | **複数機体になったときの `wheel_radius_scale`。**PC 側に持つと機体を入れ替えたとき値が付いてきてしまう。当面は `robot_id` を持たせて警告する | 保留（[maintenance](DetailedDesign-maintenance.md) §4.4） |
| ~~N-4~~ | **UI 非常停止の UI 非依存な解除経路。**押下側にラッチする以上、WebUI が押下中に落ちると再起動以外に復帰できない | **解決**（2026-08-17）。[safety](DetailedDesign-safety.md) **§6.3.1**。`/safety/clear_estop_ui`（`std_srvs/Trigger`・機体の PC から `ros2 service call`）を 1 本置く。**物理側の解放で下ろす案は却下**——独立 2 系統の片方をもう片方に効かせると二重化でなくなる。**`N-4` は安全の欠陥ではなく可用性の欠陥**なので、対処も安全経路の外に置いた |
| **N-5** | **UI 側の受信ギャップ 4592ms の切り分け。**2026-08-21 の実測（`th_ws/data/meas04_after.csv`）でスリープ／タイマー抑制の混入が疑われる。要再測定（機体＋タブレット） | 未決。`ui_gap_p99_ms`（`registry.yaml`）が `placeholder` のまま |
| **N-6** | **PC 側タイムアウトが ESP32 のウォッチドッグより短い。**`esp32_timeout_ms = 322ms` に対し `esp32_watchdog_ms = 600ms`。ESP32 が自分で止まる前に PC 側が「切断」と判定する（A6 が指す実質的な問題）。`CLAUDE.md` によればウォッチドッグは WiFi ジッタによる誤発動を避けるため 2026-08-05 に 300→600ms へ緩めた経緯があり、この関係を詰め直す必要がある | 未決 |
| ~~N-7~~ | **A1 の `v_max` クランプ（P-5）が未実装。**設計書 §4 は「`v_max` をクランプし、警告を出す」としているが `export.py` の `timeout_from_bounds` は `if timeout > upper: pass` で何もしていない。現在の値では上限に触れないため実害は出ていないが、実装と設計書が食い違っている | **解決**（2026-08-26）。`export.py` に `_apply_v_max_clamp()` を追加し、`resolve_registry()` の一般ループより前に §3.3 手順2（下限を採り、上限を超えるなら `v_max` を1回だけ下げる。反復しない＝P-5）を実施するようにした。クランプ後の `v_max` を `values` に上書きしてから残りの行を解決するため、`obstacle_stop_distance_m` / `follow_stop_distance_m` 等の `v_max` に依存する派生値も新しい `v_max` で1度だけ再計算される。クランプが実際に発火した／A5 と衝突して見送られたことは、新しい仕組みを作らず既存の `run_assertions()` の `(errors, warnings)` 経路に載せる（`export.main()` は stderr、`params_audit` はロガー。`ParamsStatus.msg` には運ぶフィールドが無いため `/system/params_status` トピック自体への反映は見送った——別範囲の変更）。**設計書が触れていなかった相互作用（クランプが A5〈`v_reverse ≤ v_slow ≤ v_max`〉を破りうる）については選択肢(b)を採用した**: クランプ後の `v_max` が `v_slow` を下回るならクランプを適用せず `v_max` を自然値のまま残し、intrusion budget 超過を解消しないまま A1 の起動拒否に委ねる（「この構成では安全な速度が存在しない」と明示するエラーを別途 `clamp_errors` に積む）。**却下した選択肢**: (a) 速度体系全体（`v_slow`/`v_reverse`/`v_jog_panel`）を比例縮小する——registry に書いた値と実際に動く値が乖離し追跡不能になるため却下。(c) `v_max` を `v_slow` より下げず警告だけ出して起動させる——A1（危険な組合せを構成不能にする）の存在意義に反するため却下。現行 `registry.yaml` では `v_slow` 等の逆算元（`venue_clearance_m` 等）が `placeholder` のままなので、この相互作用は本番データではまだ発火しない（テストは合成値で発火させて検証した。`test_params_export.py` の `test_a1_clamp_applies_and_recomputes_v_max_dependents_once` / `test_a1_clamp_skipped_when_it_would_break_a5`）（[params](DetailedDesign-params.md) §3.3・§4） |
| ~~N-8~~ | **`blind_angle_ranges` の「死角なし」と「未校正」を配列の形だけで区別できない。**[safety](DetailedDesign-safety.md) §4.4 は「全ペアが幅ゼロ → `obstacle_limiter` は AUTO の走行開始を拒否する」と定めるが、`params_generation.sanitize_node_params()` は D3 の設計どおり空リストをキーごと落とすため、生成物では「死角が無いと確認済みで空配列」と「値が抜けてバグで消えた」を区別できない。安全ゲートの判定を配列の形から推論するのは危険 | **解決**（2026-08-26）。走行体のみの構成では LiDAR に死角が無いことを確認済みなので `blind_angle_ranges` は `status: measured` / `value: []` で確定した。判定は配列の形からではなく、新設した `blind_calibrated`（`class: b` / `given` / `value: true`）という明示フラグで行う設計にした。既定を `false`（拒否）側にせず現状 `true` にしているのは、死角が実際に無いことを確認済みであるため。上部構造の追加などで死角ができたら、`WP-CALIB-01` の手順で `blind_angle_ranges` を再測定するのと同時に `blind_calibrated` を明示的に見直すこと（`registry.yaml` / [names](DetailedDesign-names.md) §7.2） |
| **N-9** | **`ParamsStatus.msg` に「A1 のクランプが発火した」ことを運ぶフィールドが無い。**[params](DetailedDesign-params.md) §3.3 末尾は「2 で `v_max` を下げたことは警告として `/system/params_status` に載せる」と明示しているが、`ParamsStatus.msg`（`th_system_msgs`）のスキーマは `placeholder_count` / `placeholder_names` / `digest` のみで、警告文字列を運ぶ場所が無い。N-7（`_apply_v_max_clamp`）実装時点では代替として `params_audit` のロガー出力（`get_logger().warn/error`）で済ませており、`/system/params_status` を購読する WebUI 等からはクランプの発火が見えない。**設計書の要求を現状満たしていない。**解消には `th_system_msgs/msg/ParamsStatus.msg` へのフィールド追加（例: `clamp_messages` 相当）と `th_system_msgs` の再ビルドが要る（N-7 の実装範囲外の変更のため未着手） | 未決（[params](DetailedDesign-params.md) §3.3・§5） |
| **N-10** | **`scan_geometry` の規約③（`[-pi, pi)` へ正規化）は `angle_min = 0` のスキャナを表現できない。**[wp2](DetailedDesign-wp2.md) `WP-CALIB-01` §4.1 の規約③に従って `angle_to_index()` は角度を `[-pi, pi)` に正規化してから `angle_min` を引く。このため `angle_min = 0` / `angle_max = 2pi` を publish するスキャナでは、後半（180〜360 度）がすべて規約⑤の「範囲外」= `None` になる。**黙って半周分のセクタが「存在しない」ことになる**ため、`lidar_filter` はその範囲をマスクせず、`obstacle_limiter`（`WP-SAFE-03`）は自機構造物を実障害物として見て停止し続ける。**現状は踏まない**: 実機は RPLIDAR 系（`sllidar_ros2`・256000 baud）で `-pi..pi`、Gazebo も `gazebo_plugins.xacro:59-60` が `-3.14159..3.14159`。両環境とも `angle_min = -pi` のため到達しない（2026-08-26 に確認）。解消するなら「`[-pi, pi)` ではなく `[angle_min, angle_min + 2pi)` へ正規化する」形に規約③を改める（両規約とも比較を一意にする目的は同じで、後者は `angle_min` の取り方に依存しない）か、表現できない構成を検出して黙って `None` を返さず明示的に失敗させる。**規約③は設計書の明文なので、変えるなら wp2.md §4.1 を先に直すこと** | 未決（[wp2](DetailedDesign-wp2.md) `WP-CALIB-01` §4.1） |
| ~~N-11~~ | **`obstacle_limiter_core`（`WP-SAFE-03`）のコーンが scan の角度範囲を全くカバーしていない（=未観測）方向を、実装が「空き」（`nearest_m = +infinity`）として扱っていた。**[safety](DetailedDesign-safety.md) §3.4.2 は `/scan` が stale のときについて「古いスキャンで『空き』と判定しない」と明記するが、同じ理屈が「その方向は scan_geometry 規約⑤で `None`（＝そもそも観測範囲外）」にも及ぶことが実装時に抜けていた。**向きが逆になる**: `lidar_filter` の「範囲外はマスクしない」は障害物を多く見る方向＝安全側だが、`obstacle_limiter` で「範囲外を空き扱い」にすると障害物を少なく見る方向＝危険側になる（同じ「安全側＝判定できない区間には手を出さない」という言葉を、詰める先を取り違えると逆の結果になる）。**採用した扱い**: 未観測方向は「空き」にも「STOP」にもしない。L5（死角セクタに向かう成分がある方向は `v_reverse` キャップ）と同じ扱いにし、`nearest_obstacle_m` は空き確認済み（`+infinity`）と区別できる「不明」の明示値 `-1.0` のまま返す。STOP にしなかったのは、360 度未満の LiDAR では後退方向のコーンが構造的に常に観測範囲外になり得て、STOP 一択だと後退が構造的に不能になるため——設計書が「見えない方向」に一貫して用意している答えは停止ではなく `v_reverse`（L4・L5）である。ヒステリシス・`v_allow` の内部計算では、未観測を `nearest_m = +infinity`（中立値。新たな障害物情報が無いという意味）として扱う——安全マージンは上記の `v_reverse` キャップが別途担保するため、ここでさらに悲観側に倒す必要はない。**到達可能性**: 実機は RPLIDAR 系・Gazebo とも 360 度カバー（`angle_min = -pi`）なので現状は踏まない。N-10（`angle_min = 0` のスキャナ）と組み合わさると到達する | **解決**（2026-08-26）。`obstacle_limiter_core.hpp/.cpp` の `observe_cone()` が `ConeObservation{covered, nearest_m}` を返す形にし、`covered == false`（未観測）を `nearest_m` の値では表現しない（`nearest_in_cone()` の `std::optional<double>` 一本化をやめた）。property test（`test_obstacle_limiter_core.cpp`）に、コーンの一部が scan の角度範囲外になる部分カバレッジの scan 形状（`partial_geometry()`）を 30% の確率で混ぜ、未観測方向は `v_reverse` キャップ・`nearest_obstacle_m == -1.0` になることを検証する項目を追加した |
| **N-12** | **`is_path_blocked()`（Python・`mapless_follow_core.py`）を最近傍距離を返す形に拡張した `observe_cone()` は、C++ 版 `th_safety::observe_cone()`（`obstacle_limiter_core.cpp`）と bit-for-bit の等価性が成り立たない。**[safety](DetailedDesign-safety.md) §3.4.3 は「既存 `is_path_blocked()` は bool しか返さないので、最近傍距離を返す形に拡張して移植する。拡張後、既存 Python 実装との等価性テストを書く」と指示するが、実測すると等価にならない。**原因は集合の作り方の違い**: Python 版は各ビームの中心角度が `direction±half_width` の範囲に入っているかを個別判定する（角度ベース）。C++ 版は `scan_geometry.sector_indices()` でコーン境界の角度を添字に変換し（`floor` による丸め）、その閉区間 `[i0, i1]` を機械的に走査する（添字ベース）。`floor` は「その角度を含むビン」を返すため、下端側（`i0`）はコーンの外側（`direction - half_width` よりさらに外）のビームを拾うことがある。この結果、**C++ が選ぶビーム集合は常に Python が選ぶビーム集合の上位集合になる**（`half_width` が `angle_increment` 程度以下の狭いコーンで頻発する）。**実測**（`test_obstacle_cone_equivalence.cpp` を自作ドライバでホスト上から実行。全周スキャン・n=72/360/1081・境界ケース+ランダムfuzz 計270ベクタ）: 一致 171 件・C++ がより近い障害物を検出（保守側）99 件・Python の方が近い障害物を検出した違反 **0 件**。**採用した扱い**: ビット一致ではなく「C++ の最近傍距離は常に Python の最近傍距離以下」という**包含性**を検証するテストに置き換えた（`covered=true` かつ `cpp.nearest_m <= python.nearest_m`（Python の `None` は `+infinity` として扱う）が全ベクタで成り立つことに加え、実際に分岐するケース（`cpp_more_conservative_count > 0`）を含んでいることも表明する）。**C++ 側は変更しない**: はみ出しビームを含める向きは常に「障害物を多く見る」side（保守的）であり、`obstacle_limiter` の安全側の意味と一致する（後段のリミッタが前段の挙動判定より見落とさないことの保証になる）。**Python 側を `scan_geometry` へ載せ替えて添字ベースに統一する一本化は、このパケットでは行わない**: `is_path_blocked()` / `observe_cone()` は [reuse](DetailedDesign-reuse.md):78 のとおり段階3で `th_transit/follow_core.py` へ移設される予定で、`th_planning` から `th_perception` の `scan_geometry` を import するには package.xml にパッケージ跨りの依存を新設する必要があり、どのみち段階3の移設時に作り直しになる。一本化は `WP-TRANSIT-01` の作業とする | **解決**（2026-08-26）。`mapless_follow_core.py` に `observe_cone()` を追加し（`is_path_blocked()` 自体は変更なし）、`test_obstacle_cone_equivalence.cpp`（`th_safety`）と生成器 `gen_obstacle_cone_vectors.py`（`th_testing/tools`）で包含性を検証する形にした。統一（`WP-TRANSIT-01` へ先送り）までは、両実装の食い違いは安全側にしか倒れないことを上記の実測で確認済み |
| ~~N-13~~ | **画面由来の速度上限が `/system/state` に載らず、`obstacle_limiter` が受け取れない。**[safety](DetailedDesign-safety.md) §3.3.1 は `applied_limit_mps = min(画面由来の上限, モード由来の上限, `v_reverse`（後退時）)` と定め、§3.1 が `obstacle_limiter` に許す購読先は `/cmd_vel_muxed` / `/scan` / `/system/state` / `/cmd_vel_manual` / `/safety/estop` / `/safety/fault_lock` の 6 つだけで `/ui/active_screen` を含まない。ところが **`SystemState.msg` には速度上限のフィールドが無い**（`zone` と `auto_brake` はある）。`th_state/zones.py` の `derive_limits()` は `Limits(zone, speed_limit, auto_brake)` を計算しているが、`state_manager.py:239` は `.zone` しか取り出しておらず、**`speed_limit` は計算されて捨てられている**。このためリミッタは画面由来の上限を知る経路を持たず、§3.3.1 を満たせない。**解決**（2026-08-26）。`SystemState.msg` の**末尾**に `string speed_limit`（registry のパラメータ名 or `"stop"`。`zones.py` の `Limits.speed_limit` の表現そのまま。数値の実体は運ばない）を追加し、`state_manager.py` から publish するようにした。**`WP-MSG-01` の `M3`（既存 msg への変更を `severity` 1 件に限定）の例外である**——`WP-SAFE-01` が `FaultStatus.msg` に `Header` を追加した前例と同種で、消費者（`obstacle_limiter`。`WP-SAFE-03`）が実在し、追加しないと当該 WP が成立しないため。`state_manager.py` 側は `derive_limits()` を1回だけ呼んで `.zone` と `.speed_limit` を同じ結果から取り出す形にした（zone と speed_limit が別時刻の結果になる事故を避けるため。呼び出し箇所（`_build_context()` と `_publish_state()`）は従来どおり別々に1回ずつ呼ぶ）。§4.1「合成は各画面の最小値。ゾーンの強弱では合成しない」（R3-39/R2-26 の再発防止）を直接検査するテストを `test_zones.py`（新設。`th_testing/CMakeLists.txt` に登録）に追加した。**設計書に無く自分で判断した点**: `DetailedDesign-names.md` §5.1 の `SystemState.msg` フィールド表にも `speed_limit` を追記した（`test_msg_definitions.py::test_fields_match_names_md` が names.md の表と実装の突き合わせを行うため、追記しないと ROS2 側のテストが壊れる）。**未着手のまま残した点**: `SystemState.auto_brake` の出どころ（`state_manager.py` の `_flags["auto_brake"]`。`/system/set_flag` で外部から設定できるフラグで、既定 `True`）は `derive_limits()` が返す `auto_brake`（`zone != "OUT"` から導出）とは**無関係の別経路**。両者は食い違いうるが、本パケットの範囲外として挙動は変えていない（[open](DetailedDesign-open.md) 本行の申し送りとして記録） | [safety](DetailedDesign-safety.md) §3.1・§3.3.1 |
| **N-14** | **`auto_brake` の定義が 3 箇所で食い違っている。**[names](DetailedDesign-names.md) §4.1 の擬似コードは `auto_brake=(zone == "IN") or default`、`th_state/zones.py:105` は `auto_brake = zone != "OUT"`（IN と NA が True）、`state_manager.py:494` は `msg.auto_brake = self._flags["auto_brake"]` で**ゾーンを一切見ない**（`/system/set_flag` で設定できる単一フラグ。既定 True）。つまり `derive_limits()` が算出したゾーン依存の既定は publish 経路で捨てられている。[safety](DetailedDesign-safety.md) §3.3 は「MANUAL / OUT → 警告のみ（**既定 OFF**）」「MANUAL / IN → 停止（**既定 ON**。試験員が OFF にできる）」と**ゾーンごとに既定が変わる**ことを求めているが、単一フラグではこれを表現できない。**帰結**: OUT ゾーンで試験員が何もしなければ `auto_brake=True` のままなので、`obstacle_limiter` は MANUAL 操作中も障害物で停止する（設計は警告のみ）。安全側の誤りだが、場外での手動操作の自由度という設計意図は失われる。**未着手の理由**: (1) `th_state` のフラグ意味論の問題で `WP-SAFE-03`（リミッタ）の射程外。(2) 「試験員が明示的に設定した」と「一度も触っていない」を区別する仕組みが設計書に無く、ゾーン変化のたびに既定へ戻すのか三値にするのか決めが要る。**解消するときは 3 箇所を同時に揃えること**（片方だけ直すと食い違いが別の形で残る） | 未決（[names](DetailedDesign-names.md) §4.1・[safety](DetailedDesign-safety.md) §3.3） |
| ~~N-15~~ | **`attributes.yaml` の `speed_limit`（モード由来の速度上限）を読んでいる `th_state` の Python コードが 1 つも無く、`SystemState.speed_limit`（`N-13`）には画面由来しか載っていなかった。**[safety](DetailedDesign-safety.md) §3.3.1 は `applied_limit_mps = min(画面由来の上限, モード由来の上限, v_reverse（後退時）)` と定め、上限の出どころが画面（[names](DetailedDesign-names.md) §4）とモード（`attributes.yaml`）の 2 系統あることを明記しているが、`th_state/th_state/*.py` を `grep -rn "speed_limit"` すると `zones.py` しか出ない。`StateCore` は `attributes` を受け取っているのに `speed_limit` フィールドは一度も参照していない（`state_core.attributes(mode)` という取得口はあるが、呼ぶ側が無かった）ことを実装を読んで確認した。**解決**（2026-08-26）。`th_state/zones.py` に `mode_speed_limit(mode_attrs, jog_active)`（`attributes.yaml` の該当モード行から、モード由来の上限を決める）と `combine_speed_limits(screen_limit, mode_limit)`（`_LIMIT_STRICTNESS` の順序で厳しい方を返す）を追加し、`state_manager.py::_publish_state()` が `derive_limits()` の結果（画面由来。1 回だけ呼ぶ既存の `_current_limits()` をそのまま再利用）と `self.core.attributes(self.mode)` から求めたモード由来を合成して `SystemState.speed_limit` に載せるようにした。`derive_limits()` を追加で呼んではいない（`_current_limits()` の 1 回呼び出し規約は`N-13`のまま）。**責務の置き場所を §3.3.1 の字面から変えた**: 設計書は「リミッタが `min` を取る」と書いているが、この変更で `state_manager` が画面とモードの合成まで済ませ、`obstacle_limiter`（別パケット `WP-SAFE-03` 本体）が受け取るのは既に 1 本化された名前になる。リミッタ側は受け取った上限に `v_reverse`（後退時）と障害物クランプを重ねるだけ。**理由**: (1) モード由来の上限の所有者は `attributes.yaml`（`th_state` の config）であり、これを C++ 側にも読ませると設定の所有者が二重になる。(2) `attributes.yaml` 冒頭コメント（※※※）のとおり `AT_PANEL` の `speed_limit` は停止だが `jog_active` の間だけ `v_jog_panel` という条件があり、`jog_active` を知っているのは `state_manager` だけ。(3) `state_manager` は画面・モード・`jog_active` の 3 つを唯一同時に持つノード。結果は同じ（合成の規則自体は変えていない）だが置き場所が変わるため、ここに明記する。**この結果 `SystemState.speed_limit` の意味が「画面由来」（`N-13`）から「画面とモードの合成結果」に変わった**——[names](DetailedDesign-names.md) §5.1 の `SystemState.msg` の記述を合わせて更新した。`AT_PANEL` × `jog_active` の判定はモード名の文字列比較をせず（`state_manager.py` の N-1 規約と同種の理由）、`speed_limit == "stop"` かつ `jog != "denied"`（=このモードは jog できる）かつ `jog_active` の 3 条件で行う——現状これを満たすのは `AT_PANEL` だけだが、将来 stop かつ jog 許可のモードが増えても同じ規則で動く。**設計書に無く自分で判断した点**: (a) `v_jog_panel` を `_LIMIT_STRICTNESS` に追加する必要があったが元々無かった。`registry.yaml` の `A5`（`v_jog_panel ≤ v_reverse ≤ v_slow ≤ v_max`）は数値の話であって名前の厳しさ順の根拠には直接使えず（本パケットの指示どおり数値は参照していない）、`v_check` / `v_calib` との相対順も `A5` の対象外で未測定。**`stop` の次に厳しい位置**に置いた——`v_jog_panel` は「本来 stop のところを jog_active の間だけ明示的に許す」例外専用の値で、配電盤という障害物至近での作業だから他のどの極低速上限よりも厳しく扱うのが安全側という判断（数値ではなく用途からの判断）。(b) `AT_PANEL` の画面は [names](DetailedDesign-names.md) §3 の表で `S-21`（`IN`/`v_slow`）のみなので、通常運用では画面側は必ず `v_slow` 以下にしかならず `combine_speed_limits` は `v_jog_panel` を選び続ける（`v_slow` より厳しい）ことをテストで確認した。**指示への確認**: 依頼文の「`attributes.yaml` の `speed_limit` を読んでいる箇所が 1 つも無い」という主張は、上記の `grep` と `StateCore` の実装読解で確認した——事実誤認ではなかった。**未着手のまま残した点**: `attributes.yaml` を全行確認したが、`speed_limit` に条件付き特例があるのは `AT_PANEL` だけで、他モードには無い。**追記（レビュー指摘・同日）**: 上記「`v_jog_panel` を `stop` の次に置いた」判断は**誤りだった**。`registry.yaml` を見ると `v_jog_panel` は `panel_clearance_m`（`brake_accel_mps2` / `brake_delay_s` とともに）から `speed_from_braking_distance` で逆算される値で、現実的な `panel_clearance_m`（例: 0.3m 前後）を入れると `v_check`(0.05m/s) より一桁大きくなる見込み——`_LIMIT_STRICTNESS`（`LIMIT_STRICTNESS` に改名）は**数値の代理**であって用途の重大さの順ではなく、`stop` の次（`v_check`/`v_calib` より厳しい側）に置くと `v_check` が画面由来・`v_jog_panel` がモード由来という組合せで `combine_speed_limits()` が実際より高い（緩い）上限を選んでしまう。**訂正**: `v_jog_panel` を `v_calib`(0.15) と `v_slow` の間に移動した（`v_jog_panel ≤ v_reverse ≤ v_slow` という `A5` の保証から `v_slow` より厳しい側にあることは確定しているが、`v_check`/`v_calib` との相対順は未測定のままの推定であることに変わりはない）。**恒久対策**: この種の「順序の前提が実測値と食い違うと安全機構が静かに壊れる」事故を再発させないため、`th_params/th_params/assertions.py` に **`A13`**（新設）を追加した——`LIMIT_STRICTNESS` の並びが `registry.yaml` の解決済みの数値の昇順と一致することを起動時に検査し、違反なら起動を拒否する（`stop` は 0 として扱う。`placeholder`＝未測定の名前は比較から除外）。`export.py::run_assertions()` に `th_state.zones.LIMIT_STRICTNESS` を渡して実行するため、`th_params/package.xml` に `<depend>th_state</depend>` を追加した（th_params が th_state を読む向きの新しいパッケージ間依存。循環にはならない——`th_state` は `th_params` を import しない）。現行 `registry.yaml` では `v_slow`/`v_reverse`/`v_jog_panel` の逆算元（`venue_clearance_m`/`blind_clearance_m`/`panel_clearance_m`）がいずれも `placeholder`（未測定）のため `A13` は今は発火しないが、`test_params_export.py` に `v_jog_panel` を意図的に `v_calib` より小さい値へ強制した合成テスト（`test_a13_fires_when_v_jog_panel_measured_value_breaks_the_order` / `test_main_rejects_startup_when_a13_violated`）を追加し、実際に起動拒否（`export.main()` の終了コード 1）になることを確認した。`DetailedDesign-params.md` §4 のアサーション表に `A13` の行を追加した。**追記2（レビュー指摘・同日）**: `export.py` が `th_state.zones.LIMIT_STRICTNESS` をモジュール先頭で import していた設計も**指摘を受けて撤回した**。理由は2つ——(1) 依存の向きが逆: `th_params` は `registry.yaml` から全ノード分の設定を生成する**上流**（`REGISTRY_NODES` に `state_manager` を含む）であり、下流の `th_state` を import すると層が反転する。(2) `python3 -m th_params.export` は launch のたびにサブプロセスとして起動され `A8` 等で**起動そのものを止める**役割を持つ経路であり、ここに新しいパッケージへのモジュールレベル import を足すと `th_state` が import できないだけでパラメータ生成が丸ごと落ちて起動不能になる（実際 `test_params_launch.py` の5件がこの import 追加で `ModuleNotFoundError` になっており、この経路が壊れやすいことは実証済みだった）。**訂正**: `th_state` への import と `th_params/package.xml` の `<depend>th_state</depend>` を削除し、代わりに `registry.yaml` に `speed_limit_strictness_rank`（`v_check`=1 〜 `v_max`=6 の連番。`stop` は rank 0 として暗黙に先頭固定）というフィールドを新設した。`export.py::speed_limit_strictness_order(rows)` がこの rank から並びを組み立て、`a13_speed_limit_strictness_order()` に渡す（`A13` の関数シグネチャ自体は変更していない——元々 `order` を引数で受け取る形だったため）。これで `registry.yaml` と `th_state/zones.py::LIMIT_STRICTNESS` に「speed_limit 名の厳しさ順」の宣言が2箇所に重複するが、`th_testing/test/test_state_registry_consistency.py`（新設。`th_state`・`th_params` の両方を import してよい唯一のテストコード）が両者の一致を機械的に照合するため、`N-14`（`auto_brake` の3箇所食い違い）のような静かな乖離にはしない。**実測で確認した**: `env -i` で `PATH`/`PYTHONPATH` を絞り、`th_state` が一切 import できない隔離環境（`PYTHONPATH` に `th_params` のソースディレクトリのみ）で `python3 -m th_params.export` を `--sim` あり・なしの両方（`--sim` なしは `A1`〜`A13` の全アサーションを含む経路）で実行し、`exit code 0` で生成物を書き切ることを確認した | [safety](DetailedDesign-safety.md) §3.3.1・[names](DetailedDesign-names.md) §4.1・[params](DetailedDesign-params.md) §4 |
| **N-16** | **`obstacle_limiter` の判定コーンが `laser_link` の回転を補正しない。**`obstacle_limiter_core` はコーン方向を前進 0 / 後退 π として `laser_link` 基準で固定しており、起動時に取得した `base_link ← laser_link` の変換の**回転成分を受け取る入力を持たない**。現行 URDF（`th_description/urdf/th_robot.urdf.xacro` の `laser_joint`）は `rpy="0 0 0"`（並進のみ）なので**現状は実害なし**。ノードは yaw を検出したら起動時に WARN を出す。**懸念**: LiDAR を回転させて搭載した場合、障害物判定コーンが実際の進行方向とずれる。WARN は見落とされうるので、[wp2](DetailedDesign-wp2.md) `WP-SAFE-03` §3.4 が TF 取得失敗に対して定める「**起動を失敗させる**（素通しで動かさない）」と同じ扱い（yaw≠0 なら起動拒否）にするか、コア側にコーン方向のオフセット入力を足すかを決める必要がある。**回転搭載にする前に必ず決着させること** | 未決（[wp2](DetailedDesign-wp2.md) `WP-SAFE-03` §3.4） |
| ~~N-17~~ | **Nav2 の `behavior_server` が `/cmd_vel` へ直接 publish しており、安全チェーンを丸ごと迂回している。**`gazebo.launch.py` を実起動して `ros2 topic info /cmd_vel --verbose` を採ったところ（デーモンを止めて名前を解決）、publisher 5 件のうち 4 件が **`behavior_server`** だった。`nav2_params.yaml` / `nav2_params_sim.yaml` の**どちらにも `behavior_server` の項目が無く**、Nav2 既定のまま `cmd_vel` に出している。**帰結**: spin / backup / drive_on_heading などの復帰挙動が `twist_mux` を通らないため、**`/safety/estop`（priority 255）と `/safety/fault_lock`（254）のロックが効かない。**`WP-SAFE-03` で `obstacle_limiter` を最終段に入れた後は、障害物リミッタも迂回する。これは今回の配線で作った穴ではなく**以前から存在していた**が、`CLAUDE.md` の不変ルール「`/cmd_vel` に直接 publish するノードを追加してはいけない」に反する。**対処**: `behavior_server` の `cmd_vel` を `/cmd_vel_behavior`（`twist_mux` の priority 20 入力）へremap する。`WP-SAFE-03` の §1 の 6 点リストにこの経路の記述が無いため、設計書側（[wp2](DetailedDesign-wp2.md) §1・[safety](DetailedDesign-safety.md) §2 の経路図）も同時に直すこと | **解決**（2026-08-27）。**(a) `nav2_params*.yaml` に項目を作る案は採らなかった**——`nav2_behaviors`（humble）のソースを確認したところ、`behavior_server` の cmd_vel publisher はトピック名がハードコード（`timed_behavior.hpp` の `configure()` に `create_publisher<Twist>("cmd_vel", 1)` と直書き）で、ROS パラメータでは変えられないことを確認した（(a) は原理的に効かない）。**(b) launch 側の remap を採用**。ただし素朴に `launch_ros.actions.SetRemap` で `IncludeLaunchDescription(navigation_launch.py)` を外から包む方法は使えなかった——`SetRemap` は「スコープ内の全ノードの同名remapを上書きする」仕様（`launch_ros` 本体の docstring に明記）で、同じスコープに同居する `controller_server` の正しい remap（`cmd_vel`→`cmd_vel_nav`）まで巻き込んで壊してしまう。`IncludeLaunchDescription` にも外から特定ノードだけへ remap を注入する口が無いため、消去法で `nav2_bringup` の `navigation_launch.py`（humble）をローカルにフォークした（`th_bringup/launch/navigation_launch.py`）。`behavior_server` の `remappings` に `('cmd_vel', 'cmd_vel_behavior')` を追加し、`gazebo.launch.py`（sim/実機 両方）・`bringup.launch.py`（実機専用 launch。同じ穴があることを確認して同時に直した）の include 元をこのフォークに差し替えた。**追加で見つけた点**: フォーク元のソースを読んだところ `velocity_smoother` も `remappings=[('cmd_vel','cmd_vel_nav'), ('cmd_vel_smoothed','cmd_vel')]` を持ち、`controller_server` の出力を横取りして平滑化した上で最終段と同じ `/cmd_vel` へ直接 publish する（`use_composition`（既定 false）に関わらず常に生成される）。これは `VISION.md` にも `CLAUDE.md` の速度指令の流れの図にも登場しない、このプロジェクトが意図していないノードで、実起動観測の「publisher 5件」のうち `behavior_server` 以外の名前不明1件はこれだと推定し（静的解析のみ。Docker 実機確認はしていない）、同じフォークで削除した。**設計書側の更新**: [wp2](DetailedDesign-wp2.md) §1 に行7として追記、[safety](DetailedDesign-safety.md) §2 の経路図に `Nav2 behavior_server` の入力を追加した。**次回 Docker 起動での確認事項（未実施）**: `ros2 topic info /cmd_vel --verbose` の publisher 数が 5→1（`obstacle_limiter` のみ）に減ること |
| ~~N-18~~ | **`teleop.launch.py` の `direct:=true` が `/cmd_vel` へ直接 publish している。**`th_bringup/launch/teleop.launch.py:61` の `crawler_teleop_direct` は `publish_topic: /cmd_vel`。`twist_mux` を通さないため **`/safety/estop`（255）も `/safety/fault_lock`（254）も効かず、`WP-SAFE-03` 後は `obstacle_limiter` も迂回する。非常停止を押しても止まらない。** | **解決**（2026-08-27。ユーザー判断）。**(b) 残す**を採用。地図作成は WebUI の手動操作で行うのが通常手順で、`direct:=true` は**それが使えないときのバックアップ手段**として意図的に残す。(a)（`/cmd_vel_behavior` 経由にする）を採らなかったのは、バックアップ経路が`obstacle_limiter` の障害物停止に引っかかると本来の目的（他が使えないときに動かす）を果たせなくなるため。**代わりに危険性を現物へ明記した**——`teleop.launch.py` の冒頭 docstring に「安全チェーンを丸ごと迂回する／非常停止を押しても止まらない／止める手段は物理非常停止と ESP32 ウォッチドッグ（600ms）だけ／使うときは人が張り付くこと」を記載し、該当 `Node(...)` 直前にも注記を置いた。N-17（`behavior_server`）とは扱いが異なる——あちらは**自律走行中に意図せず発動する**経路なので remap して塞ぐ | 
| **N-19** | **`WP-SAFE-03` §10 の完了チェックが原理的に通らない。**[wp2](DetailedDesign-wp2.md):860 は完了条件に `! grep -rn "cmd_vel_retreat" src/th_safety/ src/th_bringup/ src/th_testing/` を挙げているが、`src/th_testing/test/test_simulation_scenarios.py` が `/cmd_vel_retreat` を 9 箇所で参照している。同ファイルは新設計で廃止済みの as-built 挙動を検証しており既定でスキップ（`TH_SKIP_SIM=1`）、`WP-TRANSIT-01` で `follow_planner` ごと削除される見込みのため `WP-SAFE-03` の範囲外。**`WP-TRANSIT-01` が終わるまでこの grep は満たせない。**完了判定では本項を参照し、`th_safety/` と `th_bringup/` が 0 件であることをもって代替とすること（2026-08-27 時点で両ディレクトリは 0 件を確認済み） | 未決（[wp2](DetailedDesign-wp2.md) `WP-SAFE-03` §10） |
| **N-20** | **`nav2_bringup/launch/navigation_launch.py` を `th_bringup` にローカルフォークした（`N-17` の対処。恒久的な保守負債として記録）。**フォーク元: `https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py`（**取得日 2026-08-27 時点の `humble` ブランチ HEAD**）。`th_ws/Dockerfile` は `ros-humble-nav2-bringup` をバージョン指定なしで apt install しているため、実際にコンテナへ入る apt パッケージの正確なビルド日時・コミットハッシュは本リポジトリからは特定できない——**Nav2 を更新するときは上記 URL の履歴と `th_bringup/launch/navigation_launch.py` の diff を手動で取り直すこと**（自動追従の仕組みは無い）。フォークした理由は2点: (a) `behavior_server` の `cmd_vel` publisher がソース側でトピック名をハードコードしており（`nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp` の `configure()`）、YAML パラメータでは remap できない。(b) `launch_ros.actions.SetRemap` でスコープごと remap を被せる方法は、同じスコープに同居する `controller_server` の正しい remap（`cmd_vel`→`cmd_vel_nav`）まで巻き込んで上書きしてしまうため使えない（`launch_ros` 本体の docstring が「スコープ内の全ノードの同名remapを上書きする」と明記）。差分は `behavior_server` への remap 追加と `velocity_smoother` の削除の2点のみ（[safety](DetailedDesign-safety.md) §2・`N-17` 参照） | 記録のみ（保守負債として容認。`th_bringup/launch/navigation_launch.py` 冒頭コメント） |

---

## 5. レビュー指摘管理表

**1 回目（骨格レビュー・2026-08-16）。**対象は `-names` / `-state` / `-safety` / `-params` の 4 ファイル。
観点を分けた 2 体で並列にレビューした（R2＝安全・整合、R3＝実装可能性）。
**指摘 87 件。**

### 5.1 対応済み（致命・着手不能）

| 出所 | 指摘 | 対応 |
| --- | --- | --- |
| R2-1 | **重大フォルトが `/safety/fault_lock` に載らず、止めるのが層 4 だけだった** | `fault_lock` に `severity == CRITICAL` を OR（[safety](DetailedDesign-safety.md) §5.2.1） |
| R2-2 / R3-16 | **`T-CAL-08` が `fault.critical` と `hw.estop.press` を握り潰し、校正中だけ層 1・2 が効かなかった** | `fault.recoverable` のみに限定。破棄は effect 側で実現 |
| R2-3 | **`d_floor` と `d_behavior` が同じ式から出ており、リミッタが追従対象の脚で発火する** | `d_floor` を機体外形＋固定クリアランスに変更。A2b を追加 |
| R2-4 | **`source_class` を鮮度だけで決めると、手動パケット 1 発で自律走行が 1.0 s 緩む** | `jog_active` との AND ＋ 不変条件 L7 |
| R2-5 | **`/cmd_vel_manual` は UI が直接 publish するので、`WAIT_CLEAR` のジョグ禁止が UI 任せだった** | **`jog_gate` を新設**（[safety](DetailedDesign-safety.md) §3.4.1） |
| R2-6 / R2-7 | **`CARRY` 中の UI 非常停止を受理し、`prev_*` を上書きして復帰先を壊していた** | `C-06r` を新設。`latch_prev` を条件付きに |
| R2-8 | **`C-08` が `C-12` より先に並び、`hw_released` ガードが永久に評価されなかった** | `C-08` に `can_finish` ガード |
| R2-9 / R3-17 | **`fault.critical` で入った `ESTOP` から出る行が無くトラップ状態だった** | `C-09f` を新設 |
| R2-10 / R3-14 | `C-05` の `to_state` が `PAUSE` 固定で §7.1 と矛盾 | `$resume_state` に |
| R2-11 / R3-12 | **`OPCHECK` / `CALIB` / `IDLE` に `PAUSE` が無いのに `C-03` が落とそうとしていた** | `fault_stops_mode` の除外に追加。§4.1.1 末尾に明記 |
| R2-12 | ジョグ除外に `IDLE` が無く `IDLE/PAUSE` が生まれる | 除外に追加 |
| R2-13 | **試験画面から `PANEL_NAV`/`SUMMON`/`HOME_NAV` に入る行が無かった** | `C-15` を新設 |
| R2-14 / R3-13 | `ARRIVED` が到達不能 | 3 モードの状態集合から外す。A-4 で正本へ申し送り |
| R2-21 / R3-27 | `safety_monitor` の監視対象が廃止予定の `/person/status` のまま | `/person/targets` に。**同一パケットで差し替える**と明記 |
| R2-29 / R3-10 | `override_common` の判定が包含関係で定義されておらず解釈できない | **手続き規則**に書き直し（§3.1） |
| R3-3 / R3-4 | `sys.*` という入口と 4 事象が名前辞書に無い | 第 4 の入口として追加 |
| R3-5 | **ガード 24 件の定義が無い** | ガード定義表を新設（§4.1.1）。`ok` → `check_result_ok` に改名 |
| R3-6 / R3-7 | `Context` にガードが必要とするフィールドが無い。`prev_*` も無い | `Context` を全面的に拡張 |
| R3-8 | `$resume_run` / `$arg.kind` がトークン一覧に無い | §3.2 に追加。解決順序も明記 |
| R3-9 | `ui.goto` の `kind` → モード写像が無い（`PANEL` というモードは存在しない） | §3.5 に写像表 |
| R3-11 | **effect の一覧・引数・実行主体が無い** | effect 一覧表を新設（§3.3）。`{name, args}` に構造化 |
| R3-20 / R3-21 / R3-22 | リミッタのコーン幅・距離→速度の式・角速度の扱いが無い | §3.4.3 と L3/L5 で確定 |
| R3-23 | **`/scan` の stale 時の挙動が無い**（安全床の穴） | `scan_stale_ms` を新設（§3.4.2） |
| R3-25 | twist_mux の remap は `twist_mux.yaml` ではなく **launch 2 本**にある | 変更対象を明記。同一パケットで行うことも |
| R3-26 | `person_predictor` が `/cmd_vel_retreat` の publisher として孤児になる | **廃止**と明記（names §1.1） |
| R3-28 | **パラメータ生成が launch 評価より後になる鶏卵** | 生成（`OpaqueFunction`）と監査（ノード）を分離 |
| R3-29 | 派生値がゾーン・モードで変わるのに registry がスカラーしか持てない | `value_by` を追加 |
| R3-31 | タイムアウトの採り方と再計算の順序が無い | **反復しない**手続きを明記 |
| R3-33 | `manual_joy_timeout` が registry と `twist_mux.yaml` の 2 か所に残る | **`twist_mux.yaml` も生成対象**に |
| R3-34 | `cmd_vel_stale_ms` が 2 用途に 1 名前 | `muxed_stale_ms` と分割 |
| R3-35 | `spec_ref` が行番号で、正本を 1 行編集するとずれる | ID 参照に変更。A-1 で正本へ申し送り |
| R3-39 / R2-26 | ゾーン合成が「強弱」で、`NA` を `IN` に倒すと**上限が上がる** | **速度上限の最小値**で合成（names §4.1） |
| R3-47 | 廃止する既存サービスの一覧が無い | names §5.2 に廃止・改名表 |
| R2-18 / R2-19 / R2-20 / R2-22 / R3-32 / R3-41〜45 | 名前辞書の欠落（イベント・msg・パラメータ・ノード・フレーム・QoS・ウィンドウ ID・launch 引数） | すべて追加 |
| R2-23 / R2-24 | **(c) の抜け道**（`blocking` が任意）と、例文が (c) に暫定値を与えていた | 組合せ規則 S1〜S5 を新設。例文から数値を削除 |
| R2-25 | safety 本文に裸の数値（500 ms / 2500 ms / 2000 ms）があった | registry 参照に置換（§5.4・§7・§0） |
| R2-34 | `generated/` がディレクトリツリーに無い | 追加 |
| R2-36 | `d_floor` を割っていても角速度が無制限 | L3 に条件を追加 |
| R3-36 | 検査 4（数値リテラルの grep）の合否が主観 | **registry の現在値との一致**に定義し直し、除外リストをファイル化 |

### 5.2 1 回目の指摘の追跡（**全 18 件が対応済み**）

| # | 出所 | 指摘 | 状態 |
| --- | --- | --- | --- |
| 1 | R3-1 / R3-2 / R3-50 | `DetailedDesign.md` が 0 バイトで作業パケットの一覧・段階・依存グラフが無い | **対応済み**（本体・`-packets.md`・`-reuse.md`・`-open.md` を作成。順序制約 5 件を `-packets.md` §1 に明記） |
| 2 | R2-16 | `T-SUM-13`（`SUMMON` の `BLOCKED` からの `ui.reroute`）が無く W-5 が死にボタン | **対応済み**（行を追加） |
| 3 | R3-15 | `MANUAL/RUN` で `ui.jog.hold` を受ける行が無く 5 Hz のリース更新が全部拒否される | **対応済み**（`RUN` 発の自己ループ ＋「リース更新は遷移の採否と独立」の規則） |
| 4 | R3-18 | `T-ATP-06` が「受理された拒否」になっている | **対応済み**（`goto_allowed` ガードの否定で表現。行を削除） |
| 5 | R3-19 | `T-TEACH-06` / `T-OPC-07` / `T-CAL-09` の表の列が壊れている | **対応済み**（§4.3「モードを跨ぐ行」に集約） |
| 6 | R2-17 | 行数の主張が実数と合わず、**過剰行を検出できない** | **対応済み**（106 行に修正。**テスト 2r**（逆方向）を追加） |
| 7 | R3-37 | pytest の突き合わせ方法（marker 規約）が無い | **対応済み**（`@pytest.mark.rule(...)`） |
| 8 | R3-40 | 速度上限の出どころが画面とモードの 2 系統ある | **対応済み**（`min(画面由来, モード由来, v_reverse)`。safety §3.3.1） |
| 9 | R3-46 | `ui.jog.hold` をサービスで 5 Hz は詰まる | **対応済み**（`/ui/jog_lease` トピックへ。webui §5 も修正） |
| 10 | R3-48 | `DEBT-2` の前倒しの最小範囲が曖昧 | **対応済み**（`-packets.md` §5.1 に「作る／作らない」を明記） |
| 11 | R3-49 | `is_path_blocked` は bool しか返さない | **対応済み**（safety §3.4.3 に「最近傍距離を返す形に拡張して移植」＋等価性テスト） |
| 12 | R2-15 | 詳細設計にしか無い遷移を正本へ戻す | **対応済み**（2026-08-16。§3 A-2 ／ 内訳は §3.1） |
| 13 | R3-30 | `v_max` ほか 10 件が `derived` なのに `derive.py` に式が無い | **対応済み**（params §3.1.1 で `speed_from_braking_distance` / `v_max_from_ceiling` を新設。**`v_check` / `v_calib` / `v_leash` は逆算対象が無いので `given` に分類し直した**。§3.1.2 に `person_backstop_ms` / `clear_distance` / `hysteresis_band`） |
| 14 | R3-38 | 故障注入 13 項目が起動コマンド・注入操作・判定式に落ちていない | **対応済み**（[wp2](DetailedDesign-wp2.md) `WP-TEST-01`。**5 と 6 を別アサーションにする**・実機専用 3 本は skip ではなく明示的に落とす・対照ケースを付ける） |
| ~~15~~ | R2-28 | UI 非常停止を押下側にラッチすると、WebUI が落ちたとき復帰できない | **対応済み**（2026-08-17）→ §4.1 `N-4` ／ [safety](DetailedDesign-safety.md) §6.3.1 |
| 16 | R2-30 / R2-31 / R2-37 | 正本の矛盾（`Spec-safety.md` §6.1 と §3.5、`Spec-open.md` C-15 と `Spec-modes.md` §5、`LOCALIZATION_LOST` の位置づけ） | **対応済み**（2026-08-16。§3 A-5 / A-6 / A-7） |
| ~~17~~ | R2-32 / R2-33 / R2-35 | 改名記録の欠落、使われない名前（`speed_preset_*` / `evt.target_reacquired`）、検査 6 の A9 の扱い | **対応済み**（2026-08-17）。`speed_preset_*` は `WP-UI-03` が消費者（`consumers: [web_ui]`）と確定。**`evt.target_reacquired` は削除**——自動再捕捉で走行に戻す経路は `require_explicit_target_selection: true` と衝突する（[names](DetailedDesign-names.md) §8.2 の囲み） |
| **18** | R2-27 | `/system/state` 途絶時のリミッタの既定ゾーン・既定 `auto_brake` | **対応済み**（safety §3.2 末尾） |

---

## 5.3 2 回目のレビュー（全 13 ファイル・観点 3 つ・2026-08-16）

**指摘 84 件**（網羅性 10 ／ 安全整合 38 ／ 実装可能性 36）。

### 対応済み（致命・着手不能）

| 出所 | 指摘 | 対応 |
| --- | --- | --- |
| 網羅-5 | **`SD-*` の ID 名前空間が正本の設計思想と衝突していた**（正本 `SD-5`＝「ゾーンは画面が決める」／詳細設計 `SD-5`＝「自己位置喪失」） | **安全負債を `DEBT-1`〜`DEBT-9` に改名。**`SD-*` は正本参照専用に戻した |
| 安全-1 | **`jog_gate` が 20 Hz でゼロを出す設計だと、`/cmd_vel_manual`（priority 30）が常に非タイムアウトになり、twist_mux が priority 20/10 を永久に選ばない**＝自律走行が構造的に一切出力されない | **不通過時は沈黙する**に変更。「沈黙禁止」は多重化の**後段だけ**に課すと明記 |
| 安全-3 | `C-06` が `fault.critical` と `ui.estop.press` を 1 行にまとめ、ガードが行単位だったため**`CARRY` 中の重大フォルトが `ESTOP` へ行けなかった** | `C-06a`（フォルト・ガード無し）と `C-06b`（UI・ガード付き）に分割 |
| 安全-2 | **`fault.critical` で入った `ESTOP` が依然トラップ**（`C-09f` の `ui.resume_ack` を `resume: none` の UI が出さない） | `attributes.yaml` の `ESTOP` を `ack_only` / `resume_state: NONE` に |
| 安全-4 ／ 実装-7 | **`obstacle_floor_distance_m` の定義が 3 通りあった** | `floor_distance(body_half_length_m, floor_margin_m)`（速度非依存）に一本化。§1.1 の registry 例も差し替え |
| 安全-5 | `override_common` のマッチ規則にガード評価が無く、**`MOTOR` 項目実行中でも `T-OPC-05` が `C-07` を隠して物理 E-Stop が効かなかった** | 規則 3 を「マッチし**かつガードが真**」に |
| 安全-7 ／ 実装-17 | ゾーン導出が state §6 と names §4.1 で**2 実装・2 挙動**（途絶時に `IN`＝`v_slow` が許可される） | state §6 の擬似コードを削除し names §4.1 へ一本化 |
| 安全-8 | `opcheck_runner` / `calib_runner` が `/cmd_vel_manual` に publish する設計で、**`jog_gate` がその 2 モードでは通さないため構造的に動かなかった** | `/cmd_vel_behavior`（priority 20）へ変更 |
| 安全-10 | **リミッタの入力に `/safety/*` が挙がっておらず、L6 を満たせなかった** | 入力表に追加。**全入力について「一度も受信していない＝stale」**も明記 |
| 安全-12 | `C-11` に `no_critical_fault` が無く、**重大フォルト継続中でも `CARRY` から `FOLLOW/RUN` へ戻れた** | `hw_released_and_no_critical` に |
| 安全-13 | `$resume_state` が `ack_only` のモードにしか定義されておらず、`yes_no` のモードで `validate()` が落ちる | **全モード必須**に（`yes_no` は `PAUSE`） |
| 安全-14 | `AT_PANEL` が `resume: none` なのに回復フォルトで `PAUSE` に落ち、**出口の無い状態**になっていた | `ack_only` / `resume_state: IDLE_P` に |
| 安全-15 | `C-06r` が「受理された拒否」だった | スキーマに **`reject: true`** 列を新設 |
| 安全-18 ／ 実装-3 | `mark_arrived` / `abort_check` / `plan_line_route` が effect 一覧に無く `validate()` ③ が落ちる | 一覧に追加 |
| 安全-22 ／ 実装-22 | **`/cmd_vel_manual_raw` が名前辞書に無く**、図と本文が `jog_gate` を通らない旧経路のままだった | 辞書・図・transit §2.1 を同時に修正 |
| 実装-4 | §4.2 と §4.3 が同じ `id` を effects 違いで二重定義していた | §4.3 を**索引**に変更（行を起こさない） |
| 実装-5 | **`mode_entry.yaml` の中身がどこにも無かった** | §8.3 に全内容を転写 |
| 実装-6 | **`attributes.yaml` の 18 モード × 9 キーの実値が無かった** | §8.2 に実値表 |
| 実装-8 | S3 と S5 が両立せず、CI の 2 検査が**互いに矛盾して両方は通らない** | S3 に S5 の例外を明記 |
| 実装-9 | `blocking: true` により**段階 1・2 が実機で構造的に起動不能**になっていた | **`blocking_from_stage`** ＋ `consumers` による絞り込み |
| 実装-10 | `WP-SAFE-01` ↔ `WP-MEAS-04` の**循環依存** | `WP-SAFE-00`（リンク品質の publish のみ）を段階 0 に分離 |
| 実装-12 ／ 安全-23 | `jog_gate` 新設と WebUI の publish 先変更が別段階で、**その間ジョグが潰れる** | 順序制約 **`O-6`** |
| 実装-11 | `safety_monitor` の監視対象の publisher が段階 4・5 まで存在しない | 順序制約 **`O-7`**（`enabled_targets`） |
| 実装-13 | `WP-SAFE-03` の同時変更に **`twist_mux.yaml` の入力トピック改名**と**テスト内インライン辞書**が抜けていた | 4 項目 → **6 項目**に |
| 実装-20 | `th_safety` の `package.xml` / `CMakeLists.txt` に `geometry_msgs` すら無い | 追加依存を `WP-SAFE-03` に明記 |
| 実装-22 ／ 安全-9 | `MUX_DEAD` / `DRIVE_RUNAWAY` / `STATE_INCONSISTENT` に**検出者も検出条件も無い** | **対応済み**（[wp2](DetailedDesign-wp2.md) `WP-SAFE-01` §4.1 に 3 行の検出条件表・§3.1 に購読トピック・§7 に gtest 3 本。**`MUX_DEAD` は両方向を見る**——片方向だと twist_mux のロック中を誤検知する） |
| 実装-27 ／ 安全-26 | `WP-PARAM-02` が launch の `parameters=` を差し替えないと誰も生成値を読まない | 順序制約 **`O-8`** |
| 網羅-6・7 | `Spec-ops.md` §3（1 日の流れ）・§5（G5 の測り方）に対応が無い | `WP-MEAS-05` を新設 |
| 網羅-8 | `Spec-safety.md` §3（**加減速をファームで鈍らせる**）に対応が無い | safety §7.1 を新設 |
| 網羅-9 | `Spec-safety.md` §8（**バッテリー**）が丸ごと落ちていた | safety §7.2 を新設 |
| 網羅-10 | **`CARRY`（手押しモード）に作業パケットが 1 つも無かった** | `WP-CARRY-01` を新設 |
| 安全-26 | 段階 2 の出口で `DEBT-1` を解除するとしていたが、解除条件（始業点検 項目 1）は段階 7 | 段階 2 は「検出できるようにする」までと明記 |
| 安全-21 | onsite が削除済みの `T-ATP-06` を参照していた | `T-ATP-05` のガードに書き換え |
| 安全-29 | ガードに `not leash_taut` と否定を書いていたがスキーマは述語名のみ | `leash_slack` を述語として追加 |
| 安全-25 | **A10 が表に無い**のに 4 か所から参照されていた | A4 表に A10・A11 を追加 |
| 安全-35 | `hysteresis_band_m` が `d_behavior` スケールから導出され、**帯が `d_behavior` を越えうる** | `d_floor` スケールに変更＋A11 |
| 安全-6 | `muxed_stale_ms` と `cmd_vel_stale_ms` を混用 | 修正 |
| 安全-24 | `follow_runner` の担当範囲が 3 か所で食い違い、**`PREP` で対象確定のゲートが無いまま追従が始まる** | transit §0.0 に一本化＋ノード側ゲートを新設 |
| 安全-38 | `plane_core.py` がパッケージ構成に無い | 追加 |

### 2 回目レビューの 13 件 — 処理結果

**10 件を対応済みにした。**主な成果物は
[wp0](DetailedDesign-wp0.md) / [wp1](DetailedDesign-wp1.md) / [wp2](DetailedDesign-wp2.md)（**段階 0〜2 の 23 パケットの実体**）。

| # | 出所 | 指摘 | 対応 |
| --- | --- | --- | --- |
| **1** | 実装-1 | パケットのどれ 1 つも §0.1 のテンプレートで実体が書かれていない | **対応済み**（段階 0〜2 の **23 パケット**を `-wp0` / `-wp1` / `-wp2` に記述）。**段階 3〜8 は未記述**で、着手前に書く運用にした（[packets](DetailedDesign-packets.md) §0.2） |
| **2** | 実装-2 ／ 安全-16・17 | `spec_ref` の実値が 1 行も入っていない | **前提解消**（2026-08-16。正本に ID 列が入った → §3 A-1）。**実値を書くのは `WP-STATE-01` の作業**（`transitions.yaml` の各行） |
| **3** | 安全-9 | 重大フォルト 3 種に検出者・検出条件が無い | **対応済み**（[wp2](DetailedDesign-wp2.md) `WP-SAFE-01` §4.1。`MUX_DEAD` は**両方向**を見る、`DRIVE_RUNAWAY` は保持時間で判定、`STATE_INCONSISTENT` は状態集合との照合） |
| **4** | 実装-28 | 合否が主観の完了条件 | **対応済み**（段階 0〜2 は全パケットの §10 が実行可能なコマンド。`WP-MEAS-02` は `report.md` の 1 行目を `grep` で判定する形にした） |
| **5** | 実装-24 ／ 安全-31 | Playwright の導入手順が無い／新設の安全時定数に registry 行が無い | **対応済み**（[wp1](DetailedDesign-wp1.md) `WP-UI-01` §7 に導入手順。**`npm run dev` ではなく本番ビルド**を使う理由も。時定数は各パケット §3.3 に `class`/`status` 付きで列挙） |
| **6** | 実装-15 | `docker-compose.yml` の bind mount がどのパケットにも入っていない | **対応済み**（[wp1](DetailedDesign-wp1.md) `WP-PARAM-02` §1・§10-②） |
| **7** | 実装-19 | 死角セクタの角度→index 変換規約が無い | **対応済み**（[wp2](DetailedDesign-wp2.md) `WP-CALIB-01` §4.1。**度で持ちラジアンで使う・0 をまたぐ区間を許す・範囲外を clamp しない**。Python と C++ の等価性テストつき） |
| **8** | 実装-21 | 故障注入 #12 は Gazebo で実行できない | **対応済み**（[wp2](DetailedDesign-wp2.md) `WP-SAFE-02` §8・`WP-TEST-01` §1。**「統合テスト ＋ 実機」に読み替える**。`launch_testing` で WS モックサーバーを立てる） |
| ~~9~~ | 網羅-1・2 | `O-d5` / `CL-M-7`（物の配置が変わる前提の地図再利用可否） | **対応済み** → 下記 B。さらに **2026-08-17 に正本へ `F-41` として戻した**（§3 A-13） |
| ~~10~~ | 網羅-3・4 | `F-18` / `T-r11` / `T-r12` / `DF-D-7`（ハードウェア構成表） | **対応済み** → 下記 C（[hardware](DetailedDesign-hardware.md) を新設） |
| **11** | 安全-34 | 数の主張が実数と合わない | **対応済み**（**48 パケット**／18 モード中 **16 モード**を覆う／**ガード 27 件**。数は機械的に数える旨も明記） |
| **12** | 安全-30・31 | `tracker_lost_grace_ms`（暫定 500 ms）など裸の数値 | **対応済み**（[safety](DetailedDesign-safety.md) §5.4 を `class`/`status` の記述に置換。`WP-SAFE-01` §11 で `blocking_from_stage: 4` を明示） |
| **13** | 安全-36・37 ／ 実装-34・35 | `estop_ui_lease_ms` 未使用・`sim_override` がスキーマに無い・`/robot/mode` 参照が残る・`conftest.py` の `_repo_root` | **対応済み**（`estop_ui_lease_ms` は `UI_DISCONNECTED` の契機として用途を確定 → `WP-SAFE-01` §4.2。`sim_override` は廃し `--sim` ＋ launch 引数に。`/robot/mode` は `/system/state` へ。`_repo_root` の修正は `WP-STATE-01` §7 の作業に入れた） |

### 2 回目の指摘の追跡（**残るのは `D` の 1 件だけ**）

| # | 出所 | 指摘 |
| --- | --- | --- |
| ~~A~~ | 実装-2 ／ 安全-16・17 | **対応済み**（2026-08-17）。正本の ID 列（A-1）を受けて、**[state](DetailedDesign-state.md) §4.4 に全 128 行の `spec_ref` 実値**を書いた。**正本 119 ID の被覆率 100%・過剰 0**（A-13 の 4 行を含む）。1 正本 ID に複数行が対応する 6 組を §4.4.3 で宣言し、**宣言外の fan-out を検出するテスト 2f** を §11 に追加 |
| ~~B~~ | 網羅-1・2 | **対応済み**（2026-08-17）。[onsite](DetailedDesign-onsite.md) **§3.7**。「作り直す／そのまま使う」の二択にせず **A（そのまま）／B（書き足す）／C（作り直す）の 3 段**にし、**B のために `map_update` フラグを試験場内モードへ広げた**。判定は自動化せず、**スキャンの重ね描きと `/onsite/declare_home` の実測ずれ**で人が選ぶ。**どの段を既定にするかは `O-d5` のまま**（会場条件が要る） |
| ~~C~~ | 網羅-3・4 | **対応済み**（2026-08-17）。**[hardware](DetailedDesign-hardware.md) を新設。**機器→ノード→トピックの対応・**USB 帯域の実測手順**・ESP32 フレーム表（`LED_STATE 0x05` / `BATTERY 0x06` の新設と `ESTOP_HW` の拡張を集約）・**カメラの選定（CSI・型番は決めない）**・手配のリードタイム表 |
| **D** | — | **段階 3〜8 のパケットに実体が無い。**着手前に `DetailedDesign-wp<n>.md` を書く（[packets](DetailedDesign-packets.md) §0.2） |

**B・C はいずれも「正本にはあるが詳細設計に無い」**という向きの欠落だった
（A のような「詳細設計にしかない」の逆）。**正本を直す作業ではない。**

**`D` は意図的に残している。**段階 3〜8 のパケット本体は、
**段階 0〜2 の実装で前提が動く**（測定値が入り、`state_core` の実装で表の粒度が確かめられ、
リミッタの実物が挙動ノードの出力先を確定させる）。
先に書くと書き直しになるので、**段階 3 に着手する時点で書く。**

**A〜C が片づいたので、3 回目のレビューを行える。**D は段階 3 に着手する時点でよい
（先に書いても、段階 0〜2 の実装で前提が動く）。

### C で新たに決めたこと（`Spec.md` §6.1 #8 の決着）

`Spec.md` §6.1 の項目 8（カメラ）は **「要検討（詳細設計）」**と書かれた
**唯一のハードウェア判断**だった。[hardware](DetailedDesign-hardware.md) §4 で決着させた。

| 決めた | 決めなかった（意図的） |
| --- | --- |
| **CSI 接続**（USB 帯域を食わない唯一の選択肢） | **具体的な型番。**`WP-LINE-00` の実測と合わせて選ぶ |
| 前方 1 台・床面が画角に入ること | 取り付け高さ・俯角・キャリブレーション（`LINE` の内部設計） |
| 要求 3 つ（解像度・照明変化耐性・CSI） | — |

**型番をここで書くと `WP-LINE-00` の測定が「選んだものの確認」になる**——`DD-6` と同じ構造の誤り。

---

## 5.4 ID の引用方針（**`DD-5` の運用規則**）

**正本の ID 248 件のうち、詳細設計が直接引用するのは 171 件である。**
残り 77 件を引用しない理由をここで明文化する。**これが無いと、レビューのたびに同じ 77 件が指摘される。**
検査は `tools/check_id_coverage.py` が機械的に行う（§5.4.2）。

### 5.4.1 引用する ID・しない ID

| 族 | 性質 | 詳細設計での扱い |
| --- | --- | --- |
| `SD-n` / `G-n` / `U-nn` / `C-nn` / `F-nn` | **正本が決めたこと**（設計思想・目標・UI 要件・確認事項・新規決定） | **必ず引用する** |
| `O-an` / `O-cn` / `O-dn` | **未確定事項** | **必ず引用する**（[§4](#4-なお残る未確定完全設計書から引き継ぐ) の表がその一覧） |
| **`CL-*` / `T-r*` / `O-r*` / `C-r*` / `K-r*` / `N-*` / `Z-*` / `DF-D-*` / `E-*`** | **原典（`source/` / `restate/`）からの棚卸し番号。**「原典のどこに書いてあったか」を示す出自の記号 | **引用しない。**`Spec-open.md` §6 が各行を `F-nn` か `O-xx` に処理済みで、**詳細設計はその処理先を引用する** |

**出自の記号まで詳細設計に持ち込むと、実装者が読む文書に「原典のどの行から来たか」という
実装に無関係な情報が増える。**`DD-2`（1 パケットで完結）に反する。

### 5.4.2 それでも検査する方法（**推移的な被覆**）

「引用しない」を「落としてよい」にしないため、**2 段の検査**にする。

```bash
# ① 正本の CL-*/T-r*/… が Spec-open.md §6 で全件処理されているか（正本側の検査）
#    → Spec-open.md §6 が「未処理 0」を担保している

# ② その処理先（F-nn / O-xx）が詳細設計で引用されているか（詳細設計側の検査）
python3 docs/plan/detailed/tools/check_id_coverage.py .
#   → 「直接引用が必要な ID の漏れは 0 件」で exit 0
#   → 未分類の族が出たら §5.4.1 の表に足す（exit 1）
```

**現在の結果**: 正本 248 件 / 直接引用 171 件 / 未引用 77 件（**全部が出自の族**）→ exit 0
（2026-08-17。`F-41` の新設で正本が 1 件増え、そのまま引用側にも入った）。

**②が通れば、①を経由して出自の記号も推移的に覆われている。**

### 5.4.3 直接引用が必要なのに落ちていた 5 件（**このレビューで発見・修正済み**）

| ID | 内容 | 修正 |
| --- | --- | --- |
| **`SD-1`** | **操作系は WebUI のみ** | **1 回目レビューの `SD-*`→`DEBT-*` 一括改名が、正当な `SD-1` の参照まで書き換えていた。**[packets](DetailedDesign-packets.md) §2・§10 の「段階 7 の出口」が **`DEBT-1`（物理 E-Stop バイパス）達成**と読める状態になっていた（別物）。`SD-1` に戻し、`DEBT-1` の解除も同じ段階であることを併記 |
| `SD-2` | PC・タブレット／縦長・横長 | [webui](DetailedDesign-webui.md) §2.3 に引用を追加 |
| `SD-3` | ヘッダは常に最上位 | 同 §2.1 に引用を追加（**実装はされていた**） |
| `SD-4` | モードと状態を分ける | [state](DetailedDesign-state.md) 冒頭に引用を追加 |
| `F-06` | 停止距離の式 | [params](DetailedDesign-params.md) §3.1 `braking_distance()` の docstring に追加 |

> **`SD-1` の取り違えが 2 回のレビューを生き延びた理由**は、
> **改名を機械置換で行い、置換後に ID の意味が通るかを検査しなかった**ことである。
> **ID を一括改名したら、改名後の各出現箇所で意味が通るか 1 件ずつ確認する**——§6 に規則として追加した。

### 5.4.4 引用しないと決めた 77 件の内訳

**手で数えない。**下表は `check_id_coverage.py` の出力を族ごとに集計したものである
（2026-08-17 時点。**旧版は「78 件」と書いて内訳の合計が 77 だった**——
手で数えて表と本文が別々にずれていた典型）。

| 族 | 件数 | 内訳 |
| --- | --- | --- |
| `CL-*`（原典の確認事項） | 37 | B 3 ／ D 6 ／ G 7 ／ M 4 ／ P 3 ／ R 5 ／ T 6 ／ X 3 |
| `T-r*`（原典の技術的指摘） | 15 | — |
| `O-r*` / `C-r*`（原典の未確定・確認） | 9 | `O-r*` 8 ／ `C-r*` 1 |
| `N-*`（原典の注記） | 9 | — |
| `DF-D-*` / `Z-*` / `E-*` | 7 | 3 ／ 2 ／ 2 |

> **この表に具体的な ID を書かないこと。**書いた瞬間に検査上「引用済み」に変わり、
> **未引用の件数が表自身のせいで減る**（旧版が代表 ID を並べていたため、
> `check_id_coverage.py` の数と表の数が食い違っていた原因のひとつがこれである）。
> **族名と件数だけを書く。**

**`F-17` はこの表に入らない。**詳細設計が `F-17` に言及している（§3 A-12）ため
検査上は「引用済み」に数えられる。**中身は `C-14` へ移っており、引用すべきは `C-14` のほうである。**
番号の行を正本から消さずに残してあるのは、**欠番と書き漏れを区別できるようにするため**
（`Spec-open.md` §7 に規則として明文化した）。

---

## 5.5 3 回目のレビュー（2026-08-17・観点 3 つ）

**指摘 50 件**（網羅性 5 ／ 安全整合 22 ／ 実装可能性 28、うち重複あり）。
R1（網羅性）は機械検査で代替し、R2・R3 はサブエージェントが実施した。

### 5.5.1 発見された「2 回のレビューを生き延びた」欠陥

| # | 欠陥 | なぜ見逃されたか |
| --- | --- | --- |
| **`SD-1` → `DEBT-1` の取り違え** | 段階 7 の出口が別物になっていた | **機械置換で改名し、置換後の意味を検査しなかった**（§5.4.3・§6-2） |
| **`validate()` ⑥ の除外リスト** | 設計どおり実装すると**システムが一切起動しない** | 除外集合が §2 と §4.1.1 の 2 か所にあり、片方だけ更新した |
| **§7.1 の resume 表** | `ESTOP` が再びトラップに戻っていた | §8.2 を直したとき §7.1 を追随させなかった |
| **不変条件 L5** | 死角を正しく校正した瞬間に**全方向が `v_reverse` に固定**される | 「死角方向は」と「全方向は」の差が §4.3 と L5 で逆だった |
| **`override_common` の粒度** | モード単位に読め、**手動走行中だけ層 1・2 が消える** | 表に列が無く、注記が「モードの性質」のように書かれていた |
| **A2a の適用軸** | 極低速の軸まで課すと**設計どおりの値で起動拒否** | `value_by` を導入したとき「全軸」と書いた |

### 5.5.2 対応（主要なもの）

| 出所 | 指摘 | 対応 |
| --- | --- | --- |
| 安全-1 | `map_update` の試験場内拡張が正本と食い違うのに申し送りが 0 件 | **申し送り A-13 を新設** → **正本へ反映済み**（2026-08-17。`Spec-open.md` **F-41**） |
| 安全-2 | `map_update` ON でも保存する遷移が無い | [onsite](DetailedDesign-onsite.md) §3.7.3.1 ／ [state](DetailedDesign-state.md) §4.2 に **`T-PNAV-09` / `T-SUM-14` / `T-HNAV-08` / `T-ATP-07`** の 4 行。正本は `SM-3.1.2-099`〜`-102` |
| 安全-3 | `has_record` / S-21 / 名前辞書が未追従 | [state](DetailedDesign-state.md) §8.2 ※ ／ [webui](DetailedDesign-webui.md) §8.3.1 ／ [names](DetailedDesign-names.md) §3.1 |
| 安全-4 | `validate()` ⑥ が `OPCHECK`/`CALIB` で必ず落ちる | 除外を 6 モードに統一 |
| 安全-5 | 6 ガードが `Context` に無い `mode`/`state` を読む | **述語は `(mode, state, ctx)` の 3 引数**と §2・§4.1.1 に明記。参照列も実体に修正 |
| 安全-6 | §7.1 の resume 表が §8.2 と矛盾 | §7.1 を `ESTOP`/`AT_PANEL` = `ack_only` に。**実値の正は §8.2** と明記 |
| 安全-7 | L5 が全方向を `v_reverse` に | 「死角方向の成分だけ」に修正 |
| 安全-8 | §3.4.3 のクランプ式が政策表を無視 | `policy_stops(source_class, zone, auto_brake)` を通す形に |
| 安全-9 | A2a を全軸に課すと起動拒否 | **自律走行の 3 軸だけ**に限定（§4.1 に理由） |
| 安全-10 | **twist_mux のロックだけ fail-open** であることが未記載 | [safety](DetailedDesign-safety.md) **§1.2** を新設。「重複だから削る」を却下する根拠 |
| 安全-11 | `override_common` がモード単位に読める | 表に列を追加し、**行の属性**であることを囲みで明記 |
| 安全-12 | `jog_gate` 閉後に手動指令が流れる窓 | [safety](DetailedDesign-safety.md) **§1.3** で時系列を明示。**駆動が止まるのは `muxed_stale_ms` 後**。**A12** を追加 |
| 安全-13 / 実装-19・20 | `mux_dead_ms` ほか 12 名が辞書に無い | [names](DetailedDesign-names.md) §6.2・§7 に追加 |
| 安全-14 | `hysteresis_band_m` の導出元が 3 箇所で食い違う | 4 箇所すべて `d_floor` スケールに統一 |
| 安全-15 / 実装-1 | 遷移表の行数が 121/122/124 に分裂 | **124（＝18＋106）**に統一。**A-13 の 4 行を足して現在は 128（＝18＋110）** |
| 安全-16〜22 | 拒否キー・監視対象・ガード参照列・`C-06r` の順・LED・A11 テスト | 各所を修正 |
| **実装-1〜12** | **§10 の完了条件 31 件が動かない** | [packets](DetailedDesign-packets.md) **§0.2 の後に §0.3「完了条件の書き方」V1〜V10 を新設**し、wp0/wp1/wp2 の該当ブロックを修正 |
| 実装-13・14 | リミッタの struct 未定義・等価性テスト方式未定 | [wp2](DetailedDesign-wp2.md) `WP-SAFE-03` §4.1 に 4 struct の表、§7 に **JSON ベクタ方式**を明記 |
| 実装-21 | `blind_angle_ranges` の型が現行 flat list と食い違う | `WP-CALIB-01` §5 に**変換規則**（8 要素 → 4 ペア）を新設 |
| 実装-24 | 「この 5 件」が実際は 8 件 | 修正 |

### 5.5.3 特に重い指摘 —— 検査が「常に合格する」形になっていた 5 件

**約束 3（完了したかどうかを自分で判定できる）が、形式だけになっていた。**

| 場所 | なぜ常に合格するか |
| --- | --- |
| `WP-UI-01` §10③ | `grep` に `-E` が無く `\|` がリテラル。**この正規表現は絶対に一致しない** |
| `WP-CALIB-01` §10② | 現行 YAML は `- 40.0` の flat list で、`40, 40` という並びが**存在しない** |
| `WP-ESP32-01` §10③ | `ESTOP_HW.*3` が既存の `ESTOP_HW (0x03)` に当たり、**改修前でも合格** |
| `WP-NET-01` §10① | `.example` が存在しないと `grep` が exit 2 → **`!` で反転して合格** |
| `WP-SAFE-00` §10③ ／ `WP-PARAM-02` §10③ | ヒアドキュメントの中身が**コメント 1 行だけ**で何も実行しない |

**`V2`（否定 grep は対象の存在を先に確かめる）と `V3`（期待文字列そのものを書く）を
§0.3 の規約にしたのは、この 5 件が同じ原因だったからである。**

### 5.5.4 R3 が確認した「実在した」参照

**既存コードへの参照 12 件はすべて実在し、行番号も一致した**（`mapless_follow_core.py:119-157` の
`is_path_blocked`、`esp32_bridge.py:237` のキープアライブ、`App.jsx` の `VirtualStick` L149 ほか）。
**`DD-4`（既存資産の去就）の記述は信頼してよい。**

ただし 2 点の補足:

| 事実 | 影響 |
| --- | --- |
| `esp32/src/ws_link.h` のフレーム表は **`(9 bytes)` / `(2 bytes)`** のまま | `DEBT-7`。§10 の検査を期待文字列に直した |
| **`th_safety/CMakeLists.txt` にテスト登録が 1 件も無い** | `colcon test --packages-select th_safety` は**現在この瞬間でも合格する**。`V5`・`V6` の根拠 |

### 5.5.5 3 回目レビューの残件（2026-08-17 に消化）

| # | 指摘 | 対応 |
| --- | --- | --- |
| **実装-11** | ctest 登録名が §7 に無い | **7 パケットの §7 に「ctest 登録名」列と `CMakeLists.txt` の登録行を追加。**あわせて **`DEBT-10`（`th_safety` にテスト登録が 0 件）を台帳へ新設**し、`WP-SAFE-00` §10 に**実行件数を数える検査**を置いた（登録し忘れると `-R` が空振りして無条件合格するため） |
| **実装-15** | §2 の参照に節番号が無い（2 件） | `WP-MEAS-02` → onsite §3.1・§3.5 ／ `WP-CALIB-01` → maintenance §3.1〜§3.3・§3.6 |
| **実装-16** | 完全設計書を直接読ませる箇所の例外が未明記 | [README](README.md) **§4.1** を新設。**4 件・フルパス・写さない理由**を表にした。**4 件とも段階 0 の測定パケットで、コードを書くパケットは 0 件** |
| **実装-17** | `WP-NET-01` §2 に hardware への参照が無い | hardware **§1**（Wi-Fi と USB は別のリンク）・**§3/§3.1**（フレーム表の正本）を追加 |
| **実装-18** | 名前辞書の JSON 化の手順が無い | **`tools/export_names.py` を新規作成**（Markdown が正本・JSON は派生物）。[webui](DetailedDesign-webui.md) **§9.1** に生成手順、[names](DetailedDesign-names.md) 冒頭に相互参照、`WP-UI-01` §10⑤ に `git diff --exit-code` を追加。**実行して 18 モード / 15 画面 / 41 トピック / 33 サービス / 92 パラメータ / 62 トリガ / 23 msg を抽出できることを確認済み** |
| **実装-22** | `O-5` が段階 0 の実機測定と矛盾／§13 に `ESP32-01` の辺が無い | **例外を作らず `WP-ESP32-01` を段階 2 → 段階 0 へ移した**（下記 §5.5.6）。§13 の依存グラフに辺を引き、段階別の数を 0:11 / 2:6 に直した |
| **実装-23** | 測定パケットに §11/§12 が無い | **指摘が誤りだった。**23 パケット全部が §0〜§12 を持つことを機械検査で確認（節番号を抽出して欠番を数えた） |
| **実装-26** | 目視前提の検査が残る | `WP-CALIB-01` §10④ を**区間内が全部 `inf`・区間外を消していない**の 2 条件に機械化（広く取りすぎの検出＝§6.3① が検査になった）。`WP-SAFE-01` §10③・`WP-SAFE-02` §9・`WP-SAFE-03` §10⑦・`WP-ESP32-01` §10⑤・`WP-STATE-*` の `topic echo` を `test "$(timeout 3 …)" = "…"` へ |
| **実装-27** | `<YYYY-MM-DD>` の雛形が残る | `WP-MEAS-01` §10① に**`<...>` が残っていないことの検査**を追加（`value` / `measured_at` / `source` の 3 つ） |
| **実装-28** | `WP-SAFE-03` §2 のファイル跨ぎアンカーが脆い | **`WP-CALIB-01` §4.1（このファイルの前半）**という書き方に変更。アンカーに依存しない |
| — | `kill %1` / 裸の `topic hz` が 5 箇所に残っていた（`V4` / `V7` 違反） | 上記の作業中に発見。全部 `PID=$!` ＋ `kill -TERM` と `timeout 6` に直した |
| 安全 §3 の到達可能性 | 全 18 モードの機械的な到達可能性検査 | **`transitions.yaml` の実体が無いと回せない。`WP-STATE-01` の `validate()` ⑤ が担う**（設計上の宿題ではなく実装時に自動で回る） |

### 5.5.6 `O-5` の矛盾は「例外」ではなく「順序」で解いた

**`WP-MEAS-01`（制動加速度）と `WP-MEAS-02`（人が歩く部屋の地図）は段階 0 で実機を走らせる。**
`WP-ESP32-01`（物理非常停止のバイパス解除）が段階 2 にあると `O-5` を必ず破る。

**最初は `O-5` に例外を書こうとしたが、それは誤りだった。**
`WP-MEAS-01` §6.1 が既に「`DEBT-1` が有効な状態で走らせない」と書いており、
**例外を作ると `O-5` と `WP-MEAS-01` §6 のどちらかが嘘になる。**
加えて**急停止の測定を「急停止できない機体」で行うのは測定として成立しない。**

**`WP-ESP32-01` を段階 0 へ移した**（[packets](DetailedDesign-packets.md) §1.1）。
依存は `WP-NET-01`（同じ段階 0）だけで、`std_msgs/UInt8` しか使わないので移せる。
**総パケット数 48 は変わらない**（0:10→11 / 2:7→6）。

> **`Spec-open.md` §7 に今回加わった「検査を緩めて通すのではなく、正本を直して通す」と同じ形である。**
> 制約と現実が食い違ったとき、**制約に穴を開けるのは最後の手段**にする。

---

## 6. この文書の保守

1. 新しい指摘は §5 に追記する。**「対応済み」か「対応しない＋理由」に落とし切るまで完了としない**
2. **ID を一括改名したら、改名後の各出現箇所で意味が通るか 1 件ずつ確認する。**
   機械置換だけで済ませると**同名だが別物の ID を巻き込む**。3 回目レビューで見つけた
   `SD-1`→`DEBT-1` の取り違えは 1 回目の `SD-*`→`DEBT-*` 改名が原因で、**2 回のレビューを生き延びた**（§5.4.3）
3. §3 の申し送りは、**正本を直した時点で行を消す**（`Spec-open.md` §4「陳腐化」と同じ扱い）
4. §4 の未確定は、値が入った時点で `registry.yaml` の `status` を変え、この表から消す
5. §2 の安全負債は、解除条件の検証が通った時点で消す。**消す前に検証結果を記録する**
6. **ID の引用漏れは `tools/check_id_coverage.py` で機械検査する**（§5.4.2）。
   新しい族が増えたら §5.4.1 の表と `MUST_CITE` / `PROVENANCE` を同時に直す
