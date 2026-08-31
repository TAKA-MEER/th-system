# 段階 3 の作業パケット（手動走行）

[DetailedDesign-packets.md](DetailedDesign-packets.md) §6 の実体。
**このファイルには `WP-UI-03` と `WP-TRANSIT-01` の 2 件だけを書く。**
同じ段階の `WP-ROUTE-01`（`route_recorder`）と `WP-TRANSIT-02`（`TEACH_MANUAL` / S-13）は
着手時に追記する（`DD-2`「1 パケット＝1 セッションで完結」に従い、必要になってから書く）。

> **なぜこの 2 件を先に書くか**: 2026-08-31 の実機日に、走行を伴う 3 項目
> （故障注入 04・`WP-SAFE-03` の障害物停止・故障注入 08）が**走行画面が存在しないため実施できない**
> ことが判明した（[実機作業キュー](../../実機作業キュー.md) §1.2.1）。
> `obstacle_limiter` が `applied_limit_mps = 0.000` でクランプする
> → `/system/state.speed_limit = stop`
> → `zones.py` の `SCREEN_SPEED_LIMIT["S-01"] = "stop"`
> → **走行画面が無く S-01 に留まっている**、という連鎖である。
> S-11 が出来れば `SCREEN_SPEED_LIMIT["S-11"] = "v_max"` が効き、この 3 項目が実施できるようになる。

---

## WP-UI-03 走行タブ（`VirtualStick` / `SpeedPreset` / W-6）

### 0. 一行要旨

**S-10 / S-11 / S-12 / S-13 が「そのまま差し込む」1 枚の走行タブと、
スティックを常設できない画面のための手動操作パネル（W-6）を作る。**
画面そのものは作らない。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `parts/VirtualStick.jsx`（既存 `App.jsx` L149-237 の抽出） | S-10 / S-11 / S-12 / S-13 の画面本体（`WP-TRANSIT-01` 以降） |
| `parts/SpeedPreset.jsx`（低速／中速／高速） | `parts/CandidateRadar.jsx`（追従系のレーダー。`WP-UI-04`） |
| `screens/driveTab.jsx`（4 画面共有。`kind='follow' \| 'manual'`） | `jog_gate` ノード（`WP-SAFE-04`。**先に完了していること**） |
| `ros/useJogLease.js`（`/ui/jog_lease` ＋ `/cmd_vel_manual_raw`） | `th_state` の遷移（`T-MANUAL-01/02` は実装済み） |
| W-6（手動操作パネル）の開閉と中身 | 教示タブ（`WP-TRANSIT-02`） |

**`kind='follow'` の分岐は「レーダーの差し込み口を空けておく」までとする。**
`CandidateRadar` は `WP-UI-04` の範囲なので、このパケットでは `manual` 側だけが中身を持つ。

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [webui](DetailedDesign-webui.md) §5 | **ジョグの送出（リース）。最重要** |
| [webui](DetailedDesign-webui.md) §6.3 | **W-6 の要件（`U-16` 改訂）** |
| [webui](DetailedDesign-webui.md) §8.1 | 4 画面が 1 つの走行タブを共有する |
| [webui](DetailedDesign-webui.md) §2.1・§2.4 | 重なり順（W-6 は z=60）／スクロールさせない |
| [webui](DetailedDesign-webui.md) §1 | ファイル構成（置き場所） |
| [Spec-webui.md](../spec/Spec-webui.md) §3.5・§6 | 走行タブの中身／仮想スティックと速度プリセットの仕様 |
| `docs/plan/spec/mockup/index.html` | **見た目の正本。**`#tplStick` テンプレート（L491-514）と `#jogWin` の CSS |
| [names](DetailedDesign-names.md) §6.1・§6.4 | `/cmd_vel_manual_raw` → `jog_gate` → `/cmd_vel_manual` ／ `/ui/jog_lease` |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS | レート |
| --- | --- | --- | --- | --- |
| **pub** | **`/ui/jog_lease`** | `std_msgs/String`（`data` = このクライアントの `client_id`） | best_effort, depth 1 | **5 Hz 以上。**触れている間だけ |
| **pub** | **`/cmd_vel_manual_raw`** | `geometry_msgs/Twist` | reliable, depth 1 | **10 Hz。**触れている間＋離した直後にゼロ 1 回 |

**`client_id` は `ros/clientId.js` の `getClientId()` を使う**（`/ui/active_screen` と同じ値）。
`state_manager` は `/ui/jog_lease` の `data` をそのまま `requester` として扱う。

**離したとき**:

| やること | やらないこと |
| --- | --- |
| `/cmd_vel_manual_raw` へゼロを **1 回**送る | `/ui/jog_lease` の送信を止めるだけ |
| — | **`ui.jog.release` を送らない**（そういうトリガは無い。リース満了で FSM が解除する） |

#### 3.2 サービス／アクション

**使わない。**

> **`ui.jog.hold` をサービス `/system/trigger` で送ってはいけない**（[webui](DetailedDesign-webui.md) §5 の囲み）。
> rosbridge 越しの同期呼び出しを毎秒 5 回、WiFi 受信ギャップ 0.5〜1.2 s の環境で行うと
> **応答待ちが詰まってリースが切れる**。しかも毎回 `accepted` と `reject_reason_key` が返るので、
> 拒否のたびに UI が異常表示する。**リースは「届けばよい」性質**なので best_effort のトピックが正しい。

#### 3.3 パラメータ

| 名前 | 単位 | class | status | registry 行 |
| --- | --- | --- | --- | --- |
| `speed_preset_low` | ratio | b | given (0.3) | `registry.yaml`:173 |
| `speed_preset_mid` | ratio | b | given (0.6) | `registry.yaml`:182 |
| `speed_preset_high` | ratio | b | given (1.0) | `registry.yaml`:191 |
| `jog_lease_ms` | ms | b | given (1200) | `registry.yaml`:687（**送出周期の上限の根拠**。UI は直接使わない） |

3 つの `speed_preset_*` はいずれも `consumers: [web_ui]`。
**JSX に `0.3` などの数値を書かない**（規約 `R2`）。`scripts/gen_attributes.py` と同じ流儀で
`registry.yaml` から `src/generated/speed_presets.json` を生成し、それを import する。

> **プリセットは「上限に対する割合」であって上限そのものではない。**
> 速度の権威は `obstacle_limiter`（[transit](DetailedDesign-transit.md) §2.1 の表）。
> ブラウザ側のスケーリングは利便性にすぎない。**現行 `App.jsx:136 stickToCmd` は
> `JOG_LIN_MAX = 0.5` をブラウザに持っており、上限がブラウザにしか無い**——これを引き継がない。

#### 3.4 フレーム

`Twist` は `base_link` 基準の指令値。`frame_id` を持たない型なので TF は使わない。

### 4. 内部設計

#### 4.1 純粋コアの関数シグネチャ一覧

```js
// parts/stickGeometry.js（ROS 非依存・DOM 非依存。Node の素のテストで回せる）
stickToCmd(dx, dy, len) -> { vn, wn, label }   // 既存 App.jsx:136 をそのまま移す
rampToward(current, target, maxDelta) -> number // 既存 App.jsx:127 をそのまま移す
```

**`stickToCmd()` の中身は変えない。**8 方向セクター（45° 幅）・デッドゾーン 0.15・
斜めの旋回 0.5 倍という既存の挙動は実機で調整済みの値であり、この パケットの範囲外。
**移設と単体テストの追加だけを行う。**

#### 4.2 コンポーネントの責務

| ファイル | 責務 |
| --- | --- |
| `parts/stickGeometry.js` | 上の 2 関数だけ。React も ROS も import しない |
| `parts/VirtualStick.jsx` | pointer イベント → `stickGeometry` → `onChange(cmd)` / `onRelease()`。**送出はしない** |
| `parts/SpeedPreset.jsx` | 低速／中速／高速の 3 ボタン。選択値を親へ返すだけ |
| `ros/useJogLease.js` | `held` と `cmd` を受け取り、2 本のタイマ（5 Hz のリース／10 Hz の速度指令）を回す。**唯一の送出口** |
| `screens/driveTab.jsx` | 上 3 つを組む。`kind` で `follow` / `manual` を分ける |

**送出を `useJogLease.js` 1 箇所に閉じる**理由: `VirtualStick` にも `driveTab` にも publish を
書くと、W-6 と常設スティックの両方から二重に送られる経路ができる。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **U3-1** | **`VirtualStick` が unmount されるとき、ドラッグ中なら必ず `onRelease()` を呼ぶ** | 既存 `App.jsx` L163-168 の安全処理。タブ切替で `pointerup` が届かず、**最後のジョグ指令が流れ続ける**事故が実際にあった。**移設時に落とさない** |
| **U3-2** | **`?view=audience` では 1 バイトも送らない** | `main.jsx` が `AudienceView` を別ツリーでマウントしており、走行制御に触れないことが構造で保証されている。この構造を崩さない |
| **U3-3** | **リースの送出周期は `jog_lease_ms`（1200 ms）より十分短いこと。5 Hz（200 ms）以上** | 途絶で `sys.jog_lease_expired` → `PAUSE`。**手を離す操作が届かなくても止まる**設計の前提 |
| **U3-4** | **イベントハンドラのクロージャに対象画面 ID を閉じ込めない。ref か state から都度読む** | [webui](DetailedDesign-webui.md) §6.3 の囲み。モックアップ初版は登録時に 1 度だけ読んでおり、**W-6 のスティックが触っても何も起きなかった** |
| **U3-5** | **W-6 を開閉しても本文のレイアウトが 1 px も動かない** | §6.3。帯にすると本文を圧迫してスティックを小さくせざるを得ない。`theme.css` の `#jogWin` は `position:absolute` で実装済み |

### 5. 表駆動データ

`registry.yaml` の `speed_preset_low` / `_mid` / `_high`（3 行）。加工しない。

### 6. 安全要件

#### 6.1 触れる層

**多重化の層に入らない。**UI は `jog_gate` の**手前**に指令を置くだけで、
通す／通さないの判定も速度上限も下流（`jog_gate` / `obstacle_limiter`）が持つ。

#### 6.2 フェイルセーフ既定

| 事象 | 挙動 |
| --- | --- |
| rosbridge 切断 | 送出を止める。**再接続時に自動で再開しない**（触り直しが要る） |
| `/system/state` が stale | `useSystemState` の `stale` でスティックを非活性にする（表示上の親切。権威は `jog_gate`） |
| コンポーネントの unmount | `onRelease()` → ゼロ 1 回 → 送出停止（U3-1） |
| タブが背面に回った | 送出を止める（`document.visibilityState`）。**リースが切れて `PAUSE` に落ちるのが正しい** |

#### 6.3 FMEA

| # | 壊れ方 | 起きること |
| --- | --- | --- |
| ① | リースを送り続けたまま UI が固まる | **危険側。**`jog_gate` は通し続ける。→ ただし `VirtualStick` の pointer イベントが止まれば `cmd` も止まり、`/cmd_vel_manual_raw` の途絶で twist_mux の `manual_joy` が 1.0 s でタイムアウトする。**速度指令とリースを別タイマにしている理由がこれ** |
| ② | リースだけ止まり速度指令が続く | 安全側。`sys.jog_lease_expired` → `PAUSE`、`jog_gate` が沈黙する |
| ③ | W-6 のスティックが反応しない（U3-4 の再発） | 走らなくなるだけ。危険側ではないが、**モックアップで実際に起きた**ので受け入れ条件に入れる |

### 7. 単体試験

| テスト | 登録名（`V6`） | 満たす仕様 |
| --- | --- | --- |
| `stickGeometry.test.js`（`node --test`） | `npm run test:unit` | `stickToCmd()` の 5 セクター・デッドゾーン・`rampToward()` |
| `speed-presets-from-registry.test.js` | 同上 | 生成 JSON が `registry.yaml` の 3 行と一致（`R2`） |
| `e2e/jog-lease-rate.spec.js` | `npx playwright test` | **§9-8。5 Hz 以上で送られ、`ui.jog.release` を送っていない** |
| `e2e/jog-release-sends-zero-once.spec.js` | 同上 | 離したらゼロを **1 回だけ** |
| `e2e/w6-does-not-move-body.spec.js` | 同上 | **§9-9。**開閉前後で主表示の `getBoundingClientRect()` が一致 |
| `e2e/w6-stick-responds.spec.js` | 同上 | FMEA ③（U3-4）。W-6 のスティックを触ると送出が始まる |
| `e2e/stick-unmount-releases.spec.js` | 同上 | U3-1。画面を離れるとゼロが出て送出が止まる |

**e2e の書き方で踏んだ落とし穴**（同じ形の検査を書くときは先に読むこと）:

| 症状 | 原因 | 正しい書き方 |
| --- | --- | --- |
| `.stick svg` が strict mode violation | W-6 のスティックが常時 DOM にあり 2 件マッチする | 常設側は `#body .stick svg`、W-6 側は `#jogWin .stick svg` |
| 閉じた W-6 を待つと永久に止まる | `#jogWin:not(.show)` は `display:none` で、既定の `waitFor()` は**可視**になるまで待つ | `waitFor({ state: 'hidden' })` |
| 中身の無い印の `div` を待つと止まる | 大きさ 0 の要素は Playwright から見て不可視 | 印を置かず、**消えたはずの要素の `toHaveCount(0)`** で確かめる |
| 「離さないまま画面を離れる」が再現できない | スティックが `setPointerCapture()` でポインタを掴んでおり、`page.click()` がボタンに届かない（**それこそが再現したい状況**） | `locator.dispatchEvent('click')` で捕捉を迂回する |

**rosbridge のモックは既存の流儀を使う**: `e2e/helpers.js` の `gotoScreen()` ＋
`window.__thTestState`。送出の観測は `useActiveScreenPublisher.js` が
`window.__thActiveScreenPublishes` を使っているのと同じ形で、
`window.__thJogPublishes` に積んで数える。

### 8. Gazebo シナリオ

**無し。**このパケットは WebUI 単体で閉じる（ROS ノードを 1 つも触らない）。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部**（`ros2 topic hz /ui/jog_lease` と `ros2 topic echo /cmd_vel_manual_raw`） | 無し |

```bash
# コンテナ内 (/root/th_ws)
timeout 10 ros2 topic hz /ui/jog_lease        # スティックを触りながら。5 Hz 以上
timeout 10 ros2 topic hz /cmd_vel_manual_raw  # 10 Hz 前後
```

### 10. 完了条件

```bash
# ホスト（リポジトリルート）
cd th_ws/web_ui
npm run build                 # 本番ビルドが通ること（オフライン要件の検証は本番ビルドでないと意味がない）
npm run test:unit             # node --test。stickGeometry / speed presets
npx playwright test           # 上の e2e 5 本を含む

# ① 走行タブからの送出口が 1 箇所に閉じている（4.2）
#    publish しているのは useJogLease.js だけ。useRosbridge.js は旧 App.jsx
#    経路（WP-SAFE-04 の O-6 で publish 先だけ差し替えた孤児。§11 c2）なので
#    数から除く。topics.js は定数、names.json は辞書。
test -f src/ros/useJogLease.js
test "$(grep -rl "CMD_VEL_MANUAL_RAW" src/parts/ src/screens/ src/ros/ | wc -l)" -eq 2  # useJogLease.js と topics.js

# ② 数値リテラルを JSX に書いていない（R2）
test -f src/generated/speed_presets.json
! grep -rn "JOG_LIN_MAX\|JOG_ANG_MAX" src/parts/ src/screens/ src/ros/

# ③ ui.jog.release というトリガを送っていない
#    **コメント中の言及に当たるので `jog.release` で grep してはいけない**（V3）。
#    実際に送るなら文字列リテラルになるので、そちらを見る。
! grep -rn "'ui\.jog" src/

# ④ topics.js の全トピック名が名前辞書にある（既存テスト）
node --test test/unit/topics-in-dictionary.test.js
```

### 11. 既知の負債・未確定 (c) と、その扱い

| # | 内容 | 扱い |
| --- | --- | --- |
| c1 | `zones.py` の `SCREEN_ZONE` / `SCREEN_SPEED_LIMIT` は**辞書アクセスで `KeyError`**（`.get()` ではない）。UI が `names.json` に無い画面 ID を送ると `state_manager` が落ちる | S-10〜S-13 は両表に既存なのでこのパケットでは顕在化しない。**新しい画面 ID を足すときは先に `zones.py` へ足すこと**を [open](DetailedDesign-open.md) に登録する |
| c2 | `App.jsx` の旧ジョグ経路（`useRosbridge.js` の `publishManualCmd`）が残る | `WP-SAFE-04` で publish 先を `/cmd_vel_manual_raw` へ変えてある。`App.jsx` は `main.jsx` から到達できない孤児なので実害は無い。削除は `WP-TRANSIT-01` 完了後に別途判断する |
| c3 | `CandidateRadar` が無いので `kind='follow'` の中身が空 | `WP-UI-04` の範囲。差し込み口だけ空けておく |
| c4 | `parts/` は `U-4` の機械検査 3 ディレクトリ（`shell`/`parts`/`ros`）に含まれる。**日本語は `i18n/` にだけ置く** | `WP-UI-02` が `screens/` にも同じ意図を適用済み。それに揃える |
| **c5** | **設計時に詰めていなかった: W-6 を開く画面がこの段階に存在しない**（「手動」ボタンを置くのは S-14 / S-15 / S-16 / S-20 / S-21 で、いずれも後のパケット）。開閉の持ち主と e2e の入口が決まらない | **実装で次のとおり決めた**: 開閉状態は `AppShell` が持ち、`shell/jogPanel.js` の context で画面へ配る（`shell/confirmWindow.js` と同じ流儀）。パネルの中身は `parts/JogConsole.jsx`（スティック＋速度プリセット）で、常設側と W-6 で**同一の部品**を使う（§6.3「常設するものと同じ大きさ」を構造で保証）。e2e の入口は `main.jsx` の `__thTestScreen === 'DRIVE_S11'`（`WP-UI-02` が S-00 / S-01 に使ったフックと同じ） |
| **c6** | **`VirtualStick` の当たり判定が要素の幅と高さを別々に基準にしていた**（`App.jsx` L149-237 から移設した計算そのもの）。`svg` は `viewBox` が 1:1・`preserveAspectRatio` 既定なので、要素が正方形でないと**描画された円と操作範囲がずれる** | **このパケットで直した**（短辺基準に統一）。旧 UI では CSS が箱を正方形に保っていたため露見していなかったが、走行タブでは `.stick` が横に伸びて実測 1129×230 になり、**円の縁まで倒しても倒し量 0.08 < デッドゾーン 0.15 で完全に無反応**だった（2026-09-01 実測）。e2e の `downOnStick()` も短辺に対する割合で倒し量を与える |

### 12. 依存 WP / 被依存 WP

| | |
| --- | --- |
| 依存 WP | `WP-UI-01`（シェル）／ **`WP-SAFE-04`**（`/cmd_vel_manual_raw` の受け手。無いと誰も購読しない） |
| 被依存 WP | `WP-TRANSIT-01`（S-11）／ `WP-UI-04`（S-10）／ `WP-UI-05`（S-14）／ `WP-UI-06`・`WP-UI-07`（W-6 を使う） |

---

## WP-TRANSIT-01 `MANUAL`（S-11 手動走行）

### 0. 一行要旨

**走行タブを差し込んだ画面を 1 枚作り、メインメニューから到達できるようにする。
挙動ノードは作らない。**

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `screens/S11Manual.jsx` | **`th_transit` の `MANUAL` 用ノード**（[transit](DetailedDesign-transit.md) §2.1「作らない」） |
| `main.jsx` の画面遷移（`ui.enter_mode` 受理 → S-11） | `th_state` の遷移・ガード（`T-MANUAL-01/02` は実装済み） |
| 障害物警告表示と自動ブレーキトグル | `obstacle_limiter` 本体（`WP-SAFE-03` で完了） |
| e2e 3 本 | `follow_planner` 系 3 ノードの削除（§11 c2） |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [transit](DetailedDesign-transit.md) §2.1・§2.2・§2.3 | **`MANUAL` は挙動ノードを作らない／スティックは走行操作であってジョグ介入ではない／復帰は `ack_only`** |
| [webui](DetailedDesign-webui.md) §8（表）・§4.2 | S-11 の主表示と操作カード（**「停止」だけ**）／「手動」ボタンを置かない理由 |
| [Spec-webui.md](../spec/Spec-webui.md) §3.4・§3.5 | 手動系の走行タブの中身 |
| [state](DetailedDesign-state.md) §4.2（`MANUAL`）・§8.2 | `T-MANUAL-01/02` と `attributes.yaml` の `MANUAL` 行 |
| `docs/plan/spec/mockup/index.html` L620-656 | **S-11 のマークアップの正本** |
| このファイルの `WP-UI-03` | 走行タブの使い方 |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | 用途 |
| --- | --- | --- | --- |
| pub | `/ui/active_screen` | `th_system_msgs/ActiveScreen` | `screen_id = "S-11"`。**既存 `useActiveScreenPublisher.js` がやる。**新しく書かない |
| sub | `/system/state` | `th_system_msgs/SystemState` | `mode` / `state` / `speed_limit` / `auto_brake`。既存 `useSystemState.js` |
| sub | **`/safety/limiter_status`** | `th_system_msgs/LimiterStatus` | 障害物警告の表示（`action` / `nearest_obstacle_m` / `applied_limit_mps`） |
| pub/sub | 走行タブの 2 本 | — | `WP-UI-03` の `useJogLease.js` |

`/safety/limiter_status` は `names.json` の `topics` に既存。`topics.js` に定数を足す。
QoS は publisher 側が **best_effort, depth 1, 20 Hz**（`obstacle_limiter.cpp`）なので合わせる。

#### 3.2 サービス

| トリガ | 経路 | いつ |
| --- | --- | --- |
| `ui.enter_mode`（`arg.mode = "MANUAL"`） | `/system/trigger`（既存 `useTrigger.js`） | S-01 のボタン。**既に実装済み**（`S01Main.jsx` L93） |
| `ui.stop` | 同上 | 操作カードの「停止」（既存 `OperationCard.jsx` が送る） |
| `ui.finish` | 同上 | 左上の「終了」→ S-01 へ戻る |

**`ui.jog.hold` はここでは扱わない**（`useJogLease.js` がトピックで送る）。

#### 3.3 パラメータ

**このパケット固有のものは無い。**速度プリセットは `WP-UI-03` の生成 JSON を使う。

#### 3.4 フレーム

無し。

### 4. 内部設計

#### 4.1 純粋コアの関数シグネチャ一覧

```js
// shell/limits.js に追加（既存ファイル。表示専用ヘルパの置き場）
obstacleWarning(limiterStatus) -> { level: 'none'|'warn'|'stop', distance_m } | null
```

`action` が `"STOP"` なら `stop`、`"CLAMP"` なら `warn`、`"PASS"` なら `none`。
**`nearest_obstacle_m` の数値で UI 側が閾値判定をしない**——閾値は `obstacle_limiter` が持つ。

#### 4.2 画面の責務

`driveTab kind='manual'` を差し込み、その周りに次を置くだけ。

| 区画 | 中身 | 出典 |
| --- | --- | --- |
| 左上 操作バー | 「終了」＋ タブ 1 枚（「走行」） | mockup L622-628 |
| 障害物カード | 警告文 ＋ **自動ブレーキトグル** | mockup L630-637 |
| 後退カード | 「後方は LiDAR の死角。後退速度を制限中」 | mockup L638-641 |
| 操作カード | **「停止」だけ**。`slots={{ stop:true, check:false, run:false, save:false, manual:false }}` | [webui](DetailedDesign-webui.md) §8 |
| 手動操作カード | 走行タブ（スティック＋速度プリセット） | mockup L650-653 |

**「手動」ボタンを置かない**（§4.2）。スティックを常設するので、
**触れた瞬間に走行操作が始まる**のが唯一の入り口。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **T1-1** | **S-11 でスティックに触ると `PAUSE → RUN`**。ジョグ介入（`PAUSE` へ落ちる）ではない | `F-31`。`MANUAL` / `TEACH_MANUAL` は共通遷移 `C-01` の除外で、`T-MANUAL-01`（`override_common: true`・ガード無し）が担当する。**`guards.py` の `_JOG_EXCLUDED_MODES` に `MANUAL` が入っているのはこのため**——バグではない |
| **T1-2** | **リース満了で `RUN → PAUSE`**（`T-MANUAL-02`） | 手を離す操作が届かなくても止まる |
| **T1-3** | **速度上限の権威はリミッタ側**。画面は表示するだけ | [transit](DetailedDesign-transit.md) §2.1 の囲み |
| **T1-4** | **操作カードに「走行」ボタンを置かない** | 走行に入る契機はスティックだけ。ボタンが 2 つあると「触る前に押すのか」が分からなくなる |
| **T1-5** | 自動ブレーキの既定は **OFF**（場外） | `attributes.yaml` の `MANUAL.auto_brake_default: 'off'`。**画面にハードコードせず `attributes.json` から引く**（`U-6`） |

### 5. 表駆動データ

`attributes.yaml` の `MANUAL` 行（`generated/attributes.json` 経由）。
`initial_state: PAUSE` / `run_state: RUN` / `resume: ack_only` / `auto_brake_default: 'off'` / `jog: is_drive`。

### 6. 安全要件

#### 6.1 触れる層

**層に入らない。**画面は指令の発生源だが、通す判定は `jog_gate`、上限は `obstacle_limiter`。

#### 6.2 フェイルセーフ既定

| 事象 | 挙動 |
| --- | --- |
| `/system/state` が stale | スティックを非活性・操作カードを全非活性（`S01Main.jsx` の `disabledAll` と同じ流儀） |
| `/safety/limiter_status` が未受信 | **警告表示を「不明」にする。**「障害物なし」と表示しない |
| W-1 / W-2 が開いた | シェルが上に被せる。スティックは触れない |

#### 6.3 FMEA

| # | 壊れ方 | 起きること |
| --- | --- | --- |
| ① | 画面が `screen_id` を送らない | `derive_limits()` が使用中 0 台と判定 → `speed_limit=stop` → **走れなくなるだけ**。安全側 |
| ② | `slots.run` を出してしまう | `ui.run` が `MANUAL/PAUSE` の遷移表に無いので拒否される。UI に理由ウィンドウが出る。危険側ではないが T1-4 違反 |
| ③ | 自動ブレーキのトグルを `on_locked` のモードでも出す | `MANUAL` だけ `off` なので他モードで誤って無効化できてしまう。**`attributes.json` から引くこと**（T1-5） |

### 7. 単体試験

| テスト | 登録名 | 満たす仕様 |
| --- | --- | --- |
| `obstacle-warning.test.js`（`node --test`） | `npm run test:unit` | `obstacleWarning()` の 3 分岐と未受信 |
| `e2e/s11-stick-enters-run.spec.js` | `npx playwright test` | **T1-1。**スティックを触ると `PAUSE → RUN` |
| `e2e/s11-lease-expiry-stops.spec.js` | 同上 | **T1-2。**送出を止めると `PAUSE` |
| `e2e/s11-ops-card-stop-only.spec.js` | 同上 | **T1-4。**操作カードに「走行」「手動」が無い |
| `e2e/s11-reachable-from-menu.spec.js` | 同上 | S-01 の「手動走行」を押すと S-11 が出る（**今回の目的そのもの**） |
| `e2e/s01-no-scroll-768.spec.js` の対象に S-11 を追加 | 同上 | §9-2 |

### 8. Gazebo シナリオ

`gazebo.launch.py stage:=2`。`state_manager` を実際に通し、
**`ui.enter_mode(MANUAL)` → `/ui/jog_lease` → `RUN` → 送出停止 → `PAUSE`** を 1 往復させる。
既存の `th_ws/src/th_testing/test/fault_injection/` の `enter_manual_mode()` が
この経路を使っているので、そこから呼べる形にする。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| S-01 → S-11 の遷移／`/system/state.speed_limit` が `v_max` になること／`RUN ⇄ PAUSE` | **車輪が回ること**（走行 3 項目の実施） |

```bash
# コンテナ内 (/root/th_ws)。モータ電源断で確認できるところまで
timeout 5 ros2 topic echo /system/state --field speed_limit --once   # v_max
timeout 5 ros2 topic echo /system/state --field state --once          # スティック中は RUN
```

**これが通ると[実機作業キュー](../../実機作業キュー.md) §7 の 6・7・8（故障注入 04 /
`WP-SAFE-03` の障害物停止 / 故障注入 08）が実施可能になる。**

### 10. 完了条件

```bash
# ホスト（リポジトリルート）
cd th_ws/web_ui && npm run build && npm run test:unit && npx playwright test

# ① 操作カードが「停止」だけ（T1-4）
grep -q "stop: true" src/screens/S11Manual.jsx
! grep -n "run: true\|manual: true" src/screens/S11Manual.jsx

# ② 自動ブレーキの既定をハードコードしていない（T1-5）
grep -q "attributes" src/screens/S11Manual.jsx
! grep -n "auto_brake_default *= *['\"]off" src/screens/S11Manual.jsx

# ③ 画面 ID が名前辞書にある
grep -q '"S-11"' src/ros/names.json

# コンテナ内 (/root/th_ws)
colcon build --symlink-install --packages-select th_state th_testing && source install/setup.bash
colcon test --packages-select th_testing --event-handlers console_direct+ --ctest-args -R state_manager
colcon test-result --verbose
```

### 11. 既知の負債・未確定 (c) と、その扱い

| # | 内容 | 扱い |
| --- | --- | --- |
| c1 | `main.jsx` の画面遷移が `useState` 1 個の手組みで、S-00 → S-01 → S-11 しか無い | 15 画面が揃うまでルータを入れない方針は `WP-UI-02` からの継続。S-11 追加時に**遷移先を配列化**して次の画面で増やしやすくする |
| c2 | **`N-19` は解消しない。**`follow_planner.py` / `follow_planner_mapless.py` / `person_predictor.py` の削除は当初 `WP-TRANSIT-01` の範囲とされていたが、追従走行（`WP-TRANSIT-03`）が未着手のまま消すと Gazebo シナリオが全滅する | **`WP-TRANSIT-03` へ繰り下げる。**[open](DetailedDesign-open.md) の N-19 に追記する |
| c3 | `N-12` の「`observe_cone()` の Python / C++ 一本化」も `WP-TRANSIT-01` の作業とされていたが、`th_transit` そのものが未着手 | 同じく `WP-TRANSIT-03` へ繰り下げる。**両実装の食い違いは安全側にしか倒れない**ことは実測済み |
| c4 | 障害物警告の文言（「前方 0.6 m に障害物」）に距離を出すかどうか | mockup は出している。`nearest_obstacle_m` をそのまま表示する。**閾値判定は UI でしない**（4.1） |

### 12. 依存 WP / 被依存 WP

| | |
| --- | --- |
| 依存 WP | **`WP-UI-03`**（走行タブ）／ **`WP-SAFE-04`**（`jog_gate`）／ `WP-STATE-02`（`T-MANUAL-01/02`）／ `WP-SAFE-03`（`/safety/limiter_status`） |
| 被依存 WP | `WP-TRANSIT-02`（S-13 は S-11 ＋ 教示タブ）／ `WP-MAINT-01`（保守モードの開始条件が `IDLE` / `MANUAL`）／ 実機の走行 3 項目 |
