# 測定機能（fk-system）とのやり取り

[DetailedDesign.md](DetailedDesign.md) の詳細。**自律測定（`AUTO_MEASURE`）と、お立会試験の停止命令の間（`AT_PANEL` の `WORKING`・`ADJUST`）に、
測定機能（fk-system。`../fk-system`）と本システムがやり取りする名前と形。**

入力は [Spec-measure.md](../spec/Spec-measure.md) §1〜§7 と [Spec-onsite.md](../spec/Spec-onsite.md) §7.2、遷移は
[Spec-modes.md](../spec/Spec-modes.md) `SM-3.1.1-18`・`SM-3.1.2-062`〜`-063`・`-120`〜`-141`。
**本システムが書き、測定機能が確かめる**（2026-11-04。fk-system の申し送り 16。[Spec-open.md](../spec/Spec-open.md) §8）。
**状態: 測定機能が確かめた**（2026-11-04。fk のユーザー承認。fk は写さずにこの文書へリンクする）。**2026-11-14 に前後の並進を足した**（申し送り 21。`side` の値と断る理由を 1 つずつ足しただけで、欄は変えていない。**fk が確かめた**。2026-11-14。fk `a3caec3`）。**2026-11-15 に `side` の `LEFT`／`RIGHT` を廃止**（F-60。欄は変えず、`interface_version` は "1" のまま）。**§3 の型 5 つだけ `th_system_msgs` に入れた**（2026-11-11。申し送り 20。作業ブランチ `feat/era3-measure` にあり、確認期の試験が終わってから main へマージする）。受け手は未実装（計画書 §6 の 8-g）。

---

## 0. 結論

| 項目 | 決めたこと |
| --- | --- |
| 通り道 | **2 本。**測定機能の機能側（ROS 2 のノード）とは **ROS 2 で直接**。測定機能の画面（ブラウザ）とは、本システムの画面と同じ **rosbridge**（ユーザー決定 2026-11-04） |
| 指示 | サービス **`/measure/command`** だけで受ける。指示の名前は **`meas.*`**（§3.1） |
| 指示の結果 | **受けた指示には、終わりの結果を必ず 1 つ返す**（`/measure/result`。§3.2）。測定機能が待ち続けないため |
| 状態と自己位置 | **`/measure/status`**（§3.3）。モードと状態・自己位置（地図の世代つき）・進行中の指示 |
| 腕が振れる範囲 | サービス **`/measure/check_arm_clearance`**（§3.5） |
| 生存確認 | 測定機能が **`/measure/heartbeat`** を送り続ける（§3.4）。途絶えたら [Spec-measure.md](../spec/Spec-measure.md) §7 の「通信が途絶えた」 |
| 版の照合 | 生存確認と状態の両方に **`interface_version`** を載せ、食い違えば指示を拒否する（§1.2） |

---

## 1. 通り道

### 1.1 測定機能の機能側（ROS 2 で直接）

| 項目 | 決まり |
| --- | --- |
| 置き場所 | 本システムと同じ PC の**別のコンテナ**。`--network host`・**`ROS_DOMAIN_ID` は本システムと同じ（10）** |
| DDS | **測定機能のコンテナは FastDDS を UDP だけにする**（`FASTRTPS_DEFAULT_PROFILES_FILE` に UDP だけのプロファイル。中身は本システムの `th_ws/scripts/vibe_measure/fastdds_udp.xml` と同じ）。**本システムは変えない** |
| 理由 | `th_robot` コンテナの IPC が共有できず、同じホストと見なした FastDDS が共有メモリで送ろうとして、**一覧には出るのに 0 件しか届かない**（`CLAUDE.md`「環境の癖」2026-10-26） |
| 試験のとき | 実機の bringup が動いている PC で試験を回すと DDS で混線する。**本システムにつながない試験は `ROS_DOMAIN_ID` を分ける**（`CLAUDE.md` 2026-09-23） |

### 1.2 型の取り込みと版の照合

| 項目 | 決まり |
| --- | --- |
| 型 | **本システムの `th_ws/src/th_system_msgs` を、本システムのリポジトリから取り込んでビルドする**（写して書き換えない） |
| 版 | `interface_version`（文字列。**初版は `"1"`**）。型・名前・意味を変えたら上げる。**実装して使い始めるまでの案の直しでは上げない**（申し送り 18 の直しも `"1"` のまま）。**ただし型を `th_system_msgs` に入れたあと（2026-11-11）は、直すなら版を上げるか fk に知らせる**（fk は入った時点からその型でビルドする。申し送り 20）。型と §3 の食い違いは `test_measure_interfaces.py` が止める |
| 照合 | ROS 2 Humble には型の照合が無い。古い型でビルドすると、**黙って化けるか、まったく届かない。**そこで本システムは `/measure/heartbeat` の版を見て、**食い違えば生存とみなさず、指示を `version_mismatch` で拒否する** |

### 1.3 測定機能の画面（rosbridge）

本システムの画面と同じ rosbridge（機体の PC のポート 9090）につなぐ。§4。

---

## 2. 名前の一覧

| 名前 | 種別・型 | 向き | QoS・頻度 |
| --- | --- | --- | --- |
| **`/measure/command`** | サービス `MeasureCommand` | 測定機能 → 本システム | 指示のたび |
| **`/measure/result`** | トピック `MeasureResult` | 本システム → 測定機能 | reliable, depth 20。結果が出るたび |
| **`/measure/status`** | トピック `MeasureStatus` | 本システム → 測定機能 | reliable, **transient_local**, depth 1。**10 Hz ＋ 変化時**。モードに関係なく常に出す |
| **`/measure/heartbeat`** | トピック `MeasureHeartbeat` | 測定機能 → 本システム | reliable, depth 1。**5 Hz**。測定機能が動いている間ずっと |
| **`/measure/check_arm_clearance`** | サービス `CheckArmClearance` | 測定機能 → 本システム | 問い合わせのたび |
| `/map` | 既存（`nav_msgs/OccupancyGrid`） | 本システム → 測定機能 | 既存のまま。自律測定の作りかけの地図 |

画面側（rosbridge）の名前は §4。

---

## 3. 型

### 3.1 `MeasureCommand.srv`（指示）

```
string  command_id        # 測定機能が振る。結果（§3.2）との突き合わせに使う。空は拒否
string  command           # 下の表の meas.*
string  map_instance_id   # meas.goal だけ。/measure/status の map_instance_id と同じでなければ拒否
float64 x                 # meas.goal: map フレーム [m]
float64 y
float64 yaw               # meas.goal: map フレーム [rad]
float64 angle_rad         # meas.turn: 正＝左回り（反時計回り）[rad]
string  side              # meas.translate: FORWARD / BACKWARD（LEFT / RIGHT は 2026-11-15 に廃止。bad_argument）
float64 distance_m        # meas.translate [m]
float32 v_max_mps         # meas.set_limits: 0 は指定しない（本システムの上限のまま）
float32 w_max_radps
float32 a_max_mps2
float32 alpha_max_radps2
---
bool    accepted
string  reject_reason     # 理由キー（下の表）。受けたときは空
string  mode              # 応答した時点のモードと状態
string  state
```

| `command` | Spec の名前 | 使う欄 | 受ける状態（遷移） | 終わりの結果 |
| --- | --- | --- | --- | --- |
| `meas.goal` | 目的地 | `map_instance_id`・`x`・`y`・`yaw` | `AUTO_MEASURE`/`WAIT`（-121） | `ARRIVED`（-124。向きまで合って到着） |
| `meas.reroute` | 経路の再計算 | — | `AUTO_MEASURE`/`NAV`・`BLOCKED`（-122） | **すぐ `DONE`**。目的地はもとの `command_id` のまま続く |
| `meas.start` | 発進 | — | `AUTO_MEASURE`/`PAUSE`（-130） | **すぐ `DONE`**。止める前の指示はもとの `command_id` のまま続く |
| `meas.stop` | 停止 | — | `AUTO_MEASURE`/`NAV`・`ALIGN`・`BLOCKED`・`ADJUST`（-129） | すぐ `DONE`。`ADJUST` の指示は `CANCELLED` |
| `meas.abort` | 中止 | — | `AUTO_MEASURE`/`NAV`・`ALIGN`・`BLOCKED`・`ADJUST`・`PAUSE`（-131） | すぐ `DONE`。進行中の指示は `CANCELLED` |
| `meas.finish` | 終了 | — | `AUTO_MEASURE`/任意（-135） | すぐ `DONE`。進行中の指示は `CANCELLED` |
| `meas.turn` | 旋回 | `angle_rad` | `AUTO_MEASURE`/`WAIT`（-127）・`AT_PANEL`/`WORKING`（-136） | `DONE`（-128・-137） |
| `meas.translate` | **並進（前後）**（2026-11-14。申し送り 21） | `side`（`FORWARD`／`BACKWARD`）・`distance_m`（正の値。向きを変えずにまっすぐ） | `AUTO_MEASURE`/`WAIT`（-127）・**`AT_PANEL`/`WORKING`（-136）** | `DONE`（-128・-137）。**`LEFT`／`RIGHT` はどのモードでも `bad_argument`**（左右の並進は 2026-11-15 に廃止。F-60） |
| `meas.set_limits` | 速度・加速度の上限 | `v_max_mps` ほか | `AUTO_MEASURE`/任意・`AT_PANEL`/`WORKING`・`ADJUST` | すぐ `DONE`。**本システムの上限との小さい方だけが効く。**自律測定はモードを抜けたら、お立会試験は**停止命令が解けたら**（`WORKING`・`ADJUST` を出たら）捨てる（人のジョグに効き続けないため） |
| `meas.done` | 測定を終えた | — | `AT_PANEL`/`WORKING`（-063） | すぐ `DONE`（停止命令が解ける） |

| `reject_reason` | 意味 |
| --- | --- |
| `state_not_accepting` | いまのモード・状態では受けない（-132・-138 ほか） |
| `turning` | 旋回中は受けない（-138） |
| `corner_obstacle` | 角が描く円の中に障害物がある（-121・-127・-130・-136。[Spec-measure.md](../spec/Spec-measure.md) §4.4） |
| `path_obstacle` | **前後の並進で、進む先（指示の距離だけ進むまでに機体の形 0.55 m 角が通る範囲）に障害物がある**（-127・-136。[Spec-measure.md](../spec/Spec-measure.md) §5.3。2026-11-14） |
| `map_mismatch` | `map_instance_id` が今の地図と違う |
| `version_mismatch` | `interface_version` が食い違う（§1.2） |
| `link_lost` | 生存確認が途絶えている |
| `localization_lost` | 自己位置を見失ったあと、自律測定を抜けて入り直すまで（-142） |
| `bad_argument` | 欄が足りない・値の範囲外・`command_id` が空・`meas.translate` の `side` が `FORWARD`／`BACKWARD` 以外 |

**`ui.*` を `/measure/command` に送っても拒否する**（§5）。

### 3.2 `MeasureResult.msg`（指示の終わりの結果）

```
std_msgs/Header header
string command_id
string command
string result      # ARRIVED / DONE / CANCELLED（FAILED は予約。いまは出さない）
string reason      # CANCELLED の理由キー。ほかは空
```

**`accepted=true` で受けた指示には、`/measure/result` を必ず 1 回だけ出す。**次の場合もすべて出す（どれかが漏れると、測定機能は永遠に待つ）。

| 起きたこと | 進行中の指示の結果 | `reason` |
| --- | --- | --- |
| 測定機能の停止（`ADJUST` から）・中止・終了 | `CANCELLED` | `stopped` ／ `aborted` ／ `finished` |
| 人が抜けた（「自律測定を終了」「終了」・`SM-3.1.1-18`） | `CANCELLED` | `mode_exit` |
| `ADJUST`（旋回・並進）の最中の非常停止・手押し（`ESTOP`・`CARRY`） | `CANCELLED`（戻っても回り直さない。-130・-140） | `estop` ／ `carry` |
| `ADJUST` の最中の回復フォルト・通信途絶（-03・-133・-139） | `CANCELLED`（同上） | `fault` ／ `link_lost` |
| **自律測定で自己位置を見失った（`LOCALIZATION_LOST`。-142）** | **`CANCELLED`（目的地も旋回・並進も）** | **`localization_lost`** |

**目的地を持ったまま止まる場合は、結果を出さない。**自律測定の `NAV`・`ALIGN`・`BLOCKED` の最中に、回復フォルト・人のジョグ・通信途絶で `PAUSE` に落ちたときと、**非常停止・手押しで `ESTOP`・`CARRY` に入ったとき**がこれに当たる（自己位置を見失った `LOCALIZATION_LOST` は除く。上の表）。戻る先は `PAUSE` で（[Spec-modes.md](../spec/Spec-modes.md) §6 の `AUTO_MEASURE`）、測定機能の発進（`meas.start`。-130）でもとの `command_id` のまま続くため。目的地を捨てたとき（中止・終了・人が抜けた）に `CANCELLED` を出す。

### 3.3 `MeasureStatus.msg`（状態と自己位置）

```
std_msgs/Header header
string  interface_version
string  mode                 # SystemState と同じ文字列
string  state
bool    working              # 停止命令の間
bool    link_ok              # 本システムから見た測定機能の生存（§3.4）
bool    pose_valid           # 自己位置が使えるか
string  map_instance_id      # 地図の生存世代（MapSessionStatus.instance_id と同じ意味）
float64 x                    # map フレーム [m]
float64 y
float64 yaw                  # [rad]
string  pending_command_id   # 進行中の指示（無ければ空）
string  pending_command
string  pause_reason         # SystemState.pause_reason と同じ
bool    blocked              # 塞がれて止まっている（BLOCKED）
bool    reentry_required     # 自己位置を見失ったあと、自律測定を抜けて入り直すまで true（-142）
float32 v_limit_mps          # いま効いている速度の上限（本システムと測定機能の指定の小さい方）
float32 w_limit_radps
```

| 項目 | 決まり |
| --- | --- |
| 自己位置 | **自律測定は、そのとき作っている地図の座標。お立会試験は会場地図の座標**（[Spec-measure.md](../spec/Spec-measure.md) §3） |
| 地図の世代 | **自律測定は入るたびに地図を作り直す。**測定機能は目的地に `map_instance_id` を付け、本システムは今の地図と違えば `map_mismatch` で拒否する（前の地図の座標で走り出さない） |
| 旋回の出し直し | 旋回が途中で止まったら、測定機能はこの `yaw` から残りの角度を出す（申し送り 14） |
| 入り直しの要否 | `reentry_required` が true の間、目的地・旋回・並進・発進は `localization_lost` で拒否される。**測定機能が取り消しの結果を取りこぼしても、これで分かる**（申し送り 18） |

### 3.4 `MeasureHeartbeat.msg`（生存確認）

```
std_msgs/Header header
string interface_version
string sender            # "fk-system"
uint32 seq
```

| 項目 | 決まり |
| --- | --- |
| 送り方 | 測定機能の機能側が **5 Hz で送り続ける**（モードに関係なく） |
| 途絶 | 最後の受信から **`measure_link_timeout_ms`**（(b)。値は実装時。[Spec-params.md](../spec/Spec-params.md)）で途絶とみなす。版が食い違うものは受信に数えない |
| 使う所 | **モードに関係なく常に見る。**`IDLE` では入口の判定（-120）、自律測定では `PAUSE` への落とし（-133）、停止命令の間は停止命令のまま（-134）と旋回の打ち切り（-139） |

### 3.5 `CheckArmClearance.srv`（腕が振れる範囲）

```
geometry_msgs/Polygon region   # 回ったあとの base_link 基準。x 前・y 左 [m]。z は見ない
string[] sides                 # LEFT / RIGHT / CURRENT のいくつか
---
bool     accepted
string   reject_reason         # state_not_accepting / bad_argument
string[] sides                 # 問い合わせと同じ順
string[] results               # CLEAR / BLOCKED / UNKNOWN
string[] unknown_reasons       # UNKNOWN のときだけ: scan_stale / blind_sector
builtin_interfaces/Time checked_at
```

| 項目 | 決まり |
| --- | --- |
| 左右 | `LEFT` は `region` を機体の中心まわりに **+90°**、`RIGHT` は **−90°** 回した位置を、**回る前の今の位置から**確かめる。`CURRENT` は回さない |
| 機体の内側 | **`region` のうち機体の形（0.55 m 角）の内側は見ない。**腕の取り付け部のそばには死角マスクのかかったパイプがあり、除かないと毎回 `UNKNOWN` になる |
| 範囲の形（測定機能の決定。暫定） | 回ったあとの機体基準で**前に伸びる長方形: x 0〜1.0 m・y −0.275〜+0.275 m**（腕を起こす・格納するとき、腕の先が機体の中心から約 1 m。幅は腕の取り付けが決まるまで機体の幅いっぱい。余裕 0）。**2026-11-14 の知らせ（申し送り 21）: 腕が伸縮しなくなり、長さは約 1.76 m に伸びる見込み（暫定）。**範囲は測定機能が渡すので、本システムの受け口は変わらない。**値は測定機能のパラメータ。**機体の内側を見ないので、確かめるのは前面（x 0.275 m）から先 |
| 見るもの | 角が描く円の確認と同じ入力（障害物マップかスキャン）。[Spec-measure.md](../spec/Spec-measure.md) §5.2 |
| `UNKNOWN` | 最後のスキャンが **`arm_clearance_scan_max_age_ms`** より古い（`scan_stale`）か、範囲が LiDAR の死角にかかる（`blind_sector`）。**測定機能は塞がっているのと同じに扱う** |
| 受ける状態 | `AUTO_MEASURE`/`WAIT` と `AT_PANEL`/`WORKING` だけ（-141）。**状態は変えない** |
| 意味 | `CLEAR` は**床から 32 cm の走査面に物が無い**という意味だけ（[Spec-measure.md](../spec/Spec-measure.md) §5.2） |

---

## 4. 測定機能の画面（rosbridge）

**測定機能の画面は、本システムの画面と同じ扱いの端末になる。**送るのは人の操作に当たるものだけ。

| 名前 | 向き | 決まり |
| --- | --- | --- |
| `/ui/active_screen`（`ActiveScreen`） | 画面 → 本システム | **在席と画面の申告。**`screen_id` は、自律測定の間は **`S-60`**、お立会試験の停止命令の間は **`S-21`**（どちらもゾーン `IN`）。**`client_id` は `fk-` で始める。**`interacting`・`last_input`・2 Hz は本システムの画面と同じ（[names](DetailedDesign-names.md) §5.1）。測定機能は `/measure/status` の `mode` と `working` を見て `screen_id` を切り替える（fk の決定）。新しい画面 ID は作らない（ゾーンの表を 3 か所直さずに済む） |
| `/safety/estop_ui`（`std_msgs/Bool`） | 画面 → 本システム | ヘッダーの非常停止。**押している間 2 Hz で `true` を送り続け、解除は `false` を 1 回**（[safety](DetailedDesign-safety.md) §6.3。押下側にラッチ） |
| `/system/trigger`（`UiTrigger`） | 画面 → 本システム | **ヘッダーと異常ウィンドウへの人の応答だけ**（`ui.resume_ack` など。[names](DetailedDesign-names.md) §8.1） |
| `/measure/command`（`MeasureCommand`） | 画面 → 本システム | 測定機能の画面の**「走行」**（人のジョグで止まったあとの発進。`meas.start`）など、人の操作を測定機能の指示として送るとき。**測定機能の決定: 機能側を通して送り、画面から直接は呼ばない**（発進の前に昇降機が下端にあることを機能側で確かめるため。機能側が落ちても、本システムは生存確認の途絶で `PAUSE` に落ちる）。送るのは `meas.*` だけ |
| `/system/state`・`/safety/active_faults`・`/safety/fault`・`/measure/status` | 本システム → 画面 | ヘッダーと異常ウィンドウの中身。**本システムの画面と同じトピックを購読し、理由キーの表（`th_ws/web_ui/src/i18n/`）を共有する。**表は型と同じく本システムのリポジトリから取り込み、写さない（fk の決定）。足りなければ相談（§7） |

---

## 5. 入口の分離（安全境界）

[names](DetailedDesign-names.md) §8 の「名前空間の分離はそれ自体が安全境界」に、入口を 1 本足す。

| 決まり | 理由 |
| --- | --- |
| **`meas.*` は `/measure/command` からしか受けない。**`/system/trigger` に来た `meas.*` と、`/measure/command` に来た `ui.*` は拒否する | 測定機能の指示と人の操作を取り違えない |
| **測定機能の機能側は `ui.*` を送らない。**送ってよいのは測定機能の画面で人が押したものだけ（§4） | 非常停止の解除と異常への応答は人が行う（[Spec-measure.md](../spec/Spec-measure.md) §2） |
| `meas.*` を受けるのは `AUTO_MEASURE` と `AT_PANEL` の `WORKING`・`ADJUST` だけ | それ以外のモードで測定機能が機体を動かさない |

---

## 6. 本システムの中の受け手（2026-11-23 決定。8-g）

| 役割 | 受け手 |
| --- | --- |
| `/measure/command` の提供・`meas.*` の受理・`/measure/result` の発行 | **`th_state` の `state_manager`**（`/system/trigger` と同じく状態機械が受ける。遷移と結果の取りこぼしを 1 か所で防ぐ）。**結果を 1 回だけ出す台帳**は純コア `th_state/measure_ledger.py` |
| 生存確認の監視・`/measure/status` | **`th_measure` の `measure_link`**（§6.1） |
| `/measure/check_arm_clearance` | **`th_measure` の `measure_clearance`**（2026-11-23。P4。角が描く円の確認と同じ入力を使うため同じノード。§6.5） |
| 自律測定の目的地への走行（地図を作りながら・手前 1.5 m の検査） | **`th_onsite` の `venue_navigator` を広げて使う**（2026-11-23 変更。試験場内の走行と同じ決まり（§6.3 の塞がれたら待つ・§6.1.1 の機体の形・向き合わせ）と、実機で直した Nav2 の誤報・TF 停滞・衝突の打ち切りへの対策をそのまま使うため。新しいノードは作らない）。Nav2 を使い、速度指令は `/cmd_vel_nav`（[names](DetailedDesign-names.md) §6.1 の不変ルールのまま） |
| 角が描く円の中の障害物（発進前の確認。§4.4） | **`th_measure` の `measure_clearance`**（2026-11-23。P5b）。P4 の `/measure/check_arm_clearance` も同じノードに置く |
| 盤前の旋回・前後の並進（`ADJUST`。自律測定とお立会試験の停止命令の間） | **Nav2 を使わない直接制御**（2026-11-15。申し送り 22。F-60）。`th_measure` の挙動ノードが自己位置を見ながら超信地旋回・直進し、**速度指令は `/cmd_vel_behavior` だけ**（`venue_navigator` の `ALIGN` と同じ作り。[onsite](DetailedDesign-onsite.md) §6）。twist_mux → `obstacle_limiter` を通るので障害物の停止・速度の制限・非常停止はそのまま効く。直進の帰還に使う位置は P3 で決める |

### 6.1 `measure_link` と `state_manager` の分担

| 項目 | 決まり |
| --- | --- |
| 生存の判定 | `measure_link` が `/measure/heartbeat` を受け、**`interface_version` が `"1"` と一致するものだけ**を受信に数える。最後に数えた受信から **`measure_link_timeout_ms`（(b)。初期値 1000。5 Hz の 5 回分）** で途絶 |
| `state_manager` への渡し方 | **`/system/event` の `evt.measure_link_ok` ／ `evt.measure_link_lost`**（`arg_json` に `{"reason": "timeout"｜"version_mismatch"｜"never"}`）。**変わったときと、1 Hz で今の値を出し直す**（取りこぼしても 1 秒で揃う）。`state_manager` は最後の値をラッチし、**3 秒どちらも来なければ途絶とみなす**（`measure_link` が落ちたとき） |
| 拒否の理由 | 途絶の理由が `version_mismatch` なら `version_mismatch`、ほかは `link_lost` |
| `state_manager` から `measure_link` へ | 内部トピック **`/measure/command_state`**（`std_msgs/String` の JSON `{"pending_command_id","pending_command","reentry_required","v_cap_mps","w_cap_radps"}`。reliable・transient_local・depth 1。変わったとき）。`/measure/status` の対応する欄の材料。`v_cap`・`w_cap` は `meas.set_limits` の指定（無ければ 0）で、P1 では常に 0 |
| `/measure/status` の材料 | `/system/state`（`mode`・`state`・`working`・`pause_reason`・`speed_limit`）、TF `map→base_link`（`pose_valid` は 0.5 s 以内に取れたとき）、`/map_session/status` の `instance_id`、自分の生存判定、`/measure/command_state` |
| 速度の上限の欄 | `speed_limit`（registry の名前か `stop`）を数値に引き、`v_cap` が正ならその小さい方。`stop` は 0 |
| `/measure/command` の判定の順 | ① `command` が `meas.` で始まらない（`ui.*` を含む）・`command_id` が空・未知の `meas.*`・`meas.translate` の `side` が `FORWARD`／`BACKWARD` 以外 → `bad_argument` ② 版の食い違い → `version_mismatch` ③ 途絶 → `link_lost` ④ 遷移表に行が無い・ガードで落ちる → `state_not_accepting`（ガードが理由キーを持つならそれ） |

### 6.2 自律測定（`AUTO_MEASURE`）の作り（2026-11-23 決定。計画書 8-g の P5）

**P5a（状態機械・地図・凍結）→ P5a-ui（画面）→ P5b（目的地への走行）**の順に入れる。P5a のあいだ、目的地（`meas.goal`）・旋回・並進・腕の範囲の確認は `state_not_accepting` のまま。**状態は入口ができるパケットで足す**（P5a は `WAIT`・`PAUSE`、P5b で `NAV`・`ALIGN`・`BLOCKED`、P3 で `ADJUST`。入口の無い状態は遷移表の検証が到達不能として咎めるため。2026-11-23）。それらの状態から出る行（停止・途絶など）も、その状態を足すパケットで足す。

| 項目 | 決まり |
| --- | --- |
| 入口（-120） | 既存の `ui.goto`（`kind: AUTO_MEASURE`）。許可のダイアログ（W-4）は**画面側**で出し、「許可する」で `ui.goto` を送る（未保存の W-4 と同じ流儀。状態機械はダイアログを持たない）。ガードは `mode_entry.yaml` の許可表に `AUTO_MEASURE` を足し、**測定機能の生存（`/measure/command` の③と同じ判定）が無ければ拒否**（理由キー `measure_link_lost`／`measure_version_mismatch`） |
| 抜ける（-10・-18・-135） | 「自律測定を終了」は既存の「終了」の口（`IDLE` へ）。**腕の格納の確認（W-4）は画面側**で先に出す（状態機械は持たない）。`meas.finish` は `/measure/command` から同じ遷移へ |
| 地図（-120・§4.1） | 入ったときの effect **`start_measure_map`**（dest `map_session`）。`slam_control` は会場地図・経路地図のファイルに触れず、**`slam_toolbox` をまっさらな地図作成にする**（`/map_session/status` の `instance_id` が新しくなる。`dirty` は立てない＝未保存に数えず保存の確認を出さない）。抜けたあと試験場内で動くなら会場地図を開き直す（`SM-3.1.2-119` が拒否する） |
| 凍結（§4.1） | `map_freezer` の通すモードに `AUTO_MEASURE` を足す（`ESTOP`・`CARRY` の押下前が `AUTO_MEASURE` のときも通す。既存の `effective_mode` の流儀） |
| 指示の遷移 | P5a: `meas.stop`（-129。`WAIT` では受けない）・`meas.start`（-130。`PAUSE` から止める前の状態へ。P5a では止める前は `WAIT` だけなので `WAIT`）・`meas.abort`（-131）・`meas.finish`（-135）・`meas.set_limits`（状態は変えない）。P5b: `meas.goal`・`meas.reroute` と `NAV`／`ALIGN`／`BLOCKED` |
| 通信の途絶（-133） | `state_manager` が `evt.measure_link_lost` を**変化したときだけ**遷移表へ流す（1 Hz の出し直しは流さない）。`NAV`・`ALIGN`・`BLOCKED`・`ADJUST` から `PAUSE`（`pause_reason` は `measure_link`）。`WAIT`・`PAUSE` は変えない |
| 回復フォルト・ジョグ（-03・-01） | 既存の共通行のまま `PAUSE`。**戻るのは `meas.start` だけ**（`ui.resume_yes` では走らない。`attributes.yaml` の `resume` で表す） |
| 自己位置を見失った（-142） | `LOCALIZATION_LOST` で `ESTOP`（既存）。そのとき進行中の指示を `CANCELLED`（`localization_lost`）にし、**`reentry_required` を立てる**。モードを抜けたら下ろす。立っている間、`meas.goal`・`turn`・`translate`・`start` は `localization_lost` で拒否 |
| 結果の台帳 | **受理の判定（遷移表）→ 台帳への記録 → 遷移の実行**の順にする（P1 は遷移を先に進めていた）。モードを抜けたら残りを `CANCELLED`（`mode_exit`／`finished`）。`ESTOP`・`CARRY`・`PAUSE` へ落ちても目的地は捨てない（§3.2） |
| ゾーン・速度 | 画面 `S-60` はゾーン `IN`。`attributes.yaml`: 速度 `v_slow`、ジョグ可（`v_jog_panel`）、自動ブレーキ ON（固定） |
| 在席 | 測定機能の画面（`client_id` が `fk-`、`screen_id` `S-60`）は本システムの画面と同じに数える（既存の `/ui/active_screen` の仕組みのまま。足すものが無いことを試験で確かめる） |

**P5b（目的地への走行）の決まり**（2026-11-23）。

| 項目 | 決まり |
| --- | --- |
| 状態 | `NAV`・`ALIGN`・`BLOCKED` を足す（`safety_monitor` の状態の一覧も）。`attributes.yaml` の `run_state` を `NAV` にする（在席を失ったら `PAUSE`・非常停止から戻る先が `PAUSE` になる）。`ALIGN`・`BLOCKED` も非常停止・手押しから戻る先を `PAUSE` にする（`ESTOP_RESUME_PAUSE_EXTRA`。2026-10-22 の事故と同じ理由）。`ui.resume_yes` では走らないのは P5a のまま |
| 指示の引数 | `state_manager` は `/measure/command` の欄（`x`・`y`・`yaw`・`map_instance_id` ほか）と `command_id` を遷移表の `arg` に渡す |
| 目的地（-121） | `WAIT` の `meas.goal` だけ受ける。判定の順: `reentry_required` → `localization_lost`、`map_instance_id` が `/map_session/status` の `instance_id` と違う → `map_mismatch`、角が描く円の中に障害物 → `corner_obstacle`。受けたら `NAV` へ、effect **`measure_goal`**（dest `venue_navigator`。`command_id`・`x`・`y`・`yaw`）。`NAV`・`ALIGN`・`BLOCKED`・`PAUSE` では `state_not_accepting`（-132） |
| 角が描く円の確認 | `measure_clearance` が `/scan_filtered` と局所コストマップの占有セルから、`base_link` 中心の半径 `corner_clear_radius_m`（0.689 m。§6.8。2026-11-25 に 0.389 m から改めた。コストマップは致死のセルだけ）の円の中に障害物があるかを **5 Hz** で `/measure/corner_status`（`std_msgs/String` の JSON `{"clear": bool, "reason": "..."}`）に出す。スキャンが `arm_clearance_scan_max_age_ms` より古い・TF が取れないときは `clear=false`。`state_manager` は最後の値をラッチし、**1 秒届かなければ `clear=false`**。ガードは同期で判定する（サービスを呼ばない） |
| 走行 | `venue_navigator` は `AUTO_MEASURE` の `NAV` を走行の対象に入れ、目的地は effect `measure_goal` から取る（ピンは使わない）。**会場地図の有無の検査（`SM-3.1.2-119`）は `AUTO_MEASURE` では見ない**（`start_measure_map` で会場地図は外れている）。着いたら `evt.arrived` → `ALIGN` → `evt.align_done` → `WAIT` は試験場内と同じ |
| 手前 1.5 m の検査（-125） | 試験場内と同じ検査（機体の形 0.55 m 角・0.5 秒周期・走り出す前と走行中）に、**`AUTO_MEASURE` だけ「地図ができている」**を足す: 1.5 m の範囲の `/map` のセルが不明（-1）なら通れない扱いで `BLOCKED`。その先の不明な所は Nav2 の `allow_unknown` のまま推測の経路で通す。`BLOCKED` は時間切れにならず、空けば `NAV` へ戻る（-126） |
| 経路の再計算（-122） | `NAV`・`BLOCKED` の `meas.reroute` を受け、試験場内の「再検索」と同じ引き直しをする。すぐ `DONE`、目的地はもとの `command_id` のまま |
| 停止（-129） | `NAV`・`ALIGN`・`BLOCKED` の `meas.stop` → `PAUSE`。目的地は持ったまま（結果を出さない）。止める前の状態を `state_manager` が覚える（人のジョグ・回復フォルト・通信途絶で `PAUSE` に落ちたときも同じ） |
| 発進（-130） | `PAUSE` の `meas.start`: 止める前が `NAV`・`BLOCKED` なら `NAV`（走り出す前に手前の検査をし、通れなければ `BLOCKED`）、`ALIGN` なら `ALIGN`、それ以外は `WAIT`。`NAV`・`ALIGN` へ戻るときは角が描く円を確かめ、障害物があれば `corner_obstacle`（`PAUSE` のまま）。`reentry_required` 中は `localization_lost` |
| 中止（-131） | `NAV`・`ALIGN`・`BLOCKED`・`PAUSE` の `meas.abort` → `WAIT`。走行を止め、目的地は `CANCELLED`（`aborted`） |
| 途絶（-133） | `NAV`・`ALIGN`・`BLOCKED` → `PAUSE`（`pause_reason` `measure_link`）。目的地は持ったまま |
| 到着（-124） | `ALIGN` → `WAIT` の遷移で、目的地の `command_id` に `ARRIVED` を**ちょうど 1 回** |
| 非常停止・手押し | `NAV`・`ALIGN`・`BLOCKED` から入って戻ると `PAUSE`（走り出さない）。目的地は持ったまま。`LOCALIZATION_LOST` だけは P5a のとおり `CANCELLED`（`localization_lost`） |
| 速度・加速度の上限（`meas.set_limits`） | **P5c（§6.6）で受けるようにした（2026-11-24）。以下は P5b の時点の決まり**: 効かせる仕組み（`obstacle_limiter` が上限を下げる）ができるまで受けない（`state_not_accepting`）。P5a で入れた受ける行は外し、`/measure/status` の `v_limit_mps` も `v_cap` を混ぜない（効いていない上限を報告しない）。効かせるのは別のパケット（P5c） |

**P5a-ui（画面）の決まり**（2026-11-23）。正は [Spec-webui.md](../spec/Spec-webui.md) §3.2・§3.16 と [Spec-modes.md](../spec/Spec-modes.md) `SM-3.1.1-18`。

| 項目 | 決まり |
| --- | --- |
| S-01「自律測定」 | 「測定」の群に置く。押すと W-4「自律動作を許可しますか」（「許可しない」／「許可する」）。「許可する」で `ui.goto`（`kind: AUTO_MEASURE`）。**通信できないときは押せるが W-4 を出さずに理由を出す**。判定は `/measure/status` の `link_ok`（未受信は通信できない扱い）。状態機械が拒否したときも理由キー（`measure_link_lost`／`measure_version_mismatch`）を出す |
| S-60 | `AUTO_MEASURE` の間の画面。ゾーン `IN`（`/ui/active_screen` の `screen_id` は `S-60`）。置くのは「測定機能の画面へ」「自律測定を終了」「手動」と状態の表示（`WAIT`／`PAUSE`）だけ |
| 「測定機能の画面へ」 | **同じタブで** `http://<画面を開いているホスト>:<measure_ui_port>/` へ移る（測定機能の画面の「終了」は本システムの画面へ戻る。fk `DetailedDesign-names.md` §6）。`measure_ui_port` は registry（given、8091、`consumers: [web_ui]`）。S-50 では変えない |
| 「自律測定を終了」 | 毎回 W-4「測定機構の腕が格納位置にあることを確かめましたか」（「やめる」／「メインメニューへ」）。「メインメニューへ」で `ui.finish` |
| 「手動」 | 既存の手動操作パネル（W-6）。触れると共通行で `PAUSE`。**画面に「走行」は置かない**（戻すのは測定機能の画面の「走行」＝`meas.start`） |
| 腕の格納の確認（-18）を挟む他の抜け道 | `ESTOP`・`CARRY` で `prev_mode` が `AUTO_MEASURE` のとき、W-1 の「メインメニューへ」（`ui.resume_no`）・「確認」（`ui.resume_ack`）と W-2 の「メインメニューへ」（`ui.finish`）の前に同じ W-4 を出す。「やめる」なら送らない。`AT_PANEL` の `WORKING`・`ADJUST` は P2・P3 で同じ部品を使う |

### 6.3 お立会試験の停止命令（`AT_PANEL` の `WORKING`）の作り（2026-11-23 決定。計画書 8-g の P2）

**P2（状態機械）と P2-ui（画面）に分けて同時に進める。**P2-ui は yaml を触らない（下の表の名前と形だけを約束にする）。旋回・並進（`ADJUST`。-136〜-138）は P3、腕の範囲の確認（-141）は P4、`meas.set_limits` は P5c。P2 のあいだ `WORKING` で受ける `meas.*` は `meas.done` だけで、ほかは `state_not_accepting`。正は [Spec-modes.md](../spec/Spec-modes.md) `SM-3.1.2-062`・`-063`・`-134`・`-139`・`-140`・`SM-3.1.1-18` と [Spec-onsite.md](../spec/Spec-onsite.md) §7.2。

| 項目 | 決まり |
| --- | --- |
| 入口（-062） | 既存の `ui.working`（`{"on": true}`。`IDLE_P` → `WORKING`。T-ATP-01）を S-21 の「測定機能へ」が送る。**測定機能の生存はガードにしない**（止まる向きなので。測定機能が落ちていれば人が「停止を解く」で出る） |
| 解く（-063・-134） | 測定機能の `meas.done`（`WORKING` → `IDLE_P`。すぐ `DONE`）と、人の「停止を解く」（既存の `ui.working` `{"on": false}`。T-ATP-02）。**腕の格納の確認（W-4「やめる」／「解く」）は画面側**（状態機械は持たない） |
| ジョグ（-062） | `WORKING` の `ui.jog.hold` は **`override_common` の拒否行**で C-01 より先に落とす（理由キー `working_in_progress`）。`jog_active` は C-01 の `set_jog` でしか立たないので、拒否すれば速度上限は停止のまま。**`/cmd_vel` が 0 のままであることを本物の `state_manager`＋`obstacle_limiter` で縛る** |
| 回復フォルト（-139） | `WORKING` の `fault.recoverable` は **`override_common` の行**で `WORKING` のまま（effect `open_window`。C-03 の `PAUSE` には落とさない）。W-1 の「確認」（`ui.resume_ack`）は `WORKING` で受けて状態を変えない行を足す（ガード `fault_cleared`。effect `close_window`）。測定機能との途絶（`evt.measure_link_lost`）は `WORKING` では遷移しない |
| 非常停止・手押し（-140） | 戻る先は `WORKING`。`ESTOP` の `ui.resume_yes`（C-09c-at-panel）と `CARRY` の `ui.carry_resume`（C-11-at-panel）の前に、`prev_mode` が `AT_PANEL` かつ `prev_state` が `WORKING` のときの行を置く（→ `AT_PANEL`/`WORKING`）。それ以外の `AT_PANEL` は従来どおり `IDLE_P`。`IDLE` へ抜ける行（C-09d・C-09f・C-12）は変えない（腕の確認は画面側） |
| `/system/state.working` | **`ESTOP`・`CARRY` の間も、押下前が `AT_PANEL`/`WORKING` なら `true`**（「回復フォルト・非常停止でも `working` は外れない」。Spec-modes §4.2）。行き先拒否のガード（`is_working`）は今の形のまま（モードと状態で見る）で、出す値だけを別の関数で決める。`/measure/status.working` はこれをそのまま写す（測定機能は `working` で `screen_id` を `S-21` に切り替える） |
| 結果の台帳 | 「測定の続き」を状態で見る: `AUTO_MEASURE` の全体に加えて `AT_PANEL` の `WORKING`（P3 から `ADJUST` も）。そこから `ESTOP`・`CARRY` 以外へ出たら（`meas.done`・「停止を解く」・「終了」を含む）残りを `CANCELLED`（`mode_exit`。`meas.done` なら `finished`） |
| `meas.*` の受け口 | `AT_PANEL` の `WORKING` で `meas.done` を受ける。ほか（`turn`・`translate`・`check` 系・`set_limits`・自律測定の指示）は `state_not_accepting`。`AT_PANEL` の `IDLE_P`・`PAUSE` では従来どおりすべて拒否 |

**P2-ui（画面）の決まり**（2026-11-23）。正は [Spec-webui.md](../spec/Spec-webui.md) §3（S-21 の「測定機能へ」「停止を解く」）と `SM-3.1.1-18`。

| 項目 | 決まり |
| --- | --- |
| 「測定機能へ」 | S-21 の `AT_PANEL`/`IDLE_P` のときだけ押せる（「カメラ」の隣）。押すと `ui.working` `{"on": true}`。**受理されて `/system/state` が `WORKING` になってから**、同じタブで `http://<ホスト>:<measure_ui_port>/` へ移る（S-60 の「測定機能の画面へ」と同じ部品・同じ registry 値） |
| `WORKING` の S-21 | 状態「測定中（停止命令）」。「測定機能の画面へ」（戻るため）と「停止を解く」を出す。行き先・呼び寄せ・ジョグ（W-6）は押せなくし理由（`working_in_progress`）を出す。「カメラ」は押せる |
| 「停止を解く」 | 押すと W-4「測定機構の腕が格納位置にあることを確かめましたか」（「やめる」／「解く」）。「解く」で `ui.working` `{"on": false}`、「やめる」は何も送らない。（P3 で `ADJUST` の間は押せなくする） |
| W-1（回復フォルト） | `AT_PANEL`/`WORKING` でフォルトが出ている間も W-1 を出す（今は `PAUSE`・`ESTOP` だけ）。ボタンは「確認」（`ui.resume_ack`）。状態は `WORKING` のまま |
| 腕の格納の確認（-18） | P5a-ui の判定関数（`modes/armConfirm.js`）を広げる: `ESTOP`・`CARRY` では **`prev_mode` が `AUTO_MEASURE`、または `/system/state.working` が `true`** のとき W-1 の「メインメニューへ」・「確認」と W-2 の「メインメニューへ」の前に出す。**`AT_PANEL`/`WORKING` からの「終了」（`ui.finish`）の前にも出す** |

### 6.4 盤前の旋回・前後の並進（`ADJUST`）の作り（2026-11-23 決定。計画書 8-g の P3）

**P3（機体側）と P3-ui（画面）に分けて同時に進める。**P3-ui は yaml を触らない。正は [Spec-modes.md](../spec/Spec-modes.md) `SM-3.1.2-127`・`-128`・`-132`・`-136`〜`-140` と [Spec-measure.md](../spec/Spec-measure.md) §5・§5.3。左右の並進は受けない（`bad_argument`。P1 のまま）。

| 項目 | 決まり |
| --- | --- |
| 受ける状態 | `meas.turn`（`angle_rad`）・`meas.translate`（`side` `FORWARD`／`BACKWARD`・`distance_m` 正）を **`AUTO_MEASURE`/`WAIT`（-127）と `AT_PANEL`/`WORKING`（-136）だけ**で受け、`ADJUST` へ。ほか（`ADJUST` 中を含む）は `state_not_accepting`（-132・-138）。`angle_rad` が 0・有限でない、`distance_m` が 0 以下・有限でない は `bad_argument` |
| 受ける前の確認（同期のガード） | **旋回・並進とも角が描く円**（P5b の `/measure/corner_status` のラッチ。1 秒届かなければ塞がり扱い）→ 理由 `corner_obstacle`。**並進は加えて進む先**: `measure_clearance` が `/measure/corner_status` の JSON に **`free_forward_m`・`free_backward_m`** を足し、`distance_m` がそれ以下でなければ拒否 → 理由 **`path_obstacle`**。値は「今の位置からまっすぐ進んで機体の形 0.55 m 角が障害物マップかスキャンの障害物に当たるまでの距離」から**リミッタの止まった位置の余裕 `floor_margin_m` を引いたもの**（引かないと、盤へ向かう並進を受けたあとリミッタが手前で止め、終わらずに打ち切りになる）。**LiDAR の死角（`blind_angle_ranges`）にかかる向き・スキャンが古い・TF が取れないときは 0**（確かめられないものを空きと扱わない。§5.2 と同じ考え）。返りの無い方向（見える範囲の外）は、見える範囲の端までを空きとする。判定の順: `reentry_required`（自律測定のみ）→ 角 → 進む先 |
| 動かすノード | **`th_measure` の新しいノード `measure_adjuster`**。effect **`measure_adjust`**（`{command_id, kind: turn｜translate, angle_rad, side, distance_m}`）を受けて、`/system/state` が `ADJUST` の間だけ `/cmd_vel_behavior` に出す。**帰還は TF `odom→base_link`**（数十 cm・数十度の相対の動きなので、スキャン照合で跳ぶ `map` ではなく連続な `odom` を使う。**旋回は始点からのヨーの変化を巻き戻さずに積み上げて比べる**（目標を (-π, π] に丸めると、π を超える指示が逆向きの近道を回る）。正は左回り。並進は始点からの進んだ距離で、間は向きを保つ）。`ADJUST` から出たら（どの理由でも）すぐ 0 を 1 回出して黙る |
| 速さと精度 | 旋回は試験場内の向き合わせ（`ALIGN`）の registry 値（`align_kp`・`align_w_max_rps`・`w_align_max`・向き合わせの旋回の速さの下限）と許容差 `align_tolerance_rad` を使う。並進は新しい registry 値（class b・given）: 並進の速さ・許容差。**速度の上限は `ADJUST` の間 `v_jog_panel`**（`AUTO_MEASURE` の `v_slow` でも `AT_PANEL` の停止でもない。上限の決め方に「状態 `ADJUST` はジョグ中と同じに `jog_speed_limit`」を足す）。障害物の停止・非常停止は `obstacle_limiter` がふだんどおり効かせる（後退も） |
| 終わり（-128・-137） | `measure_adjuster` が `/system/event` に **`evt.adjust_done`**（`arg_json` に `command_id`）→ `ADJUST` から `WAIT`（自律測定）／`WORKING`（停止命令）。`state_manager` が進行中の `turn`／`translate` に **`DONE` を 1 回**。`command_id` が進行中のものと違う `evt.adjust_done` は捨てる |
| 進まないとき | **締め切りは `state_manager` が持つ**（`ADJUST` に入ってから **`adjust_timeout_ms`**（新しい registry 値）。`sys.jog_lease_expired` と同じくタイマで `sys.adjust_timeout` を流す）→ `WAIT`／`WORKING` へ戻り、結果は **`CANCELLED`（`adjust_timeout`）**（`FAILED` は予約のまま使わない）。`measure_adjuster` が落ちても `ADJUST` に閉じ込められないため（停止命令の `ADJUST` では `meas.done`・停止を解く・行き先を拒否するので、ほかに出口が無い）。`measure_adjuster` は、進まないと分かれば早めに **`evt.adjust_failed`**（`{command_id, reason}`）を出してよい |
| 途中でやめる | **`ADJUST` から出たら進行中の旋回・並進は `CANCELLED`**（理由は出た契機: `stopped`（`meas.stop`）・`aborted`（`meas.abort`）・`fault`・`jog`・`link_lost`・`estop`・`carry`・`localization_lost`）。目的地（`meas.goal`）と違い、**持ったまま止まらない**（勝手に動き直さない。測定機能が自己位置から指示し直す） |
| 自律測定の `ADJUST` | `meas.stop` → `PAUSE`（-129）。`meas.abort` → `WAIT`（-131）。ジョグ・回復フォルト → 共通行で `PAUSE`。途絶 → `PAUSE`（`measure_link`。-133）。非常停止・手押し → 戻ると `PAUSE`（走行状態と同じ。`is_driving` の追加に `ADJUST`）。`PAUSE` からの `meas.start` は `WAIT`（止める前が `ADJUST` でも。指示は捨ててある） |
| 停止命令の `ADJUST` | ジョグは拒否（P2 の行を `ADJUST` にも）。回復フォルト・途絶 → `WORKING`（-139。動きをやめて捨てる）。非常停止・手押し → 戻ると `WORKING`（-140。押下前が `ADJUST` でも `WORKING`）。`meas.done`・`ui.working` off（停止を解く）・行き先は拒否（-138。理由 **`adjust_in_progress`**）。`/system/state.working` は `ADJUST` でも `true` |
| 結果の台帳・`working` | 「測定の続き」に `AT_PANEL`/`ADJUST` を足す（P2 の片付けをそのまま）。`published_working` も `ADJUST` を含める |

**P3-ui（画面）の決まり**（2026-11-23）。

| 項目 | 決まり |
| --- | --- |
| 状態の文言 | `ADJUST` を「測定機能の指示で動作中」（S-21・S-60・ヘッダー）。S-21 は停止命令の表示のまま |
| 「停止を解く」 | `AT_PANEL`/`ADJUST` では押せなくし、理由 `adjust_in_progress`（「旋回中・並進中は腕が起きているため解けません」）を出す |
| 理由キー | `path_obstacle`（進む先に障害物）・`adjust_in_progress`・`adjust_timeout` の文言を `i18n/reasons.js` に足す（測定機能の画面も同じ表を取り込む） |
| 腕の確認・W-1 | P2-ui の判定（`working`）がそのまま効くこと（`ADJUST` でも `working` は `true`）。`AT_PANEL`/`ADJUST` のフォルトでも W-1 を出す |

### 6.5 腕が振れる範囲の確認（`/measure/check_arm_clearance`）の作り（2026-11-23 決定。計画書 8-g の P4）

**機体側だけ（画面は無い）。**型と意味は §3.5 が正。正は [Spec-modes.md](../spec/Spec-modes.md) `SM-3.1.2-141` と [Spec-measure.md](../spec/Spec-measure.md) §5.2。**状態は変えない**ので `state_manager` を通さず、`measure_clearance` がサービスを直接持つ。

| 項目 | 決まり |
| --- | --- |
| 受け手 | `measure_clearance` がサービス `/measure/check_arm_clearance` を持つ。`/system/state` を購読して状態を覚える（未受信は「受けない状態」） |
| 拒否の順 | ① `bad_argument`: `region` の頂点が 3 未満・座標が有限でない／`sides` が空・`LEFT`／`RIGHT`／`CURRENT` 以外・重複 → ② `state_not_accepting`: `AUTO_MEASURE`/`WAIT` と `AT_PANEL`/`WORKING` 以外（`ADJUST` を含む）。拒否のとき `sides`・`results`・`unknown_reasons` は空 |
| 1 つの向きの判定 | `region` を `base_link` の原点まわりに `LEFT` +90°・`RIGHT` −90°・`CURRENT` 0° 回し、**機体の形 0.55 m 角（\|x\|・\|y\| ≤ 0.275 m）の内側を除いた部分**を見る。順に: **① スキャンが `arm_clearance_scan_max_age_ms` より古い・届いていない・TF（`base_link`←`laser_link`）が取れない → `UNKNOWN`（`scan_stale`）**（全部の向き）→ **② 範囲の中に障害物（`/scan_filtered` の有効点か、局所コストマップの占有セル（内接 99 以上）。角の円と同じ入力。しきい値は角の円（§6.8 で致死だけに改めた）と違う。コストマップが届いていなければスキャンだけ）→ `BLOCKED`**（見えている物があれば、死角があっても塞がり）→ **③ 範囲が LiDAR の死角（`blind_angle_ranges`）にかかる → `UNKNOWN`（`blind_sector`）。ただし `laser_link` の中心から **`arm_clearance_blind_near_m`**（新しい registry 値。class b・given・初期値 0.65 m）以内の部分は死角の判定から除く**（四隅の支柱の影。2026-11-23 ユーザー決定。[Spec-measure.md](../spec/Spec-measure.md) §5.2 の既知のリスク。除かないと、いまの支柱ではどの向きも毎回 `UNKNOWN`。障害物の判定 ② は除かない）。**範囲の平均の向きなど代表の向きで決めない**（遠くで死角にかかる部分を見逃す） → ④ `CLEAR`。返りの無い方向（見える範囲の外）は空きとする（§6.4 の進む先と同じ） |
| 返す形 | `sides` は問い合わせと同じ順。`results` と `unknown_reasons` は `sides` と同じ長さで、`unknown_reasons` は `UNKNOWN` 以外では空文字。`checked_at` は判定に使ったスキャンの時刻（スキャンが無ければ 0） |
| 時点 | 問い合わせの時点の最新のスキャンで同期に判定する（溜めない・待たない）。腕を動かす直前に問い合わせ直すのは測定機能 |

### 6.6 速度・加速度の上限（`meas.set_limits`）の作り（2026-11-24 決定。計画書 8-g の P5c）

**機体側だけ（画面は無い）。**正は [Spec-measure.md](../spec/Spec-measure.md) §3・§5.1（下げる方向にだけ効く・障害物・非常停止・途絶による停止は鈍らせない）と §3.1 の表の `meas.set_limits` の行。P5b で「効かせる仕組みができるまで受けない」とした行（§6.2）を、この節で置き換える。

| 項目 | 決まり |
| --- | --- |
| 受ける状態 | `AUTO_MEASURE` の全状態と `AT_PANEL`/`WORKING`・`ADJUST`（§3.1 の表）。**状態は変えない**。すぐ `DONE` |
| 引数 | `v_max_mps`・`w_max_radps`・`a_max_mps2`・`alpha_max_radps2`。**0 は「指定しない」**（その項目は本システムの上限のまま）。負・有限でないは `bad_argument`。**1 回の指示で 4 つまとめて置き換える**（前の指示の値は残さない。0 を送ればその項目は外れる） |
| 持つ所・捨てる時 | `state_manager` が持つ。**測定の続きの間だけ持ち**（結果の台帳の片付けと同じ判定。§6.3・§6.4。`ESTOP`・`CARRY` を挟んでも持つ）、続きでなくなったら 4 つとも 0 に戻す（自律測定を抜けた・停止命令が解けた。人のジョグに効き続けないため） |
| 運び方 | `SystemState` の末尾に **`measure_v_cap_mps`・`measure_w_cap_radps`・`measure_a_cap_mps2`・`measure_alpha_cap_radps2`**（float32。0＝指定なし）を足す（内部の型。測定機能の型は変えない）。`/measure/command_state` の `v_cap_mps`・`w_cap_radps` も同じ値 |
| 効かせ方（`obstacle_limiter`） | 速度: **直線の上限 = min(今の上限, `measure_v_cap_mps`)・角速度の上限 = min(`w_max`, `measure_w_cap_radps`)**（指定があるときだけ。障害物・画面・モードの上限と同じく、ランプを通さず即座に絞る）。加速度: **通常時ランプの加速度の上限 = min(`normal_accel_mps2`, `measure_a_cap_mps2`)・角加速度 = min(`normal_angular_accel_rps2`, `measure_alpha_cap_radps2`)**（ランプは指令の変化にだけかかるので、加速と指令による減速の両方に効く。`normal_*` が 0（ランプなし）のときは指定の値だけで）。**ロック・途絶・障害物の上限による停止はいまの作りのままランプを通らない**（鈍らせない。§5.1）。`/system/state` が古いときは最後に受けた指定を使い続ける（下げた上限を勝手に外さない） |
| 報告（`/measure/status`） | `v_limit_mps` = min(本システムの上限, `v_cap`)・`w_limit_radps` = min(`w_max`, `w_cap`)（指定が無ければ本システムの上限だけ）。**いま効いている値を出す** |
| 既知の注意 | 旋回（`ADJUST`）の角速度の指定を向き合わせの旋回の速さの下限（`turn_in_place_w_min_rps`）より小さくすると回り切れず、締め切りで取り消しになりうる（測定機能が値を決めるときに考える。fk `FK-V5`） |

### 6.7 シミュレーション（Gazebo）で自律測定を通す（2026-11-24 決定。fk 申し送り 23・24。計画書 8-g の P6）

**機体側の起動の作りだけ（挙動の spec は変えない）。**測定機能が `gazebo.launch.py sim:=true` の本物の一式に繋いで自律測定を通せるようにする。

| 項目 | 決まり |
| --- | --- |
| 目的地への走行の受け手 | `gazebo.launch.py` でも `venue_navigator` を起動する（`bringup.launch.py` の `onsite_enabled` の下と同じ生成 yaml）。`venue_navigator` が自律測定の走行で頼るもの（Nav2 の `FollowPath`・`ComputePathToPose`、`/map`、`/map_session/status` など）が sim で揃っているかを確かめ、足りなければ同じく起動する。足すのは自律測定の走行に要るものだけ（前日準備・試験場内の画面の機能は広げない） |
| 起動時の疎通（`evt.link_ok`） | **開発モードを流用しない。**既存の `connectivity_checker` の `sim`（`gazebo.launch.py` が渡す。Gazebo に居ない ESP32 の 2 項目と必須ノードを判定から外す）を広げ、**`sim` のときは物理非常停止の状態を受信済みであることを求めない**（出すのは `esp32_bridge` だけで、sim には居ない）。押されていると分かっている間は、`sim` でも出さない（開発モードと同じ境界）。新しい起動引数・名前は作らない |
| 確かめ方 | `gazebo.launch.py sim:=true gui:=false` を起動し、手を加えずに `IDLE` に入る。測定機能の代役（心拍・`/measure/command`）で `ui.goto`（`{"kind":"AUTO_MEASURE"}`）→ `meas.goal` を送り、Gazebo の機体が動いて `ARRIVED` が 1 回返る |

### 6.8 発進・旋回の前の確認の円を 0.689 m にする（2026-11-25 決定。fk 申し送り 25。計画書 8-g の P7）

[Spec-measure.md](../spec/Spec-measure.md) §4.4 の改定（0.389 m → 0.689 m）。`measure_clearance` の角の円の確認（`/measure/corner_status`）だけを変える。

| 項目 | 決まり |
| --- | --- |
| 円の半径 | 新しい registry 値 **`corner_clear_radius_m`**（class b・given・0.689 m。`consumers: [params_audit, measure_clearance]`）。スキャンと局所コストマップの両方をこの半径で見る。生成 yaml から渡し、走らせたまま変えられる（`arm_clearance_blind_near_m` と同じ扱い） |
| 局所コストマップのしきい値 | **角の円では致死（OccupancyGrid の値 100＝物そのもの）だけを数える。**内接（99）とそれ未満の膨らみは数えない。数えると `inflation_radius` の分（sim・実機とも約 0.3 m 以上）さらに手前で断り、0.689 m に余裕を二重に足すことになる（申し送り 25 の発端） |
| 変えないもの | 腕が振れる範囲の確認（§6.5）と前後の並進の進む先（§6.4）は、いままでどおり内接（99）以上を数える。理由の名前（`obstacle_scan`・`obstacle_costmap`）、5 Hz、鮮度、`state_manager` 側のラッチと 1 秒の途絶は変えない |
| 確かめ方 | 本物の `measure_clearance` を生成 yaml 先渡しで起動し: 中心から 0.6 m のスキャン点 → `clear=false`／0.75 m → `clear=true`／円の中に値 99 のセルだけ → `clear=true`／値 100 のセル → `clear=false`／`corner_clear_radius_m` を走らせたまま変えると判定が変わる |

## 7. 未決と依存

| 項目 | 状態 |
| --- | --- |
| 腕が振れる範囲の多角形の形と大きさ | **決定（暫定）**: §3.5。値は測定機能のパラメータ |
| ヘッダーと異常ウィンドウの中身の渡し方 | **決定**: 生のトピックの購読と理由キーの表の取り込みで始める（§4）。実装して足りなければ相談 |
| `measure_link_timeout_ms`・`arm_clearance_scan_max_age_ms` の値 | `measure_link_timeout_ms` は初期値 1000（2026-11-23。§6.1）。`arm_clearance_scan_max_age_ms` は初期値 500（2026-11-23。P5b の角の円の確認と共用。§6.5） |
| **依存: 自律測定の間は `/map` を凍結しない** | `map_freezer` が `/map_live` をそのまま通す場面に `AUTO_MEASURE` を足す（[names](DetailedDesign-names.md) §6.4 の `/map` の行。[Spec-measure.md](../spec/Spec-measure.md) §4.1）。8-g |
| 参考（測定機能の決定。本システムは変えない） | 自律測定で腕が出ている間の人のジョグは人の責任（既知のリスク）。お立会試験では、昇降機を測定前の高さへ戻してから `meas.done` を返す |
| 測定機能の最初の版の使い方（参考。fk `Spec-auto.md`） | 盤前の並進は前後だけ（2026-11-14。点ごとに「その場旋回 → 前後の並進」。左右は旋回と前後の組み合わせで動く。2026-11-15）／速度・加速度の上限は当面指定しない／一時停止からの再開はいつも人の「走行」。**本システムの実装の順番の手がかりにする** |
