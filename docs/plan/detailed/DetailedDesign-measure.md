# 測定機能（fk-system）とのやり取り

[DetailedDesign.md](DetailedDesign.md) の詳細。**自律測定（`AUTO_MEASURE`）と、お立会試験の停止命令の間（`AT_PANEL` の `WORKING`・`ADJUST`）に、
測定機能（fk-system。`../fk-system`）と本システムがやり取りする名前と形。**

入力は [Spec-measure.md](../spec/Spec-measure.md) §1〜§7 と [Spec-onsite.md](../spec/Spec-onsite.md) §7.2、遷移は
[Spec-modes.md](../spec/Spec-modes.md) `SM-3.1.1-18`・`SM-3.1.2-062`〜`-063`・`-120`〜`-141`。
**本システムが書き、測定機能が確かめる**（2026-11-04。fk-system の申し送り 16。[Spec-open.md](../spec/Spec-open.md) §8）。
**状態: 案（測定機能の確認待ち）。まだ実装していない**（計画書 §6 の 8-g）。

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
| 版 | `interface_version`（文字列。**初版は `"1"`**）。型・名前・意味を変えたら上げる |
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
string  side              # meas.translate: LEFT / RIGHT
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
| `meas.translate` | 並進 | `side`・`distance_m` | `AUTO_MEASURE`/`WAIT`（-127） | `DONE`（-128）。**測定機能の最初の版は使わない** |
| `meas.set_limits` | 速度・加速度の上限 | `v_max_mps` ほか | `AUTO_MEASURE`/任意・`AT_PANEL`/`WORKING`・`ADJUST` | すぐ `DONE`。**本システムの上限との小さい方だけが効く。**自律測定はモードを抜けたら、お立会試験は**停止命令が解けたら**（`WORKING`・`ADJUST` を出たら）捨てる（人のジョグに効き続けないため） |
| `meas.done` | 測定を終えた | — | `AT_PANEL`/`WORKING`（-063） | すぐ `DONE`（停止命令が解ける） |

| `reject_reason` | 意味 |
| --- | --- |
| `state_not_accepting` | いまのモード・状態では受けない（-132・-138 ほか） |
| `turning` | 旋回中は受けない（-138） |
| `corner_obstacle` | 角が描く円の中に障害物がある（-121・-127・-130・-136。[Spec-measure.md](../spec/Spec-measure.md) §4.4） |
| `map_mismatch` | `map_instance_id` が今の地図と違う |
| `version_mismatch` | `interface_version` が食い違う（§1.2） |
| `link_lost` | 生存確認が途絶えている |
| `bad_argument` | 欄が足りない・値の範囲外・`command_id` が空 |

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

**目的地を持ったまま止まる場合は、結果を出さない。**自律測定の `NAV`・`ALIGN`・`BLOCKED` の最中に、回復フォルト・人のジョグ・通信途絶で `PAUSE` に落ちたときと、**非常停止・手押し（自己位置を見失った `LOCALIZATION_LOST` を含む）で `ESTOP`・`CARRY` に入ったとき**がこれに当たる。戻る先は `PAUSE` で（[Spec-modes.md](../spec/Spec-modes.md) §6 の `AUTO_MEASURE`）、測定機能の発進（`meas.start`。-130）でもとの `command_id` のまま続くため。目的地を捨てたとき（中止・終了・人が抜けた）に `CANCELLED` を出す。

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
float32 v_limit_mps          # いま効いている速度の上限（本システムと測定機能の指定の小さい方）
float32 w_limit_radps
```

| 項目 | 決まり |
| --- | --- |
| 自己位置 | **自律測定は、そのとき作っている地図の座標。お立会試験は会場地図の座標**（[Spec-measure.md](../spec/Spec-measure.md) §3） |
| 地図の世代 | **自律測定は入るたびに地図を作り直す。**測定機能は目的地に `map_instance_id` を付け、本システムは今の地図と違えば `map_mismatch` で拒否する（前の地図の座標で走り出さない） |
| 旋回の出し直し | 旋回が途中で止まったら、測定機能はこの `yaw` から残りの角度を出す（申し送り 14） |

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
| 見るもの | 角が描く円の確認と同じ入力（障害物マップかスキャン）。[Spec-measure.md](../spec/Spec-measure.md) §5.2 |
| `UNKNOWN` | 最後のスキャンが **`arm_clearance_scan_max_age_ms`** より古い（`scan_stale`）か、範囲が LiDAR の死角にかかる（`blind_sector`）。**測定機能は塞がっているのと同じに扱う** |
| 受ける状態 | `AUTO_MEASURE`/`WAIT` と `AT_PANEL`/`WORKING` だけ（-141）。**状態は変えない** |
| 意味 | `CLEAR` は**床から 32 cm の走査面に物が無い**という意味だけ（[Spec-measure.md](../spec/Spec-measure.md) §5.2） |

---

## 4. 測定機能の画面（rosbridge）

**測定機能の画面は、本システムの画面と同じ扱いの端末になる。**送るのは人の操作に当たるものだけ。

| 名前 | 向き | 決まり |
| --- | --- | --- |
| `/ui/active_screen`（`ActiveScreen`） | 画面 → 本システム | **在席と画面の申告。**`screen_id` は、自律測定の間は **`S-60`**、お立会試験の停止命令の間は **`S-21`**（どちらもゾーン `IN`）。**`client_id` は `fk-` で始める。**`interacting`・`last_input`・2 Hz は本システムの画面と同じ（[names](DetailedDesign-names.md) §5.1）。新しい画面 ID は作らない（ゾーンの表を 3 か所直さずに済む） |
| `/safety/estop_ui`（`std_msgs/Bool`） | 画面 → 本システム | ヘッダーの非常停止。**押している間 2 Hz で `true` を送り続け、解除は `false` を 1 回**（[safety](DetailedDesign-safety.md) §6.3。押下側にラッチ） |
| `/system/trigger`（`UiTrigger`） | 画面 → 本システム | **ヘッダーと異常ウィンドウへの人の応答だけ**（`ui.resume_ack` など。[names](DetailedDesign-names.md) §8.1） |
| `/measure/command`（`MeasureCommand`） | 画面 → 本システム | 測定機能の画面の**「走行」**（人のジョグで止まったあとの発進。`meas.start`）など、人の操作を測定機能の指示として送るとき。**画面から rosbridge で直接呼んでも、測定機能の機能側を通してもよい（測定機能が決める）。**どちらでも送るのは `meas.*` だけ |
| `/system/state`・`/safety/active_faults`・`/safety/fault`・`/measure/status` | 本システム → 画面 | ヘッダーと異常ウィンドウの中身。**本システムの画面と同じトピックを購読し、理由キーの表（`th_ws/web_ui/src/i18n/`）を共有する**（足りなければ集約トピックを足す。§7） |

---

## 5. 入口の分離（安全境界）

[names](DetailedDesign-names.md) §8 の「名前空間の分離はそれ自体が安全境界」に、入口を 1 本足す。

| 決まり | 理由 |
| --- | --- |
| **`meas.*` は `/measure/command` からしか受けない。**`/system/trigger` に来た `meas.*` と、`/measure/command` に来た `ui.*` は拒否する | 測定機能の指示と人の操作を取り違えない |
| **測定機能の機能側は `ui.*` を送らない。**送ってよいのは測定機能の画面で人が押したものだけ（§4） | 非常停止の解除と異常への応答は人が行う（[Spec-measure.md](../spec/Spec-measure.md) §2） |
| `meas.*` を受けるのは `AUTO_MEASURE` と `AT_PANEL` の `WORKING`・`ADJUST` だけ | それ以外のモードで測定機能が機体を動かさない |

---

## 6. 本システムの中の受け手（案。8-g で決める）

| 役割 | 案 |
| --- | --- |
| `/measure/command` の提供・`meas.*` の受理・`/measure/result` の発行 | **`th_state`**（`/system/trigger` と同じく状態機械が受ける。遷移と結果の取りこぼしを 1 か所で防ぐ） |
| 生存確認の監視・`/measure/status`・`/measure/check_arm_clearance` | 新しいパッケージ **`th_measure`** のノード（名前は 8-g で [names](DetailedDesign-names.md) に足す） |
| 自律測定の走行（地図を作りながら・手前 1.5 m の検査）と、旋回（`ADJUST`） | `th_measure` の挙動ノード。速度指令は `/cmd_vel_behavior`・`/cmd_vel_nav`（[names](DetailedDesign-names.md) §6.1 の不変ルールのまま） |

---

## 7. 未決と依存

| 項目 | 状態 |
| --- | --- |
| 腕が振れる範囲の多角形の形と大きさ | **測定機能が決める**（範囲の決め方は測定機能と突き合わせる） |
| ヘッダーと異常ウィンドウの中身の渡し方 | まず生のトピックの購読で始める（§4）。測定機能の実装を見て、足りなければ集約トピックを足す（**要相談**） |
| `measure_link_timeout_ms`・`arm_clearance_scan_max_age_ms` の値 | (b)。実装時に決める |
| **依存: 自律測定の間は `/map` を凍結しない** | `map_freezer` が `/map_live` をそのまま通す場面に `AUTO_MEASURE` を足す（[names](DetailedDesign-names.md) §6.4 の `/map` の行。[Spec-measure.md](../spec/Spec-measure.md) §4.1）。8-g |
| 測定機能の最初の版の使い方（参考。fk `Spec-auto.md`） | 盤前の並進を使わない／速度・加速度の上限は当面指定しない／一時停止からの再開はいつも人の「走行」。**本システムの実装の順番の手がかりにする** |
