# 安全チェーンの実装

[DetailedDesign.md](DetailedDesign.md) の詳細。**`Spec-safety.md` を実装に落とす。**

> **G1 は他のすべてに優先する。**このファイルの決定は、他のファイルの都合で覆さない。

---

## 0. 先に読む — 安全負債（着手前提条件）

**現物が仕様どおりでない箇所が 5 件ある。**いずれも「いつか直す TODO」ではなく、
**特定の段階の着手前提条件（blocking precondition）**として扱う。
台帳は [DetailedDesign-open.md](DetailedDesign-open.md) §2。

| # | 事実 | 影響 | 解除する段階 |
| --- | --- | --- | --- |
| **DEBT-1** | `th_ws/esp32/src/config.h:130` の `ESTOP_BENCH_TEST_BYPASS` が有効で、`main.cpp:119-121` が `estopActive` を無条件に `false` にする | **安全チェーン層 1 が存在しない。**`Spec-safety.md` §10「層 1・2 は開発モードでも無効化できない」に反する | §11.1 |
| **DEBT-2** | `th_bringup/config/perception_params.yaml:15-23` の `blind_angle_ranges` が幅ゼロ（40/40, 130/130, 220/220, 310/310） | 死角マスクが実質無効。**この状態で `obstacle_limiter` を有効にすると自分のアルミ角柱で永久停止する** | §11.2 |
| **DEBT-3** | `lidar_timeout_ms` / `esp32_timeout_ms` = 2000 ms | 計画 `v_max` = 0.7 m/s で**タイムアウトまでに 1.4 m 走る** | §11.3 |
| **DEBT-4** | `/cmd_vel` が途絶しても `esp32_bridge` がキープアライブで**最後の非ゼロ指令を 20 Hz で送り続ける**（`esp32_bridge.py:237-238`）。ESP32 ウォッチドッグも WHEEL_CMD が来ているので発火しない | **上流が死んでも誰も止めない。**`obstacle_limiter` を多重化の後段に置く前提が崩れる | §3.4 |
| **DEBT-5** | `map_and_localization_slam_toolbox_node` の SIGSEGV を `slam_control.py` が検出しても、ログと String を出すだけで `/safety/fault` を立てない | **`map→odom` が凍結しても機体は走り続ける** | §5.3 |

**`DEBT-4` は最優先である。**これを塞がないうちは、`obstacle_limiter` の新設が
「安全装置を足した」ではなく「単一障害点を足した」になる。

---

## 1. 4 層の実装対応

| 層 | 仕組み | 実装 | 反応時間 | 依存 |
| --- | --- | --- | --- | --- |
| **1** | 物理非常停止ボタン | ESP32 GPIO32（`ESTOP_LOW_ACTIVE`）＋ モータードライバ電源の電気的遮断 | 即時 | 何にも依存しない |
| **2** | ESP32 ウォッチドッグ | `config.h` の `WATCHDOG_MS = 600`。`last_cmd_ms` からの経過で判定 | 600 ms 以内 | ESP32 単体 |
| **3** | 安全監視の非常停止・フォルトロック | `safety_monitor`（C++）→ `/safety/estop` `/safety/fault_lock` → twist_mux ロック ＋ **`esp32_bridge` の独立ロック層** | 100 ms 以内 | PC 上 |
| **4** | 状態機械の遷移 | `th_state`。回復フォルトは `PAUSE`、重大フォルトは `ESTOP` | 上位が止めた後 | PC 上 |

### 1.1 層 3 は `obstacle_limiter` を通らない

**これが後段配置を許す根拠である。**

```
safety_monitor ──► /safety/estop / /safety/fault_lock ──┬──► twist_mux（ロック）
                                                         └──► esp32_bridge（独立ロック層）
```

`esp32_bridge` は `/safety/estop` と `/safety/fault_lock` を**直接購読**し、
いずれかが有効な間は `/cmd_vel` の内容によらずゼロを送る。
さらに**ロックトピック自体が 0.5 s 途絶したらロック扱い**にする（起動直後の未受信も含む）。

**この層を「`obstacle_limiter` と重複するから消そう」としてはいけない。**
`twist_mux` と `obstacle_limiter` が両方死んでも効く唯一の PC 側経路であり、
`Spec-safety.md` §1「上位が下位の故障に依存しない」の実装そのものである。

---

## 2. 速度指令の経路

```
th_transit / th_onsite / th_route ──► /cmd_vel_behavior (20) ─┐
th_maintenance（点検・校正の走行）  ──► /cmd_vel_behavior (20) ─┤
                                                               │ twist_mux
WebUI ─► /cmd_vel_manual_raw ─► [ jog_gate ] ─► /cmd_vel_manual (30) ─┤
                                    ▲ /system/state              │
Nav2 controller_server ────────────►/cmd_vel_nav (10) ───────────┘
                                                                 ▼
                                                        /cmd_vel_muxed
                                                                 │
   /scan ────────┐                                               │
   /system/state ─┤──────────► [ obstacle_limiter ] ◄────────────┘
   /cmd_vel_manual│                    │ 20 Hz 固定レート
   /safety/estop  │                    ▼
   /safety/fault_lock ─┘            /cmd_vel ──► esp32_bridge ──► ESP32
                                                （独立ロック層は維持）
```

**不変ルール（現行から変更）**: `/cmd_vel` を publish してよいのは **`obstacle_limiter` だけ**。
`twist_mux` の出力は `/cmd_vel_muxed` に変える。

> **変更する場所は `twist_mux.yaml` ではない。**remap は launch にある。
>
> | ファイル | 現行 | 変更後 |
> | --- | --- | --- |
> | `th_bringup/launch/bringup.launch.py`（`remappings=[('cmd_vel_out','/cmd_vel')]`） | `/cmd_vel` | `/cmd_vel_muxed` |
> | `th_bringup/launch/gazebo.launch.py`（同） | `/cmd_vel` | `/cmd_vel_muxed` |
>
> **両方を直さないとシミュレーションだけ旧経路のままになる。**
> `th_testing/test/test_twist_mux_priority.py` も旧トピック名で書かれているので同時に直す。
> **これらは `obstacle_limiter` の実装と同一の作業パケットで行う**
> （先に remap だけ変えると `/cmd_vel` の publisher が消えて機体が動かなくなる）。

---

## 3. `obstacle_limiter`（新設）

`Spec-safety.md` §2.4 が「速度指令の多重化の**手前**に置くリミッタを新設する」と書いているが、
**後段に置く。**

### 3.1 なぜ後段か

| 観点 | 後段（1 個） | 前段（速度源ごと） |
| --- | --- | --- |
| `SD-6`「駆動へ直接指令するものを作らない」 | ◎ 単一の絞り口 | △ 速度源を足すたび素通しが増える |
| 速度源の追加耐性 | ◎ | × |
| Nav2 の `/cmd_vel_nav` に効く | ◎ | ◎ |
| **追従対象の脚を障害物と誤検知しない** | **× 後段は「誰が対象か」を知らない** | ◎ |
| 単一障害点 | **× `DEBT-4`** | ◎ |

**→ 二層に分ける。**「後段 1 個に一本化」はしない。

| 層 | 置き場所 | 責務 | 判定距離 |
| --- | --- | --- | --- |
| **挙動側の障害物判定** | 各挙動ノード内（`is_path_blocked` を流用） | 方式ごとの意味論。**追従対象の除外**、`follow_stop_distance_m`、線ロスト | `obstacle_stop_distance_m`（＝`d_behavior`） |
| **`obstacle_limiter`** | twist_mux の後段 | 速度源に依存しない最終床。速度上限クランプ | `obstacle_floor_distance_m`（＝`d_floor`） |

```
d_floor  <  d_behavior   かつ   d_floor  <  follow_stop_distance_m
```

**`d_floor` を制動距離から求めてはいけない。**
`d_behavior` も `follow_stop_distance_m` も同じ `braking_distance()` から出るので、
マージンの差しか無くなり、**リミッタが追従対象の脚で発火して追従が成立しなくなる**
（リミッタは「誰が対象か」を知らない）。

**`d_floor` は機体外形からの固定クリアランスとして定義する。**

```python
def floor_distance(body_half_length_m: float, floor_margin_m: float) -> float:
    """速度に依存しない。『これ以上近づいたら何が起きていてもおかしい』距離。"""
    return body_half_length_m + floor_margin_m
```

これにより **リミッタは上位が壊れたときだけ発火する**。
`Spec-safety.md` §1「上位が下位の故障に依存しない」の正しい形になる。

**アサーション A2 を 2 本にする**（[DetailedDesign-params.md](DetailedDesign-params.md) §4）:

```
A2a: obstacle_floor_distance_m < obstacle_stop_distance_m
A2b: obstacle_floor_distance_m < follow_stop_distance_m
```

### 3.2 自律／手動の判定は状態機械から取らない

`Spec-modes.md` §8 により、**ジョグ介入中は自律系モードでも手動系として扱う**。
だからモードだけでは決まらない。しかし `jog_active` を `/system/state` 経由で取ると、
**WiFi の受信ギャップ（実測 0.5〜1.2 s）がそのまま分類のずれ時間になる。**
危険側のずれは「FSM はまだ `jog_active`、実際は挙動ノードが走行を再開している」。

**`/cmd_vel_manual` の鮮度と `jog_active` の AND で決める。**

```cpp
bool manual_fresh = (now - last_cmd_vel_manual_stamp) <= manual_joy_timeout;
bool state_fresh  = (now - last_system_state_stamp)   <= state_stale_ms;
// MANUAL を名乗れるのは両方が揃ったときだけ。どちらか不明なら AUTO（厳しい側）
source_class = (manual_fresh && state_fresh &&
                (st.jog_active || st.mode == "MANUAL" || st.mode == "TEACH_MANUAL"))
               ? MANUAL : AUTO;
```

**鮮度だけで決めてはいけない。**迷い込んだ手動パケット 1 発で、
最大 `manual_joy_timeout`（1.0 s）のあいだ自律走行中の指令が
**`MANUAL` 政策（場外＝警告のみ）で通ってしまう**。`Spec-safety.md` §2.1
「自律系は停止する・**無効化不可**」に反する。

**不変条件 L7 を足す**: `AUTO` 政策は決して緩まない。
分類に迷いがある入力（片方が stale・値が矛盾）は必ず `AUTO` に倒す。

`manual_joy_timeout` は `twist_mux.yaml` の値（1.0 s）と**同一のパラメータ**を読む。
**`twist_mux.yaml` も `export.py` の生成対象に含める**ので、数値の実体は `registry.yaml` にしかない
（[DetailedDesign-params.md](DetailedDesign-params.md) §5）。

`/system/state` は**上限値の選択と `source_class` の AND 条件にだけ**使う。
`auto_brake` の既定もここから引く。**`/system/state` が `state_stale_ms` 以上途絶したら、
ゾーン＝最も低い上限（停止）・`auto_brake = ON` に倒す。**

### 3.3 政策表

| `source_class` | ゾーン | 障害物を検知したとき | 根拠 |
| --- | --- | --- | --- |
| `AUTO` | 問わず | **停止。無効化不可** | `Spec-safety.md` §2.1 |
| `MANUAL` | `OUT` | 警告のみ（`auto_brake` が ON なら停止） | 同（既定 OFF） |
| `MANUAL` | `IN` | 停止（`auto_brake` 既定 ON。試験員が OFF にできる） | 同（**既定であって固定ではない**） |
| `MANUAL` | `NA` | 停止 | `OPCHECK` / `CALIB` は極低速で人が張り付いている |

**開発モードでも `AUTO` の障害物停止は無効化できない**（`Spec-safety.md` §10）。

### 3.3.1 速度上限は「画面由来」と「モード由来」の最小値

上限の出どころは 2 系統ある（画面＝[names](DetailedDesign-names.md) §4、モード＝`attributes.yaml`）。
`OPCHECK` は S-30 と S-31 の両方に出るなど、1 対 1 ではない。

```
applied_limit_mps = min( 画面由来の上限, モード由来の上限, v_reverse（後退時） )
```

**迷ったら低いほうを採る**（L7 と同じ向き）。`AT_PANEL` のジョグだけは `v_jog_panel` を明示的に選ぶ。

### 3.4 `DEBT-4` を塞ぐ（後段配置の前提条件）

| # | 対処 | 置き場所 |
| --- | --- | --- |
| 1 | **`obstacle_limiter` は 20 Hz の固定レートで publish する。**入力 `/cmd_vel_muxed` が `muxed_stale_ms` 以上途絶したら**明示的にゼロを出す**（沈黙しない） | `obstacle_limiter` |
| 2 | **`esp32_bridge` に `/cmd_vel` の staleness タイムアウトを追加する。**`cmd_vel_stale_ms` を超えたらキープアライブの参照値をゼロに書き換える | `esp32_bridge.py` |
| 3 | `obstacle_limiter` は `/safety/limiter_status` を 20 Hz で publish する（heartbeat 兼用） | `obstacle_limiter` |
| 4 | `safety_monitor` が `/safety/limiter_status` の途絶を監視し、**重大フォルト**として `ESTOP` へ落とす | `safety_monitor` |

対処 2 は `obstacle_limiter` の有無に関わらず**現在すでに開いている穴**である。
`twist_mux` はロック中に無出力（silent）になりうる実装で、そのとき `/cmd_vel` は更新されない。
既存コードは `/safety/estop` `/safety/fault_lock` を直接見ることで**ロック時だけ**これを回避しているが、
**「上流プロセスが落ちた」場合は依然として最後の非ゼロ指令が送られ続ける。**

### 3.4.1 手動指令のゲート（`jog_gate`・**新設**）

**`/cmd_vel_manual` は WebUI が rosbridge から直接 publish する**（現行の流儀を維持）。
つまり `th_state` は経路上にいない。このままだと次の禁止が**「UI を非活性にする」だけの非構造的な担保**になる。

| 禁止 | 出典 |
| --- | --- |
| `SUMMON` / `WAIT_CLEAR` 中のジョグ | `F-28`「ゲートが止めているはずの状況で機体が動く」 |
| `OPCHECK` / `CALIB` 中のジョグ | `Spec-modes.md` §3.1.1 の除外 |
| `IDLE` / `INIT` / `ESTOP` / `CARRY` 中のジョグ | 同上 |

**UI を差し替えれば動いてしまう**ので、構造的に塞ぐ。

```
WebUI ──► /cmd_vel_manual_raw ──► [ jog_gate ] ──► /cmd_vel_manual ──► twist_mux (30)
                                       ▲
                                  /system/state（jog_allowed 相当）
```

| 項目 | 仕様 |
| --- | --- |
| ノード | `th_safety` の `jog_gate`（C++） |
| 通す条件 | `/system/state` が新鮮、かつ `attributes[mode].jog != "denied"`、かつ `(mode, state)` が除外に当たらない |
| **通さないとき** | **publish を止める（沈黙する）。ゼロを撃たない** |
| **`/system/state` 途絶** | **沈黙**（安全側） |
| レート | **入力駆動**（`/cmd_vel_manual_raw` を受けたときだけ出す）。固定レートで撃たない |
| `effect: disable_jog_ui` | UI の非活性化と**同時に** `jog_gate` にも届く。表示と実体を一致させる |

> **`jog_gate` にゼロを撃たせてはいけない。**
> `/cmd_vel_manual` は twist_mux の **priority 30（最高）**である。
> 20 Hz でゼロを出し続けると `manual_joy` が**常に非タイムアウト**になり、
> **twist_mux は priority 20 / 10 を永久に選ばなくなる**
> ——自律走行と Nav2 の指令が構造的に一切出力されない。
>
> **「沈黙禁止」を課すのは多重化の後段だけ**（`obstacle_limiter` と `esp32_bridge`）。
> 多重化の**前段**にいるノードは、[transit](DetailedDesign-transit.md) §0.1 B2 と同じく
> **止めるときは黙る**。止めるのは twist_mux のタイムアウトの役目である。

**`th_state` の `jog_allowed` ガードと `jog_gate` は同じ表を読む。**
`attributes.yaml` を両者が参照し、判定を二重に持たない。

### 3.4.2 入力の同期

`obstacle_limiter` は **20 Hz の固定ループ**で、各入力の最新値を保持して評価する。

| 入力 | レート | stale の閾値 | stale のときの扱い |
| --- | --- | --- | --- |
| `/cmd_vel_muxed` | 不定 | `muxed_stale_ms` | **出力ゼロ**（`action = ZERO_STALE`） |
| **`/scan`** | 約 10 Hz | **`scan_stale_ms`** | **`action = STOP`。**古いスキャンで「空き」と判定しない |
| `/cmd_vel_manual` | 不定 | `manual_joy_timeout` | `source_class = AUTO` |
| `/system/state` | 10 Hz | `state_stale_ms` | 最も低い上限・`auto_brake = ON`・`source_class = AUTO` |
| **`/safety/estop`** | 10 Hz | `lock_stale_ms` | **ロック扱い＝出力ゼロ**（L6） |
| **`/safety/fault_lock`** | 10 Hz | `lock_stale_ms` | 同上 |

**全入力について「一度も受信していない」は stale と同じ扱いにする。**
起動直後にリミッタが素通しになるのを防ぐ。**`jog_gate` にも同じ規則を課す**
（ただし `jog_gate` の「安全側」は**沈黙**であって、ゼロではない → §3.4.1）。

**`scan_stale_ms` が無いと、LiDAR が止まった直後の `lidar_timeout_ms`（現状 2000 ms）のあいだ、
リミッタは古いスキャンで通し続ける。**ここが安全床の穴になる。

### 3.4.3 距離 → 許容速度

**二値の停止にしない。**`d_floor` の手前で急停止すると `CLAMP` が出ないまま `STOP` になる。

```cpp
// mapless_follow_core.py の mapless_target_speed() と同型（braking_distance の逆関数）
double v_allow(double nearest_m, const P& p) {
  double margin = nearest_m - p.obstacle_floor_distance_m;
  if (margin <= 0.0) return 0.0;                       // action = STOP
  return std::sqrt(2.0 * p.brake_accel_mps2 * margin); // action = CLAMP
}
out.linear.x = clamp_toward_zero(in.linear.x, std::min(v_allow(d, p), applied_limit_mps));
```

**停止と再開のヒステリシス**: 一度 `STOP` に入ったら、
`nearest_m ≥ obstacle_floor_distance_m + hysteresis_band_m` になるまで `STOP` を維持する。

**判定コーン**: 進行方向を中心に `obstacle_cone_half_width_rad`
（後退時は `obstacle_cone_half_width_reverse_rad`）。
既存 `is_path_blocked()` は **bool しか返さない**ので、
**最近傍距離を返す形に拡張して移植する**（`LimiterStatus.nearest_obstacle_m` に要る）。
拡張後、既存 Python 実装との等価性テストを書く。

### 3.5 不変条件（property test にする）

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **L1** | `\|out.linear\| ≤ \|in.linear\|` かつ `\|out.angular\| ≤ \|in.angular\|`、符号は保存 | **リミッタは絶対に速度を上げない。**後段配置の安全性はこの 1 本に依存する |
| **L2** | 入力が stale → `out = 0`。**沈黙しない** | §3.4 |
| **L3** | **前方障害物では `linear.x` のみクランプし、`angular.z` は殺さない。ただし `d_floor` を割っている間は `angular.z` にも `w_align_max` を掛ける** | 角速度まで 0 にすると `ALIGN`（超信地旋回）と「人が手動操作で避ける」（`Spec-safety.md` §2.1）が実行不能になり、壁際でデッドロックする。一方、上限が無いと障害物へ振り込める |
| **L4** | `out.linear.x < 0` のとき `\|out.linear.x\| ≤ v_reverse` | `Spec-safety.md` §2.3。LiDAR 死角 |
| **L5** | 死角セクタが非ゼロ幅で存在する間は、**`v_reverse` を全方向の上限にする** | §4.3 |
| **L6** | ロック中（`/safety/estop` or `/safety/fault_lock`）は無条件にゼロ | 二重化 |
| **L7** | **`AUTO` 政策は決して緩まない。**分類に迷いがある入力は `AUTO` に倒す | §3.2 |

### 3.6 実装形態

**C++（`th_safety`）。**理由: 20 Hz の固定レートで `/cmd_vel` を出す最終段であり、
GC ポーズの影響を受けたくない。判定ロジックは `obstacle_limiter_core.hpp/cpp` に
**ROS2 非依存の純粋関数**として切り出し、gtest でテストする。

`is_path_blocked` の考え方は `mapless_follow_core.py:119-157` にある実装をそのまま移植する
（コーン走査・レンジ有効性・NaN の扱い）。**ただし人物除外は移植しない**（後段は対象を知らない）。

---

## 4. 障害物検知の入力

### 4.1 挙動ノード側は `/scan_filtered`

死角マスク済み。追従対象の脚を除外する処理（`person_exclude_*`）もここで効く。

### 4.2 `obstacle_limiter` は生 `/scan` を使う

**マスク済みの `/scan_filtered` を使ってはいけない。**

既存の `is_path_blocked` は `inf` / `nan` を「障害物なし」として扱う（`mapless_follow_core.py:143-144`）。
死角マスクは該当セクタを `inf` にするので、**死角対策の当事者であるリミッタが死角を「空き」と読む。**
`Spec-safety.md` §2.3 が想定している事故そのもの。

### 4.3 死角は第 3 の扱い

| セクタの状態 | リミッタの扱い |
| --- | --- |
| 有効なレンジ値がある | 距離で判定 |
| `inf` / `nan`（本当に何も無い） | 空き |
| **`blind_angle_ranges` に入る角度** | **未知。**その方向へは `v_reverse` 以下に制限する |

`obstacle_limiter` は `blind_angle_ranges` を**自分のパラメータとして持つ**
（`lidar_filter` と同じ値を `registry.yaml` から引く）。

### 4.4 死角マスクが未校正なら自律走行を拒否する

**`DEBT-2` への構造的な対処。**

```
blind_angle_ranges の全ペアが幅ゼロ  →  obstacle_limiter は AUTO の走行開始を拒否する
```

検出は始業点検の項目 4（`Spec-checks.md` §2.3）、是正は校正の項目 4（同 §3.4）。
**どちらも既に仕様にある**ので、新しい仕組みを作る必要はない。
拒否は `/safety/limiter_status.action = "BLOCKED_UNCALIBRATED"` で伝え、
`th_state` が `reject_reason_key = "blind_mask_uncalibrated"` を返す。

---

## 5. フォルトの 2 階級

### 5.1 分類（`F-20`）

| 階級 | 該当するもの | 落ちる先 | 解除後 |
| --- | --- | --- | --- |
| **`RECOVERABLE`** | `LIDAR_LOST` / `ESP32_DISCONNECTED` / `PERSON_TRACKER_LOST` / `UI_DISCONNECTED` | モードを保持し状態を `PAUSE` へ | 元モードの `run_state` または `PAUSE` |
| **`CRITICAL`** | `LIMITER_DEAD` / `MUX_DEAD` / `DRIVE_RUNAWAY` / `STATE_INCONSISTENT` / **`LOCALIZATION_LOST`** | `ESTOP` | `IDLE` のみ |

**必須通信系統の断を `CRITICAL` に含めない。**含めると最も起きやすいフォルトが毎回 `ESTOP` に落ち、
解除が `IDLE` のみになって `Spec-safety.md` §5 の再開フローが丸ごと死ぬ。
通信断で駆動が止まることは層 2・層 3 が別途担保している。

### 5.2 `safety_monitor` の変更点

| 項目 | 現行 | 新設計 |
| --- | --- | --- |
| `FaultStatus` | `active` / `fault_type` / `description` | **`severity` を追加** |
| **`/safety/fault_lock`** | `LIDAR_LOST \|\| ESP32_DISCONNECTED` | **`LIDAR_LOST \|\| ESP32_DISCONNECTED \|\| severity == CRITICAL`。**§5.2.1 |
| 監視対象 | `/scan` / `/person/status` / `/esp32/wheel_feedback` | `/scan` / **`/person/targets`** / `/esp32/wheel_feedback` ＋ **`/safety/limiter_status`**（重大） ＋ **`/map_session/status`**（自己位置喪失・重大） |
| タイムアウト | 固定値 | **導出値**（§7） |
| リンク品質 | 単発警告のみ | **`/safety/link_quality` として p50/p99/max を publish** |

> `PersonStatus.msg` は廃止して `PersonTargets` に統合するので、
> **`safety_monitor.cpp` の include とサブスクリプションも同じ作業パケットで差し替える**
> （片方だけ進めると `th_safety` がビルド不能になる）。

### 5.2.1 重大フォルトは層 3 で止める（**致命的な穴だった**）

初版は重大フォルトを `/safety/fault_lock` に載せず、
**止めるのが層 4（`th_state` の `ESTOP` 遷移）だけ**になっていた。
`Spec-safety.md` §1 は層 3 を「**モード管理の処理を待たずに** 100 ms 以内」と定めており、
これでは §10 の試験 11（リミッタの死 → **駆動ゼロ**）を自分で満たせない。

```
/safety/fault_lock = LIDAR_LOST || ESP32_DISCONNECTED || (severity == CRITICAL)
```

**`th_state` の `ESTOP` 遷移は表示と復帰のためだけ**であり、駆動を止める役目は持たない。

### 5.3 自己位置喪失を重大フォルトに加える（`DEBT-5`）

`Spec-safety.md` §3.5 の重大フォルト列挙に無いが、**「原因が解消してもそのまま続けると危険が残る」の定義に合致する。**
`map→odom` が凍結したまま Nav2 が走ると、機体は誤った自己位置に基づいて動き続ける。

検出: `slam_control` 相当（`th_route` の `map_session`）が
`map→odom` の更新途絶と slam_toolbox プロセスの死を検出して `evt` ではなくフォルトとして上げる。

> **申し送り**: `Spec-safety.md` §3.5 の重大フォルト列挙に `LOCALIZATION_LOST` を足すべき。
> [DetailedDesign-open.md](DetailedDesign-open.md) の申し送り表へ。

### 5.4 人物ロストは 3 段になる

**現状は 1 つしかない。**用途で猶予が違う（`C-05`）ので明示的に分ける。

| 段 | パラメータ | 用途 | 誰が持つか |
| --- | --- | --- | --- |
| 1 | **猶予なし（0）** | 登録（座標を確定する） | `th_onsite` の `pin_registrar` |
| 2 | `tracker_lost_grace_ms`（**(c)・`placeholder`**。`blocking_from_stage: 4`） | 追従・呼び寄せの停止判断 | 挙動ノード |
| 3 | `person_timeout_ms`（**(b)・derived**。`person_backstop_ms()`） | `safety_monitor` の遅いバックストップ | `safety_monitor` |

`0 < tracker_lost_grace_ms < person_timeout_ms` を起動時にアサートする。

---

## 6. 非常停止（`O-d7` の解決）

`Spec-open.md` `O-d7`「UI 非常停止ボタンの内部動作と物理 E-Stop との二重化」は
**詳細設計へ送られていた**ので、ここで決める。

### 6.1 系統

```
[物理スイッチ] ─GPIO32─► ESP32 ─ESTOP_HW(0x03)─► esp32_bridge ─► /safety/estop_hw ─┐
                    │                                                              ├─ OR ─► /safety/estop
                    └─（電気的にモータードライバ電源を遮断。ROS2 に依存しない）      │
                                                                                    │
[WebUI ボタン] ─rosbridge─► /safety/estop_ui ────────────────────────────────────────┘
                                                                                    │
                                                    ┌───────────────────────────────┴──┐
                                              twist_mux lock 255              esp32_bridge 独立ロック
                                                                                    │
                                                                            th_state → ESTOP
```

### 6.2 UI 非常停止は `safety_monitor` を経由する（直結しない）

**WebUI から `/cmd_vel` や `th_state` へ直接効かせない。**

| 理由 | |
| --- | --- |
| 単一の集約点を保つ | 物理と UI の OR を 1 か所でとる。二重化の意味が出る |
| ロックの発行者を 1 つにする | `twist_mux` のロックトピックを複数ノードが叩くと、解除の責任者が消える |
| 状態機械を安全経路から外す | `th_state` は `/safety/estop` を**購読して追従する**だけ。FSM が固まっても駆動は止まる |

### 6.3 UI ボタンの生存確認

**UI が落ちても押しっぱなしにならないようにする。**

| 項目 | 仕様 |
| --- | --- |
| 送信 | WebUI は `/safety/estop_ui` を**押下中は 2 Hz で送り続ける**（`true` のラッチではなく継続送信） |
| 解除 | `false` を明示送信。**かつ** `estop_ui_lease_ms` の途絶でも `false` にはしない — **押下側にラッチする** |
| 非対称の理由 | 「押した」の取りこぼしは危険、「解除した」の取りこぼしは安全。**安全側に倒すなら押下をラッチする** |
| 解除の手段 | UI の解除ボタンのみ（下端に固定・小さめ・色と形を変える。`Spec-webui.md` §1.2） |

### 6.4 `CARRY` 中は UI 非常停止を受け付けない（`F-26`）

`th_state` が `reject_reason_key = "estop_disabled_in_carry"` を返す。
**`safety_monitor` 側では止めない**（安全経路に条件分岐を入れない）。
ボタンは隠さず、効かないことが分かる見た目にし、W-2 に「駆動は既に切れています」と出す。

### 6.5 起動時に押されたまま（`CL-B-6`）

`T-INIT-03`（[DetailedDesign-state.md](DetailedDesign-state.md) §4.2）。
`CARRY` へ落とさず `INIT/CHECK` に留まり、「非常停止ボタンが押されています。解除してください」を出す。
解除するまで `evt.link_ok` を出さない。

---

## 7. タイムアウトは導出値にする（`DEBT-3`）

**2000 ms は不注意ではない。**`safety_monitor.yaml:16-19` のコメントに理由が記録されている
（WiFi 経由の受信ギャップが平常時 0.5〜1.2 s。500 ms では誤フォルトが頻発した）。
根本原因はリンク品質であり、`Spec-params.md` §8 も「WiFi リンクの改善は速度を上げる全案の前提条件」と書いている。

**「100 ms にせよ」は誤った処方である。**代わりに 2 本の制約を同時にかける。

```
① timeout ≥ p99(受信ギャップ) + 余裕          ← 誤フォルトを出さない
② v_max × (timeout / 1000) ≤ intrusion_budget_m ← タイムアウトまでに走ってよい距離
```

**② を満たせないなら `v_max` を下げる。**タイムアウトを縮めて誤フォルトを増やすのではない。
判定は `th_params` の起動時アサーションで行い、**満たさなければ起動を拒否するか `v_max` をクランプする**
（[DetailedDesign-params.md](DetailedDesign-params.md)）。

`p99` は `/safety/link_quality` の実測から入れる。**実測が無い間は `status: placeholder`** になり、
ヘッダにバッジが出る。

---

## 7.1 加減速はファームウェアで鈍らせる（`Spec-safety.md` §3）

**「上位が急な指令を出しても機体側で鈍らせる」**が正本の規定である。
PC 側の `rate_limit()`（`follow_core.py`）は**追従にしか効かない**ので、これだけでは足りない。

| 層 | 実装 | 効く範囲 |
| --- | --- | --- |
| **ESP32 ファーム** | `TARGET_RAMP_ACCEL_MPS2`（既存）を PID の手前で適用 | **すべての速度源**（ジョグ・Nav2・挙動ノード） |
| PC 側の挙動ノード | `rate_limit()` | その方式の中だけ。**滑らかさのためであって安全のためではない** |

| 項目 | 決定 |
| --- | --- |
| 権威 | **ファーム側。**`TARGET_RAMP_ACCEL_MPS2` を `registry.yaml` の写しとして持ち、A6 と同じく突き合わせる |
| 前提 | **重心が高くなることが予想される**ので、加減速はゆっくり（`Spec-safety.md` §3） |
| 制動距離との関係 | `brake_accel_mps2`（実測）はこのランプを含んだ**実際の止まり方**である。ランプを変えたら**測り直す** |

> **`obstacle_limiter` は加減速を鈍らせない。**上限をクランプするだけ（L1）。
> リミッタでランプを掛けると、停止指令まで鈍って制動距離が伸びる。

---

## 7.2 バッテリー（`Spec-safety.md` §8）

| 項目 | 仕様 |
| --- | --- |
| 低下時 | **警告を出す** |
| **自動で止めるか** | **止めない。**最終的に試験員が判断する |
| 警告が無いこと | **長時間の動作を保証しない** |
| 表示 | ヘッダに残量。`battery_warn_v` を下回ったら警告色。S-01 にも出す |
| トピック | `/esp32/battery`（`sensor_msgs/BatteryState`）。**ESP32 のフレームに `BATTERY (0x06)` を追加する** |
| パラメータ | `battery_warn_v` / `battery_critical_v`（ともに given）／ `battery_endurance_min`（(c)） |

**`battery_critical_v` を下回っても停止させない。**警告の色と文言を変えるだけ。
`Spec-safety.md` §8 が「させない」と明記しており、走行中に勝手に止まるほうが危険な場面がある。

**未確定**（`O-c7` / `CL-B-4`）: 200 m 往復＋試験場内を 1 日持つのか。充電の扱い。
目標 G5 の検証項目であり、`WP-MEAS-05` で測る（[packets](DetailedDesign-packets.md) §3）。

---

## 8. 通信断

### 8.1 停止する条件

| 条件 | 挙動 |
| --- | --- |
| 必須 3 者（ESP32 / RaspberryPi4 / PC）のいずれかが切れた | 回復フォルト → `PAUSE`。層 2・3 が駆動をゼロに |
| **実際に操作に使われていた端末**が切れた | 同上（`UI_DISCONNECTED`） |
| 接続されていただけの端末 | 停止しない |

### 8.2 「使用中」の判定（`E-7`）

`ActiveScreen.interacting` が真、かつ最後の操作から `ui_active_window_s` 以内。
画面を開いているだけ・見ているだけは含まない。
**複数端末が使用中なら、そのいずれかが切れたら停止する**（安全側）。

### 8.3 Wi-Fi AP は単一障害点

必須通信系統の定義に AP を**含めない**（`C-01`）。AP 障害は 3 者全滅として検出できるので監視上は等価。
S-00 とヘッダに「Wi-Fi AP ＝単一障害点」を明示する。
フォールバックは**手押し**（実装ゼロで成立する正式なフォールバック）。

---

## 9. 「待つ」ときの打ち切り（`F-11`）

| 待ちの種類 | タイムアウト | 実装 |
| --- | --- | --- |
| **人が動くのを待つ** | **する** | `wait_clear_gate` の `clear_timeout_ms` → `evt.clear_timeout` → `POINT` へ戻る |
| **経路が通れるようになるのを待つ** | **しない** | `BLOCKED` 状態。`evt.unblocked` か `ui.reroute` でしか出ない |
| **機器が繋がるのを待つ** | **する**（起動時） | `sys.link_timeout` → 制御系を再起動 |

---

## 10. 故障注入試験（`Spec-safety.md` §9）

**「接触 0 件」は数えて確かめるものではない。意図的に壊して止まることを確かめる。**
各行を Gazebo と実機の両方で自動化し、`th_testing` に置く。

| # | 検証項目 | 方法 | 合格条件 | 自動化 |
| --- | --- | --- | --- | --- |
| 1 | 障害物リミッタ（前進） | 障害物へ向けて走らせる | 停止する | Gazebo |
| 2 | **障害物リミッタ（後退）** | 同上を後退で | 停止する（`CL-X-4`） | Gazebo |
| 3 | 非常停止（UI） | 走行中に押す | 停止する | Gazebo ＋ 実機 |
| 4 | 非常停止（物理） | 走行中に押す | 停止する | **実機のみ**（通電必要） |
| 5 | **通信断 → フォルト検知** | LiDAR / ESP32 の通信を切る | `lidar_timeout_ms` / `esp32_timeout_ms` 以内にフォルトが立つ | Gazebo ＋ 実機 |
| 6 | **フォルト検知 → 停止** | 5 の**続き**を測る | フォルトから **100 ms 以内**に速度指令が 0 | Gazebo ＋ 実機 |
| 7 | 呼び寄せの退避待ち | 退かずに待つ | 発進しない → タイムアウトで中止 | Gazebo |
| 8 | ESP32 ウォッチドッグ | ROS2 側を落とす | 600 ms 以内に停止 | **実機のみ** |
| 9 | 人検知 OFF とフォルトの区別 | 意図的に OFF | フォルトにならない・強制遷移しない | Gazebo |
| 10 | 物理ボタン起動時押下 | 押したまま起動 | 運用に入れず解除を案内 | **実機のみ** |
| **11** | **リミッタの死** | `obstacle_limiter` を SIGKILL | **重大フォルト → `ESTOP` → 駆動ゼロ**（`DEBT-4`） | Gazebo ＋ 実機 |
| **12** | **`/cmd_vel` の途絶** | `obstacle_limiter` を止めたまま非ゼロ指令を残す | `cmd_vel_stale_ms` 以内に ESP32 への指令がゼロ | Gazebo ＋ 実機 |
| **13** | **自己位置喪失** | slam_toolbox を SIGKILL | 重大フォルト → `ESTOP` | Gazebo |

**5 と 6 を分ける理由**（`F-21`）: 1 行で「通信を切ってから 100 ms 以内に停止」と書くと、
`lidar_timeout_ms` = 2000 ms の現状では**正常な実装でも必ず不合格**になる。
100 ms は層 3 の応答時間であって通信断からの合計ではない。
**合計（通信断 → 停止）が実際の危険量**であり、それが §7 の ② に入る。

```bash
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R fault_injection
```

---

## 11. 安全負債の解除手順

### 11.1 `DEBT-1` 物理 E-Stop バイパス

| 順 | 対処 |
| --- | --- |
| 1 | **ランタイムで検出可能にする。**ESP32 が `ESTOP_HW (0x03)` フレームに**ファーム構成フラグ**を載せる（`bypass_active` ビット）。既存の「ファーム世代検出」（`esp32_bridge.py:328-343` の `_feedback_has_dt`）と同じ流儀 |
| 2 | `safety_monitor` は通常モードでバイパス検出を**重大フォルト**として扱い、運用に入れない |
| 3 | 開発モードではヘッダに常時表示（`Spec-webui.md` §5 の色替えに相乗り） |
| 4 | **合格判定**: 始業点検 項目 1（押す／離すの両方を検出。`Spec-checks.md` §2.4）は**バイパス有効では構造的に通らない**。よって「**項目 1 が OK になったことをもってバイパス無効の証明とする**」 |

**検証手段が既に仕様の中にある**ので、新しい試験を作る必要はない。

### 11.2 `DEBT-2` 死角マスク

§4.4 のゲート（未校正なら自律走行を拒否）＋ **校正の項目 4 だけを段階 2 に前倒しする**。
保守機能全体（`OPCHECK` / `CALIB`）は後の段階だが、この 1 項目は
`obstacle_limiter` の着手前提条件なので先に作る。

### 11.3 `DEBT-3` タイムアウト

§7 の導出化 ＋ リンク品質計測ノード ＋ 起動時アサーション。
`task.md` の「1. ネットワークの改善」と同じ作業なので**同時に片づける**。

### 11.4 併せて片づける（安全ではないが同じ場所を触る）

| 事実 | 対処 |
| --- | --- |
| `th_ws/esp32/src/wifi_credentials.h` が実 SSID・実パスワードを含んだままコミットされている。ポートも `8765` で `params.yaml`（8766）と不一致 | `.gitignore` に入れ、履歴から除去するか無効な値に置換。`.example` を正とする |
| `esp32/src/ws_link.h` のフレーム表が古い（`WHEEL_FEEDBACK` を 9 byte と書くが実装は 13 byte） | 表を実装に合わせる。**プロトコルを触る作業パケットの冒頭で直す** |
| `esp32/tools/ws_test_server.py:86` が `unpack_wheel_feedback` の 3 要素返却に追従しておらず `ValueError` で落ちる | 修正するか削除 |
| `main.cpp:196-202` の `[DBG]` printf が「原因切り分け後に削除すること」のまま残っている | 削除するか、開発モードのログ選択に載せる |

---

## 12. 開発モードの境界

| 無視できる | **無視できない** |
| --- | --- |
| 機器未接続の警告 | **物理非常停止ボタン**（層 1） |
| バッテリー電圧低下の警告 | **ESP32 のウォッチドッグ**（層 2） |
| 始業点検の総合ステータス NG | **UI 非常停止ボタン** |
| 必須通信系統の不通による運用開始の禁止 | **`AUTO` の障害物停止**（`obstacle_limiter`） |
| 手動系の自動ブレーキ（場内でも OFF 可） | **`DEBT-1` バイパス検出の重大フォルト**（表示は消せるが駆動は許可しない） |
| パラメータ暫定値のバッジ | **§7 ② の起動時アサーション** |

**層 1・2 は開発モードでも無効化できない**（`Spec-safety.md` §10）。
実装上は、開発モードのフラグを `safety_monitor` と `obstacle_limiter` に**渡さない**ことで構造的に保証する。
警告の抑止は WebUI と `th_state` の層でだけ行う。

> **未確定**（`O-d6`）: 誰が有効化できるか（権限）、通常運用中に誤って有効化されないための対策。
> [DetailedDesign-open.md](DetailedDesign-open.md) へ。

---

## 13. 逆引き

| 完全設計書 | ここでの反映 |
| --- | --- |
| `Spec-safety.md` §1（4 層） | §1 |
| §2.1（方式ごとの障害物） / `C-11` / `U-2` | §3.3 |
| §2.2（停止距離の式） | §3.1・[DetailedDesign-params.md](DetailedDesign-params.md) |
| §2.3（後退は死角） / `CL-X-4` | §3.5 L4・§4.3 |
| §2.4（リミッタは新機能） / `CL-X-3` | §3 |
| §3.5（フォルト 2 階級） / `F-20` | §5 |
| §4（非常停止 2 系統） / `F-26` / `O-d7` | §6 |
| §5（異常ウィンドウ） | [DetailedDesign-state.md](DetailedDesign-state.md) §7.1 ＋ [DetailedDesign-webui.md](DetailedDesign-webui.md) |
| §6（通信断） / `E-7` / `C-01` | §8 |
| §7（待ちの打ち切り） / `F-11` | §9 |
| §9（故障注入） / `F-21` | §10 |
| §10（開発モード） | §12 |
