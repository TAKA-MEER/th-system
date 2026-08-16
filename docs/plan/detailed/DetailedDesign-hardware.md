# ハードウェア構成の実装対応

[DetailedDesign.md](DetailedDesign.md) の詳細。
**`Spec.md` §6.1 のハードウェア構成表（9 項目）を、ノード・トピック・フレームに繋ぐ。**

> **`F-18`**（`T-r11` / `T-r12` / `DF-D-7`）: カメラとリードデバイスは原典の構成表に無く、
> 完全設計書で追加された。**RaspberryPi4 は LiDAR と USB 帯域を共有する**という注意事項も同時に入った。
> **詳細設計にはこれに対応する節が無かった**ので新設する。

**このファイルは機材の手配とファームの改修の唯一の索引である。**
リードタイムのある買い物（§5）と、ファームのフレーム追加（§3）がここに集まる。

---

## 1. 機器 → ノード → トピック

| # | 機器 | 接続 | 読むノード | 出るトピック | 段階 |
| --- | --- | --- | --- | --- | --- |
| 1 | **Wi-Fi AP** | — | — | — | **単一障害点。必須通信系統に数えない**（`C-01`） |
| 2 | **PC** | Wi-Fi | 全ノード | — | **機体に載せない・有線接続もしない** |
| 3 | **ESP32** | Wi-Fi / WebSocket | `esp32_bridge` | `/odom` ／ `/esp32/wheel_feedback` ／ `/esp32/wheel_cmd_speed` ／ `/esp32/imu_data` ／ `/esp32/imu_calib_status` ／ `/safety/estop_hw` ／ **`/safety/firmware_flags`** ／ **`/esp32/battery`** | 既存 ＋ 段階 2 |
| 4 | **RaspberryPi4** | Wi-Fi | （RPi4 上で `rplidar_ros`） | `/scan` | 既存 |
| 5 | **LiDAR**（RPLIDAR S1） | RPi4 に **USB** | 同上 | `/scan` → `lidar_filter` → `/scan_filtered` | 既存 |
| 6 | **IMU**（BNO055） | ESP32 に GPIO | `esp32_bridge` | `/esp32/imu_data` → `ekf_filter_node` | 既存 |
| 7 | **CuGoV3**（モーター×2・エンコーダ×2） | ESP32 に GPIO | `esp32_bridge` | `/esp32/wheel_feedback` | 既存 |
| **8** | **カメラ**（`LINE` 用・前方 1 台） | **§4 で決める** | `line_runner` | `/line/status` | **段階 8**（手配は §5） |
| **9** | **リードデバイス**（Arduino） | RPi4 に **USB** | `leash_runner` | `/leash/status` | **段階 8** |

**`obstacle_limiter` は 5 の生 `/scan` を使う**（`/scan_filtered` ではない。
死角マスクが `inf` になり「空き」と読まれるため。[safety](DetailedDesign-safety.md) §4.2）。

### 1.1 フレーム

| frame_id | 由来する機器 | 発行者 |
| --- | --- | --- |
| `base_link` | 機体 | URDF |
| `laser_link` | 5 LiDAR | URDF。**`obstacle_limiter` は起動時に 1 度だけ固定変換を取得して保持する** |
| `imu_link` | 6 IMU | URDF |
| `odom` | 3+7 のホイールオドメトリ ＋ 6 のジャイロ | **`ekf_filter_node` だけ**（不変ルール） |
| `map` | 5 → slam_toolbox | `map_session` |
| **`camera_link`** | 8 カメラ | **URDF に追加が要る**（段階 8。`th_description`） |

**9 リードデバイスは幾何を持たない**（張力の有無と向きだけ）。TF に載せない。

---

## 2. RaspberryPi4 の USB 帯域（**`F-18` の注意事項の実体化**）

**RPi4 の USB には LiDAR が既に居る。**リードデバイス（9）とカメラ（8）を足すと 3 つになる。

| 機器 | 帯域の性質 |
| --- | --- |
| LiDAR RPLIDAR S1 | **256000 baud のシリアル。**10 Hz × 全周。**途切れると `lidar_timeout_ms` でフォルト** |
| リードデバイス（Arduino） | シリアル。**数バイト × 低頻度。**影響は小さい |
| **カメラ** | **これが問題。**USB カメラは帯域を占有する |

### 2.1 判定は実測でしかできない

**「たぶん足りる」で進めない。**RPi4 の USB は内部ハブ経由で、実効帯域は仕様値より低い。

```bash
# WP-LINE-00 の測定手順（LiDAR を回しながら）
ros2 topic hz /scan                       # ベースライン（カメラ無し）
#   カメラを接続して取得を開始してから
ros2 topic hz /scan                       # 悪化していないこと
ros2 topic echo /safety/link_quality --once   # p99 が WP-MEAS-04 の実測から悪化していないこと
```

**合格条件**: `/scan` の受信間隔 p99 が、カメラ無しの実測値から **`link_quality_regression_ratio`** を超えて悪化しない。
この比は `given`（方針値）として `registry.yaml` に置く。

### 2.2 だから CSI を推奨する

[transit](DetailedDesign-transit.md) §6.3 が結論を出している（**① CSI ポート**）。
**USB 帯域を食わない唯一の選択肢**であり、上の測定で落ちる可能性が構造的に無い。

**それでも §2.1 の測定は行う。**CSI でも CPU と電力は食う。

---

## 3. ESP32 のフレーム（**このファイルが正**）

**現行 4 種に 2 種を足し、1 種を拡張する。**
散らばっていた決定（[state](DetailedDesign-state.md) §12.4 の LED、[safety](DetailedDesign-safety.md) §7.2 のバッテリー、
[wp2](DetailedDesign-wp2.md) `WP-ESP32-01` の構成フラグ）をここに集約する。

| type | 名前 | 向き | 内容 | 状態 |
| --- | --- | --- | --- | --- |
| `0x01` | `WHEEL_CMD` | PC → ESP32 | 左右速度指令 | 既存・**変更しない** |
| `0x02` | `WHEEL_FEEDBACK` | ESP32 → PC | 左右実速度 ＋ **`dt`**（13 byte） | 既存・**変更しない**。**旧 9 byte も受理する** |
| `0x03` | **`ESTOP_HW`** | ESP32 → PC | `[type:1][pressed:1]` **＋ `[flags:1]`** | **拡張**（`WP-ESP32-01`） |
| `0x04` | `IMU_DATA` | ESP32 → PC | BNO055 | 既存・**変更しない** |
| **`0x05`** | **`LED_STATE`** | PC → ESP32 | 起動完了の提示（`CL-B-8`） | **新設**（段階 1〜2） |
| **`0x06`** | **`BATTERY`** | ESP32 → PC | 電圧 | **新設**（段階 2） |

### 3.1 拡張と新設の規約

**すべて「旧形式を受理する」。**ファームと PC の更新順序を強制しない。

| フレーム | 旧形式が来たときの扱い |
| --- | --- |
| `ESTOP_HW`（2 byte） | **`flags = 0xFF`（不明）＝ バイパスの可能性ありとして安全側に倒す**（`WP-ESP32-01` E-1） |
| `WHEEL_FEEDBACK`（9 byte） | 既存の `_feedback_has_dt` の流儀のまま |
| `LED_STATE` / `BATTERY` | **来なければ来ないだけ。**フォルトにしない（`enabled_targets` に入れない） |

### 3.2 `LED_STATE (0x05)`

**現場で PC を見るとは限らない**（`CL-B-8`）。

| 項目 | 仕様 |
| --- | --- |
| 内容 | `[type:1][state:1]`。`0`＝起動中／`1`＝運用可（`evt.link_ok` 後）／`2`＝フォルト／`3`＝非常停止 |
| 送る側 | `esp32_bridge` が `/system/state` と `/safety/fault` から決めて **1 Hz ＋ 変化時**で送る |
| **ハードウェア** | **GPIO に LED を 1 個足す。**部品と工作が要る（§5） |
| **無くても動く** | LED が無い機体でも `LED_STATE` を送るだけ。ESP32 側は未実装なら無視する |

> **`WHEEL_CMD` に相乗りさせない。**指令フレームは 20 Hz のキープアライブ経路で、
> ここに表示用のデータを混ぜると `DEBT-4` の対処（stale 判定）が読みにくくなる。

### 3.3 `BATTERY (0x06)`

| 項目 | 仕様 |
| --- | --- |
| 内容 | `[type:1][millivolt:2]`（uint16。リトルエンディアン。既存フレームと同じ流儀） |
| レート | **1 Hz** |
| PC 側 | `esp32_bridge` → `/esp32/battery`（`sensor_msgs/BatteryState`） |
| **止めるか** | **止めない。**警告の色と文言を変えるだけ（[safety](DetailedDesign-safety.md) §7.2。`Spec-safety.md` §8） |
| パラメータ | `battery_warn_v` / `battery_critical_v`（ともに `given`）／ `battery_endurance_min`（**(c)**・`WP-MEAS-05`） |

### 3.4 GPIO

| 用途 | ピン | 備考 |
| --- | --- | --- |
| 物理非常停止 | **GPIO32**（`ESTOP_LOW_ACTIVE`） | 既存。**`ESTOP_BENCH_TEST_BYPASS` は無効化する**（`DEBT-1`） |
| モーター・エンコーダ | CuGoV3 の既定 | 既存・変更しない |
| IMU | I2C | 既存・変更しない |
| **状態 LED** | **未割当** | §3.2。**空きピンの選定は実機で行う**（`WP-ESP32-01` の作業中に決める） |
| **バッテリー電圧** | **未割当**（ADC ＋ 分圧） | §3.3。同上 |

> **物理非常停止はモータードライバ電源を電気的に遮断する。**
> `ESTOP_HW` フレームは**その事実を PC へ伝えるだけ**であって、停止の手段ではない
> （[safety](DetailedDesign-safety.md) §1 の層 1）。**ROS2 に一切依存しない。**

---

## 4. カメラの選定（`Spec.md` §6.1 #8「要検討（詳細設計）」の決着）

**原典が明示的に詳細設計へ送っている唯一のハードウェア判断。**

| 決定 | 内容 |
| --- | --- |
| **接続** | **CSI（RPi4 のカメラポート）**（[transit](DetailedDesign-transit.md) §6.3） |
| 台数・位置 | 前方 1 台。**進行方向の床面が画角に入る**こと（区画線と分岐を見る） |
| 要求 | ① 区画線と分岐が判別できる解像度 ② **屋内照明の変化に耐える**（自動露出） ③ **CSI 接続** |
| **決めない** | **具体的な型番。**`WP-LINE-00` で §2.1 の測定と合わせて選ぶ |
| 段階 | **手配は先**（リードタイム）。**実装は段階 8** |

**「決めない」を明記するのが決定である。**型番をここで書くと、
`WP-LINE-00` の測定が「選んだものの確認」になる（`DD-6` と同じ構造の誤り）。

### 4.1 画角と取り付けは `LINE` の内部設計に属する

**このファイルでは「CSI で 1 台」までしか決めない。**
取り付け高さ・俯角・キャリブレーションは `LINE` の内部設計（段階 8）で決める。
**先に決めると、実装が始まったときに必ず変わる。**

---

## 5. 手配（**リードタイムがあるもの**）

**買い物と工作は実装の前に始める必要がある。**

| # | 品目 | いつまでに | 誰が決めるか | 依存 |
| --- | --- | --- | --- | --- |
| 1 | **状態 LED ＋ 抵抗**（§3.2） | 段階 1 の実機確認まで | 実装側（安価・入手容易） | — |
| 2 | **分圧抵抗**（§3.3 バッテリー電圧） | 段階 2 まで | 同上 | バッテリーの公称電圧 |
| 3 | **カメラ（CSI）**（§4） | **段階 8 の着手前**。ただし**発注は早いほどよい** | **`WP-LINE-00`** | §2.1 の測定 |
| 4 | **リードデバイス（Arduino）**（§6） | 段階 8 の着手前 | **未着手**（`Spec-transit.md` §7 が後回しを宣言） | — |

**1 と 2 は段階 1〜2 の作業に含まれる**ので、**そこで初めて気づくと止まる。**
`WP-ESP32-01` の §1 に部品の確認を入れてある。

---

## 6. リードデバイス（`LEASH`・後回し）

**インターフェースだけ決める**（[transit](DetailedDesign-transit.md) §7）。

| 項目 | 仕様 |
| --- | --- |
| 機器 | Arduino。**RPi4 に USB** |
| 取るもの | **張力の有無**（bool）と**リードの向き**（角度） |
| トピック | `/leash/status`（reliable, depth 1） |
| 事象 | `evt.leash_present` / `evt.leash_absent` / `evt.leash_taut` / `evt.leash_slack` |
| **接続確認** | **`DEV_CHECK` 状態が存在する理由。**未接続なら走行開始を非活性にする（`T-LEASH-07`） |
| 帯域 | §2 のとおり影響は小さい |

**プロトコルは決めない。**機器が決まっていない（§5-4）。

---

## 7. 受け入れ条件

| # | 検証 | 手段 |
| --- | --- | --- |
| 1 | `ESTOP_HW` の旧形式（2 byte）が `flags = 0xFF` として扱われる | `test_esp32_ws_protocol.py`（`WP-ESP32-01`） |
| 2 | `LED_STATE` / `BATTERY` が来なくてもフォルトにならない | `test_safety_monitor`（`enabled_targets` に無い） |
| 3 | **`ws_link.h` のフレーム表が実装と一致する** | `WP-ESP32-01` §10-③ |
| 4 | `registry.yaml` の `esp32_watchdog_ms` が `config.h` の `WATCHDOG_MS` と一致 | **A6 / A7**（[params](DetailedDesign-params.md) §4） |
| 5 | **カメラを繋いだ状態で `/scan` の p99 が悪化しない** | §2.1（`WP-LINE-00`） |
| 6 | `camera_link` が URDF にあり TF が繋がる | 段階 8 |

---

## 8. 逆引き

| 完全設計書 | ここでの反映 |
| --- | --- |
| `Spec.md` §6.1（ハードウェア構成 9 項目） / **`F-18`** / `T-r11` / `T-r12` / `DF-D-7` | §1 |
| `Spec.md` §6.1 の注記（USB 帯域の共有） | §2 |
| `Spec.md` §6.1 #8「要検討（詳細設計）」 | **§4（決着）** |
| `Spec-safety.md` §4（非常停止 2 系統） | §3.4 |
| `Spec-safety.md` §8（バッテリー） | §3.3 |
| `Spec-ops.md` §2.4（起動完了の提示） / `CL-B-8` | §3.2 |
| `Spec-transit.md` §6（ライン誘導） | §4 ＋ [transit](DetailedDesign-transit.md) §6.3 |
| `Spec-transit.md` §7（電子リード） | §6 |
