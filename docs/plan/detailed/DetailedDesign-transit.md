# 走行方式の実装

[DetailedDesign.md](DetailedDesign.md) の詳細。**保管場所⇔試験場の 7 方式。**

**`LINE`（ライン誘導走行）と `LEASH`（電子リード走行）は §6・§7 でインターフェースと
受け入れ条件だけを確定させ、内部設計は後回しにする**（ユーザー決定）。
遷移表・名前辞書・画面は他方式と同じ粒度で確定済みなので、後から内部を埋めれば繋がる。

---

## 0. 挙動ノードの共通形

**すべての挙動ノードはこの形に従う。**例外を作らない。

```python
class BehaviorNode(Node):
    MY_MODE = "FOLLOW"          # 担当するモード
    ACTIVE_STATES = {"RUN"}     # このときだけ速度指令を出す

    # sub: /system/state (transient_local)
    # pub: /cmd_vel_behavior (Twist), /system/event (StateEvent)
    def on_tick(self):          # update_rate_hz
        if st.mode != self.MY_MODE or st.state not in self.ACTIVE_STATES:
            self._release()      # publish を止める。ゼロを撃たずに黙る
            return
        out = self.core.update(...)     # 純粋コアを呼ぶだけ
        self.pub_cmd.publish(out.twist)
```

### 0.0 `follow_runner` の担当範囲（**ここが唯一の定義**）

`follow_runner` は 3 モードで使い回す。**他の節でこの値を書き直さない。**

```python
MY_MODE       = {"FOLLOW", "TEACH_FOLLOW", "PREP"}
ACTIVE_STATES = {"CONFIRM", "RUN", "REC", "MAPPING"}
```

| モード | 走る状態 | 走り出す条件 |
| --- | --- | --- |
| `FOLLOW` | `CONFIRM`（旋回のみ）／ `RUN` | `SELECT` → `ui.confirm` → `ui.run` で対象が確定している |
| `TEACH_FOLLOW` | `CONFIRM` ／ `REC` | 同上（記録も同時に始まる） |
| **`PREP`** | **`MAPPING`** | **`PREP` に `CONFIRM` は無い**ので、代わりに**ノード側でゲートする**（下記） |

**`PREP/MAPPING` で走り出す条件**（`CONFIRM` の代わり）:

```
target_selected かつ confidence ≥ follow_start_min_confidence が
follow_start_hold_ms 継続してから、はじめて速度指令を出す
```

`Spec-modes.md` §9 は `PREP` の人物追跡を「登録時のみ要」としているが、
**連れ回し（手順 5）でも追従はする。**対象確定のゲートが無いまま追従を始めると、
別人や椅子の脚を追って地図を汚す。**状態遷移は増やさず、ノード側の条件で塞ぐ。**

### 0.1 4 つの規約

| # | 規約 | なぜ |
| --- | --- | --- |
| **B1** | **ROS2 非依存の純粋コア＋薄いノード。**アルゴリズムは `*_core.py` に置き、ノードは配線だけ | `follow_planner_core.py`（422 行・28 テスト）の流儀。pytest が ROS2 なしで回る |
| **B2** | **`ACTIVE_STATES` 以外では publish を止める。**ゼロ Twist を撃たない | 止めるのは `twist_mux` のタイムアウト（0.5 s）と `obstacle_limiter` の役目。ゼロを撃つと `manual_joy` との競合で挙動が読めなくなる |
| **B3** | **状態遷移を自分で決めない。**`evt.*` を `/system/event` へ出すだけ | 遷移の正本は `transitions.yaml`（[DetailedDesign-state.md](DetailedDesign-state.md)） |
| **B4** | **数値リテラルを書かない。**すべて `registry.yaml` 由来 | [DetailedDesign-params.md](DetailedDesign-params.md) |

### 0.2 障害物判定は挙動ノード側にも置く

[DetailedDesign-safety.md](DetailedDesign-safety.md) §3.1 の二層構造の上側。

| 層 | 判定距離 | 追従対象の除外 |
| --- | --- | --- |
| **挙動ノード**（ここ） | `obstacle_stop_distance_m` | **する**（自分が追っている人を障害物と読まない） |
| `obstacle_limiter` | `obstacle_floor_distance_m`（より小さい） | しない |

流用元は `mapless_follow_core.py:119-157` の `is_path_blocked()`。
**人物除外（`person_exclude_*`）を含めてそのまま使う。**

### 0.3 「停止」「保存」「終了」

`Spec-transit.md` §0.1 の 3 語は、**トリガ 3 つに 1:1 で対応する**。

| 語 | トリガ | 挙動ノードから見ると |
| --- | --- | --- |
| 停止 | `ui.stop` | 状態が `PAUSE` になる → `ACTIVE_STATES` から外れる → publish を止める |
| 保存 | `ui.save` | `effect: finalize_route` などが `th_route` に届く |
| 終了 | `ui.finish` | モードが `IDLE` になる → 同上 |

**挙動ノードは 3 語を知らない。**状態だけを見る。

### 0.4 前提条件の確認（`CL-T-2` / `Spec-transit.md` §0.6）

メインメニュー（S-01）で走行方式ボタンを非活性にする判定。
**`th_state` が `mode_entry_allowed` ガードで行い、UI は結果を表示するだけ。**

| 前提 | 判定 | `reject_reason_key` |
| --- | --- | --- |
| 教示済み経路があるか | `th_route` の `/route/list` が空でない | `no_route_recorded` |
| 人物追跡が動いているか | `tracker_enabled == true` | `tracker_disabled` |
| リードデバイスが繋がっているか | `evt.leash_present` を受けている | `device_not_connected` |
| カメラが繋がっているか | `/camera/status` が生きている | `device_not_connected` |
| **いま保管場所にいるか**（教示 2 種） | **人の責任。**判定しない | — |
| 区画線があるか | **人の責任。**判定しない | — |

**教示 2 種は復路では選べない**（`F-25`）。
判定できないので、S-01 に「保管場所から開始してください」を常時出す。

---

## 1. 追従走行（`FOLLOW`）

### 1.1 流用元は `mapless_follow_core.py` である

> **注意**: 名前から `follow_planner_core.py` を使いたくなるが**違う**。
> `FollowPlannerCore.update()` は **Nav2 のゴール（`kind="nav_goal"`）を返す**実装で、
> 「専用の速度トピックへ出す」という本設計と噛み合わない。
> さらに `PREPARE` / `EVADING`（近接退避）は `Spec-transit.md` §1.2
> 「対象に近づいたとき: **停止する**」と食い違うので**捨てる**。

| 流用するもの | 出どころ |
| --- | --- |
| `to_absolute` / `to_relative` / `update_trail` / `get_trail_goal` / `pure_pursuit_control` | `follow_planner_core.py`（幾何のみ。`mapless_follow_core.py` が既に import している） |
| `next_mapless_state` / `should_stop_for_lost` / `is_path_blocked` / `mapless_target_speed` / `rate_limit` | `mapless_follow_core.py` |
| **捨てるもの** | `FollowPlannerCore` 全体（`PREPARE` / `EVADING` / `find_nearest_open_direction` / `compute_evade_goal`） |

**Nav2 を使わない理由**: `Spec-transit.md` §7 は追従走行の「事前に必要なもの: **なし**」と定めている。
場外 200 m でグローバルコストマップ（＝事前地図）を前提にはできない。

### 1.2 ノード

`th_transit/scripts/follow_runner.py`（コア: `th_transit/th_transit/follow_core.py`）

| 項目 | 内容 |
| --- | --- |
| sub | `/system/state` / `/person/targets` / `/scan_filtered` / TF `odom→base_link` |
| pub | `/cmd_vel_behavior` / `/system/event` |
| `ACTIVE_STATES` | §0.0 が正。`CONFIRM` でも動く（その場で対象の方を向く） |
| レート | `update_rate_hz`（10 Hz） |

### 1.3 状態別の出力

| 状態 | 出力 |
| --- | --- |
| `SELECT` | 何も出さない |
| **`CONFIRM`** | **角速度のみ。**対象方向へ超信地旋回して向き続ける。`linear.x = 0`（`Spec-transit.md` §1.1 手順 3〜4「移動はしない」） |
| `RUN` | 軌跡追従。`pure_pursuit_control` で (v, ω) |
| `PAUSE` | 何も出さない |

### 1.4 追従アルゴリズム

```
1. /person/targets の選択中の位置を、受信時刻の odom 姿勢で絶対座標へ凍結
   （DR-SPAAM は約 2 Hz。自機が回ると見かけ位置が流れる）
2. update_trail() で軌跡に追加（trail_sample_interval_m ごと・絶対座標系で保持）
3. get_trail_goal() で lookback_distance だけ遡った点を目標にする
   → 「直角に曲がってもショートカットしない」の実体
4. 対象との距離 d で停止判定:
     d < follow_stop_distance_m         → 停止（退避しない）
     d ≥ follow_stop_distance_m + 帯    → 再開
5. is_path_blocked() で進路上の障害物を確認 → 塞がれていれば停止
6. mapless_target_speed() で制動距離から速度を決め、rate_limit() で加減速を鈍らせる
7. pure_pursuit_control() で (v, ω)
```

**フェイルセーフ**: TF が未確立、または `/scan_filtered` を一度も受信していない場合は**停止する**
（`mapless_follow_core.py:128-133` と同じ。他に障害物安全層が無かった名残ではなく、二重化として維持）。

### 1.5 ロストの扱い

| 経過 | 挙動 |
| --- | --- |
| `< tracker_lost_grace_ms` | **停止しない。**最後の位置へ向かって軌跡追従を続ける |
| `≥ tracker_lost_grace_ms` | 停止し、`evt.target_lost` を出す → `SELECT` へ（選択解除・案内 W-3） |

**捜索旋回（`person_predictor` の `SEARCHING`）は作らない。**
`Spec-transit.md` §1.2 は「選択を解除して停止し、案内ウィンドウを出す」であり、
自分で探し回るとは書いていない。`/cmd_vel_retreat` を廃止したのと同じ理由。

### 1.6 対象選択

| 契機 | 実装 |
| --- | --- |
| 試験員がレーダーの候補をタップ | `ui.select_target` → `effect: set_target` → `person_tracker` の `~/select_target` サービスを呼ぶ |
| 候補が 1 人だけの状態が `auto_select_hold_s` 続く | `person_tracker_bridge` が計測し `evt.auto_selected` |

**`require_explicit_target_selection: true` は維持する**（`leg_tracker_param.yaml`）。
机・椅子の脚へ乗り移る誤追跡（2026-07-11 実機で確認）の再発防止。

### 1.7 KPI

ロスト回数/100 m ／ ロスト→復帰時間 ／ **誤追従（別人・別物）回数** ／ 追従距離のばらつき。
`params_digest` を必ず記録に埋める（[DetailedDesign-params.md](DetailedDesign-params.md) §5.1）。

---

## 2. 手動走行（`MANUAL`）

### 2.1 挙動ノードを作らない

**WebUI が `/cmd_vel_manual_raw` へ publish し、`jog_gate` が `/cmd_vel_manual` へ通す。**
`th_transit` に `MANUAL` 用のノードは無い。

| 責務 | 担当 |
| --- | --- |
| 速度指令 | WebUI（`/cmd_vel_manual_raw`、10 Hz）→ `jog_gate` → `/cmd_vel_manual` |
| モード・状態によるゲート | **`jog_gate`**（[safety](DetailedDesign-safety.md) §3.4.1）。通さないときは**沈黙する** |
| 速度上限 | **`obstacle_limiter`**（ブラウザ側のスケーリングは利便性であって権威ではない） |
| 障害物（警告のみ／自動ブレーキ） | `obstacle_limiter`（`source_class = MANUAL`） |
| 状態（`RUN` ⇄ `PAUSE`） | `th_state`（`T-MANUAL-01/02`） |

> **現行の危うい事実**: 速度プリセットのスケーリングは `App.jsx:136 stickToCmd` で行われており、
> **上限がブラウザにしか無い。**新設計では上限の権威をリミッタへ移す。

### 2.2 スティックは走行操作であってジョグ介入ではない

`F-31`。`MANUAL` / `TEACH_MANUAL` は `C-01`（ジョグで `PAUSE` に落ちる）の**除外**。
触れると `PAUSE → RUN` に入る。

`ui.jog.hold` は**リースなので送り続ける**（`F-29`）。
途絶で `sys.jog_lease_expired` → `PAUSE`。**手を離す操作が届かなくても止まる。**

### 2.3 復帰

異常解決後は**「確認」1 択 → `PAUSE`**（`attributes.yaml` の `resume: ack_only`）。
手動操作は人が改めて始めるものなので、自動再開の対象にしない。

---

## 3. 教示（`TEACH_FOLLOW` / `TEACH_MANUAL`）

### 3.1 走行部分は追従走行・手動走行と完全に同じ

`U-5` / `Spec-transit.md` §0.5。**教示は「走らせ方」を変えない。**

| モード | 走行を担当するもの | 記録を担当するもの |
| --- | --- | --- |
| `TEACH_FOLLOW` | `follow_runner`（`MY_MODE` に `TEACH_FOLLOW` も含める） | `route_recorder` |
| `TEACH_MANUAL` | WebUI の `/cmd_vel_manual` | `route_recorder` |

**担当範囲は §0.0 が唯一の定義。**ここで書き直さない。

### 3.2 `route_recorder`（`th_route`）

| 項目 | 内容 |
| --- | --- |
| sub | `/system/state` / `/odometry/filtered` / `/map` / `/esp32/imu_data` |
| pub | `/route/status` / `/system/event` |
| 記録する状態 | `REC` のみ（`PAUSE` 中は記録しない＝一時停止は経路に残らない） |
| 記録するもの | §3.3 |

### 3.3 記録される 3 つ

| 種類 | 情報源 | 保存形式 |
| --- | --- | --- |
| 地図 | LiDAR（slam_toolbox） | `map.pgm` / `map.yaml` / `map.posegraph` |
| 経路 | **EKF 出力 `/odometry/filtered`** | `path.csv`（`t, x, y, yaw`。`route_sample_interval_m` ごと） |
| 走行開始時の向き | IMU 融合後のヨー（`REC` に最初に入った瞬間） | `route.yaml` の `start_yaw` |

**生のエンコーダ odom ではなく EKF 出力を使う。**
`ekf_filter_node` がジャイロの `vyaw` を融合しており、クローラのスリップによるヨードリフトが抑えられている。

### 3.4 保存（`ui.save` → `finalize_route`）

```
routes/<route_id>/
├── current/     ← 今回の記録
└── previous/    ← 直前の版（1 世代）
```

| 規則 | 内容 |
| --- | --- |
| 既存経路の再設定 | **上書きしない。**`current` を `previous` へ移し、新しい記録を `current` にする（`F-04`） |
| 世代 | **新版＋旧版 1 世代のみ。**`previous` が既にあれば捨てる |
| 保存の瞬間 | **ここで初めて終点が確定する**（`F-03`） |
| メタ | `RouteInfo`（長さ・点数・`start_yaw`・記録日時・**`params_digest`**） |
| エクスポート／インポート | `/route/export` `/route/import`。ディレクトリを tar で固めるだけ |

### 3.5 保存後も記録を続けられる（`F-32`）

`T-TEACH-05`（`SAVED` → `REC`）。続けて「保存」すると**さらに新版**になる
（`current` → `previous`、新しいものが `current`）。**旧版は 1 世代しか残らない**ので、
2 回続けて保存すると最初の版は消える。**UI に「旧版は 1 世代のみ」と出す。**

`TEACH_FOLLOW` は「走行」ボタン、`TEACH_MANUAL` は**スティック操作**で `REC` に戻る（`F-38`）。
S-13 の操作カードに「走行」は無い。

### 3.6 対象の取り違え防止（`DF-D-6`）

`TEACH_FOLLOW` は「確認」を省くが、**記録開始前に対象を 1 度画面に提示する。**
機体は動かさない。実装は S-12 の教示タブに対象を出すだけで、状態遷移は伴わない。

### 3.7 記録の連続性が切れたら

`evt.record_broken` → `IDLE`（`T-TEACH-06`）。
**未保存の記録は破棄せず、保存可否を問う**（`Spec-modes.md` §5・§6.1 の例外）。

切れたと判定する条件:

| 条件 | 理由 |
| --- | --- |
| `/odometry/filtered` が `route_gap_timeout_ms` 以上途絶 | 経路に穴があく |
| slam_toolbox が死んだ（`LOCALIZATION_LOST`） | 地図と経路の対応が取れない |
| odom が不連続（`route_jump_m` を超える飛び） | 位置推定が破綻した |

---

## 4. 教示再生走行（`REPLAY`）

### 4.1 Nav2 を使わない

**明示する。**書かないと Nav2 に手が伸びる。

| 根拠 | |
| --- | --- |
| `Spec-transit.md` §4.2 | 「障害物は**停止する。自動回避はしない**」 |
| 同 §0.3 | 「走行開始時に決定した経路を逸脱しない」 |

Nav2 のコントローラは局所回避を行う設計なので、この 2 つと噛み合わない。
記録経路の追従は **`pure_pursuit_control()`**（`follow_planner_core.py:220-245`）で足りる。

### 4.2 ノード

`th_route/scripts/replay_runner.py`（コア: `th_route/th_route/replay_core.py`）

| 項目 | 内容 |
| --- | --- |
| sub | `/system/state` / `/scan_filtered` / TF `map→base_link` / `/map_session/status` |
| pub | `/cmd_vel_behavior` / `/system/event` / `/route/status` |
| `ACTIVE_STATES` | `{"RUN"}` |

### 4.3 状態ごとの中身

| 状態 | 中身 |
| --- | --- |
| `ROUTE_SEL` | 経路と順／逆を選ぶ。**角度の指定は不要** |
| `LOCALIZE` | `map_session` を `LOCALIZING` で開く → 初期姿勢推定。確度が低ければ**探索範囲を広げる** → `ui.localize_global` で完全グローバル |
| `READY` | 地図・経路・現在位置を表示。試験員が確かめる |
| `RUN` | ① `start_yaw` まで**超信地旋回** → ② 経路を Pure Pursuit で追従 |
| `PAUSE` | 停止。**経路上の進捗インデックスを保持**（続きから再開） |
| `SAVED` | 地図の書き足しを確定（`map_update` が ON のときだけ） |

### 4.4 逆再生

**独立した方式ではなくこの画面の選択肢**（`C-03`）。実装は `path.csv` を逆順に読み、
`start_yaw` を「終端点での進行方向 + π」に置き換えるだけ。
**KPI では順・逆を分けて計測する。**

### 4.5 終端に達したとき（`F-36`）

`evt.arrived` → **`PAUSE` ＋案内 W-3**。自動で「終了」はしない。
`Spec-transit.md` §0.2 の「自動停止はライン誘導走行のみ」とは矛盾しない
（**辿るべき経路データが尽きただけ**で、到着を判定したわけではない）。

### 4.6 地図の更新（`F-05`）

| 項目 | 内容 |
| --- | --- |
| 対象 | **自己位置推定に使う地図への書き足し**（経路そのものではない） |
| 既定 | OFF。ON の間だけ slam_toolbox をマッピングモードにする |
| 保存 | 既定では保存しない。`ui.save` か「終了」時に問う |
| 途中保存 | **できる。**保存後も「走行」で経路の続きから再開できる（`F-32` ／ `T-REPLAY-08`→`ui.run`） |

### 4.7 未解決（`CL-T-4` / `O-c4`）

**走行中のドリフト補正手段が無い。**`LOCALIZE` は開始時の位置合わせであって走行中の補正ではない。
ホイールオドメトリ＋ジャイロの EKF で 200 m を再生すると流れる。

| 対処 | 内容 |
| --- | --- |
| 実装 | **しない**（この段階では）。1 経路の長さの上限を実測で決める |
| 計測 | `replay_drift_m_per_100m`（`class: c`）。終点位置誤差を順・逆それぞれで測る |
| 表示 | `RUN` 中、経路からの横方向偏差を画面に出す。試験員が発散に気づけるように |

**地図があるなら scan matching で補正できるのでは、という案は保留する。**
`map_session` が `LOCALIZING` で `map→odom` を出し続けているので技術的には可能だが、
「走行開始時に決定した経路を逸脱しない」との関係を先に詰める必要がある
（[DetailedDesign-open.md](DetailedDesign-open.md) へ申し送り）。

---

## 5. 地図セッション（`REPLAY` / `PREP` 共通）

**`th_route/scripts/map_session.py`。**現行 `th_config_manager/scripts/slam_control.py`（489 行）を土台にする。

### 5.1 既存の流儀をそのまま使う

| 既存の判断 | 評価 |
| --- | --- |
| 「地図作成停止」＝ `set_localization_mode(true)`。**自己位置推定は続く** | **正しい。**`pause_new_measurements` は `map→odom` を凍結させる実害があり 2026-08-07 に廃止済み |
| respawn によるクラッシュ復帰＋モード再適用 | **正しい。**そのまま維持 |
| `deserialize_map` は localization モード中に黙って何もせず success を返す | **既知の落とし穴。**§5.3 で回避する |

`PREP` の `MAPPING` ⇄ `PAUSE` ⇄ `EDIT` は、この toggle をそのまま状態機械にぶら下げるだけで成立する
（手順 9 の `RETURN` が「作りかけの地図で自己位置推定して戻る」＝ localization モードそのもの）。

### 5.2 スロットを 2 つ持つ

**同時には 1 つしか開けない。**2 つの地図は同じ `map` フレームに同時に存在できない。

| スロット | 実体 | 使うモード |
| --- | --- | --- |
| `VENUE` | `/root/th_data/venue/` | `PREP` / `PANEL_NAV` / `AT_PANEL` / `SUMMON` / `HOME_NAV` |
| `ROUTE` | `/root/th_data/routes/<id>/current/` | `TEACH_*` / `REPLAY` |

**同時には要らない。**この 2 群は排他的モードであり、間に必ず `IDLE` が入る（`Spec-modes.md` §4.2）。

> **現行の制約**: `slam_control.py:358-360` の `_map_basename()` が `~/th_maps/th_map` に
> **ハードコードされている。**地図は 1 枚しか持てない。**引数化は必須。**

### 5.3 切替は `deserialize` ではなく再起動で行う

```
/map_session/open(slot, session_id, mode)
  → slam_toolbox に SIGTERM → respawn で起動 → 指定のスロットを読む
```

`_cb_discard_map`（`slam_control.py:397-433`）の仕組みがそのまま使える。

| 理由 | |
| --- | --- |
| `deserialize_map` は localization モード中に**黙って何もしない**（2026-08-07 に実害） |
| **空に近いグラフの読み込みで SIGSEGV**（`slam_control.py:46-50`） |
| 数秒かかるが、`REPLAY/LOCALIZE` も `PREP/MAPPING` も待ち状態を持つので運用上吸収できる |

### 5.4 自己位置喪失をフォルトにする（`DEBT-5`）

`map→odom` の更新途絶と slam_toolbox プロセスの死を検出し、
**重大フォルト `LOCALIZATION_LOST`** として `/safety/fault` に出す
（[DetailedDesign-safety.md](DetailedDesign-safety.md) §5.3）。
現行はログと String を出すだけで、**凍結しても機体は走り続ける。**

---

## 6. ライン誘導走行（`LINE`）— **内部設計は後回し**

### 6.1 いま確定させること

| 項目 | 確定 |
| --- | --- |
| ノード | `th_transit/scripts/line_runner.py`（コア `line_core.py`） |
| sub | `/system/state` / `/camera/line` / `/scan_filtered` / `/linemap` |
| pub | `/cmd_vel_behavior` / `/system/event` / `/line/status` |
| `ACTIVE_STATES` | `{"RUN"}` |
| 出す `evt` | `evt.setup_done` / `evt.arrived` / `evt.line_lost` |
| 遷移 | `T-LINE-01`〜`06`（[DetailedDesign-state.md](DetailedDesign-state.md) §4.2）。**確定済み** |
| 画面 | S-15。**確定済み**（[DetailedDesign-webui.md](DetailedDesign-webui.md)） |
| データ | `/root/th_data/linemap/`（ノードとエッジ） |
| 初期姿勢の指定 | **盤登録の 2 点指示と同じ幾何。共通部品を使う**（`T-r18`） |
| 終点判定 | **7 方式で唯一の自動停止**（`F-02`） |
| 障害物 | 停止（自律系） |

### 6.2 後回しにするもの

| 項目 | 状態 |
| --- | --- |
| カメラの選定（前方 1 台） | **未確定**（`Spec.md` §6.1 #8「要検討（詳細設計）」）。§6.3 |
| 区画線・分岐の認識アルゴリズム | 未着手 |
| ラインマップの編集画面 | **設計しない**（`O-d1`。原典が明示的に後回しを宣言） |
| ノード／エッジ探索 | 未着手 |

### 6.3 カメラの選定だけは先に要る

`LINE` の実装は後回しだが、**機材の手配はリードタイムがある。**

| 決めること | 制約 |
| --- | --- |
| 接続先 | **RaspberryPi4 は LiDAR と USB 帯域を共有する**（`Spec.md` §6.1 の注記）。カメラを USB で足すと `/scan` の帯域を食う |
| 候補 | ① Pi の CSI ポート（USB 帯域を食わない）／ ② PC 側に USB 接続（機体に PC を載せないので不可）／ ③ ESP32-CAM（独立） |
| **推奨** | **① CSI。**USB 帯域の競合を避けられる唯一の選択肢 |
| 検証 | LiDAR を回しながら同時に取得して `/scan` の受信間隔が悪化しないことを実測する |

**これは `WP-LINE-00`（機材選定）として段階に載せる。**実装本体より前に置く。

### 6.4 受け入れ条件（内部設計が入ったとき）

1. `T-LINE-01`〜`06` の全遷移に pytest がある
2. 線をロストしたら**停止し、案内 W-3 を出す。自動で探し回らない**（`T-r13`）
3. 目的地ノード到達で自動停止し、**「終了」はしない**
4. 初期姿勢の指定が S-20 の 2 点指示と**同じ部品**で行われている
5. 故障注入: 走行中にカメラを切る → 停止する

---

## 7. 電子リード走行（`LEASH`）— **内部設計は後回し**

### 7.1 いま確定させること

| 項目 | 確定 |
| --- | --- |
| ノード | `th_transit/scripts/leash_runner.py`（コア `leash_core.py`） |
| デバイス | **RaspberryPi4 に USB 接続する Arduino。**LiDAR と**ポートと帯域を共有する** |
| sub | `/system/state` / `/leash/raw`（張力の有無と向き） / `/scan_filtered` |
| pub | `/cmd_vel_behavior` / `/system/event` / `/leash/status` |
| `ACTIVE_STATES` | `{"RUN"}` — **`HOLD` では出さない**（張力なし＝停止） |
| 出す `evt` | `evt.leash_present` / `evt.leash_absent` / `evt.leash_taut` / `evt.leash_slack` |
| 遷移 | `T-LEASH-01`〜`07`。**確定済み** |
| 速度 | **一定（`v_leash`）。**張力の大きさでは変えない。加減速はゆっくり |
| 向き | リードの向きに追従 |
| 軌跡 | **なぞらない**（追従走行の「ショートカットしない」規定は電子リードには無い） |

### 7.2 `HOLD` と `PAUSE` を絶対に混ぜない（`F-27`）

| 状態 | 張力がかかったら |
| --- | --- |
| `HOLD` | **自動で `RUN` に戻る**（引かれている＝進みたい、の表明） |
| `PAUSE` | **発進しない。**再開は「走行開始」の押下による |

**混ぜると「停止」を押してリードを置いた瞬間に発進する。**目標 G1 の「意図しない挙動」そのもの。
画面には別々の文言を出す（`HOLD`＝「リードが張られていません」／ `PAUSE`＝「一時停止中（走行開始で再開）」）。

**意図して離したか落としたかは区別しない。**どちらも止まる。

### 7.3 後回しにするもの

リードデバイスの回路・ファームウェア・プロトコル・張力と向きの検出方式。

### 7.4 受け入れ条件（内部設計が入ったとき）

1. `T-LEASH-01`〜`07` の全遷移に pytest がある
2. **`PAUSE` 中に張力をかけても発進しない**（故障注入で確認）
3. デバイス未接続なら S-16 の「走行開始」が非活性になる
4. リードを離してから停止までの時間が `leash_stop_latency_ms` 以内
5. **LiDAR の受信間隔がリードデバイス接続前後で悪化しない**（USB 帯域の競合）

---

## 8. 方式の比較と KPI

`Spec-transit.md` §7 のとおり。**率ではなく回数で書く**（`C-08`）。

| 段階 | 場所 | n | 目的 |
| --- | --- | --- | --- |
| 社内試験（PoC 期） | 社内 | 1 方式あたり **10〜30** | **方式の比較と取捨選択はここでしかできない** |
| 実証実験（MVP） | 先方の教育用施設 | 1 方式あたり **1〜2** | 選び抜いた方式のデータ取得 |

**KPI 採番は UI 採番とずれる**（KPI では逆再生を分けて数えるので 8 になる。`C-03`）。
記録には**方式名を書き、番号を使わない。**

### 8.1 記録に必ず埋めるもの

`params_digest` ／ 方式名 ／ 順/逆 ／ 距離 ／ 介入回数 ／ 日時 ／ 実施者。
**`params_digest` が無いと、暫定値で取ったデータかどうかが後から分からない。**

---

## 9. 逆引き

| 完全設計書 | ここでの反映 |
| --- | --- |
| `Spec-transit.md` §0.1（3 語） / `F-01` / `U-6` | §0.3 |
| §0.2（到着判定） / `F-02` / `F-36` | §4.5 |
| §0.3（障害物） / `C-11` / `U-2` | §0.2 |
| §0.4（手動 UI 常設） / `U-4` / `F-29` / `F-31` | §2.2 |
| §0.5（教示は＋タブ） / `U-5` | §3.1 |
| §0.6（前提条件） / `CL-T-2` / `F-25` | §0.4 |
| §1（追従走行） / `F-07` | §1 |
| §2（手動走行） | §2 |
| §3（教示） / `F-03` / `F-04` / `F-32` / `F-38` / `DF-D-6` | §3 |
| §4（教示再生） / `F-05` / `C-03` / `C-10` / `CL-T-4` | §4 |
| §5（ライン誘導） / `O-d1` / `T-r13` / `T-r18` | §6 |
| §6（電子リード） / `C-06` / `F-27` | §7 |
| §7（比較と KPI） / `C-08` | §8 |
| `Spec.md` §6.1 #8（カメラ要検討） | §6.3 |
