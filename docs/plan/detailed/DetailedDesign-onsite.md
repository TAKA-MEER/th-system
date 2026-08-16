# 試験場内の実装

[DetailedDesign.md](DetailedDesign.md) の詳細。**`PREP` / `PANEL_NAV` / `AT_PANEL` / `SUMMON` / `HOME_NAV`。**

先方の要望「立ち会い試験で配電盤上部を安全に確認したい」「前日準備が大変」に
**直接あたるのがこの区間**である。ここでは**速度は評価軸にならない**（1 回の移動は数 m）。
効くのは**盤前に何 cm・何度で着けるか**と、**1 盤あたり何操作かかるか**（盤の数だけ掛かる）。

---

## 0. パッケージ構成

```
th_onsite/
├── th_onsite/two_point_core.py      2 点指示の幾何（純粋）
├── th_onsite/wait_clear_core.py     退避待ちゲート（純粋）
├── th_onsite/align_core.py          超信地旋回（純粋）
├── th_onsite/pin_store.py           ピンの永続化（純粋）
├── th_onsite/plane_core.py          LiDAR 平面認識（純粋・§3.6／§8.3。O-a1 が決まるまで保留）
├── scripts/pin_registrar.py         PREP の登録
├── scripts/venue_navigator.py       PANEL_NAV / SUMMON / HOME_NAV を 1 ノードで
└── scripts/wait_clear_gate.py       退避待ち
```

**`venue_navigator` を 3 モードで 1 ノードにする理由**: 3 つは行き先の決め方が違うだけで、
`NAV` → `BLOCKED` → `ALIGN` → `ARRIVED` の走らせ方が完全に同じ。
別ノードにすると `FollowPath` の扱い（§3）が 3 か所に散る。

---

## 1. Nav2 の使い方（**最重要**）

### 1.1 `NavigateToPose` ではなく `FollowPath` を使う

**これを書かないと `NavigateToPose` が使われ、「停止」のたびに経路が変わる。**

| 要求 | 出典 |
| --- | --- |
| `PAUSE` → `NAV` は「**同じ経路の続きから。再検索はしない**」 | `Spec-modes.md` §3.1.2（`T-PNAV-03`） |
| 「**最初に決めたルートが通れるようになるのを待つ。自動でルートを変えない**」 | `Spec-onsite.md` §6 |
| 「走行開始時に決定した経路を逸脱しない」 | `Spec-transit.md` §0.3 |

| 案 | 評価 |
| --- | --- |
| `NavigateToPose` のゴールを cancel して再送 | **× 再プランが走る。**上の 3 つすべてに違反 |
| `controller_server` を lifecycle で deactivate / activate | △ 復帰が重く、costmap も落ちる |
| **走行開始時に `/plan` を保存し、`FollowPath` アクションで走らせる** | **◎** |

### 1.2 手順

```
1. ui.goto / evt.clear_ok で行き先が確定
2. planner_server の ComputePathToPose を 1 回だけ呼ぶ → nav_msgs/Path を得る
3. Path を保持し、controller_server の FollowPath アクションへ送る  → NAV
4. ui.stop → FollowPath を cancel。Path と進捗インデックスは保持    → PAUSE
5. ui.run  → 保持した Path の残りを FollowPath へ再送               → NAV
6. evt.blocked → cancel。Path は保持したまま待つ                    → BLOCKED
7. evt.unblocked → 残りを再送                                        → NAV
8. ui.reroute → ここで初めて ComputePathToPose をやり直す            → NAV
```

**`replan` という effect が呼ばれたときだけ再計算する。**それ以外では Path を触らない。

### 1.3 塞がれた／通れるようになったの判定

| 事象 | 判定 |
| --- | --- |
| `evt.blocked` | 保持した Path 上の前方 `blocked_lookahead_m` 以内に、グローバルコストマップで通行不能なセルがある状態が `blocked_hold_ms` 続いた |
| `evt.unblocked` | 同じ条件が解消した状態が `unblocked_hold_ms` 続いた |

**タイムアウトしない**（`F-11`）。`BLOCKED` は待ち続ける。
`BLOCKED` からも `ui.stop` で `PAUSE` へ抜けられる（待たされている間に手で動かしたい場合）。

### 1.4 速度と障害物

| 項目 | 担当 |
| --- | --- |
| 速度上限 `v_slow` | **`obstacle_limiter`**（ゾーン `IN`）。Nav2 の `max_vel_x` は上限として置くが権威ではない |
| 障害物で停止 | `obstacle_limiter`（`source_class = AUTO`）＋ Nav2 のローカルコストマップ |
| **自動回避しない** | Nav2 の局所回避を切る。`FollowPath` のコントローラを純粋な経路追従に寄せる |

---

## 2. 2 点指示（`Spec-onsite.md` §3.1 ／ `U-3`）

### 2.1 幾何（`two_point_core.py`）

```python
def two_point_pose(p1: tuple[float, float], p2: tuple[float, float]) -> Pose2D:
    """① が立ってほしい位置（＝ゴール）、② は向きを与えるためだけの点。
    ①→② のベクトルが盤の法線方向になる。Spec-onsite.md §3.1 / U-3"""
    yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    return Pose2D(x=p1[0], y=p1[1], theta=yaw)   # ← ① がゴール
```

> **原典との差異に注意**: `NewDesign-onsite.md` は「②が立ってほしい位置」と書いているが、
> **`U-3` で①がゴールに上書きされている。**間違えると盤の 1 m 手前で止まる。

| 項目 | 仕様 |
| --- | --- |
| 押すもの | **同じボタンを、異なる 2 か所で押す**（`DF-D-2` の解決） |
| 座標の出どころ | **押した瞬間の追従対象の位置**（`/person/targets` の選択中） |
| 点間距離 | `two_point_spacing_m`（推奨 1 m）。**実測して `RegisterPin` の応答に返す** |
| 精度 | `two_point_angle_error_deg(spacing, sigma)`（[DetailedDesign-params.md](DetailedDesign-params.md) §3.2） |
| 用途 | **待機場所登録（`F-08`）・盤登録・呼び寄せ（`F-09`）・ライン誘導の初期姿勢**。4 つとも同じ部品 |
| 保存の有無 | 登録は保存する。**呼び寄せは保存しない。**それだけが違う |

### 2.2 受付条件（`CL-M-10` / `C-05`）

| 条件 | 挙動 |
| --- | --- |
| 追従対象を確実に認識している | 登録できる |
| **ロスト中、またはロスト直後** | **拒否。猶予は無い**（走行時の `tracker_lost_grace_ms` は効かない） |
| 信頼度が `register_min_confidence` 未満 | 拒否 |
| 候補が 2 人以上で対象未選択 | **登録ボタンを非活性**にする |

**誤った座標が地図に永続化されると当日まで気づけない**ので、走行より厳しくする。
拒否は `evt.register_rejected` ＋ `reject_reason_key`。**対象選択の操作に被らない位置**に W-3 を出す。

### 2.3 点間距離が短すぎるとき

`two_point_spacing_m` を大きく下回ると角度誤差が発散する。
**実測距離が `two_point_min_spacing_m` 未満なら②を拒否し、「もう少し離れて押してください」を出す。**

---

## 3. `PREP`（前日の地図作成と登録）

### 3.1 手順と状態の対応

| `Spec-onsite.md` §2 の手順 | 状態 | 実装 |
| --- | --- | --- |
| 1 試験準備画面を開く | `IDLE` → `PREP` / `MAPPING` | `map_session.open(VENUE, MAPPING)` |
| 2 地図作成を開始 | `MAPPING` | slam_toolbox マッピングモード |
| 3 待機場所に立って登録 | `REGISTER` | `RegisterPin(kind=HOME)` |
| 4 待機場所ピンが立つ | `MAPPING` | **この姿勢を地図の初期位置とする** |
| 5 追従で連れ回しながら地図を生成 | `MAPPING` | **`follow_runner` が `PREP` でも動く**（§3.2） |
| 6〜7 各盤の前で登録 → ピンが立つ | `REGISTER` → `MAPPING` | `RegisterPin(kind=PANEL)` |
| 8 ピンをタップして名前を設定 | `MAPPING` | `EditPin` |
| 9 1 ボタンで待機場所に戻す | `RETURN` | §3.4 |
| 10 地図を修正（人の映り込みを消す） | `EDIT` | §3.5 |
| 11 「保存」で地図を確定 | `SAVED` | `map_session.save()` |
| 12 「終了」 | → `IDLE` | |

**手順どおりの順番でなくてもよい**（`Spec-onsite.md` §2 の追記）。
`EDIT` には手順 9 を経由せず入れるし、`SAVED` のあとも「走行」で `MAPPING` に戻れる（`F-32`）。

### 3.2 連れ回しは追従走行そのもの

**`follow_runner` が `PREP/MAPPING` でも動く。**担当範囲は
[transit](DetailedDesign-transit.md) §0.0 が唯一の定義。専用の追従ロジックを作らない。

**対象確定のゲートは同 §0.0 のノード側条件で行う**（`PREP` に `CONFIRM` 状態が無いため）。

| 違い | |
| --- | --- |
| 速度上限 | `v_slow`（ゾーン `IN`。`obstacle_limiter` が適用） |
| 自動ブレーキ | 既定 ON |
| 「確認」 | 無い（`PREP` に `CONFIRM` 状態は無い） |

### 3.3 ピンのランタイム登録（**現行に無い機能**）

> 現行は `th_bringup/config/panels.yaml` を `panel_navigator` が**起動時に 1 度読むだけ**で、
> 追加・編集・削除・再登録の手段が無い。`panels.yaml` は廃止する。

| サービス | 動作 |
| --- | --- |
| `/onsite/two_point` | ①②の押下を受ける。②で `evt.two_point_done` |
| `/onsite/register_pin` | 2 点指示の結果からピンを作り、`pins.yaml` へ書いて `/onsite/pins` を publish |
| `/onsite/edit_pin` | 名前変更・削除・再登録（2 点指示をやり直す） |

`pin_store.py` は `PinList` ⇄ `pins.yaml` の相互変換だけを持つ純粋モジュール。
書き込みは `yaml_writer.py`（コメント保持。既存流用）を使う。

| 制約 | |
| --- | --- |
| `kind=HOME` | **1 つだけ。**2 つ目は既存を置き換える（確認ウィンドウ W-4 で二段階） |
| `kind=PANEL` | 何個でも。`id` は `panel_01` から自動採番 |
| 座標系 | **`map` フレーム。**登録時に `map→base_link` ではなく、**追従対象の位置を `map` へ変換して使う** |

### 3.4 手順 9「1 ボタンで待機場所へ」

**保存前の、作りかけの地図で自己位置推定して戻る**（`Spec-onsite.md` §2.1）。想定どおり。

| 項目 | 実装 |
| --- | --- |
| 行き先 | 待機場所ピン（手順 3 で既に地図上にある） |
| 走らせ方 | **`venue_navigator` と同じ `FollowPath`**（§1） |
| 地図 | マッピングモードのまま。`map→odom` は生きている |
| 到着 | `evt.arrived` → `EDIT` |

### 3.5 地図の修正（`CL-M-3`）

| 項目 | 仕様 |
| --- | --- |
| 誰が | **試験員**（WebUI の操作者）。2 人目の補助を要しない（目標 G3） |
| 何を | 地図上で範囲を選び、**人の映り込みを消す** |
| 元に戻す | できる（`EraseMapRegion(undo=true)`） |
| 実装 | 占有格子の該当セルを `-1`（未知）にする。**`0`（空き）にしない** — 通れると誤認させないため |
| いつ | 保存前 |

**人の映り込みが実際に問題になるかは未検証**（`O-c2`）。
「作業者が写り込んで特殊なツールで整形が必要」は伝聞であり、この構成で再現するか未確認。
**人物マスク実装の要否はこのテストで決まる**（最も安いテスト。`Spec-params.md` §8 の③）。

→ **`WP-ONSITE-00` として「人が歩いている部屋で地図を作ってみる」を段階の先頭に置く。**
実装ではなく測定の作業パケットである。

### 3.6 平面認識による 1 ボタン登録（方式 B）

| 順 | 動作 |
| --- | --- |
| 1 | 「登録」を押す。**押した時点の立ち位置を暫定ゴールとして記録**（`F-12`） |
| 2 | 試験員がその場を退く → **退避待ちゲート**（§5）を使う |
| 3 | 暫定ゴールまで移動する |
| 4 | 盤面の平面を認識して向きを決める → `evt.plane_done` |
| 5 | ピンが立つ |

**手順 1 が要る理由**（`O-r6`）: その時点で盤の位置はまだ登録されていないので、
機体は「配電盤前」がどこか知らない。

**遮蔽の問題**（`CL-M-2`）: 登録の瞬間、試験員はロボットと盤の間に立っており盤面が隠れる。
手順 2 の退避で解消するが、**退避完了の判定は §5 と同じ仕組みを使う**。

平面検出は LiDAR の 2D スキャンから直線を当てる（RANSAC）。
`th_onsite/plane_core.py` に純粋関数として置く。**方式 A（2 点指示）が本命で、これは付随機能。**

---

## 4. 当日（`PANEL_NAV` / `AT_PANEL` / `SUMMON` / `HOME_NAV`）

### 4.1 待機場所にいることの宣言（`CL-D-2`）

**試験画面を開いた直後は `IDLE` のまま**（`Spec-modes.md` §4.2）。この間に宣言と照合を行う。

| 順 | 実装 |
| --- | --- |
| 1 | `map_session.open(VENUE, LOCALIZING)` |
| 2 | 試験員が「いま待機場所にいる」を押す → `/onsite/declare_home` |
| 3 | 待機場所ピンの姿勢を初期姿勢として与える |
| 4 | LiDAR で照合し、**推定結果とピンのずれ**を返す |
| 5 | `home_declare_tolerance_m` / `_deg` を超えていれば警告ウィンドウ → 「置き直す」か「このまま宣言する」 |

`force=true` で 5 を押し切れる。**押し切ったことを記録に残す**（後で誤差の原因を追える）。

### 4.2 (a) ピンを選ぶ

`ui.goto{kind: PANEL, pin_id}` → `PANEL_NAV` / `NAV`。
行き先は**登録済みピンの姿勢**（位置＋向き）。

### 4.3 (b) その場で呼ぶ

`ui.goto{kind: SUMMON}` → `SUMMON` / `POINT` → 2 点指示 → `WAIT_CLEAR` → `NAV`。
**(a) と同じ幾何で、保存しないだけ**（`F-09`）。

### 4.4 次の行き先

(a) のあとも (b) のあとも**同じ 3 つ**を選べる（待機場所／次の配電盤／その場で呼ぶ）。
原典は書き分けが違っていたが**統一した**（`O-r7`）。

`working == true` の間はすべて拒否（`T-ATP-05` のガード `goto_allowed` が偽になる。`reject_reason_key = "working_in_progress"`）。

---

## 5. 退避待ちゲート（`F-10` / `CL-D-6`）

**試験員が退く前に機体が発進してはいけない。**

`th_onsite/wait_clear_core.py`（純粋）:

```python
def evaluate(target_xy, goal_xy, satisfied_since_ms, now_ms, p) -> ClearVerdict:
    d = dist(target_xy, goal_xy)
    if d >= p.clear_distance_m:
        if satisfied_since_ms and now_ms - satisfied_since_ms >= p.clear_hold_ms:
            return ClearVerdict.OK
        return ClearVerdict.WAITING
    return ClearVerdict.NOT_CLEAR      # 保持時刻はリセットされる
```

| 項目 | 仕様 |
| --- | --- |
| 条件 | 対象がゴールから `clear_distance_m` 以上離れ、`clear_hold_ms` 継続 |
| 満たされない間 | **発進しない。**画面に「退いてください」＋現在の距離を出す（`/onsite/wait_clear`） |
| 追跡が切れたら | 発進しない → `evt.target_lost` → `POINT` へ |
| **タイムアウト** | `clear_timeout_ms` 超過で `evt.clear_timeout` → `POINT` へ（**中止してやり直す**） |
| 適用先 | 呼び寄せ (b) ／ 平面認識 1 ボタン登録（§3.6 手順 2〜3） |
| **ジョグ介入** | **不可**（`F-28` の除外）。手動 UI を非活性にし「退避待ちの中止」を代わりに出す |

**タイムアウトする理由**（`F-11`）: 人は戻ってこられる。永久に発進しない状態を作らない。

### 5.1 退避方向のサニティチェック（任意・`C-02` の①の残し方）

退避待ちで退避方向は自動的に観測できる。
**観測した退避がほぼ放射方向だった場合に限り**、2 点指示で得た向きと突き合わせる。
食い違ったら試験員に再指示を促す。

**単独の根拠にはしない。**試験員は盤とロボットの間から**横に退く**のが自然で、
退避ベクトルは法線に対しておよそ 90° ずれるため。**実装は任意。**

---

## 6. `ALIGN`（到着後の向き合わせ）

`th_onsite/align_core.py`（純粋）:

```python
def align_command(current_yaw, target_yaw, p) -> tuple[float, float]:
    err = wrap_to_pi(target_yaw - current_yaw)
    if abs(err) <= p.align_tolerance_rad:
        return (0.0, 0.0)                      # 完了 → evt.align_done
    w = clamp(p.align_kp * err, -p.align_w_max, p.align_w_max)
    return (0.0, w)                            # 超信地旋回。並進しない
```

| 項目 | 仕様 |
| --- | --- |
| 動き | **超信地旋回のみ。**`linear.x = 0` |
| 目標 | (a) 登録済みピンの向き ／ (b) 2 点指示で与えた向き |
| 完了 | `evt.align_done` → `AT_PANEL` / `IDLE_P` |
| 速度 | `v_slow` に対応する角速度上限。`obstacle_limiter` の **L3** により角速度は殺されない |

---

## 7. `AT_PANEL`（配電盤前）

### 7.1 範囲外

| 項目 | 扱い |
| --- | --- |
| ピッチ・ヨー・高さ（カメラ昇降） | **別担当。**扱わない（`O-d2`） |
| 盤前での低速並進 | **含めない**（`O-d3`） |

### 7.2 作業中ボタン（暫定インターフェース）

| `working` | 挙動 |
| --- | --- |
| ON | 待機場所・別の配電盤・呼び寄せの**いずれを選んでも拒否**。画面に理由を出す |
| OFF | 通常どおり次の行き先を選べる |

**未確定**（`O-a6`）: 誰がいつ押すか、OFF ＝作業完了とみなしてよいか、
正式なインターフェース（到着通知／昇降完了／撮影完了／中断通知）。

**将来の置き換えに備えて、`working` を立てる経路を 2 本用意しておく。**

| 経路 | 実装 |
| --- | --- |
| いま | `ui.working{on}` |
| 将来 | `/lift/status` を購読して自動で立てる。**トピック名だけ予約しておく** |

### 7.3 到着ずれのリカバリ（`O-d3`）

低速並進を含めないので、外れたときに詰め直す手段が無い。当面は 3 つ。

| 案 | 実装 |
| --- | --- |
| **1. ジョグ介入で手で詰める** | `T-ATP-03`。速度は **`v_jog_panel`**（`F-24`）。`v_reverse` 相当まで落とす |
| 2. 呼び寄せ (b) でやり直す | `ui.goto{kind: SUMMON}` |
| 3. 到着後の LiDAR 盤面正対 | §8。**作るかどうかは `O-a1` 次第** |

`v_jog_panel` は `AT_PANEL` のジョグ中**だけ**適用する。
`obstacle_limiter` が `/system/state` の `mode == AT_PANEL && jog_active` で選ぶ
（[DetailedDesign-safety.md](DetailedDesign-safety.md) §3.2 — 上限の選択にだけ状態を使う、の唯一の例）。

---

## 8. 到着精度と盤面正対（`F-22` / `O-a1`）

### 8.1 誤差は 2 つの合成である

```
① 2 点指示が与える「目標の向き」自体の誤差 = two_point_angle_error_deg(L, σ)
② Nav2 がその目標にどれだけ着けるか        = nav_tolerance_deg

盤前で出る向き誤差 ≒ √(①² + ②²)      … 独立なら二乗和平方根
                     最悪の重なりなら ① + ②
```

**②だけを敷居にすると「足りる」と誤判定する帯域ができる**（`F-22`）。
`derive.py` の `combined_heading_error()` が両方を返す。

### 8.2 正対を作るかは要求許容差で決まる

```
camera_lift_tolerance_deg  >  合成誤差  →  2 点指示だけで足りる。正対は作らない
camera_lift_tolerance_deg  <  合成誤差  →  到着後に別ループで詰める正対が必要
```

`camera_lift_tolerance_deg` は **(a) 未取得**（`O-a1`）なので、現時点では決まらない。

**先に「どこまで緩められるか」を交渉する**（`Spec-params.md` §8 の②）。
聞くより交渉が先、という順序をここでも守る。

### 8.3 作る場合の設計（保留）

到着後、LiDAR で正面の直線を当てて法線を求め、`ALIGN` と同じ超信地旋回で詰める。
`th_onsite/plane_core.py` を §3.6 と共用できる。
**`O-a1` が決まるまで着手しない。**

---

## 9. 受け入れ条件

| # | 検証 | 手段 |
| --- | --- | --- |
| 1 | `two_point_pose()` で**①がゴール**になっている（`U-3`） | `test_two_point_core.py`。**②をゴールにしたら失敗するテストを書く** |
| 2 | ロスト中・低信頼度で登録が拒否される（猶予なし） | `test_pin_registrar.py` |
| 3 | `PAUSE` → `ui.run` で**再プランが走らない** | `FollowPath` の goal id が同一であることを確認する統合テスト |
| 4 | `BLOCKED` がタイムアウトしない | 10 分待っても `BLOCKED` のままであること |
| 5 | **退避待ちで発進しない → タイムアウトで中止** | Gazebo 故障注入 §10-7 |
| 6 | `WAIT_CLEAR` 中は手動 UI が非活性 | UI テスト＋`th_state` が `ui.jog.hold` を拒否 |
| 7 | `working == true` で行き先が拒否される | `test_state_core.py`（`T-ATP-05` のガード `goto_allowed`） |
| 8 | ピンの追加・改名・削除・再登録が再起動後も残る | `test_pin_store.py` ＋ 実機 |
| 9 | 地図修正で消したセルが**未知（-1）**になる | `test_map_erase.py` |
| 10 | 待機場所の宣言でずれが検出される | Gazebo（ピンから 1 m ずらして置く） |

```bash
python3 -m pytest src/th_testing/test/test_two_point_core.py \
                  src/th_testing/test/test_wait_clear_core.py \
                  src/th_testing/test/test_align_core.py -v
ros2 launch th_bringup gazebo.launch.py scenario:=panel_shuttle
```

---

## 10. KPI（`Spec-onsite.md` §8）

| 指標 | 見方 |
| --- | --- |
| 平面検出の成功率 | 方式 B について |
| 法線誤差 [deg] | **2 点指示との差分**を取れば絶対基準がなくても相対評価できる |
| **押下回数** | 1 盤あたり試験員が画面を何回触るか。**盤の数だけ掛かるので効く** |
| 盤が壁と面一のときの挙動 | 段差がない場合に破綻しないか |
| 地図作成＋盤登録の所要時間と手修正回数 | 目標 G3。**ベースラインが未取得**（`O-a3`） |

---

## 11. 逆引き

| 完全設計書 | ここでの反映 |
| --- | --- |
| `Spec-onsite.md` §0（骨子） | §4 |
| §1（扱うデータ） / `CL-M-9` | §3.3・[DetailedDesign-params.md](DetailedDesign-params.md) §10 |
| §2（前日の手順） / `U-7` / `F-32` | §3 |
| §2.1（作りかけの地図で戻る） / `O-r8` | §3.4 |
| §2.2（地図の修正） / `CL-M-3` / `O-c2` | §3.5 |
| §2.3（ピンの編集） / `CL-M-6` | §3.3 |
| §3.1（2 点指示） / `U-3` / `DF-D-2` / `F-08` / `F-09` | §2 |
| §3.2（平面認識） / `F-12` / `O-r6` / `CL-M-2` | §3.6 |
| §3.3（盤面正対） / `F-22` / `O-a1` / `O-c3` | §8 |
| §3.4（採らない方式） / `C-02` | §5.1 |
| §3.5（受付条件） / `CL-M-10` / `C-05` | §2.2 |
| §4.0（待機場所の宣言） / `CL-D-2` | §4.1 |
| §4.1〜4.3（当日の 2 つの行き方） / `C-12` / `O-r7` / `DF-D-4` | §4 |
| §5（退避待ちゲート） / `F-10` / `F-11` / `F-28` | §5 |
| §6（Nav2 方針） | §1 |
| §7（配電盤前） / `O-a6` / `O-d3` / `F-24` | §7 |
| §8（KPI） | §10 |
