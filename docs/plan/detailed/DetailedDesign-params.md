# パラメータの実体化

[DetailedDesign.md](DetailedDesign.md) の詳細。**`Spec-params.md` §0 の 3 分類を、実装の性質として守る仕掛け。**

> **`DD-6`**: 未確定値 (c) に事前の目標値を置いてはいけない。
> 放置すると実装者が「それらしい数字」を埋め、**測定が目標の確認になる。**

---

## 1. 数値は 1 か所にしか書かない

```
th_ws/src/th_params/
├── config/registry.yaml        ← 全パラメータ。数値が存在する唯一の場所
├── th_params/schema.py         ← registry の型と検証（純粋）
├── th_params/derive.py         ← (b) の導出式（純粋関数）
├── th_params/assertions.py     ← 危険な組合せの検査（純粋）
├── th_params/export.py         ← ノード別の ROS2 パラメータ YAML を生成
└── scripts/params_audit.py     ← 起動時監査ノード
```

**ノードは `registry.yaml` を直接読まない。**ビルド時／起動時に `export.py` が
ノードごとの ROS2 パラメータ YAML を生成し、ノードは従来どおりそれを読む。
これにより「ノード側の実装は普通の ROS2 のまま」で、数値の出どころだけが 1 本化される。

### 1.1 `registry.yaml` のスキーマ

```yaml
- name: obstacle_stop_distance_m
  unit: m
  class: b                      # a | b | c
  status: derived               # measured | derived | given | placeholder
  value: null                   # derived のときは null（起動時に計算）
  value_by: [v_max, v_slow, v_reverse, v_jog_panel]   # 速度軸ごとに値を持つ（§1.4）
  derived_from: [brake_accel_mps2, brake_delay_s, safety_margin_m]
  formula: braking_distance_plus_margin              # derive.py の関数名
  consumers: [follow_runner, replay_runner, venue_navigator]
  spec_ref: "Spec-safety.md §2.2 / Spec-params.md §3【B】"
  note: "挙動ノード側の判定距離（d_behavior）。obstacle_floor_distance_m より必ず大きい（A2a）"
```

| 列 | 必須 | 意味 |
| --- | --- | --- |
| `name` / `unit` | ○ | [DetailedDesign-names.md](DetailedDesign-names.md) §7 と 1:1 |
| `class` | ○ | `a`＝外部から与えられる／`b`＝逆算できる／`c`＝測らないと分からない |
| `status` | ○ | §2 |
| `value` | `derived` 以外は ○ | `derived` は起動時に計算 |
| `derived_from` / `formula` | `class: b` なら ○ | `derive.py` の関数名 |
| `measured_at` / `source` | `status: measured` なら ○ | いつ・何で測ったか |
| `consumers` | ○ | このパラメータを読むノード。**空なら CI で落とす**（死んだパラメータの検出） |
| `spec_ref` | ○ | 完全設計書の該当節 |
| `blocking` | — | `true` なら placeholder のまま**起動を拒否する**。§1.3 の規則により自動で決まる |
| `value_by` | — | 派生値がベクトルになる場合の軸（§1.4） |

### 1.3 `class` と `status` の組合せ規則（**(c) の抜け道を塞ぐ**）

初版は `blocking` を任意項目にしていたため、
**(c) を `status: placeholder`（`blocking` なし）で書けば、それらしい数字のまま起動できた。**
`Spec-params.md` §0 が塞ぎたかった穴がそのまま残っていた。

| 規則 | 内容 |
| --- | --- |
| **S1** | `class: c` かつ `status != measured` ⇒ **`blocking: true` が必須**、`value` は **`TBD_MEASURE` のみ許可** |
| **S2** | `class: a` かつ `status != given` ⇒ `status` は `placeholder` のみ。`value` は `TBD_MEASURE` |
| **S3** | `class: b` は `derived` か `given` のみ。`derived` なら `formula` と `derived_from` が必須 |
| **S4** | `status: measured` は `measured_at` と `source` が必須 |
| **S5** | `derived_from` のいずれかが `placeholder` なら、出力も `placeholder` になる（伝播） |

**S1〜S5 は `schema.py` が検証し、CI（§8）でも回す。**

**`blocking: true` だけでは段階 1・2 が実機で起動できなくなる。**
`class: c` は 10 件以上あり、`replay_drift_m_per_100m`（段階 5）・`calib_*_tolerance`（段階 7）・
`battery_endurance_min`・`leash_stop_latency_ms`（段階 8）は**段階 2 までに測りようがない**。

**`blocking_from_stage` を持たせる。**

```yaml
- name: replay_drift_m_per_100m
  class: c
  status: placeholder
  value: TBD_MEASURE
  blocking: true
  blocking_from_stage: 5        # 段階 5 以降でだけ起動を止める
  consumers: [replay_runner]
```

| 判定 | 内容 |
| --- | --- |
| 起動を止める条件 | `blocking: true` **かつ** `blocking_from_stage ≤ 現在の段階` **かつ** そのパラメータの `consumers` が**今回の launch で起動するノードに含まれる** |
| 現在の段階 | launch 引数 `stage`（既定は最大値＝全部止める） |
| Gazebo・単体テスト | `sim:=true` で `allow_placeholder: true`（測定そのものができないため） |

**「使わないノードのパラメータで起動が止まる」ことを防ぐ**のが要点。
`consumers` を見る設計にしてあるので追加の仕掛けは要らない。

### 1.4 ゾーン・モードで変わる派生値

`obstacle_stop_distance_m` / `obstacle_floor_distance_m` などは `v_limit` の関数なので
**スカラーではなく表**になる。

```yaml
- name: obstacle_stop_distance_m
  class: b
  status: derived
  value_by: [v_max, v_slow, v_reverse, v_jog_panel, v_check, v_calib, v_leash]
  formula: braking_distance_plus_margin
  derived_from: [brake_accel_mps2, brake_delay_s, safety_margin_m]
```

`export.py` は `obstacle_stop_distance_m` を**辞書**として出す（キーは速度パラメータ名）。
アサーション A2 は**すべての軸について**検査する。

**`obstacle_floor_distance_m` だけは `value_by` を持たない**（速度に依存しない固定クリアランス。
[DetailedDesign-safety.md](DetailedDesign-safety.md) §3.1）。

### 1.2 `status` の意味

| `status` | 意味 | ヘッダ表示 |
| --- | --- | --- |
| `measured` | 実測して入れた。`measured_at` と `source` が必須 | なし |
| `derived` | 他の値から計算した | なし |
| `given` | 外部から与えられた（(a)）。仕様書・先方回答が根拠 | なし |
| **`placeholder`** | **まだ根拠が無い暫定値** | **バッジを出す** |

---

## 2. sentinel を使わない

**`NaN` / `-1` を「未設定」の印にしてはいけない。**
チェック漏れが 1 箇所でもあると、そのまま `v_max` の既定に落ちる。

代わりに 3 つの原則を置く。

| # | 原則 | 内容 |
| --- | --- | --- |
| **P1 fail-slow** | 暫定値は**安全側の極値**にする | `v_slow` の暫定は「動く最低速度」、`obstacle_*_distance_m` の暫定は「もっともらしい**最悪の** `a`」から計算した値。**現在たまたま入っている数字（0.45 m など）を暫定値として引き継がない** |
| **P2 出どころをデータで持つ** | 数値の隣に必ず `class` / `status` / `source` / `spec_ref` を置く | 「(c) に目標値を置かない」の実態は「値の隣に出どころが無いこと」なので、出どころを構造的に強制すれば禁止が守れる |
| **P3 (b) は書かずに計算する** | `Spec-params.md` §3【B】は式である。`derive.py` に純粋関数として置き、**ロード時に導出する** | 現状の `obstacle_check_distance_m = 0.45` と制動距離 0.49 の不整合は、**手で打っているから**起きている |

### 2.1 grep 可能なマーカーは 1 種類だけ

```
TBD_MEASURE
```

**`registry.yaml` にしか現れない。**コードにも他の YAML にも書かない。

```bash
grep -n TBD_MEASURE th_ws/src/th_params/config/registry.yaml   # 未測定の全件
```

---

## 3. 導出式（`derive.py`）

`Spec-params.md` §3 の 3 本の連鎖を関数にする。**すべて純粋関数・pytest 対象。**

### 3.1 【B】制動の連鎖（最も広く効く）

```python
def braking_distance(v: float, a: float, t_delay: float) -> float:
    """Spec-safety.md §2.2 / Spec-params.md §3【B】 / F-06（停止距離の定義）"""
    return v * v / (2.0 * a) + v * t_delay
```

| 導出されるもの | 式 |
| --- | --- |
| `obstacle_stop_distance_m`（`d_behavior`） | `braking_distance(v_limit, a, t_delay) + safety_margin_m` |
| `obstacle_floor_distance_m`（`d_floor`） | **この式では出さない。**`floor_distance(body_half_length_m, floor_margin_m)`（速度非依存。§3.1.1 の下） |
| `follow_stop_distance_m` | `braking_distance(v_max, a, t_delay) + person_margin_m` |
| `lidar_timeout_ms` / `esp32_timeout_ms` | §3.3 |
| 停止・再開のヒステリシス帯 | **`hysteresis_ratio × obstacle_floor_distance_m`**（§4 の A11。`obstacle_stop_distance_m` から導くと帯が `d_behavior` を超えうる） |

`v_limit` は**ゾーンとモードで変わる**ので、`obstacle_limiter` は
`v_max` / `v_slow` / `v_reverse` / `v_jog_panel` それぞれに対する距離を起動時に表として持つ。

### 3.1.1 速度は「止まれる距離」から逆算する

**`v_slow` などを「なんとなく遅い値」で置かない。**制動距離の式を速度について解く。

```python
def speed_from_braking_distance(d_allow: float, a: float, t_delay: float) -> float:
    """braking_distance(v, a, t_delay) == d_allow を満たす v。
    v²/(2a) + v·t = D を解く（正の根）。"""
    return a * (-t_delay + math.sqrt(t_delay ** 2 + 2.0 * d_allow / a))
```

| 導出されるもの | `d_allow` に入れるもの | 意味 |
| --- | --- | --- |
| `v_slow` | `venue_clearance_m` | 試験場内。**納品する製品が置いてある**ので、それに触れずに止まれる距離 |
| `v_reverse` | `blind_clearance_m` | 後退。**LiDAR 死角**なので、見えていない範囲でも止まれる距離 |
| `v_jog_panel` | `panel_clearance_m` | 盤前。盤に触れずに止まれる距離 |

`venue_clearance_m` / `blind_clearance_m` / `panel_clearance_m` は **(a)／方針値**
（`O-a2` / `O-a4` が決まるまで `placeholder`）。

**`v_max` だけは別の出どころ**（駆動系の天井から）:

```python
def v_max_from_ceiling(ceiling_mps: float, headroom_ratio: float) -> float:
    """出力上限を使い切らず、補正の余地を残す。Spec-params.md §1"""
    return ceiling_mps * headroom_ratio
```

**`v_check` / `v_calib` / `v_leash` は `derived` ではなく `given`（方針値）にする。**
逆算する対象が無い（`v_check`＝「押している間だけ動く極低速」、
`v_calib`＝「校正の再現性が出る速度」、`v_leash`＝「人の歩行速度」）。
**式が無いのに `derived` と名乗ると、起動時に `None` のままになる。**

### 3.1.2 その他の導出

```python
def person_backstop_ms(grace_ms: float, link_p99_ms: float, factor: float) -> float:
    """safety_monitor の遅いバックストップ。挙動側の grace より必ず長い。"""
    return max(grace_ms * factor, link_p99_ms * factor)

def clear_distance(body_half_length_m: float, clear_margin_m: float) -> float:
    """退避待ちゲート：対象がゴールから離れるべき距離。"""
    return body_half_length_m + clear_margin_m

def hysteresis_band(floor_distance_m: float, ratio: float) -> float:
    """★ d_floor スケール。d_behavior から導くと帯が d_behavior を超えうる（A11）。"""
    return floor_distance_m * ratio
```

**`status: derived` の全パラメータに `formula` が存在すること**を CI で検査する（§8-5）。

### 3.2 【A】停止精度の連鎖

```python
def combined_heading_error(two_point_deg: float, nav_tolerance_deg: float) -> tuple[float, float]:
    """(二乗和平方根, 最悪の重なり) を返す。Spec-onsite.md §3.3 / F-22"""
    rss = math.hypot(two_point_deg, nav_tolerance_deg)
    worst = two_point_deg + nav_tolerance_deg
    return rss, worst

def two_point_angle_error_deg(spacing_m: float, sigma_m: float) -> float:
    """2 点指示が与える向きの誤差。L と σ から。Spec-params.md §1"""
    return math.degrees(math.atan2(sigma_m * math.sqrt(2.0), spacing_m))
```

**`nav_tolerance_deg` だけを敷居にしてはいけない**（`F-22`）。
「足りる」と誤判定する帯域ができる。

### 3.3 タイムアウトの 2 本制約

```python
def timeout_lower_bound_ms(p99_gap_ms: float, margin_ratio: float) -> float:
    """誤フォルトを出さない下限"""
    return p99_gap_ms * (1.0 + margin_ratio)

def timeout_upper_bound_ms(v_max: float, intrusion_budget_m: float) -> float:
    """タイムアウトまでに走ってよい距離からの上限"""
    return intrusion_budget_m / v_max * 1000.0
```

**採る値の決め方（反復しない）**:

```
1. timeout = timeout_lower_bound_ms(p99, margin_ratio)      ← 下限を採る
2. if timeout > timeout_upper_bound_ms(v_max, budget):
       v_max = budget / (timeout / 1000)                    ← v_max を下げて確定
3. 以上。ここで打ち切り、v_max に依存する派生値を 1 度だけ再計算する（反復しない）
```

**タイムアウトを縮めて誤フォルトを増やすのではない**
（[DetailedDesign-safety.md](DetailedDesign-safety.md) §7）。
**不動点反復にしない**のは、収束を保証できず起動時間が読めなくなるため。
2 で `v_max` を下げたことは警告として `/system/params_status` に載せる。

### 3.4 【C】経路長の連鎖

```python
def deviation_budget_m(corridor_width_m: float, body_width_m: float, margin_m: float) -> float:
    return (corridor_width_m - body_width_m) / 2.0 - margin_m
```

`corridor_width_m` は (a) 未取得（`O-a4`）なので、これに依存する値はすべて `placeholder` になる。

---

## 4. 起動時アサーション（危険な組合せを構成不能にする）

**文書の注意書きにせず、機械で不可能にする。**`assertions.py` に純粋関数として置く。

| # | 検査 | 違反時 |
| --- | --- | --- |
| **A1** | `v_max × (lidar_timeout_ms / 1000) ≤ intrusion_budget_m`（`esp32_timeout_ms` も同様） | **`v_max` をクランプし、警告を出す。**`blocking` 指定なら起動を拒否 |
| **A2a** | `obstacle_floor_distance_m < obstacle_stop_distance_m`（**自律走行に使う軸だけ**: `v_max` / `v_slow` / `v_reverse`） | **起動を拒否**（リミッタが先に発火して自律走行が成立しない）。**全軸に課してはいけない** → §4.1 |
| **A2b** | `obstacle_floor_distance_m < follow_stop_distance_m` | **起動を拒否**（リミッタが追従対象の脚で発火する） |
| **A3** | `0 < tracker_lost_grace_ms < person_timeout_ms` | 起動を拒否 |
| **A4** | `jog_lease_ms ≥ twist_mux の manual_joy timeout` | 起動を拒否 |
| **A5** | `v_reverse ≤ v_slow ≤ v_max` かつ `v_jog_panel ≤ v_reverse` | 起動を拒否 |
| **A6** | `esp32_timeout_ms > WATCHDOG_MS`（ESP32 側 600 ms） | 警告 |
| **A7** | `cmd_vel_stale_ms < WATCHDOG_MS` | 起動を拒否（ESP32 より先に PC 側で止まること） |
| **A8** | `status: placeholder` かつ `blocking: true` かつ **`blocking_from_stage ≤ 現在の段階`** かつ **その `consumers` が今回の launch に含まれる**行が 0 | 起動を拒否 |
| **A9** | 全パラメータの `consumers` が空でない | **CI で落とす**（起動時ではない） |
| **A10** | `\|wheel_radius_scale − 1\| ≤ wheel_radius_scale_max_dev` | **起動を拒否。**10% を超えるずれは校正ではなく機械的な異常（[maintenance](DetailedDesign-maintenance.md) §4.3） |
| **A11** | `obstacle_floor_distance_m + hysteresis_band_m < obstacle_stop_distance_m` | 起動を拒否（ヒステリシス帯が `d_behavior` を越えない） |

| **A12** | `muxed_stale_ms < manual_joy_timeout` | 起動を拒否（[safety](DetailedDesign-safety.md) §1.3。逆にすると `jog_gate` を閉じてから駆動が止まるまでが `manual_joy_timeout` に伸びる） |

**`hysteresis_band_m` は `obstacle_floor_distance_m × hysteresis_ratio` から導く**（`value_by` なし）。
`obstacle_stop_distance_m` から導くと**帯が `d_behavior` を超えうる**。

### 4.1 A2a を全軸に課してはいけない

`d_behavior` は軸ごとに `braking_distance(v_limit) + safety_margin_m` で変わるが、
**`d_floor` は速度非依存の機体クリアランス**（`body_half_length_m + floor_margin_m`）である。

| 軸 | `d_behavior` のおおよその大きさ | A2a |
| --- | --- | --- |
| `v_max` / `v_slow` / `v_reverse` | 制動距離ぶん大きい | **課す** |
| **`v_jog_panel` / `v_check` / `v_calib`** | **`panel_clearance_m` 程度（数 cm〜）。`d_floor` より小さくなる** | **課さない** |

**極低速の軸まで A2a を課すと、設計どおりの値を入れた時点で起動しなくなる。**
逃げようとすると `floor_margin_m` を負にする（＝`d_floor` を機体内部に置く）ことになり、
§3.1「リミッタは上位が壊れたときだけ発火する」という後段配置の根拠自体が崩れる。

**極低速の軸では、リミッタは距離ではなく政策表（[safety](DetailedDesign-safety.md) §3.3）で制御する。**
`AT_PANEL` のジョグは `MANUAL` × `IN` なので `auto_brake` の既定 ON で止まり、
試験員が OFF にすれば盤の手前まで詰められる。

**`WATCHDOG_MS` はファームのコンパイル時定数なので、`registry.yaml` には
`esp32_watchdog_ms`（`status: given`、`source: esp32/src/config.h`）として写しを持つ。**
値がずれたら A6/A7 が拾う。ファーム側が `FW_INFO` で実値を通知するようになれば実測に置き換える
（[DetailedDesign-safety.md](DetailedDesign-safety.md) §11.1）。

---

## 5. 監査ノードと画面表示

**生成と監査を分ける。**同じノードにすると鶏卵になる
（launch はノード起動前に `parameters=[...]` を評価するので、初回起動時にファイルが無い）。

| 段 | 実行 | 責務 |
| --- | --- | --- |
| **① 生成** | **launch の `OpaqueFunction`**（ノード起動より前に同期実行） | `registry.yaml` を読み、`derive.py` で (b) を計算し、`assertions.py` を回し、**ノード別の ROS2 パラメータ YAML と `twist_mux.yaml` を `/root/th_data/generated/` へ書く。**アサーション違反ならここで launch を止める |
| **② 監査** | ROS2 ノード `params_audit` | 生成結果を読んで `/system/params_status` を publish。`/params/get` `/params/set` `/params/save` を提供 |

`export.py` は**純粋関数＋薄い CLI** にする（`python3 -m th_params.export --out <dir>`）。
launch からも CI からも同じものを呼ぶ。

| `params_audit` の責務 | 内容 |
| --- | --- |
| publish | `/system/params_status`（`ParamsStatus`。transient_local） |
| サービス | `/params/get` / `/params/set` / `/params/save`。**`IDLE` / `MANUAL` 以外は拒否**（現行 `config_manager` の流儀を維持） |
| 校正値の取り込み | `/root/th_data/calib/current.yaml` を読み、該当行の `status` を `measured` に上書き |

**`twist_mux.yaml` も生成対象に含める。**`manual_joy.timeout` を registry と YAML の 2 か所に
持つと `DD-6` が最初のパケットで破れる（[DetailedDesign-safety.md](DetailedDesign-safety.md) §3.2）。

### 5.1 `params_digest`

```
digest = sha1( sorted( f"{name}={status}:{value}" for all params ) )[:12]
```

**KPI 記録・rosbag・校正履歴・教示経路のメタに必ず埋める。**
暫定値で取ったデータが後から「暫定だった」と分かるようにする。
これが `Spec-params.md` §0「測定が目標の確認になる」への実質的な対策である。

### 5.3 現場での調整（`Spec-params.md` §6.1 の実装。**2026-08-18 追加**）

**`registry.yaml` は設計値の正であり続ける。**現場の調整は**重ね書き**で持つ
（校正値を `/root/th_data/calib/current.yaml` から重ねるのと同じ流儀）。

| 段 | 実体 | 内容 |
| --- | --- | --- |
| 1 | `registry.yaml` | 設計値。**リポジトリの中。**現場からは書き換えない |
| 2 | `/root/th_data/calib/current.yaml` | 校正の出力（`status: measured` へ） |
| **3** | **`/root/th_data/params/overrides.yaml`** | **現場調整の出力。**`{name: {value, set_at, set_by, reason}}` だけを持つ |

**読み込み順は 1 → 2 → 3。**後のものが勝つ。`params_digest` は**重ね書き後の値**で計算する
（暫定値・調整値で取ったデータが後から識別できる）。

#### 5.3.1 `/params/set` の手続き

```
1. 対象が status: given の行か検査する（derived / measured は拒否 → reject_reason_key）
2. モードが IDLE / MANUAL か検査する（それ以外は拒否。現行 config_manager の流儀）
3. 重ね書きを当てた registry のコピーを作り、derive.py を回し、assertions.py を全部通す
4. 1 つでも違反したらコピーを捨てて拒否する（★部分適用しない）
5. 通ったら overrides.yaml へ書き、生成物を作り直し、走っているノードへ反映する
6. 反映できないノードがあれば「要再起動」を /system/params_status に載せる
```

| 不変条件 | 内容 |
| --- | --- |
| **PT-1** | **アサーションを通らない値は 1 つも当たらない。**検査は当てる前に、コピーの上で行う |
| **PT-2** | `derived` の行は `/params/set` の対象にならない。**元の値を変える**（例: `obstacle_stop_distance_m` ではなく `brake_accel_mps2` や `safety_margin_m`） |
| **PT-3** | 走行中（`IDLE` / `MANUAL` 以外）は**受理そのものを拒否**する。`accepted=false` を返し、値を保持しない |
| **PT-4** | `overrides.yaml` の各行に `set_at` / `set_by` / `reason` が必須。**出どころの無い数値を作らない**（P2） |

#### 5.3.2 反映の手段

**停止中に限られるので、その場での反映を許す。**`params_audit` が対象ノードの
ROS2 パラメータを `set_parameters` で更新する。**宣言していないノードは「要再起動」**とし、
UI が再起動を促す（黙って古い値で走らせない）。

#### 5.3.3 画面

**段階 1 に最小版**（`WP-UI-08a`。開発モードタブ内。値の一覧と編集と保存だけ）、
**正式版は段階 7**（`WP-UI-08`。S-50 の 3 タブ）。
機体が動き出すのは段階 3 だが、**寸法と速度上限は段階 1・2 の実機起動でも要る**ので前倒しする。

### 5.2 ヘッダのバッジ

`placeholder_count > 0` の間、ヘッダに「暫定値 n 件」を出す。
タップすると S-50 の該当タブへ飛び、`placeholder_names` を一覧する。
**開発モードでのみ非表示にできる**（`Spec-webui.md` §5）。

---

## 6. パラメータ一覧（数値は書かない）

**`registry.yaml` の内容の索引である。値そのものは registry を見る。**

### 6.1 いま置ける値（`class: b` / `given`）

| 名前 | 分類 | 根拠 |
| --- | --- | --- |
| `drivetrain_ceiling_mps` | b | 出力上限 ÷ フィードフォワード係数 |
| `v_max` | b | 計画値。**A1 でクランプされうる** |
| `nav_tolerance_m` / `nav_tolerance_deg` | given | Nav2 の現行設定値 |
| `two_point_spacing_m` | b | 推奨 1.0 m |
| `esp32_watchdog_ms` | given | `esp32/src/config.h` の写し |
| `fault_to_stop_ms` | given | 安全チェーンの要求（100 ms） |
| `body_width_m` | given | 機体幅 |
| `wheel_base_m` / `wheel_radius_m` | measured | 校正の出力（§7） |

### 6.2 測らないと分からない（`class: c`・**目標値を置かない**）

| 名前 | これが決まると決まるもの | 元 ID |
| --- | --- | --- |
| **`brake_accel_mps2`** | `v_max` / 全障害物距離 / タイムアウト。**最も多くを従属させる 1 つ** | `O-c1` |
| `person_position_sigma_m` | 2 点指示の角度精度（【A】） | `O-c3` |
| `link_gap_p99_ms`（ESP32 / LiDAR / UI 別） | タイムアウトの下限（§3.3） | `O-c6` |
| `tracker_lost_grace_ms` | 追従・呼び寄せの停止判断 | `C-05` |
| `replay_drift_m_per_100m` | 1 経路の長さの上限 | `O-c4` |
| `calib_linear_tolerance_ratio` / `calib_rotation_tolerance_deg` / `calib_blind_tolerance_deg` | 校正の合否 | `O-c5` / `F-13` |
| `calib_interval_days` | 定期校正の催促 | `O-c5` |
| `battery_endurance_min` | 目標 G5 | `O-c7` |

### 6.3 外部から与えられる（`class: a`・**未取得の間は `placeholder`**）

| 名前 | 元 ID | これが決まると決まるもの |
| --- | --- | --- |
| `camera_lift_tolerance_m` / `camera_lift_tolerance_deg` | `O-a1` | 到着後の盤面正対を作るか（G2 の合否基準） |
| `panel_count` / `panel_spacing_m` / `panel_front_space_m` | `O-a2` | ピン方式の価値・平面認識の成立性 |
| `prep_baseline_min` | `O-a3` | 目標 G3 の検証 |
| `corridor_width_m` / `floor_type` | `O-a4` | 許容逸脱（【C】）・ライン誘導の成立性 |
| `unmanned_permission` | `O-a5` | 先導付きで測るか無人で測るか |
| `carry_baseline_min` | `O-a7` | 目標 G4 の検証 |

**これらが `placeholder` の間、依存する (b) も `placeholder` に伝播する。**
`derive.py` は入力のいずれかが `placeholder` なら出力も `placeholder` にする。

### 6.4 導出値（`class: b` / `status: derived`）

| 名前 | `formula` | 主な `derived_from` |
| --- | --- | --- |
| `v_max` | `v_max_from_ceiling` | `drivetrain_ceiling_mps`, `v_max_headroom_ratio` |
| `v_slow` | `speed_from_braking_distance` | `venue_clearance_m`, `brake_accel_mps2`, `brake_delay_s` |
| `v_reverse` | 同上 | `blind_clearance_m`, … |
| `v_jog_panel` | 同上 | `panel_clearance_m`, … |
| `obstacle_stop_distance_m` | `braking_distance_plus_margin` | `brake_accel_mps2`, `safety_margin_m`（`value_by` あり） |
| `obstacle_floor_distance_m` | `floor_distance` | `body_half_length_m`, `floor_margin_m`（`value_by` **なし**） |
| `follow_stop_distance_m` | `braking_distance_plus_margin` | `person_margin_m`, … |
| `lidar_timeout_ms` ／ `esp32_timeout_ms` | `timeout_from_bounds` | `link_gap_p99_ms`, `intrusion_budget_m`, `v_max` |
| `person_timeout_ms` | `person_backstop_ms` | `tracker_lost_grace_ms`, `link_gap_p99_ms` |
| `clear_distance_m` | `clear_distance` | `body_half_length_m`, `clear_margin_m` |
| `hysteresis_band_m` | `hysteresis_band` | **`obstacle_floor_distance_m`**, `hysteresis_ratio`（A11。§4） |
| `deviation_budget_m` | `deviation_budget_m` | `corridor_width_m`, `body_width_m` |

**`v_check` / `v_calib` / `v_leash` は `given`（方針値）。**逆算する対象が無い（§3.1.1）。

**これらをコードにハードコードしたら CI で落とす**（§8）。

---

## 7. 校正値との関係

校正（`CALIB`）が書き換える値は `registry.yaml` に置かず、
**`/root/th_data/calib/current.yaml` を正とする**（3 世代の履歴つき。`F-14`）。

| 値 | 適用先 | 適用の重さ |
| --- | --- | --- |
| `wheel_radius_m` | **ESP32 のコンパイル時定数** | 書き換え＋再書き込み（`O-d10` → [DetailedDesign-maintenance.md](DetailedDesign-maintenance.md)） |
| `wheel_base_m` | `esp32_bridge` のランタイムパラメータ | 即時反映・永続化 |
| `imu_offsets` | BNO055 の内部オフセット | 保存し次回起動時に自動復元 |
| `blind_angle_ranges` | `lidar_filter` ＋ **`obstacle_limiter`** | 即時反映・永続化 |

`params_audit` は起動時に `calib/current.yaml` を読み、
`registry.yaml` の該当行の `status` を `measured` に上書きする（`measured_at` は校正日時）。

---

## 8. CI / pytest

| # | 検査 | ファイル |
| --- | --- | --- |
| 1 | コードが使う全パラメータが `registry.yaml` に存在する | `test_registry_completeness.py` |
| 2 | `registry.yaml` の全行に `spec_ref` がある | 同上 |
| 3 | `consumers` が空の行が無い | 同上 |
| 4 | **registry にある (b) パラメータの現在値と同じ数値が、コード／`generated/` 以外の YAML に現れない** | `test_no_hardcoded_numbers.py`。**除外リストは `th_params/config/literal_allowlist.yaml`**（QoS 深さ・配列長・単位変換など） |
| 5 | `derive.py` の全関数に単体テストがある。**`status: derived` の全パラメータに `formula` が存在する** | `test_derive.py` |
| 6 | `assertions.py` の **A1〜A12**（A9 は CI 専用。A11・A12 を含む）が違反入力で正しく落ちる | `test_assertions.py` |
| 9 | **`class`／`status` の組合せ規則 S1〜S5 に違反する行が無い** | `test_registry_schema.py` |
| 7 | 入力が `placeholder` の (b) は出力も `placeholder` になる（伝播） | `test_placeholder_propagation.py` |
| 8 | `TBD_MEASURE` が `registry.yaml` 以外に現れない | `test_marker_isolation.py` |

```bash
python3 -m pytest src/th_testing/test/test_registry_completeness.py \
                  src/th_testing/test/test_derive.py \
                  src/th_testing/test/test_assertions.py -v
```

---

## 9. 詳細設計書での書き方（全ファイル共通の規約）

各作業パケットの「パラメータ」節に、**裸の数値を絶対に書かない。**書くのは次の形。

> `obstacle_floor_distance_m` — class (b) / status `derived` /
> `derived_from: [body_half_length_m, floor_margin_m]`。速度に依存しない固定クリアランス。
> **`obstacle_stop_distance_m` と `follow_stop_distance_m` の両方より小さいこと（A2a / A2b）。**
>
> `brake_accel_mps2` — class (c) / status `placeholder` / **value `TBD_MEASURE` / `blocking: true`**。
> これに依存する値はすべて `placeholder` として伝播し、**実機では起動を止める**。
> 測定手順は `Spec-params.md` §8 の①。

**暫定の数値を書かない。**「現行設定値の 6 割」のような書き方も禁止する
（`Spec-params.md` §1 は減速度 0.5 を「現行設定値・**未実測**」＝(c) と分類しており、
そこから暫定値を作ると (c) に事前の目標値を置いたことになる）。

---

## 10. 保存先（`C-04` / `F-14`）

実体は [DetailedDesign-names.md](DetailedDesign-names.md) §9.2。要点だけ再掲する。

| データ | 場所 | 世代 |
| --- | --- | --- |
| 試験場内地図＋ピン | `/root/th_data/venue/` | **1 枚のみ** |
| 教示の経路と地図 | `/root/th_data/routes/<id>/{current,previous}/` | **新版＋旧版 1 世代**（`F-04`） |
| 校正の補正値と履歴 | `/root/th_data/calib/` | **3 世代**（`F-14`） |
| ラインマップ | `/root/th_data/linemap/` | — |
| ログ | `/root/th_data/logs/` | — |
| 生成パラメータ | `/root/th_data/generated/` | 起動のたび再生成 |

**Docker は継続する**（`C-04`）。変わったのはホスト OS だけ。
`docker-compose.yml` に `./data:/root/th_data` の bind mount を足す
（`dr_spaam_weights` と同じ流儀で、`--rm` の使い捨てコンテナでも消えない）。

**試験場内地図と経路地図は UI 上でも別の場所に置く**（`CL-M-9`）。取り違え防止。

---

## 11. 逆引き

| 完全設計書 | ここでの反映 |
| --- | --- |
| `Spec-params.md` §0（3 分類） / `SD-7` | §1.1・§2 |
| §1（いま置ける値） | §6.1 |
| §2（速度の階層） | §6.4 |
| §3【A】【B】【C】（逆算の連鎖） | §3 |
| §4・§5（時間・距離） | §6 |
| §6（保存先） / `C-04` / `F-14` | §10 |
| §7（未取得の外部値） | §6.3 |
| §8（先に潰す 4 つ） | §6.2 |
| `Spec-safety.md` §2.2（停止距離） | §3.1 |
| `Spec-checks.md` §3.7（許容範囲） / `F-13` | §6.2・§7 |
