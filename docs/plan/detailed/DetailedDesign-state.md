# 状態機械

[DetailedDesign.md](DetailedDesign.md) の詳細。**`Spec-modes.md` §3.1 を実装可能な形に正規化する。**

> **`DD-3`**: 遷移表は**データ**であり、実装はそれを読む。
> `Spec-modes.md` §3.1 の「表に無い遷移は起こらない／新しい遷移が必要なら先に表へ行を足す」を、
> コードの性質として担保する。

---

## 1. 構成

```
th_state/
├── config/transitions.yaml     ← §4 の表そのもの（唯一の正）
├── config/mode_entry.yaml      ← Spec-modes.md §4.2 のモード遷移許可表
├── config/attributes.yaml      ← Spec-modes.md §6 のモード別属性表
├── th_state/state_core.py      ← ROS2 非依存。YAML を解釈して遷移を計算する
├── th_state/guards.py          ← ガード述語（純粋関数）
└── scripts/state_manager.py    ← ROS2 ノード。コアを呼ぶだけ
```

**`state_core.py` は `rclpy` を import しない。**`th_testing` から直接 pytest できること。
既存の `follow_planner_core.py`（422 行・28 テスト）と同じ流儀。

---

## 2. 純粋コアの API

```python
@dataclass(frozen=True)
class Context:
    """遷移の判定に必要な、状態機械の外にある情報。ノードが毎回詰めて渡す。
    ★ 全ガードは Context のフィールドだけで判定できること（validate() の検査項目）。"""
    # --- 直前の状態（ESTOP / CARRY / PREP の復帰用。ラッチはノードが保持する） ---
    prev_mode: str                  # "" なら未ラッチ
    prev_state: str
    prev_sub: str                   # PREP が PAUSE に入る直前の内部状態
    # --- フラグ ---
    flags: dict[str, bool]          # jog_active, working, map_update, tracker_enabled, auto_brake
    zone: str                       # "IN" | "OUT" | "NA"
    # --- 人物追跡 ---
    candidate_count: int
    target_selected: bool
    target_confident: bool
    # --- 安全 ---
    fault_active: bool
    fault_severity: str             # "" | "RECOVERABLE" | "CRITICAL"
    fault_type: str
    hw_estop: bool
    ui_estop: bool
    # --- 各機能の可否 ---
    route_ids: tuple[str, ...]      # 教示済み経路
    pin_kinds: tuple[str, ...]      # 登録済みピンの種別
    leash_present: bool
    leash_taut: bool
    line_visible: bool
    camera_present: bool
    check_item: str                 # OPCHECK で実行中の項目（"" なら無し）
    check_result: str               # "" | "OK" | "WARN" | "NG"
    calib_item: str
    calib_preview_sane: bool
    map_update_available: bool
    # --- その他 ---
    now_ms: int
    arg: dict                       # トリガの arg_json をパースしたもの

@dataclass(frozen=True)
class Effect:
    name: str
    args: dict                      # 文字列パースをしない。構造化して渡す

@dataclass(frozen=True)
class Decision:
    accepted: bool
    to_mode: str                    # 解決済み（$トークンは残さない）
    to_state: str
    effects: tuple[Effect, ...]
    rule_id: str
    reject_reason_key: str          # accepted=False のときだけ

class StateCore:
    def __init__(self, transitions, mode_entry, attributes, guards): ...
    def validate(self) -> list[str]:
        """起動時に呼ぶ。空でなければ起動を止める。検査するもの:
        ① モード名・状態名が §2/§3 の集合に含まれる
        ② ガード名が guards.py に存在し、Context のフィールドだけを読む
        ③ effect 名が §3.3 の一覧に存在し、引数の型が合う
        ④ to_mode/to_state の $トークンが §3.2 の一覧に含まれる
        ⑤ 到達不能な状態が無い
        ⑥ PAUSE を持たないモードが無い（INIT/ESTOP/CARRY/IDLE を除く）
        ⑦ 全行に spec_ref がある"""
    def step(self, mode: str, state: str, event: str, ctx: Context) -> Decision: ...
    def initial_state(self, mode: str) -> str: ...
    def attributes(self, mode: str) -> dict: ...
```

**`step()` は副作用を持たない。**`effects` を返すだけで、実行はノードが行う。
**ラッチ（`prev_*`）もノードが持つ。**コアは `Context` で受け取り、`Decision` で解決済みの値を返す。
`latch_prev` / `clear_prev` は effect であり、ノードがそれを見て `prev_*` を更新する。

---

## 3. 遷移表のスキーマ

| 列 | 型 | 意味 |
| --- | --- | --- |
| `id` | str | `C-01` / `T-FOLLOW-03`。**詳細設計・pytest・UI がこの ID で参照する** |
| `layer` | int | `0`＝共通（`Spec-modes.md` §3.1.1）／`10`＝モード内（同 §3.1.2）。**小さい方が優先** |
| `mode` | str \| list \| `"*"` | 遷移元モード |
| `state` | str \| list \| `"*"` | 遷移元状態 |
| `event` | str \| list | `ui.*` / `evt.*` / `fault.*` / `hw.*` / `sys.*` |
| `guard` | str \| null | `guards.py` の述語名。`null` は常に真 |
| `to_mode` | str | `"="`＝不変。`$` トークンは §3.2 |
| `to_state` | str | 同上 |
| `effects` | list | `{name, args}` の配列。**文字列に引数を埋め込まない** |
| `reject_reason_key` | str | ガード不成立時、または `reject: true` のときに UI へ返す文言キー |
| **`reject`** | bool | `true` なら**マッチした時点で `accepted=false`** と `reject_reason_key` を返す。`to_mode` / `to_state` / `effects` は持たない |
| `override_common` | bool | §3.1 |
| `spec_ref` | str | **全行必須。**`SM-3.1.2-034` 形式（§3.4） |

### 3.1 マッチ規則（**手続きとして書く**）

```
1. layer 昇順・同 layer 内は表の記載順に走査する
2. 行がマッチする ⇔ mode/state/event がそれぞれ一致（"*" は全一致・list は要素のいずれか）
3. override_common: true の行が【マッチし、かつガードが真】になった時点で、その行を採る。
   同じ event を持つ layer:0 の行は以後いっさい評価しない
   （★ ガード評価を省くと、T-OPC-05 のようにガード付きの override 行が
      MOTOR 項目実行中でも C-07 を隠してしまい、物理 E-Stop が効かなくなる）
4. それ以外は、マッチしてガードが真の最初の行を採る
5. reject: true の行を採った場合は accepted=False とその行の reject_reason_key を返す
6. どの行にも当たらなければ accepted=False, reject_reason_key="not_allowed"
```

**「同じ `(mode, state, event)`」という包含関係で定義しない。**
`C-01` は `(*, *, ui.jog.hold)`、`T-MANUAL-01` は `(MANUAL, PAUSE, ui.jog.hold)` で
両者は「同じ」ではないため、旧表現では実装者が解釈できなかった。
**「override 行がマッチしたら共通行を見ない」という手続きに直した。**

### 3.2 実行時解決トークン

| トークン | 解決 | 使える列 |
| --- | --- | --- |
| `=` | 変更しない | `to_mode` / `to_state` |
| `$initial` | `attributes[to_mode].initial_state` | `to_state` |
| `$resume_run` | `attributes[mode].run_state` | `to_state` |
| `$resume_state` | `attributes[mode].resume_state` | `to_state` |
| `$prev_mode` / `$prev_state` / `$prev_sub` | `ctx.prev_*` | 両方 |
| `$arg.<key>` | `ctx.arg[key]`。**`ui.goto` だけ §3.5 の写像を通す** | 両方 |

**解決順序**: `$arg` → `attributes` → `prev`。解決できなければ `validate()` が起動時に落とす。

### 3.3 effect の一覧

| effect | args | 実行するノード | 方式 |
| --- | --- | --- | --- |
| `set_jog` | `{on: bool}` | `th_state` 自身 | フラグ更新 |
| `latch_prev` | — | `th_state` 自身 | **`mode ∉ {ESTOP, CARRY}` のときだけ記録**（§7） |
| `clear_prev` | — | `th_state` 自身 | |
| `set_screen` | `{screen_id}` | `th_state` 自身 | ゾーン再計算 |
| **`mark_arrived`** | `{kind}` | `th_state` 自身 | `/system/state.last_event` に載せる。滞在する状態を作らない（[names](DetailedDesign-names.md) §3） |
| **`abort_check`** | — | `opcheck_runner` | 実行中の点検項目を中止する（`T-OPC-08`） |
| `set_target` | `{index}` | `person_tracker` | srv `~/select_target` |
| `clear_selection` | — | `person_tracker` | srv `~/reset_tracking` |
| `face_target` | — | `follow_runner` | 状態を見て自動（サービス不要） |
| `open_window` / `close_window` | `{id: "W-1".."W-6"}` | WebUI | `/system/state` の表示で伝える |
| `guide` | `{key}` | WebUI | 同上（W-3） |
| `ask_save_if_unsaved` / `ask_save` | — | WebUI | W-4 |
| `enable_main_menu` | — | WebUI | |
| `show_resume` | — | WebUI | W-2 |
| `restart_control_stack` | — | `connectivity_checker` | プロセス再起動 |
| `start_record` / `resume_record` / `finalize_route` | `{route_id?}` | `route_recorder` | srv |
| `load_route` | `{route_id, reverse}` | `replay_runner` | srv |
| `rotate_to_start_yaw` / `resume_path` | — | `replay_runner` | 状態で自動 |
| `widen_search` / `global_localize` | — | `replay_runner` | srv |
| `commit_map_patch` / `commit_venue_map` | — | `map_session` | srv `/map_session/save` |
| `keep_all` | — | — | 何もしない（意図の明示） |
| `begin_two_point` | `{kind}` | `pin_registrar` | srv |
| `place_pin` | — | `pin_registrar` | srv |
| `reject_register` | — | `pin_registrar` | |
| `disable_jog_ui` / `enable_jog_ui` | — | WebUI ＋ **`jog_gate`** | §5.3 |
| `cancel_follow_path` / `resume_follow_path` / `replan` | — | `venue_navigator` | アクション操作 |
| `start_monitor` | `{item}` | `opcheck_runner` | srv `/opcheck/run_item` |
| `offer_calib` / `offer_opcheck` | `{item}` | WebUI | 導線を出す |
| `record_result` | `{item, result}` | `opcheck_runner` | |
| `feed_check_input` | `{pressed: bool}` | `opcheck_runner` | |
| `begin_wizard` / `run_measurement` / `build_preview` / `apply_and_verify` / `commit_calib` / `revert_calib` / `discard_calib` | `{item}` | `calib_runner` | srv |

**`th_state` は effect を `/system/state.pending_effects` に載せず、各ノードへサービスで送る。**
宛先は上表が正。**UI 向けの effect だけは `/system/state` に載せて配る**（W-n の開閉・案内キー）。

### 3.4 `spec_ref` は行番号ではなく ID で書く

正本 `Spec-modes.md` §3.1.1 / §3.1.2 の表に **ID 列を足す**（`SM-3.1.1-01` / `SM-3.1.2-034`）。
行番号を使うと、正本を 1 行編集した瞬間に全 `spec_ref` がずれる。

> **申し送り**: 正本側に ID 列を足す作業が要る。[DetailedDesign-open.md](DetailedDesign-open.md) §3。

テストが正本を読む経路も決めておく: `th_testing` の `conftest.py` が
`docs/plan/spec/Spec-modes.md` をリポジトリルートからの相対パスで開く
（colcon の install space には入らないので `--symlink-install` 前提。CI では repo root を渡す）。

### 3.5 `ui.goto` の `kind` → モード写像

| `arg.kind` | モード |
| --- | --- |
| `PANEL` | `PANEL_NAV` |
| `HOME` | `HOME_NAV` |
| `SUMMON` | `SUMMON` |

`to_mode: $arg.kind` はこの表を通してから解決する。**`PANEL` というモードは存在しない。**

---

## 4. 遷移表

### 4.1 共通（`layer: 0`）— `Spec-modes.md` §3.1.1 ＋ §4.1

| id | mode | state | event | guard | to_mode | to_state | effects |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `C-01` | `*` | `*` | `ui.jog.hold` | `jog_allowed` | `=` | `PAUSE` | `set_jog{on:true}` |
| `C-02` | `*` | `PAUSE` | `sys.jog_lease_expired` | — | `=` | `PAUSE` | `set_jog{on:false}` |
| `C-03` | `*` | `*` | `fault.recoverable` | `fault_stops_mode` | `=` | `PAUSE` | `open_window{id:W-1}` |
| `C-04` | `*` | `PAUSE` | `ui.resume_yes` | `fault_cleared` | `=` | `$resume_run` | `close_window{id:W-1}` |
| `C-05` | `*` | `PAUSE` | `ui.resume_no` ／ `ui.resume_ack` | `fault_cleared` | `=` | **`$resume_state`** | `close_window{id:W-1}` |
| **`C-06a`** | `*` | `*` | `fault.critical` | — | `ESTOP` | `NONE` | `latch_prev` |
| **`C-06b`** | `*` | `*` | `ui.estop.press` | `estop_ui_allowed` | `ESTOP` | `NONE` | `latch_prev` |
| **`C-06r`** | `CARRY` | `NONE` | `ui.estop.press` | — | — | — | **`reject: true`**（`reject_reason_key: estop_disabled_in_carry`） |
| `C-07` | `*` | `*` | `hw.estop.press` | `not_checking_estop` | `CARRY` | `NONE` | `latch_prev`, `open_window{id:W-2}` |
| `C-08` | `*` | `*` | `ui.finish` | **`can_finish`** | `IDLE` | `NONE` | `ask_save_if_unsaved`, `clear_prev` |
| `C-09` | `ESTOP` | `NONE` | `ui.estop.release` | `no_critical_fault` | `IDLE` | `NONE` | `clear_prev` |
| **`C-09f`** | `ESTOP` | `NONE` | `ui.resume_ack` | **`fault_cleared_and_ui_released`** | `IDLE` | `NONE` | `clear_prev`, `close_window{id:W-1}` |
| `C-10` | `CARRY` | `NONE` | `hw.estop.release` | — | `=` | `=` | `show_resume` |
| `C-11` | `CARRY` | `NONE` | `ui.carry_resume` | **`hw_released_and_no_critical`** | `$prev_mode` | `$prev_state` | `close_window{id:W-2}` |
| `C-12` | `CARRY` | `NONE` | `ui.finish` | `hw_released` | `IDLE` | `NONE` | `clear_prev` |
| `C-13` | `IDLE` ／ `MANUAL` | `*` | `ui.enter_mode` | `mode_entry_allowed` | `$arg.mode` | `$initial` | — |
| `C-14` | `*` | `*` | `ui.screen` | — | `=` | `=` | `set_screen{screen_id}` |
| **`C-15`** | `IDLE` | `NONE` | `ui.goto` | `goto_allowed` | **`$arg.kind`**（§3.5） | `$initial` | — |

**`C-06r` / `C-09f` / `C-15` は新設。**それぞれ次の穴を塞ぐ。

| 行 | 塞ぐ穴 |
| --- | --- |
| `C-06a` / `C-06b` | **1 行にまとめると `CARRY` 中の重大フォルトまで拒否される。**`estop_ui_allowed` は UI 起因にだけ掛けるガードなので、事象ごとに行を割る |
| `C-06r` | **`CARRY` 中の UI 非常停止を受理して `prev_*` を `CARRY` に上書きし、復帰先を壊していた**（`F-26`）。`layer: 0` で `C-06b` より先に置く |
| `C-09f` | **`fault.critical` で入った `ESTOP` から出る行が無く、トラップ状態だった**（`ui.estop.release` は UI ボタンを押していないので来ない）。**あわせて `attributes.yaml` の `ESTOP` を `resume: ack_only` / `resume_state: NONE` にし、W-1 に「確認」を出す**（`resume: none` のままだと UI が選択肢を出さず、この行を起こせない） |
| `C-11` | **重大フォルトが継続していても押下前のモードへ戻れてしまった。**`Spec-safety.md` §3.5「重大フォルトの解除後は `IDLE` のみ」に反する |
| `C-15` | **試験画面（S-21）から `PANEL_NAV` / `SUMMON` / `HOME_NAV` に入る行が無く、試験を開始できなかった**（`Spec-modes.md` §4.2「行き先を選んだ時点で入る」） |

**`latch_prev` は `mode ∈ {ESTOP, CARRY}` のときは記録しない**（§3.3）。
`ESTOP` 中の物理押下・`CARRY` 中の重大フォルトで復帰先が壊れるのを防ぐ。

### 4.1.1 ガードの定義（**全 28 件**）

**`guards.py` の実装はこの表が正。**`Context` のフィールドだけで判定できること。

| ガード | 真になる条件 | 参照する `Context` |
| --- | --- | --- |
| `jog_allowed` | 下の除外表のいずれにも当たらない | `flags`, `zone` |
| `fault_stops_mode` | 下の表で true | `fault_type` |
| `fault_cleared` | `not fault_active` | `fault_active` |
| `fault_cleared_and_ui_released` | `not fault_active and not ui_estop and not hw_estop` | 同上 |
| `estop_ui_allowed` | `mode != "CARRY"` | — |
| `not_checking_estop` | `not (mode == "OPCHECK" and check_item == "ESTOP")` | `check_item` |
| `checking_estop_item` | `mode == "OPCHECK" and check_item == "ESTOP"` | `check_item` |
| `no_critical_fault` | `fault_severity != "CRITICAL"` | `fault_severity` |
| `hw_released` | `not hw_estop` | `hw_estop` |
| **`hw_released_and_no_critical`** | `not hw_estop and fault_severity != "CRITICAL"` | `hw_estop`, `fault_severity` |
| **`leash_slack`** | `not leash_taut` | `leash_taut` |
| **`can_finish`** | `mode not in {"ESTOP"}` かつ（`mode != "CARRY"` または `not hw_estop`） | `hw_estop` |
| `mode_entry_allowed` | `mode_entry.yaml` が許し、かつ前提条件（§`DetailedDesign-transit.md` §0.4）を満たす | `route_ids`, `flags`, `leash_present`, `camera_present` |
| `goto_allowed` | `not flags["working"]` かつ `kind == "PANEL"` なら該当ピンが存在 | `flags`, `pin_kinds`, `arg` |
| `not_working` | `not flags["working"]` | `flags` |
| `candidate_exists` | `candidate_count > 0` | `candidate_count` |
| `target_selected` | `target_selected` | 同名 |
| `target_confident` | `target_selected and target_confident` | 同 |
| `route_arg_valid` | `arg["id"] in route_ids` または `arg.get("new") is True` | `route_ids`, `arg` |
| `route_exists` | `arg["id"] in route_ids` | 同 |
| `home_pin_exists` | `"HOME" in pin_kinds` | `pin_kinds` |
| `map_update` | `flags["map_update"] and map_update_available` | `flags` |
| `line_visible` | `line_visible` | 同名 |
| `leash_taut` | `leash_taut` | 同名 |
| `preview_sane` | `calib_preview_sane` | 同名 |
| `check_result_ok` | `check_result == "OK"` | `check_result` |
| `ng_and_calibrable` | `check_result == "NG" and check_item in {"IMU","LIDAR"}` | 同 |
| `ng_and_not_calibrable` | `check_result == "NG" and check_item in {"ESTOP","MOTOR"}` | 同 |

> **改名**: `ok` → **`check_result_ok`**。「何が ok なのか読めない」ため。

**`C-01` のガード `jog_allowed` が false になる条件**（`Spec-modes.md` §3.1.1 の除外）:

| 除外 | 理由 |
| --- | --- |
| `mode ∈ {INIT, IDLE, ESTOP, CARRY, OPCHECK, CALIB}` | 駆動が切れている、状態を持たない、または手順そのものが動きを規定している。**`IDLE` は `Spec-modes.md` §6 属性表で「ジョグ介入 不可」**（入れると `IDLE/PAUSE` という未定義状態が生まれる） |
| `mode == SUMMON and state == WAIT_CLEAR` | **ゲートが止めている状況で機体を動かさない**（`F-28` の除外） |
| **`mode ∈ {MANUAL, TEACH_MANUAL}`** | **スティックそのものが走行操作である**（`F-31`） |

**`OPCHECK` / `CALIB` は `PAUSE` を持たない。**したがって `C-03`（回復フォルト → `PAUSE`）も
この 2 モードには効かせない。`fault_stops_mode` の除外に加え、
復帰は `attributes.yaml` の `resume_state: LIST` で行う（`Spec-modes.md` §6「確認 1 択 → `LIST`」）。
**`IDLE` も同じ**（状態は `NONE` のみ）。

**`C-03` のガード `fault_stops_mode`**:

| 条件 | 結果 |
| --- | --- |
| `fault_type == "PERSON_TRACKER_LOST"` かつ `mode ∉ {FOLLOW, TEACH_FOLLOW, SUMMON}` | **false**（`PAUSE` にしない） |
| `fault_type == "PERSON_TRACKER_LOST"` かつ `mode == PREP` | **false**（登録を拒否するだけ・`C-15`） |
| **`mode ∈ {IDLE, INIT, OPCHECK, CALIB}`** | **false。**`PAUSE` を持たないモードなので落とせない（§4.1.1 末尾） |
| それ以外 | true |

`OPCHECK` / `CALIB` は代わりに `T-OPC-08` / `T-CAL-08` が `LIST` へ落とす（回復フォルトのみ）。

**`C-04` の `$resume_run`**: モードごとの「走行状態」。`attributes.yaml` の `run_state` を引く
（`FOLLOW`→`RUN`、`TEACH_*`→`REC`、`PANEL_NAV`→`NAV` …）。
属性表で `resume: ack_only` のモード（`MANUAL` / `TEACH_MANUAL` / `SUMMON` / `OPCHECK` / `CALIB`）では
`ui.resume_yes` 自体を UI が出さない（§7）。

### 4.2 モード内（`layer: 10`）— `Spec-modes.md` §3.1.2

`Spec-modes.md` §3.1.2 は **75 行**（初版 64 行 ＋ §9 の反映で 11 行）。
**「A／B」と併記されている契機を 1 行 1 事象に割り、`LINE` / `LEASH` の行も含める**と
詳細設計側は **106 行**になる。

**行数は主張ではなく機械的に数える。**§11 のテスト 2（正本 → 詳細の欠落）と
**テスト 2r（詳細 → 正本の過剰）**の両方で突き合わせる。
片方向だけだと**正本に無い遷移を足したことに気づけない**
（`Spec-open.md` §7.1 手順 6 が禁じている）。

**`to_mode` 列を持つのは、モードを跨ぐ行があるモードの表だけ**
（`INIT` / `TEACH_*` / `PANEL_NAV` / `AT_PANEL` / `SUMMON` / `HOME_NAV` / `OPCHECK` / `CALIB`）。
それ以外の表は `to_mode` を省略し、すべて `=` とみなす。§4.3 に索引がある。

**`spec_ref` はここには書かない。**`transitions.yaml` の各行に持たせる
（表に列を足すと読めなくなるため）。**値は正本の ID**（§3.4）。
**正本への ID 列追加（[open](DetailedDesign-open.md) §3 A-1）は `WP-STATE-01` の着手前提**である。

#### `INIT`

| id | state | event | guard | to_mode | to_state | effects |
| --- | --- | --- | --- | --- | --- | --- |
| `T-INIT-01` | `CHECK` | `evt.link_ok` | — | `IDLE` | `NONE` | `enable_main_menu` |
| `T-INIT-02` | `CHECK` | `sys.link_timeout` | — | `=` | `=` | `restart_control_stack` |
| `T-INIT-03` | `CHECK` | `hw.estop.press` | — | `=` | `=` | `guide("estop_held_at_boot")` — **`C-07` を打ち消す**（`override_common`） |

`T-INIT-03` は `CL-B-6`（起動時に押されたまま）。`CARRY` へ落とすと運用開始の案内が出せない。

#### `FOLLOW`

| id | state | event | guard | to_state | effects |
| --- | --- | --- | --- | --- | --- |
| `T-FOLLOW-01` | `SELECT` | `ui.select_target` | `candidate_exists` | `SELECT` | `set_target` |
| `T-FOLLOW-02` | `SELECT` | `evt.auto_selected` | — | `SELECT` | `set_target` |
| `T-FOLLOW-03` | `SELECT` | `ui.confirm` | `target_selected` | `CONFIRM` | `face_target` |
| `T-FOLLOW-04` | `CONFIRM` | `ui.run` | `target_confident` | `RUN` | — |
| `T-FOLLOW-05` | `RUN` | `ui.stop` | — | `PAUSE` | — |
| `T-FOLLOW-06` | `RUN` | `evt.target_lost` | — | `SELECT` | `clear_selection`, `guide{key:...}` |
| `T-FOLLOW-07` | `PAUSE` | `ui.run` | `target_selected` | `RUN` | — |

`T-FOLLOW-07` は `C-01`（ジョグで `PAUSE` に落ちる）と
`Spec-modes.md` §8「再開は試験員が改めて『走行』を押す」から必要。
**正本 `Spec-modes.md` §3.1.2 に反映済み**（§9-(g) ／ `Spec-open.md` F-35）。

#### `MANUAL`

| id | state | event | guard | to_state |
| --- | --- | --- | --- | --- |
| `T-MANUAL-01` | `PAUSE` ／ **`RUN`** | `ui.jog.hold` | — | `RUN` |
| `T-MANUAL-02` | `RUN` | `sys.jog_lease_expired` ／ `ui.stop` | — | `PAUSE` |

**`override_common: true`**（`C-01` はこの 2 モードに効かない）。

**`T-MANUAL-01` は `RUN` からの自己ループを含む。**
リースは 5 Hz 以上で送り続けるので、`PAUSE` 発だけにすると
**`RUN` 中の毎秒 5 回の `ui.jog.hold` がすべて `not_allowed` で拒否される。**

**あわせて規則を 1 つ置く**:

> **リース時刻の更新は、遷移の採否と独立に行う。**
> `th_state` は `ui.jog.hold` を受け取った時点で `last_jog_ms` を更新し、
> そのあとで `step()` に渡す。`accepted=false` でもリースは延びる。

これが無いと、拒否された瞬間からリースが延びなくなり `jog_lease_ms` 後に必ず停止する。
`TEACH_MANUAL` の `REC` も同じ（`T-TEACHM-01` に `REC` 発の自己ループを含める）。

#### `TEACH_FOLLOW` ／ `TEACH_MANUAL`

| id | mode | state | event | guard | to_state | effects |
| --- | --- | --- | --- | --- | --- | --- |
| `T-TEACH-01` | 両方 | `ROUTE_SEL` | `ui.route_select` | `route_arg_valid` | `REC` | `start_record` |
| `T-TEACH-02` | 両方 | `REC` | `ui.stop` | — | `PAUSE` | — （**終点を確定しない**） |
| `T-TEACH-03` | `TEACH_FOLLOW` | `PAUSE` | `ui.run` | — | `REC` | `resume_record`（一時停止は記録に残さない） |
| **`T-TEACH-03M`** | **`TEACH_MANUAL`** | `PAUSE` | **`ui.jog.hold`** | — | `REC` | 同上。**S-13 に「走行」ボタンが無い**（`Spec-webui.md` §3.4） |
| `T-TEACH-04` | 両方 | `REC` ／ `PAUSE` | `ui.save` | — | `SAVED` | `finalize_route` |
| `T-TEACH-05` | `TEACH_FOLLOW` | `SAVED` | `ui.run` | — | `REC` | `resume_record` |
| **`T-TEACH-05M`** | **`TEACH_MANUAL`** | `SAVED` | **`ui.jog.hold`** | — | `REC` | 同上 |
| `T-TEACH-06` | 両方 | `*` | `evt.record_broken` | — | **§4.3 参照** | `ask_save` |

`T-TEACH-05` / `-05M` は**正本 `Spec-modes.md` §3.1.2 に反映済み**（§9-(d) ／ `Spec-open.md` F-32）。
続けて `ui.save` した場合は `F-04` に従い**新版**として保存する。
`T-TEACH-06` は `Spec-modes.md` §5「そのモードを続けられないときだけ `IDLE` へ落とす」の実体。

`TEACH_MANUAL` は `MANUAL` と同じく `override_common: true`（`T-MANUAL-01/02` 相当の行を持つ）。
**`-03M` / `-05M` を分けたのは、手動系ではスティックが走行操作そのものだからである**
（§9-(j) ／ `Spec-open.md` F-38）。`ui.run` を `TEACH_MANUAL` に残すと、
**押せるボタンが存在しない遷移**が表に入る。

#### `REPLAY`

| id | state | event | guard | to_state | effects |
| --- | --- | --- | --- | --- | --- |
| `T-REPLAY-01` | `ROUTE_SEL` | `ui.route_select` | `route_exists` | `LOCALIZE` | `load_route` |
| `T-REPLAY-02` | `LOCALIZE` | `evt.localize_low` | — | `LOCALIZE` | `widen_search` |
| `T-REPLAY-03` | `LOCALIZE` | `ui.localize_global` | — | `LOCALIZE` | `global_localize` |
| `T-REPLAY-04` | `LOCALIZE` | `evt.localize_done` | — | `READY` | — |
| `T-REPLAY-05` | `READY` | `ui.run` | — | `RUN` | `rotate_to_start_yaw` |
| `T-REPLAY-06` | `RUN` | `ui.stop` | — | `PAUSE` | — |
| `T-REPLAY-07` | `PAUSE` | `ui.run` | — | `RUN` | `resume_path`（**再プランしない**） |
| `T-REPLAY-08` | `PAUSE` | `ui.save` | `map_update` | `SAVED` | `commit_map_patch` |
| `T-REPLAY-09` | `RUN` | `evt.arrived` | — | `PAUSE` | `guide("route_end")` |
| `T-REPLAY-10` | `SAVED` | `ui.run` | — | `RUN` | `resume_path`（**再プランしない**） |

`T-REPLAY-09` / `-10` は**正本 `Spec-modes.md` §3.1.2 に反映済み**
（§9-(h) / §9-(d) ／ `Spec-open.md` F-36 / F-32）。
終端で自動停止するのは「到着判定」ではなく**経路データが尽きただけ**なので、
`F-02`（自動停止は `LINE` のみ）とは矛盾しない。`PAUSE` に落として案内し、終了は人が押す。

#### `PREP`

| id | state | event | guard | to_state | effects |
| --- | --- | --- | --- | --- | --- |
| `T-PREP-01` | `MAPPING` | `ui.register` | `target_selected and target_confident` | `REGISTER` | `begin_two_point` |
| `T-PREP-02` | `REGISTER` | `evt.register_ok` | — | `MAPPING` | `place_pin` |
| `T-PREP-03` | `REGISTER` | `evt.plane_done` | — | `MAPPING` | `place_pin` |
| `T-PREP-04` | `REGISTER` | `evt.register_rejected` | — | `MAPPING` | `guide{key:...}` |
| `T-PREP-05` | `REGISTER` | `evt.target_lost` | — | `MAPPING` | `reject_register`, `guide{key:...}` |
| `T-PREP-06` | `MAPPING` | `ui.return_home` | `home_pin_exists` | `RETURN` | — |
| `T-PREP-07` | `RETURN` | `evt.arrived` | — | `EDIT` | — |
| `T-PREP-08` | `MAPPING` ／ `EDIT` | `ui.map_edit` | — | `EDIT` | — |
| `T-PREP-09` | `EDIT` | `ui.run` | — | `MAPPING` | — |
| `T-PREP-10` | `*` | `ui.stop` | — | `PAUSE` | `keep_all`（地図・ピン・修正を保持） |
| `T-PREP-11` | `PAUSE` | `ui.run` | — | `$prev_sub` | — |
| `T-PREP-12` | `*` | `ui.save` | — | `SAVED` | `commit_venue_map` |
| `T-PREP-13` | `SAVED` | `ui.run` | — | `MAPPING` | — |

`T-PREP-08` / `-09` / `-11` / `-13` は**正本 `Spec-modes.md` §3.1.2 に反映済み**
（§9-(d) / §9-(g) ／ `Spec-open.md` F-32 / F-35）。
`$prev_sub` は `PAUSE` に入る直前の `PREP` 内状態を 1 段ラッチしたもの。

#### `PANEL_NAV`

| id | state | event | guard | to_mode | to_state | effects |
| --- | --- | --- | --- | --- | --- | --- |
| `T-PNAV-01` | `NAV` | `evt.arrived` | — | `=` | `ALIGN` | — |
| `T-PNAV-02` | `NAV` | `ui.stop` | — | `=` | `PAUSE` | `cancel_follow_path` |
| `T-PNAV-03` | `PAUSE` | `ui.run` | — | `=` | `NAV` | `resume_follow_path`（**再検索しない**） |
| `T-PNAV-04` | `NAV` | `evt.blocked` | — | `=` | `BLOCKED` | `open_window(W-5)` |
| `T-PNAV-05` | `BLOCKED` | `evt.unblocked` | — | `=` | `NAV` | `close_window(W-5)` |
| `T-PNAV-06` | `BLOCKED` | `ui.stop` | — | `=` | `PAUSE` | — |
| `T-PNAV-07` | `BLOCKED` | `ui.reroute` | — | `=` | `NAV` | `replan` |
| `T-PNAV-08` | `ALIGN` | `evt.align_done` | — | `AT_PANEL` | `IDLE_P` | — |

#### `AT_PANEL`

| id | state | event | guard | to_mode | to_state | effects |
| --- | --- | --- | --- | --- | --- | --- |
| `T-ATP-01` | `IDLE_P` | `ui.working` (`on=true`) | — | `=` | `WORKING` | — |
| `T-ATP-02` | `WORKING` | `ui.working` (`on=false`) | — | `=` | `IDLE_P` | — |
| `T-ATP-03` | `IDLE_P` | `ui.jog.hold` | — | `=` | `PAUSE` | `set_jog(true)` |
| `T-ATP-04` | `PAUSE` | `sys.jog_lease_expired` | — | `=` | **`IDLE_P`** | `set_jog(false)` |
| `T-ATP-05` | `IDLE_P` ／ `WORKING` | `ui.goto` | `goto_allowed` | `$arg.kind`（§3.5） | `$initial` | — |

**`T-ATP-06` は置かない。**「作業中に行き先を選ぼうとした」の拒否は
**ガード `goto_allowed` の否定**で表す（`not flags["working"]`）。
遷移表に「受理された拒否」（`accepted=true` なのに `reject` を effect に持つ行）を作らない
—— `Decision` の定義上、`reject_reason_key` は `accepted=false` のときだけ入るため、
UI がエラー表示すべきか判断できなくなる。

**`T-ATP-04` は `override_common: true`。**`C-02` は「`PAUSE` のまま」だが、`AT_PANEL` は `IDLE_P` へ戻す。
**正本の中で食い違っていた箇所だが、`AT_PANEL` 側を採り、`Spec-modes.md` §3.1.1 に
「例外はこの 1 つに限る」と明記済み**（§9-(b) ／ `Spec-open.md` F-30）。`AT_PANEL` 側を採る理由は
「到着ずれを手で詰める」用途で、詰め終わったら盤前の待機に戻るのが自然だから。

#### `SUMMON`

| id | state | event | guard | to_mode | to_state | effects |
| --- | --- | --- | --- | --- | --- | --- |
| `T-SUM-01` | `POINT` | `evt.two_point_done` | — | `=` | `WAIT_CLEAR` | `disable_jog_ui` |
| `T-SUM-02` | `WAIT_CLEAR` | `evt.clear_ok` | — | `=` | `NAV` | `enable_jog_ui` |
| `T-SUM-03` | `WAIT_CLEAR` | `evt.clear_timeout` | — | `=` | `POINT` | `enable_jog_ui`, `guide{key:...}` |
| `T-SUM-04` | `WAIT_CLEAR` | `evt.target_lost` | — | `=` | `POINT` | `enable_jog_ui`, `guide{key:...}` |
| `T-SUM-05` | `WAIT_CLEAR` | `ui.abort` | — | `=` | `POINT` | `enable_jog_ui` |
| `T-SUM-06` | `NAV` | `ui.stop` | — | `=` | `PAUSE` | `cancel_follow_path` |
| `T-SUM-07` | `PAUSE` | `ui.run` | — | `=` | `NAV` | `resume_follow_path`（**退避待ちはやり直さない**） |
| `T-SUM-08` | `NAV` | `evt.blocked` | — | `=` | `BLOCKED` | `open_window(W-5)` |
| `T-SUM-09` | `BLOCKED` | `evt.unblocked` | — | `=` | `NAV` | `close_window(W-5)` |
| `T-SUM-10` | `BLOCKED` | `ui.stop` | — | `=` | `PAUSE` | — |
| **`T-SUM-13`** | `BLOCKED` | `ui.reroute` | — | `=` | `NAV` | `replan` |
| `T-SUM-11` | `NAV` | `evt.arrived` | — | `=` | `ALIGN` | — |
| `T-SUM-12` | `ALIGN` | `evt.align_done` | — | `AT_PANEL` | `IDLE_P` | — |

#### `HOME_NAV`

| id | state | event | guard | to_mode | to_state | effects |
| --- | --- | --- | --- | --- | --- | --- |
| `T-HNAV-01` | `NAV` | `evt.arrived` | — | `IDLE` | `NONE` | `guide("home_arrived")` |
| `T-HNAV-02` | `NAV` | `ui.stop` | — | `=` | `PAUSE` | `cancel_follow_path` |
| `T-HNAV-03` | `PAUSE` | `ui.run` | — | `=` | `NAV` | `resume_follow_path` |
| `T-HNAV-04` | `NAV` | `evt.blocked` | — | `=` | `BLOCKED` | `open_window(W-5)` |
| `T-HNAV-05` | `BLOCKED` | `evt.unblocked` | — | `=` | `NAV` | `close_window(W-5)` |
| `T-HNAV-06` | `BLOCKED` | `ui.stop` | — | `=` | `PAUSE` | — |
| `T-HNAV-07` | `BLOCKED` | `ui.reroute` | — | `=` | `NAV` | `replan` |

#### `OPCHECK`

| id | state | event | guard | to_state | effects |
| --- | --- | --- | --- | --- | --- |
| `T-OPC-01` | `LIST` | `ui.check_item` | — | `RUNNING_CHECK` | `start_monitor` |
| `T-OPC-02` | `RUNNING_CHECK` | `evt.check_result` | `ng_and_calibrable` | `LIST` | `offer_calib` |
| `T-OPC-03` | `RUNNING_CHECK` | `evt.check_result` | `ng_and_not_calibrable` | `REPAIR` | — |
| `T-OPC-04` | `RUNNING_CHECK` | `evt.check_result` | `check_result_ok` | `LIST` | `record_result` |
| `T-OPC-05` | `RUNNING_CHECK` | `hw.estop.press` ／ `hw.estop.release` | `checking_estop_item` | `RUNNING_CHECK` | `feed_check_input` — **`override_common`（`C-07` を打ち消す）** |
| `T-OPC-06` | `REPAIR` | `ui.stop` | — | `LIST` | — |
| `T-OPC-07` | `LIST` | `ui.enter_mode` | `mode_entry_allowed` | **§4.3 参照** | — |
| `T-OPC-08` | `*` | `fault.recoverable` | — | `LIST` | `abort_check` — **`override_common`** |

`ng_and_calibrable` ＝ `item ∈ {IMU, LIDAR}`、`ng_and_not_calibrable` ＝ `item ∈ {ESTOP, MOTOR}`（`F-16`）。

#### `CALIB`

| id | state | event | guard | to_state | effects |
| --- | --- | --- | --- | --- | --- |
| `T-CAL-01` | `LIST` | `ui.calib_item` | — | `S1` | `begin_wizard` |
| `T-CAL-02` | `S1` | `ui.calib_next` | — | `S2` | `run_measurement` |
| `T-CAL-03` | `S2` | `evt.calib_step_done` | — | `S3` | `build_preview` |
| `T-CAL-04` | `S3` | `ui.calib_next` | `preview_sane` | `S4` | `apply_and_verify` |
| `T-CAL-05` | `S4` | `evt.calib_step_done` | — | `LIST` | `commit_calib`, `offer_opcheck` |
| `T-CAL-06` | `S4` | `evt.calib_verify_ng` | — | `S2` | `revert_calib` |
| `T-CAL-07` | `*` | `ui.abort` | — | `LIST` | `discard_calib` |
| `T-CAL-08` | `*` | **`fault.recoverable` のみ** | — | `LIST` | `discard_calib` — **`override_common`**（`K-r5`「途中再開はしない」） |
| `T-CAL-09` | `LIST` | `ui.enter_mode` | `mode_entry_allowed` | **§4.3 参照** | — |

`T-CAL-08` は `C-03` より優先する。校正は**補正値を確定せずに終わる**のが正しい振る舞いで、
`PAUSE` にして再開させると中途半端な測定値が残る。

> **`fault.critical` と `hw.estop.press` を握り潰してはいけない。**
> `Spec-modes.md` §4.1 は「重大フォルト＝**どのモードからでも** `ESTOP`」
> 「物理押下＝どのモードからでも `CARRY`（例外は始業点検の非常停止確認中の 1 つだけ）」と定めており、
> `CALIB` を 2 つ目の例外にすると**安全チェーンの層 1・2 がモードによって効いたり効かなかったりする。**
> 補正値の破棄は `C-06` / `C-07` の `latch_prev` と同時に `discard_calib` を走らせて実現する
> （`th_state` が `mode == CALIB` のときだけ追加で発火させる。遷移表ではなく effect の条件）。

#### `LINE` ／ `LEASH`（**実装は後回し。表だけ先に確定させる**）

| id | mode | state | event | guard | to_state | effects |
| --- | --- | --- | --- | --- | --- | --- |
| `T-LINE-01` | `LINE` | `SETUP` | `evt.setup_done` | — | `PLANNED` | `plan_line_route` |
| `T-LINE-02` | `LINE` | `PLANNED` | `ui.run` | — | `RUN` | — |
| `T-LINE-03` | `LINE` | `RUN` | `evt.arrived` | — | `ARRIVED` | **7 方式で唯一の自動停止** |
| `T-LINE-04` | `LINE` | `RUN` | `evt.line_lost` | — | `PAUSE` | `guide{key:...}` |
| `T-LINE-05` | `LINE` | `RUN` | `ui.stop` | — | `PAUSE` | — |
| `T-LINE-06` | `LINE` | `PAUSE` ／ `ARRIVED` | `ui.run` | `line_visible` | `RUN` | — |
| `T-LEASH-01` | `LEASH` | `DEV_CHECK` | `evt.leash_present` | — | `READY` | — |
| `T-LEASH-02` | `LEASH` | `READY` ／ `PAUSE` | `ui.run` | `leash_taut` | `RUN` | — |
| `T-LEASH-03` | `LEASH` | `READY` ／ `PAUSE` | `ui.run` | `leash_slack` | `HOLD` | — |
| `T-LEASH-04` | `LEASH` | `RUN` | `evt.leash_slack` | — | `HOLD` | — |
| `T-LEASH-05` | `LEASH` | `HOLD` | `evt.leash_taut` | — | `RUN` | **自動で再開する** |
| `T-LEASH-06` | `LEASH` | `RUN` ／ `HOLD` | `ui.stop` | — | `PAUSE` | **以後、張力がかかっても発進しない**（`F-27`） |
| `T-LEASH-07` | `LEASH` | `DEV_CHECK` | `evt.leash_absent` | — | `DEV_CHECK` | `guide("leash_not_connected")` |

---

### 4.3 モードを跨ぐ行（**索引。再掲ではない**）

**§4.2 の行のうち `to_mode` が `=` でないものはこれだけ**、という索引である。
**行の定義は §4.2 が正。**ここに effects や guard を書かない（二重管理になる）。

| id | 遷移元 → 遷移先 | 定義の場所 |
| --- | --- | --- |
| `T-INIT-01` | `INIT` → `IDLE` | §4.2 `INIT` |
| `T-TEACH-06` | `TEACH_*` → `IDLE` | §4.2 `TEACH_*` |
| `T-PNAV-08` | `PANEL_NAV` → `AT_PANEL` | §4.2 `PANEL_NAV` |
| `T-SUM-12` | `SUMMON` → `AT_PANEL` | §4.2 `SUMMON` |
| `T-HNAV-01` | `HOME_NAV` → `IDLE` | §4.2 `HOME_NAV` |
| `T-ATP-05` | `AT_PANEL` → `$arg.kind`（§3.5 の写像） | §4.2 `AT_PANEL` |
| `T-OPC-07` | `OPCHECK` → `CALIB` | §4.2 `OPCHECK` |
| `T-CAL-09` | `CALIB` → `OPCHECK` | §4.2 `CALIB` |
| `C-06` / `C-06r` / `C-07` / `C-08` / `C-09` / `C-09f` / `C-11` / `C-12` / `C-13` / `C-15` | §4.1 の共通行 | §4.1 |

**`transitions.yaml` に入れるのは §4.1 と §4.2 の行だけ。**§4.3 から行を起こさない
（`id` の一意性が壊れる）。

---

## 5. ジョグ介入

**`Spec-modes.md` §8 の実装は 3 層に分かれる。権威は挙動ノード側の publish 停止である。**

| 層 | 役割 | 根拠 |
| --- | --- | --- |
| **`th_state`** | `ui.jog.hold` のリースを受け、`jog_active = true` かつ**元モードを `PAUSE` へ**。**これが権威** | `Spec-modes.md` §3.1.1・§8 |
| **挙動ノード** | `/system/state` を購読し、自モードの状態が `RUN`/`REC`/`NAV` でなければ **`/cmd_vel_behavior` への publish を止める** | 同 §8 |
| **`twist_mux`** | `/cmd_vel_manual` の priority 30 が、**触れてから挙動ノードが反応するまでの 1 tick を覆う**だけ | — |
| **`obstacle_limiter`** | **政策の選択にのみ使う**（自律＝停止／手動＝警告のみ、`AT_PANEL` なら `v_jog_panel`）。トピックのゲートには使わない | 同 §8 障害物行 |

### 5.1 優先度だけでは解けない

`/cmd_vel_manual` は既に priority 30 で最高だが、**`timeout: 1.0` があるため、
スティックを離して 1.0 s 後に `nav` / `behavior` が自動的に復活する。**
これは `Spec-modes.md` §3.1.1「勝手に走り出さない」の直接違反。
だから **FSM が `PAUSE` に落として挙動ノードを黙らせる**必要がある。

### 5.2 リースの制約

```
jog_lease_ms  ≥  /cmd_vel_manual の twist_mux timeout (1.0 s)
```

逆にすると「多重化はまだ手動を流しているのに FSM は `jog_active=false`」という窓ができる。

---

## 6. ゾーンの導出

**モードから引かない**（`F-19`）。`/ui/active_screen` から決める。3 値。

**実装は [DetailedDesign-names.md](DetailedDesign-names.md) §4.1 の `derive_limits()` が唯一の正。**
ここに擬似コードを再掲しない（2 実装になると、強弱合成と最小値合成のどちらを採るかで
**途絶時に `v_slow` が許可されてしまう**方に倒れうる）。

要点だけ再掲する。

| 項目 | 決定 |
| --- | --- |
| 判定窓 | `interacting` かつ最後の操作から `ui_active_window_s` 以内（`Spec-safety.md` §6.2 の「使用中」と同一） |
| 合成 | **速度上限の最小値。**ゾーンの強弱ではない |
| 途絶・使用中 0 台 | **停止**・`auto_brake = ON` |

**`ESTOP` / `CARRY` はゾーンを変えない。**ウィンドウを乗せるだけで画面は変わらないので、
下の画面のゾーンがそのまま保たれる（`Spec-modes.md` §7）。

---

## 7. `ESTOP` / `CARRY` のラッチ

**1 段だけラッチする。**入れ子にしない。

| 状況 | 挙動 |
| --- | --- |
| 動作系 → `ESTOP` | `prev_mode` / `prev_state` に記録。解除後は `IDLE` のみ（`prev_*` は捨てる） |
| 動作系 → `CARRY` | 同じく記録。**`ui.carry_resume` で `prev_*` へ戻る** |
| **`CARRY` 中に `ui.estop.press`** | **受け付けない**（`F-26`）。`reject_reason_key = "estop_disabled_in_carry"` を返し、W-2 に「駆動は既に切れています」と出す |
| **`CARRY` 中に `fault.critical`** | `ESTOP` へ移る。**`prev_*` は上書きしない**（`CARRY` を復帰先にしない） |
| **`ESTOP` 中に `hw.estop.press`** | `CARRY` へ移る。**`prev_*` は上書きしない** |
| `ESTOP` / `CARRY` 中の `ui.finish` | `IDLE` へ。`prev_*` を捨てる |

### 7.1 フォルト解消は遷移ではない

`fault.cleared` は**遷移を起こさない**。W-1 を「再開しますか」に変えるだけである。
実際に遷移するのは `ui.resume_yes` / `ui.resume_no` / `ui.resume_ack`（`C-04` / `C-05`）。

**どの選択肢を出すかは UI がハードコードしない。**`attributes.yaml` の `resume` を読む。

| `resume` | 出す選択肢 | 対象モード |
| --- | --- | --- |
| `yes_no` | はい／いいえ | `FOLLOW` / `TEACH_FOLLOW` / `REPLAY` / `LINE` / `LEASH` / `PREP` / `PANEL_NAV` / `HOME_NAV` |
| `ack_only` | 確認（1 択） | `MANUAL` / `TEACH_MANUAL` / `SUMMON` / `OPCHECK` / `CALIB` |
| `none` | 出さない | `INIT` / `IDLE` / `ESTOP` / `CARRY` / `AT_PANEL` |

`ack_only` のうち `SUMMON` だけ復帰先が `POINT`（2 点指示からやり直す）、
`OPCHECK` / `CALIB` は `LIST`。これも `attributes.yaml` の `resume_state` に書く。

---

## 8. `attributes.yaml`（`Spec-modes.md` §6 の転写）

### 8.1 キーの意味

| キー | 値 | 用途 |
| --- | --- | --- |
| `initial_state` | 状態名 | `C-13` / `C-15` の `$initial` |
| `run_state` | 状態名 | `C-04` の `$resume_run` |
| `resume` | `yes_no` / `ack_only` / `none` | §7.1。W-1 の選択肢 |
| `resume_state` | 状態名 | `C-05` の `$resume_state` |
| `needs_tracker` | `required` / `optional` / `unused` / `keep` | `Spec-modes.md` §9 |
| `speed_limit` | パラメータ名 or `stop` | モード由来の上限（画面由来との **min** を採る） |
| `auto_brake_default` | `on` / `off` / `on_locked` | `on_locked` は無効化不可 |
| `jog` | `allowed` / `denied` / `is_drive` | `is_drive` は `MANUAL` / `TEACH_MANUAL` |
| `has_record` | bool | 「保存」ボタンを出すか |

**ゾーンは属性表に持たせない。**画面から引く（[names](DetailedDesign-names.md) §4.1）。

### 8.2 実値（**18 行 × 9 列。これが `attributes.yaml` の中身**）

| モード | `initial_state` | `run_state` | `resume` | `resume_state` | `needs_tracker` | `speed_limit` | `auto_brake_default` | `jog` | `has_record` |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `INIT` | `CHECK` | — | `none` | — | `optional` | `stop` | `on_locked` | `denied` | false |
| `IDLE` | `NONE` | — | `none` | — | `optional` | `stop` | `on_locked` | `denied` | false |
| `ESTOP` | `NONE` | — | **`ack_only`** | **`NONE`** | `keep` | `stop` | `on_locked` | `denied` | false |
| `CARRY` | `NONE` | — | `none` | — | `keep` | `stop` | `on_locked` | `denied` | false |
| `FOLLOW` | `SELECT` | `RUN` | `yes_no` | `PAUSE` | `required` | `v_max` | `on_locked` | `allowed` | false |
| `MANUAL` | `PAUSE` | `RUN` | `ack_only` | `PAUSE` | `unused` | `v_max` | `off` | **`is_drive`** | false |
| `TEACH_FOLLOW` | `ROUTE_SEL` | `REC` | `yes_no` | `PAUSE` | `required` | `v_max` | `on_locked` | `allowed` | **true** |
| `TEACH_MANUAL` | `ROUTE_SEL` | `REC` | `ack_only` | `PAUSE` | `unused` | `v_max` | `off` | **`is_drive`** | **true** |
| `REPLAY` | `ROUTE_SEL` | `RUN` | `yes_no` | `PAUSE` | `unused` | `v_max` | `on_locked` | `allowed` | **true**※ |
| `LINE` | `SETUP` | `RUN` | `yes_no` | `PAUSE` | `unused` | `v_max` | `on_locked` | `allowed` | false |
| `LEASH` | `DEV_CHECK` | `RUN` | `yes_no` | `PAUSE` | `unused` | `v_leash` | `on_locked` | `allowed` | false |
| `PREP` | `MAPPING` | `MAPPING` | `yes_no` | `PAUSE` | `required`※※ | `v_slow` | `on_locked` | `allowed` | **true** |
| `PANEL_NAV` | `NAV` | `NAV` | `yes_no` | `PAUSE` | `unused` | `v_slow` | `on_locked` | `allowed` | false |
| `AT_PANEL` | `IDLE_P` | — | **`ack_only`** | **`IDLE_P`** | `unused` | `stop`※※※ | `on_locked` | `allowed` | false |
| `SUMMON` | `POINT` | `NAV` | **`ack_only`** | **`POINT`** | `required` | `v_slow` | `on_locked` | `allowed` | false |
| `HOME_NAV` | `NAV` | `NAV` | `yes_no` | `PAUSE` | `unused` | `v_slow` | `on_locked` | `allowed` | false |
| `OPCHECK` | `LIST` | — | **`ack_only`** | **`LIST`** | `unused` | `v_check` | `on_locked` | `denied` | false |
| `CALIB` | `LIST` | — | **`ack_only`** | **`LIST`** | `unused` | `v_calib` | `on_locked` | `denied` | false |

※ `REPLAY` の `has_record` は **`map_update` が ON のときだけ true**（`T-REPLAY-08` のガード）。
※※ `PREP` の `needs_tracker` は **`REGISTER` 状態のときだけ `required`**、他は `optional`。
※※※ `AT_PANEL` は停止だが、**`jog_active` の間だけ `v_jog_panel`**（`F-24`）。

**`resume_state` は全モード必須。**`yes_no` のモードは `PAUSE` を書く
（`C-05`（いいえ／確認）は全モードの `PAUSE` に効くので、`ack_only` のモードだけ定義すると
`FOLLOW` などで `$resume_state` が解決できず `validate()` ④ が落ちる）。

**`run_state` が `—` のモードは `ui.resume_yes` を出さない**（`C-04` が起こせない）。
ただし **`ESTOP` と `AT_PANEL` は `ack_only`** にする。

| モード | なぜ `ack_only` が要るか |
| --- | --- |
| `ESTOP` | `resume: none` だと W-1 に選択肢が出ず、**`C-09f` を起こせない**＝重大フォルトで入った `ESTOP` がトラップになる |
| `AT_PANEL` | `fault_stops_mode` の除外に入っていないので回復フォルトで `PAUSE` に落ちる。`resume: none` だと**出口の無い状態**になる（ジョグ中でなければ `sys.jog_lease_expired` も来ない） |

---

## 8.3 `mode_entry.yaml`（`Spec-modes.md` §4.2 の転写）

`C-13` / `C-15` / `T-OPC-07` / `T-CAL-09` のガード `mode_entry_allowed` が読む。

| 遷移元 | 入れる先 |
| --- | --- |
| `INIT` | `IDLE` **のみ**（`T-INIT-01` 経由。`ui.enter_mode` では入れない） |
| **`IDLE`** | `FOLLOW` / `MANUAL` / `TEACH_FOLLOW` / `TEACH_MANUAL` / `REPLAY` / `LINE` / `LEASH` / `PREP` / **`PANEL_NAV` / `SUMMON` / `HOME_NAV`** / `OPCHECK` / `CALIB` |
| `MANUAL` | `IDLE` ／ **`OPCHECK` / `CALIB`**（保守は `IDLE` と `MANUAL` から始められる） |
| `FOLLOW` / `TEACH_FOLLOW` / `TEACH_MANUAL` / `REPLAY` / `LINE` / `LEASH` | `IDLE` **のみ** |
| `PREP` | `IDLE` **のみ** |
| `PANEL_NAV` | `AT_PANEL`（到着）／ `IDLE` |
| `AT_PANEL` | `PANEL_NAV` / `SUMMON` / `HOME_NAV` / `IDLE` |
| `SUMMON` | `AT_PANEL`（到着）／ `IDLE` |
| `HOME_NAV` | `IDLE` |
| `OPCHECK` | `CALIB` ／ `IDLE` |
| `CALIB` | `OPCHECK` ／ `IDLE` |
| `ESTOP` | `IDLE` **のみ** |
| `CARRY` | 押下前のモード（`C-11`）／ `IDLE`（`C-12`） |

**表に無い組合せは拒否**（`reject_reason_key: mode_entry_denied`）。
**走行方式どうしの直接遷移は無い**（`CL-T-1`）。

`mode_entry_allowed` は**この表に加えて前提条件**も見る
（教示済み経路の有無・`tracker_enabled`・デバイス接続。[transit](DetailedDesign-transit.md) §0.4）。

---

## 9. 正本 `Spec-modes.md` §3.1 に見つかった矛盾・欠落（**正本へ反映済み・2026-08-16**）

**詳細設計で埋めたうえで、正本側にも戻した。**
`Spec-modes.md` §3.1 が「表に無い遷移は起こらない」と宣言している以上、
**詳細設計にしか無い遷移を残してはいけない**（この突き合わせは `Spec-open.md` §7.1 の 6 番目として常設した）。
決定の経緯は `Spec-open.md` §3.2（`F-29`〜`F-38`）が正本。

| # | 内容 | 本設計での扱い | 正本の反映先 |
| --- | --- | --- | --- |
| **(a)** | ジョグが離散イベント（「触れた」「手を離した」）で書かれている。WiFi の受信ギャップ（実測 0.5〜1.2 s）で解放が落ちると `jog_active` が永久にラッチする | **リース方式**にした（§5.2） | `Spec-modes.md` §3.1.1・§8 ／ `Spec-params.md` §4（`jog_lease_ms`）／ **F-29** |
| **(b)** | §3.1.1 行 2 は「`PAUSE`（jog）→ 手を離した → `PAUSE` のまま」、§3.1.2 の `AT_PANEL` 行は「`PAUSE` → 手を離した → `IDLE_P`」。**同じ表の中で食い違っている** | `AT_PANEL` 側を採り `override_common` にした（`T-ATP-04`） | `Spec-modes.md` §3.1.1（**例外はこの 1 つに限ると明記**）／ **F-30** |
| **(c)** | `MANUAL` / `TEACH_MANUAL` が §3.1.1 のジョグ除外リストに無い。§6 属性表は「―（本体が手動）」、`Spec-webui.md` §3.3 は「スティックそのものが走行操作」。**このままだと触った瞬間に `RUN`→`PAUSE` に落ちて走れない** | 除外に追加し、`T-MANUAL-01/02` を `override_common` にした | `Spec-modes.md` §3.1.1 除外表・§6・§8 ／ **F-31** |
| **(d)** | `SAVED` からの出口が「終了」しか無い。`TEACH_*` / `PREP` / `REPLAY` で**保存直後に固まる** | `T-TEACH-05` / `T-PREP-13` / **`T-REPLAY-10`** を追加 | `Spec-modes.md` §3・§3.1.2 ／ `Spec-transit.md` §3.2・§4.2 ／ `Spec-onsite.md` §2 ／ **F-32** |
| **(e)** | `ESTOP` / `CARRY` の入れ子（`ESTOP` 中の物理押下、`CARRY` 中の重大フォルト）が未定義 | §7 で 4 通りすべて明示した | `Spec-modes.md` §4.1（入れ子の小節を新設）／ **F-33** |
| **(f)** | `fault.cleared` が遷移として書かれているように読める | 遷移させず、選択肢は `attributes.yaml` から導出（§7.1） | `Spec-modes.md` §3.1.1 ／ `Spec-webui.md` §4.1 ／ **F-34** |
| **(g)** | `FOLLOW` / `PREP` の `PAUSE` からの復帰行が無い（`REPLAY` も同様だった） | `T-FOLLOW-07` / `T-PREP-11` / `T-REPLAY-07` | `Spec-modes.md` §3.1.2 ／ **F-35** |
| **(h)** | `REPLAY` が経路の終端に達したときの状態が無い | `T-REPLAY-09`（`PAUSE` ＋案内）を追加。自動停止は `LINE` のみ（`F-02`）という規則は守る | `Spec-modes.md` §3.1.2 ／ `Spec-transit.md` §0.2・§4.1（**到着判定ではないと明記**）／ **F-36** |
| **(i)** | §3.1.1 の「はい → 同モードの `RUN`」が、`REC`（教示）/ `NAV`（Nav2 系）のモードで成り立たない。§6.1 と `C-04` は既に走行状態を引く形だった | `C-04` の `$resume_run`（`attributes.yaml` の `run_state`） | `Spec-modes.md` §3.1.1・§6.1 ／ `Spec-webui.md` §4.1 ／ **F-37** |
| **(j)** | `TEACH_*` の `PAUSE` → `REC` が「走行」ボタン前提だが、**S-13 の操作カードは「停止／保存」だけで「走行」が無い** | `T-TEACH-03M` / `T-TEACH-05M` に分け、`TEACH_MANUAL` は `ui.jog.hold` を契機にした | `Spec-modes.md` §3.1.2 ／ `Spec-webui.md` §3.4 ／ **F-38** |

**(i) と (j) は、(a)〜(h) を正本へ戻す作業の中で同じ表から出てきたもの。**
(a) / (c) / (j) は同根で、**手で触る操作を状態機械の言葉に写すときのずれ**である
（`Spec-open.md` §3.2 末尾）。

---

## 10. 拒否理由キー

`reject_reason_key` は UI の日本語辞書（[DetailedDesign-webui.md](DetailedDesign-webui.md) §7）のキーになる。
**`th_state` は日本語を返さない。**

| キー | 意味 |
| --- | --- |
| `not_allowed` | 遷移表に該当行が無い |
| `mode_entry_denied` | `mode_entry.yaml` が禁じている |
| `tracker_disabled` | `tracker_enabled == false` なのに人物追跡を要するモード |
| `tracker_lost` | ロスト中／信頼度が低い |
| `no_target_selected` | 対象が選ばれていない |
| `working_in_progress` | `working == true` で行き先を選ぼうとした |
| `estop_disabled_in_carry` | `CARRY` 中の UI 非常停止 |
| `estop_engaged` | 非常停止中 |
| `hw_estop_still_pressed` | 物理ボタンが押されたまま再開しようとした |
| `route_not_found` | 指定の経路が無い |
| `no_route_recorded` | 教示済み経路が 1 つも無い |
| `map_update_off` | 地図更新 OFF なのに保存しようとした |
| `device_not_connected` | リードデバイス／カメラ未接続 |
| `link_not_ready` | 必須 3 者が疎通していない |
| `wait_clear_active` | 退避待ち中の手動操作 |
| `calib_not_verified` | 検証を通していない |
| `params_placeholder_blocking` | 起動を止める暫定値が残っている |

---

## 11. テスト要件（`WP-STATE-*` の受け入れ条件）

| # | 検証 | 手段 |
| --- | --- | --- |
| 1 | `transitions.yaml` の全行に `spec_ref` がある | `test_transition_table_traceability` |
| 2 | **`Spec-modes.md` §3.1.1 の 8 行・§3.1.2 の 75 行すべてに、対応する行が 1 つ以上ある** | `spec_ref` の逆引き。**欠けたら失敗** |
| **2r** | **逆方向: 全行の `spec_ref` が指す正本の ID が実在する** | **過剰行の検出。**詳細設計にしか無い遷移は正本へ戻す（`Spec-open.md` §7.1 手順 6） |
| 3 | 全行に対応する pytest がある | **`@pytest.mark.rule("T-FOLLOW-03")` で申告する。**`conftest.py` が収集して `transitions.yaml` の `id` 集合と突き合わせる |
| 4 | 到達不能な状態が 0 | `StateCore.validate()` |
| 5 | `PAUSE` を持たないモードが 0（`INIT`/`ESTOP`/`CARRY` を除く） | 同上 |
| 6 | 表に無い `(mode, state, event)` は必ず `accepted=False` | property test（ランダム 10,000 通り） |
| 7 | `ui.*` を `/system/event` から入れたら拒否される | ノード統合テスト |
| 8 | `evt.*` を `/system/trigger` から入れたら拒否される | 同上 |
| 9 | `jog_lease_ms ≥ twist_mux の manual_joy timeout` | 起動時アサーション＋テスト |
| 10 | フォルト → `PAUSE` → 解消 → `ui.resume_yes` → 元モードの `run_state` | シナリオテスト |

```bash
python3 -m pytest src/th_testing/test/test_state_core.py -v
python3 -m pytest src/th_testing/test/test_transition_table.py -v
```

---

## 12. 運用シーケンス（`Spec-ops.md` の実装）

`Spec-ops.md` は専用の詳細設計ファイルを持たない。**起動と終了は状態機械の両端**なのでここに書く。

### 12.1 起動（`INIT`）

| 順 | 主体 | 実装 |
| --- | --- | --- |
| 1〜3 | 人 | 電源 ON → Wi-Fi 接続 → `start.sh` |
| 4 | 自動 | `docker compose` → launch の `OpaqueFunction` が `generated/` を作る（[params](DetailedDesign-params.md) §5） |
| 5 | 自動 | `connectivity_checker` が**実際にデータが行き来していること**を確かめる（§12.2） |
| 6 | 人 | S-00 で結果を確認 |
| 7 | 自動 | `link_wait_timeout_ms` を超えたら `sys.link_timeout` → `restart_control_stack` |

**必須通信系統がすべて疎通するまで `IDLE` へ進めない**（`T-INIT-01` のみ）。

### 12.2 疎通の合否条件（`Spec-ops.md` §2.2）

**「リンクしている」だけでは足りない。**

| 対象 | 合格条件 | 実装 |
| --- | --- | --- |
| ESP32 | `esp32_alive_timeout_ms` 以内にホイールフィードバックが届き続けている | `/esp32/wheel_feedback` の受信間隔 |
| ESP32 | 速度指令が折り返し確認できる（キープアライブが成立） | `/esp32/wheel_cmd_speed` と `/esp32/wheel_feedback` の対応 |
| RaspberryPi4 | スキャンが規定の周期・規定の点数で届いている | `/scan` の `ranges.size()` と受信間隔 |
| PC | 必要なノードがすべて起動している | `connectivity_checker` が `get_node_names()` を照合 |

**Wi-Fi AP は必須通信系統に数えない**（`C-01`）。ただし S-00 とヘッダに
**「単一障害点」と明示する**。フォールバックは**手押し**（実装ゼロで成立する正式なフォールバック）。

### 12.3 `restart_control_stack`（`CL-B-2` / `E-4`）

| 項目 | 実装 |
| --- | --- |
| 何を再起動するか | **PC 側の ROS2 ノード群のみ。**機体の電源は落とさない |
| どうやるか | `connectivity_checker` が launch のプロセスグループへ SIGTERM → supervisor が再起動 |
| 進捗 | S-00 に「制御系を再起動しています（n 回目）」 |
| 回数・待ち時間 | **`O-d4`。マニュアル側で決める**ので `registry.yaml` に `placeholder` として置く |

### 12.4 起動完了の提示（`CL-B-8`）

**現場で PC を見るとは限らない。**2 つ持つ。

| 手段 | 実装 |
| --- | --- |
| WebUI | S-00 の機器ごとの状態と総合判定 |
| **機体側** | ESP32 の GPIO に LED を足し、`evt.link_ok` を受けて点灯させる。**`WHEEL_CMD` とは別のフレーム**（`LED_STATE 0x05`）で伝える |

> **申し送り**: LED の追加はハードウェア変更を伴う。`WP-ESP32-01` に含めるか別途かは
> [DetailedDesign-open.md](DetailedDesign-open.md) で扱う。

### 12.5 終了（`Spec-ops.md` §4）

**順序が重要**（保存前に電源を切ると作業が消える）。

| 順 | 実装 |
| --- | --- |
| 1 | `ui.finish` で `IDLE` へ（`C-08`） |
| 2 | 未保存の検出（`SystemState.unsaved`。各機能が `evt.unsaved.set` / `.clear` で申告） |
| 3 | S-01「運用の終了」→ `/shutdown/prepare` が未保存一覧を返す → W-4 で項目ごとに保存／破棄 |
| 4 | `/shutdown/execute`。**未保存が残っている間は拒否**（`reject_reason_key: unsaved_remains`） |
| 5 | 停止の進捗と**完了**を S-01 に出す（ここまで確認してから電源を切る） |

**`unsaved` になりうるもの**: 試験場内地図 ／ 教示経路 ／ 校正の補正値 ／ 教示再生中の地図の書き足し。

### 12.6 起動時に物理ボタンが押されたまま（`CL-B-6`）

`T-INIT-03`。`CARRY` へ落とさず `INIT/CHECK` に留まり、案内を出す。
**解除するまで `evt.link_ok` を出さない**（疎通が揃っていても運用に入れない）。

---

## 13. 逆引き

| 完全設計書 | ここでの反映 |
| --- | --- |
| `Spec-modes.md` §1（モードと状態の二層） | §2 |
| §3・§3.1（**状態モデルの正本**） | §4 |
| §4.1（共通規則） / `F-33` | §4.1・§7 |
| §4.2（モード遷移許可表） | `mode_entry.yaml`（§4.1 `C-13`） |
| §5（フォルト時の挙動） / `F-20` / `C-15` | §4.1 `C-03` |
| §5.1（意図的な人検知 OFF） / `CL-X-1` | `tracker_enabled` フラグ |
| §6（モード別属性表） / `F-37` | §8 |
| §6.1（異常解決後の復帰） / `C-13` / `U-8` | §7.1 |
| §7（ゾーン） / `F-19` / `U-1` | §6 ／ [names](DetailedDesign-names.md) §4.1 |
| §8（ジョグ介入） / `U-4` / `F-28` / `F-29` / `F-31` | §5 |
| §9（人物追跡の要否） / `E-9` | §8 `needs_tracker` |
| §10（現行実装との差分） | [reuse](DetailedDesign-reuse.md) §2.2 |
| **`Spec-ops.md` §1〜§5（運用シーケンス）** / `CL-B-2` / `CL-B-3` / `CL-B-6` / `CL-B-8` / `E-4` | **§12** |
| `Spec-open.md` `F-30`〜`F-38` | §9（正本に反映済み） |
