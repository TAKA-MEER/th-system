# 作業パケット — 段階 0（測定・ネットワーク・契約）

[DetailedDesign-packets.md](DetailedDesign-packets.md) §3 の実体。
**`§0` の実装規約 `R1`〜`R8` と §0.1 のテンプレートは packets.md にある。先に読む。**

> **`DD-2`**: 1 パケット＝1 セッションで完結する。
> **各パケットの §2 に挙げた節だけ読めば実装できる**ことがこの文書の要件である。
> §2 に無い仕様が必要になったら、**実装せずに先に設計書へ行を足す**（`R1`）。

**段階 0 の出口**: `pytest` が通る純粋コア（`state_core` / `derive` / `assertions`）と、
実測値の入った `registry.yaml`、そしてビルドできる `th_system_msgs`。
**この段階では ROS2 ノードを 1 つも作らない**（`WP-SAFE-00` を除く）。

**この表は実施順である。**`WP-ESP32-01` が測定 5 件より前にあるのは `O-5`
（物理非常停止が効かないまま実機で走らせない）による
——[packets](DetailedDesign-packets.md) **§1.1** に、例外を作らず段階 2 から移した理由がある。

| 順 | WP | 種別 | 通電 |
| --- | --- | --- | --- |
| 1 | [`WP-MSG-01`](#wp-msg-01-th_system_msgs-の新設追加のみ) | 実装（型定義） | 不要 |
| 2 | [`WP-STATE-01`](#wp-state-01-state_core--transitionsyaml--guardspy) | 実装（純粋コア） | 不要 |
| 3 | [`WP-PARAM-01`](#wp-param-01-registry--derive--assertions--export) | 実装（純粋コア） | 不要 |
| 4 | [`WP-SAFE-00`](#wp-safe-00-リンク品質の計測だけを先に作る) | 実装（ノード） | 不要 |
| 5 | [`WP-NET-01`](#wp-net-01-ネットワークの改善) | 環境整備 | **必要** |
| **6** | **[`WP-ESP32-01`](#wp-esp32-01-バイパス解除--ファーム構成フラグdebt-1)** | **実装（ファーム＋PC）** | **必要** |
| 7 | [`WP-MEAS-01`](#wp-meas-01-制動加速度の実測) | 測定 | **必要** |
| 8 | [`WP-MEAS-02`](#wp-meas-02-人が歩いている部屋で地図を作る) | 測定 | **必要** |
| 9 | [`WP-MEAS-03`](#wp-meas-03-手押し搬送のベースライン) | 測定 | 機体のみ |
| 10 | [`WP-MEAS-04`](#wp-meas-04-リンク品質の実測) | 測定 | 不要 |
| 11 | [`WP-MEAS-05`](#wp-meas-05-1-日の運用を通した完走試験) | 測定（段階 6 の後） | **必要** |

---

## `WP-MSG-01` `th_system_msgs` の新設（**追加のみ**）

### 0. 一行要旨

新しい msg **16 種**・srv 20 種を `th_system_msgs` に**足す**。**旧 msg は 1 つも消さない。**
（旧記述は 15 種だったが、[names](DetailedDesign-names.md) §5.1 の表の実数は 16 種。**表が正**）

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| [names](DetailedDesign-names.md) §5.1 の新 msg 全部 | これらを publish／subscribe するノード |
| 同 §5.2 の新 srv 全部 | 旧 msg・旧 srv の削除（**`O-3`**。`WP-CLEAN-01` の担当） |
| `CMakeLists.txt` / `package.xml` への登録 | 既存ノードの型の差し替え |

**`RobotMode.msg` / `PersonStatus.msg` / `SetMode.srv` などは残したまま。**
`th_mode_manager` / `th_planning` / `web_ui` / 既存テストがまだ使っており、
このパケットで消すと**ワークスペース全体がビルド不能になる。**

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [names](DetailedDesign-names.md) §5.1 | msg のフィールド定義（**この表が正**） |
| [names](DetailedDesign-names.md) §5.2 | srv の要求・応答 |
| [names](DetailedDesign-names.md) §2.1 | **モードは文字列。`uint8` の定数を作らない** |
| [names](DetailedDesign-names.md) §3・§3.1 | 状態名・フラグ名（`SystemState` のフィールド） |
| [reuse](DetailedDesign-reuse.md) §2.1 | 既存 msg のどれを後で消すか（**今回は消さない**） |

### 3. インターフェース契約

#### 3.1 トピック

**このパケットは publish/subscribe を一切しない。**型を定義するだけ。

#### 3.2 サービス

同上。`.srv` ファイルを置くだけ。

#### 3.3 パラメータ

なし。

#### 3.4 フレーム

`Pin.msg` の `geometry_msgs/Pose` は **`map` フレーム**。`PersonTargets.msg` の
`geometry_msgs/Point[]` は **`base_link` フレーム**（現行 `PersonStatus` の流儀を維持）。
**フレームは msg のコメントに書く**（フィールドを増やさない）。

### 4. 内部設計

#### 4.1 純粋コア

なし。

#### 4.2 ノードの責務

なし。

#### 4.3 不変条件

| # | 不変条件 |
| --- | --- |
| M1 | **文字列フィールドに `uint8` の定数を併設しない。**`RobotMode.msg` の失敗を繰り返さない |
| M2 | **`Header header` を持つのは publish されるトピック型だけ。**`Pin.msg` / `RouteInfo.msg` は配列要素なので持たない |
| M3 | 既存の 9 msg・5 srv のファイルは**バイト単位で変更しない**。**例外は `FaultStatus.msg` の 1 件だけ**——[names](DetailedDesign-names.md) §5.1 と §6.3 FMEA ① が `string severity`（`RECOVERABLE` / `CRITICAL`）の**追加**を要求しており、これを落とすと `WP-SAFE-01` が着手不能になる。**フィールドは末尾に足す**（既存の publisher はそのままビルドが通る）。`test_legacy_msgs_untouched` はこの 1 件を対象から外し、代わりに**「`severity` が末尾に 1 行足されただけで、既存フィールドの順序と型が変わっていない」**ことを検査する |

### 5. 表駆動データ

[names](DetailedDesign-names.md) §5.1・§5.2 の表を**そのまま**転写する。列の追加・省略をしない。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | なし |
| 6.2 フェイルセーフ既定 | なし |
| 6.3 FMEA | ① `FaultStatus.severity` を足し忘れる → `WP-SAFE-01` が着手不能。② `LimiterStatus` の `action` の値を減らす → `BLOCKED_UNCALIBRATED` が表現できず `DEBT-2` の対処が入らない。③ 旧 msg を消す → ワークスペース全体がビルド不能 |

### 7. 単体試験

| テスト | 満たす仕様 |
| --- | --- |
| `test_msg_definitions.py::test_all_new_msgs_importable` | §5.1 の全 msg が Python から import できる |
| `test_msg_definitions.py::test_no_uint8_mode_constants` | M1。`SystemState` に `uint8` 定数が無い |
| `test_msg_definitions.py::test_legacy_msgs_untouched` | M3。旧 msg のファイルハッシュが変わっていない |
| `test_msg_definitions.py::test_fields_match_names_md` | §5.1 の表を機械読みして突き合わせる |

### 8. Gazebo シナリオ

なし。

### 9. 実機での確認手順

なし（**電源断でも通電でも不要**）。

### 10. 完了条件

```bash
# ① ビルドが通る
cd /root/th_ws && colcon build --symlink-install --packages-select th_system_msgs

# ② 既存パッケージが壊れていない（旧 msg を消していないことの確認）
colcon build --symlink-install
colcon test --packages-select th_testing --event-handlers console_direct+
colcon test-result --verbose        # 失敗 0

# ③ names.md §5.1・§5.2 の全型が引ける（件数を主張せず、辞書と突き合わせる）
source install/setup.bash
python3 - <<'EOF'
import re, subprocess, sys, io
md = io.open("../../docs/plan/detailed/DetailedDesign-names.md", encoding="utf-8").read()
want  = set(re.findall(r'\*{0,2}`(\w+)\.msg`', md))
want |= set(re.findall(r'\| \*{0,2}`/[\w/]+`\*{0,2} \| `(\w+)`:', md))
have = set(subprocess.run(["ros2","interface","list"], capture_output=True, text=True)
           .stdout.split())
missing = [w for w in sorted(want) if not any(w in h for h in have)]
assert not missing, f"未定義: {missing}"
print("ok", len(want), "型")
EOF

# ④ 単体試験
python3 -m pytest src/th_testing/test/test_msg_definitions.py -v
```

### 11. 既知の負債・未確定 (c)

なし（型定義に数値は現れない）。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | **なし**（最初に着手できる 1 つ） |
| 被依存 WP | `WP-SAFE-00` / `WP-STATE-02` / `WP-SAFE-01` / `WP-PERC-01` ほぼ全部 |

---

## `WP-STATE-01` `state_core` ＋ `transitions.yaml` ＋ `guards.py`

### 0. 一行要旨

**ROS2 に一切触れずに**状態機械を作る。`transitions.yaml` を読んで `Decision` を返す純粋コア。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `th_state/th_state/state_core.py`（`rclpy` を import しない） | `state_manager.py`（ROS2 ノード。`WP-STATE-02`） |
| `th_state/th_state/guards.py`（27 述語） | effect の**実行**（コアは返すだけ） |
| `th_state/config/transitions.yaml`（**128 行**＝共通 18 ＋ モード内 110、`LINE`/`LEASH` 含む） | `/system/state` の publish |
| `th_state/config/attributes.yaml`（18 行 × 9 列） | ラッチ（`prev_*`）の保持。**Context で受け取る** |
| `th_state/config/mode_entry.yaml` | |
| `th_state/package.xml` / `CMakeLists.txt`（`ament_cmake_python`） | |

### 2. 参照する設計書の節

**この 7 節だけ読めばよい。**

| 節 | 何のために |
| --- | --- |
| [state](DetailedDesign-state.md) §2 | `Context` / `Effect` / `Decision` / `StateCore` の**シグネチャ（そのまま実装する）** |
| [state](DetailedDesign-state.md) §3・§3.1・§3.2・§3.3・§3.5 | スキーマ・**マッチ規則の 6 手順**・解決トークン・effect 一覧・`ui.goto` の写像 |
| [state](DetailedDesign-state.md) §4.1・§4.1.1 | **共通行 18** と**ガード 27 件の定義** |
| [state](DetailedDesign-state.md) §4.2 | モード内 110 行（**`transitions.yaml` の中身はこの表**） |
| **[state](DetailedDesign-state.md) §4.4** | **`spec_ref` の実値（全 128 行の写像）。転記するだけでよい** |
| [state](DetailedDesign-state.md) §8.2・§8.3 | `attributes.yaml` の実値表・`mode_entry.yaml` の許可表 |
| [state](DetailedDesign-state.md) §10 | `reject_reason_key` の一覧（**日本語を返さない**） |
| [state](DetailedDesign-state.md) §11 | テスト要件 1〜10（**§10 の完了条件と 1:1**） |
| [names](DetailedDesign-names.md) §2.2・§3・§3.1・§8 | モード 18・状態・フラグ・トリガ名の**集合**（`validate()` が照合する） |

### 3. インターフェース契約

#### 3.1 トピック／3.2 サービス

**なし。**このパケットは ROS2 に接続しない。

#### 3.3 パラメータ

| 名前 | 単位 | class | status | 備考 |
| --- | --- | --- | --- | --- |
| `jog_lease_ms` | ms | b | derived | **コアは読まない。**ノード（`WP-STATE-02`）がリース判定に使う |

**`state_core.py` に数値リテラルを 1 つも書かない**（`R2`）。時間の判定はノード側。
`Context.now_ms` を受け取るが、**コアは差分をとらない**（`sys.*` はノードが生成する）。

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コアの関数シグネチャ

[state](DetailedDesign-state.md) §2 の通り。**追加も省略もしない。**

```python
class StateCore:
    def __init__(self, transitions: list[dict], mode_entry: dict,
                 attributes: dict, guards: "GuardRegistry"): ...
    def validate(self) -> list[str]: ...          # ①〜⑦（§2）
    def step(self, mode: str, state: str, event: str, ctx: Context) -> Decision: ...
    def initial_state(self, mode: str) -> str: ...
    def attributes(self, mode: str) -> dict: ...
```

`guards.py` は**関数の辞書**にする。

```python
GUARDS: dict[str, Callable[[str, str, Context], bool]] = {
    "jog_allowed": lambda mode, state, ctx: ...,
    ...
}
```

**ガードは `(mode, state, ctx)` の 3 引数を取る。**`estop_ui_allowed`（`mode != "CARRY"`）や
`not_checking_estop` のように**遷移元モードを見る**ガードがあるため、`ctx` だけでは足りない。

#### 4.2 ノードの責務

このパケットには無い。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **S-1** | `step()` は副作用を持たない。**`Decision` を返すだけ** | effect の実行順をノードが決められるようにする |
| **S-2** | `Decision.to_mode` / `to_state` に `$` トークンが残らない | 解決漏れを呼び出し側が検出できない |
| **S-3** | `accepted=False` のとき `reject_reason_key` が必ず非空 | UI が「何も言わずに拒否」する状態を作らない |
| **S-4** | ガードは `Context` のフィールドと `(mode, state)` **だけ**を読む。時刻・ファイル・環境変数を読まない | `validate()` ② が検査できる形を保つ |
| **S-5** | `validate()` が非空を返したら、呼び出し側は**起動を止める** | [state](DetailedDesign-state.md) §2 |

### 5. 表駆動データ

**[state](DetailedDesign-state.md) §4.1 と §4.2 の表を、加工せずに `transitions.yaml` へ落とす。**

```yaml
- id: C-06a
  layer: 0
  mode: "*"
  state: "*"
  event: fault.critical
  guard: null
  to_mode: ESTOP
  to_state: NONE
  effects:
    - {name: latch_prev, args: {}}
  spec_ref: SM-3.1.1-06
```

| 注意 | 内容 |
| --- | --- |
| §4.3 から行を起こさない | **索引であって定義ではない**（`id` が重複する） |
| `to_mode` 省略の扱い | §4.2 の表で `to_mode` 列が無いモードは **`"="` を明示的に書く**（YAML では省略しない） |
| `override_common` を持つ行 | `T-INIT-03` / `T-MANUAL-01` / `T-MANUAL-02` / `T-TEACH-*`（手動側）/ `T-ATP-04` / `T-OPC-05` / `T-OPC-08` / `T-CAL-08` |
| `reject: true` を持つ行 | `C-06r` のみ |

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 4**（状態機械）。層 1〜3 には触れない |
| 6.2 フェイルセーフ既定 | どの行にも当たらなければ **`accepted=False`**（`not_allowed`）。**既定で通すことはしない** |
| 6.3 FMEA | ① `override_common` のガード評価を省く → `T-OPC-05` が `C-07` を隠し、**MOTOR 点検中に物理 E-Stop が効かなくなる**（§3.1 手順 3）。② `latch_prev` を `ESTOP`/`CARRY` でも記録する → 復帰先が壊れ、`ui.carry_resume` で `ESTOP` に戻る。③ `C-06r` を落とす → `CARRY` 中の UI 非常停止が `prev_*` を `CARRY` に上書きする（`F-26`） |

### 7. 単体試験

**[state](DetailedDesign-state.md) §11 の 1〜10 がそのまま受け入れ条件。**

| テスト | 満たす仕様行 |
| --- | --- |
| `test_transition_table.py::test_all_rows_have_spec_ref` | §11-1 |
| `test_transition_table.py::test_spec_rows_covered` | §11-2（正本 §3.1.1 の 8 行・§3.1.2 の 75 行） |
| `test_transition_table.py::test_no_extra_rows` | §11-2r（**逆方向**。詳細にしか無い遷移の検出） |
| `test_transition_table.py::test_every_rule_has_a_test` | §11-3（`@pytest.mark.rule` の収集） |
| `test_state_core.py::test_validate_returns_empty` | §11-4・5（到達不能・`PAUSE` 欠落） |
| `test_state_core.py::test_unknown_triple_is_rejected` | §11-6（property test 10,000 通り） |
| `test_state_core.py::test_override_evaluates_guard` | §3.1 手順 3（**FMEA ①**） |
| `test_state_core.py::test_latch_skipped_in_estop_carry` | §7 のラッチ表（**FMEA ②**） |
| `test_state_core.py::test_estop_in_carry_rejected` | `C-06r`（**FMEA ③**） |
| `test_state_core.py::test_estop_is_not_a_trap` | `C-09f`。`fault.critical` → `ESTOP` → `ui.resume_ack` → `IDLE` |
| `test_state_core.py::test_goto_kind_mapping` | §3.5（`PANEL` というモードは無い） |
| `test_state_core.py::test_manual_run_self_loop` | `T-MANUAL-01`（`RUN` 発の自己ループ） |

**`conftest.py` の `_repo_root` を直す。**現行はリポジトリルートではなく `th_ws/src` を指しており、
`docs/plan/spec/Spec-modes.md` に届かない（§11-2 が実行不能）。
`--symlink-install` 前提で `Path(__file__).resolve().parents[N]` を数え直し、
**`docs/plan/spec/` の存在を assert する**（CI では環境変数 `TH_REPO_ROOT` を優先）。

### 8. Gazebo シナリオ

なし（ROS2 に接続しない）。

### 9. 実機での確認手順

なし。

### 10. 完了条件

```bash
# コンテナ内 (/root/th_ws)。V1・V8
cd /root/th_ws
colcon build --symlink-install --packages-select th_state && source install/setup.bash

# ① 純粋コアが ROS2 なしで動く（最重要）
python3 -c "import sys; sys.path.insert(0,'src/th_state'); import th_state.state_core"
test -d src/th_state/th_state && ! grep -rq "import rclpy\|from rclpy" src/th_state/th_state/   # V2

# ② 数値リテラルが無い（R2）
! grep -rnE "^[^#]*[^_a-zA-Z0-9]([0-9]+\.[0-9]+|[0-9]{3,})" src/th_state/th_state/state_core.py

# ③ 全テスト
python3 -m pytest src/th_testing/test/test_state_core.py -v
python3 -m pytest src/th_testing/test/test_transition_table.py -v
#   テスト 2（欠落）・2r（過剰）・2f（宣言外の分割）の 3 本が通ること

# ④ validate() が空を返す（起動可能であることの証明）
PYTHONPATH=src/th_state python3 -m th_state.validate_cli --config src/th_state/config   # exit 0・出力なし

# ⑤ 行数を機械的に数える（主張ではなく実測）
python3 -c "import yaml;d=yaml.safe_load(open('src/th_state/config/transitions.yaml'));\
print(len(d), len({r['id'] for r in d}))"    # 128 128（重複 0）
```

### 11. 既知の負債・未確定 (c)

| 項目 | 扱い |
| --- | --- |
| ~~正本 `Spec-modes.md` への ID 列追加~~（[open](DetailedDesign-open.md) §3 A-1） | **完了。**`spec_ref` の実値は [state](DetailedDesign-state.md) §4.4 にある。**推測しない・転記する** |
| `LINE` / `LEASH` の行 | **表には入れる**（実装は段階 8）。`validate()` は通る |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | **なし**（`WP-MSG-01` とも独立。並行して着手できる） |
| 前提条件 | **なし**（正本への ID 列追加は 2026-08-16 に完了） |
| 被依存 WP | `WP-STATE-02` / `WP-SAFE-04`（`attributes.yaml` を共有） |

---

## `WP-PARAM-01` registry ＋ derive ＋ assertions ＋ export

### 0. 一行要旨

**数値が存在する唯一の場所**を作る。導出・検査・生成はすべて純粋関数で、ROS2 に触れない。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `th_params/config/registry.yaml`（全パラメータ） | `params_audit.py`（ノード。`WP-PARAM-02`） |
| `th_params/th_params/schema.py`（S1〜S5） | launch への組み込み（同上） |
| `th_params/th_params/derive.py`（§3 の全式） | `docker-compose.yml` の bind mount（同上） |
| `th_params/th_params/assertions.py`（A1〜A11） | 実測値そのもの（`WP-MEAS-01` / `-04`） |
| `th_params/th_params/export.py` ＋ CLI | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [params](DetailedDesign-params.md) §1.1・§1.2・§1.3・§1.4 | `registry.yaml` のスキーマ・`status` の意味・**S1〜S5**・`value_by` |
| [params](DetailedDesign-params.md) §2・§2.1 | sentinel を使わない・`TBD_MEASURE` は 1 種類 |
| [params](DetailedDesign-params.md) §3.1〜§3.4 | **導出式の全部**（そのまま実装する） |
| [params](DetailedDesign-params.md) §4 | **A1〜A11**（違反時の挙動まで） |
| [params](DetailedDesign-params.md) §5・§5.1 | 生成と監査の分離・`params_digest` |
| [params](DetailedDesign-params.md) §6.1〜§6.4 | registry に入れる行の索引 |
| [params](DetailedDesign-params.md) §8 | CI 検査 |
| [names](DetailedDesign-names.md) §7.1〜§7.5 | **パラメータ名の正**（`registry.yaml` の `name` はここと 1:1） |
| [safety](DetailedDesign-safety.md) §3.1 | `floor_distance()` が制動距離を使わない理由（A2a/A2b） |
| [safety](DetailedDesign-safety.md) §7 | タイムアウト 2 本制約（§3.3 の実装根拠） |

### 3. インターフェース契約

#### 3.1 トピック／3.2 サービス

**なし。**`export.py` は CLI として呼ばれる。

```bash
python3 -m th_params.export --registry <path> --out <dir> --stage <n> [--sim] [--nodes a,b,c]
```

| 引数 | 意味 |
| --- | --- |
| `--stage` | A8 の `blocking_from_stage` 判定。**既定は最大値**（全部止める） |
| `--sim` | `allow_placeholder: true`（測定できない環境） |
| `--nodes` | 今回の launch で起動するノード。**A8 の `consumers` 絞り込みに使う** |

**終了コード**: 0＝成功／**1＝アサーション違反（launch を止める）**／2＝スキーマ違反。

#### 3.3 パラメータ

**このパケットが全パラメータの定義元である。**§10 の完了条件で名前の網羅を検査する。

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コアの関数シグネチャ

[params](DetailedDesign-params.md) §3 の 9 関数をそのまま。

```python
def braking_distance(v, a, t_delay) -> float
def speed_from_braking_distance(d_allow, a, t_delay) -> float
def v_max_from_ceiling(ceiling_mps, headroom_ratio) -> float
def floor_distance(body_half_length_m, floor_margin_m) -> float      # ★ 速度非依存
def person_backstop_ms(grace_ms, link_p99_ms, factor) -> float
def clear_distance(body_half_length_m, clear_margin_m) -> float
def hysteresis_band(stop_distance_m, ratio) -> float
def combined_heading_error(two_point_deg, nav_tolerance_deg) -> tuple[float, float]
def two_point_angle_error_deg(spacing_m, sigma_m) -> float
def deviation_budget_m(corridor_width_m, body_width_m, margin_m) -> float
def timeout_lower_bound_ms(p99_gap_ms, margin_ratio) -> float
def timeout_upper_bound_ms(v_max, intrusion_budget_m) -> float
```

#### 4.2 ノードの責務

なし。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **P-1** | `derive.py` / `assertions.py` / `schema.py` は**ファイルも環境変数も読まない** | pytest で全組合せを流せる形を保つ |
| **P-2** | `TBD_MEASURE` は `registry.yaml` にしか現れない | §2.1。grep 1 発で未測定が全部出る |
| **P-3** | `status: derived` の行は `value` を持たない | 手打ちの値が生き残る経路を作らない |
| **P-4** | **`hysteresis_band_m` は `obstacle_floor_distance_m` から導く** | `obstacle_stop_distance_m` から導くと帯が `d_behavior` を超えうる（A11） |
| **P-5** | §3.3 の `v_max` 引き下げは**1 回だけ**。不動点反復にしない | 起動時間が読めなくなる |

### 5. 表駆動データ

`registry.yaml` の初期内容は [params](DetailedDesign-params.md) §6.1〜§6.4 と
[names](DetailedDesign-names.md) §7 の**全行**。**数値を書けるのは `class: b`(given) と `given` の行だけ**。

**`class: c` の行はすべて `value: TBD_MEASURE` / `blocking: true` / `blocking_from_stage` 付き**（S1）。

| パラメータ | `blocking_from_stage` |
| --- | --- |
| `brake_accel_mps2` | **0**（すべての導出の根） |
| `link_gap_p99_ms`（ESP32/LiDAR/UI） | **1** |
| `tracker_lost_grace_ms` / `person_position_sigma_m` | 4 |
| `replay_drift_m_per_100m` | 5 |
| `calib_*_tolerance` / `calib_interval_days` | 7 |
| `battery_endurance_min` / `leash_stop_latency_ms` | 8 |

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **なし（ただし全層の閾値を決める）** |
| 6.2 フェイルセーフ既定 | 暫定値は**安全側の極値**（P1 fail-slow）。「現在たまたま入っている 0.45 m」を引き継がない |
| 6.3 FMEA | ① A2a/A2b を落とす → **リミッタが追従対象の脚で発火し、追従が構造的に成立しない**。② A7 を落とす → `cmd_vel_stale_ms > WATCHDOG_MS` になり、PC 側より ESP32 が先に止まる（`DEBT-4` の対処が無効化される）。③ `blocking_from_stage` を落とす → 段階 1・2 が実機で**起動できない** |

### 7. 単体試験

| テスト | 満たす仕様 |
| --- | --- |
| `test_params_schema.py::test_s1..test_s5` | S1〜S5 を 1 本ずつ |
| `test_params_derive.py::test_braking_roundtrip` | `speed_from_braking_distance(braking_distance(v)) == v` |
| `test_params_derive.py::test_floor_is_speed_independent` | `floor_distance` が `v` を引数に取らない |
| `test_params_assertions.py::test_a1..test_a11` | **A1〜A11 を 1 本ずつ**（違反ケースと合格ケースの両方） |
| `test_params_assertions.py::test_a8_respects_consumers` | 起動しないノードのパラメータで止まらない |
| `test_params_export.py::test_twist_mux_generated` | `twist_mux.yaml` が生成対象に入っている |
| `test_params_export.py::test_digest_stable` | 同じ registry から同じ `params_digest` |
| `test_params_registry.py::test_names_match_names_md` | `registry.yaml` の `name` 集合 == [names](DetailedDesign-names.md) §7 |
| `test_params_registry.py::test_consumers_nonempty` | A9（死んだパラメータの検出） |

### 8. Gazebo シナリオ

なし。

### 9. 実機での確認手順

なし（値を入れるのは `WP-MEAS-01` / `-04`）。

### 10. 完了条件

```bash
# コンテナ内 (/root/th_ws)。V1・V8
cd /root/th_ws
colcon build --symlink-install --packages-select th_params && source install/setup.bash

# ① ROS2 に依存しない
test -d src/th_params/th_params && ! grep -rq "import rclpy\|from rclpy" src/th_params/th_params/

# ② 未測定の全件が 1 コマンドで出る（P-2）
grep -c TBD_MEASURE src/th_params/config/registry.yaml   # 1 件以上（未測定がある）
#   registry 以外に無い（V2）。ただし sentinel を比較する定数の定義 1 行だけは除く
#   —— コードから参照する以上どこかに 1 度は書く必要がある。文字列を分割して
#   grep を避ける書き方は禁止（定義箇所が grep で見つからなくなる）
test $(grep -rl TBD_MEASURE --include='*.py' --include='*.cpp' src/ | wc -l) -le 1
test "$(grep -rl TBD_MEASURE --include='*.py' --include='*.cpp' src/)" = "src/th_params/th_params/schema.py"
test $(grep -c TBD_MEASURE src/th_params/th_params/schema.py) -eq 1

# ③ 全テスト
python3 -m pytest src/th_testing/test/test_params_*.py -v

# ④ export が動く（sim 相当。placeholder を許す）
PYTHONPATH=src/th_params python3 -m th_params.export \
  --registry src/th_params/config/registry.yaml --out /tmp/gen --stage 0 --sim
test $(ls /tmp/gen/*.yaml | wc -l) -ge 1        # V9

# ⑤ 段階 0 で blocking な placeholder が残っていれば exit 1 になる（brake 未測定時）
PYTHONPATH=src/th_params python3 -m th_params.export \
  --registry src/th_params/config/registry.yaml --out /tmp/gen --stage 0 \
  --nodes obstacle_limiter; test $? -eq 1
```

### 11. 既知の負債・未確定 (c)

**このパケットの成果物は「(c) が空のまま動く仕組み」であって、値ではない。**
`brake_accel_mps2` は `WP-MEAS-01` が入れるまで `TBD_MEASURE` のまま。
**それでも `--sim` で単体試験は全部通る**ことが完了条件（⑤ が exit 1 になるのは正しい振る舞い）。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | **なし**（`WP-MEAS-01` の結果は後から `registry.yaml` に入れる。実装は先行できる） |
| 被依存 WP | `WP-PARAM-02` / `WP-SAFE-01` / `WP-SAFE-03` / `WP-CALIB-01`・以降ほぼ全部 |

---

## `WP-SAFE-00` リンク品質の計測だけを先に作る

### 0. 一行要旨

`safety_monitor` に**受信間隔の p50/p99/max を publish する機能だけ**を足す。
**判定には使わない**（`WP-MEAS-04` の測定器を先に用意する）。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `/safety/link_quality` の publish（ESP32 / LiDAR / UI の 3 本） | フォルトの 2 階級（`WP-SAFE-01`） |
| 受信間隔のリングバッファと分位点計算 | タイムアウトの導出値化（同上） |
| `safety_monitor.cpp` への最小の追加 | 監視対象の `enabled_targets` 化（同上） |

**このパケットは `WP-SAFE-01` から意図的に切り出した。**
`WP-SAFE-01` は `WP-MEAS-04`（実測 p99）に依存し、`WP-MEAS-04` は測定器に依存するので、
**測定器だけを先に出さないと循環する。**

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §7 | なぜ p99 が要るのか（タイムアウトの 2 本制約） |
| [names](DetailedDesign-names.md) §5.1 `LinkQuality.msg` | フィールド定義 |
| [names](DetailedDesign-names.md) §6.2 | QoS（best_effort・1 Hz） |
| [reuse](DetailedDesign-reuse.md) §2.3 | `safety_monitor.cpp` の既存構造（**壊さない**） |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS | レート |
| --- | --- | --- | --- | --- |
| sub | `/esp32/wheel_feedback` | `th_system_msgs/WheelFeedback` | reliable, depth 1 | 既存 |
| sub | `/scan` | `sensor_msgs/LaserScan` | SensorDataQoS | 約 10 Hz |
| sub | `/ui/active_screen` | `th_system_msgs/ActiveScreen` | reliable, depth 5 | 2 Hz |
| **pub** | **`/safety/link_quality`** | `th_system_msgs/LinkQuality` | **best_effort, depth 1** | **1 Hz × 3 本**（`link` フィールドで区別） |

**`/ui/active_screen` の publisher はまだ存在しない**（`WP-UI-01`）。
**受信 0 でも落ちないこと**（`p50=p99=max=0`, `window_sec=0` で publish する）。

#### 3.2 サービス

なし。

#### 3.3 パラメータ

| 名前 | 単位 | class | status | registry |
| --- | --- | --- | --- | --- |
| `link_quality_window_sec` | s | b | given | 新規行。**分位点を取る窓** |

**`registry.yaml` にまだ `params_audit` が無い段階なので、
`safety_monitor.yaml` に直書きせず `registry.yaml` に行を足したうえで、
生成が入るまでは既存の `safety_monitor.yaml` 経由で読む**（`WP-PARAM-02` で自動生成に切り替わる）。

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コア

```cpp
// link_quality_core.hpp — ROS2 非依存
struct Quantiles { double p50_ms, p99_ms, max_ms; uint32_t window_sec; };
class GapTracker {
 public:
  void push(double stamp_sec);          // 受信のたび
  Quantiles compute(double now_sec, double window_sec) const;
};
```

**分位点は近似しない。**窓内の全ギャップを保持してソートする（1 Hz × 3 本なら十分安い）。

#### 4.2 ノードの責務

既存 `safety_monitor.cpp` に `GapTracker` を 3 つ持ち、1 Hz のタイマで publish するだけ。
**既存のフォルト判定ロジックには一切触れない。**

#### 4.3 不変条件

| # | 不変条件 |
| --- | --- |
| **Q-1** | **この機能は何も止めない。**`/safety/link_quality` は診断用で、フォルト判定に一切入らない |
| **Q-2** | 受信 0 でも publish は続く（沈黙しない） |
| **Q-3** | 既存 `safety_monitor` の挙動が変わらない（既存テストが無変更で通る） |

### 5. 表駆動データ

なし。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 3 のノードに触るが、層 3 の判定には触れない** |
| 6.2 フェイルセーフ既定 | 受信 0 → ゼロ値を publish（Q-2） |
| 6.3 FMEA | ① `GapTracker` が例外を投げる → **`safety_monitor` が落ちて層 3 が消える。**必ず `try`/`catch` で囲み、失敗しても publish をスキップするだけにする。② 既存判定を巻き込んで壊す → Q-3 のテストで検出。③ 1 Hz のタイマがメインループを圧迫 → 別コールバックグループに置く |

### 7. 単体試験

| テスト | ctest 登録名（`V6`） | 満たす仕様 |
| --- | --- | --- |
| `test_link_quality_core`（gtest） | `test_link_quality_core`（`th_safety`） | 既知の系列で p50/p99/max が一致 |
| `test_link_quality_core::EmptyWindow` | 同上（同一ターゲット内のケース） | Q-2。受信 0 でゼロ値 |
| `test_safety_monitor.py`（**既存・無変更**） | `safety_monitor`（`th_testing`・**既存**） | Q-3 |

**`th_safety/CMakeLists.txt` にはテスト登録が現在 1 件も無い**（`DEBT-10`）。
**このパケットが最初の 1 件を登録する。**登録しないと `colcon test --packages-select th_safety` は
**テスト 0 件で exit 0** ＝ §10 ② が無条件に合格する（`V5` / `V6`）。

```cmake
# src/th_safety/CMakeLists.txt — if(BUILD_TESTING) の中
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_link_quality_core test/test_link_quality_core.cpp)
target_link_libraries(test_link_quality_core link_quality_core)
```

### 8. Gazebo シナリオ

`gazebo.launch.py`（既定）。`/safety/link_quality` が 3 本出ることの確認。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部**（受信間隔の測定に走行は不要） | なし |

```bash
ros2 topic echo /safety/link_quality --once
timeout 6 ros2 topic hz /safety/link_quality   # 3.0 Hz 前後（3 本 × 1 Hz）。V4
```

### 10. 完了条件

```bash
cd /root/th_ws && colcon build --symlink-install --packages-select th_safety

# ① 既存テストが無変更で通る（Q-3）
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R "safety_monitor|fault_detection"
colcon test-result --verbose

# ② gtest。DEBT-10 の解除。「0 件で合格」を潰すため実行件数を数える（V5・V6・V9）
colcon test --packages-select th_safety --event-handlers console_direct+ \
  --ctest-args -R test_link_quality_core
colcon test-result --verbose                    # V5。colcon test は落ちても exit 0
grep -c "ament_add_gtest\|ament_add_pytest_test" src/th_safety/CMakeLists.txt   # 1 以上
test "$(ctest --test-dir build/th_safety -N | tail -1 | grep -oE '[0-9]+$')" -ge 1

# ③ 実機（電源断）で 3 本出る。V4・V10
timeout 5 ros2 topic echo /safety/link_quality --field link --csv > /tmp/lq.txt
test "$(sort -u /tmp/lq.txt | tr -d '\r' | paste -sd, -)" = "esp32,lidar,ui\"
```

### 11. 既知の負債・未確定 (c)

`link_gap_p99_ms` は**このパケットでは埋まらない**（測定は `WP-MEAS-04`）。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-MSG-01`（`LinkQuality.msg`） |
| 被依存 WP | **`WP-MEAS-04`**（測定器） → `WP-SAFE-01` |

---

## `WP-NET-01` ネットワークの改善

> **申し送り（2026-08-18）**: このパケットの作業のうち **3 件は `WP-ESP32-01` で先に済ませた**（ファームを触る作業が同じ日に集中するため）——`wifi_credentials.h.example` のポート是正（8765 → 8766）、`ws_link.h` のフレーム表の是正、`registry.yaml` への `esp32_ws_port` の登録。**残るのは AP・チャンネル・帯域・配置の見直しだけ**（手順書 B の 3 案比較）。

### 0. 一行要旨

`docs/plan/task.md` 1。受信ギャップ 0.5〜1.2 s を減らす。**`DEBT-6`（秘匿情報）も同時に片づける。**

### 1. 対象と非対象

| やる | やらない |
| --- | --- |
| AP・チャンネル・帯域・配置の見直し | ROS2 側のタイムアウト値の変更（`WP-SAFE-01`） |
| ~~`esp32/src/wifi_credentials.h` を `.gitignore` へ~~ **→ 実機確認の結果、既に解消済み**（untracked・履歴にも無い・`.example` はダミー値）。**残る作業は `.example` の値を実態に合わせること**（下記） | 有線化・別無線方式への変更（範囲外） |
| ポートの記述の是正。**`config.h` にポート定義は無い**（2026-08-18 実機確認）。実値は `wifi_credentials.h` の `WS_SERVER_PORT`（機体ごと・`.gitignore` 対象）で、`params.yaml` の `ws_port: 8766` と**既に一致**している。**古いのは `.example` の 8765** | |
| `esp32/src/ws_link.h` のフレーム表を実装に合わせる（`WHEEL_FEEDBACK` は 13 byte） | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §7 | **なぜタイムアウトを縮めるのが誤った処方なのか** |
| [safety](DetailedDesign-safety.md) §11.3・§11.4 | `DEBT-3` の解除手順・併せて片づける 4 件 |
| [safety](DetailedDesign-safety.md) §8.3 | Wi-Fi AP は単一障害点（**改善しても必須系統には数えない**） |
| [hardware](DetailedDesign-hardware.md) **§1**（機器 → ノード → トピック） | **どのリンクを改善するのか**——ESP32 は Wi-Fi、LiDAR は USB で、**同じ「通信」ではない。**AP を直しても USB 側のギャップは消えない |
| [hardware](DetailedDesign-hardware.md) **§3**（ESP32 のフレーム。**このファイルが正**）・**§3.1**（拡張と新設の規約） | **`ws_link.h` のフレーム表をどこに合わせるか**（`DEBT-7`）。表の正本は `ws_link.h` ではなく hardware §3 である |
| `docs/plan/task.md` 1 | 元の要求 |

### 3. インターフェース契約

**プロトコルを変えない。**`ws_protocol.py` と ESP32 のフレーム定義は byte 互換のまま
（[reuse](DetailedDesign-reuse.md) §1）。変えるのは**表の記述と接続先の設定だけ**。

| 項目 | 現行 | 変更後 |
| --- | --- | --- |
| ポート | `wifi_credentials.h`（実値・8766）／`params.yaml`（8766）／**`.example`（8765・古い）** | **`.example` を 8766 に揃える。**`registry.yaml` に `esp32_ws_port` を `given` で新規登録する（**`WP-PARAM-01` 時点で未登録だった**） |
| 認証情報 | **対処済み。**`.gitignore` 済み・untracked・履歴にも無い | 確認のみ（下記 §10-①） |

### 4. 内部設計

実装ではなく環境整備。**ソフトウェアの変更は上表の 2 点と表の記述だけ。**

### 5. 表駆動データ

なし。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 2・3 の前提**（受信ギャップがタイムアウトの下限を決める） |
| 6.2 フェイルセーフ既定 | 変更なし |
| 6.3 FMEA | ① ポートを片方だけ直す → **ESP32 に一切繋がらない。**`WP-MEAS-04` の直前に必ず疎通確認。② 認証情報を履歴から消す作業でリポジトリを壊す → **履歴の書き換えはユーザーの判断を仰ぐ**（`.gitignore` ＋無効値への置換だけなら安全）。③ チャンネル変更で他機器に影響 |

### 7. 単体試験

なし（`test_esp32_ws_protocol.py` が**無変更で通る**ことを確認する）。

### 8. Gazebo シナリオ

なし。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| 疎通・受信間隔の測定・ポート確認 | **なし**（走行は不要） |

### 10. 完了条件

```bash
# ホスト（リポジトリルート・Git Bash）。V1

# ① 認証情報がリポジトリに無い
! git ls-files | grep -q "wifi_credentials\.h$"
git ls-files | grep -q "wifi_credentials\.h\.example$"
#    .example が「実物のコピー」になっていないこと（V2・V3。現行は #define 形式）
test -f th_ws/esp32/src/wifi_credentials.h.example
! grep -qE '#define +WIFI_(SSID|PASS[A-Z]*) +"(?!your-)' th_ws/esp32/src/wifi_credentials.h.example \
  2>/dev/null || grep -qE '#define +WIFI_SSID +"your-' th_ws/esp32/src/wifi_credentials.h.example

# ② ポートが揃っている（値そのものを比較する。目視しない）
#    実値の wifi_credentials.h は .gitignore 対象なので検査できない。
#    検査できるのは .example と params.yaml と registry.yaml の 3 つ。
EX=$(grep -oE 'WS_SERVER_PORT +[0-9]+' th_ws/esp32/src/wifi_credentials.h.example | grep -oE '[0-9]+')
PC=$(grep -oE 'ws_port: *[0-9]+' th_ws/src/th_esp32_bridge/config/params.yaml | grep -oE '[0-9]+')
RG=$(python3 -c "import yaml;print({p['name']:p for p in yaml.safe_load(open('th_ws/src/th_params/config/registry.yaml'))}['esp32_ws_port']['value'])")
test -n "$EX" && test "$EX" = "$PC" && test "$EX" = "$RG"

# ③ フレーム表が実装と一致（V3。期待文字列そのものを書く）
grep -q "WHEEL_FEEDBACK.*13 bytes" th_ws/esp32/src/ws_link.h
grep -q "ESTOP_HW.*3 bytes"        th_ws/esp32/src/ws_link.h

# ④ プロトコルテストが無変更で通る（コンテナ内）
docker compose run --rm th_robot bash -lc \
  'cd /root/th_ws && python3 -m pytest src/th_testing/test/test_esp32_ws_protocol.py -v'

# ⑤ 改善の効果は WP-MEAS-04 で測る（このパケット単独では判定しない）
```

### 11. 既知の負債・未確定

**「どこまで改善できるか」は事前に決められない。**目標値を置かず、
`WP-MEAS-04` の実測を `registry.yaml` に入れて、その値から `v_max` が決まる（[safety](DetailedDesign-safety.md) §7 ②）。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | なし |
| 被依存 WP | **`WP-MEAS-04`** |

---

## `WP-ESP32-01` バイパス解除 ＋ ファーム構成フラグ（`DEBT-1`）

### 0. 一行要旨

`ESTOP_BENCH_TEST_BYPASS` を無効にし、**バイパスが有効なことをランタイムで検出できるようにする。**

### 1. 対象と非対象

| やる | やらない |
| --- | --- |
| `esp32/src/config.h` の `ESTOP_BENCH_TEST_BYPASS` を無効化 | 始業点検 項目 1 の実装（`WP-MAINT-01`。段階 7） |
| `ESTOP_HW (0x03)` フレームに `bypass_active` ビットを載せる | `DEBT-1` の**正式な解除**（同上） |
| `esp32_bridge` が `bypass_active` を `/safety/firmware_flags` へ出す | 機体側 LED（`LED_STATE 0x05`。申し送り） |
| `main.cpp:196-202` の `[DBG]` printf の削除 | プロトコルの他フレームの変更 |
| `esp32/src/ws_link.h` のフレーム表の是正（`WHEEL_FEEDBACK` は 13 byte） | `LED_STATE (0x05)` / `BATTERY (0x06)` の**実装**（表への追記だけ行う。§11） |
| **部品の確認**（状態 LED ＋ 抵抗・分圧抵抗・空きピン。[hardware](DetailedDesign-hardware.md) §3.4・§5） | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §0 `DEBT-1` | 何が壊れているか（`config.h:130` / `main.cpp:119-121`） |
| [safety](DetailedDesign-safety.md) §11.1 | **解除手順の 4 段**（合格判定は始業点検 項目 1） |
| [safety](DetailedDesign-safety.md) §11.4 | 併せて片づける 4 件 |
| [safety](DetailedDesign-safety.md) §1（層 1・2） | 物理ボタンとウォッチドッグの位置づけ |
| [safety](DetailedDesign-safety.md) §12 | **開発モードでも層 1・2 は無効化できない** |
| [reuse](DetailedDesign-reuse.md) §2.13 | ファームの既存構造（**プロトコルは byte 互換のまま**） |
| [reuse](DetailedDesign-reuse.md) §2.6 | `esp32_bridge` の「ファーム世代検出」の流儀（`_feedback_has_dt`） |
| **[hardware](DetailedDesign-hardware.md) §3・§3.1・§3.4** | **ESP32 フレーム表の正・「旧形式を受理する」規約・GPIO 割り当て** |
| **[hardware](DetailedDesign-hardware.md) §5** | **手配（LED と分圧抵抗はここで初めて必要になる）** |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS | レート |
| --- | --- | --- | --- | --- |
| **pub**（`esp32_bridge`） | **`/safety/firmware_flags`** | `std_msgs/UInt8` | **transient_local, depth 1** | 変化時＋接続時 |
| pub（既存） | `/safety/estop_hw` | `std_msgs/Bool` | reliable | 10 Hz |

**bit 0 = `bypass_active`。**残りは予約（0）。
**新しい msg 型を作らない**（`WP-MSG-01` は締めてある。`UInt8` で足りる）。

#### 3.2 プロトコル

```
ESTOP_HW (0x03):  [type:1][pressed:1][flags:1]      ← flags を 1 byte 追加
```

**旧形式（2 byte）も受理する。**`esp32_bridge` は長さで判別し、
2 byte なら `flags = 0xFF`（**「不明」＝バイパスの可能性あり ＝ 安全側**）として扱う。
既存の `_feedback_has_dt` と同じ流儀。

#### 3.3 パラメータ

| 名前 | class | status | 用途 |
| --- | --- | --- | --- |
| `esp32_watchdog_ms` | given | given | `config.h` の `WATCHDOG_MS` の写し。A6/A7 が突き合わせる |
| `esp32_ws_port` | given | given | `WP-NET-01` で統一済み |

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コア

`th_esp32_bridge/th_esp32_bridge/ws_protocol.py` に
`unpack_estop_hw(payload) -> tuple[bool, int]` を足す（**既存関数の戻り値を変えない**）。

#### 4.2 ノードの責務

`esp32_bridge` は受け取った `flags` を `/safety/firmware_flags` へ流すだけ。
**判定は `safety_monitor` が行う**（`WP-SAFE-01`）。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **E-1** | **旧形式のフレームは「不明」として安全側に倒す** | ファーム更新前の機体で無言に素通ししない |
| **E-2** | `ESTOP_BENCH_TEST_BYPASS` の定義そのものを**残す**（無効化するだけ） | 台上試験の手段は必要。**検出できることが要件**であって禁止ではない |
| **E-3** | 開発モードのフラグを ESP32 に渡さない | 層 1・2 は無効化できない（§12） |

### 5. 表駆動データ

なし。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 1（物理非常停止）そのもの** |
| 6.2 フェイルセーフ既定 | `flags` 不明 → バイパスの可能性ありとして扱う（E-1） |
| 6.3 FMEA | ① バイパスを無効化したが `estopActive` の反映漏れがある → **押しても止まらない。**通電での実測が必須。② `flags` を足したことで旧 `esp32_bridge` が例外を投げる → **`esp32_bridge` を先に更新してからファームを焼く**（順序）。③ `[DBG]` printf を消したことでシリアルのタイミングが変わり、既存の不具合が再現／消失する → 削除前後で `WHEEL_FEEDBACK` の受信間隔を比較する |

### 7. 単体試験

| テスト | 満たす仕様 |
| --- | --- |
| `test_esp32_ws_protocol.py::test_unpack_estop_hw_3byte` | §3.2 新形式 |
| `test_esp32_ws_protocol.py::test_unpack_estop_hw_2byte_unknown` | E-1 |
| `test_esp32_ws_protocol.py`（**既存の全ケース**） | 後方互換（無変更で通ること） |

### 8. Gazebo シナリオ

**なし**（Gazebo に `esp32_bridge` は起動しない）。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| `flags` が届くこと・`/safety/firmware_flags` が出ること | **押して実際に駆動が切れること**（テスタでモータードライバ電源を確認） |

### 10. 完了条件

```bash
# ① バイパスが無効
grep -n "ESTOP_BENCH_TEST_BYPASS" th_ws/esp32/src/config.h      # 定義はあるが 0 / #undef
! grep -n "estopActive *= *false" th_ws/esp32/src/main.cpp       # 無条件代入が消えている

# ② [DBG] printf が消えている
! grep -n "\[DBG\]" th_ws/esp32/src/main.cpp

# ③ フレーム表が実装と一致（V3。現行は "(9 bytes)" "(2 bytes)" なので期待文字列で当てる）
grep -q "WHEEL_FEEDBACK.*13 bytes" th_ws/esp32/src/ws_link.h
grep -q "ESTOP_HW.*3 bytes"        th_ws/esp32/src/ws_link.h
grep -q "LED_STATE.*0x05"          th_ws/esp32/src/ws_link.h
grep -q "BATTERY.*0x06"            th_ws/esp32/src/ws_link.h

# ④ プロトコルテスト
python3 -m pytest th_ws/src/th_testing/test/test_esp32_ws_protocol.py -v

# ⑤ 実機（電源断）。V4。目視せず値を比較する
test "$(timeout 3 ros2 topic echo /safety/firmware_flags --field data --once)" = "0"
#   ここで物理ボタンを押す（人が関与）
test "$(timeout 5 ros2 topic echo /safety/estop_hw --field data --once)" = "True"
#   離すと戻る（ラッチしないこと。押しっぱなし検出ではない）
test "$(timeout 5 ros2 topic echo /safety/estop_hw --field data --once)" = "False"

# ⑥ 通電（人が関与）— 押下でモータードライバ電源が落ちる
```

### 11. 既知の負債・未確定 (c)

**`DEBT-1` はこのパケットでは解除されない。**
解除条件は「始業点検 項目 1 が OK」であり、それを実行できるのは `WP-MAINT-01`（段階 7）。
**ここで行うのは「検出できるようにすること」まで**（[safety](DetailedDesign-safety.md) §11.1 の 4）。

| 項目 | 扱い |
| --- | --- |
| **`LED_STATE (0x05)` / `BATTERY (0x06)` の実装** | **このパケットでは表に足すだけ。**GPIO の空きピン選定と部品が要る（[hardware](DetailedDesign-hardware.md) §3.4・§5）。**来なくてもフォルトにしない**設計なので、後から足せる |
| 分圧比 | バッテリーの公称電圧が要る。**`battery_warn_v` / `battery_critical_v` は `given`**（方針値） |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-NET-01`（ポート統一）が**望ましい**。**`WP-MSG-01` には依存しない**（`std_msgs/UInt8` しか使わない） |
| 被依存 WP | **`WP-MEAS-01` / `WP-MEAS-02`**（段階 0 で実機を走らせる 2 件。`O-5`）／ **`WP-SAFE-01`**（バイパス検出を重大フォルトに）／ 実機で走らせる全パケット（`O-5`） |

**このパケットが段階 2 ではなく段階 0 にある理由**は
[packets](DetailedDesign-packets.md) **§1.1**。`O-5` に例外を作らないため移した。
**ファームを焼く作業なので、機体が手元にある日にまとめて済ませる**
（`WP-NET-01` の `wifi_credentials.h` 差し替えと同じ日にやるのが早い）。

---

## `WP-MEAS-01` 制動加速度の実測

### 0. 一行要旨

**実装ではなく測定。**`brake_accel_mps2` を `status: measured` にする。
**`Spec-params.md` §8 の「先に潰す 4 つ」の 1 番。**

### 1. 対象と非対象

| 測る | 測らない |
| --- | --- |
| `brake_accel_mps2`（**最も悪い値**） | `brake_delay_s`（(b)。指令周期から導出） |
| 前進・後退・場内速度域の 3 条件 × 5 回 | 積載時・傾斜での値（**申し送り**） |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [params](DetailedDesign-params.md) §3.1・§3.1.1 | この値から何が導かれるか（**ほぼ全部の距離と速度**） |
| [params](DetailedDesign-params.md) §1.2 | `measured_at` / `source` が必須 |
| [safety](DetailedDesign-safety.md) §7.1 | **ファームのランプを変えたら測り直す** |
| [packets](DetailedDesign-packets.md) §3.1 | 手順 1〜5 |
| [packets](DetailedDesign-packets.md) §12.1 | 通電が要る作業の扱い |

### 3. インターフェース契約

なし（測定）。記録先は `registry.yaml` の 1 行。

```yaml
- name: brake_accel_mps2
  unit: m/s2
  class: c
  status: measured
  value: <実測の最悪値>
  measured_at: "<YYYY-MM-DD>"
  source: "odom 積分 + 巻尺。前進/後退/場内 各 5 回。TARGET_RAMP_ACCEL_MPS2=<値>"
  consumers: [obstacle_limiter, follow_runner, replay_runner, venue_navigator, params_audit]
  spec_ref: "Spec-params.md §8 / Spec-safety.md §2.2"
```

### 4〜5. 内部設計・表駆動データ

なし。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 1・2 が生きていることが測定の前提。**`DEBT-1`（バイパス）が有効な状態で走らせない |
| 6.2 フェイルセーフ既定 | 測定は**人が張り付いた状態で**行う。周囲 3 m に人・物を置かない |
| 6.3 FMEA | ① 平均を採る → **制動距離が短く見積もられ、以降の全距離が危険側にずれる。**最も悪い値を採る。② ランプ設定を記録し忘れる → 後でファームを触ったとき再測定の要否が判断できない。③ odom 積分だけで済ませる → ホイールスリップを拾えない。**巻尺と両方**記録する |

### 7〜8. 単体試験・Gazebo

なし（**シミュレーションの値を使ってはいけない**）。

### 9. 実機での確認手順

**通電が必要。**`ImplementationPlan.md` の「実機での動作確認のみ人が関与」に該当。

| 順 | 内容 |
| --- | --- |
| 1 | `DEBT-1` が有効なままでないことを確認（物理ボタンを押して止まること） |
| 2 | 一定速度で走らせ、`/cmd_vel` をゼロにしてから停止するまでの距離を測る |
| 3 | `/odometry/filtered` の積分と巻尺の**両方**を記録 |
| 4 | 前進・後退・場内速度域の 3 条件 × 5 回 |
| 5 | **最も悪い値**を採る |

### 10. 完了条件

```bash
# ホスト（リポジトリルート）。① ② はホスト、③ はコンテナ内。V1

# ① registry の該当行が measured になっている
python3 - <<'EOF'
import yaml
r = {p["name"]: p for p in yaml.safe_load(open("th_ws/src/th_params/config/registry.yaml"))}
p = r["brake_accel_mps2"]
assert p["status"] == "measured", p["status"]
assert "measured_at" in p and "source" in p        # S4
assert p["value"] != "TBD_MEASURE"
# §3 の雛形の穴埋めが残っていないこと（<...> はすべて実値に置き換わっている）
import re
for k in ("value", "measured_at", "source"):
    assert not re.search(r"<[^>]*>", str(p[k])), (k, p[k])
print("ok", p["value"], p["measured_at"])
EOF

# ② 15 回分の生データが残っている（V9。数える）
test $(ls docs/plan/detailed/data/brake_accel_*.csv | wc -l) -eq 15

# ③ この値に依存する導出が全部埋まる（コンテナ内。V8）
docker compose run --rm th_robot bash -lc 'cd /root/th_ws && \
  PYTHONPATH=src/th_params python3 -m th_params.export \
  --registry src/th_params/config/registry.yaml --out /tmp/gen --stage 0 \
  --nodes obstacle_limiter'      # exit 0
```

### 11. 既知の負債

**積載時・傾斜での再測定**は未計画（[open](DetailedDesign-open.md) へ申し送り）。
`Spec-safety.md` §3 は「重心が高くなることが予想される」と書いており、
**積載形態が決まったら測り直しが要る。**

### 12. 依存

| | |
| --- | --- |
| 依存 WP | **`WP-ESP32-01`（`DEBT-1` 解除）が必須**（`O-5`。§6.1 が「バイパスが有効な状態で走らせない」と書いている）。人が張り付くことでは代替しない |
| 被依存 WP | `WP-PARAM-01` の値埋め → `WP-SAFE-03` / 段階 1 以降の実機起動すべて |

---

## `WP-MEAS-02` 人が歩いている部屋で地図を作る

### 0. 一行要旨

**人物マスク実装の要否を決める**（`O-c2`）。**最も安いテスト。**

### 1. 対象と非対象

| やる | やらない |
| --- | --- |
| 人が普通に歩いている室内で SLAM を回し、地図への写り込みを記録 | 人物マスクの実装（要否が決まってから） |
| 写り込んだ場合の消去コスト（手作業で何分か）の記録 | `PREP` の地図修正機能（`WP-ONSITE-04`） |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [onsite](DetailedDesign-onsite.md) §3.1（`PREP` の手順と状態）・**§3.5（地図の修正）** | 写り込みを消す機能の位置づけ |
| [packets](DetailedDesign-packets.md) §9 | `WP-ONSITE-04` の範囲がこの結果で変わる |

### 3〜5. 契約・内部設計・表

なし。

### 6. 安全要件

歩いている人の周囲で機体を動かす。**手動走行のみ**（自律走行は使わない）。
**`WP-ESP32-01` が済んでいること**——人が歩いている中で走らせるので、
物理非常停止が効かない状態では行わない（`O-5` ／ [packets](DetailedDesign-packets.md) §1.1）。

### 7〜8. 単体試験・Gazebo

なし。

### 9. 実機での確認手順

**通電が必要**（機体を動かして地図を作る）。

### 10. 完了条件

```bash
# ① 地図と記録が残っている
ls docs/plan/detailed/data/meas02/*.pgm docs/plan/detailed/data/meas02/*.yaml
test -f docs/plan/detailed/data/meas02/report.md

# ② 判定が明文で書かれている（どちらかが report.md の 1 行目にある）
grep -qE "^判定: (人物マスクが要る|人物マスクは不要)" docs/plan/detailed/data/meas02/report.md
```

**「要る」なら `WP-ONSITE-04` に人物マスクを足す。「不要」なら地図修正の手作業だけで済ませる。**
どちらでも `WP-ONSITE-04` は成立するので、この測定が段階 6 を止めることはない。

### 11〜12. 負債・依存

| | |
| --- | --- |
| 依存 WP | **`WP-ESP32-01`**（`O-5`。人が歩いている中で走らせる） |
| 被依存 WP | `WP-ONSITE-04`（**範囲が変わる**） |

---

## `WP-MEAS-03` 手押し搬送のベースライン

### 0. 一行要旨

`O-a7`。**目標 G4 の比較対象**を作る。片道 200 m の所要時間・人数・負担。

### 1. 対象と非対象

| 測る | 測らない |
| --- | --- |
| 現在の手押し搬送の所要時間・人数・主観的負担 | ロボットを使った場合の値（段階 3 以降） |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| `Spec.md` §2（目標 G4） | 何と比べるのか |
| [packets](DetailedDesign-packets.md) §6 | 段階 3 の出口で比較が始まる |

### 3〜8

なし。

### 9. 実機での確認手順

**機体（未通電・手押し）と実際の経路が要る。**ROS2 は不要。

### 10. 完了条件

```bash
# ホスト（リポジトリルート）。V1・V9
test -f docs/plan/detailed/data/meas03/baseline.md
#   所要時間 / 人数 / 主観負担（5 段階）を 1 行 1 回で記録し、3 回以上あること
test $(grep -cE '^\| *[0-9]+ *\|' docs/plan/detailed/data/meas03/baseline.md) -ge 3
```

### 11〜12. 負債・依存

| | |
| --- | --- |
| 依存 WP | なし |
| 被依存 WP | 段階 3 の出口（KPI 比較） |

---

## `WP-MEAS-04` リンク品質の実測

### 0. 一行要旨

`link_gap_p99_ms` を ESP32 / LiDAR / UI 別に `measured` にする。
**この値がタイムアウトを決め、タイムアウトが `v_max` を決める。**

### 1. 対象と非対象

| 測る | 測らない |
| --- | --- |
| `/safety/link_quality` の p50/p99/max を 3 本とも、**運用相当の時間**（1 時間以上） | タイムアウト値そのもの（`derive.py` が計算する） |
| `WP-NET-01` の改善**前後**（効果の確認） | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §7 | **2 本制約。②を満たせないなら `v_max` を下げる** |
| [params](DetailedDesign-params.md) §3.3 | `timeout_lower_bound_ms` / `timeout_upper_bound_ms` |
| [params](DetailedDesign-params.md) §1.2 | `measured_at` / `source` が必須 |

### 3. インターフェース契約

`registry.yaml` に 3 行（`link_gap_p99_ms` は `value_by: [esp32, lidar, ui]` の表）。

### 4〜8

なし。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部** | なし |

**ただし「運用相当」であること。**LiDAR を回し、UI を開き、`/cmd_vel` を 20 Hz で流した状態で測る。
**静止した空きチャンネルの値を採ってはいけない**（実運用より良い値が出て、タイムアウトが短すぎになる）。

### 10. 完了条件

```bash
# ① 3 本とも measured
python3 - <<'EOF'
import yaml
r = {p["name"]: p for p in yaml.safe_load(open("th_ws/src/th_params/config/registry.yaml"))}
p = r["link_gap_p99_ms"]
assert p["status"] == "measured" and set(p["value"]) == {"esp32","lidar","ui"}
print(p["value"])
EOF

# ② 改善前後の記録がある
ls docs/plan/detailed/data/meas04/before.csv docs/plan/detailed/data/meas04/after.csv

# ③ タイムアウトが導出でき、A1 が通る（コンテナ内。V8）
docker compose run --rm th_robot bash -lc 'cd /root/th_ws && \
  PYTHONPATH=src/th_params python3 -m th_params.export \
  --registry src/th_params/config/registry.yaml --out /tmp/gen --stage 1 \
  --nodes safety_monitor && grep -q "lidar_timeout_ms" /tmp/gen/safety_monitor.yaml'
```

### 11. 既知の負債

**A1 で `v_max` がクランプされる可能性が高い。**それは正しい振る舞いであって不具合ではない
（[safety](DetailedDesign-safety.md) §7）。クランプされたことは `/system/params_status` に載せる。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | **`WP-SAFE-00`**（測定器）／ `WP-NET-01`（改善後の値） |
| 被依存 WP | **`WP-SAFE-01`**（タイムアウトの導出値化） |

---

## `WP-MEAS-05` 1 日の運用を通した完走試験

### 0. 一行要旨

目標 G5。**再起動なし・バッテリー交換なしで 1 日を完走する。**`n=1` で足りる。

### 1. 対象と非対象

| 測る | 測らない |
| --- | --- |
| 往復＋前日準備＋当日試験を通しで 1 回 | 統計的な信頼性（`Spec-ops.md` §5 が `n=1` で足りるとしている） |
| `battery_endurance_min` | |

**§2.4 の自動再起動（`restart_control_stack`）が 1 度でも走ったら未達。**

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| `Spec-ops.md` §5 | **`n=1` で足りる根拠** |
| [safety](DetailedDesign-safety.md) §7.2 | バッテリーは警告のみ・止めない |
| [state](DetailedDesign-state.md) §12.3 | `restart_control_stack` の定義（**これが走ったら未達**） |

### 3〜8

なし。

### 9. 実機での確認手順

**通電・1 日・現場。**段階 6 が完了していること。

### 10. 完了条件

```bash
# コンテナ内 (/root/th_ws)。V1

# ① restart_control_stack が 0 回（V2。grep -c は 0 件で exit 1 を返す）
LOGDIR=$(ls -d /root/th_data/logs/*/ | tail -1)
test "$(grep -c 'restart_control_stack' "$LOGDIR/state_manager.log" || true)" -eq 0

# ② バッテリー交換 0 回・充電 0 回が記録されている
test -f docs/plan/detailed/data/meas05/run.md

# ③ registry
python3 -c "import yaml;r={p['name']:p for p in yaml.safe_load(open('th_ws/src/th_params/config/registry.yaml'))};\
p=r['battery_endurance_min'];assert p['status']=='measured';print(p['value'])"
```

### 11〜12. 負債・依存

| | |
| --- | --- |
| 依存 WP | **段階 6 の完了**（`WP-UI-07` まで） |
| 被依存 WP | なし（目標の検証） |

---

## 段階 0 の出口チェック

```bash
cd /root/th_ws
colcon build --symlink-install
python3 -m pytest src/th_testing/test/test_state_core.py \
                  src/th_testing/test/test_transition_table.py \
                  src/th_testing/test/test_params_schema.py \
                  src/th_testing/test/test_params_derive.py \
                  src/th_testing/test/test_params_assertions.py \
                  src/th_testing/test/test_msg_definitions.py -v
grep -c TBD_MEASURE src/th_params/config/registry.yaml    # brake_accel と link_gap は消えている

# 物理非常停止が効く状態で段階 1 へ渡す（O-5。実機・電源断で確認）
test "$(timeout 3 ros2 topic echo /safety/firmware_flags --field data --once)" = "0"
test -f esp32/src/main.cpp && \
  ! grep -n "estopActive *= *false" esp32/src/main.cpp    # V2。無条件代入が無い

# th_safety にテストが 1 件以上登録されている（DEBT-10。V5・V6）
grep -c "ament_add_gtest\|ament_add_pytest_test" src/th_safety/CMakeLists.txt
```

| 出口条件 | 判定 |
| --- | --- |
| 純粋コアが `rclpy` なしで動く | `grep` で 0 件 |
| **`DEBT-1`（バイパス）が実機で無効**——測定 2 件より前 | `/safety/firmware_flags` が `0`（`WP-ESP32-01` §10） |
| **`DEBT-10`（`th_safety` のテスト 0 件）が解除** | `CMakeLists.txt` に登録 1 件以上（`WP-SAFE-00` §10-②） |
| `brake_accel_mps2` が `measured` | `registry.yaml` |
| `link_gap_p99_ms` が 3 本とも `measured` | 同上 |
| 遷移表 **128 行**・`id` 重複 0・`validate()` が空 | `validate_cli` |
| 旧 msg が 1 つも消えていない | `colcon test` 全通過 |
