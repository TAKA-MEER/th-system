# 作業パケット — 段階 1（状態機械と UI 骨格）

[DetailedDesign-packets.md](DetailedDesign-packets.md) §4 の実体。
**§0 の実装規約 `R1`〜`R8` と §0.1 のテンプレートは packets.md にある。先に読む。**

**段階 1 の出口**: 実機（**モータ電源断**）で `INIT → IDLE → MANUAL` が確認でき、
**異常ウィンドウを開いた状態で非常停止ボタンが押せる**こと。

> **この段階ではまだ機体は動かない。**`/cmd_vel` の publisher は
> 既存の twist_mux のままで、`obstacle_limiter` も `jog_gate` も無い（段階 2）。
> **手動走行を試そうとしないこと**（`WP-TRANSIT-01` は段階 3）。

| WP | 種別 | 通電 |
| --- | --- | --- |
| [`WP-STATE-02`](#wp-state-02-state_manager-ノード) | 実装（ノード） | 不要 |
| [`WP-STATE-03`](#wp-state-03-connectivity_checker) | 実装（ノード） | 不要 |
| [`WP-PARAM-02`](#wp-param-02-params_audit--launch-の-opaquefunction) | 実装（ノード＋launch） | 不要 |
| [`WP-UI-01`](#wp-ui-01-シェルヘッダ操作カードウィンドウi18n) | 実装（WebUI） | 不要 |
| [`WP-UI-02`](#wp-ui-02-s-00-接続確認--s-01-メインメニュー) | 実装（WebUI） | 不要 |
| [`WP-CARRY-01`](#wp-carry-01-手押しモードcarry一式) | 実装（横断） | **最終確認のみ必要** |

---

## `WP-STATE-02` `state_manager` ノード

### 0. 一行要旨

`state_core` を ROS2 に繋ぐ**薄いノード**。`/system/state` を出し、`/system/trigger` を受ける。
**遷移の判断はコアがする。ノードはラッチと effect の配送とタイマだけを持つ。**

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `th_state/scripts/state_manager.py` | 遷移の判断（`WP-STATE-01` のコア） |
| `prev_mode` / `prev_state` / `prev_sub` のラッチ | effect の**中身**（各ノードが実装。今はほぼ宛先が居ない） |
| `sys.jog_lease_expired` / `sys.link_timeout` の生成 | `connectivity_checker`（`WP-STATE-03`） |
| `/safety/fault` `/safety/estop_hw` `/safety/estop_ui` の購読 → `fault.*` / `hw.*` / `ui.estop.*` の内部生成 | `safety_monitor` の改修（`WP-SAFE-01`） |
| `/ui/active_screen` → `derive_limits()` → `zone` | `jog_gate`（`WP-SAFE-04`） |
| `/ui/jog_lease` の購読 → `ui.jog.hold` | |
| `/system/set_flag` | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [state](DetailedDesign-state.md) §2 | コアの呼び方（`Context` を詰めて `step()` を呼ぶ） |
| [state](DetailedDesign-state.md) §3.3 | **effect の宛先表**（どのノードへサービスで送るか。UI 向けだけ `/system/state` に載せる） |
| [state](DetailedDesign-state.md) §5・§5.1・§5.2 | ジョグの 3 層。**リース時刻の更新は遷移の採否と独立** |
| [state](DetailedDesign-state.md) §7・§7.1 | **ラッチの 6 通り**・フォルト解消は遷移ではない |
| [state](DetailedDesign-state.md) §8.1・§8.2 | `attributes.yaml` の読み方 |
| [state](DetailedDesign-state.md) §10 | `reject_reason_key`（**日本語を返さない**） |
| [state](DetailedDesign-state.md) §11-7・§11-8 | **名前空間の分離は安全境界**（`ui.*` を `/system/event` から入れたら拒否） |
| [names](DetailedDesign-names.md) §4.1 | **`derive_limits()` が唯一の正**（ゾーンと速度上限） |
| [names](DetailedDesign-names.md) §5.1・§5.2 | `SystemState` / `StateEvent` / `ActiveScreen` / `UiTrigger` / `SetFlag` |
| [names](DetailedDesign-names.md) §6.2 | QoS（`/system/state` は transient_local・10 Hz ＋変化時即時） |
| [names](DetailedDesign-names.md) §8・§8.4 | トリガの入口 3 本・リース |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS | レート |
| --- | --- | --- | --- | --- |
| **pub** | `/system/state` | `SystemState` | **transient_local, depth 1, reliable** | **10 Hz ＋ 変化時即時** |
| sub | `/system/event` | `StateEvent` | reliable, depth 10 | 事象時 |
| sub | `/ui/active_screen` | `ActiveScreen` | reliable, depth 5 | 2 Hz × 端末数 |
| sub | `/ui/jog_lease` | `std_msgs/String` | **best_effort, depth 1** | 5 Hz 以上 |
| sub | `/safety/fault` | `FaultStatus` | reliable, depth 5 | 変化時 |
| sub | `/safety/estop_hw` | `std_msgs/Bool` | reliable | 10 Hz |
| sub | `/safety/estop_ui` | `std_msgs/Bool` | reliable | 2 Hz ＋変化時 |

**`/system/state` を 10 Hz で出し続ける（沈黙しない）。**購読側（`obstacle_limiter` / `jog_gate`）は
`state_stale_ms` で途絶を検出して安全側に倒すので、**沈黙は「最も低い上限」を意味する。**

#### 3.2 サービス

| サービス | 型 | 拒否理由キー |
| --- | --- | --- |
| `/system/trigger` | `UiTrigger` | [state](DetailedDesign-state.md) §10 の全キー |
| `/system/set_flag` | `SetFlag` | `not_allowed` / `tracker_disabled` / `working_in_progress` |
| `/shutdown/prepare` | `std_srvs/Trigger` | — （**未保存一覧を `message` に JSON で返す**） |
| `/shutdown/execute` | `std_srvs/Trigger` | `unsaved_remains` |

**`/system/trigger` は `ui.*` しか受け付けない。**`evt.*` / `fault.*` / `hw.*` / `sys.*` が来たら
`accepted=false`, `reject_reason_key="not_allowed"`。**逆も同じ**（`/system/event` は `evt.*` のみ）。

#### 3.3 パラメータ

| 名前 | 単位 | class | status | 用途 |
| --- | --- | --- | --- | --- |
| `jog_lease_ms` | ms | b | derived | `sys.jog_lease_expired` の契機 |
| `link_wait_timeout_ms` | ms | b | derived | `sys.link_timeout` の契機 |
| `ui_active_window_s` | s | b | derived | `derive_limits()` の判定窓 |
| `screen_stale_ms` | ms | b | derived | `/ui/active_screen` の途絶判定 |

**裸の数値を書かない**（`R2`）。`generated/state_manager.yaml` から読む。

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コアの関数シグネチャ

**このパケットは新しい純粋関数を作らない。**`state_core.StateCore` と
[names](DetailedDesign-names.md) §4.1 の `derive_limits()` を呼ぶだけ。

**`derive_limits()` の置き場所**: `th_state/th_state/zones.py`（純粋・`rclpy` 非依存）。
**`jog_gate` と `obstacle_limiter` は `/system/state.zone` を読むだけ**なので、この実装は 1 つでよい。

#### 4.2 ノードの責務

**「コアを呼ぶだけ」。**具体的には次の 6 つ以外を持たない。

| # | 責務 |
| --- | --- |
| 1 | 入力を `Context` に詰める |
| 2 | `step()` を呼び、`Decision` を受ける |
| 3 | `effects` を §3.3 の宛先表に従って配送する（`th_state` 自身の分はここで実行） |
| 4 | `prev_mode` / `prev_state` / `prev_sub` のラッチを保持する（`latch_prev` / `clear_prev` を見て） |
| 5 | `sys.*` の 2 事象をタイマで生成する |
| 6 | `/system/state` を publish する |

```python
def on_trigger(self, req, res):
    if not req.trigger.startswith("ui."):
        res.accepted, res.reject_reason_key = False, "not_allowed"; return res
    if req.trigger == "ui.jog.hold":
        self.last_jog_ms = self.now_ms()          # ★ 遷移の採否と独立に更新（§5）
    d = self.core.step(self.mode, self.state, req.trigger, self.build_context(req))
    self.apply(d)
    ...
```

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **N-1** | **`state_manager.py` に遷移の `if` を 1 つも書かない** | 書いた瞬間 `DD-3` が破れ、表と実装が乖離する |
| **N-2** | リース時刻は `accepted=false` でも更新する | 拒否された瞬間からリースが延びず、`jog_lease_ms` 後に必ず停止する（§5） |
| **N-3** | `latch_prev` は `mode ∈ {ESTOP, CARRY}` では記録しない | 復帰先が壊れる |
| **N-4** | `fault.cleared` で遷移しない | W-1 の文言が変わるだけ（§7.1） |
| **N-5** | `/system/state` は起動直後から出る（`INIT/CHECK`） | transient_local なので後から繋いだ端末にも最新が届く |

### 5. 表駆動データ

`WP-STATE-01` の `transitions.yaml` / `attributes.yaml` / `mode_entry.yaml` を**そのまま読む**。
**このパケットで表を編集しない。**

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 4 のみ。**層 1〜3 には入らない（`th_state` は `/safety/*` を**購読するだけ**） |
| 6.2 フェイルセーフ既定 | `/ui/active_screen` 途絶・使用中 0 台 → **ゾーン `NA`・速度上限 `stop`・`auto_brake` ON**（[names](DetailedDesign-names.md) §4.1） |
| 6.3 FMEA | ① `state_manager` が固まる → `/system/state` が途絶 → **`obstacle_limiter` と `jog_gate` が安全側に倒れる**（設計どおり。駆動は止まる）。② ラッチを取り違える → `ui.carry_resume` で走行モードに戻れず運用が止まる（**危険ではないが運用不能**）。③ `ui.*` の入口を分けない → 挙動ノードのバグが UI 操作として通り、**FSM が任意の状態に飛ばされる** |

### 7. 単体試験

| テスト | 満たす仕様 |
| --- | --- |
| `test_state_manager_node.py::test_ui_event_namespace_rejected` | §11-7 |
| `test_state_manager_node.py::test_evt_via_trigger_rejected` | §11-8 |
| `test_state_manager_node.py::test_lease_extends_on_reject` | N-2 |
| `test_state_manager_node.py::test_state_published_at_10hz` | §3.1 |
| `test_state_manager_node.py::test_transient_local_late_joiner` | N-5 |
| `test_state_manager_node.py::test_latch_roundtrip` | N-3・§7 の 6 通り |
| `test_state_manager_node.py::test_zone_from_active_screen` | `derive_limits()` の 3 ケース |
| `test_state_manager_node.py::test_no_transition_if_in_node` | **N-1。`state_manager.py` の AST を検査し、モード名の文字列比較が無いこと** |

### 8. Gazebo シナリオ

`gazebo.launch.py`（既定）。`/system/state` が出て、`ros2 service call /system/trigger` で
`IDLE → MANUAL` が動くこと。**走行は確認しない**（`jog_gate` がまだ無い）。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部**（モード遷移・拒否理由・ラッチ） | なし |

```bash
ros2 topic echo /system/state --once
ros2 service call /system/trigger th_system_msgs/srv/UiTrigger \
  "{trigger: 'ui.enter_mode', arg_json: '{\"mode\":\"MANUAL\"}', requester: 'cli'}"
```

### 10. 完了条件

```bash
cd /root/th_ws && colcon build --symlink-install --packages-select th_state

# ① 遷移の if がノードに無い（N-1）
python3 - <<'EOF'
import ast, sys
src = open("src/th_state/scripts/state_manager.py").read()
MODES = {"INIT","IDLE","ESTOP","CARRY","FOLLOW","MANUAL","TEACH_FOLLOW","TEACH_MANUAL",
         "REPLAY","LINE","LEASH","PREP","PANEL_NAV","AT_PANEL","SUMMON","HOME_NAV",
         "OPCHECK","CALIB"}
bad = [n.value for n in ast.walk(ast.parse(src))
       if isinstance(n, ast.Constant) and n.value in MODES]
assert not bad, f"モード名がノードに直書きされている: {bad}"
print("ok")
EOF

# ② 名前空間の分離
python3 -m pytest src/th_testing/test/test_state_manager_node.py -v

# ③ 統合
colcon test --packages-select th_testing --event-handlers console_direct+
colcon test-result --verbose

# ④ 実機（電源断）
ros2 topic hz /system/state          # 10.0 前後
ros2 service call /system/trigger th_system_msgs/srv/UiTrigger \
  "{trigger: 'evt.arrived', arg_json: '', requester: 'cli'}"    # accepted: false
```

### 11. 既知の負債・未確定 (c)

**effect の宛先ノードがまだほとんど存在しない。**
`start_record` / `load_route` / `begin_two_point` などは段階 3 以降。
**このパケットでは「宛先が居なければログに残して捨てる」**（例外を投げない）。
宛先の実装が入るたびに配送先を有効化していく（`O-7` と同じ考え方）。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-STATE-01`（コア）／ `WP-MSG-01`（型） |
| 被依存 WP | `WP-STATE-03` / `WP-UI-01` / `WP-SAFE-04` / `WP-CARRY-01` ほか多数 |

---

## `WP-STATE-03` `connectivity_checker`

### 0. 一行要旨

**「リンクしている」ではなく「実際にデータが行き来している」**ことを確かめて `evt.link_ok` を出す。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `th_state/scripts/connectivity_checker.py` | S-00 の画面（`WP-UI-02`） |
| 4 項目の合否判定（[state](DetailedDesign-state.md) §12.2） | 機体側 LED（`LED_STATE 0x05`。[state](DetailedDesign-state.md) §12.4 の申し送り） |
| `evt.link_ok` の発行 | フォルト判定（`safety_monitor` の担当） |
| `restart_control_stack` の実行 | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [state](DetailedDesign-state.md) §12.1 | 起動の 7 手順 |
| [state](DetailedDesign-state.md) §12.2 | **疎通の合否条件 4 項目（この表が正）** |
| [state](DetailedDesign-state.md) §12.3 | `restart_control_stack` の実装（**PC 側の ROS2 ノード群のみ**） |
| [state](DetailedDesign-state.md) §12.6 | 起動時に物理ボタンが押されたまま（`T-INIT-03`） |
| [safety](DetailedDesign-safety.md) §8.3 | **Wi-Fi AP は必須通信系統に数えない** |
| [names](DetailedDesign-names.md) §7.3 | `esp32_alive_timeout_ms` / `link_wait_timeout_ms` |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | 用途 |
| --- | --- | --- | --- |
| sub | `/esp32/wheel_feedback` | `WheelFeedback` | 受信間隔（§12.2 行 1） |
| sub | `/esp32/wheel_cmd_speed` | `WheelFeedback` | 折り返し確認（同 行 2） |
| sub | `/scan` | `sensor_msgs/LaserScan` | 周期と `ranges.size()`（同 行 3） |
| sub | `/safety/estop_hw` | `std_msgs/Bool` | §12.6。**押されている間は `evt.link_ok` を出さない** |
| **pub** | `/system/event` | `StateEvent` | `evt.link_ok` |

#### 3.2 サービス

なし。ノード一覧の照合は `get_node_names()`（rclpy の API）で行う。

#### 3.3 パラメータ

| 名前 | 単位 | class | status | 用途 |
| --- | --- | --- | --- | --- |
| `esp32_alive_timeout_ms` | ms | b | derived | 起動時の疎通判定**専用**（運用中は `esp32_timeout_ms`） |
| `link_wait_timeout_ms` | ms | b | derived | 超えたら `sys.link_timeout`（**発行するのは `th_state`**） |
| `scan_expected_points` | — | given | given | §12.2 行 3。`ranges.size()` の期待値 |
| `required_nodes` | string[] | given | given | §12.2 行 4。**launch 引数で段階ごとに変える** |
| `restart_max_count` / `restart_wait_ms` | — / ms | c | **placeholder** | `O-d4`。**マニュアル側で決める**（`blocking_from_stage: 7`） |

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コア

```python
# th_state/th_state/connectivity_core.py
@dataclass(frozen=True)
class LinkReport:
    esp32_feedback: bool; esp32_loopback: bool; lidar: bool; nodes: bool
    missing_nodes: tuple[str, ...]
    def all_ok(self) -> bool: ...

def evaluate(now_ms, last_fb_ms, last_cmd_ms, last_scan_ms, scan_points,
             present_nodes, p) -> LinkReport: ...
```

#### 4.2 ノードの責務

1 Hz で `evaluate()` を呼び、`all_ok()` かつ **物理 E-Stop が押されていない**とき `evt.link_ok` を 1 回出す。
`link_wait_timeout_ms` の管理は `th_state` 側（`sys.link_timeout` は `th_state` の内部タイマ）。
**`connectivity_checker` は `restart_control_stack` の effect を受けて実行するだけ。**

#### 4.3 不変条件

| # | 不変条件 |
| --- | --- |
| **L-1** | **Wi-Fi AP を判定項目に入れない**（`C-01`） |
| **L-2** | 物理 E-Stop 押下中は `evt.link_ok` を出さない（`CL-B-6`） |
| **L-3** | `evt.link_ok` は**立ち上がりで 1 回だけ**。10 Hz で撃たない |
| **L-4** | `restart_control_stack` は**機体の電源に触れない**。PC 側のプロセスグループへ SIGTERM |

### 5. 表駆動データ

なし。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | なし（層 4 の入口を開けるかどうかだけ） |
| 6.2 フェイルセーフ既定 | 判定できない項目は**不合格**として扱う（未受信＝不合格） |
| 6.3 FMEA | ① `evt.link_ok` を誤って出す → **疎通していないのに運用に入れる。**`Spec-ops.md` §2.2「リンクしているだけでは足りない」の趣旨が消える。② `restart_control_stack` が無限ループ → `restart_max_count` で打ち切り（値は `O-d4`）。③ SIGTERM の対象を誤って機体側まで落とす → L-4 |

### 7. 単体試験

| テスト | 満たす仕様 |
| --- | --- |
| `test_connectivity_core.py::test_four_items` | §12.2 の 4 行を 1 本ずつ |
| `test_connectivity_core.py::test_unreceived_is_fail` | 6.2 |
| `test_connectivity_core.py::test_ap_not_a_criterion` | L-1 |
| `test_connectivity_checker_node.py::test_estop_blocks_link_ok` | L-2（`CL-B-6`） |
| `test_connectivity_checker_node.py::test_link_ok_once` | L-3 |

### 8. Gazebo シナリオ

`gazebo.launch.py`。**Gazebo には `esp32_bridge` が居ない**ので、
`required_nodes` と ESP32 の 2 項目を `sim:=true` で除外する
（**除外の事実を S-00 に表示する**。黙って通さない）。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部**（ESP32 は電源断でも WiFi・フィードバックは生きる構成が前提） | なし |

**ESP32 のモータ電源だけを切っている状態で `wheel_feedback` が来ることを確認する。**
来ないなら、その構成では段階 1 の実機確認そのものが成立しないので**ユーザーに確認する。**

### 10. 完了条件

```bash
cd /root/th_ws && colcon build --symlink-install --packages-select th_state
python3 -m pytest src/th_testing/test/test_connectivity_core.py -v
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R connectivity

# 実機（電源断）: 疎通が揃うと IDLE へ進む
ros2 topic echo /system/state --field mode --once     # IDLE

# LiDAR を止めると INIT に戻らない（運用中は safety_monitor の担当）が、
# 再起動すれば INIT/CHECK で止まる
```

### 11. 既知の負債・未確定 (c)

| 項目 | 扱い |
| --- | --- |
| `restart_max_count` / `restart_wait_ms` | **`O-d4`。placeholder のまま**（`blocking_from_stage: 7`） |
| 機体側 LED | [state](DetailedDesign-state.md) §12.4 の申し送り。**このパケットには含めない** |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-STATE-02` |
| 被依存 WP | `WP-UI-02`（S-00 の表示源） |

---

## `WP-PARAM-02` `params_audit` ＋ launch の `OpaqueFunction`

### 0. 一行要旨

`registry.yaml` から**起動のたびに** ROS2 パラメータ YAML を生成し、
アサーション違反なら **launch を止める**。生成と監査は分ける。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `th_bringup/launch/` の `OpaqueFunction`（生成＋アサーション） | `export.py` / `assertions.py` の中身（`WP-PARAM-01`） |
| `th_params/scripts/params_audit.py`（監査ノード） | 校正値の書き込み（`WP-MAINT-02`） |
| **`bringup.launch.py` / `gazebo.launch.py` の `parameters=` を `generated/` へ差し替える**（`O-8`） | |
| **`docker-compose.yml` に `./data:/root/th_data` の bind mount を足す** | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [params](DetailedDesign-params.md) §5 | **生成（launch）と監査（ノード）の分離。鶏卵を避ける理由** |
| [params](DetailedDesign-params.md) §5.1・§5.2 | `params_digest` ・ヘッダのバッジ |
| [params](DetailedDesign-params.md) §4 | A1〜A11（**違反時に launch を止める**） |
| [params](DetailedDesign-params.md) §1.3 | `blocking_from_stage` と `consumers` の絞り込み |
| [params](DetailedDesign-params.md) §10 | 保存先 |
| [names](DetailedDesign-names.md) §9.2 | **`/root/th_data/` のレイアウト**（bind mount の対象） |
| [names](DetailedDesign-names.md) §5.1 `ParamsStatus.msg` | フィールド |
| [reuse](DetailedDesign-reuse.md) §2.9 | `th_bringup` の既存 launch 構造（**壊さない**） |
| [reuse](DetailedDesign-reuse.md) §2.14 | Docker の既存の流儀（`dr_spaam_weights` と同じ形で足す） |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS |
| --- | --- | --- | --- |
| **pub** | `/system/params_status` | `ParamsStatus` | **transient_local, depth 1**・変化時 |

#### 3.2 サービス

| サービス | 型 | 拒否 |
| --- | --- | --- |
| `/params/get` | `GetParams` | — |
| `/params/set` | `SetParams` | **`IDLE` / `MANUAL` 以外は拒否**（現行 `config_manager` の流儀を維持） |
| `/params/save` | `std_srvs/Trigger` | 同上 |

#### 3.3 パラメータ

| 名前 | class | status | 用途 |
| --- | --- | --- | --- |
| （launch 引数）`stage` | — | — | A8 の判定。**既定は最大値** |
| （launch 引数）`sim` | — | — | `allow_placeholder` |

**`params_audit` 自身のパラメータは持たない**（registry のパスだけ）。

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コア

なし（`WP-PARAM-01` の `export.py` / `assertions.py` を呼ぶだけ）。

#### 4.2 ノードの責務

`generated/` を読んで `/system/params_status` を publish し、3 サービスを提供するだけ。
**生成はしない**（launch の `OpaqueFunction` が済ませている）。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **G-1** | **生成はノード起動より前に同期実行する** | launch は `parameters=[...]` をノード起動前に評価する。初回起動時にファイルが無い |
| **G-2** | アサーション違反なら **launch を止める**（`params_audit` の起動を待たない） | 「起動はしたが値が危険」という状態を作らない |
| **G-3** | `twist_mux.yaml` も生成対象 | `manual_joy.timeout` を 2 か所に持つと `DD-6` が最初のパケットで破れる |
| **G-4** | `bringup` と `gazebo` の**両方**の `parameters=` を差し替える（`O-8`） | 片方だけだとシミュレーションが旧値で動く |

### 5. 表駆動データ

なし。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **全層の閾値を配る**（層そのものには触れない） |
| 6.2 フェイルセーフ既定 | 生成に失敗したら **launch を止める**（古い `generated/` を使わない） |
| 6.3 FMEA | ① 古い `generated/` が残って使われる → **起動のたびに削除してから生成する。**② `twist_mux.yaml` の生成を落とす → `manual_joy.timeout` が registry とずれ、A4（`jog_lease_ms ≥ timeout`）が空振りする。③ bind mount を忘れる → `--rm` のコンテナで `/root/th_data` が毎回消え、**校正値も教示経路も残らない** |

### 7. 単体試験

| テスト | 満たす仕様 |
| --- | --- |
| `test_params_launch.py::test_generated_before_nodes` | G-1（`OpaqueFunction` の評価順） |
| `test_params_launch.py::test_assertion_stops_launch` | G-2 |
| `test_params_launch.py::test_twist_mux_generated` | G-3 |
| `test_params_launch.py::test_both_launch_files_use_generated` | **G-4。2 ファイルを grep で検査** |
| `test_params_audit_node.py::test_status_published` | `/system/params_status` |
| `test_params_audit_node.py::test_set_rejected_outside_idle` | §3.2 |

### 8. Gazebo シナリオ

`gazebo.launch.py sim:=true stage:=1`。**`allow_placeholder: true` で起動できること。**

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部** | なし |

```bash
ros2 launch th_bringup bringup.launch.py stage:=1
ls /root/th_data/generated/
ros2 topic echo /system/params_status --once
```

### 10. 完了条件

```bash
cd /root/th_ws

# ① 両方の launch が generated/ を使う（G-4）
grep -c "th_data/generated" src/th_bringup/launch/bringup.launch.py   # >= 1
grep -c "th_data/generated" src/th_bringup/launch/gazebo.launch.py    # >= 1
# 旧パスが残っていない
! grep -rn "config/planning_params.yaml\|config/safety_monitor.yaml" \
    src/th_bringup/launch/bringup.launch.py

# ② bind mount がある
grep -n "th_data" docker-compose.yml

# ③ アサーション違反で launch が止まる（G-2）
python3 - <<'EOF'
# registry を一時的に壊して（A2a 違反）launch が非ゼロで終わることを確認
EOF

# ④ 生成物
ros2 launch th_bringup bringup.launch.py stage:=1 &
sleep 15 && ls /root/th_data/generated/twist_mux.yaml
ros2 topic echo /system/params_status --once | grep placeholder_count

# ⑤ テスト
python3 -m pytest src/th_testing/test/test_params_launch.py \
                  src/th_testing/test/test_params_audit_node.py -v
```

### 11. 既知の負債・未確定 (c)

`stage` 引数の既定を「最大値」にしてあるので、**段階を指定し忘れると起動できない。**
これは意図した設計（`DD-6`）で、`stage:=` の指定を各段階の起動手順に必ず書く。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-PARAM-01` |
| 被依存 WP | `WP-SAFE-02` / `WP-SAFE-03` / `WP-ROUTE-01` ほか、パラメータを読む全ノード |

---

## `WP-UI-01` シェル（ヘッダ・操作カード・ウィンドウ・i18n）

### 0. 一行要旨

既存 React の**3 タブシェルを捨てて**、ヘッダ最上位・操作カード・W-1〜W-6 の骨格に作り直す。
**画面の中身はまだ作らない。**

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `shell/` 一式（`AppShell` / `Header` / `OperationCard` / `OperationBar` / `Windows` / `theme.css`） | 15 画面の中身（`WP-UI-02` 以降） |
| `ros/useSystemState.js` / `useTrigger.js` / `topics.js` | ジョグの送出（`WP-UI-03`） |
| `i18n/modes.js` / `states.js` / `reasons.js` / `guides.js` | S-00・S-01（`WP-UI-02`） |
| **Playwright の導入と §9-1〜11 のうち 1・3・4・5・7・10 の実行環境** | 音声・観客ビュー（既存を維持） |
| `parts/ArmedButton.jsx`（既存 `App.jsx` L262-286 の抽出） | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [webui](DetailedDesign-webui.md) §1 | **ファイル構成（そのまま作る）** |
| [webui](DetailedDesign-webui.md) §2.1〜§2.5 | **重なり順・非常停止の寸法・文字サイズ・スクロール禁止・畳み順** |
| [webui](DetailedDesign-webui.md) §3 | **既存資産の流用表**（捨ててよいもの・残すもの） |
| [webui](DetailedDesign-webui.md) §4・§4.1・§4.2・§4.3 | 操作カードの格子・ボタン→トリガ・色 |
| [webui](DetailedDesign-webui.md) §6・§6.1・§6.2・§6.3 | W-1〜W-6 |
| [webui](DetailedDesign-webui.md) §7・§7.1・§7.2 | 日本語辞書・**UI に説明文を置かない** |
| [webui](DetailedDesign-webui.md) §9 | **受け入れ条件 1〜11（機械判定）** |
| [names](DetailedDesign-names.md) §4・§4.2 | 画面 ID・ゾーン・ウィンドウ ID |
| [names](DetailedDesign-names.md) §8.1 | `ui.*` の一覧（`useTrigger` が送れるもの） |
| [state](DetailedDesign-state.md) §7.1 | **W-1 の選択肢は `attributes.yaml` の `resume` から引く**（ハードコードしない） |
| [state](DetailedDesign-state.md) §10 | `reject_reason_key` → `i18n/reasons.js` |
| `docs/plan/spec/mockup/index.html` | **CSS とレイアウト規則の正本**（`container-type: size` ＋ `clamp()`） |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS |
| --- | --- | --- | --- |
| sub | `/system/state` | `SystemState` | transient_local, depth 1 |
| sub | `/system/params_status` | `ParamsStatus` | transient_local, depth 1（**ヘッダのバッジ**） |
| **pub** | `/ui/active_screen` | `ActiveScreen` | reliable, depth 5・**2 Hz** |
| **pub** | `/safety/estop_ui` | `std_msgs/Bool` | reliable・**押下中は 2 Hz で送り続ける** |

**`/safety/estop_ui` は `safety_monitor` 宛て。**`th_state` へ直接送らない（[safety](DetailedDesign-safety.md) §6.2）。

#### 3.2 サービス

| サービス | 用途 |
| --- | --- |
| `/system/trigger` | `ui.*` の送出（`ui.jog.hold` を**除く**） |

#### 3.3 パラメータ

| 名前 | 由来 | 用途 |
| --- | --- | --- |
| `estop_ui_repeat_hz` | registry（given） | §3.1 の 2 Hz |
| `speed_preset_low/_mid/_high` | registry（given, `consumers: [web_ui]`） | `WP-UI-03` で使う |

**WebUI は `registry.yaml` を読めない。**`/params/get` で取得し、
**取得できるまでは操作を出さない**（数字をソースに書かない）。

#### 3.4 フレーム

`MapCanvas` は `map` / `odom` / `base_link`。**このパケットでは地図を描かない。**

### 4. 内部設計

#### 4.1 純粋コア

```js
// shell/limits.js — 表示のためだけ。判定の正本ではない
export function operationCardLayout(mode, state, attributes) -> {slots: [...]}
export function resumeChoices(mode, attributes) -> "yes_no" | "ack_only" | "none"
```

**`resumeChoices` は `attributes.yaml` を JSON 化したものを読む**（[state](DetailedDesign-state.md) §7.1）。
ビルド時に `th_state/config/attributes.yaml` → `web_ui/src/generated/attributes.json` へ変換する。

#### 4.2 ノードの責務

React。`useSystemState` が `/system/state` を購読し、context で配る。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **U-1** | **ヘッダの z-index が最大**（100）。W-1〜W-6 より上 | 異常ウィンドウを開いた状態で非常停止が押せること（§9-1） |
| **U-2** | **`alert` / `confirm` / `prompt` を使わない** | ヘッダより上に出る |
| **U-3** | 外部ホストへのリクエスト 0 件 | 機体の AP 配下にインターネットが無い（§9-5） |
| **U-4** | 日本語文字列は `i18n/` にしかない（`R4`） | ノードはキーを返す |
| **U-5** | `topics.js` の全トピック名が名前辞書に存在する | §9-10 |
| **U-6** | W-1 の選択肢をハードコードしない | §7.1 |

### 5. 表駆動データ

`i18n/modes.js` は `Spec-webui.md` §8.1 の 18 行。**スナップショットテストで一致を検査**（§9-7）。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 3 の入力**（`/safety/estop_ui`）。ここが押せなくなる設計は違反 |
| 6.2 フェイルセーフ既定 | `/system/state` が途絶したら**ヘッダに「制御系と通信できません」を出し、操作ボタンを全部非活性にする。**モードの表示は最後の値のままにしない（「不明」にする） |
| 6.3 FMEA | ① ウィンドウがヘッダを覆う → **非常停止が押せない。**§9-1 が検出。② `/safety/estop_ui` の継続送信を止める → 押下が伝わらない（**`safety_monitor` 側は押下にラッチするので危険側ではないが、押し直しが必要になる**）。③ CDN からフォントを読む → AP 配下で**画面が出ない**。§9-5 が検出 |

### 7. 単体試験

**[webui](DetailedDesign-webui.md) §9 の 1・3・4・5・6・7・10 がこのパケットの範囲**
（2・8・9・11 は画面が揃ってから ＝ `WP-UI-02` 以降）。

| テスト | 満たす仕様 |
| --- | --- |
| `e2e/estop-clickable.spec.js` | §9-1（U-1） |
| `e2e/estop-tallest.spec.js` | §9-3 |
| `e2e/header-two-lines.spec.js` | §9-4 |
| `e2e/no-external-requests.spec.js` | §9-5（U-3） |
| `e2e/one-primary-button.spec.js` | §9-6 |
| `unit/i18n-modes.test.js` | §9-7 |
| `unit/topics-in-dictionary.test.js` | §9-10（U-5） |

**Playwright の導入手順**（現行リポジトリには無い）:

```bash
cd th_ws/web_ui
npm i -D @playwright/test
npx playwright install --with-deps chromium     # ★ ネットワークが要る。開発 PC で行う
# playwright.config.js: baseURL は vite preview（http://localhost:4173）
# webServer: { command: 'npm run build && npm run preview', port: 4173 }
```

**`npm run dev` ではなく `npm run build && npm run preview` を使う。**
オフライン要件（§9-5）は本番ビルドでないと検証にならない。

### 8. Gazebo シナリオ

`gazebo.launch.py`。rosbridge 経由で `/system/state` が読めること。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部**（表示・ウィンドウ・非常停止ボタンの押下） | なし |

**タブレット実機で短辺 768 px 以上の実端末を 1 台は使う**（§9-2 の前提）。

### 10. 完了条件

```bash
cd th_ws/web_ui

# ① 本番ビルドが通る（現行は誰も実行していない）
npm run build

# ② Playwright
npx playwright test

# ③ 3 タブシェルが消えている
! grep -rn "運用\s*|\s*準備\s*|\s*診断" src/App.jsx 2>/dev/null
test ! -f src/App.css

# ④ 日本語がコンポーネントに無い（R4 / U-4）
! grep -rlnP '[\x{3040}-\x{30ff}\x{4e00}-\x{9fff}]' src/shell/ src/parts/ src/ros/

# ⑤ topics.js が名前辞書と一致
node --test test/unit/topics-in-dictionary.test.js

# ⑥ 外部ホスト 0 件（②に含まれるが単独でも回せる）
npx playwright test e2e/no-external-requests.spec.js
```

### 11. 既知の負債・未確定 (c)

| 項目 | 扱い |
| --- | --- |
| `speed_preset_*` | `given`。**`/params/get` で取る**（画面には書かない） |
| §9-2（全 15 画面スクロールなし） | **画面が揃うまで実行できない。**`WP-UI-08` の完了条件に置く |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-STATE-02`（`/system/state`）／ `WP-PARAM-02`（`/system/params_status`） |
| 被依存 WP | `WP-UI-02` 〜 `WP-UI-08` / `WP-CALIB-01`（S-40 の 1 ペイン） |

---

## `WP-UI-02` S-00 接続確認 ／ S-01 メインメニュー

### 0. 一行要旨

**必須 3 者が揃うまで S-01 へ進めない。**S-01 に「運用の終了」区画を置く。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `screens/S00Connect.jsx`（機器ごとの状態と総合判定） | 走行方式の画面（段階 3 以降） |
| `screens/S01Main.jsx`（モード選択・運用の終了区画） | 保守の画面（段階 7） |
| `/shutdown/prepare` → W-4 → `/shutdown/execute` の導線 | 実際の保存処理（各機能ノード） |
| Wi-Fi AP が単一障害点である旨の明示 | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [webui](DetailedDesign-webui.md) §8.3 | **S-01 の「運用の終了」区画（5 手順）。カードに展開しない理由** |
| [webui](DetailedDesign-webui.md) §8.4・§8.5 | S-50 への導線（人物追跡 OFF で追従系を非活性に） |
| [webui](DetailedDesign-webui.md) §2.4 | **短辺 768 px でスクロールさせない**（この画面が最も厳しい） |
| [state](DetailedDesign-state.md) §12.1・§12.2 | S-00 に出す 4 項目 |
| [state](DetailedDesign-state.md) §12.4 | 起動完了の提示 |
| [state](DetailedDesign-state.md) §12.5 | **終了の 5 手順**（順序が重要） |
| [safety](DetailedDesign-safety.md) §8.3 | Wi-Fi AP は単一障害点。**S-00 とヘッダに明示** |
| [names](DetailedDesign-names.md) §8.1 | `ui.enter_mode` の `arg_json` |
| [state](DetailedDesign-state.md) §8.3 `mode_entry.yaml` | **`IDLE` から入れるモードだけボタンを出す** |

### 3. インターフェース契約

#### 3.1 トピック

`WP-UI-01` と同じ。追加は無い。

#### 3.2 サービス

| サービス | 用途 | 拒否理由キーの表示 |
| --- | --- | --- |
| `/system/trigger`（`ui.enter_mode`） | モード選択 | `mode_entry_denied` / `tracker_disabled` / `no_route_recorded` / `link_not_ready` |
| `/shutdown/prepare` | 未保存一覧 | — |
| `/shutdown/execute` | 停止 | `unsaved_remains` |

#### 3.3 パラメータ／3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コア

```js
// screens/mainMenuItems.js
export function menuItems(systemState, modeEntry, attributes) -> [{mode, enabled, reasonKey}]
```

**「押せるか」は `mode_entry.yaml` ＋ 前提条件から導く。**画面に条件を書かない。

#### 4.2 ノードの責務

React のみ。

#### 4.3 不変条件

| # | 不変条件 |
| --- | --- |
| **M-1** | `INIT` の間は S-01 の項目を 1 つも押せない |
| **M-2** | 「終了」という語を**モード選択の文脈でしか使わない**。停止は「制御系を停止する」 |
| **M-3** | 未保存の一覧を**カードに展開しない**（短辺 768 px でスクロールする。モックアップで実測済み） |
| **M-4** | 「制御系を停止する」ボタンは**未保存があっても押せる**（押したら理由が出る） |

### 5. 表駆動データ

`mode_entry.yaml` の `IDLE` 行（13 モード）。**画面にモード名を並べない。**

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | なし |
| 6.2 フェイルセーフ既定 | `evt.link_ok` が来ていなければ S-00 に留まる。**「進む」ボタンを出さない** |
| 6.3 FMEA | ① 疎通不十分でも S-01 へ進める → 走行方式に入れてしまう。**`th_state` 側の `T-INIT-01` が唯一の出口なので UI 単独では破れない**（構造的に安全）。② 停止手順を飛ばして電源を切る → 教示経路と校正値が消える。§8.3 の 5 手順を守る。③ 未保存の判定源（`SystemState.unsaved`）がまだ空 → 段階 1 では常に 0 件。**それでよい**（申告するノードが居ない） |

### 7. 単体試験

| テスト | 満たす仕様 |
| --- | --- |
| `e2e/s00-blocks-until-link.spec.js` | 6.2 |
| `e2e/s01-menu-from-mode-entry.spec.js` | §5・M-1 |
| `e2e/s01-shutdown-flow.spec.js` | §12.5 の 5 手順 |
| `e2e/s01-no-scroll-768.spec.js` | M-3（§9-2 の S-01 分だけ先に） |
| `unit/main-menu-items.test.js` | `menuItems()` |

### 8. Gazebo シナリオ

`gazebo.launch.py`。`sim:=true` で除外した疎通項目が**除外と分かる形で表示される**こと。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部** | なし |

### 10. 完了条件

```bash
cd th_ws/web_ui && npm run build && npx playwright test e2e/s00-*.spec.js e2e/s01-*.spec.js

# 実機（電源断）
ros2 topic echo /system/state --field mode --once     # INIT → IDLE を確認
# LiDAR を止めて再起動 → S-00 で止まり、進むボタンが出ないこと
```

### 11. 既知の負債

`SystemState.unsaved` を立てるノードが段階 1 には無いので、
**W-4 の未保存一覧は空のまま。**導線が動くことだけを確認する。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-UI-01` / `WP-STATE-03` |
| 被依存 WP | `WP-CARRY-01`（W-2 の下に S-01 が居る）／ 以降の全画面 |

---

## `WP-CARRY-01` 手押しモード（`CARRY`）一式

### 0. 一行要旨

**物理非常停止を押したら `CARRY` に入り、解除して「再開」で押下前のモードへ戻る。**
`CARRY` 中は UI 非常停止を受け付けない（`F-26`）。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `C-07` / `C-10` / `C-11` / `C-12` / `C-06r` の**動作確認と不足分の実装** | 遷移表の行そのもの（`WP-STATE-01` で入っている） |
| W-2（手押しウィンドウ）の実装 | 物理 E-Stop のバイパス解除（`WP-ESP32-01`） |
| `estop_disabled_in_carry` の表示 | フォルトの 2 階級（`WP-SAFE-01`） |
| `esp32_bridge` → `/safety/estop_hw` の経路の確認 | |

**このパケットは「横断」である。**新しいノードを作らず、
`th_state` ＋ WebUI ＋ 既存 `esp32_bridge` の 3 者が正しく繋がっていることを成立させる。

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [state](DetailedDesign-state.md) §4.1（`C-06r` / `C-07` / `C-10` / `C-11` / `C-12`） | **5 行の定義** |
| [state](DetailedDesign-state.md) §7 | **ラッチの 6 通り**（`CARRY` 中の重大フォルトで `prev_*` を上書きしない） |
| [state](DetailedDesign-state.md) §4.1.1 | `hw_released_and_no_critical` / `can_finish` / `estop_ui_allowed` |
| [safety](DetailedDesign-safety.md) §6.1 | 系統図（物理 → ESP32 → `esp32_bridge` → `/safety/estop_hw`） |
| [safety](DetailedDesign-safety.md) §6.4 | **`safety_monitor` 側では止めない**（安全経路に条件分岐を入れない） |
| [safety](DetailedDesign-safety.md) §6.5 | 起動時に押されたまま（`T-INIT-03`） |
| [webui](DetailedDesign-webui.md) §6 | W-2 は**解除するまで閉じない** |
| [names](DetailedDesign-names.md) §2.2 | `CARRY` の画面は**直前の画面**・ゾーンも直前のまま |
| [reuse](DetailedDesign-reuse.md) §2.6 | `esp32_bridge` の `ESTOP_HW (0x03)` の扱い（**既存を維持**） |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | 備考 |
| --- | --- | --- | --- |
| sub（`th_state`） | `/safety/estop_hw` | `std_msgs/Bool` | 立ち上がり → `hw.estop.press`、立ち下がり → `hw.estop.release` |
| pub（`esp32_bridge`・既存） | `/safety/estop_hw` | `std_msgs/Bool` | **10 Hz。既存実装をそのまま使う** |

#### 3.2 サービス

| サービス | トリガ | 期待 |
| --- | --- | --- |
| `/system/trigger` | `ui.carry_resume` | `C-11`。`hw_released_and_no_critical` が真なら `$prev_mode` / `$prev_state` |
| `/system/trigger` | `ui.estop.press`（`CARRY` 中） | **`accepted=false`, `reject_reason_key="estop_disabled_in_carry"`** |
| `/system/trigger` | `ui.finish`（`CARRY` 中） | `C-12`。`hw_released` が真なら `IDLE` |

#### 3.3 パラメータ

なし。

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コア

なし（`state_core` が既に持っている）。

#### 4.2 ノードの責務

`state_manager` が `latch_prev` / `clear_prev` を処理し、
WebUI が `/system/state` の `mode == "CARRY"` を見て W-2 を出す。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **C-1** | **W-2 は物理ボタンを解除するまで閉じられない** | 駆動が切れていることを試験員に認識させる |
| **C-2** | `CARRY` 中の UI 非常停止ボタンは**隠さない。**効かないことが分かる見た目にする | 隠すと「押したはずなのに反応しない」より悪い |
| **C-3** | `CARRY` 中に `fault.critical` が来たら `ESTOP` へ移り、**`prev_*` は上書きしない** | `CARRY` を復帰先にしない |
| **C-4** | `ESTOP` 中に物理押下が来たら `CARRY` へ移り、**`prev_*` は上書きしない** | 同上 |
| **C-5** | 画面もゾーンも変わらない | `CARRY` は**ウィンドウを乗せるだけ**（[state](DetailedDesign-state.md) §6） |

### 5. 表駆動データ

`transitions.yaml` の 5 行を**そのまま使う**。ここで行を足さない。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 1 の帰結を層 4 に映す。**層 1 そのものには触れない |
| 6.2 フェイルセーフ既定 | `/safety/estop_hw` が途絶したら **`safety_monitor` が `ESP32_DISCONNECTED` を立てる**（`WP-SAFE-01`）。段階 1 の時点では途絶＝押下扱いにはしない（**`WP-SAFE-01` で `lock_stale_ms` により塞ぐ**） |
| 6.3 FMEA | ① `C-06r` を落とす → `CARRY` 中の UI 押下が `prev_*` を `CARRY` に上書きし、**再開先が `CARRY` 自身になって出られなくなる**。② `hw_released_and_no_critical` を `hw_released` だけにする → **重大フォルトが継続したまま走行モードへ戻る**（`Spec-safety.md` §3.5 違反）。③ W-2 を閉じられるようにする → 駆動が切れていることに気づかず操作を続ける |

### 7. 単体試験

| テスト | 満たす仕様 |
| --- | --- |
| `test_state_core.py::test_carry_roundtrip` | `C-07` → `C-10` → `C-11`（`@pytest.mark.rule`） |
| `test_state_core.py::test_estop_in_carry_rejected` | `C-06r`（`WP-STATE-01` と共有） |
| `test_state_core.py::test_critical_in_carry_keeps_prev` | C-3 |
| `test_state_core.py::test_hw_press_in_estop_keeps_prev` | C-4 |
| `test_state_core.py::test_carry_resume_blocked_by_critical` | FMEA ② |
| `e2e/w2-cannot-close.spec.js` | C-1 |
| `e2e/w2-estop-shows-reason.spec.js` | C-2。`estop_disabled_in_carry` が W-2 に出る |

### 8. Gazebo シナリオ

`gazebo.launch.py`。**Gazebo には物理ボタンが無い**ので、
`/safety/estop_hw` を `ros2 topic pub` で模擬する。

```bash
ros2 topic pub -1 /safety/estop_hw std_msgs/Bool "{data: true}"
ros2 topic echo /system/state --field mode --once     # CARRY
```

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| `/safety/estop_hw` の押下・解除の検出／`CARRY` への遷移／W-2 の表示／`ui.carry_resume` での復帰 | **駆動が実際に切れることの確認**（モータードライバ電源の電気的遮断） |

**モータ電源断のままでも `CARRY` の全遷移は確認できる。**
物理ボタンが `/safety/estop_hw` を立てること自体は ESP32 の GPIO と WiFi だけで成立する。
**ただし `DEBT-1`（バイパス）が有効な間は `estopActive` が常に false なので、
`ESTOP_HW` フレームが飛ばない可能性がある** → その場合は `WP-ESP32-01` を先に済ませる。

### 10. 完了条件

```bash
cd /root/th_ws

# ① 遷移
python3 -m pytest src/th_testing/test/test_state_core.py -v -k "carry or estop"

# ② UI
cd th_ws/web_ui && npx playwright test e2e/w2-*.spec.js

# ③ 実機（電源断）— 押下 → CARRY → 解除 → 再開 → 元のモード
ros2 service call /system/trigger th_system_msgs/srv/UiTrigger \
  "{trigger: 'ui.enter_mode', arg_json: '{\"mode\":\"MANUAL\"}', requester: 'cli'}"
#   物理ボタンを押す
ros2 topic echo /system/state --field mode --once            # CARRY
ros2 service call /system/trigger th_system_msgs/srv/UiTrigger \
  "{trigger: 'ui.estop.press', arg_json: '', requester: 'cli'}"
#   → accepted: false, reject_reason_key: 'estop_disabled_in_carry'
#   物理ボタンを解除する
ros2 service call /system/trigger th_system_msgs/srv/UiTrigger \
  "{trigger: 'ui.carry_resume', arg_json: '', requester: 'cli'}"
ros2 topic echo /system/state --field mode --once            # MANUAL

# ④ 通電での最終確認（人が関与）
#   物理ボタンを押した状態でモータードライバ電源が実際に落ちていること（テスタ）
```

### 11. 既知の負債・未確定 (c)

| 項目 | 扱い |
| --- | --- |
| **`DEBT-1`（バイパス）** | 有効なままだと ③ が成立しない可能性がある。**`WP-ESP32-01` を先に済ませてよい**（段階 2 だが依存の向きは逆でない） |
| `/safety/estop_hw` の途絶 | 段階 1 では未対処。`WP-SAFE-01` の `lock_stale_ms` で塞ぐ |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-STATE-02` / `WP-UI-01`（W-2） |
| 望ましい先行 | `WP-ESP32-01`（`DEBT-1`） |
| 被依存 WP | なし（段階 1 の出口の一部） |

---

## 段階 1 の出口チェック

```bash
cd /root/th_ws && colcon build --symlink-install
colcon test --packages-select th_testing --event-handlers console_direct+
colcon test-result --verbose
cd th_ws/web_ui && npm run build && npx playwright test

# 実機（モータ電源断）
ros2 launch th_bringup bringup.launch.py stage:=1
ros2 topic echo /system/state --field mode --once      # INIT → IDLE
#  S-01 から MANUAL へ。W-1 を開いた状態で非常停止ボタンが押せること（手で確認）
```

| 出口条件 | 判定 |
| --- | --- |
| 実機で `INIT → IDLE → MANUAL` | `/system/state` |
| **異常ウィンドウを開いた状態で非常停止が押せる** | Playwright §9-1 ＋ 実機で手動確認 |
| 物理押下 → `CARRY` → 解除 → 再開 | `WP-CARRY-01` §10-③ |
| `generated/` が起動のたびに作られる | `ls /root/th_data/generated/` |
| **機体は動かない**（`obstacle_limiter` も `jog_gate` も無い） | 想定どおり。段階 2 へ |
