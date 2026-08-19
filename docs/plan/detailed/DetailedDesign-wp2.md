# 作業パケット — 段階 2（安全層）

[DetailedDesign-packets.md](DetailedDesign-packets.md) §5 の実体。
**§0 の実装規約 `R1`〜`R8` と §0.1 のテンプレートは packets.md にある。先に読む。**

> **この段階が最重要である。**ここで作るものが「上位が壊れても止まる」経路の全部になる。
> **順序制約 `O-1`・`O-2`・`O-4`・`O-5` を破ると、安全装置の無い状態か起動しない中間状態ができる。**

**段階 2 の出口**: Gazebo で故障注入 13 項目が自動で回り、
実機（モータ電源断）で `/cmd_vel` がゼロになることをトピックで確認できること。
**`DEBT-2`〜`DEBT-4` が解除され、`DEBT-1` が重大フォルトとして検出されること**
（`DEBT-1` の検出は `WP-ESP32-01` が段階 0 で用意する。段階 2 は `WP-SAFE-01` でそれを
**重大フォルトに格上げする**ところを担う）。

| 順 | WP | 種別 | 通電 |
| --- | --- | --- | --- |
| 1 | [`WP-SAFE-02`](#wp-safe-02-esp32_bridge-の-cmd_vel-stale-タイムアウトdebt-4) | 実装（ノード） | 不要 |
| 2 | [`WP-CALIB-01`](#wp-calib-01-lidar-死角マスク校正前倒し最小範囲debt-2) | 実装（ノード＋画面） | **必要** |
| 3 | [`WP-SAFE-01`](#wp-safe-01-safety_monitor-改修) | 実装（ノード） | 不要 |
| 4 | [`WP-SAFE-03`](#wp-safe-03-obstacle_limiter-新設--twist_mux-remap--テスト更新) | 実装（ノード＋launch＋テスト） | **必要** |
| 5 | [`WP-SAFE-04`](#wp-safe-04-jog_gate-新設) | 実装（ノード＋WebUI） | 不要 |
| 6 | [`WP-TEST-01`](#wp-test-01-故障注入-13-項目の自動化) | 試験 | **一部必要** |

> **`WP-ESP32-01`（バイパス解除）は段階 0 にある**（[wp0](DetailedDesign-wp0.md)）。
> `O-5` に例外を作らないため段階 2 から移した（[packets](DetailedDesign-packets.md) **§1.1**）。
> **段階 2 に入る時点でバイパスは既に解除されている**——`WP-SAFE-03` と `WP-TEST-01` は
> 走行を伴うので、これが前提である。

---

## `WP-SAFE-02` `esp32_bridge` の `/cmd_vel` stale タイムアウト（`DEBT-4`）

### 0. 一行要旨

`/cmd_vel` が途絶したら**キープアライブの参照値をゼロに書き換える。**
**これを塞がないうちに後段へノードを足すと、安全装置ではなく単一障害点になる**（`O-1`）。

### 1. 対象と非対象

| やる | やらない |
| --- | --- |
| `esp32_bridge.py` に `cmd_vel_stale_ms` の判定を足す | キープアライブそのものの廃止（**残す**） |
| stale 検出を `/safety/fault` へは上げない（**駆動を止めるだけ**） | 独立ロック層の変更（**残す**） |
| `esp32_bridge` の数値部（オドメトリ積分・スタンプ再同期）の純粋コア抽出 | プロトコルの変更 |

**キープアライブと独立ロック層は絶対に消さない**（[reuse](DetailedDesign-reuse.md) §1）。
消すと WiFi の受信ギャップでウォッチドッグが誤発火する。

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §0 `DEBT-4` | 何が起きているか（`esp32_bridge.py:237-238`） |
| [safety](DetailedDesign-safety.md) §3.4 | **対処 1〜4 のうち、このパケットは 2 番** |
| [safety](DetailedDesign-safety.md) §1.1 | **独立ロック層は残す**（消してはいけない理由） |
| [safety](DetailedDesign-safety.md) §10-12 | 故障注入 12（`/cmd_vel` の途絶） |
| [names](DetailedDesign-names.md) §7.3 | **`cmd_vel_stale_ms` と `muxed_stale_ms` は別物** |
| [params](DetailedDesign-params.md) §4 A7 | `cmd_vel_stale_ms < WATCHDOG_MS`（**PC 側が先に止まる**） |
| [reuse](DetailedDesign-reuse.md) §2.6 | `th_esp32_bridge` の去就（純粋コア抽出） |

### 3. インターフェース契約

#### 3.1 トピック

**変更なし。**`/cmd_vel` を購読し、`WHEEL_CMD (0x01)` を送る既存の形のまま。

#### 3.3 パラメータ

| 名前 | 単位 | class | status | 制約 |
| --- | --- | --- | --- | --- |
| **`cmd_vel_stale_ms`** | ms | b | derived | **`< esp32_watchdog_ms`（A7。起動を拒否）** |
| `esp32_watchdog_ms` | ms | given | given | `config.h` の写し |

### 4. 内部設計

#### 4.1 純粋コア

```python
# th_esp32_bridge/th_esp32_bridge/keepalive_core.py
def keepalive_value(last_cmd_ms: int, now_ms: int, last_cmd: Twist2,
                    stale_ms: int, locked: bool) -> Twist2:
    """ロック中 or stale ならゼロ。それ以外は last_cmd をそのまま返す。"""
    if locked or (now_ms - last_cmd_ms) > stale_ms:
        return Twist2(0.0, 0.0)
    return last_cmd
```

#### 4.2 ノードの責務

20 Hz のキープアライブタイマが `keepalive_value()` を呼び、結果を送る。
**「送るのをやめる」のではなく「ゼロを送る」。**送信を止めるとウォッチドッグ経由になり、
最大 `esp32_watchdog_ms`（600 ms）遅れる。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **K-1** | **20 Hz の送信は止めない**（沈黙禁止。多重化の後段だから） | ウォッチドッグより先に止める |
| **K-2** | ロック（`/safety/estop` / `/safety/fault_lock`）は**stale 判定より優先** | 二重化 |
| **K-3** | **ロックトピック自体が `lock_stale_ms` 途絶したらロック扱い**（既存の挙動を維持） | 起動直後の未受信も含む |
| **K-4** | `cmd_vel_stale_ms < esp32_watchdog_ms` | A7 |

### 5. 表駆動データ

なし。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 3 の PC 側末端**（独立ロック層と同じノード） |
| 6.2 フェイルセーフ既定 | 入力途絶 → **ゼロを送り続ける**（K-1） |
| 6.3 FMEA | ① 送信を止めてしまう → ウォッチドッグ待ちになり 600 ms 遅れる。**その間 ESP32 は最後の指令で走る。**② `cmd_vel_stale_ms` を `esp32_watchdog_ms` より長くする → **PC 側の対処が一切効かない**（A7 が起動時に落とす）。③ ロックより stale を先に判定 → ロック中に stale でない指令があると通ってしまう（K-2） |

### 7. 単体試験

| テスト | ctest 登録名（`V6`） | 満たす仕様 |
| --- | --- | --- |
| `test_keepalive_core.py::test_stale_returns_zero` | `keepalive_core` | §4.1 |
| `test_keepalive_core.py::test_locked_returns_zero` | 同上 | K-2 |
| `test_keepalive_core.py::test_fresh_returns_last` | 同上 | 既存挙動の維持 |
| `test_esp32_bridge_node.py::test_still_publishes_at_20hz_when_stale` | `esp32_bridge_node` | **K-1（沈黙しない）** |

```cmake
# src/th_testing/CMakeLists.txt
ament_add_pytest_test(keepalive_core    test/test_keepalive_core.py)
ament_add_pytest_test(esp32_bridge_node test/test_esp32_bridge_node.py)
```

### 8. Gazebo シナリオ

**`esp32_bridge` は Gazebo で起動しない。**
故障注入 12 は**ノード単体の統合テスト**として書く（`launch_testing` で `esp32_bridge` ＋ WS モックサーバー）。
[safety](DetailedDesign-safety.md) §10 の表で「Gazebo ＋ 実機」となっている行のうち、
**12 番だけは「統合テスト ＋ 実機」に読み替える**（申し送り済み）。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部**（`WHEEL_CMD` の中身を WS モニタで観測） | なし |

```bash
# /cmd_vel の publisher を止めて、ESP32 への指令がゼロになることを見る
# V7: kill %1 は非対話シェルでジョブ制御が無効なので使わない。PID を取る
ros2 topic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" & PUB=$!
sleep 2 && kill -TERM $PUB
sleep 1   # cmd_vel_stale_ms（既定 300 ms 想定）より十分長く待つ
test "$(timeout 3 ros2 topic echo /esp32/wheel_cmd_speed --field left --once)" = "0.0"
```

### 10. 完了条件

```bash
cd /root/th_ws && colcon build --symlink-install --packages-select th_esp32_bridge

# ① 純粋コアが ROS2 非依存
! grep -rn "rclpy" src/th_esp32_bridge/th_esp32_bridge/keepalive_core.py

# ② テスト
python3 -m pytest src/th_testing/test/test_keepalive_core.py -v
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R "keepalive_core|esp32_bridge_node|fault_injection_12"
colcon test-result --verbose                    # V5

# ③ 独立ロック層とキープアライブが残っている（消していないことの確認）
grep -n "safety/estop\|safety/fault_lock" src/th_esp32_bridge/scripts/esp32_bridge.py
grep -n "20\|keepalive" src/th_esp32_bridge/scripts/esp32_bridge.py | head

# ④ A7 が効く
python3 -m th_params.export --registry src/th_params/config/registry.yaml \
  --out /tmp/gen --stage 2 --nodes esp32_bridge; echo "exit=$?"    # 0
```

### 11. 既知の負債・未確定 (c)

なし（`cmd_vel_stale_ms` は導出値）。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-PARAM-02` |
| 被依存 WP | **`WP-SAFE-03`（`O-1`。これより前に必ず済ませる）** |

---

## `WP-CALIB-01` LiDAR 死角マスク校正（前倒し・最小範囲）（`DEBT-2`）

### 0. 一行要旨

**幅ゼロのマスクのままリミッタを有効にすると、自分のアルミ角柱で永久停止する**（`O-4`）。
`blind_angle_ranges` を実測して `registry.yaml` に入れる**最小限**だけを作る。

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| S-40 の「LiDAR 死角」ペイン（ライブスキャン＋ドラッグ選択） | `CALIB` の他 3 項目・ウィザードの 4 ステップ骨格 |
| `/calib/submit` の `BLIND` 項目だけ | 履歴 3 世代・ロールバック |
| 結果を `registry.yaml` へ書く | `/root/th_data/calib/` の永続化構造 |
| `OPCHECK` 項目 4（LiDAR）の自動判定 | `OPCHECK` の他 3 項目・故障診断 |
| **角度 → インデックス変換の共通関数**（§4.1） | |

**段階 7 で本格版に作り直す。**ここは「リミッタを動かすために必要な最小限」と割り切る。

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §0 `DEBT-2` | 何が壊れているか（`perception_params.yaml:15-23`） |
| [safety](DetailedDesign-safety.md) §11.2 | **前倒しの範囲**（保守機能全体は作らない） |
| [safety](DetailedDesign-safety.md) §4.3 | 死角は第 3 の扱い（未知 → `v_reverse` 以下） |
| [safety](DetailedDesign-safety.md) §4.4 | **未校正なら自律走行を拒否する**（`BLOCKED_UNCALIBRATED`） |
| [maintenance](DetailedDesign-maintenance.md) **§3.1（4 項目の BLIND 行）・§3.2（ウィザードの骨格）・§3.3（項目ごとの流れ）** | 手順（本格版と同じ手順を最小構成で） |
| [maintenance](DetailedDesign-maintenance.md) §3.6 | 履歴とロールバック。**最小構成版でも履歴は残す**（`K-r3`） |
| [names](DetailedDesign-names.md) §5.2 `/calib/start` `/calib/submit` | サービスの型 |
| [names](DetailedDesign-names.md) §4 | S-40 のゾーンは `NA`・速度上限 `v_calib` |
| [webui](DetailedDesign-webui.md) §1・§4 | 画面の置き場所と操作カード |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | 備考 |
| --- | --- | --- | --- |
| sub | `/scan` | `sensor_msgs/LaserScan` | **生スキャン**（マスク前） |
| pub | `/calib/status` | `CalibStatus` | reliable, depth 5 |

#### 3.2 サービス

| サービス | 要求 | 応答 |
| --- | --- | --- |
| `/calib/start` | `{item: "BLIND"}` | `started` |
| `/calib/submit` | `{item: "BLIND", arg_json: "{\"ranges\":[[a0,a1],...]}"}` | `preview_before` / `preview_after` |
| `/calib/apply` | `{item: "BLIND"}` | `registry.yaml` に書く |

**`BLIND` 以外の `item` は `started=false`, `message="not_implemented"`**（段階 7 で実装）。

#### 3.3 パラメータ

| 名前 | 単位 | class | status | consumers |
| --- | --- | --- | --- | --- |
| `blind_angle_ranges` | deg[] | **c** | **placeholder → measured** | `[lidar_filter, obstacle_limiter]` |
| `calib_blind_tolerance_deg` | deg | c | placeholder（`blocking_from_stage: 7`） | `[calib_runner]` |
| `v_calib` | m/s | b | given | `[obstacle_limiter, calib_runner]` |

#### 3.4 フレーム

`/scan` は `laser_link`。**角度は `laser_link` 基準の `angle_min` からの相対**（§4.1）。

### 4. 内部設計

#### 4.1 純粋コア — **角度 → インデックス変換の規約（`lidar_filter` と共有）**

**この規約が無いと、`lidar_filter` と `obstacle_limiter` が同じ `blind_angle_ranges` を
違うセクタとして解釈する。**両者で 1 つの関数を使う。

```python
# th_perception/th_perception/scan_geometry.py（Python 正本）
# th_safety/include/th_safety/scan_geometry.hpp（C++ 移植。等価性テストあり）

def angle_to_index(angle_rad: float, scan) -> int:
    """LaserScan の angle_min / angle_increment から添字を出す。
    規約:
      ① 角度は laser_link 基準・**ラジアン**・反時計回り正
      ② registry.yaml の blind_angle_ranges は **度**で持ち、ロード時に 1 度だけ変換する
      ③ [-pi, pi) に正規化してから比較する
      ④ 添字は floor((angle - angle_min) / angle_increment)
      ⑤ 範囲外は clamp せず「そのセクタは存在しない」として扱う
    """

def sector_indices(a0_deg: float, a1_deg: float, scan) -> tuple[int, int]:
    """[a0, a1] を添字の閉区間に。a0 > a1 なら 0 をまたぐ 2 区間に割る（呼び出し側で連結）。"""
```

| 規約 | 理由 |
| --- | --- |
| **度で持ち、ラジアンで使う** | 人が入れる値は度（校正画面）。計算はラジアン。**変換は 1 か所** |
| **0 をまたぐ区間を許す** | 現行の `310〜40` のようなマスクが表現できない |
| **範囲外を clamp しない** | clamp すると端のセクタが不当に広がる |

#### 4.2 ノードの責務

`calib_runner`（最小版）が `/scan` を受けて S-40 へ流し、
提出された角度組を `angle_to_index` で検証してから `registry.yaml` に書く。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **B-1** | **幅ゼロの組を受理しない**（`a0 == a1` は拒否） | `DEBT-2` の再発を構造的に防ぐ |
| **B-2** | 提出値は**必ず実スキャンで検証してから**書く（角柱がその角度で `inf` になっているか） | 目視だけで入れない |
| **B-3** | `lidar_filter` と `obstacle_limiter` が同じ関数で解釈する | §4.1 |
| **B-4** | `calib_runner` の走行は **`/cmd_vel_behavior`**（`/cmd_vel_manual` ではない） | `jog_gate` が `CALIB` を塞ぐので `/cmd_vel_manual` からは出せない |

### 5. 表駆動データ — `blind_angle_ranges` の型と変換

**現行 `perception_params.yaml:15-23` は flat な 8 要素リスト**（`- 40.0` / `- 40.0` / …）。
**registry では pair 列にする。**型が変わるので変換規則を決めておく。

| 項目 | 仕様 |
| --- | --- |
| registry の型 | **`list[[float, float]]`**（度。`[[a0, a1], ...]`） |
| 現行からの変換 | **8 要素を先頭から 2 つずつ組にする**（`[40,40], [130,130], [220,220], [310,310]`） |
| 検証 | 変換後に**幅ゼロのペアが 1 つでもあれば `status: placeholder` のまま**（B-1） |
| 単位 | **度で持ち、ラジアンで使う**（§4.1 の規約 ②） |

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 3 の入力**（リミッタが読む死角定義） |
| 6.2 フェイルセーフ既定 | 全ペアが幅ゼロ → `obstacle_limiter` は **`AUTO` の走行開始を拒否**（§4.4） |
| 6.3 FMEA | ① マスクを広く取りすぎる → **本物の障害物が見えなくなる。**`/scan` の生値と重ねて表示し、目視できるようにする。② 角度の符号を取り違える → 反対側をマスクする。**B-2 の実スキャン検証で検出。**③ `lidar_filter` だけ更新して `obstacle_limiter` を忘れる → B-3 |

### 7. 単体試験

| テスト | ctest 登録名（`V6`） | 満たす仕様 |
| --- | --- | --- |
| `test_scan_geometry.py::test_angle_to_index` | `scan_geometry`（`th_testing`） | §4.1 ①〜④ |
| `test_scan_geometry.py::test_wraparound_sector` | 同上 | 0 をまたぐ区間 |
| `test_scan_geometry.py::test_out_of_range_not_clamped` | 同上 | 規約 ⑤ |
| `test_scan_geometry_equivalence`（gtest） | `test_scan_geometry_equivalence`（**`th_safety`**） | **B-3。Python と C++ が同じ添字を返す** |
| `test_calib_blind.py::test_zero_width_rejected` | `calib_blind`（`th_testing`） | B-1 |
| `test_calib_blind.py::test_writes_registry` | 同上 | `/calib/apply` |

```cmake
# src/th_testing/CMakeLists.txt
ament_add_pytest_test(scan_geometry test/test_scan_geometry.py)
ament_add_pytest_test(calib_blind   test/test_calib_blind.py)
# src/th_safety/CMakeLists.txt（DEBT-10 は WP-SAFE-00 で解除済み）
ament_add_gtest(test_scan_geometry_equivalence test/test_scan_geometry_equivalence.cpp)
```

**`th_safety` 側と `th_testing` 側で同じ `-R scan_geometry` が両方に当たる。**
§10 ③ は**パッケージを分けて 2 回**回す。

### 8. Gazebo シナリオ

`gazebo.launch.py`。**Gazebo の機体には角柱が無い**ので、
`narrow_room` にマスク検証用の障害物を置くか、`/scan` を差し替えた再生テストで代替する。
**実測は実機でしかできない**（B-2）。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **死角の実測そのもの**（機体は止まったまま。`/scan` を見るだけ） | **旋回して確認する場合のみ**（`v_calib`） |

**この校正は電源断でできる**（角柱は機体に固定されているので回さなくても分かる）。

### 10. 完了条件

```bash
cd /root/th_ws && colcon build --symlink-install --packages-select th_perception th_maintenance

# ① 幅ゼロが 0 件（DEBT-2 の解除）
python3 - <<'EOF'
import yaml
r = {p["name"]: p for p in yaml.safe_load(open("src/th_params/config/registry.yaml"))}
p = r["blind_angle_ranges"]
assert p["status"] == "measured", p["status"]
for a0, a1 in p["value"]:
    assert a0 != a1, f"幅ゼロ: {a0},{a1}"
print("ok", p["value"])
EOF

# ② 旧 YAML から生成に切り替わっている（V2・V3）
#    現行は flat な 8 要素リスト（- 40.0 / - 40.0 / ...）。§5 の変換規則で pair 列にする
test -f src/th_bringup/config/perception_params.yaml && \
  ! grep -q "blind_angle_ranges" src/th_bringup/config/perception_params.yaml
grep -q "blind_angle_ranges" /root/th_data/generated/lidar_filter.yaml

# ③ 等価性。V5・V6。th_safety と th_testing を分けて回す
python3 -m pytest src/th_testing/test/test_scan_geometry.py -v
colcon test --packages-select th_safety --event-handlers console_direct+ \
  --ctest-args -R test_scan_geometry_equivalence
colcon test-result --verbose
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R "scan_geometry|calib_blind"
colcon test-result --verbose

# ④ 実機（電源断）で角柱の方向がマスクされる（V4。目視しない・数える）
#    registry の blind_angle_ranges の全区間で inf、区間外に inf が増えていないこと
timeout 5 ros2 topic echo /scan_filtered --once > /tmp/filtered.yaml
timeout 5 ros2 topic echo /scan --once           > /tmp/raw.yaml
python3 - <<'PY'
import math, yaml
raw = yaml.safe_load(open('/tmp/raw.yaml'))
flt = yaml.safe_load(open('/tmp/filtered.yaml'))
reg = {p['name']: p for p in yaml.safe_load(
    open('src/th_params/config/registry.yaml'))}
pairs = reg['blind_angle_ranges']['value']          # [[a0,a1], ...] deg
def idx(deg):
    return int(round((math.radians(deg) - flt['angle_min']) / flt['angle_increment']))
masked = set()
for a0, a1 in pairs:
    assert a1 != a0, ('幅ゼロが残っている', a0, a1)   # B-1 / DEBT-2
    i0, i1 = idx(a0), idx(a1)
    masked.update(range(i0, i1 + 1) if i0 <= i1 else
                  list(range(i0, len(flt['ranges']))) + list(range(0, i1 + 1)))
for i in masked:                                     # 区間内は全部 inf
    assert math.isinf(flt['ranges'][i]), ('マスクされていない', i)
extra = [i for i in range(len(flt['ranges']))
         if i not in masked
         and math.isinf(flt['ranges'][i]) and not math.isinf(raw['ranges'][i])]
assert not extra, ('区間外を消している', extra[:10])   # 広く取りすぎの検出（§6.3 ①）
print('ok masked=%d extra=0' % len(masked))
PY
```

### 11. 既知の負債・未確定 (c)

| 項目 | 扱い |
| --- | --- |
| `calib_blind_tolerance_deg` | **`placeholder`（`blocking_from_stage: 7`）。**最小版では許容判定をしない |
| 履歴 3 世代・ロールバック | 段階 7（`WP-MAINT-02`） |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-UI-01`（S-40 の 1 ペインを載せるシェル）／ `WP-PARAM-02` |
| 被依存 WP | **`WP-SAFE-03`（`O-4`。これより前に必ず済ませる）** |

---

## `WP-SAFE-01` `safety_monitor` 改修

### 0. 一行要旨

フォルトを 2 階級にし、**重大フォルトを `/safety/fault_lock` に載せる**（層 3 で止める）。
タイムアウトを導出値にし、監視対象を段階ごとに有効化できるようにする。

### 1. 対象と非対象

| やる | やらない |
| --- | --- |
| `FaultStatus.severity` の判定と publish | `th_state` 側の `C-03` / `C-06a`（`WP-STATE-01` で入っている） |
| **`fault_lock = LIDAR_LOST \|\| ESP32_DISCONNECTED \|\| severity == CRITICAL`** | `LOCALIZATION_LOST` の検出（`WP-SAFE-05`。段階 5） |
| 監視対象の `enabled_targets` 化（`O-7`） | `obstacle_limiter` の実装（`WP-SAFE-03`） |
| タイムアウトの導出値化（`DEBT-3`） | |
| `PersonStatus` → `PersonTargets` への購読差し替え | |
| バイパス検出（`/safety/firmware_flags`）を重大フォルトに | |
| **Executor の見直し**（`WP-SAFE-00` からの申し送り。下記） | |

> **`WP-SAFE-00` からの申し送り（2026-08-18）**: `link_quality` の 1 Hz タイマは
> `FMEA ③` に従って別のコールバックグループに置いたが、**`main()` は `rclcpp::spin()`
> のままなので実行は単一スレッドで、分離の効果は出ていない。**
> `WP-SAFE-00` で Executor を替えなかったのは、**既存のフォルト判定が
> `bool` と `rclcpp::Time` をロック無しで共有しており、スレッドセーフに書かれていない**ため
> （替えると新たな競合を作り、`Q-3`「既存挙動不変」に反する）。
> **このパケットが `safety_monitor` を書き換えるので、ここで決める**——
> マルチスレッド Executor にするなら**共有状態の保護も同時に入れる**こと。
> 入れないなら「単一スレッドのままでよい」と明記して申し送りを閉じる。
| **`MUX_DEAD` / `DRIVE_RUNAWAY` / `STATE_INCONSISTENT` の検出条件**（§4.1） | |
| `test_safety_monitor.py` / `test_fault_detection.py` の**同時更新** | |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §5.1 | **2 階級の分類表**（必須通信系統の断を CRITICAL に含めない理由） |
| [safety](DetailedDesign-safety.md) §5.2 | **変更点の 5 行**（現行 → 新設計） |
| [safety](DetailedDesign-safety.md) §5.2.1 | **重大フォルトは層 3 で止める**（致命的な穴だった） |
| [safety](DetailedDesign-safety.md) §5.4 | 人物ロストの 3 段 |
| [safety](DetailedDesign-safety.md) §6.3 | UI 非常停止の生存確認（**押下側にラッチする**） |
| [safety](DetailedDesign-safety.md) §7 | タイムアウトの 2 本制約 |
| [safety](DetailedDesign-safety.md) §8.1・§8.2 | 通信断で止める条件・「使用中」の判定 |
| [safety](DetailedDesign-safety.md) §11.1 | バイパス検出を重大フォルトに |
| [names](DetailedDesign-names.md) §5.1 `FaultStatus` | `severity` フィールド |
| [names](DetailedDesign-names.md) §6.2 | `/safety/*` の QoS とレート |
| [params](DetailedDesign-params.md) §3.3 | `timeout_*_bound_ms` |
| [reuse](DetailedDesign-reuse.md) §2.3 | `safety_monitor.cpp` の既存構造 |
| [packets](DetailedDesign-packets.md) §1 `O-7` | **監視対象は publisher ができるまで有効にしない** |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS | レート |
| --- | --- | --- | --- | --- |
| sub | `/scan` | `LaserScan` | SensorDataQoS | 約 10 Hz |
| sub | **`/person/targets`** | `PersonTargets` | reliable, depth 1 | **段階 4 まで無効** |
| sub | `/esp32/wheel_feedback` | `WheelFeedback` | reliable | 20 Hz |
| sub | **`/safety/limiter_status`** | `LimiterStatus` | best_effort, depth 1 | **段階 2（`WP-SAFE-03` 後）から有効** |
| sub | **`/map_session/status`** | `MapSessionStatus` | transient_local | **段階 5 まで無効** |
| sub | **`/safety/firmware_flags`** | `std_msgs/UInt8` | transient_local | 変化時 |
| sub | `/safety/estop_ui` | `std_msgs/Bool` | reliable | 2 Hz |
| sub | `/cmd_vel_muxed` ／ `/cmd_vel` | `Twist` | reliable, depth 1 | **`MUX_DEAD` / `DRIVE_RUNAWAY` の検出用** |
| sub | `/system/state` | `SystemState` | transient_local | **`STATE_INCONSISTENT` の検出用** |
| pub | `/safety/fault` | `FaultStatus` | reliable, depth 5 | 変化時 |
| pub | `/safety/fault_lock` | `std_msgs/Bool` | reliable | **10 Hz** |
| pub | `/safety/estop` | `std_msgs/Bool` | reliable | **10 Hz** |
| pub | `/safety/link_quality` | `LinkQuality` | best_effort | 1 Hz ×3（`WP-SAFE-00` で入っている） |

#### 3.2 サービス

| サービス | 型 | 備考 |
| --- | --- | --- |
| `/safety/estop_ui` | — | **`/safety/tablet_estop` から改名**（端末はタブレットとは限らない）。**トピックであってサービスではない** |
| **`/safety/clear_estop_ui`** | `std_srvs/Trigger` | **UI ラッチの非 UI 解除**（[safety](DetailedDesign-safety.md) §6.3.1 ／ `N-4`）。受理条件は **`estop_hw == false` かつ重大フォルト無し**。拒否時は `success=false` と理由を `message` に入れる。**受理・拒否のどちらもログに残す**（`who=cli`・時刻・そのときの `mode`/`state`） |

#### 3.3 パラメータ

| 名前 | 単位 | class | status | 備考 |
| --- | --- | --- | --- | --- |
| `lidar_timeout_ms` / `esp32_timeout_ms` / `person_timeout_ms` | ms | b | **derived** | `DEBT-3`。§3.3 の 2 本制約 |
| `lock_stale_ms` | ms | b | derived | ロックトピックの途絶 |
| **`estop_ui_lease_ms`** | ms | b | derived | §4.1 の `UI_ESTOP_STALE` |
| `enabled_targets` | string[] | given | given | **段階ごとに launch から渡す**（`O-7`） |
| `mux_dead_ms` / `runaway_ratio` / `runaway_hold_ms` / `state_stale_ms` | ms / — | b | derived | §4.1 |
| `tracker_lost_grace_ms` | ms | **c** | **placeholder**（`blocking_from_stage: 4`） | **裸の数値を書かない。**§5.4 の「暫定 500 ms」は registry の初期値ではなく**目安の記述** |

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 重大フォルト 3 種の検出条件（**初出。ここが正**）

[safety](DetailedDesign-safety.md) §5.1 は名前だけを挙げていた。**検出器をここで決める。**

| フォルト | 検出条件 | 検出器 | 誤検知を避ける工夫 |
| --- | --- | --- | --- |
| **`LIMITER_DEAD`** | `/safety/limiter_status` が `limiter_dead_ms` 途絶 | `safety_monitor` | 20 Hz publish の 5 周期分 |
| **`MUX_DEAD`** | **`/cmd_vel_muxed` に非ゼロが流れているのに `/cmd_vel` が `mux_dead_ms` 途絶**、または逆に **`/cmd_vel_muxed` が途絶しているのに `/cmd_vel` に非ゼロが出続けている** | `safety_monitor` | 両方向を見る。**片方向だけだと `twist_mux` のロック時（正常な無出力）を誤検知する** |
| **`DRIVE_RUNAWAY`** | `/cmd_vel` の指令と `/esp32/wheel_feedback` の実測が **`runaway_ratio` を超えて乖離した状態が `runaway_hold_ms` 継続** | `safety_monitor` | 加減速中は乖離するので**保持時間**で判定。停止指令中に実測が非ゼロ、が最も重要なケース |
| **`STATE_INCONSISTENT`** | `/system/state` が `state_stale_ms` 途絶、または **`mode`/`state` の組が `attributes.yaml` に存在しない** | `safety_monitor` | 状態集合を起動時に読み込む。**`th_state` が壊れた形の状態を出したときの最後の網** |

**`DRIVE_RUNAWAY` は「停止指令中に動いている」を最優先で拾う。**
`|cmd| ≈ 0` かつ `|feedback| > runaway_zero_threshold` が `runaway_hold_ms` 続いたら即座に重大。

#### 4.2 UI 非常停止の生存確認（`estop_ui_lease_ms` の用途）

[safety](DetailedDesign-safety.md) §6.3 は「途絶しても `false` にはしない（押下側にラッチする）」と定めている。
**では `estop_ui_lease_ms` は何に使うのか** ——

| 状況 | 挙動 |
| --- | --- |
| `true` を受けてから `estop_ui_lease_ms` 途絶 | **ラッチは維持**（押下のまま）。**あわせて `UI_DISCONNECTED`（回復フォルト）を立てる** |
| `false` を明示受信 | ラッチ解除 |

**これで「UI が落ちて押しっぱなしになった」ことが試験員に見える。**
ラッチしたまま黙っていると、原因が分からないまま動かなくなる。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **F-1** | **`/safety/estop` と `/safety/fault_lock` は 10 Hz で出し続ける**（沈黙禁止） | 購読側が `lock_stale_ms` でロック扱いにするので、沈黙＝ロックになる。**それでよい**が、意図しない沈黙を作らない |
| **F-2** | `severity == CRITICAL` は必ず `fault_lock` に載る | §5.2.1 |
| **F-3** | 必須通信系統の断は `RECOVERABLE` | `CRITICAL` にすると再開フローが丸ごと死ぬ |
| **F-4** | 開発モードのフラグを `safety_monitor` に渡さない | §12（構造的な保証） |
| **F-5** | `enabled_targets` に無い対象は**監視しない**（フォルトを立てない） | `O-7`。段階 2 で `/person/targets` の publisher は居ない |

### 5. 表駆動データ

`enabled_targets` の段階別の値。**launch 引数 `stage` から決める。**

| 段階 | `enabled_targets` |
| --- | --- |
| 1 | `[lidar, esp32]` |
| 2 | `[lidar, esp32, limiter, mux, runaway, state, firmware]` |
| 4 | ＋ `[person]` |
| 5 | ＋ `[localization]` |

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 3 そのもの** |
| 6.2 フェイルセーフ既定 | 監視対象の未受信＝フォルト（`enabled_targets` に入っていれば）。ロックは**出し続ける** |
| 6.3 FMEA | ① `severity` を `fault_lock` に載せ忘れる → **重大フォルトを止めるのが層 4 だけになり、100 ms 要件を自分で満たせない**（§5.2.1。初版の穴）。② `MUX_DEAD` を片方向でしか見ない → **twist_mux のロック中（正常）を誤検知して毎回 `ESTOP`**。③ `enabled_targets` を段階 2 で全部 ON → `/person/targets` の publisher が居ないので**起動直後に必ずフォルト** |

### 7. 単体試験

| テスト | ctest 登録名（`V6`） | 満たす仕様 |
| --- | --- | --- |
| `test_safety_monitor.py`（**更新**） | `safety_monitor`（`th_testing`・既存） | `severity` 追加で既存が壊れる。**同一パケットで直す** |
| `test_fault_detection.py`（**更新**） | `fault_detection`（`th_testing`・既存） | 同上 |
| `test_fault_severity`（gtest） | `test_safety_monitor_core`（`th_safety`） | §5.1 の分類表を 1 行ずつ |
| `test_fault_lock_includes_critical`（gtest） | 同上 | F-2（§5.2.1） |
| `test_mux_dead_bidirectional`（gtest） | 同上 | §4.1。**ロック中を誤検知しない** |
| `test_runaway_zero_cmd`（gtest） | 同上 | §4.1。停止指令中の非ゼロ実測 |
| `test_state_inconsistent`（gtest） | 同上 | §4.1 |
| `test_ui_estop_latch`（gtest） | 同上 | §4.2。途絶でラッチ維持＋`UI_DISCONNECTED` |
| **`test_clear_estop_ui_requires_hw_released`**（gtest） | 同上 | **§3.2。物理側が押されている間は拒否する**（`N-4` の受理条件） |
| **`test_clear_estop_ui_logs`**（gtest） | 同上 | 受理・拒否の両方が記録される |
| `test_enabled_targets`（gtest） | 同上 | F-5 |

**gtest 7 本は 1 ターゲットにまとめる**（`test/test_safety_monitor_core.cpp` の中の 7 つの `TEST()`）。
左列は**ケース名**であってターゲット名ではない。`-R` に渡すのは**右列**。

```cmake
# src/th_safety/CMakeLists.txt
ament_add_gtest(test_safety_monitor_core test/test_safety_monitor_core.cpp)
target_link_libraries(test_safety_monitor_core safety_monitor_core)
```

### 8. Gazebo シナリオ

`gazebo.launch.py stage:=2`。故障注入 5・6・9・11 が回ること。

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| 通信断 → フォルト検知の時間測定（§10-5）／フォルト → `/cmd_vel` ゼロ（§10-6） | **`DRIVE_RUNAWAY` の実測**（車輪が回る必要がある） |

### 10. 完了条件

```bash
cd /root/th_ws && colcon build --symlink-install --packages-select th_safety

# ① 既存テスト 2 本が更新されて通る（severity 追加で両方壊れるので同時に直す）
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R "safety_monitor|fault_detection"
colcon test-result --verbose

# ② gtest（V6。ターゲット名は §7 の右列）
colcon test --packages-select th_safety --event-handlers console_direct+ \
  --ctest-args -R test_safety_monitor_core
colcon test-result --verbose     # V5

# ③ 重大フォルトが fault_lock に載る（F-2）
ros2 topic pub -1 /safety/limiter_status th_system_msgs/msg/LimiterStatus "{alive: false}"
sleep 1
test "$(timeout 3 ros2 topic echo /safety/fault_lock --field data --once)" = "True"
test "$(timeout 3 ros2 topic echo /safety/fault --field severity --once)" = "CRITICAL"

# ④ タイムアウトが導出値（DEBT-3）
grep -n "lidar_timeout_ms" /root/th_data/generated/safety_monitor.yaml
! grep -n "2000" /root/th_data/generated/safety_monitor.yaml    # 導出結果が 2000 でないこと

# ⑤ 段階 2 で /person/targets のフォルトが出ない（F-5）
#    V4: 合格＝何も来ないので timeout の 124 を期待値にする
timeout 10 ros2 topic echo /safety/fault > /tmp/f.txt; test $? -eq 124
! grep -q PERSON_TRACKER_LOST /tmp/f.txt
```

### 11. 既知の負債・未確定 (c)

| 項目 | 扱い |
| --- | --- |
| `tracker_lost_grace_ms` | **(c)・`placeholder`（`blocking_from_stage: 4`）。**§5.4 の「暫定 500 ms」は目安の記述であって registry の値ではない |
| `LOCALIZATION_LOST` | 段階 5（`WP-SAFE-05`）。`enabled_targets` に入れない |
| `runaway_ratio` / `runaway_zero_threshold` | **derived**（`brake_accel_mps2` と `v_max` から）。値は `WP-MEAS-01` 依存 |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-MSG-01` / **`WP-MEAS-04`**（p99）／ `WP-ESP32-01`（`firmware_flags`）／ `WP-PARAM-02` |
| 被依存 WP | `WP-SAFE-03`（`limiter_status` の監視）／ `WP-TEST-01` |

---

## `WP-SAFE-03` `obstacle_limiter` 新設 ＋ twist_mux remap ＋ テスト更新

### 0. 一行要旨

**1 パケットで 6 つを同時に変える**（`O-2`）。分けると起動しない中間状態ができる。

### 1. 対象と非対象

**この 6 つを同時に行う。**

| # | 変更 |
| --- | --- |
| 1 | `th_safety/src/obstacle_limiter.cpp` ＋ `include/th_safety/obstacle_limiter_core.hpp` を新設 |
| 2 | `th_bringup/launch/bringup.launch.py` の `remappings=[('cmd_vel_out','/cmd_vel')]` → **`/cmd_vel_muxed`** |
| 3 | `th_bringup/launch/gazebo.launch.py` の同じ行 → **`/cmd_vel_muxed`** |
| 4 | `twist_mux.yaml` の入力トピック改名: `retreat: /cmd_vel_retreat` → **`behavior: /cmd_vel_behavior`**（priority 20 は維持） |
| 5 | `th_testing/test/test_twist_mux_priority.py` の**出力トピック名と、テスト内のインラインのパラメータ辞書（L39-51）の両方** |
| 6 | `th_safety/package.xml` / `CMakeLists.txt` に依存追加: **`geometry_msgs` / `tf2_ros` / `tf2_geometry_msgs` / `ament_cmake_gtest`** |

| 作らない |
| --- |
| 挙動ノード側の障害物判定（各挙動ノードの担当。二層の上側） |
| 追従対象の除外（**後段は「誰が対象か」を知らない**） |
| `jog_gate`（`WP-SAFE-04`） |

> **4 と 5 を落とすと矛盾が検出できない。**テストは `twist_mux.yaml` を読まずに
> インライン辞書を使っているので、`/cmd_vel_retreat` を publish し続けたまま**通ってしまう。**
> 実機では `/cmd_vel_behavior` に誰も反応しない。

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §2 | **速度指令の経路図と、変更する場所が launch であること** |
| [safety](DetailedDesign-safety.md) §3・§3.1 | **なぜ後段か・二層に分ける・`d_floor` を制動距離から求めない** |
| [safety](DetailedDesign-safety.md) §3.2 | **自律／手動の判定（`manual_fresh && state_fresh && ...`）** |
| [safety](DetailedDesign-safety.md) §3.3・§3.3.1 | 政策表・速度上限は画面由来とモード由来の最小値 |
| [safety](DetailedDesign-safety.md) §3.4.2 | **入力の同期（6 入力の stale 閾値と、そのときの扱い）** |
| [safety](DetailedDesign-safety.md) §3.4.3 | **距離 → 許容速度（`v_allow`）とヒステリシス・判定コーン** |
| [safety](DetailedDesign-safety.md) §3.5 | **不変条件 L1〜L7（property test にする）** |
| [safety](DetailedDesign-safety.md) §3.6 | 実装形態（C++・純粋コアを分ける） |
| [safety](DetailedDesign-safety.md) §4.2・§4.3・§4.4 | **生 `/scan` を使う理由**・死角は第 3 の扱い・未校正なら自律を拒否 |
| [safety](DetailedDesign-safety.md) §7.1 | **リミッタは加減速を鈍らせない**（上限をクランプするだけ） |
| [names](DetailedDesign-names.md) §6.1・§6.5 | トピックと QoS |
| [names](DetailedDesign-names.md) §5.1 `LimiterStatus` | `action` の 5 値 |
| [names](DetailedDesign-names.md) §1.2 | **`laser_link` の変換は起動時に 1 度取得して保持**（20 Hz で TF を引かない） |
| [params](DetailedDesign-params.md) §4 A2a/A2b/A11 | 距離の大小関係 |
| **`WP-CALIB-01` §4.1**（このファイルの前半） | **角度 → インデックス変換の規約**（`lidar_filter` と共有）。**`th_safety` の C++ 側と `th_perception` の Python 側で同じ添字を返すこと**が `B-3` |
| [reuse](DetailedDesign-reuse.md) §2.3 | `th_safety` の既存構成 |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS | レート |
| --- | --- | --- | --- | --- |
| sub | **`/cmd_vel_muxed`** | `Twist` | reliable, depth 1 | 不定 |
| sub | **`/scan`**（**生。`/scan_filtered` ではない**） | `LaserScan` | SensorDataQoS | 約 10 Hz |
| sub | `/system/state` | `SystemState` | transient_local, depth 1 | 10 Hz |
| sub | `/cmd_vel_manual` | `Twist` | reliable, depth 1 | 不定 |
| sub | `/safety/estop` | `std_msgs/Bool` | reliable | 10 Hz |
| sub | `/safety/fault_lock` | `std_msgs/Bool` | reliable | 10 Hz |
| **pub** | **`/cmd_vel`** | `Twist` | reliable, depth 1 | **20 Hz 固定（沈黙禁止）** |
| **pub** | `/safety/limiter_status` | `LimiterStatus` | best_effort, depth 1 | **20 Hz（heartbeat 兼用）** |

**`/cmd_vel` を publish してよいのはこのノードだけ。**`twist_mux` も直接は出さない。

#### 3.2 サービス

なし。**リミッタにサービスを生やさない**（外から止められる経路を作らない）。

#### 3.3 パラメータ

| 名前 | 単位 | class | status |
| --- | --- | --- | --- |
| `obstacle_floor_distance_m` | m | b | derived（**`value_by` なし**。速度非依存） |
| `brake_accel_mps2` | m/s² | c | **measured**（`WP-MEAS-01`） |
| `hysteresis_band_m` | m | b | derived（`obstacle_floor_distance_m × hysteresis_ratio`） |
| `obstacle_cone_half_width_rad` / `_reverse_rad` | rad | b | derived |
| `v_max` / `v_slow` / `v_reverse` / `v_jog_panel` / `v_check` / `v_calib` / `v_leash` | m/s | b | derived / given |
| `w_max` / `w_align_max` | rad/s | b | derived |
| `muxed_stale_ms` / `scan_stale_ms` / `state_stale_ms` / `lock_stale_ms` / `manual_joy_timeout` | ms / s | b / given | derived / given |
| `blind_angle_ranges` | deg[] | c | **measured**（`WP-CALIB-01`） |
| `body_half_length_m` / `floor_margin_m` | m | given / b | given / derived |

**`dev_mode` を受け取らない**（launch から渡さない。[safety](DetailedDesign-safety.md) §12）。

#### 3.4 フレーム

`/scan` は `laser_link`、判定は `base_link`。
**起動時に 1 度だけ `base_link ← laser_link` の固定変換を取得して保持する**（20 Hz で TF を引かない）。
取得に失敗したら**起動を失敗させる**（素通しで動かさない）。

### 4. 内部設計

#### 4.1 純粋コアの関数シグネチャ

**4 つの struct のフィールドを先に決める。**これが無いと property test の入力空間が決まらない。

| struct | フィールド | 由来 |
| --- | --- | --- |
| `Twist2` | `double linear, angular` | `geometry_msgs/Twist` の x と z のみ |
| `ScanView` | `double angle_min, angle_increment; const float* ranges; size_t n; double range_min, range_max` | `/scan`（**生**） |
| `StateView` | `std::string mode, state, zone; bool jog_active, auto_brake` | `/system/state`（§3.1） |
| `P` | §3.3 の全パラメータを `double` で 1 つずつ（`blind_angle_ranges` だけ `std::vector<std::pair<double,double>>`） | `generated/obstacle_limiter.yaml` |

```cpp
// include/th_safety/obstacle_limiter_core.hpp — ROS2 非依存
enum class Action { PASS, CLAMP, STOP, ZERO_STALE, BLOCKED_UNCALIBRATED };
enum class SourceClass { MANUAL, AUTO };

struct Inputs {
  Twist2 cmd; double cmd_age_ms;
  ScanView scan; double scan_age_ms;
  StateView state; double state_age_ms;
  double manual_age_ms;
  bool estop, fault_lock; double lock_age_ms;
  bool stopped_last_tick;          // ヒステリシス
};

struct Output { Twist2 cmd; Action action; double nearest_m;
                SourceClass source_class; double applied_limit_mps; };

SourceClass classify(const Inputs&, const P&);        // §3.2
double nearest_in_cone(const ScanView&, double dir_sign, const P&);  // 最近傍距離
double v_allow(double nearest_m, const P&);           // §3.4.3
double applied_limit(const StateView&, bool reversing, const P&);    // §3.3.1
Output evaluate(const Inputs&, const P&);             // 全部まとめる
```

**`nearest_in_cone` は `mapless_follow_core.py:119-157` の `is_path_blocked` を移植する。**
既存は bool しか返さないので**最近傍距離を返す形に拡張する**（`LimiterStatus.nearest_obstacle_m` に要る）。
**拡張後、既存 Python 実装との等価性テストを書く。**
**人物除外は移植しない**（後段は対象を知らない）。

#### 4.2 ノードの責務

20 Hz の固定タイマで `evaluate()` を呼び、結果を `/cmd_vel` と `/safety/limiter_status` に出すだけ。
**入力コールバックは最新値を保持するだけで、判定をしない。**

#### 4.3 不変条件

**[safety](DetailedDesign-safety.md) §3.5 の L1〜L7 をそのまま property test にする。**

| # | 不変条件（要約） |
| --- | --- |
| L1 | **速度を上げない。**符号を保存する |
| L2 | 入力 stale → `out = 0`。**沈黙しない** |
| L3 | 前方障害物では `linear.x` のみクランプ。`d_floor` を割っている間は `angular.z` に `w_align_max` |
| L4 | 後退時は `\|out.linear.x\| ≤ v_reverse` |
| L5 | 死角セクタが非ゼロ幅で存在する間は `v_reverse` を全方向の上限に |
| L6 | ロック中は無条件にゼロ |
| L7 | **`AUTO` 政策は決して緩まない。**迷いがある入力は `AUTO` へ倒す |

### 5. 表駆動データ

なし（政策表 §3.3 は `if` で書いてよい。3 行 × 3 ゾーンで表駆動にする利得が無い）。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **層 3 の直後・駆動の直前。**`/cmd_vel` の唯一の publisher になる |
| 6.2 フェイルセーフ既定 | 全入力について「一度も受信していない」は stale と同じ。起動直後に素通しにしない |
| 6.3 FMEA | ① **`obstacle_limiter` が落ちる → `/cmd_vel` の publisher が消える。**`WP-SAFE-02` が塞いだので `cmd_vel_stale_ms` 以内にゼロになる（`O-1` の理由）。**加えて `safety_monitor` が `LIMITER_DEAD` を重大フォルトとして立てる。**② `d_floor` を制動距離から求める → **追従対象の脚で発火して追従が成立しない**（A2b が起動時に落とす）。③ `/scan_filtered` を使う → 死角が `inf`＝「空き」と読まれ、**死角対策の当事者が死角を見落とす** |

### 7. 単体試験

| テスト | ctest 登録名（`V6`） | 満たす仕様 |
| --- | --- | --- |
| `test_limiter_core`（gtest）`L1`〜`L7` | `test_limiter_core`（`th_safety`） | **property test（ランダム入力 10,000 通り）** |
| `test_limiter_core::HysteresisHolds` | 同上 | §3.4.3 |
| `test_limiter_core::ConeReverse` | 同上 | 後退時のコーン幅 |
| `test_limiter_core::BlindSectorLimitsAll` | 同上 | L5 |
| `test_limiter_core::ClassifyNeedsBoth` | 同上 | §3.2。**鮮度だけでは `MANUAL` にならない** |
| `test_limiter_core::UncalibratedBlocksAuto` | 同上 | §4.4 |
| `test_limiter_equivalence`（gtest） | `test_limiter_equivalence`（`th_safety`） | **`is_path_blocked` との等価性。**方式は**「Python 側で入出力ベクタを生成して `test/data/limiter_vectors.json` に落とし、gtest はそれを読むだけ」**。pybind11 は使わない（現行リポジトリに前例が無い）。生成スクリプトは `th_testing/tools/gen_limiter_vectors.py` |
| **`test_twist_mux_priority.py`（更新）** | `twist_mux_priority`（`th_testing`・既存） | **出力名 `/cmd_vel_muxed` ＋ インライン辞書の `behavior`** |

```cmake
# src/th_safety/CMakeLists.txt
ament_add_gtest(test_limiter_core       test/test_limiter_core.cpp)
ament_add_gtest(test_limiter_equivalence test/test_limiter_equivalence.cpp)
target_link_libraries(test_limiter_core       limiter_core)
target_link_libraries(test_limiter_equivalence limiter_core)
```

### 8. Gazebo シナリオ

| シナリオ | 見るもの |
| --- | --- |
| `narrow_room` | 故障注入 1（前進で停止） |
| `narrow_room`（後退） | 故障注入 2（`CL-X-4`） |
| 既定 | 故障注入 11（リミッタを SIGKILL → 重大フォルト → 駆動ゼロ） |

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| `/cmd_vel` の内容の観測（クランプ・ゼロ）／`limiter_status` の `action` の遷移 | **障害物へ向けて実際に走らせて止まること** |

### 10. 完了条件

```bash
cd /root/th_ws && colcon build --symlink-install --packages-select th_safety th_bringup

# ① 6 つの変更がすべて入っている（O-2）
grep -n "cmd_vel_muxed" src/th_bringup/launch/bringup.launch.py       # 1 件以上
grep -n "cmd_vel_muxed" src/th_bringup/launch/gazebo.launch.py        # 1 件以上
grep -n "cmd_vel_behavior" src/th_safety/config/twist_mux.yaml        # 1 件以上
! grep -rn "cmd_vel_retreat" src/th_safety/ src/th_bringup/ src/th_testing/
grep -n "cmd_vel_muxed\|cmd_vel_behavior" src/th_testing/test/test_twist_mux_priority.py
grep -n "tf2_ros\|ament_cmake_gtest" src/th_safety/package.xml

# ② /cmd_vel の publisher が obstacle_limiter だけ
ros2 topic info /cmd_vel --verbose | grep -c "Node name: obstacle_limiter"   # 1
ros2 topic info /cmd_vel --verbose | grep -c "Publisher count: 1"            # 1

# ③ property test（V6）
colcon test --packages-select th_safety --event-handlers console_direct+ \
  --ctest-args -R "test_limiter_core|test_limiter_equivalence"
colcon test-result --verbose

# ④ twist_mux のテストが更新されて通る
python3 -m pytest src/th_testing/test/test_twist_mux_priority.py -v

# ⑤ 故障注入 1 / 2 / 11
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R "fault_injection_(01|02|11)"

# ⑥ アサーション
python3 -m th_params.export --registry src/th_params/config/registry.yaml \
  --out /tmp/gen --stage 2 --nodes obstacle_limiter; echo "exit=$?"   # 0（A2a/A2b/A11 が通る）

# ⑦ 実機（電源断）。V4。hz は Ctrl-C まで戻らないので timeout で包む
timeout 6 ros2 topic hz /cmd_vel                | grep -qE "average rate: 19\.|20\."
timeout 6 ros2 topic hz /safety/limiter_status  | grep -qE "average rate: 19\.|20\."
#   停止指令中は PASS（障害物が無い状態で ZERO_STALE / BLOCKED_* にならないこと）
test "$(timeout 3 ros2 topic echo /safety/limiter_status --field action --once)" = "PASS"
```

### 11. 既知の負債・未確定 (c)

| 項目 | 扱い |
| --- | --- |
| `brake_accel_mps2` | `WP-MEAS-01` が入っていないと `v_allow` が計算できない。**A8 が起動を止める**（正しい） |
| `blind_angle_ranges` | `WP-CALIB-01` が入っていないと `BLOCKED_UNCALIBRATED` で自律が始まらない（正しい） |
| 追従対象の除外 | **後段では原理的にできない。**二層の上側（挙動ノード）の担当 |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | **`WP-SAFE-02`（`O-1`）／ `WP-CALIB-01`（`O-4`）／ `WP-PARAM-02` ／ `WP-MEAS-01`** |
| 被依存 WP | `WP-SAFE-04` / `WP-TEST-01` / `WP-TRANSIT-*` 以降すべて |

---

## `WP-SAFE-04` `jog_gate` 新設

### 0. 一行要旨

**UI を差し替えても手動ジョグが通らないように構造的に塞ぐ。**
**通さないときは沈黙する。ゼロを撃たない。**

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `th_safety/src/jog_gate.cpp` | UI の非活性化（表示。`WP-UI-03`） |
| **WebUI の publish 先を `/cmd_vel_manual_raw` に変える**（`O-6`。同一パケット） | 走行タブそのもの（`WP-UI-03`。段階 3） |
| `attributes.yaml` の `jog` 列と除外表の解釈 | `th_state` の `jog_allowed` ガード（`WP-STATE-01` で入っている） |

**`O-6`: WebUI の publish 先変更と同一パケットで行う。**
分けると旧 UI と `jog_gate` が同じ `/cmd_vel_manual` に publish し、
**手動ジョグが断続的に潰される。**

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §3.4.1 | **`jog_gate` の仕様表と「ゼロを撃たせてはいけない」理由（最重要）** |
| [safety](DetailedDesign-safety.md) §3.4.2 | 入力の同期（`jog_gate` にも「未受信＝stale」を課す） |
| [state](DetailedDesign-state.md) §4.1.1 | **`jog_allowed` が false になる条件（除外表）** |
| [state](DetailedDesign-state.md) §5・§5.1 | ジョグの 3 層。**権威は挙動ノードの publish 停止** |
| [state](DetailedDesign-state.md) §8.1・§8.2 | `attributes.yaml` の `jog` 列（`allowed` / `denied` / `is_drive`） |
| [names](DetailedDesign-names.md) §6.1 | `/cmd_vel_manual_raw` → `jog_gate` → `/cmd_vel_manual` |
| [webui](DetailedDesign-webui.md) §5 | UI 側の送出（`/cmd_vel_manual_raw` へ 10 Hz） |

### 3. インターフェース契約

#### 3.1 トピック

| 方向 | トピック | 型 | QoS | レート |
| --- | --- | --- | --- | --- |
| sub | **`/cmd_vel_manual_raw`** | `Twist` | reliable, depth 1 | 10 Hz（UI） |
| sub | `/system/state` | `SystemState` | transient_local, depth 1 | 10 Hz |
| **pub** | **`/cmd_vel_manual`** | `Twist` | reliable, depth 1 | **入力駆動。通さないときは沈黙** |

**固定レートで撃たない。**`/cmd_vel_manual_raw` を受けたときだけ出す。

#### 3.2 サービス

なし。

#### 3.3 パラメータ

| 名前 | 単位 | class | status | 備考 |
| --- | --- | --- | --- | --- |
| `state_stale_ms` | ms | b | derived | `/system/state` の途絶判定 |

**`attributes.yaml` を直接読む**（`th_state` と同じファイル。判定を二重に持たない）。

#### 3.4 フレーム

なし。

### 4. 内部設計

#### 4.1 純粋コア

```cpp
// include/th_safety/jog_gate_core.hpp
bool jog_passes(const StateView& st, double state_age_ms,
                const Attributes& attrs, const P& p);
```

**判定は 3 つの AND。**

| # | 条件 |
| --- | --- |
| 1 | `state_age_ms <= state_stale_ms`（**未受信は不合格**） |
| 2 | `attrs[st.mode].jog != "denied"` |
| 3 | `(st.mode, st.state)` が除外表に当たらない（`SUMMON/WAIT_CLEAR`） |

**`jog == "is_drive"`（`MANUAL` / `TEACH_MANUAL`）は通す。**スティックが走行操作そのもの。

#### 4.2 ノードの責務

`/cmd_vel_manual_raw` のコールバックで `jog_passes()` を呼び、真なら**そのまま転送**、偽なら**何もしない**。

#### 4.3 不変条件

| # | 不変条件 | なぜ |
| --- | --- | --- |
| **J-1** | **通さないときは publish しない（沈黙する）。ゼロを撃たない** | `/cmd_vel_manual` は priority 30。ゼロを撃ち続けると `manual_joy` が常に非タイムアウトになり、**twist_mux が priority 20/10 を永久に選ばない**＝**自律走行と Nav2 が構造的に一切出力されない** |
| **J-2** | `/system/state` 途絶 → **沈黙** | 安全側。ゼロではない |
| **J-3** | 速度の大きさは変えない（**ゲートであってリミッタではない**） | 上限のクランプは `obstacle_limiter` の仕事 |
| **J-4** | `th_state` の `jog_allowed` と**同じ `attributes.yaml`** を読む | 判定を二重に持たない |

> **「沈黙禁止」を課すのは多重化の後段だけ**（`obstacle_limiter` と `esp32_bridge`）。
> 前段のノードは**止めるときは黙る。**止めるのは twist_mux のタイムアウトの役目。

### 5. 表駆動データ

`attributes.yaml` の `jog` 列（18 行）＋ [state](DetailedDesign-state.md) §4.1.1 の除外表。

### 6. 安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **多重化の前段。**層 3 には入らない |
| 6.2 フェイルセーフ既定 | **沈黙**（J-2） |
| 6.3 FMEA | ① **ゼロを 20 Hz で撃つ → 自律走行が構造的に一切出力されない**（J-1。設計初版の致命的な穴）。② `jog_gate` が落ちる → 手動ジョグが効かなくなる。**危険側ではない**（走らなくなるだけ）。③ `is_drive` を `denied` と同じに扱う → **`MANUAL` で走れない** |

### 7. 単体試験

| テスト | ctest 登録名（`V6`） | 満たす仕様 |
| --- | --- | --- |
| `test_jog_gate_core`（gtest）`SilentWhenBlocked` | `test_jog_gate_core`（`th_safety`） | **J-1。publish 回数が 0** |
| `test_jog_gate_core::SilentWhenStateStale` | 同上 | J-2 |
| `test_jog_gate_core::PassthroughUnchanged` | 同上 | J-3 |
| `test_jog_gate_core::IsDrivePasses` | 同上 | `MANUAL` / `TEACH_MANUAL` |
| `test_jog_gate_core::WaitClearBlocked` | 同上 | `F-28` |
| `test_jog_gate_core::AllModesFromAttributes` | 同上 | **18 モードを attributes から回す** |
| **`test_jog_gate_node.py::test_mux_can_select_priority_20`** | `jog_gate_node`（`th_testing`） | **J-1 の帰結。`jog_gate` が黙っている間に `/cmd_vel_behavior` が `/cmd_vel_muxed` に出る** |

```cmake
# src/th_safety/CMakeLists.txt
ament_add_gtest(test_jog_gate_core test/test_jog_gate_core.cpp)
target_link_libraries(test_jog_gate_core jog_gate_core)
# src/th_testing/CMakeLists.txt
ament_add_pytest_test(jog_gate_node test/test_jog_gate_node.py)
```

**§10 ① の `-R jog_gate` は両方の前方一致**（`test_jog_gate_core` / `jog_gate_node`）。

### 8. Gazebo シナリオ

`gazebo.launch.py stage:=2`。`SUMMON/WAIT_CLEAR` は段階 6 まで作らないので、
**`OPCHECK` / `CALIB` / `IDLE` で塞がることを確認する。**

### 9. 実機での確認手順

| 電源断でできる | 通電が要る |
| --- | --- |
| **全部**（`/cmd_vel_manual` の publish 有無をトピックで確認） | なし |

```bash
# IDLE でスティックを倒しても /cmd_vel_manual が出ない
ros2 topic pub -r 10 /cmd_vel_manual_raw geometry_msgs/Twist "{linear: {x: 0.2}}" & PUB=$!
timeout 5 ros2 topic hz /cmd_vel_manual 2>&1 | grep -q "no new messages"   # V4・V7
kill -TERM $PUB
```

### 10. 完了条件

```bash
cd /root/th_ws && colcon build --symlink-install --packages-select th_safety

# ① 沈黙すること（J-1。最重要）
colcon test --packages-select th_safety --event-handlers console_direct+ \
  --ctest-args -R jog_gate
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R jog_gate

# ② WebUI の publish 先が変わっている（O-6）
grep -rn "cmd_vel_manual_raw" th_ws/web_ui/src/ros/topics.js
! grep -rn "'/cmd_vel_manual'" th_ws/web_ui/src/     # 旧トピックへの直接 publish が無い

# ③ 実機（電源断）で IDLE のジョグが構造的にゼロ。V4・V7
ros2 topic pub -r 10 /cmd_vel_manual_raw geometry_msgs/Twist "{linear: {x: 0.2}}" & PUB=$!
sleep 2; test "$(timeout 3 ros2 topic echo /cmd_vel --field linear.x --once)" = "0.0"
kill -TERM $PUB

# ④ twist_mux が priority 20 を選べる（J-1 の帰結）
ros2 topic pub -r 20 /cmd_vel_behavior geometry_msgs/Twist "{linear: {x: 0.1}}" & PUB=$!
sleep 2; test "$(timeout 3 ros2 topic echo /cmd_vel_muxed --field linear.x --once)" = "0.1"
kill -TERM $PUB
```

### 11. 既知の負債・未確定 (c)

なし。

### 12. 依存

| | |
| --- | --- |
| 依存 WP | `WP-SAFE-03` / `WP-STATE-02` |
| 被依存 WP | `WP-TRANSIT-01`（`MANUAL`）／ `WP-ONSITE-03`（`WAIT_CLEAR`） |

---

## `WP-TEST-01` 故障注入 13 項目の自動化

### 0. 一行要旨

**「接触 0 件」は数えて確かめるものではない。意図的に壊して止まることを確かめる。**

### 1. 対象と非対象

| 作る | 作らない |
| --- | --- |
| `th_testing/test/fault_injection/` の 13 本 | 新しい安全機能（すべて既存パケットの成果物） |
| `colcon test --ctest-args -R fault_injection` で回る形 | 実機専用 3 本（4・8・10）の自動化 |

**[safety](DetailedDesign-safety.md) §10 の 13 行が仕様。**

| # | 自動化の形 | 備考 |
| --- | --- | --- |
| 1・2・7・9・13 | **Gazebo** | 7 と 13 は段階 6・5 で有効化（`O-7` と同じ考え方） |
| 3・5・6・11 | **Gazebo ＋ 実機** | |
| **12** | **統合テスト ＋ 実機**（Gazebo ではない） | **`esp32_bridge` は Gazebo で起動しない。**`launch_testing` で `esp32_bridge` ＋ WS モックサーバーを立てる |
| 4・8・10 | **実機のみ**（通電。人が関与） | 手順書として `docs/` に残す |

### 2. 参照する設計書の節

| 節 | 何のために |
| --- | --- |
| [safety](DetailedDesign-safety.md) §10 | **13 行の表（方法と合格条件）** |
| [safety](DetailedDesign-safety.md) §10 の注 | **5 と 6 を分ける理由**（`F-21`。1 行にすると正常な実装でも必ず不合格） |
| [packets](DetailedDesign-packets.md) §12・§12.1 | 検証の手段・電源断／通電の分離 |
| [reuse](DetailedDesign-reuse.md) §2.11 | `th_testing` の既存の置き場 |

### 3. インターフェース契約

**新しいトピック・サービス・パラメータを作らない。**既存の観測点だけを使う。

| 観測点 | 何を見るか |
| --- | --- |
| `/cmd_vel` | 駆動指令がゼロになるか |
| `/safety/fault` | `fault_type` と `severity` |
| `/safety/fault_lock` / `/safety/estop` | ロックが立つか |
| `/system/state` | `mode` が `ESTOP` / `PAUSE` になるか |
| `/esp32/wheel_cmd_speed` | ESP32 へ送られた指令（12 番） |

### 4. 内部設計

#### 4.1 共通の骨格

```python
# fault_injection/conftest.py
@pytest.fixture
def sim_stack(request): ...          # gazebo.launch.py を stage 指定で起動

def assert_stops_within(topic, field, ms): ...   # 時間つきの合格判定
```

**5 と 6 は別のアサーションにする。**

```python
def test_05_link_loss_raises_fault(sim_stack):
    kill_lidar()
    assert_fault_within("LIDAR_LOST", ms=params["lidar_timeout_ms"])

def test_06_fault_to_stop_100ms(sim_stack):
    t_fault = wait_fault("LIDAR_LOST")
    assert_zero_within("/cmd_vel", since=t_fault, ms=100)    # ★ 層 3 の応答時間
```

#### 4.3 不変条件

| # | 不変条件 |
| --- | --- |
| **T-1** | **合格条件の時間はパラメータから引く。**テストに数値を書かない（`R2`） |
| **T-2** | 5 と 6 を 1 本にしない（`F-21`） |
| **T-3** | 実機専用の 3 本は**スキップではなく「手順書へのリンク」として明示的に落とす**（黙って通さない） |

### 5〜6. 表駆動データ・安全要件

| 項目 | 内容 |
| --- | --- |
| 6.1 触れる層 | **全層を壊す。**実機で回すときは人が張り付く |
| 6.3 FMEA | ① テストが誤って通る（観測点が間違っている） → **各テストに「壊さない場合は失敗すること」の対照ケースを付ける。**② SIGKILL を繰り返して DDS discovery を壊す（`CLAUDE.md` の環境の癖） → **`kill -TERM` を既定にし、`SIGKILL` が要る 11・13 は毎回コンテナを作り直す** |

### 7〜9. 単体試験・Gazebo・実機

このパケット自体がテストである。

**ctest 登録名（`V6`）。**1 ファイル 1 登録で、名前は `fault_injection_<2 桁>` に固定する。
**§10 の `-R` はすべてこの命名の前方一致・正規表現である。**

| ファイル | ctest 登録名 |
| --- | --- |
| `fault_injection/test_01_*.py` 〜 `test_13_*.py` | `fault_injection_01` 〜 `fault_injection_13` |
| `fault_injection/test_control.py`（対照ケース。§6.3 ①） | `fault_injection_control` |

```cmake
# src/th_testing/CMakeLists.txt — 13 本＋対照を明示列挙する（glob にしない）
foreach(n 01 02 03 04 05 06 07 08 09 10 11 12 13)
  file(GLOB _f "test/fault_injection/test_${n}_*.py")
  ament_add_pytest_test(fault_injection_${n} "${_f}")
endforeach()
ament_add_pytest_test(fault_injection_control test/fault_injection/test_control.py)
```

### 10. 完了条件

```bash
cd /root/th_ws

# ① 13 本のうち Gazebo/統合で回るもの（10 本）が全部通る
colcon test --packages-select th_testing --event-handlers console_direct+ \
  --ctest-args -R fault_injection
colcon test-result --verbose

# ② 本数の確認（V9）。対照ケース test_control.py は別に数える
test "$(ls src/th_testing/test/fault_injection/test_[0-9][0-9]_*.py | wc -l)" -eq 13
test -f src/th_testing/test/fault_injection/test_control.py
# ②' 13 本＋対照が ctest に登録されている（V6。0 件で合格するのを防ぐ）
test "$(ctest --test-dir build/th_testing -N -R fault_injection \
        | grep -c 'Test *#')" -eq 14

# ③ 実機専用 3 本が「スキップ」ではなく明示的に落ちる（T-3）
colcon test --packages-select th_testing --ctest-args -R "fault_injection_(04|08|10)"
#   → 「実機で手順書 docs/... を実行すること」と出て fail する

# ④ 対照ケース（壊さなければ止まらない）
colcon test --packages-select th_testing --ctest-args -R fault_injection_control

# ⑤ 数値がテストに直書きされていない（T-1）
! grep -rnE "assert.*\b(100|600|2000)\b" src/th_testing/test/fault_injection/
```

### 11. 既知の負債・未確定 (c)

| 項目 | 扱い |
| --- | --- |
| 7（退避待ち）・13（自己位置喪失） | **段階 6・5 の機能に依存する。**段階 2 では skip マーク付きで置き、該当段階で有効化する |
| 4・8・10 | **実機・通電・人が関与。**自動化しない |

### 12. 依存

| | |
| --- | --- |
| 依存 WP | **段階 2 の他 6 パケットすべて** |
| 被依存 WP | 各段階の出口判定（回帰試験として回し続ける） |

---

## 段階 2 の出口チェック

```bash
cd /root/th_ws && colcon build --symlink-install
colcon test --packages-select th_safety th_testing --event-handlers console_direct+
colcon test-result --verbose
colcon test --packages-select th_testing --ctest-args -R fault_injection

# 実機（モータ電源断）
ros2 launch th_bringup bringup.launch.py stage:=2
ros2 topic info /cmd_vel --verbose | grep "Publisher count"    # 1（obstacle_limiter のみ）
timeout 6 ros2 topic hz /cmd_vel /safety/limiter_status        # ともに 20 Hz。V4
```

| 出口条件 | 判定 |
| --- | --- |
| 故障注入 13 項目が自動で回る（実機専用 3 本を除く 10 本が通る） | `WP-TEST-01` §10-① |
| 実機（電源断）で `/cmd_vel` がゼロになる | `WP-SAFE-02` / `WP-SAFE-04` §10 |
| **`DEBT-2`（死角）が解除** | `blind_angle_ranges` が `measured`・幅ゼロ 0 件 |
| **`DEBT-3`（タイムアウト）が解除** | `generated/safety_monitor.yaml` が導出値 |
| **`DEBT-4`（キープアライブ）が解除** | 故障注入 12 |
| **`DEBT-1` が重大フォルトとして扱われる**（検出の口は段階 0 の `WP-ESP32-01`。解除は段階 7） | `/safety/firmware_flags` → `severity: CRITICAL` |
| 自律走行の指令が出力できる（`jog_gate` が黙っている） | `WP-SAFE-04` §10-④ |
