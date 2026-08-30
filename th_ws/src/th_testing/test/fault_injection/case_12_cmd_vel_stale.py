"""
case_12_cmd_vel_stale.py — 故障注入 12「`/cmd_vel` の途絶」（ctest 登録名: fault_injection_12）
================================================================================================
`DetailedDesign-safety.md` §10 #12: `obstacle_limiter` を止めたまま非ゼロ指令を残し、
`cmd_vel_stale_ms` 以内に ESP32 への指令（`/esp32/wheel_cmd_speed` と、その対
ESP32 送信用の WHEEL_CMD フレーム）がゼロになることを確認する。
自動化: **統合テスト ＋ 実機（Gazebo ではない）**。

──────────────────────────────────────────────
【なぜ `sim_stack`（Gazebo）を使わないか】
──────────────────────────────────────────────
`esp32_bridge` は `gazebo.launch.py` の `condition=UnlessCondition(sim)` により
**sim では起動しない**。そしてこの検証の対象はまさにその `esp32_bridge` の
stale 判定（`keepalive_core.py` の `keepalive_value()` ＋ `esp32_bridge.py` の
`_send_wheel_cmd()`）である。測定対象である `esp32_bridge` 本体が sim に
存在しないため、この項目だけ `launch_testing` で `esp32_bridge` ＋
WS モックサーバーを直接立てる。

構造は既存の `test_esp32_bridge_node.py`（WP-SAFE-02 の統合テスト）をそのまま
引き継ぐ: `generate_test_description()` で `esp32_bridge.py` を起動し、
`_WsMockEsp32`（esp32_bridge の WS サーバーへ **ESP32 の代わり**に接続し、
受信した WHEEL_CMD フレームを記録するクライアント）で送信内容を観測する。
`_WsMockEsp32` はテスト間の結合を増やさないため import ではなくコピーにしてある
（唯一の差分: 記録時刻を `time.time()` から `time.monotonic()` へ。後述の
実測レイテンシを `t_last_cmd` と同じ時計軸で測るため）。
WS ポートは `test_esp32_bridge_node.py`(18765)・実機/既定(8766) と衝突しない
**18766** を使う。

──────────────────────────────────────────────
【このテストが「何でないか」—— 偽陰性を絶対に許さない3点】
──────────────────────────────────────────────
1. **独立ロック層（K-2）を解除し続ける。** `esp32_bridge` は
   `/safety/estop` `/safety/fault_lock` を一度も受信していない間（このテストに
   `safety_monitor` は居ない）をフェイルセーフで「ロック中」として扱い、
   `/cmd_vel` の内容によらず**常にゼロを送る**。これを怠ると stale 判定を
   一切検証していないのに合格してしまう（偽陰性）。テストノードが
   `Bool(data=False)` を**両トピックへ publish し続ける**ことで解除状態を作る
   （`test_esp32_bridge_node.py` と同じ手法。safety_monitor の代わりを務める）。
2. **途絶させる前に、非ゼロの WHEEL_CMD が実際に届いたことを証明する。**
   「ゼロになった」ことを主張する前に、非ゼロフレームの受信実績が無ければ、
   「最初から常にゼロを送る壊れた実装」でも見かけ上合格してしまう。
   その場合は「沈黙」ではなく**判定不能**として明示的に落とす
   （このリポジトリで繰り返し効いている型。`conftest.py` の
   `assert_silent_within` / `assert_no_nonzero_after` が `since` 以前の
   受信歴を要求するガードと同じ発想）。
3. **沈黙とゼロを区別する（K-1）。** `/cmd_vel` が途絶しても、キープアライブの
   20Hz 送信そのものは止めず「ゼロを送る」ことが仕様
   （`keepalive_core.py` 冒頭 docstring・`WP-SAFE-02 §4.3`）。送るのをやめると
   停止は ESP32 ウォッチドッグ（`esp32_watchdog_ms`=600ms）経由になり、
   PC 側の対処（`cmd_vel_stale_ms`=400ms）より遅れてしまうため。途絶後も
   フレームが届き続けていることと、その内容がゼロであることの両方を別途
   アサートする。

──────────────────────────────────────────────
【合格条件の時間（規約 T-1）と判定しきい値】
──────────────────────────────────────────────
合格条件 `cmd_vel_stale_ms` はテストに直書きしない。`registry.yaml`（正本）を
`th_params.export.resolve_registry()` で解決し、モジュールロード時に `_STALE_MS`
へ持つ（**launch_testing のテストは `unittest.TestCase` なので pytest フィクスチャを
受け取れない**。そのため `fault_params` フィクスチャの代わりに、同じ解決ロジック
`conftest.py::_registry_rows()` / `_resolved()` をモジュールレベルで直接呼ぶ）。
`status == 'placeholder'` のまま解決できない場合は、`_resolved()` が
`pytest.fail` を投げて**黙って既定値に落とさず明示的に失敗**させる。
解決値は `esp32_bridge` ノードの `cmd_vel_stale_ms` パラメータとして渡すので、
判定と実装が自己整合する（同じ値でテストする）。

「`cmd_vel_stale_ms` 以内にゼロ」を厳密に `<= 400ms` と解釈すると**正しい
実装でも必ず落ちる**。判定しきい値 `_LATENCY_BUDGET_MS` は
`cmd_vel_stale_ms + キープアライブ2周期`（registry 既定値 400ms なら
`400 + 50*2 = 500ms`）とする。2 周期は役割が異なる:

- **1 周期目＝原理的な量子化。** `esp32_bridge` のキープアライブは 20Hz の
  タイマー（`esp32_bridge.py` の
  `self.create_timer(0.05, self._cb_cmd_vel_keepalive)`）で
  `keepalive_value(last_cmd_ms, now_ms, …, stale_ms, locked)` を評価し、
  `now - last_cmd > stale_ms` となった**次のタイマー周期**で初めてゼロを送る。
  よってゼロ化の実測値は原理的に `(cmd_vel_stale_ms, cmd_vel_stale_ms + 50ms]`
  に量子化される。この分は`実装が正しくても必ず観測に現れる`ため、予算に
  必ず含める。
- **2 周期目＝計測系の裕度。** `t_last_cmd` はテスト側が `/cmd_vel` を
  publish した時刻（`time.monotonic()`）であって、`esp32_bridge` が受信した
  時刻ではない。DDS 配送遅延・送信コアレッシング（D-1。`esp32_bridge.py` の
  `coalescer_offer`: in-flight 中は保留フレームを上書きし、次の送信機会まで
  送れない）・WS 往復・モック側の記録時刻のずれが上乗せされる。この分には
  原理的な上限が無いため、**実測に対して** 1 周期分の裕度を置く。

実測（Docker・健全時 3 連続、2026-08-30）: **441.6ms / 405.5ms / 415.8ms**。
1 周期のみの予算（450ms）では、最大 441.6ms に対して残り 8.4ms しか無く、
しかも実測値は `(cmd_vel_stale_ms, cmd_vel_stale_ms + 50ms]` にほぼ一様に
散らばるため、上端付近で DDS 配送遅延が数 ms 乗れば 450ms を超える
境界フレークになる（実際に 3 連続とも合格したが最悪ケースの余裕は 8.4ms）。
2 周期（500ms）なら最大実測に対して約 58ms の余裕が取れる。後任が「なぜ
2 周期なのか」を再現できるよう、この実測 3 値を根拠として記録しておく。

さらに**判定予算と ESP32 ウォッチドッグ上限は別物として扱う**:
- 実測が `_LATENCY_BUDGET_MS` を超える = **判定予算違反**（設計上の期待から
  の逸脱。安全連鎖が壊れたとは限らないので、失敗は予算側のメッセージで
  報告する）。
- 実測が `esp32_watchdog_ms`（registry 正本から解決。既定 600ms）に**達して
  しまう** = 「PC 側の対処が ESP32 のウォッチドッグより遅れる」＝ 規約 A7
  （`cmd_vel_stale_ms < esp32_watchdog_ms`）が意図した順序が実際には成立して
  いない、**設計の前提そのものの破綻**。こちらは予算とは**別のアサーション**
  （`measured_ms < esp32_watchdog_ms`）で機械的に検出し、失敗メッセージも
  別に書き分ける。加えてモジュールロード時に `_LATENCY_BUDGET_MS <
  esp32_watchdog_ms` を恒久チェックし、将来予算を広げすぎて安全側の条件を
  黙って飲み込もうとしたらその場で明示的に失敗させる。

──────────────────────────────────────────────
【実測レイテンシの測り方】
──────────────────────────────────────────────
- `t_last_cmd`: 最後に `/cmd_vel` を publish した時刻（`time.monotonic()`）。
  esp32_bridge 側の `_last_cmd_vel_time` をこの時刻に概ね揃えるため、
  連続 publish の**最後の 1 発の直後**に採る。
- 途絶後も spin を続け、`t >= t_last_cmd` かつ left/right がともに
  `|値| <= 1e-3`（`_ZERO_ATOL`）である最初の WHEEL_CMD の受信時刻 `t_zero`
  を求める。`t >= t_last_cmd` を課すのは、途絶前の過渡期（起動直後・最初の
  publish 前など）に届いたゼロフレームを拾わないため。
- `(t_zero - t_last_cmd)` をミリ秒に直し、**アサーションメッセージに実測値を
  必ず含める**（合否に関わらず値が読み取れるようにする）。
"""
from __future__ import annotations

import os
import sys
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ── パス解決（self-contained。conftest からのインポート順に依存しない） ─────
_TEST_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
_SRC_ROOT = os.path.abspath(os.path.join(_TEST_ROOT, '..', '..'))
_WS_PROTOCOL_DIR = os.path.join(_SRC_ROOT, 'th_esp32_bridge', 'th_esp32_bridge')
if _WS_PROTOCOL_DIR not in sys.path:
    sys.path.insert(0, _WS_PROTOCOL_DIR)

from ws_protocol import WHEEL_CMD, peek_type, unpack_wheel_cmd  # noqa: E402
from fault_injection.conftest import _registry_rows, _resolved  # noqa: E402
from th_params import export as params_export  # noqa: E402


# ── T-1: 合格条件の時間は registry.yaml からモジュールロード時に解決する ────
# unittest.TestCase（launch_testing）は pytest フィクスチャを受け取れないため、
# fault_params と同じ解決ロジック（conftest.py::_registry_rows/_resolved）を
# 直接呼ぶ。status=='placeholder' のまま解決できない場合は _resolved() が
# pytest.fail を投げて明示的に落とす（黙って既定値に落とさない）。
def _resolve_registry_params() -> tuple[int, int]:
    """registry.yaml 正本（T-1）から `cmd_vel_stale_ms` と `esp32_watchdog_ms`
    を**1 回の `resolve_registry()`** で取り出す（同じ registry を 2 回解決
    しない。`esp32_watchdog_ms` は下の `_ESP32_WATCHDOG_MS` と恒久ガード、
    および実測アサーションに使う）。

    どちらも status=='placeholder' のまま解決できない場合は `_resolved()` が
    `pytest.fail` を投げて黙って既定値に落とさず明示的に失敗させる。
    """
    resolved = params_export.resolve_registry(_registry_rows())
    return (_resolved(resolved, 'cmd_vel_stale_ms'),
            _resolved(resolved, 'esp32_watchdog_ms'))


# 実機・他テストの esp32_bridge（既定 8766）や test_esp32_bridge_node.py
# （18765）と衝突しないポートを使う。
_WS_PORT = 18766

# esp32_bridge のキープアライブ周期 [ms]。esp32_bridge.py の
# `self.create_timer(0.05, self._cb_cmd_vel_keepalive)`（20Hz）が由来。
# stale 判定（keepalive_value の `now - last_cmd > stale_ms`）はこの周期の
# タイマーでしか評価されないため、ゼロ化の観測は「判定は stale_ms ちょうどで
# 下りたのに送信は次の周期まで待つ」という量子化を受ける（下参照）。
_KEEPALIVE_PERIOD_MS = 50

# 「ゼロ」と「非ゼロ」を切り分けるしきい値 [m/s]。ESP32 への指令 speed は
# 1e-3 を超えると物理的に車輪が動き得る。`assertAlmostEqual(places=3)`
# （|差| <= 1e-3）と同じ桁で統一する。
_ZERO_ATOL = 1e-3

# T-1: 合格条件の時間（cmd_vel_stale_ms）と安全上の絶対条件
# （esp32_watchdog_ms）を registry.yaml 正本から一度に持ち出す。前者は
# esp32_bridge ノードの cmd_vel_stale_ms パラメータにも同じ値を渡すので、
# 判定と実装が自己整合する。後者は実測アサーションとモジュールロード時の
# 恒久ガード（下参照）に使う。
_STALE_MS, _ESP32_WATCHDOG_MS = _resolve_registry_params()

# 判定しきい値 = cmd_vel_stale_ms + キープアライブ2周期（registry 既定値
# 400ms なら 500ms）。2 周期は根拠が異なる
# （モジュール docstring「合格条件の時間（規約 T-1）と判定しきい値」参照）:
#   - 1 周期目 = 原理的な量子化。stale 判定（keepalive_value の
#     `now - last_cmd > stale_ms`）は 20Hz タイマー（50ms 周期）でしか
#     評価されず、ゼロ化は (cmd_vel_stale_ms, cmd_vel_stale_ms + 50ms] に
#     必ず量子化される。実装が正しくても観測に現れるので予算に含める。
#   - 2 周期目 = 計測系の裕度。t_last_cmd はテスト側が /cmd_vel を publish
#     した時刻であって esp32_bridge の受信時刻ではない。DDS 配送遅延・
#     送信コアレッシング（D-1: coalescer_offer）・WS 往復・モック側の記録
#     時刻のずれが上乗せされる。原理的な上限が無いため、実測最大 441.6ms
#     に対して 1 周期分を置く。
# この予算は「設計上の期待」であり、安全上の絶対条件（実測 < ESP32
# ウォッチドッグ、A7）は _ESP32_WATCHDOG_MS の別アサーションが検証する。
_LATENCY_BUDGET_MS = _STALE_MS + _KEEPALIVE_PERIOD_MS * 2

# 安全上の絶対条件（A7）: ESP32 のローカルウォッチドッグ。registry.yaml 正本
# （esp32_watchdog_ms。既定 600ms は esp32/src/config.h の WATCHDOG_MS の写し
# であり、registry の source 注記にも「ファーム側が正」とある）。ハードコード
# せず上記 _resolve_registry_params() の 1 回の解決から持ち出す。
# これが破れる＝「PC 側の対処（cmd_vel_stale_ms でのゼロ化）が ESP32 の
# ウォッチドッグより遅れる」＝ A7（cmd_vel_stale_ms < esp32_watchdog_ms）が
# 意図した順序が実際には成立していない、**設計の前提そのものの破綻**。
# 判定予算（設計上の期待）とウォッチドッグ上限（安全上の絶対条件）の間に
# 順序を保証する恒久ガード。モジュールロード時（テスト収集時）に
# `_LATENCY_BUDGET_MS < _ESP32_WATCHDOG_MS` が成り立つことを確認し、成り
# 立たなければ**その場で明示的に失敗**させる。将来どちらかを広げて安全側の
# 条件を黙って飲み込んでしまわないための防護で、失敗は pytest の収集エラー
# として見える。
if not (_LATENCY_BUDGET_MS < _ESP32_WATCHDOG_MS):
    raise AssertionError(
        f'判定予算 _LATENCY_BUDGET_MS={_LATENCY_BUDGET_MS}ms が安全絶対条件'
        f' _ESP32_WATCHDOG_MS={_ESP32_WATCHDOG_MS}ms 以上。A7'
        f' (cmd_vel_stale_ms < esp32_watchdog_ms) が意図した順序が成立し得ない。'
        f' 予算を広げる前に安全設計の前提'
        f' (docs/plan/detailed/DetailedDesign-safety.md) を再確認すること。')


@pytest.mark.launch_test
def generate_test_description():
    esp32_bridge = launch_ros.actions.Node(
        package='th_esp32_bridge',
        executable='esp32_bridge.py',
        name='esp32_bridge',
        parameters=[{
            'ws_host': '127.0.0.1',
            'ws_port': _WS_PORT,
            # T-1: 合格条件と同じ値を実装側へ渡す（テストが自己整合する）。
            'cmd_vel_stale_ms': _STALE_MS,
            # フィードバック死活監視のタイムアウト警告でログが汚れないよう緩める
            # (WHEEL_FEEDBACK を送るモックではないため常に鳴るが、テストの
            # 合否には関与しない)。
            'feedback_timeout_ms': 60000,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        esp32_bridge,
        launch_testing.actions.ReadyToTest(),
    ]), {'esp32_bridge': esp32_bridge}


class _WsMockEsp32:
    """esp32_bridge の WS サーバーへ ESP32 の代わりに接続し、受信した
    WHEEL_CMD フレームを記録するモッククライアント。

    `test_esp32_bridge_node.py` の同名クラスからのコピー（結合を増やさない
    ため import にしない）。唯一の差分は記録時刻を `time.time()` から
    `time.monotonic()` へ変えたこと——case_12 は `t_zero - t_last_cmd` の
    実測レイテンシを測るため、`t_last_cmd`（`time.monotonic()`）と同じ
    時計軸で受信時刻を記録する必要がある。

    接続は別スレッドの asyncio イベントループで非同期に確立する
    (esp32_bridge 自身の実装と同じ構造)。呼び出し側は wait_connected() で
    接続完了を明示的に待ってから publish を始めること — 待たずに
    publish を始めると、接続がまだ確立していない間のフレームを
    取りこぼして偽陰性になる (レース条件)。
    """

    def __init__(self, uri: str):
        self._uri = uri
        self.frames: "list[tuple[float, float, float]]" = []  # (recv_time, left, right)
        self._lock = threading.Lock()
        self._loop = None
        self._ws = None
        self._connected = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def wait_connected(self, timeout: float):
        """接続確立を待つバリア。タイムアウトすれば理由が分かる形で失敗させる。"""
        if not self._connected.wait(timeout):
            raise AssertionError(
                f'ESP32 モッククライアントが {timeout}s 以内に esp32_bridge の '
                f'WS サーバー ({self._uri}) へ接続できなかった')

    def stop(self):
        """接続を明示的に閉じる。次のテストの接続とすれ違わせないため
        tearDown で必ず呼ぶ (呼ばずに放置すると、ノード終了時に
        ConnectionClosedError が別スレッドで飛んでログが汚れる)。"""
        if self._loop is not None and self._ws is not None:
            import asyncio

            async def _close():
                await self._ws.close()
            try:
                fut = asyncio.run_coroutine_threadsafe(_close(), self._loop)
                fut.result(timeout=2.0)
            except Exception:
                pass  # 既に切断済み等 — テスト後始末なので握りつぶしてよい

    def _run(self):
        import asyncio
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._connect_and_recv())
        except Exception:
            pass  # バックグラウンドスレッドの後始末。テストの合否は wait_connected() 等で判定する

    async def _connect_and_recv(self):
        import asyncio
        import websockets

        deadline = asyncio.get_event_loop().time() + 15.0
        while asyncio.get_event_loop().time() < deadline:
            try:
                async with websockets.connect(self._uri) as ws:
                    self._ws = ws
                    self._connected.set()
                    try:
                        async for message in ws:
                            if not isinstance(message, (bytes, bytearray)):
                                continue
                            data = bytes(message)
                            if peek_type(data) == WHEEL_CMD:
                                left, right = unpack_wheel_cmd(data)
                                with self._lock:
                                    self.frames.append((time.monotonic(), left, right))
                    except websockets.exceptions.ConnectionClosed:
                        # stop() での明示的な close、またはノード終了時の
                        # 突然の切断。テスト後始末の一部として想定内。
                        pass
                return
            except (OSError, ConnectionRefusedError):
                await asyncio.sleep(0.3)

    def snapshot(self):
        with self._lock:
            return list(self.frames)

    def clear(self):
        with self._lock:
            self.frames.clear()


class TestFaultInjection12CmdVelStale(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('case_12_cmd_vel_stale')
        self.pub_cmd_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)
        # K-2: esp32_bridge の独立ロック層は /safety/estop・/safety/fault_lock を
        # 一度も受信していない間（このテストでは safety_monitor が居ない）を
        # フェイルセーフで「ロック中」として扱う。ここで false を送り続けない
        # と常時ロック扱いになり、/cmd_vel の内容によらず常にゼロになって
        # しまい、stale タイムアウトの検証にならない（偽陰性対策。モジュール
        # docstring「このテストが何でないか」の 1 参照）。
        self.pub_estop = self.node.create_publisher(Bool, '/safety/estop', 10)
        self.pub_fault_lock = self.node.create_publisher(Bool, '/safety/fault_lock', 10)

        self.client = _WsMockEsp32(f'ws://127.0.0.1:{_WS_PORT}')
        self.client.start()
        # ① 接続確立そのものを待つ (publish を接続前に始めるとフレームを
        #    取りこぼして偽陰性になる)。
        self.client.wait_connected(15.0)
        # ② ロック解除状態が伝わり、独立ロック層が解除されるまで待つ
        #    (lock_stale_ms=0.5s より十分長く)。接続直後はキープアライブの
        #    次周期 (最大 50ms) まで何も来ないので、最初のフレームが届く
        #    のもここで併せて待つ。
        self._spin(0.6)
        self._wait_for_frames(min_count=1, timeout=2.0)
        self.client.clear()

    def tearDown(self):
        self.client.stop()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, sec: float):
        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            # safety_monitor 相当: ロック解除状態を送り続ける (上の setUp 参照)。
            self.pub_estop.publish(Bool(data=False))
            self.pub_fault_lock.publish(Bool(data=False))
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _wait_for_frames(self, min_count: int, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if len(self.client.snapshot()) >= min_count:
                return
            self._spin(0.05)
        self.fail(f'{min_count} 件のフレームを {timeout}s 以内に受信できなかった'
                   ' (esp32_bridge の WS サーバーに接続できていない可能性)')

    def _wait_first_zero(self, t_last_cmd: float, wait_ms: float):
        """`t_last_cmd` 以降に受信した WHEEL_CMD のうち、left/right がともに
        `|値| <= _ZERO_ATOL` である**最初の**フレームの受信時刻
        （`time.monotonic()`）を返す。`wait_ms` 待っても観測できなければ None。

        `t >= t_last_cmd` を課す理由: 途絶前の過渡期（起動直後・最初の
        publish 前など）に届いたゼロフレームを「途絶後のゼロ化」として
        誤って拾わないため。途絶前に非ゼロ指令が反映されていれば、途絶後は
        stale 判定でゼロに切り替わるまで非ゼロが流れ続けるので、この条件を
        満たす最初のフレーム＝stale によるゼロ化そのものになる。
        """
        deadline = t_last_cmd + wait_ms / 1000.0
        while True:
            for t, left, right in self.client.snapshot():
                if t >= t_last_cmd and abs(left) <= _ZERO_ATOL and abs(right) <= _ZERO_ATOL:
                    return t
            if time.monotonic() >= deadline:
                return None
            self._spin(0.01)

    # ════════════════════════════════════════════════════════
    def test_stale_zeroes_wheel_cmd_within_budget(self):
        """#12 本命: `/cmd_vel` が `cmd_vel_stale_ms` 途絶したら、ESP32 への
        WHEEL_CMD（/esp32/wheel_cmd_speed と同じ値）が
        `cmd_vel_stale_ms + キープアライブ2周期` 以内にゼロ化する
        （DEBT-4 に対する WP-SAFE-02 の対処の検証）。

        さらに安全連鎖の絶対条件（A7: `measured_ms < esp32_watchdog_ms`）を
        予算とは別のアサーションで検証する（破れたら設計の前提そのものの
        破綻。根拠はモジュール docstring「合格条件の時間（規約 T-1）と
        判定しきい値」参照）。"""

        # ── 非ゼロ指令（直進 0.2 m/s）を連続で流す ──
        cmd = Twist()
        cmd.linear.x = 0.2
        for _ in range(4):
            self.pub_cmd_vel.publish(cmd)
            self._spin(0.05)
        # 最後の 1 発: esp32_bridge 側の _last_cmd_vel_time をこの時刻に概ね
        # 揃えるためのアンカー（途絶の起点）。publish 直後に t_last_cmd を採る。
        self.pub_cmd_vel.publish(cmd)
        t_last_cmd = time.monotonic()

        # ── 前提: 途絶前に非ゼロの WHEEL_CMD が実際に届いていること ──
        # これが無いと「最初から常にゼロを送る壊れた実装」でも合格してしまい、
        # 「判定不能」（ゼロ化の検証になっていない）として明示的に落とす。
        # 受信0件（ノード未起動・トピック誤り等）も同様に落とす。
        frames = self.client.snapshot()
        self.assertTrue(
            frames,
            '判定不能: 途絶前に WHEEL_CMD を1件も受信していない。'
            'esp32_bridge の WS サーバーに接続できていない可能性。')
        self.assertTrue(
            any(abs(left) > _ZERO_ATOL or abs(right) > _ZERO_ATOL
                for _, left, right in frames),
            '判定不能: 途絶前の WHEEL_CMD が全てゼロ。非ゼロ指令が一度も'
            'ESP32 側へ届いていなければ、stale でゼロ化したことの検証にならない。')

        # ── 途絶後、最初のゼロフレームの受信時刻を実測する ──
        # 観測窓は予算より 2 周期長めに取る: 「予算は超えたがウォッチドッグは
        # 超えていない」ゼロもあえて実測として拾い、上の予算判定と下の
        # ウォッチドッグ判定を切り分けられるようにするため。窓を切らないと
        # 予算違反の実測値を「観測できなかった」という別の失敗で覆い隠す。
        t_zero = self._wait_first_zero(
            t_last_cmd, wait_ms=_LATENCY_BUDGET_MS + _KEEPALIVE_PERIOD_MS * 2)
        if t_zero is None:
            self.fail(
                f'/cmd_vel 途絶後のゼロの WHEEL_CMD を観測できなかった'
                f'(待機 {_LATENCY_BUDGET_MS + _KEEPALIVE_PERIOD_MS * 2}ms / '
                f't_last_cmd={t_last_cmd:.3f})。送信そのものが止まったか、'
                f'WHEEL_CMD が届いていない（K-1 違反の疑い）。')
        measured_ms = (t_zero - t_last_cmd) * 1000.0

        # 合否に関わらず実測値を残す。「通った／落ちた」だけでは、予算に対して
        # どれだけ余裕があるのか（量子化の下端で通っているのか上端ぎりぎりなのか）
        # が分からず、境界フレークの兆候を見逃す。pytest は失敗時にしか
        # 捕捉済み標準出力を表示しないので、値を確認したいときは
        # `pytest -s` で直接実行すること。
        print(f'[fault_injection_12] /cmd_vel 途絶 → ゼロ化 実測 {measured_ms:.1f}ms '
              f'(判定予算 {_LATENCY_BUDGET_MS}ms = cmd_vel_stale_ms {_STALE_MS}ms + '
              f'キープアライブ2周期 {_KEEPALIVE_PERIOD_MS * 2}ms / '
              f'安全絶対条件 esp32_watchdog_ms {_ESP32_WATCHDOG_MS}ms)',
              flush=True)

        # 実測値を必ずメッセージに載せる（合否に関わらず読めるようにする）。
        # 判定予算は「設計上の期待」（cmd_vel_stale_ms + 量子化1周期 +
        # 計測系裕度1周期）。破れても安全連鎖が壊れたとは限らないので、
        # 失敗メッセージは予算の意味（期待からの逸脱）を明示する。
        self.assertLessEqual(
            measured_ms, _LATENCY_BUDGET_MS,
            f'判定予算違反（設計上の期待からの逸脱）: /cmd_vel 途絶 → ESP32 '
            f'指令ゼロ化が {_LATENCY_BUDGET_MS}ms 以内に完了しなかった '
            f'(実測 {measured_ms:.1f}ms / t_last_cmd={t_last_cmd:.3f} → '
            f't_zero={t_zero:.3f})。cmd_vel_stale_ms={_STALE_MS}ms に量子化'
            f'1周期(={_KEEPALIVE_PERIOD_MS}ms)＋計測系裕度1周期を見込んだ'
            f'予算で、安全絶対条件 esp32_watchdog_ms={_ESP32_WATCHDOG_MS}ms '
            f'よりは短い。実装(esp32_bridge.py の keepalive_value 評価周期)'
            f'または計測系を疑うこと。')

        # 安全連鎖の絶対条件（A7）: 予算とは独立に、実測が ESP32 ウォッチ
        # ドッグに達しないことを機械的に検証する。予算違反とは「遅いけれど
        # まだ設計の誤差の範囲」なのに対し、こちらが破れる＝「PC 側の対処
        # （cmd_vel_stale_ms でのゼロ化）が ESP32 のウォッチドッグより
        # 遅れる」＝ A7 が意図した順序（cmd_vel_stale_ms <
        # esp32_watchdog_ms）が実際には成立していない、設計の前提そのもの
        # の破綻。判定予算（設計上の期待）と別物なので、失敗メッセージも
        # 別に書き分ける。
        self.assertLess(
            measured_ms, _ESP32_WATCHDOG_MS,
            f'安全規約 A7 の破綻: ゼロ化実測 {measured_ms:.1f}ms が ESP32 '
            f'ウォッチドッグ {_ESP32_WATCHDOG_MS}ms 以上。PC 側の対処 '
            f'(cmd_vel_stale_ms={_STALE_MS}ms) がファーム側の最後手段より'
            f'遅れ、ロボットの停止を ESP32 の独立ウォッチドッグに丸投げ'
            f'している（t_last_cmd={t_last_cmd:.3f} → t_zero={t_zero:.3f}）。'
            f'安全設計の前提そのものが崩れているため、予算を広げて黙って'
            f'通すのではなく設計を再検討すること。')

    def test_still_publishes_at_20hz_when_stale(self):
        """K-1: `/cmd_vel` が stale になっても、20Hz のキープアライブ送信
        そのものは止めず「ゼロを送り続ける」ことを確認する。

        「送るのをやめてしまう」と、停止は ESP32 ウォッチドッグ
        （esp32_watchdog_ms=600ms）に賭けることになり、PC 側の対処
        （cmd_vel_stale_ms=400ms）より遅れるため、仕様は「ゼロを送る」。
        沈黙ではなくゼロ送信であることを、件数の継続（受信の継続）と
        内容がゼロ（値のゼロ化）の両方で示す。
        """
        cmd = Twist()
        cmd.linear.x = 0.2
        for _ in range(5):
            self.pub_cmd_vel.publish(cmd)
            self._spin(0.05)

        # 送信が「止まっていない」ことを測る: クリア後に最初のフレームが
        # 届くのを待ってから件数を数え始める。
        self.client.clear()
        self._wait_for_frames(min_count=1, timeout=2.0)
        count_before = len(self.client.snapshot())

        # stale_ms + 4周期 待てば、cmd_vel_stale_ms を超えて stale 判定
        # （ゼロ化）は確実に完了している。
        self._spin((_STALE_MS + _KEEPALIVE_PERIOD_MS * 4) / 1000.0)

        frames_after = self.client.snapshot()
        count_after = len(frames_after)
        self.assertGreater(
            count_after, count_before,
            'stale 判定後もキープアライブが送信され続けていること (K-1: 沈黙しない)。'
            f'待機前 {count_before} 件 → 待機後 {count_after} 件')

        # 「送り続けている内容」がゼロであることも確認する
        # （沈黙ではなくゼロ送信）。
        _, last_left, last_right = frames_after[-1]
        self.assertAlmostEqual(last_left, 0.0, places=3,
                                msg=f'stale 後の WHEEL_CMD.left が非ゼロ（ゼロを送るはず）: {last_left}')
        self.assertAlmostEqual(last_right, 0.0, places=3,
                                msg=f'stale 後の WHEEL_CMD.right が非ゼロ（ゼロを送るはず）: {last_right}')


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])