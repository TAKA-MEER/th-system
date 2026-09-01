"""
test_state_manager_node.py
============================
DetailedDesign-wp1.md `WP-STATE-02` §7 — state_manager ノードの単体試験。

state_manager.py は「薄いノード」（state_core.StateCore を呼ぶだけ）なので、
ここでは state_manager.py 自体の責務（Context 詰め・effects 配送・ラッチ・
sys.* タイマ生成・/system/state の publish・名前空間の分離）だけを検証する。
遷移表そのものの網羅性は test_transition_table.py（WP-STATE-01）の対象。

パラメータ（jog_lease_ms 等）は `generated/state_manager.yaml`（WP-PARAM-02、未着手）が
まだ無いため、launch の `parameters=[...]` で直接注入する
（DetailedDesign-wp1.md WP-STATE-02 冒頭の注記）。
"""
import ast
import importlib.util
import json
import os
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from ament_index_python.packages import get_package_prefix
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)

from std_msgs.msg import Bool, String
from th_system_msgs.msg import (ActiveScreen, FaultStatus, RouteInfo, RouteList,
                                StateEffect, StateEvent, SystemState)
from th_system_msgs.srv import SetFlag, UiTrigger


JOG_LEASE_MS = 300
LINK_WAIT_TIMEOUT_MS = 60_000   # テスト中に誤発火して他ケースを汚さないよう十分大きく取る
UI_ACTIVE_WINDOW_S = 5
SCREEN_STALE_MS = 60_000


def _load_state_manager_module():
    """scripts/state_manager.py は th_state パッケージの一部として import path に
    無い（ament_python_install_package の対象外・install(PROGRAMS) でしか入らない）ので、
    インストール済みの実体をファイルパスから直接 import する。"""
    path = os.path.join(get_package_prefix('th_state'), 'lib', 'th_state', 'state_manager.py')
    spec = importlib.util.spec_from_file_location('state_manager_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# 注意（launch_testing の挙動）: このファイルは @pytest.mark.launch_test を持つため、
# launch_testing の pytest プラグインがファイル全体を1つの pytest アイテムとして
# 収集し、その中の unittest.TestCase だけを実行する。モジュール直下に置いた
# 素の pytest 関数は収集されず黙って実行されない（実測して確認した）。
# そのため N-1 (test_no_transition_if_in_node) と N-2 (test_lease_extends_on_reject)
# も TestStateManagerNode のメソッドとして置く。


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[{
            'jog_lease_ms': JOG_LEASE_MS,
            'link_wait_timeout_ms': LINK_WAIT_TIMEOUT_MS,
            'ui_active_window_s': UI_ACTIVE_WINDOW_S,
            'screen_stale_ms': SCREEN_STALE_MS,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        state_manager,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_manager}


_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestStateManagerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_state_manager_client')

        self._state_history = []
        self.node.create_subscription(
            SystemState, '/system/state', self._state_history.append, _STATE_QOS)

        # P2: /system/effect の配送観測（self effect 以外がここへ流れる）
        self._effect_history = []
        self.node.create_subscription(
            StateEffect, '/system/effect', self._effect_history.append, 10)

        self.pub_event = self.node.create_publisher(StateEvent, '/system/event', 10)
        self.pub_screen = self.node.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_jog = self.node.create_publisher(String, '/ui/jog_lease', 1)
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_ui_estop = self.node.create_publisher(Bool, '/safety/estop_ui', 10)

        # P2: /routes/list は route_recorder が latched (TRANSIENT_LOCAL) で publish する前提
        self.pub_routes = self.node.create_publisher(RouteList, '/routes/list', _STATE_QOS)

        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_set_flag = self.node.create_client(SetFlag, '/system/set_flag')
        assert self.cli_trigger.wait_for_service(timeout_sec=5.0), \
            'state_manager が起動していない'

        self._reset_to_idle()
        self._state_history.clear()
        self._effect_history.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _trigger(self, trigger: str, arg: dict = None, requester: str = 'test') -> UiTrigger.Response:
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = requester
        future = self.cli_trigger.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        return future.result()

    def _latest(self) -> SystemState:
        self._spin(0.2)
        assert self._state_history, '/system/state を1件も受信していない'
        return self._state_history[-1]

    def _wait_mode(self, expected: str, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._state_history and self._state_history[-1].mode == expected:
                return True
        return False

    def _reset_to_idle(self):
        """どの状態からでも IDLE に戻す（前のテストの残留状態を消す）。"""
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.1)
        self.pub_hw.publish(Bool(data=True))
        self._spin(0.1)
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.1)
        self.pub_ui_estop.publish(Bool(data=False))
        self._spin(0.1)
        self._trigger('ui.finish')
        self._wait_mode('IDLE', timeout=3.0)

    # ════════════════════════════════════════════════════════
    # N-1 / N-2
    # ════════════════════════════════════════════════════════
    def test_no_transition_if_in_node(self):
        """N-1: state_manager.py にモード名の文字列比較が無い（完了条件①と同一の検査）。"""
        path = os.path.join(
            get_package_prefix('th_state'), 'lib', 'th_state', 'state_manager.py')
        src = open(path, encoding='utf-8').read()
        modes = {"INIT", "IDLE", "ESTOP", "CARRY", "FOLLOW", "MANUAL", "TEACH_FOLLOW",
                  "TEACH_MANUAL", "REPLAY", "LINE", "LEASH", "PREP", "PANEL_NAV", "AT_PANEL",
                  "SUMMON", "HOME_NAV", "OPCHECK", "CALIB"}
        bad = [n.value for n in ast.walk(ast.parse(src))
               if isinstance(n, ast.Constant) and n.value in modes]
        assert not bad, f"モード名がノードに直書きされている: {bad}"

    def test_lease_extends_on_reject(self):
        """N-2: リース時刻は accepted=false でも更新される。

        state_manager.py 自体の私有な lease タイムスタンプは外部トピックから
        観測できないので、別インスタンスを直接構築してホワイトボックスで見る
        （rclpy コンテキストは setUpClass で初期化済みのものを共有する）。
        IDLE は jog 対象外モード（guards.py の `_JOG_EXCLUDED_MODES`）なので
        ui.jog.hold は必ず accepted=False になるが、それでも last_jog_ms は進む。
        """
        from rclpy.parameter import Parameter as RclpyParameter
        module = _load_state_manager_module()
        node = module.StateManager(parameter_overrides=[
            RclpyParameter('jog_lease_ms', RclpyParameter.Type.INTEGER, JOG_LEASE_MS),
            RclpyParameter('link_wait_timeout_ms', RclpyParameter.Type.INTEGER,
                             LINK_WAIT_TIMEOUT_MS),
            RclpyParameter('ui_active_window_s', RclpyParameter.Type.INTEGER,
                             UI_ACTIVE_WINDOW_S),
            RclpyParameter('screen_stale_ms', RclpyParameter.Type.INTEGER, SCREEN_STALE_MS),
        ])
        try:
            before = node._last_jog_ms
            time.sleep(0.05)
            decision = node._process("ui.jog.hold", {}, "test-client")
            assert not decision.accepted, "IDLE では ui.jog.hold は拒否されるはず"
            after = node._last_jog_ms
            assert after > before, "拒否されても last_jog_ms は更新されるはず（N-2）"
        finally:
            node.destroy_node()

    # ════════════════════════════════════════════════════════
    # §11-7 / §11-8 — 名前空間の分離（安全境界。FMEA③）
    # ════════════════════════════════════════════════════════
    def test_ui_event_namespace_rejected(self):
        """§11-7: ui.* を /system/event（evt.* 専用）から入れても FSM は動かない。"""
        before = self._latest().mode
        assert before == 'IDLE'
        self.pub_event.publish(StateEvent(
            event='ui.enter_mode', source_node='rogue_node',
            arg_json=json.dumps({'mode': 'MANUAL'})))
        self._spin(0.5)
        after = self._latest().mode
        assert after == 'IDLE', f'ui.* が /system/event 経由で通ってしまった: {after}'

    def test_evt_via_trigger_rejected(self):
        """§11-8: evt.* を /system/trigger（ui.* 専用）から入れると拒否される。"""
        res = self._trigger('evt.arrived')
        assert res.accepted is False
        assert res.reject_reason_key == 'not_allowed'

    # ════════════════════════════════════════════════════════
    # §3.1 — /system/state のレートと N-5 の起動直後発行
    # ════════════════════════════════════════════════════════
    def test_state_published_at_10hz(self):
        self._state_history.clear()
        self._spin(1.0)
        count = len(self._state_history)
        # 10Hz ±ジッタ。1秒間の観測なので目安として 7〜16 件を許容する。
        assert 7 <= count <= 16, f'/system/state のレートが10Hzから外れている: {count} 件/秒'

    def test_transient_local_late_joiner(self):
        """N-5: transient_local なので、後から購読しても直近の1件が保持配信される。

        state_manager は 10Hz で publish し続けるため、「届くまでの時間」で
        transient_local と volatile を区別することはできない（volatile でも
        次の周期を待てば届いてしまう。DDS のディスカバリ自体も数百ms かかりうる）。
        QoS 自体を検査する方が決定的なので、まずそれを見る。そのうえで実際に
        後から購読しても届くこと自体も確認する。
        """
        infos = self.node.get_publishers_info_by_topic('/system/state')
        assert infos, '/system/state の publisher が見つからない'
        durability = infos[0].qos_profile.durability
        assert durability == QoSDurabilityPolicy.TRANSIENT_LOCAL, (
            f'/system/state の durability が transient_local ではない: {durability}')

        late_node = rclpy.create_node('test_late_joiner')
        try:
            received = []
            late_node.create_subscription(
                SystemState, '/system/state', received.append, _STATE_QOS)
            deadline = time.time() + 3.0
            while time.time() < deadline and not received:
                rclpy.spin_once(late_node, timeout_sec=0.1)
            assert received, 'transient_local の後から購読しても最新状態が届かない（N-5）'
        finally:
            late_node.destroy_node()

    # ════════════════════════════════════════════════════════
    # §7・N-3 — ESTOP/CARRY ラッチの6通り
    # ════════════════════════════════════════════════════════
    def test_latch_roundtrip(self):
        """state.md §7 の6行すべてを実際に踏む。

        注意（設計書との食い違い）: §7 の表は「ESTOP / CARRY 中の ui.finish → IDLE」を
        1行にまとめているが、guards.py の `_can_finish()` は mode が ESTOP のときは
        常に False を返す（transitions.yaml の C-08 は `can_finish` ガード付き）。
        つまり実装上 ui.finish は ESTOP からは絶対に受理されない
        （ESTOP の出口は C-09 ui.estop.release / C-09f ui.resume_ack のみ）。
        WP-STATE-01 の成果物（表・ガード）は編集しないので、ここでは実装の
        実際の経路（ESTOP は ui.estop.release、CARRY は ui.finish）に合わせて検証する。
        """
        # 1) 動作系 → ESTOP（fault.critical）。prev_mode/prev_state を記録する。
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('FOLLOW')

        self.pub_fault.publish(FaultStatus(active=True, fault_type='OTHER', severity='CRITICAL'))
        assert self._wait_mode('ESTOP')
        snap = self._latest()
        assert snap.prev_mode == 'FOLLOW'
        assert snap.prev_state == 'SELECT'

        # 1') 解除後は IDLE のみ・prev_* は捨てる（§7 行1。出口は ui.estop.release）。
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.1)
        res = self._trigger('ui.estop.release')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('IDLE')
        snap = self._latest()
        assert snap.prev_mode == '' and snap.prev_state == '', \
            'ESTOP 解除後は prev_* を捨てるはず（§7 行1）'

        # 2) 動作系 → CARRY（hw.estop.press）。prev_mode/prev_state を記録する。
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('FOLLOW')

        self.pub_hw.publish(Bool(data=True))
        assert self._wait_mode('CARRY')
        snap = self._latest()
        assert snap.prev_mode == 'FOLLOW'
        assert snap.prev_state == 'SELECT'

        # 3) CARRY 中の ui.estop.press は受け付けない（§7 行3）。
        res = self._trigger('ui.estop.press')
        assert res.accepted is False
        assert res.reject_reason_key == 'estop_disabled_in_carry'

        # 4) CARRY 中の fault.critical → ESTOP。prev_* は上書きしない（§7 行4）。
        self.pub_fault.publish(FaultStatus(active=True, fault_type='OTHER', severity='CRITICAL'))
        assert self._wait_mode('ESTOP')
        snap = self._latest()
        assert snap.prev_mode == 'FOLLOW', \
            'CARRY 中の fault.critical で prev_* が上書きされてはいけない（§7 行4）'

        # 5) ESTOP 中の hw.estop.press → CARRY。prev_* は上書きしない（§7 行5）。
        #    hw はすでに True のままなので、いったん False に落として立て直す。
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.1)
        self.pub_hw.publish(Bool(data=True))
        assert self._wait_mode('CARRY')
        snap = self._latest()
        assert snap.prev_mode == 'FOLLOW', \
            'ESTOP 中の hw.estop.press で prev_* が上書きされてはいけない（§7 行5）'

        # 2') CARRY → ui.carry_resume で prev_* へ戻る（§7 行2）。
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.1)
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.1)
        res = self._trigger('ui.carry_resume')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('FOLLOW'), 'ui.carry_resume で prev_mode へ戻れていない（§7 行2）'

        # 6) CARRY 中の ui.finish → IDLE。prev_* を捨てる（§7 行6。実装上 CARRY 側のみ）。
        self.pub_hw.publish(Bool(data=True))
        assert self._wait_mode('CARRY')
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.1)
        res = self._trigger('ui.finish')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('IDLE')
        snap = self._latest()
        assert snap.prev_mode == '' and snap.prev_state == '', \
            'CARRY 中の ui.finish 後は prev_* を捨てるはず（§7 行6）'

    # ════════════════════════════════════════════════════════
    # names.md §4.1 — derive_limits() の3ケース（zone）
    # ════════════════════════════════════════════════════════
    def test_zone_from_active_screen(self):
        client = 'tablet-zone-test'

        # ケース1: 使用中の端末が0台 → NA（フェイルセーフ既定）
        snap = self._latest()
        assert snap.zone == 'NA'

        # ケース2: IN 画面（S-21・試験）を使用中 → IN
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-21', client_id=client, interacting=True,
            last_input=self.node.get_clock().now().to_msg()))
        self._spin(0.3)
        assert self._latest().zone == 'IN'

        # ケース3: OUT 画面（S-01・メインメニュー）を使用中 → OUT
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-01', client_id=client, interacting=True,
            last_input=self.node.get_clock().now().to_msg()))
        self._spin(0.3)
        assert self._latest().zone == 'OUT'

        # 後片付け: 他のテストへゾーンが残らないようにする。
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-01', client_id=client, interacting=False,
            last_input=self.node.get_clock().now().to_msg()))
        self._spin(0.2)

    # ════════════════════════════════════════════════════════
    # P2 — effect の配送と route_ids の供給
    # ════════════════════════════════════════════════════════
    def _wait_effect(self, name: str, timeout: float = 3.0) -> list:
        """/system/effect に name が届くまで待つ。届いたものを返す。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            hit = [m for m in self._effect_history if m.name == name]
            if hit:
                return hit
        return []

    def test_route_select_emits_start_record_effect(self):
        """教示記録: ui.route_select {new:true, id} → start_record が /system/effect に届く。"""
        res = self._trigger('ui.enter_mode', {'mode': 'TEACH_MANUAL'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('TEACH_MANUAL')

        self._effect_history.clear()
        res = self._trigger('ui.route_select', {'new': True, 'id': 'r1'})
        assert res.accepted, res.reject_reason_key

        hits = self._wait_effect('start_record')
        assert len(hits) == 1, f'start_record が {len(hits)} 通届いた（1 通を期待）'
        assert hits[0].dest == 'route_recorder', hits[0].dest
        assert json.loads(hits[0].args_json).get('route_id') == 'r1', hits[0].args_json

    def test_routes_list_populates_route_ids_for_replay(self):
        """/routes/list の latched 配信で route_ids が埋まり、REPLAY の route_exists が通る。"""
        self.pub_routes.publish(RouteList(routes=[RouteInfo(id='r9')]))
        self._spin(0.4)

        res = self._trigger('ui.enter_mode', {'mode': 'REPLAY'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('REPLAY')

        self._effect_history.clear()
        res = self._trigger('ui.route_select', {'id': 'r9'})
        assert res.accepted, \
            f'replay の route_exists ガードが通らなかった: {res.reject_reason_key}'
        hits = self._wait_effect('load_route')
        assert len(hits) == 1, f'load_route が {len(hits)} 通届いた（1 通を期待）'
        assert hits[0].dest == 'replay_runner', hits[0].dest
        assert json.loads(hits[0].args_json).get('route_id') == 'r9', hits[0].args_json


if __name__ == '__main__':
    unittest.main()
