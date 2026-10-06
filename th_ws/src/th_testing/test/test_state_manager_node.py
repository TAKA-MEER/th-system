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
from th_system_msgs.msg import (ActiveScreen, CalibStatus, FaultStatus, PersonTargets, RouteInfo,
                                RouteList, StateEffect, StateEvent, SystemState)
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
        # WP-MAINT-02: calib_runner が出す /calib/status（reliable depth 5）
        self.pub_calib = self.node.create_publisher(CalibStatus, '/calib/status', 5)

        # P2: /route/catalog は route_recorder が latched (TRANSIENT_LOCAL) で publish する前提
        self.pub_routes = self.node.create_publisher(RouteList, '/route/catalog', _STATE_QOS)

        # WP-ONSITE-F2: /person/targets（person_tracker_bridge / stub の RELIABLE/VOLATILE に合わせる）
        self.pub_person = self.node.create_publisher(PersonTargets, '/person/targets', 1)

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

    def _set_flag(self, flag: str, value: bool) -> SetFlag.Response:
        req = SetFlag.Request()
        req.flag = flag
        req.value = value
        future = self.cli_set_flag.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        return future.result()

    def _publish_tracked_person(self):
        """/person/targets に追跡中の対象 1 人を publish する（PREP/REGISTER 用）。"""
        from geometry_msgs.msg import Point
        msg = PersonTargets()
        msg.candidates.append(Point(x=1.0, y=0.0, z=0.0))
        msg.selected_index = 0
        msg.confidence = 0.9
        msg.is_lost = False
        msg.lost_reason = ''
        self.pub_person.publish(msg)
        self._spin(0.3)

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

        手順 1' の出口は 2026-09-04 の WS-9O で書き換えられている。当時までは
        「ESTOP 解除後は IDLE のみ」だったが、現在は**入口（UI ボタン／
        fault.critical）を問わず、フォールトが消え物理ボタンが解放されていれば
        ui.resume_yes で $prev_mode の PAUSE へ出る**
        （Spec-safety.md §3.5.2・Spec-modes.md §4.1。§7 も 8692b62 で更新済み）。
        どれの行が効くか（C-09／C-09c／C-09d／C-09f）は手順 1' で実際に踏む。

        ui.finish は ESTOP からは絶対に受理されない（guards._can_finish は
        mode==ESTOP で常に False。C-08 の can_finish ガード）。ESTOP の出口は
        C-09 / C-09c / C-09d / C-09f の4行で、CARRY 側の出口が ui.finish。
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

        # 1') フォールト解消後の出口（§7 行1。2026-09-04 WS-9O で書き換え）。
        #     ui.estop.release は IDLE に落とすのではなく、ESTOP のまま
        #     「戻る／メニューへ」を出すだけ（C-09。to_mode/to_state は '='）。
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.1)
        res = self._trigger('ui.estop.release')
        assert res.accepted, res.reject_reason_key
        snap = self._latest()
        assert snap.mode == 'ESTOP', \
            f'ui.estop.release で IDLE に落ちてしまった（C-09 ではなく C-09b が効いた）: {snap.mode}'

        #     その先の「確認」→ IDLE。prev_* は捨てる（C-09f。SM-3.1.1-12）。
        res = self._trigger('ui.resume_ack')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('IDLE')
        snap = self._latest()
        assert snap.prev_mode == '' and snap.prev_state == '', \
            'ESTOP を IDLE で出たら prev_* を捨てるはず（§7 行1・C-09f）'

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
    # SM-3.1.1-11 / 12 — ESTOP からの復帰先。
    # 2026-09-01: UI ボタン由来の ESTOP（重大フォールト無し）は解除後に
    #   「戻る／メニューへ」を選べる。
    # 2026-09-04 WS-9O: 入口が UI ボタンか重大フォールトかを問わない。
    #   フォールトが消えていれば「戻る」で押下前のモードへ、
    #   「メニューへ」／「確認」は IDLE へ出る（Spec-safety.md §3.5.2）。
    #   依然として要求されるのは「フォールトが実際に消えていること」と
    #   「物理非常停止が解放されていること」の2点。
    # 2026-10-05 1b-2（SG-A3）: 「戻る」の戻り先は §6 の復帰先へ。
    #   走行中なら同モードの PAUSE、止まっている状態なら押下前の状態へ
    #   入口からやり直す（勝手には走り出さない）。
    # ════════════════════════════════════════════════════════
    def test_ui_estop_resume_to_prev_mode(self):
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('MANUAL')

        # UI 非常停止ボタン押下 → ESTOP。prev_mode を記録。
        self.pub_ui_estop.publish(Bool(data=True))
        assert self._wait_mode('ESTOP')
        assert self._latest().prev_mode == 'MANUAL'

        # 解除（true→false）→ ESTOP のまま（W-1 が「戻る／メニューへ」に変わる）。
        self.pub_ui_estop.publish(Bool(data=False))
        self._spin(0.3)
        assert self._latest().mode == 'ESTOP', 'UI estop 解除で即 IDLE に落ちてはいけない'

        # 「元のモードに戻る」→ 押下前モードの PAUSE へ。
        res = self._trigger('ui.resume_yes')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('MANUAL')
        assert self._latest().state == 'PAUSE'

    def test_fault_estop_resume_to_prev_pause(self):
        """重大フォールト起因の ESTOP も、フォールトが消えれば押下前のモードへ戻る（WS-9O）。

        SM-3.1.1-11（`C-09c-*`）／Spec-safety.md §3.5.2。2026-09-04 までは UI ボタン
        起因だけが戻せたので、26ms で消えるような一瞬の重大フォールトでも作業の
        最初からやり直しになっていた（同節冒頭）。
        2026-10-05 1b-2（SG-A3）: 復帰先は §6 の復帰先へ。FOLLOW/SELECT のように
        止まっている状態からは押下前の状態へ戻り（SELECT から選び直す）、
        走行中（RUN）からだけ PAUSE（停止）に戻る。走り出すには操作者が
        もう一度走行を押す（勝手には走り出さない）。

        手順 3 は「重大フォールトが解けた」と「すべてのフォールトが解けた」を
        分けて踏む。`C-09c-*` のガードには severity の項だけでなく `not ctx.fault_active`
        の項もあるため、severity を落とした回復フォールトが残っている間は戻せない。
        """
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('FOLLOW')

        # 1) 重大フォールトで ESTOP。押下前のモードと状態をラッチする。
        self.pub_fault.publish(FaultStatus(active=True, fault_type='OTHER', severity='CRITICAL'))
        assert self._wait_mode('ESTOP')
        snap = self._latest()
        assert snap.prev_mode == 'FOLLOW' and snap.prev_state == 'SELECT', \
            f'重大フォールトで押下前がラッチされていない: {snap.prev_mode}/{snap.prev_state}'

        # 2) フォールトがまだ継続している間は「戻る」を拒否する
        #    （guards._estop_resume_prev。Spec-safety.md §3.5.2「依然として要求すること」）。
        res = self._trigger('ui.resume_yes')
        assert res.accepted is False, \
            'フォールト継続中に ui.resume_yes が通ってしまった（_estop_resume_prev が効いていない）'
        assert res.reject_reason_key == 'not_allowed', res.reject_reason_key
        snap = self._latest()
        assert snap.mode == 'ESTOP', f'フォールト継続中に ESTOP を離れた: {snap.mode}'

        # 3) 重大フォールトだけ解消しても、**他のフォールトが残っている間は**
        #    「戻る」を拒否する（guards._estop_resume_prev の `not ctx.fault_active`
        #    項。Spec-safety.md §3.5.2「依然として要求すること」の
        #    「フォールトが実際に消えていること」）。
        #    severity を落とした回復フォールト（active=True / severity=''）を
        #    送ると severity の項では止められないので、active の項を踏む。
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.2)
        self.pub_fault.publish(
            FaultStatus(active=True, fault_type='LIDAR_LOST', severity=''))
        self._spin(0.2)
        res = self._trigger('ui.resume_yes')
        assert res.accepted is False, \
            '回復フォールトが残っているのに ui.resume_yes が通ってしまった（not ctx.fault_active が効いていない）'
        assert res.reject_reason_key == 'not_allowed', res.reject_reason_key
        snap = self._latest()
        assert snap.mode == 'ESTOP', f'回復フォールト継続中に ESTOP を離れた: {snap.mode}'

        # 4) フォールトがすべて消えたら「戻る」で押下前の状態へ。prev_* は捨てる。
        #    FOLLOW/SELECT は止まっている状態なので SELECT に戻る（C-09c-generic。
        #    1b-2 SG-A3。旧仕様の PAUSE ではない。対象を選び直してから走行する）。
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.2)
        res = self._trigger('ui.resume_yes')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('FOLLOW'), '「戻る」で押下前のモードへ戻っていない（C-09c-generic）'
        snap = self._latest()
        assert snap.state == 'SELECT', \
            f'復帰先が押下前の状態（SELECT）ではない: {snap.mode}/{snap.state}'
        assert snap.prev_mode == '' and snap.prev_state == '', \
            f'復帰後も prev_* が残っている: {snap.prev_mode}/{snap.prev_state}'

    def test_fault_estop_resume_ack_and_no_go_to_idle(self):
        """重大フォールトで入った ESTOP の「確認」と「メニューへ」はどちらも IDLE へ出る。

        「確認」= `ui.resume_ack`（`C-09f`。SM-3.1.1-12。UI ボタンを押していないので
        「解除」は来ないが、この行が無いと出られない）／
        「メニューへ」= `ui.resume_no`（`C-09d`。SM-3.1.1-11）。どちらも `prev_*` は捨てる。
        """
        for trigger, label in (('ui.resume_ack', '確認'), ('ui.resume_no', 'メニューへ')):
            res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
            assert res.accepted, res.reject_reason_key
            assert self._wait_mode('FOLLOW')
            self.pub_fault.publish(
                FaultStatus(active=True, fault_type='OTHER', severity='CRITICAL'))
            assert self._wait_mode('ESTOP')
            self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
            self._spin(0.2)

            res = self._trigger(trigger)
            assert res.accepted, f'{label}（{trigger}）が拒否された: {res.reject_reason_key}'
            assert self._wait_mode('IDLE'), f'{label}（{trigger}）で IDLE へ出なかった'
            snap = self._latest()
            assert snap.prev_mode == '' and snap.prev_state == '', \
                f'{label} で IDLE へ出たのに prev_* が残っている: {snap.prev_mode}/{snap.prev_state}'

    def test_fault_estop_resume_blocked_while_hw_pressed(self):
        """物理非常停止が押されたままなら、重大フォールトが解消していても戻れない。

        `guards._estop_resume_prev` の `not ctx.hw_estop` 項
        （Spec-safety.md §3.5.2「依然として要求すること」）。物理非常停止を押すと
        `C-07` で CARRY へ移るため、「ESTOP に居る状態で押されている」状態は
        「CARRY のまま重大フォールト」で作る。`latch_prev` は CARRY/ESTOP では記録
        されない（state_core.NO_LATCH_MODES）ので、prev_mode は CARRY に入った時の
        FOLLOW のまま残る。
        """
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('FOLLOW')

        self.pub_hw.publish(Bool(data=True))
        assert self._wait_mode('CARRY')

        self.pub_fault.publish(FaultStatus(active=True, fault_type='OTHER', severity='CRITICAL'))
        assert self._wait_mode('ESTOP')
        assert self._latest().prev_mode == 'FOLLOW', \
            'CARRY → ESTOP で prev_mode が上書きされた'

        # 重大フォールトは解消するが、物理非常停止は押したままにする。
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.2)
        res = self._trigger('ui.resume_yes')
        assert res.accepted is False, \
            '物理非常停止が押されたまま「戻る」が通ってしまった（not ctx.hw_estop が効いていない）'
        snap = self._latest()
        assert snap.mode == 'ESTOP', f'物理非常停止が押されたまま ESTOP を離れた: {snap.mode}'

        # 後片付け。物理非常停止を解放してから「確認」で IDLE へ（SM-3.1.1-12）。
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.2)
        res = self._trigger('ui.resume_ack')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('IDLE')

    # ════════════════════════════════════════════════════════
    # 1b-2（SG-A5・SG-A3・SG-B22）— state_manager を実際に起動する試験。
    # launch したノードは setUp で IDLE まで進んでいるため、INIT からの往復と
    # 経路ラッチはホワイトボックス（別インスタンスを直接構築。起動直後の
    # INIT/CHECK から始まる）で縛る。C-04 の None ケースは launch したノードで踏む。
    # ════════════════════════════════════════════════════════
    def _make_whitebox_node(self):
        from rclpy.parameter import Parameter as RclpyParameter
        module = _load_state_manager_module()
        return module.StateManager(parameter_overrides=[
            RclpyParameter('jog_lease_ms', RclpyParameter.Type.INTEGER, JOG_LEASE_MS),
            RclpyParameter('link_wait_timeout_ms', RclpyParameter.Type.INTEGER,
                           LINK_WAIT_TIMEOUT_MS),
            RclpyParameter('ui_active_window_s', RclpyParameter.Type.INTEGER,
                           UI_ACTIVE_WINDOW_S),
            RclpyParameter('screen_stale_ms', RclpyParameter.Type.INTEGER, SCREEN_STALE_MS),
        ])

    def test_init_estop_returns_to_check(self):
        """SG-A5: 起動直後（INIT/CHECK）に UI 非常停止を押して離すと
        INIT/CHECK に戻り疎通確認からやり直す（IDLE に入らない。C-09b-init）。"""
        node = self._make_whitebox_node()
        try:
            assert (node.mode, node.state) == ('INIT', 'CHECK'), \
                f'起動直後が INIT/CHECK ではない: {node.mode}/{node.state}'
            d1 = node._process('ui.estop.press', {}, 'test')
            assert d1.accepted, d1.reject_reason_key
            assert node.mode == 'ESTOP'
            assert (node.prev_mode, node.prev_state) == ('INIT', 'CHECK')

            d2 = node._process('ui.estop.release', {}, 'test')
            assert d2.accepted, d2.reject_reason_key
            assert d2.rule_id == 'C-09b-init', f'効いた行が違う: {d2.rule_id}'
            assert (node.mode, node.state) == ('INIT', 'CHECK'), \
                f'INIT/CHECK に戻っていない: {node.mode}/{node.state}'
            assert node.prev_mode == '' and node.prev_state == '', \
                'INIT に戻ったのに prev_* が残っている'
            node._publish_state()  # 落ちずに publish できること
        finally:
            node.destroy_node()

    def test_replay_route_latched_for_estop_resume(self):
        """SG-A3: REPLAY の経路選択をラッチし、C-09c-localize の load_route に載せる。
        TEACH_* の選択では上書きしない（別モードの経路で復帰しない）。"""
        node = self._make_whitebox_node()
        try:
            node._route_ids = ['R1']
            # ホワイトボックスは INIT/CHECK 始動。IDLE へ出してから入る。
            res = node._process('evt.link_ok', {}, 'test')
            assert res.accepted and node.mode == 'IDLE', res.reject_reason_key
            res = node._process('ui.enter_mode', {'mode': 'REPLAY'}, 'test')
            assert res.accepted, res.reject_reason_key
            res = node._process(
                'ui.route_select', {'id': 'R1', 'reverse': True}, 'test')
            assert res.accepted, res.reject_reason_key
            assert res.rule_id == 'T-REPLAY-01'
            assert node._replay_route == {'id': 'R1', 'reverse': True}

            node._fault_active = True
            node._fault_severity = 'CRITICAL'
            res = node._process('fault.critical', {}, 'test')
            assert node.mode == 'ESTOP'
            assert (node.prev_mode, node.prev_state) == ('REPLAY', 'LOCALIZE')

            node._fault_active = False
            node._fault_severity = ''
            res = node._process('ui.resume_yes', {}, 'test')
            assert res.accepted, res.reject_reason_key
            assert res.rule_id == 'C-09c-localize', f'効いた行が違う: {res.rule_id}'
            assert (node.mode, node.state) == ('REPLAY', 'LOCALIZE')
            by_name = {e.name: e.args for e in res.effects}
            assert by_name.get('load_route') == {'route_id': 'R1', 'reverse': True}, \
                f'load_route の引数が違う: {by_name}'

            # TEACH_MANUAL の選択では上書きしない。
            node._process('ui.finish', {}, 'test')
            assert node.mode == 'IDLE'
            res = node._process('ui.enter_mode', {'mode': 'TEACH_MANUAL'}, 'test')
            assert res.accepted, res.reject_reason_key
            res = node._process('ui.route_select', {'new': True}, 'test')
            assert res.accepted, res.reject_reason_key
            assert node._replay_route == {'id': 'R1', 'reverse': True}, \
                'TEACH の選択で REPLAY のラッチが上書きされた'
        finally:
            node.destroy_node()

    def test_c04_none_rejected_and_node_survives(self):
        """SG-B22: AT_HOME/PAUSE で ui.resume_yes は拒否され、state=None を作らない。
        publish の型検査で state_manager が落ちないこと（拒否後も応答すること）。

        PAUSE へは回復フォルト（C-03）で入る。ジョグ経由だとリース満了
        （T-ATH-02・300ms）で IDLE_H に戻ってしまい timing が不安定なため。
        フォルト解消後は PAUSE のまま残る（fault.cleared は遷移を起こさない。
        §7.1）ので、フォルト無しの状態で ui.resume_yes を踏める。
        """
        res = self._trigger('ui.goto', {'kind': 'HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('HOME_NAV')
        self.pub_event.publish(StateEvent(
            event='evt.arrived', source_node='test', arg_json=''))
        assert self._wait_mode('AT_HOME')
        self.pub_fault.publish(
            FaultStatus(active=True, fault_type='LIDAR_LOST', severity=''))
        self._spin(0.3)
        snap = self._latest()
        assert (snap.mode, snap.state) == ('AT_HOME', 'PAUSE'), \
            f'AT_HOME/PAUSE に入れていない: {snap.mode}/{snap.state}'
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.3)
        snap = self._latest()
        assert (snap.mode, snap.state) == ('AT_HOME', 'PAUSE'), \
            f'フォルト解消で PAUSE を離れた: {snap.mode}/{snap.state}'

        res = self._trigger('ui.resume_yes')
        assert res.accepted is False, \
            'run_state が null の AT_HOME で ui.resume_yes が通ってしまった（SG-B22）'
        snap = self._latest()
        assert (snap.mode, snap.state) == ('AT_HOME', 'PAUSE'), \
            f'拒否後に状態が動いた: {snap.mode}/{snap.state}'
        assert snap.state is not None
        # 落ちていないこと: 次の trigger に応答する。
        res = self._trigger('ui.resume_ack')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('AT_HOME')
        assert self._latest().state == 'IDLE_H'

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
    # brief-tracker-default-off §3.1: tracker_enabled の自動停止と OFF 拒否
    # （Spec-modes.md §9「動かさない」自動停止／§5.1 要 person での OFF 拒否）
    # ════════════════════════════════════════════════════════
    def test_set_flag_tracker_enabled_off_allowed_in_idle(self):
        """IDLE では OFF できる（『動かさない』モードとは逆の要 person でない状態）。"""
        res = self._set_flag('tracker_enabled', True)
        assert res.accepted, res.reject_reason_key
        assert self._latest().tracker_enabled is True
        res = self._set_flag('tracker_enabled', False)
        assert res.accepted, res.reject_reason_key
        assert self._latest().tracker_enabled is False

    def test_set_flag_tracker_enabled_off_rejected_in_summon(self):
        """SUMMON（要 person・どの状態でも）では OFF が拒否され理由キーが返る。"""
        res = self._set_flag('tracker_enabled', True)
        assert res.accepted, res.reject_reason_key
        res = self._trigger('ui.goto', {'kind': 'SUMMON'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('SUMMON')

        res = self._set_flag('tracker_enabled', False)
        assert res.accepted is False, 'SUMMON では OFF できるようになってしまった'
        assert res.reject_reason_key == 'tracker_required', res.reject_reason_key
        assert self._latest().tracker_enabled is True, '拒否されたのにフラグが落ちた'

    def test_set_flag_tracker_enabled_off_rejected_in_prep_register(self):
        """PREP の REGISTER（ピン登録中）では OFF が拒否される（MAPPING では許可）。"""
        self._publish_tracked_person()
        res = self._trigger('ui.enter_mode', {'mode': 'PREP'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('PREP')

        # MAPPING では OFF も ON もできる。
        res = self._set_flag('tracker_enabled', True)
        assert res.accepted, res.reject_reason_key

        res = self._trigger('ui.register', {'kind': 'HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('PREP')
        assert self._latest().state == 'REGISTER'

        res = self._set_flag('tracker_enabled', False)
        assert res.accepted is False, 'PREP/REGISTER では OFF できるようになってしまった'
        assert res.reject_reason_key == 'tracker_required', res.reject_reason_key
        assert self._latest().tracker_enabled is True

    def test_tracker_autostop_on_entering_mode_without_person(self):
        """フラグ true のまま「動かさない」モード（MANUAL）へ遷移すると自動で false。"""
        res = self._set_flag('tracker_enabled', True)
        assert res.accepted, res.reject_reason_key
        assert self._latest().tracker_enabled is True

        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('MANUAL')
        assert self._latest().tracker_enabled is False, \
            '「動かさない」モードへ入ったのに tracker_enabled が落ちていない'

    def test_tracker_autostop_not_on_prep_internal_moves(self):
        """試験準備（PREP）のモード内移動では落とさない（試験準備・試験当日は保持）。"""
        self._publish_tracked_person()
        res = self._trigger('ui.enter_mode', {'mode': 'PREP'})
        assert res.accepted, res.reject_reason_key
        res = self._set_flag('tracker_enabled', True)
        assert res.accepted, res.reject_reason_key
        assert self._latest().tracker_enabled is True

        # PREP 内で MAPPING → REGISTER → MAPPING と動いてもフラグは保持される。
        res = self._trigger('ui.register', {'kind': 'HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._latest().state == 'REGISTER'
        # evt.* は §11-8（名前空間分離。test_evt_via_trigger_rejected）により
        # /system/trigger（ui.* 専用）からは拒否される。/system/event に
        # publish する（§11-7 のテストと同じ経路。self._trigger() は使えない）。
        self.pub_event.publish(StateEvent(event='evt.register_ok', source_node='test'))
        assert self._latest().state == 'MAPPING'
        assert self._latest().tracker_enabled is True, \
            'PREP のモード内移動で tracker_enabled が落ちてしまった'

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
        """/route/catalog の latched 配信で route_ids が埋まり、REPLAY の route_exists が通る。"""
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

    # ════════════════════════════════════════════════════════
    # WP-MAINT-01 — OPCHECK: Context.check_item/check_result の配線
    # ════════════════════════════════════════════════════════
    def test_opcheck_estop_item_press_does_not_carry(self):
        """T-OPC-05: ESTOP 項目の実行中は物理ボタンを押しても CARRY へ落ちない。

        修正前は state_manager.py が Context.check_item を常に空文字で渡して
        いたため guards.py の _checking_estop_item が一度も成立せず、共通の
        hw.estop.press 遷移（CARRY へ強制遷移）に落ちていた（2026-09-25 修正）。
        """
        res = self._trigger('ui.enter_mode', {'mode': 'OPCHECK'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('OPCHECK')
        assert self._latest().state == 'LIST'

        res = self._trigger('ui.check_item', {'item': 'ESTOP'})
        assert res.accepted, res.reject_reason_key
        snap = self._latest()
        assert snap.mode == 'OPCHECK' and snap.state == 'RUNNING_CHECK', \
            f'T-OPC-01 が RUNNING_CHECK に進まなかった: mode={snap.mode} state={snap.state}'

        self._effect_history.clear()
        self.pub_hw.publish(Bool(data=True))
        self._spin(0.3)
        snap = self._latest()
        assert snap.mode == 'OPCHECK' and snap.state == 'RUNNING_CHECK', (
            'ESTOP 項目中の物理ボタン押下で CARRY に落ちた（check_item の配線漏れ）: '
            f'mode={snap.mode} state={snap.state}')
        hits = self._wait_effect('feed_check_input', timeout=1.0)
        assert hits, 'T-OPC-05 が発火せず feed_check_input が配送されなかった'
        assert hits[0].dest == 'opcheck_runner', hits[0].dest

        self.pub_hw.publish(Bool(data=False))
        self._spin(0.3)
        snap = self._latest()
        assert snap.mode == 'OPCHECK' and snap.state == 'RUNNING_CHECK', (
            'ESTOP 項目中の物理ボタン解除で CARRY に落ちた: '
            f'mode={snap.mode} state={snap.state}')

        # 後始末（他テストを汚さない）。can_finish は ESTOP/CARRY 以外なら常に真。
        res = self._trigger('ui.finish')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('IDLE')

    def test_opcheck_check_result_ok_and_ng_transitions(self):
        """evt.check_result の OK/NG で T-OPC-02/03/04 のとおり遷移する。

        - NG かつ非校正項目（MOTOR）→ REPAIR（T-OPC-03）
        - NG かつ校正可能項目（IMU）→ LIST + offer_calib（T-OPC-02）
        - OK（ESTOP）→ LIST + record_result（T-OPC-04）
        修正前は Context.check_result を常に空文字で渡していたため、この3遷移
        すべてが不成立で RUNNING_CHECK から先に進めなかった（2026-09-25 修正）。

        T-OPC-02/03 の record_result effect（2026-09-25 追加）: 修正前は
        offer_calib（T-OPC-02）のみ・effects なし（T-OPC-03）で、
        opcheck_runner 側の self._item を閉じる手段が無く、以後すべての
        run_item が「別の項目が実行中」で拒否され続けた（実装管理担当の穴
        指摘 §2）。
        """
        res = self._trigger('ui.enter_mode', {'mode': 'OPCHECK'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('OPCHECK')

        # NG・非校正 (MOTOR) → REPAIR + record_result（T-OPC-03）
        self._effect_history.clear()
        res = self._trigger('ui.check_item', {'item': 'MOTOR'})
        assert res.accepted, res.reject_reason_key
        assert self._latest().state == 'RUNNING_CHECK'
        self.pub_event.publish(StateEvent(
            event='evt.check_result', source_node='opcheck_runner',
            arg_json=json.dumps({'item': 'MOTOR', 'result': 'NG'})))
        self._spin(0.3)
        snap = self._latest()
        assert snap.mode == 'OPCHECK' and snap.state == 'REPAIR', \
            f'MOTOR NG が REPAIR に進まなかった（T-OPC-03）: {snap.mode}/{snap.state}'
        hits = self._wait_effect('record_result', timeout=1.0)
        assert hits, \
            'T-OPC-03 で record_result が配送されなかった（opcheck_runner の項目が閉じない）'
        assert hits[0].dest == 'opcheck_runner', hits[0].dest
        assert json.loads(hits[0].args_json).get('item') == 'MOTOR', hits[0].args_json
        assert json.loads(hits[0].args_json).get('result') == 'NG', hits[0].args_json

        res = self._trigger('ui.stop')
        assert res.accepted, res.reject_reason_key
        assert self._latest().state == 'LIST', 'T-OPC-06 (REPAIR→LIST) が通らない'

        # NG・校正可能 (IMU) → LIST + offer_calib + record_result（T-OPC-02）
        res = self._trigger('ui.check_item', {'item': 'IMU'})
        assert res.accepted, res.reject_reason_key
        assert self._latest().state == 'RUNNING_CHECK'
        self._effect_history.clear()
        self.pub_event.publish(StateEvent(
            event='evt.check_result', source_node='opcheck_runner',
            arg_json=json.dumps({'item': 'IMU', 'result': 'NG'})))
        self._spin(0.3)
        snap = self._latest()
        assert snap.mode == 'OPCHECK' and snap.state == 'LIST', \
            f'IMU NG が LIST に進まなかった（T-OPC-02）: {snap.mode}/{snap.state}'
        hits = self._wait_effect('offer_calib', timeout=1.0)
        assert hits, 'T-OPC-02 で offer_calib が配送されなかった'
        assert json.loads(hits[0].args_json).get('item') == 'IMU', hits[0].args_json
        hits = self._wait_effect('record_result', timeout=1.0)
        assert hits, \
            'T-OPC-02 で record_result が配送されなかった（opcheck_runner の項目が閉じない）'
        assert hits[0].dest == 'opcheck_runner', hits[0].dest
        assert json.loads(hits[0].args_json).get('item') == 'IMU', hits[0].args_json
        assert json.loads(hits[0].args_json).get('result') == 'NG', hits[0].args_json

        # OK (ESTOP) → LIST + record_result
        res = self._trigger('ui.check_item', {'item': 'ESTOP'})
        assert res.accepted, res.reject_reason_key
        assert self._latest().state == 'RUNNING_CHECK'
        self._effect_history.clear()
        self.pub_event.publish(StateEvent(
            event='evt.check_result', source_node='opcheck_runner',
            arg_json=json.dumps({'item': 'ESTOP', 'result': 'OK'})))
        self._spin(0.3)
        snap = self._latest()
        assert snap.mode == 'OPCHECK' and snap.state == 'LIST', \
            f'ESTOP OK が LIST に進まなかった（T-OPC-04）: {snap.mode}/{snap.state}'
        hits = self._wait_effect('record_result', timeout=1.0)
        assert hits, 'T-OPC-04 で record_result が配送されなかった'
        assert hits[0].dest == 'opcheck_runner', hits[0].dest
        assert json.loads(hits[0].args_json).get('result') == 'OK', hits[0].args_json

        res = self._trigger('ui.finish')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('IDLE')

    # ════════════════════════════════════════════════════════
    # WP-MAINT-02 — CALIB: Context.calib_item / calib_preview_sane の配線
    # ════════════════════════════════════════════════════════
    def _publish_calib_status(self, item: str, result: str):
        self.pub_calib.publish(CalibStatus(item=item, result=result, step='S3'))
        self._spin(0.3)

    def _publish_evt(self, event: str, arg: dict = None):
        self.pub_event.publish(StateEvent(
            event=event, source_node='calib_runner', arg_json=json.dumps(arg or {})))
        self._spin(0.3)

    def test_calib_item_latch_and_preview_sane_wiring(self):
        """T-CAL-01〜08 が配線されている（修正前は calib_item="" ・preview_sane=False 固定）。

        - ui.calib_item でラッチした項目が、item を付けない ui.calib_next / evt.* の
          effect 引数（$arg.item）へ補われる
        - T-CAL-04 のガード preview_sane は、calib_runner の /calib/status
          （result=="PREVIEW_OK"・実行中の項目と一致・S3 の間だけ）でだけ通る
        - S3 を出たらラッチが落ち、次の S3 で出し直しになる
        """
        res = self._trigger('ui.enter_mode', {'mode': 'CALIB'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('CALIB')

        assert self._trigger('ui.calib_item', {'item': 'LINEAR'}).accepted
        assert self._latest().state == 'S1'

        # item 無しの ui.calib_next（T-CAL-02）→ run_measurement に item が補われる
        self._effect_history.clear()
        assert self._trigger('ui.calib_next').accepted
        assert self._latest().state == 'S2'
        hits = self._wait_effect('run_measurement', timeout=1.0)
        assert hits and hits[0].dest == 'calib_runner', 'run_measurement が配送されない'
        assert json.loads(hits[0].args_json).get('item') == 'LINEAR', \
            f'$arg.item が補われていない: {hits[0].args_json}'

        # item 無しの evt.calib_step_done（T-CAL-03）→ S3 + build_preview
        self._effect_history.clear()
        self._publish_evt('evt.calib_step_done')
        assert self._latest().state == 'S3', 'evt.calib_step_done で S3 へ進まない'
        hits = self._wait_effect('build_preview', timeout=1.0)
        assert hits and json.loads(hits[0].args_json).get('item') == 'LINEAR'

        # sane でない間は S3 から進めない（T-CAL-04 のガード preview_sane）
        res = self._trigger('ui.calib_next')
        assert not res.accepted, 'プレビューが sane でないのに S4 へ進めた（配線が常に真）'
        self._publish_calib_status('LINEAR', 'PREVIEW_INSANE')
        assert not self._trigger('ui.calib_next').accepted
        # 別項目の PREVIEW_OK は無視する
        self._publish_calib_status('ROTATION', 'PREVIEW_OK')
        assert not self._trigger('ui.calib_next').accepted, '別項目の PREVIEW_OK で進めた'

        # 実行中の項目の PREVIEW_OK で進める
        self._publish_calib_status('LINEAR', 'PREVIEW_OK')
        self._effect_history.clear()
        res = self._trigger('ui.calib_next')
        assert res.accepted, f'sane なプレビューで S4 へ進めない（配線が常に偽）: {res.reject_reason_key}'
        assert self._latest().state == 'S4'
        hits = self._wait_effect('apply_and_verify', timeout=1.0)
        assert hits and json.loads(hits[0].args_json).get('item') == 'LINEAR'

        # 検証 NG（T-CAL-06）→ S2 + revert_calib
        self._effect_history.clear()
        self._publish_evt('evt.calib_verify_ng')
        assert self._latest().state == 'S2'
        hits = self._wait_effect('revert_calib', timeout=1.0)
        assert hits and json.loads(hits[0].args_json).get('item') == 'LINEAR'

        # もう一度 S3 へ。前回の PREVIEW_OK は S3 を出た時点で落ちているので、進めない
        self._publish_evt('evt.calib_step_done')
        assert self._latest().state == 'S3'
        assert not self._trigger('ui.calib_next').accepted, \
            '前回の S3 の sane が持ち越された（S3 を出てもラッチが落ちない）'
        self._publish_calib_status('LINEAR', 'PREVIEW_OK')
        assert self._trigger('ui.calib_next').accepted
        assert self._latest().state == 'S4'

        # 検証 OK（T-CAL-05）→ LIST + commit_calib（item 付き）+ offer_opcheck
        self._effect_history.clear()
        self._publish_evt('evt.calib_step_done')
        assert self._latest().state == 'LIST'
        hits = self._wait_effect('commit_calib', timeout=1.0)
        assert hits and hits[0].dest == 'calib_runner'
        assert json.loads(hits[0].args_json).get('item') == 'LINEAR', hits[0].args_json

        # LIST に戻ったらラッチは空。次の項目を始められ、abort（T-CAL-07）は新しい item で discard
        assert self._trigger('ui.calib_item', {'item': 'ROTATION'}).accepted
        self._effect_history.clear()
        assert self._trigger('ui.abort').accepted
        assert self._latest().state == 'LIST'
        hits = self._wait_effect('discard_calib', timeout=1.0)
        assert hits and json.loads(hits[0].args_json).get('item') == 'ROTATION', \
            hits[0].args_json if hits else 'discard_calib が配送されない'

        assert self._trigger('ui.finish').accepted
        assert self._wait_mode('IDLE')

    def test_calib_fault_discards_with_item(self):
        """校正中のフォルト（T-CAL-08）で discard_calib が項目付きで配送される（確定しない）。"""
        assert self._trigger('ui.enter_mode', {'mode': 'CALIB'}).accepted
        assert self._wait_mode('CALIB')
        assert self._trigger('ui.calib_item', {'item': 'LINEAR'}).accepted
        assert self._trigger('ui.calib_next').accepted
        self._effect_history.clear()
        self.pub_fault.publish(FaultStatus(
            active=True, fault_type='LIDAR_LOST', severity='RECOVERABLE'))
        self._spin(0.4)
        hits = self._wait_effect('discard_calib', timeout=1.0)
        assert hits, '校正中のフォルトで discard_calib が配送されない（T-CAL-08）'
        assert json.loads(hits[0].args_json).get('item') == 'LINEAR', hits[0].args_json
        assert not self._wait_effect('commit_calib', timeout=0.2), 'フォルトで確定が走った'
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.2)


    # ════════════════════════════════════════════════════════
    # 1b-1 SG-A12: 走行中の在席喪失 → PAUSE＋pause_reason=presence_lost
    # ════════════════════════════════════════════════════════
    def _publish_presence(self, client='tablet-presence-test', interacting=True):
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-11', client_id=client, interacting=interacting,
            last_input=self.node.get_clock().now().to_msg()))

    def _wait_mode_state(self, mode, state, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._state_history and (
                    self._state_history[-1].mode,
                    self._state_history[-1].state) == (mode, state):
                return True
        hist = self._state_history[-1] if self._state_history else None
        last = (hist.mode, hist.state) if hist else None
        return last == (mode, state)

    def test_presence_lost_while_driving_pauses_with_reason(self):
        """走行中（MANUAL/RUN）に /ui/active_screen が途絶えると、窓
        （ui_active_window_s）経過後に PAUSE＋pause_reason=presence_lost。
        再び流し始めても PAUSE のまま（勝手に走らない）。"""
        self._publish_presence()
        self._spin(0.3)
        assert self._trigger('ui.enter_mode', {'mode': 'MANUAL'}).accepted
        assert self._wait_mode_state('MANUAL', 'PAUSE')
        # スティック（jog lease）で走行に入る（T-MANUAL-01）。
        self.pub_jog.publish(String(data='presence-test'))
        assert self._wait_mode_state('MANUAL', 'RUN', timeout=3.0), \
            'MANUAL/RUN に入らない'
        assert self._latest().pause_reason == ''
        # 流し続けている間は走行のまま。
        for _ in range(4):
            self._publish_presence()
            self._spin(1.0)
        assert (self._latest().mode, self._latest().state) == ('MANUAL', 'RUN')

        # 流すのを止める → 窓の経過後に PAUSE＋理由。速度上限は stop。
        assert self._wait_mode_state(
            'MANUAL', 'PAUSE', timeout=UI_ACTIVE_WINDOW_S + 5.0), \
            '在席喪失で PAUSE に落ちない（SG-A12）'
        snap = self._latest()
        assert snap.pause_reason == 'presence_lost', snap.pause_reason
        assert snap.speed_limit == 'stop', snap.speed_limit

        # 再び流し始めても PAUSE のまま（勝手に走らない）。理由も保つ。
        for _ in range(4):
            self._publish_presence()
            self._spin(1.0)
        snap = self._latest()
        assert (snap.mode, snap.state) == ('MANUAL', 'PAUSE'), \
            '端末が戻っただけで走り出した（SD-8 違反）'
        assert snap.pause_reason == 'presence_lost'

        # 「確認」は受理される（MANUAL は ack_only → PAUSE のまま。生き残る）。
        res = self._trigger('ui.resume_ack')
        assert res.accepted, res.reject_reason_key
        assert self._trigger('ui.finish').accepted
        assert self._wait_mode('IDLE')

    def test_presence_lost_while_idle_changes_nothing(self):
        """走行中でなければ在席喪失でも状態は変わらない（速度上限 0 だけ）。
        確認は求めない。"""
        self._publish_presence()
        self._spin(0.3)
        assert (self._latest().mode, self._latest().state) == ('IDLE', 'NONE')
        self._spin(UI_ACTIVE_WINDOW_S + 2.0)
        snap = self._latest()
        assert (snap.mode, snap.state) == ('IDLE', 'NONE'), \
            '走行中でないのに在席喪失で動いた'
        assert snap.pause_reason == '', snap.pause_reason

    def test_presence_lost_does_not_fire_without_prior_presence(self):
        """起動直後（一度も使用中の端末が無い）の _on_timer では、走行中の
        mode/state を積んでいても発火しない（「前回が使用中あり」の条件）。
        変異「前回条件を外す」はここが赤くなる。"""
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
            node.mode = 'FOLLOW'
            node.state = 'RUN'
            node._presence_active = False
            node._screens = {}
            node._on_timer()
            assert (node.mode, node.state) == ('FOLLOW', 'RUN'), \
                '一度も在席が無いのに presence_lost が発火した'
            # 前回あり → 今なしなら発火する（本番の経路。ガードの二重化）。
            node._presence_active = True
            node._on_timer()
            assert (node.mode, node.state) == ('FOLLOW', 'PAUSE'), \
                '在席喪失で PAUSE に落ちない'
            assert node._pause_reason == 'presence_lost', node._pause_reason
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
