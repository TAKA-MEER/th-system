"""
test_measure_link_node.py
=========================
8-g P1 — 本物の `measure_link` と `state_manager` を起動して縛る試験。
正本は docs/plan/detailed/DetailedDesign-measure.md §3・§6.1。

起動は「生成 yaml を先に渡す」本番の経路（registry → params_generation →
生成 yaml → ノード。CLAUDE.md「rclpy の declare_parameter は既定値の型で
決まり…」）。代役は heartbeat の送り手だけ。

  a. heartbeat 無し → /measure/command は link_lost。status.link_ok=false
  b. 版 "2" の heartbeat → version_mismatch（版一致は一度も数えない）
  c. 版 "1" の heartbeat 5 Hz → link_ok=true、meas.goal は
     state_not_accepting（遷移表に meas.* の行が無いため。P1）
  d. heartbeat を止める → timeout 後に link_lost に戻る
  e. ui.*・command_id 空・meas.translate の side=LEFT・未知の meas.* は
     すべて bad_argument
  f. /system/trigger に meas.goal → 拒否（入口の分離。§5）
  g. /measure/status は transient_local で後から購読しても 1 件受け取れる
     （publisher の QoS も直接確かめる。10 Hz 周期発行だけでは volatile の
     変異を落とせないため）。mode は /system/state と一致する

注意（launch_testing の挙動）: このファイルは @pytest.mark.launch_test を
持つため、launch_testing の pytest プラグインがファイル全体を1つの pytest
アイテムとして収集し、その中の unittest.TestCase だけを実行する。
モジュール直下の素の pytest 関数は収集されないので、全部メソッドにする。
メソッドは名前順に実行される（a→g の順に積む）。
"""
import json
import os
import sys
import tempfile
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from th_system_msgs.msg import (MeasureHeartbeat, MeasureStatus, StateEvent,
                                SystemState)
from th_system_msgs.srv import MeasureCommand, UiTrigger

# params_generation の取り込み（test_sgb21_registry_wiring.py と同じ流儀）。
_THIS_DIR = os.path.dirname(__file__)
_LAUNCH_DIR = os.path.abspath(os.path.join(
    _THIS_DIR, '..', '..', 'th_bringup', 'launch'))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
import params_generation as pg  # noqa: E402

_GEN_DIR = None


def _generated_yaml(name):
    global _GEN_DIR
    if _GEN_DIR is None:
        _GEN_DIR = tempfile.mkdtemp(prefix='measure_link_test_gen_')
        out_dir = os.path.join(_GEN_DIR, 'generated')
        pg.run_generation(
            stage=4, sim=False, nodes=list(pg.REGISTRY_NODES), out_dir=out_dir,
            registry_path=os.path.join(
                _THIS_DIR, '..', '..', 'th_params', 'config', 'registry.yaml'),
            calib_dir=os.path.join(_GEN_DIR, 'no_calib'),
            overrides_path=os.path.join(_GEN_DIR, 'no_overrides.yaml'))
    return os.path.join(_GEN_DIR, 'generated', name)


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[_generated_yaml('state_manager.yaml')],
        output='screen',
    )
    measure_link = launch_ros.actions.Node(
        package='th_measure',
        executable='measure_link.py',
        name='measure_link',
        parameters=[_generated_yaml('measure_link.yaml')],
        output='screen',
    )
    return launch.LaunchDescription([
        state_manager,
        measure_link,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_manager, 'measure_link': measure_link}


_LATCHED_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestMeasureLinkNode(unittest.TestCase):

    _cid = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_measure_link_node')
        self._statuses = []
        self.node.create_subscription(
            MeasureStatus, '/measure/status', self._statuses.append, _LATCHED_QOS)
        self._states = []
        self.node.create_subscription(
            SystemState, '/system/state', self._states.append, _LATCHED_QOS)
        self._events = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._events.append, 10)

        self.pub_hb = self.node.create_publisher(
            MeasureHeartbeat, '/measure/heartbeat', 10)
        self._hb_version = None    # None = 送らない
        self._hb_seq = 0
        self._hb_timer = self.node.create_timer(0.2, self._publish_hb)

        self.cli_measure = self.node.create_client(
            MeasureCommand, '/measure/command')
        self.cli_trigger = self.node.create_client(
            UiTrigger, '/system/trigger')
        assert self.cli_measure.wait_for_service(timeout_sec=10.0)
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0)
        # ノードの起動・discovery の余裕（受信の取りこぼしを避ける）。
        self._spin(1.0)

    def tearDown(self):
        self._hb_version = None
        if self._hb_timer is not None:
            self._hb_timer.cancel()
            self.node.destroy_timer(self._hb_timer)
            self._hb_timer = None
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_hb(self):
        if self._hb_version is None:
            return
        self._hb_seq += 1
        msg = MeasureHeartbeat()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.interface_version = self._hb_version
        msg.sender = 'fk-system'
        msg.seq = self._hb_seq
        self.pub_hb.publish(msg)

    def _next_cid(self):
        type(self)._cid += 1
        return f"test-{type(self)._cid}"

    def _call_measure(self, command, command_id=None, side=""):
        req = MeasureCommand.Request()
        req.command_id = self._next_cid() if command_id is None else command_id
        req.command = command
        req.side = side
        future = self.cli_measure.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert future.done(), f'/measure/command が返らない: {command}'
        return future.result()

    def _call_trigger(self, trigger):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = '{}'
        req.requester = 'test'
        future = self.cli_trigger.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert future.done(), f'/system/trigger が返らない: {trigger}'
        return future.result()

    def _latest_status(self):
        self._spin(0.2)
        assert self._statuses, '/measure/status を1件も受信していない'
        return self._statuses[-1]

    def _wait_status_link_ok(self, want, timeout=6.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._statuses and self._statuses[-1].link_ok == want:
                return True
        return False

    def _wait_command_reason(self, command, want_reason, timeout=6.0, side=""):
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            last = self._call_measure(command, side=side)
            if last.reject_reason == want_reason:
                return last
            self._spin(0.2)
        return last

    # ════════════════════════════════════════════════════════
    def test_a_no_heartbeat_link_lost(self):
        """a. heartbeat 無し → link_lost / status.link_ok=false。"""
        self._hb_version = None
        res = self._wait_command_reason("meas.goal", "link_lost")
        assert res is not None and res.accepted is False
        assert res.reject_reason == "link_lost", res.reject_reason
        status = self._latest_status()
        assert status.link_ok is False
        # evt.measure_link_lost が来ていることも確かめる（§6.1 の渡し方）。
        lost = [e for e in self._events
                if e.event == 'evt.measure_link_lost'
                and e.source_node == 'measure_link']
        assert lost, 'evt.measure_link_lost が出ていない'
        arg = json.loads(lost[-1].arg_json)
        assert arg.get("reason") in ("never", "timeout", "version_mismatch"), arg

    def test_b_version_mismatch(self):
        """b. 版 "2" だけ → version_mismatch（数えない）。"""
        self._hb_version = "2"
        res = self._wait_command_reason("meas.goal", "version_mismatch")
        assert res is not None and res.accepted is False
        assert res.reject_reason == "version_mismatch", res.reject_reason
        assert self._wait_status_link_ok(False, timeout=2.0) is True

    def test_c_valid_heartbeat_state_not_accepting(self):
        """c. 版 "1" 5 Hz → link_ok=true、meas.goal は state_not_accepting。"""
        self._hb_version = "1"
        assert self._wait_status_link_ok(True) is True, 'link_ok にならない'
        res = self._call_measure("meas.goal")
        assert res.accepted is False
        assert res.reject_reason == "state_not_accepting", res.reject_reason
        # 応答に mode / state が入っている（§3.1）。
        assert res.mode and res.state
        # 仕上げに evt.measure_link_ok が出ていることも確かめる。
        ok = [e for e in self._events if e.event == 'evt.measure_link_ok']
        assert ok, 'evt.measure_link_ok が出ていない'

    def test_d_heartbeat_stop_timeout_link_lost(self):
        """d. heartbeat を止める → timeout 後に link_lost に戻る。"""
        assert self._hb_version == "1"
        self._hb_version = None
        assert self._wait_status_link_ok(False) is True, \
            'timeout 後に link_ok が落ちない'
        res = self._wait_command_reason("meas.goal", "link_lost")
        assert res is not None and res.reject_reason == "link_lost", \
            (None if res is None else res.reject_reason)

    def test_e_bad_argument(self):
        """e. ui.*・command_id 空・side=LEFT・未知の meas.* は bad_argument。"""
        self._hb_version = "1"
        assert self._wait_status_link_ok(True) is True
        res = self._call_measure("ui.anything")
        assert res.accepted is False and res.reject_reason == "bad_argument", \
            (res.accepted, res.reject_reason)
        res = self._call_measure("meas.goal", command_id="")
        assert res.accepted is False and res.reject_reason == "bad_argument"
        res = self._call_measure("meas.translate", side="LEFT")
        assert res.accepted is False and res.reject_reason == "bad_argument"
        res = self._call_measure("meas.unknown")
        assert res.accepted is False and res.reject_reason == "bad_argument"

    def test_f_trigger_meas_rejected(self):
        """f. /system/trigger に meas.goal → 拒否（入口の分離。§5）。"""
        res = self._call_trigger("meas.goal")
        assert res.accepted is False, 'meas.* が /system/trigger で受理された'

    def test_g_status_durability_and_mode(self):
        """g. transient_local で後から購読しても 1 件受け取れる。mode 一致。"""
        # publisher の QoS を直接確かめる（10 Hz 周期発行だけでは volatile の
        # 変異を落とせないため）。
        infos = self.node.get_publishers_info_by_topic('/measure/status')
        ml = [i for i in infos if i.node_name == 'measure_link']
        assert ml, 'measure_link の /measure/status publisher が見つからない'
        assert ml[0].qos_profile.durability == \
            QoSDurabilityPolicy.TRANSIENT_LOCAL, ml[0].qos_profile
        # 後から購読しても 1 件受け取れる。
        late = rclpy.create_node('test_measure_link_late')
        try:
            got = []
            late.create_subscription(
                MeasureStatus, '/measure/status', got.append, _LATCHED_QOS)
            deadline = time.time() + 3.0
            while not got and time.time() < deadline:
                rclpy.spin_once(late, timeout_sec=0.05)
            assert len(got) >= 1, '後から購読しても /measure/status が届かない'
        finally:
            late.destroy_node()
        # mode は /system/state と一致する。
        status = self._latest_status()
        self._spin(0.2)
        assert self._states, '/system/state を1件も受信していない'
        assert status.mode == self._states[-1].mode, (
            status.mode, self._states[-1].mode)
        assert status.interface_version == "1"
