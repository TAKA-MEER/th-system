"""
test_auto_measure_node.py
=========================
8-g P5a — 本物の `state_manager` と `measure_link` を起動して縛る試験。
正本は docs/plan/spec/Spec-modes.md（-120・-130・-131・-135・§6）と
docs/plan/spec/Spec-measure.md、詳細は DetailedDesign-measure.md §6.2。

起動は「生成 yaml を先に渡す」本番の経路（P1 の test_measure_link_node.py と同じ）。
代役は heartbeat の送り手だけ。

  a. 生存確認なしで ui.goto AUTO_MEASURE → 拒否 measure_link_lost
  b. 版 "1" の生存確認ありで入れる → /system/state と /measure/status が AUTO_MEASURE/WAIT
  c. meas.goal は state_not_accepting、meas.set_limits（v 0.1）→ DONE が
     /measure/result にちょうど 1 回、/measure/status.v_limit_mps が 0.1 以下
  d. 同じ command_id をもう一度 → bad_argument、/measure/result は増えない
  e. meas.finish → IDLE、/measure/result に DONE
  f. 入り直して ui.jog.hold で PAUSE → ui.resume_yes では戻らない → meas.start で WAIT
  g. 生存確認を止めても WAIT のまま（-133 は走行中だけ）
  h. evt.measure_link_lost は値が変わったときだけ遷移表へ流れる
     （WAIT で流れたら last_reject_reason が変わる。変わっていない出し直しは変えない）
  i. AUTO_MEASURE → LOCALIZATION_LOST → ESTOP → IDLE では片付けが走る
     （reentry_required false・v_cap 0）
  j. ESTOP から ui.resume_yes で AUTO_MEASURE へ戻ったときは片付けない
     （reentry_required true のまま）

注意（launch_testing の挙動）: ノードはファイル単位で共有されるため、
メソッド同士の状態に頼らない（各メソッドが _ensure_auto_measure で自分の状態を作る）。
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
from std_msgs.msg import String
from th_system_msgs.msg import (FaultStatus, MeasureHeartbeat, MeasureResult,
                                MeasureStatus, StateEvent, SystemState)
from th_system_msgs.srv import MeasureCommand, UiTrigger

# params_generation の取り込み（test_measure_link_node.py と同じ流儀）。
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
        _GEN_DIR = tempfile.mkdtemp(prefix='auto_measure_test_gen_')
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


class TestAutoMeasureNode(unittest.TestCase):

    _cid = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_auto_measure_node')
        self._statuses = []
        self.node.create_subscription(
            MeasureStatus, '/measure/status', self._statuses.append, _LATCHED_QOS)
        self._states = []
        self.node.create_subscription(
            SystemState, '/system/state', self._states.append, _LATCHED_QOS)
        self._results = []
        self.node.create_subscription(
            MeasureResult, '/measure/result', self._results.append, 20)

        self._cmd_states = []
        self.node.create_subscription(
            String, '/measure/command_state', self._cmd_states.append, _LATCHED_QOS)
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 10)
        self.pub_hb = self.node.create_publisher(
            MeasureHeartbeat, '/measure/heartbeat', 10)
        self._hb_version = None    # None = 送らない
        self._hb_seq = 0
        self._hb_timer = self.node.create_timer(0.2, self._publish_hb)

        self.pub_event = self.node.create_publisher(
            StateEvent, '/system/event', 10)
        self.cli_measure = self.node.create_client(
            MeasureCommand, '/measure/command')
        self.cli_trigger = self.node.create_client(
            UiTrigger, '/system/trigger')
        assert self.cli_measure.wait_for_service(timeout_sec=10.0)
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0)
        self._spin(1.0)
        self._drive_to_idle()

    def tearDown(self):
        self._hb_version = None
        if self._hb_timer is not None:
            self._hb_timer.cancel()
            self.node.destroy_timer(self._hb_timer)
            self._hb_timer = None
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _publish_fault(self, active, severity, fault_type='LOCALIZATION_LOST'):
        msg = FaultStatus()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.active = bool(active)
        msg.fault_type = fault_type
        msg.description = 'test'
        msg.severity = severity
        self.pub_fault.publish(msg)

    def _latest_cmd_state(self):
        self._spin(0.2)
        assert self._cmd_states, '/measure/command_state を1件も受信していない'
        return json.loads(self._cmd_states[-1].data)

    def _wait_cmd_state(self, pred, timeout=8.0, desc=''):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.2)
            if self._cmd_states and pred(json.loads(self._cmd_states[-1].data)):
                return True
        return False

    def _drive_to_idle(self):
        """起動直後の INIT/CHECK から IDLE へ出す（connectivity_checker の代役）。

        ノードはファイル単位で共有されるため、既に INIT でなければ何もしない。"""
        self._spin(0.3)
        if not self._states or self._states[-1].mode != 'INIT':
            return
        deadline = time.time() + 8.0
        while time.time() < deadline:
            msg = StateEvent()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.event = 'evt.link_ok'
            msg.source_node = 'connectivity_checker'
            msg.arg_json = '{}'
            self.pub_event.publish(msg)
            self._spin(0.3)
            if self._states and self._states[-1].mode == 'IDLE':
                return
        # 起動猶予で何度かリスタートする前の到達を期待する。
        assert self._states and self._states[-1].mode == 'IDLE', \
            (self._states[-1].mode, self._states[-1].state) if self._states else 'no state'

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
        return f"am-{type(self)._cid}"

    def _call_measure(self, command, command_id=None, v_max=0.0, w_max=0.0):
        req = MeasureCommand.Request()
        req.command_id = self._next_cid() if command_id is None else command_id
        req.command = command
        req.v_max_mps = float(v_max)
        req.w_max_radps = float(w_max)
        future = self.cli_measure.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert future.done(), f'/measure/command が返らない: {command}'
        return future.result()

    def _call_trigger(self, trigger, arg_json='{}'):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = arg_json
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

    def _latest_state(self):
        self._spin(0.2)
        assert self._states, '/system/state を1件も受信していない'
        return self._states[-1]

    def _wait_status_link_ok(self, want, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._statuses and self._statuses[-1].link_ok == want:
                return True
        return False

    def _wait_mode_state(self, mode, state, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._states and (self._states[-1].mode, self._states[-1].state) == (mode, state):
                return True
        return False

    def _goto_auto_measure(self):
        return self._call_trigger(
            'ui.goto', json.dumps({"kind": "AUTO_MEASURE"}))

    def _ensure_auto_measure_wait(self):
        """自分の前提を作る：heartbeat を版 "1" で送り、AUTO_MEASURE/WAIT に入る。"""
        self._hb_version = "1"
        assert self._wait_status_link_ok(True) is True, 'link_ok にならない'
        st = self._latest_state()
        if (st.mode, st.state) != ("AUTO_MEASURE", "WAIT"):
            if (st.mode, st.state) == ("AUTO_MEASURE", "PAUSE"):
                res = self._call_measure("meas.start")
                assert res.accepted is True, res.reject_reason
            else:
                res = self._goto_auto_measure()
                assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state("AUTO_MEASURE", "WAIT") is True
        # ノードはメソッド間で共有されるため、前の試験の last_reject_reason を
        # 自分で消して前提を確定させる（ui.screen は全モードで受理される）。
        res = self._call_trigger('ui.screen', json.dumps({"screen_id": "S-21"}))
        assert res.accepted is True
        deadline = time.time() + 4.0
        while time.time() < deadline:
            self._spin(0.2)
            if self._states and self._states[-1].last_reject_reason == "":
                return
        assert self._states and self._states[-1].last_reject_reason == "", \
            self._states[-1].last_reject_reason if self._states else 'no state'

    # ════════════════════════════════════════════════════════
    def test_a_entry_without_link_rejected(self):
        """a. 生存確認なしで ui.goto AUTO_MEASURE → 拒否 measure_link_lost。"""
        self._hb_version = None
        res = self._goto_auto_measure()
        assert res.accepted is False
        assert res.reject_reason_key == "measure_link_lost", res.reject_reason_key

    def test_b_entry_with_link_to_wait(self):
        """b. 版 "1" の生存確認ありで入れる。/system/state と /measure/status が一致。"""
        self._hb_version = "1"
        assert self._wait_status_link_ok(True) is True, 'link_ok にならない'
        res = self._goto_auto_measure()
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state("AUTO_MEASURE", "WAIT") is True
        status = self._latest_status()
        assert (status.mode, status.state) == ("AUTO_MEASURE", "WAIT"), \
            (status.mode, status.state)

    def test_c_goal_rejected_set_limits_done(self):
        """c. meas.goal は state_not_accepting。meas.set_limits → DONE 1 回。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure("meas.goal", command_id=cid)
        assert res.accepted is False
        assert res.reject_reason == "state_not_accepting", res.reject_reason
        self._spin(0.3)
        assert not [r for r in self._results if r.command_id == cid], \
            '拒否された指示に結果が出ている'

        cid = self._next_cid()
        before = len([r for r in self._results if r.command_id == cid])
        res = self._call_measure("meas.set_limits", command_id=cid, v_max=0.1)
        assert res.accepted is True, res.reject_reason
        self._spin(0.5)
        done = [r for r in self._results if r.command_id == cid]
        assert len(done) - before == 1, done
        assert done[0].result == "DONE"
        # v_limit は v_cap(0.1) 以下のうち小さい方（本システムの上限も効く）
        deadline = time.time() + 4.0
        ok = False
        while time.time() < deadline:
            self._spin(0.2)
            if self._statuses and self._statuses[-1].v_limit_mps <= 0.1:
                ok = True
                break
        assert ok, ('/measure/status.v_limit_mps が 0.1 以下にならない: '
                    + str(self._statuses[-1].v_limit_mps if self._statuses else None))

    def test_d_duplicate_command_id_bad_argument(self):
        """d. 同じ command_id をもう一度 → bad_argument。/measure/result は増えず遷移しない。

        台帳の順（受理判定 → 台帳 → 遷移）を戻すと、重複した meas.start が
        先に遷移して WAIT に戻る（受理判定のときは PAUSE のはず）。"""
        self._ensure_auto_measure_wait()
        # meas.start の重複版：遷移しないことも縛る。
        cid = self._next_cid()
        res = self._call_trigger('ui.jog.hold')
        assert res.accepted is True
        assert self._wait_mode_state("AUTO_MEASURE", "PAUSE") is True
        res = self._call_measure("meas.start", command_id=cid)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state("AUTO_MEASURE", "WAIT") is True
        res = self._call_trigger('ui.jog.hold')
        assert res.accepted is True
        assert self._wait_mode_state("AUTO_MEASURE", "PAUSE") is True
        count_before = len(self._results)
        res = self._call_measure("meas.start", command_id=cid)
        assert res.accepted is False
        assert res.reject_reason == "bad_argument", res.reject_reason
        self._spin(0.5)
        assert len(self._results) == count_before, \
            '重複した command_id に結果が増えている'
        st = self._latest_state()
        assert (st.mode, st.state) == ("AUTO_MEASURE", "PAUSE")
        # meas.set_limits の重複版：結果が増えないことも縛る。
        cid = self._next_cid()
        res = self._call_measure("meas.set_limits", command_id=cid, v_max=0.1)
        assert res.accepted is True, res.reject_reason
        self._spin(0.5)
        count_before = len(self._results)
        res = self._call_measure("meas.set_limits", command_id=cid, v_max=0.1)
        assert res.accepted is False
        assert res.reject_reason == "bad_argument", res.reject_reason
        self._spin(0.5)
        assert len(self._results) == count_before, \
            '重複した command_id に結果が増えている'
        st = self._latest_state()
        assert (st.mode, st.state) == ("AUTO_MEASURE", "PAUSE")

    def test_e_finish_to_idle_with_done(self):
        """e. meas.finish → IDLE。/measure/result に DONE。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure("meas.finish", command_id=cid)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state("IDLE", "NONE") is True
        self._spin(0.5)
        done = [r for r in self._results if r.command_id == cid]
        assert len(done) == 1 and done[0].result == "DONE", done

    def test_f_pause_resume_yes_stays_start_returns(self):
        """f. ui.jog.hold で PAUSE → ui.resume_yes では戻らない → meas.start で WAIT。"""
        self._ensure_auto_measure_wait()
        res = self._call_trigger('ui.jog.hold')
        assert res.accepted is True
        assert self._wait_mode_state("AUTO_MEASURE", "PAUSE") is True
        res = self._call_trigger('ui.resume_yes')
        assert res.accepted is False
        st = self._latest_state()
        assert (st.mode, st.state) == ("AUTO_MEASURE", "PAUSE")
        res = self._call_measure("meas.start")
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state("AUTO_MEASURE", "WAIT") is True

    def test_g_link_loss_keeps_wait(self):
        """g. 生存確認を止めても WAIT のまま（-133 は走行中だけ）。"""
        self._ensure_auto_measure_wait()
        self._hb_version = None
        assert self._wait_status_link_ok(False) is True, 'link_ok が落ちない'
        self._spin(2.0)  # state_manager に lost が届いたあとも
        st = self._latest_state()
        assert (st.mode, st.state) == ("AUTO_MEASURE", "WAIT"), (st.mode, st.state)

    def test_h_link_lost_flows_only_on_change(self):
        """h. evt.measure_link_lost は値が変わったときだけ遷移表へ流れる。

        WAIT で流れたら last_reject_reason が "not_allowed" に変わる
        （行が無いため）。受理される操作で "" に戻したあと、変わっていない
        出し直しを送っても変わらない（変異=毎回流すではここが赤になる）。"""
        self._ensure_auto_measure_wait()
        st = self._latest_state()
        assert st.last_reject_reason == "", st.last_reject_reason

        self._hb_version = None
        assert self._wait_status_link_ok(False) is True, 'link_ok が落ちない'
        deadline = time.time() + 8.0
        changed = False
        while time.time() < deadline:
            self._spin(0.2)
            if self._states and self._states[-1].last_reject_reason == "not_allowed":
                changed = True
                break
        assert changed, ('値が変わった evt.measure_link_lost が遷移表に流れない'
                         '（last_reject_reason が not_allowed にならない）')

        # 受理される操作（ui.screen は状態を変えない自己ループ）で "" に戻す。
        res = self._call_trigger('ui.screen', json.dumps({"screen_id": "S-21"}))
        assert res.accepted is True
        deadline = time.time() + 4.0
        cleared = False
        while time.time() < deadline:
            self._spin(0.2)
            if self._states and self._states[-1].last_reject_reason == "":
                cleared = True
                break
        assert cleared, 'last_reject_reason が "" に戻らない'

        # 変わっていない evt.measure_link_lost の出し直しを直接送る。
        pub = self.node.create_publisher(StateEvent, '/system/event', 10)
        msg = StateEvent()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.event = 'evt.measure_link_lost'
        msg.source_node = 'measure_link'
        msg.arg_json = json.dumps({"reason": "timeout"})
        pub.publish(msg)
        self._spin(1.0)
        st = self._latest_state()
        assert st.last_reject_reason == "", st.last_reject_reason
        pub.destroy()

    def test_i_cleanup_via_estop_to_idle(self):
        """i. AUTO_MEASURE → LOCALIZATION_LOST → ESTOP → IDLE では片付けが走る。

        AUTO_MEASURE → ESTOP → IDLE（C-09d）で /measure/command_state の
        reentry_required が false、v_cap_mps が 0 になる（-142「抜けて入り直すまで」）。"""
        self._ensure_auto_measure_wait()
        res = self._call_measure("meas.set_limits", v_max=0.1)
        assert res.accepted is True, res.reject_reason
        assert self._wait_cmd_state(
            lambda c: abs(c.get('v_cap_mps', -1) - 0.1) < 1e-6) is True, \
            self._latest_cmd_state()

        self._publish_fault(True, 'CRITICAL')
        assert self._wait_mode_state("ESTOP", "NONE") is True
        assert self._wait_cmd_state(
            lambda c: c.get('reentry_required') is True) is True, \
            self._latest_cmd_state()

        self._publish_fault(False, 'RECOVERABLE')
        res = self._call_trigger('ui.resume_no')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state("IDLE", "NONE") is True
        assert self._wait_cmd_state(
            lambda c: c.get('reentry_required') is False
            and c.get('v_cap_mps') == 0.0
            and c.get('w_cap_radps') == 0.0) is True, \
            self._latest_cmd_state()

    def test_j_return_to_auto_measure_keeps_reentry(self):
        """j. ESTOP から ui.resume_yes で AUTO_MEASURE に戻ったときは片付けない。

        C-09c-generic の復帰では reentry_required が true のまま、v_cap も残る。"""
        self._ensure_auto_measure_wait()
        res = self._call_measure("meas.set_limits", v_max=0.1)
        assert res.accepted is True, res.reject_reason
        assert self._wait_cmd_state(
            lambda c: abs(c.get('v_cap_mps', -1) - 0.1) < 1e-6) is True

        self._publish_fault(True, 'CRITICAL')
        assert self._wait_mode_state("ESTOP", "NONE") is True
        assert self._wait_cmd_state(
            lambda c: c.get('reentry_required') is True) is True

        self._publish_fault(False, 'RECOVERABLE')
        res = self._call_trigger('ui.resume_yes')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state("AUTO_MEASURE", "WAIT") is True
        self._spin(1.0)
        c = self._latest_cmd_state()
        assert c.get('reentry_required') is True, c
        assert abs(c.get('v_cap_mps', -1) - 0.1) < 1e-6, c
