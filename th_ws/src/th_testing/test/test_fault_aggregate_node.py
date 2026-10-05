"""
test_fault_aggregate_node.py — SG-A8「フォルトの集約」

safety_monitor は active_faults_ を集合で持つのに、どれか 1 つが消えると
publishFault(false, "NONE") を出していた（safety_monitor.cpp updateFaultState）。
state_manager は /safety/fault の最後のメッセージで上書きするので、
フォルト A・B が同時に出ていて A だけ消えると B が残っているのに「解消」と
判定され、W-1 の「はい」で走行状態へ戻れた。

正しい振る舞い（このファイルが縛る）:
  1. 1 つ消えても他が残っていれば /safety/fault は active=true を出し続ける。
     残っているもののうち最も重いもの（重大 > 回復）を代表として出す。
     全部消えたときだけ active=false, NONE。
  2. state_manager を含む本番の経路で「A・B 発生 → A 解消 →
     ui.resume_yes を送っても走行状態へ戻らない」を縛る。

safety_monitor と state_manager を実際に起動する launch_testing。
2 つ同時に起こしやすい組み合わせ（LiDAR 途絶 + ESP32 途絶。どちらも回復）と、
重大 (LIMITER_DEAD) + 回復 (LIDAR_LOST) の組み合わせを使う。
"""

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

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from th_system_msgs.msg import (FaultStatus, LimiterStatus, StateEvent, SystemState,
                                WheelFeedback)
from th_system_msgs.srv import UiTrigger


LIDAR_TIMEOUT_MS = 400
ESP32_TIMEOUT_MS = 400
LIMITER_DEAD_MS = 1500
CRITICAL_HOLD_MS = 100
JOG_LEASE_MS = 300
LINK_WAIT_TIMEOUT_MS = 60_000
UI_ACTIVE_WINDOW_S = 5
SCREEN_STALE_MS = 60_000

_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def _generated_yaml(node: str) -> str:
    for d in (os.environ.get('TH_GENERATED_DIR', ''), '/root/th_data/generated',
              os.path.join(_WS_ROOT, 'data', 'generated')):
        path = os.path.join(d, f'{node}.yaml') if d else ''
        if path and os.path.exists(path):
            return path
    raise FileNotFoundError(f'生成 yaml {node}.yaml が見つからない')


@pytest.mark.launch_test
def generate_test_description():
    # 本番の bringup と同じ生成 yaml を先に読ませ、短縮値は dict で後から上書きする。
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[_generated_yaml('safety_monitor'), {
            'lidar_timeout_ms': LIDAR_TIMEOUT_MS,
            'esp32_timeout_ms': ESP32_TIMEOUT_MS,
            'limiter_dead_ms': LIMITER_DEAD_MS,
            'critical_fault_hold_ms': CRITICAL_HOLD_MS,
            'check_period_ms': 50,
            'startup_grace_sec': 1,
            'enabled_targets': ['lidar', 'esp32', 'limiter'],
        }],
        output='screen',
    )
    state_manager = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[_generated_yaml('state_manager'), {
            'jog_lease_ms': JOG_LEASE_MS,
            'link_wait_timeout_ms': LINK_WAIT_TIMEOUT_MS,
            'ui_active_window_s': UI_ACTIVE_WINDOW_S,
            'screen_stale_ms': SCREEN_STALE_MS,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        safety,
        state_manager,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety, 'state_manager': state_manager}


_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


def _scan_msg(stamp):
    msg = LaserScan()
    msg.header.stamp = stamp
    msg.angle_min = -3.14
    msg.angle_max = 3.14
    msg.angle_increment = 0.01
    msg.range_min = 0.1
    msg.range_max = 12.0
    msg.ranges = [1.0] * 628
    return msg


class _FaultDriver:
    """safety_monitor の 3 入力（scan / wheel_feedback / limiter_status）を操る。"""

    def __init__(self, node):
        self.node = node
        self.faults: list[FaultStatus] = []
        self.locks: list[bool] = []
        node.create_subscription(
            FaultStatus, '/safety/fault', self.faults.append, 10)
        node.create_subscription(
            Bool, '/safety/fault_lock', lambda m: self.locks.append(m.data), 10)
        self.pub_scan = node.create_publisher(LaserScan, '/scan', 10)
        self.pub_wf = node.create_publisher(
            WheelFeedback, '/esp32/wheel_feedback', 10)
        self.pub_limiter = node.create_publisher(
            LimiterStatus, '/safety/limiter_status', 10)
        self.pub_hw = node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_ui_estop = node.create_publisher(Bool, '/safety/estop_ui', 10)

    def pump(self, sec: float, keep_alive=(), period: float = 0.05):
        deadline = time.time() + sec
        while time.time() < deadline:
            for p in keep_alive:
                p()
            rclpy.spin_once(self.node, timeout_sec=period)

    def pub_scan_once(self):
        self.pub_scan.publish(_scan_msg(self.node.get_clock().now().to_msg()))

    def pub_wf_once(self):
        self.pub_wf.publish(WheelFeedback())

    def pub_limiter_once(self):
        self.pub_limiter.publish(LimiterStatus(alive=True, action='PASS'))

    def all_alive(self):
        return (self.pub_scan_once, self.pub_wf_once, self.pub_limiter_once)

    def go_alive_and_clean(self, timeout: float = 60.0, stable_sec: float = 2.0):
        """送達ハンドシェイク付きの前提条件回復。

        1. 全入力を止めてフォルトが立つのを待つ（monitor→test 方向の証明）。
           edge を取り逃がしていても /safety/fault_lock（常時 20Hz）で active を見る。
        2. 3 入力を送り続け、/safety/fault_lock が stable_sec 連続で false のまま
           なのを確認する（test→monitor 方向の証明。単発の false では、
           scan がバースト的に届く不安定な送達での瞬間的解消を拾ってしまう）。
        3. バッファを空にして返す。
        """
        deadline = time.time() + timeout
        seen_active = False
        while time.time() < deadline:
            self.pump(0.1)
            if any(f.active for f in self.faults):
                seen_active = True
                break
            if self.locks and self.locks[-1] is True:
                seen_active = True
                break
        assert seen_active, \
            '入力停止後にフォルトが立たない（safety_monitor へ届いていないか受信できていない）'
        self.faults.clear()
        self.locks.clear()
        stable_since = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.pump(0.1, self.all_alive())
            if self.locks and self.locks[-1] is True:
                stable_since = None
            elif self.locks:
                if stable_since is None:
                    stable_since = time.time()
                if time.time() - stable_since >= stable_sec:
                    break
        assert stable_since is not None and (time.time() - stable_since) >= stable_sec, \
            'alive 送信後も /safety/fault_lock が false に安定しない' \
            f'（入力の送達が不安定。{self.fault_summary()}）'
        self.faults.clear()
        self.locks.clear()

    def wait_for_fault(self, fault_type: str, timeout: float = 8.0,
                       keep_alive=()) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.pump(0.1, keep_alive)
            for f in self.faults:
                if f.active and f.fault_type == fault_type:
                    return True
        return False

    def fault_summary(self) -> str:
        kinds = [(f.active, f.fault_type, f.severity) for f in self.faults]
        return f'n={len(self.faults)} locks={len(self.locks)} last_lock={self.locks[-1] if self.locks else None} kinds={kinds[-12:]}'

    def wait_for_cleared(self, timeout: float = 5.0, keep_alive=()) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.pump(0.1, keep_alive)
            for f in reversed(self.faults):
                if not f.active:
                    return True
        return False


class TestFaultAggregateMessage(unittest.TestCase):
    """SG-A8 の safety_monitor 側: 1 つ消えても他が残れば active=true のまま。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_fault_aggregate_msg')
        self.drv = _FaultDriver(self.node)
        time.sleep(3.5)  # 起動猶予 + discovery（test_safety_monitor と同じ）
        self.drv.go_alive_and_clean()

    def tearDown(self):
        self.node.destroy_node()

    def test_two_recoverable_one_clears_stays_active(self):
        """LIDAR_LOST + ESP32_DISCONNECTED → scan だけ再開しても解消にならない。"""
        drv = self.drv
        # 両方を途絶させる（limiter だけ alive 維持）
        assert drv.wait_for_fault(
            'LIDAR_LOST', keep_alive=(drv.pub_limiter_once,)), \
            f'LIDAR_LOST が立たなかった: {drv.fault_summary()}'
        assert drv.wait_for_fault(
            'ESP32_DISCONNECTED', keep_alive=(drv.pub_limiter_once,)), \
            'ESP32_DISCONNECTED が立たなかった'
        assert drv.locks and drv.locks[-1] is True, \
            '2 重フォルト中に /safety/fault_lock が true にならなかった'

        mark = len(drv.faults)
        # scan だけ再開（wheel_feedback は止めたまま）→ LIDAR_LOST だけ解消
        keep = (drv.pub_scan_once, drv.pub_limiter_once)
        deadline = time.time() + 5.0
        saw_representative = False
        while time.time() < deadline:
            drv.pump(0.1, keep)
            new = drv.faults[mark:]
            for f in new:
                assert f.active, \
                    f'ESP32_DISCONNECTED が残っているのに active=false ({f.fault_type}) が出た（SG-A8）'
            if new and new[-1].active and new[-1].fault_type == 'ESP32_DISCONNECTED':
                saw_representative = True
                break
        assert saw_representative, \
            'LIDAR_LOST 解消後に残った ESP32_DISCONNECTED の active=true が出なかった（SG-A8）'
        assert drv.locks and drv.locks[-1] is True, \
            'フォルトが残っているのに /safety/fault_lock が外れた'

        # 両方再開 → 初めて active=false
        assert drv.wait_for_cleared(5.0, keep_alive=drv.all_alive()), \
            '両入力の再開後に active=false が出なかった'
        assert drv.faults[-1].fault_type == 'NONE', \
            f"全解消の type が NONE ではない: {drv.faults[-1].fault_type}"

    def test_critical_representative_beats_recoverable(self):
        """LIMITER_DEAD(CRITICAL) + LIDAR_LOST → 重大だけ消えたら回復が代表になる。"""
        drv = self.drv
        # limiter を先に途絶させる（scan + wf は維持）
        assert drv.wait_for_fault(
            'LIMITER_DEAD', timeout=8.0,
            keep_alive=(drv.pub_scan_once, drv.pub_wf_once)), \
            'LIMITER_DEAD が立たなかった'
        crit = [f for f in drv.faults if f.active and f.fault_type == 'LIMITER_DEAD'][-1]
        assert crit.severity == 'CRITICAL', \
            f'LIMITER_DEAD の severity が CRITICAL ではない: {crit.severity}'
        # さらに scan を止めて LIDAR_LOST を重ねる。
        # LIMITER_DEAD が残っているため、追加時の自型は出ず代表（LIMITER_DEAD）
        # が再送される。ここでは「何か active な edge が来た」だけを見る。
        # 種別の特定は次の代表フェーズで行う。
        mark = len(drv.faults)
        deadline = time.time() + 10.0
        new = []
        while time.time() < deadline:
            drv.pump(0.1, (drv.pub_wf_once,))
            new = [f for f in drv.faults[mark:] if f.active]
            if new:
                break
        assert new, f'scan 停止後に active なメッセージが来ない: {drv.fault_summary()}'

        mark = len(drv.faults)
        # limiter だけ再開（scan は止めたまま）→ 代表は LIDAR_LOST のはず
        keep = (drv.pub_limiter_once, drv.pub_wf_once)
        deadline = time.time() + 8.0
        saw_representative = False
        while time.time() < deadline:
            drv.pump(0.1, keep)
            new = drv.faults[mark:]
            for f in new:
                assert f.active, \
                    f'LIDAR_LOST が残っているのに active=false ({f.fault_type}) が出た（SG-A8）'
            if new and new[-1].active and new[-1].fault_type == 'LIDAR_LOST':
                assert new[-1].severity == 'RECOVERABLE', \
                    f"代表の severity が RECOVERABLE ではない: {new[-1].severity}"
                saw_representative = True
                break
        assert saw_representative, \
            'LIMITER_DEAD 解消後に残った LIDAR_LOST の active=true が出なかった（SG-A8）'

        # 全部再開 → 初めて active=false
        assert drv.wait_for_cleared(8.0, keep_alive=drv.all_alive()), \
            '全入力の再開後に active=false が出なかった'


class TestStateManagerAggregateResume(unittest.TestCase):
    """SG-A8 の state_manager 側: A・B 発生 → A 解消 →「はい」でも戻らない。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_fault_aggregate_sm')
        self.drv = _FaultDriver(self.node)
        self._state_history: list[SystemState] = []
        self.node.create_subscription(
            SystemState, '/system/state', self._state_history.append, _STATE_QOS)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0), \
            'state_manager が起動していない'
        time.sleep(3.5)  # 起動猶予 + discovery（test_safety_monitor と同じ）
        self.drv.go_alive_and_clean()
        self._reset_to_idle()
        self._state_history.clear()
        self.drv.faults.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──
    def _spin(self, duration: float = 0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _trigger(self, trigger: str, arg: dict = None, requester: str = 'test'):
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

    def _wait_mode_state(self, mode: str, state: str, timeout: float = 8.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._state_history:
                cur = self._state_history[-1]
                if cur.mode == mode and cur.state == state:
                    return True
        return False

    def _wait_mode_state_keep(self, mode: str, state: str, keep=(),
                              timeout: float = 8.0) -> bool:
        """_wait_mode_state の keep-alive 付き版。待ちの間も入力を出し続け、
        関係ないフォルトの発火でモードが動くのを防ぐ（CARRY/ESTOP 滞在用）。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.drv.pump(0.1, keep)
            if self._state_history:
                cur = self._state_history[-1]
                if cur.mode == mode and cur.state == state:
                    return True
        return False

    def _wait_mode_keep(self, mode: str, keep=(), timeout: float = 8.0) -> bool:
        """モード到達待ち（状態は問わない）。入場直後の SELECT/PAUSE 競合
        （C-03 が先に PAUSE へ落とす）を避けたいときに使う。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.drv.pump(0.1, keep)
            if self._state_history and self._state_history[-1].mode == mode:
                return True
        return False

    def _reset_to_idle(self):
        self.drv.pump(0.5, self.drv.all_alive())
        self.drv.pub_hw.publish(Bool(data=True))
        self._spin(0.2)
        self.drv.pub_hw.publish(Bool(data=False))
        self._spin(0.2)
        self.drv.pub_ui_estop.publish(Bool(data=False))
        self._spin(0.2)
        self._trigger('ui.finish')
        assert self._wait_mode_state('IDLE', 'NONE', timeout=5.0), \
            'IDLE に戻せなかった（前のテストの残留）'

    def test_resume_yes_blocked_while_second_fault_remains(self):
        """FOLLOW で A・B 発生 → A 解消 → ui.resume_yes は拒否される（C-04）。"""
        drv = self.drv
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('FOLLOW', 'SELECT', timeout=5.0)

        # FOLLOW 遷移中は入力を出さないギャップがあるため、その間にフォルトが
        # 立って edge を取り逃がしていることがある。ここで送達を立て直す。
        self.drv.go_alive_and_clean()
        self._state_history.clear()
        self.drv.faults.clear()

        # A・B（LIDAR_LOST + ESP32_DISCONNECTED）を発生させる
        assert drv.wait_for_fault(
            'LIDAR_LOST', keep_alive=(drv.pub_limiter_once,)), \
            f'LIDAR_LOST が立たなかった: {drv.fault_summary()}'
        assert drv.wait_for_fault(
            'ESP32_DISCONNECTED', keep_alive=(drv.pub_limiter_once,)), \
            'ESP32_DISCONNECTED が立たなかった'
        # 回復フォルトで FOLLOW/RUN…ではなく SELECT→PAUSE（C-03・W-1）
        assert self._wait_mode_state('FOLLOW', 'PAUSE', timeout=5.0), \
            '2 重フォルトで FOLLOW/PAUSE に落ちなかった'

        # A（scan）だけ再開 → B が残っているので「はい」は通らない。
        # 代表メッセージ（残った ESP32_DISCONNECTED の active=true）が
        # safety_monitor から出たのを確認してから送る（state_manager の受信順を確定させる）。
        keep = (drv.pub_scan_once, drv.pub_limiter_once)
        mark = len(drv.faults)
        deadline = time.time() + 8.0
        saw_representative = False
        while time.time() < deadline:
            drv.pump(0.1, keep)
            new = drv.faults[mark:]
            if new and new[-1].active and new[-1].fault_type == 'ESP32_DISCONNECTED':
                saw_representative = True
                break
        assert saw_representative, \
            'LIDAR_LOST 解消後に残った ESP32_DISCONNECTED の active=true が出なかった（SG-A8）'
        self._spin(0.5)  # state_manager が _on_fault を処理する余裕
        res = self._trigger('ui.resume_yes')
        assert res.accepted is False, \
            'ESP32_DISCONNECTED が残っているのに ui.resume_yes が通った（SG-A8・C-04）'
        snap = self._latest()
        assert (snap.mode, snap.state) == ('FOLLOW', 'PAUSE'), \
            f'残フォルト中に FOLLOW/PAUSE を離れた: {snap.mode}/{snap.state}'

        # B も解消 → 今度は「はい」で RUN へ戻る
        assert drv.wait_for_cleared(5.0, keep_alive=drv.all_alive()), \
            '全解消の active=false が出なかった'
        res = self._trigger('ui.resume_yes')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('FOLLOW', 'RUN', timeout=5.0), \
            '全解消後の「はい」で FOLLOW/RUN に戻らなかった（C-04）'

    def test_carry_resume_blocked_while_critical_remains(self):
        """重大→回復の順で発生 → 最後の severity は CRITICAL のまま。

        発生時は自型が基本だが、新しいフォルトが残存の最重より軽いときだけ
        代表（重大）を出し直す（2026-10-05 受け入れ検査）。こうしないと
        state_manager の _fault_severity が RECOVERABLE に落ち、
        C-11（CARRY→復帰。guard hw_released_and_no_critical）が重大継続中に
        通ってしまう。最後に全解消→「手押し復帰」で FOLLOW に戻る対照付き。
        """
        drv = self.drv
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert res.accepted, res.reject_reason_key
        # 入場直後は SELECT と PAUSE（C-03 先行）のどちらもありうる。
        # モード到達だけ見て、戻り先はラッチされた prev_* で動的に決める。
        assert self._wait_mode_keep('FOLLOW', drv.all_alive(), timeout=5.0)
        self.drv.go_alive_and_clean()
        self._state_history.clear()
        self.drv.faults.clear()

        # 手押しへ（prev=FOLLOW をラッチ）。離しても CARRY のまま。
        # 待ちの間も入力を出し続ける（無発行ギャップで LIMITER_DEAD 等が
        # 立って ESTOP に運ばれるのを防ぐ）。
        drv.pub_hw.publish(Bool(data=True))
        assert self._wait_mode_state_keep(
            'CARRY', 'NONE', drv.all_alive(), timeout=5.0)
        drv.pub_hw.publish(Bool(data=False))
        drv.pump(0.5, drv.all_alive())
        snap = self._latest()
        assert snap.mode == 'CARRY', 'HW 解除で CARRY を離れた'
        assert snap.prev_mode == 'FOLLOW', 'CARRY 入場時の latch が無い'
        ret_mode, ret_state = snap.prev_mode, snap.prev_state

        # 重大を先に（limiter 停止。scan + wf は維持）
        assert drv.wait_for_fault(
            'LIMITER_DEAD', timeout=10.0,
            keep_alive=(drv.pub_scan_once, drv.pub_wf_once)), \
            f'LIMITER_DEAD が立たなかった: {drv.fault_summary()}'
        assert self._wait_mode_state_keep(
            'ESTOP', 'NONE', (drv.pub_scan_once, drv.pub_wf_once),
            timeout=5.0), \
            '重大フォルトで ESTOP に落ちなかった（C-06a）'
        assert self._latest().prev_mode == 'FOLLOW', \
            'CARRY からの C-06a で prev_* が上書きされた（N-3 違反）'

        # ESTOP 遷移待ちは入力を出さないため余計なフォルトが立っている。
        # 立て直す（全解消。fault.cleared は ESTOP で遷移しないので ESTOP のまま）。
        self.drv.go_alive_and_clean()
        self._state_history.clear()
        self.drv.faults.clear()
        assert self._latest().mode == 'ESTOP', '立て直しで ESTOP を離れた'

        # 改めて重大だけ立てる（空集合への追加なので自型が出る）。
        assert drv.wait_for_fault(
            'LIMITER_DEAD', timeout=10.0,
            keep_alive=(drv.pub_scan_once, drv.pub_wf_once)), \
            f'LIMITER_DEAD が立たなかった: {drv.fault_summary()}'

        # 回復を重ねる（scan 停止。wf は維持）。代表の再送を待つ。
        mark = len(drv.faults)
        deadline = time.time() + 10.0
        new = []
        while time.time() < deadline:
            drv.pump(0.1, (drv.pub_wf_once,))
            new = [f for f in drv.faults[mark:] if f.active]
            if new:
                break
        assert new, f'scan 停止後に active なメッセージが来ない: {drv.fault_summary()}'
        assert new[-1].severity == 'CRITICAL', \
            f'重大継続中の追加フォルトで severity が落ちた: {new[-1].fault_type}/{new[-1].severity}'
        assert new[-1].fault_type == 'LIMITER_DEAD', \
            f'代表が最重（LIMITER_DEAD）ではない: {new[-1].fault_type}'

        # ESTOP → 手押しへ戻す（prev=FOLLOW は維持される）。
        # hw は押下済みでないと C-07 が出ないため立て直す。
        drv.pub_hw.publish(Bool(data=False))
        self._spin(0.2)
        drv.pub_hw.publish(Bool(data=True))
        # limiter + scan は止めたまま（wf のみ維持）。重大・回復とも残す。
        assert self._wait_mode_state_keep(
            'CARRY', 'NONE', (drv.pub_wf_once,), timeout=5.0), \
            'ESTOP から手押しに戻らなかった（C-07）'
        assert self._latest().prev_mode == 'FOLLOW', \
            'ESTOP 中の hw.estop.press で prev_* が上書きされた'
        drv.pub_hw.publish(Bool(data=False))
        drv.pump(0.5, (drv.pub_wf_once,))
        assert self._latest().mode == 'CARRY', 'HW 解除で CARRY を離れた'

        # 重大継続中は元の走行系モード（FOLLOW）へ戻れない（本試験の本質 b）。
        # 代表（重大）の再送は fault.critical として届き、C-06a で CARRY→ESTOP に
        # 移ることがある。重大フォルトは任意の状態から ESTOP なので安全側の正しい挙動。
        # したがって CARRY / ESTOP のどちらに居てもよいが、FOLLOW には戻らない。
        drv.pump(0.5, (drv.pub_wf_once,))
        res = self._trigger('ui.carry_resume')
        assert res.accepted is False, \
            '重大フォルト継続中に ui.carry_resume が通った（C-11・hw_released_and_no_critical）'
        # 「戻る」（C-09c。estop_resume_prev が重大を見る）も同様に通らない。
        res = self._trigger('ui.resume_yes')
        assert res.accepted is False, \
            '重大フォルト継続中に ui.resume_yes が通った（C-09c・estop_resume_prev）'
        drv.pump(0.5, (drv.pub_wf_once,))
        snap = self._latest()
        assert snap.mode in ('CARRY', 'ESTOP') and snap.mode != ret_mode, \
            f'重大継続中に走行系モードへ戻った: {snap.mode}/{snap.state}'

        # 対照：全解消 → 復帰できる。CARRY なら C-11（手押し復帰）で元の
        # モード/状態へ、ESTOP なら C-09c（resume_yes）で元のモードの PAUSE へ。
        assert drv.wait_for_cleared(8.0, keep_alive=drv.all_alive()), \
            '全解消の active=false が出なかった'
        drv.pump(0.5, drv.all_alive())
        if self._latest().mode == 'CARRY':
            res = self._trigger('ui.carry_resume')
            want = (ret_mode, ret_state)
        else:
            res = self._trigger('ui.resume_yes')
            want = (ret_mode, 'PAUSE')
        assert res.accepted, res.reject_reason_key
        deadline = time.time() + 5.0
        back = False
        while time.time() < deadline:
            self.drv.pump(0.1, self.drv.all_alive())
            if self._state_history:
                cur = self._state_history[-1]
                if (cur.mode, cur.state) == want:
                    back = True
                    break
        assert back, \
            f'全解消後の復帰で {want} に戻らなかった（C-11 / C-09c）'
