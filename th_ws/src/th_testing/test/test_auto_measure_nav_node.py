"""
test_auto_measure_nav_node.py
=============================
8-g P5b — 本物の `state_manager`・`measure_link`・`venue_navigator` を起動して
縛る試験。正本は docs/plan/spec/Spec-modes.md（-121〜-126・-129〜-133）と
docs/plan/spec/Spec-measure.md（§4.2・§4.4）、詳細は DetailedDesign-measure.md §6.2。

起動は「生成 yaml を先に渡す」本番の経路（P5a の test_auto_measure_node.py と同じ）。
Nav2 の縁（ComputePathToPose／FollowPath）は既存の代役で与え、`/measure/corner_status`
と `/map_session/status`・`/map`・TF・障害物マップは試験側が出す。

  a. meas.goal（今の instance_id）→ NAV → 代役の Nav2 が着く → ALIGN → WAIT、
     /measure/result に ARRIVED がちょうど 1 回
  b. map_instance_id 違い → map_mismatch、corner が clear=false → corner_obstacle。
     どちらも WAIT のまま・結果は出ない
  c. NAV で meas.stop → PAUSE、FollowPath が取り消される。meas.start で NAV へ戻る。
     目的地の結果はまだ出ない
  d. NAV で非常停止 → 解除 → ui.resume_yes で PAUSE に戻り、走り出さない。
     meas.start で NAV に戻る
  e. NAV で生存確認を止める → PAUSE（pause_reason=measure_link）。戻っても PAUSE のまま
  f. meas.abort → WAIT、目的地に CANCELLED（aborted）が 1 回
  g. 手前 1.5 m の /map が不明 → BLOCKED。meas.reroute は DONE
  h. NAV の間 meas.goal は state_not_accepting、meas.set_limits はどの状態でも
     state_not_accepting
  i. 到着のあと次の meas.goal は新しい経路を計画する（古い経路の再送が無い）
  j. abort のあと非常停止でも捨てた目的地で走らない
  k. VENUE/UNLOADED でも meas.goal で NAV に入り ComputePath・FollowPath が呼ばれる
     （試験場内の SM-3.1.2-119 の門が自律測定に効かない）

注意（launch_testing の挙動）: ノードはファイル単位で共有されるため、
メソッド同士の状態に頼らない（各メソッドが _reset_to_idle で自分の前提を作る）。
"""
import json
import os
import sys
import tempfile
import threading
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import (MapSessionStatus, MeasureHeartbeat,
                                MeasureResult, MeasureStatus, StateEvent,
                                SystemState)
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
        _GEN_DIR = tempfile.mkdtemp(prefix='auto_measure_nav_test_gen_')
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
    venue_navigator = launch_ros.actions.Node(
        package='th_onsite',
        executable='venue_navigator.py',
        name='venue_navigator',
        parameters=[{'blocked_recheck_period_s': 0.5}],
        output='screen',
    )
    return launch.LaunchDescription([
        state_manager,
        measure_link,
        venue_navigator,
        launch_testing.actions.ReadyToTest(),
    ]), {}


_LATCHED_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


def _clear_grid_frame(frame_id, unknown_band=None):
    m = OccupancyGrid()
    m.header.frame_id = frame_id
    m.info.resolution = 0.05
    m.info.width = 200
    m.info.height = 200
    m.info.origin.position.x = -5.0
    m.info.origin.position.y = -5.0
    m.info.origin.orientation.w = 1.0
    data = [0] * (200 * 200)
    if unknown_band is not None:
        x0, x1 = unknown_band
        for iy in range(200):
            y = -5.0 + (iy + 0.5) * 0.05
            if not (-0.5 <= y <= 0.5):
                continue
            for ix in range(200):
                x = -5.0 + (ix + 0.5) * 0.05
                if x0 <= x < x1:
                    data[iy * 200 + ix] = -1
    m.data = data
    return m


class TestAutoMeasureNavNode(unittest.TestCase):

    _cid = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_auto_measure_nav_node')
        self._lock = threading.Lock()
        self._statuses = []
        self.node.create_subscription(
            MeasureStatus, '/measure/status', self._statuses.append, _LATCHED_QOS)
        self._states = []
        self.node.create_subscription(
            SystemState, '/system/state', self._states.append, _LATCHED_QOS)
        self._results = []
        self.node.create_subscription(
            MeasureResult, '/measure/result', self._results.append, 20)

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

        # 代役の出方（試験ごとに触る）。
        self.robot_x = 0.0
        self.corner_clear = True
        self.corner_live = True
        # 本番の自律測定では start_measure_map のあと venue_session_mode は
        # MAPPING（地図作成）。VENUE/LOCALIZING は出ない値を既定にしない。
        self.session_mode = 'MAPPING'
        self.map_unknown_band = None
        self.follow_succeed = False
        self.follow_goals = 0
        self.follow_cancel_count = 0
        self.compute_goals = []  # ComputePathToPose の goal の x（順）

        self.pub_corner = self.node.create_publisher(
            String, '/measure/corner_status', 10)
        self._corner_timer = self.node.create_timer(0.2, self._publish_corner)
        self.pub_session = self.node.create_publisher(
            MapSessionStatus, '/map_session/status', _LATCHED_QOS)
        self._session_timer = self.node.create_timer(1.0, self._publish_session)
        self.pub_tf = self.node.create_publisher(TFMessage, '/tf_static', _LATCHED_QOS)
        self._tf_timer = self.node.create_timer(0.2, self._publish_tf)
        self._publish_tf()
        self.pub_costmap = self.node.create_publisher(
            OccupancyGrid, '/global_costmap/costmap', _LATCHED_QOS)
        self._costmap_timer = self.node.create_timer(1.0, self._publish_costmap)
        self.pub_map = self.node.create_publisher(OccupancyGrid, '/map', _LATCHED_QOS)
        self._map_timer = self.node.create_timer(1.0, self._publish_map)
        self._publish_costmap()
        self._publish_map()
        self.pub_estop_ui = self.node.create_publisher(Bool, '/safety/estop_ui', 10)

        cbg = ReentrantCallbackGroup()
        self._compute_server = ActionServer(
            self.node, ComputePathToPose, 'compute_path_to_pose',
            execute_callback=self._exec_compute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _req: CancelResponse.ACCEPT,
            callback_group=cbg)
        self._follow_server = ActionServer(
            self.node, FollowPath, 'follow_path',
            execute_callback=self._exec_follow,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=self._on_follow_cancel,
            callback_group=cbg)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        time.sleep(2.0)
        self._reset_to_idle()

    def tearDown(self):
        self._hb_version = None
        self.corner_live = True
        self.corner_clear = True
        self.session_mode = 'MAPPING'
        self.map_unknown_band = None
        try:
            self._executor.shutdown()
        except Exception:
            pass
        self._spin_thread.join(timeout=5.0)
        for t in (getattr(self, '_hb_timer', None), getattr(self, '_corner_timer', None),
                  getattr(self, '_session_timer', None), getattr(self, '_tf_timer', None),
                  getattr(self, '_costmap_timer', None), getattr(self, '_map_timer', None)):
            if t is not None:
                try:
                    self.node.destroy_timer(t)
                except Exception:
                    pass
        self.node.destroy_node()

    # ── 代役出版（このノードの executor が裏で回している） ──────────
    def _now_msg(self):
        return self.node.get_clock().now().to_msg()

    def _publish_hb(self):
        if self._hb_version is None:
            return
        self._hb_seq += 1
        msg = MeasureHeartbeat()
        msg.header.stamp = self._now_msg()
        msg.interface_version = self._hb_version
        msg.sender = 'fk-system'
        msg.seq = self._hb_seq
        self.pub_hb.publish(msg)

    def _publish_corner(self):
        if not self.corner_live:
            return
        msg = String()
        msg.data = json.dumps({"clear": bool(self.corner_clear),
                               "reason": "clear" if self.corner_clear else "test"})
        self.pub_corner.publish(msg)

    def _publish_session(self):
        msg = MapSessionStatus()
        msg.header.stamp = self._now_msg()
        msg.slot = 'VENUE'
        msg.mode = self.session_mode
        msg.instance_id = 'm1'
        msg.dirty = False
        self.pub_session.publish(msg)

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self._now_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.robot_x)
        t.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[t]))

    def _publish_costmap(self):
        self.pub_costmap.publish(_clear_grid_frame('map'))

    def _publish_map(self):
        self.pub_map.publish(_clear_grid_frame('map', self.map_unknown_band))

    def _exec_compute(self, goal_handle):
        req = goal_handle.request
        gx = float(req.goal.pose.position.x)
        gy = float(req.goal.pose.position.y)
        with self._lock:
            self.compute_goals.append(gx)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self._now_msg()
        for i in range(5):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = gx * i / 4.0
            p.pose.position.y = gy * i / 4.0
            path.poses.append(p)
        goal_handle.succeed()
        return ComputePathToPose.Result(path=path)

    def _on_follow_cancel(self, _req):
        with self._lock:
            self.follow_cancel_count += 1
        return CancelResponse.ACCEPT

    def _exec_follow(self, goal_handle):
        with self._lock:
            self.follow_goals += 1
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowPath.Result()
            if self.follow_succeed:
                goal_handle.succeed()
                return FollowPath.Result()
            time.sleep(0.05)
        goal_handle.succeed()
        return FollowPath.Result()

    # ── 補助 ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        """ノードは executor の裏スレッドが回しているので、ここは待つだけ。
        （executor と spin_once の二重に回すと rclpy が競合する。）"""
        time.sleep(duration)

    def _next_cid(self):
        type(self)._cid += 1
        return f"nav-{type(self)._cid}"

    def _call_measure(self, command, command_id=None, map_instance_id='', x=0.0, y=0.0,
                      yaw=0.0, v_max=0.0):
        req = MeasureCommand.Request()
        req.command_id = self._next_cid() if command_id is None else command_id
        req.command = command
        req.map_instance_id = map_instance_id
        req.x = float(x)
        req.y = float(y)
        req.yaw = float(yaw)
        req.v_max_mps = float(v_max)
        future = self.cli_measure.call_async(req)
        deadline = time.time() + 8.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        assert future.done(), f'/measure/command が返らない: {command}'
        return future.result()

    def _call_trigger(self, trigger, arg_json='{}'):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = arg_json
        req.requester = 'test'
        future = self.cli_trigger.call_async(req)
        deadline = time.time() + 8.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        assert future.done(), f'/system/trigger が返らない: {trigger}'
        return future.result()

    def _latest_state(self):
        with self._lock:
            return self._states[-1] if self._states else None

    def _mode_state(self):
        with self._lock:
            return (self._states[-1].mode, self._states[-1].state) if self._states else None

    def _wait_mode_state(self, mode, state, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._mode_state() == (mode, state):
                return True
            time.sleep(0.05)
        return self._mode_state() == (mode, state)

    def _wait_status_link_ok(self, want, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._spin(0.1)
            if self._statuses and self._statuses[-1].link_ok == want:
                return True
        return False

    def _wait_result(self, cid, result=None, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                found = [r for r in self._results
                         if r.command_id == cid and (result is None or r.result == result)]
            if found:
                return found
            self._spin(0.1)
        with self._lock:
            return [r for r in self._results if r.command_id == cid]

    def _results_for(self, cid):
        with self._lock:
            return [r for r in self._results if r.command_id == cid]

    def _wait_follow_goals(self, n, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self.follow_goals >= n:
                    return True
            time.sleep(0.05)
        return False

    def _drive_to_idle(self):
        self._spin(0.3)
        if not self._states or self._states[-1].mode != 'INIT':
            if self._mode_state() == ('IDLE', 'NONE'):
                return
        deadline = time.time() + 10.0
        while time.time() < deadline:
            msg = StateEvent()
            msg.header.stamp = self._now_msg()
            msg.event = 'evt.link_ok'
            msg.source_node = 'connectivity_checker'
            msg.arg_json = '{}'
            self.pub_event.publish(msg)
            self._spin(0.3)
            if self._mode_state() == ('IDLE', 'NONE'):
                return
        # 既に IDLE なら通す（前メソッドの到達で spin が間に合う）。
        assert self._mode_state() == ('IDLE', 'NONE'), self._mode_state()

    def _reset_to_idle(self):
        self.robot_x = 0.0
        self.follow_succeed = False
        self.corner_live = True
        self.corner_clear = True
        self.map_unknown_band = None
        ms = self._mode_state()
        if ms is None or ms[0] == 'INIT':
            self._drive_to_idle()
            ms = self._mode_state()
        if ms and ms[0] == 'AUTO_MEASURE':
            res = self._call_trigger('ui.finish')
            assert res.accepted is True, res.reject_reason_key
            assert self._wait_mode_state('IDLE', 'NONE', timeout=8.0) is True
        elif ms and ms[0] == 'ESTOP':
            self.pub_estop_ui.publish(Bool(data=False))
            self._spin(0.5)
            res = self._call_trigger('ui.resume_no')
            assert res.accepted is True, res.reject_reason_key
            assert self._wait_mode_state('IDLE', 'NONE', timeout=8.0) is True
        else:
            assert ms == ('IDLE', 'NONE'), ms

    def _ensure_auto_measure_wait(self):
        self._reset_to_idle()
        self._hb_version = '1'
        assert self._wait_status_link_ok(True) is True, 'link_ok にならない'
        assert self._wait_mode_state('IDLE', 'NONE', timeout=5.0) is True
        res = self._call_trigger('ui.goto', json.dumps({"kind": "AUTO_MEASURE"}))
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT', timeout=8.0) is True
        # corner・instance_id が state_manager に届いたことを確かめる
        # （goal の拒否判定がこれで決まる）。
        time.sleep(1.5)
        ms = self._mode_state()
        assert ms == ('AUTO_MEASURE', 'WAIT'), ms

    # ════════════════════════════════════════════════════════
    def test_a_goal_arrives_once(self):
        """a. meas.goal → NAV → Nav2 が着く → ALIGN → WAIT、ARRIVED がちょうど 1 回。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(1, timeout=10.0) is True, \
            'FollowPath が送られていない（NAV での走行）'
        # 到着位置へ移して代役を成功させる。
        self.robot_x = 3.0
        self._publish_tf()
        time.sleep(0.5)
        self.follow_succeed = True
        # ALIGN は向きが合っているとごく短い（観測のポーリングで取りこぼす）。
        # 最新値ではなく履歴で「ALIGN を経由した」ことを確かめる。
        deadline = time.monotonic() + 10.0
        seen_align = False
        while time.monotonic() < deadline:
            with self._lock:
                seen_align = any((s.mode, s.state) == ('AUTO_MEASURE', 'ALIGN')
                                 for s in self._states)
            if seen_align:
                break
            time.sleep(0.05)
        assert seen_align, '到着で ALIGN を経由しない'
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT', timeout=10.0) is True, \
            '向き合わせで WAIT に戻らない'
        found = self._wait_result(cid, result='ARRIVED', timeout=8.0)
        assert len(found) == 1, found
        self._spin(1.0)
        assert len(self._results_for(cid)) == 1, self._results_for(cid)

    def test_b_map_mismatch_and_corner_obstacle(self):
        """b. map_instance_id 違い → map_mismatch。corner=false → corner_obstacle。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid, map_instance_id='old',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is False
        assert res.reject_reason == 'map_mismatch', res.reject_reason
        self._spin(0.5)
        assert not self._results_for(cid)
        assert self._mode_state() == ('AUTO_MEASURE', 'WAIT')

        self.corner_clear = False
        time.sleep(0.6)  # 5 Hz の corner_status が state_manager に届く
        cid2 = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid2, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is False
        assert res.reject_reason == 'corner_obstacle', res.reject_reason
        self._spin(0.5)
        assert not self._results_for(cid2)
        assert self._mode_state() == ('AUTO_MEASURE', 'WAIT')

        # corner_status が止まると 1 秒で clear=false に倒れる。
        # 最後に届いた値を true にしてから止める（鮮度判定を見ないと受理される）。
        self.corner_clear = True
        self.corner_live = True
        time.sleep(0.6)  # clear=true が state_manager に届く
        # いま受理されることを確認してから、abort で WAIT に戻る。
        cid0 = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid0, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        res = self._call_measure('meas.abort')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT', timeout=8.0) is True
        self.corner_live = False
        time.sleep(1.5)
        cid3 = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid3, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is False
        assert res.reject_reason == 'corner_obstacle', res.reject_reason
        self._spin(0.5)
        assert not self._results_for(cid3)
        assert self._mode_state() == ('AUTO_MEASURE', 'WAIT')

    def test_c_stop_to_pause_and_start_back_to_nav(self):
        """c. NAV で meas.stop → PAUSE（FollowPath 取り消し）。meas.start で NAV へ。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(1, timeout=10.0) is True
        with self._lock:
            c0 = self.follow_cancel_count
        res = self._call_measure('meas.stop')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'PAUSE', timeout=8.0) is True
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self._lock:
                if self.follow_cancel_count > c0:
                    break
            time.sleep(0.05)
        with self._lock:
            assert self.follow_cancel_count > c0, \
                'PAUSE で FollowPath が取り消されていない'
        assert not self._results_for(cid), '目的地の結果はまだ出ない'
        res = self._call_measure('meas.start')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(2, timeout=10.0) is True, \
            'meas.start で NAV に戻ったのに残りの経路が再送されていない'
        assert not self._results_for(cid), '目的地の結果はまだ出ない'

    def test_d_estop_resume_lands_in_pause_not_nav(self):
        """d. NAV で非常停止 → 解除 → ui.resume_yes は PAUSE で走り出さない。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(1, timeout=10.0) is True
        self.pub_estop_ui.publish(Bool(data=True))
        assert self._wait_mode_state('ESTOP', 'NONE', timeout=8.0) is True
        self.pub_estop_ui.publish(Bool(data=False))
        self._spin(0.5)
        res = self._call_trigger('ui.resume_yes')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AUTO_MEASURE', 'PAUSE', timeout=8.0) is True
        # 走り出さない: 新しい FollowPath が送られない。
        time.sleep(2.0)
        with self._lock:
            goals = self.follow_goals
        assert goals == 1, f'PAUSE へ戻ったのに FollowPath が送られた（{goals}）'
        assert not self._results_for(cid), '目的地の結果はまだ出ない'
        res = self._call_measure('meas.start')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(2, timeout=10.0) is True

    def test_e_link_loss_pauses_with_reason_and_stays(self):
        """e. NAV で生存確認を止める → PAUSE（pause_reason=measure_link）。
        戻っても PAUSE のまま（発進は meas.start だけ）。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        self._hb_version = None
        assert self._wait_status_link_ok(False) is True
        assert self._wait_mode_state('AUTO_MEASURE', 'PAUSE', timeout=10.0) is True
        time.sleep(0.5)
        st = self._latest_state()
        assert st.pause_reason == 'measure_link', st.pause_reason
        # 生存確認が戻っても PAUSE のまま（走らない）。
        self._hb_version = '1'
        assert self._wait_status_link_ok(True) is True
        time.sleep(2.0)
        assert self._mode_state() == ('AUTO_MEASURE', 'PAUSE'), self._mode_state()
        assert not self._results_for(cid), '目的地の結果はまだ出ない'

    def test_f_abort_cancels_destination_once(self):
        """f. meas.abort → WAIT、目的地に CANCELLED（aborted）が 1 回。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(1, timeout=10.0) is True
        res = self._call_measure('meas.abort')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT', timeout=8.0) is True
        found = self._wait_result(cid, timeout=8.0)
        assert len(found) == 1, found
        assert found[0].result == 'CANCELLED' and found[0].reason == 'aborted', found
        self._spin(1.0)
        assert len(self._results_for(cid)) == 1, self._results_for(cid)

    def test_g_unknown_map_ahead_blocks(self):
        """g. 手前 1.5 m の /map が不明 → BLOCKED。meas.reroute は DONE。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(1, timeout=10.0) is True
        # 手前 1.5 m の /map を不明にする。
        self.map_unknown_band = (0.5, 1.0)
        self._publish_map()
        assert self._wait_mode_state('AUTO_MEASURE', 'BLOCKED', timeout=12.0) is True, \
            '/map が不明なのに BLOCKED に行かない'
        rcid = self._next_cid()
        res = self._call_measure('meas.reroute', command_id=rcid)
        assert res.accepted is True, res.reject_reason
        found = self._wait_result(rcid, result='DONE', timeout=8.0)
        assert len(found) == 1, found
        # 目的地はまだ続いている（結果が出ない）。
        assert not self._results_for(cid)

    def test_h_goal_and_set_limits_rejected_in_nav(self):
        """h. NAV の間 meas.goal は state_not_accepting。meas.set_limits は
        P5c で受け、状態は変えない（NAV のまま）。"""
        self._ensure_auto_measure_wait()
        cid = self._next_cid()
        res = self._call_measure('meas.goal', command_id=cid, map_instance_id='m1',
                                 x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        res = self._call_measure('meas.goal', map_instance_id='m1', x=1.0)
        assert res.accepted is False
        assert res.reject_reason == 'state_not_accepting', res.reject_reason
        res = self._call_measure('meas.set_limits', v_max=0.1)
        assert res.accepted is True, res.reject_reason
        assert self._mode_state() == ('AUTO_MEASURE', 'NAV')
        # set_limits は WAIT でも受ける（状態は変えない）。
        res = self._call_trigger('ui.finish')
        assert res.accepted is True
        assert self._wait_mode_state('IDLE', 'NONE', timeout=8.0) is True
        self._hb_version = '1'
        res = self._call_trigger('ui.goto', json.dumps({"kind": "AUTO_MEASURE"}))
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT', timeout=8.0) is True
        time.sleep(1.0)
        res = self._call_measure('meas.set_limits', v_max=0.1)
        assert res.accepted is True, res.reject_reason
        assert self._mode_state() == ('AUTO_MEASURE', 'WAIT')

    def test_i_new_goal_plans_fresh_path(self):
        """i. 目的地 A へ到着したあとの meas.goal（目的地 B）は新しい経路を計画する。
        古い経路の再送が無いこと。"""
        self._ensure_auto_measure_wait()
        res = self._call_measure('meas.goal', command_id=self._next_cid(),
                                 map_instance_id='m1', x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(1, timeout=10.0) is True
        with self._lock:
            assert self.compute_goals == [3.0], self.compute_goals
            follows_before = self.follow_goals
        self.robot_x = 3.0
        self._publish_tf()
        time.sleep(0.5)
        self.follow_succeed = True
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT', timeout=10.0) is True
        # 目的地 B：新しい経路を計画し、その経路で走る。
        self.robot_x = 0.0
        self._publish_tf()
        self.follow_succeed = False
        res = self._call_measure('meas.goal', command_id=self._next_cid(),
                                 map_instance_id='m1', x=-2.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(follows_before + 1, timeout=10.0) is True
        with self._lock:
            assert self.compute_goals[-1] == -2.0, self.compute_goals
            assert len(self.compute_goals) == 2, self.compute_goals

    def test_j_abort_then_estop_start_goes_to_wait(self):
        """j. NAV→stop→PAUSE→abort→WAIT のあと非常停止を挟んでも、捨てた目的地で
        走り出さない（戻り先は WAIT で、meas.start は受け付けられない）。"""
        self._ensure_auto_measure_wait()
        res = self._call_measure('meas.goal', command_id=self._next_cid(),
                                 map_instance_id='m1', x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(1, timeout=10.0) is True
        res = self._call_measure('meas.stop')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'PAUSE', timeout=8.0) is True
        res = self._call_measure('meas.abort')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT', timeout=8.0) is True
        with self._lock:
            follows_before = self.follow_goals
        self.pub_estop_ui.publish(Bool(data=True))
        assert self._wait_mode_state('ESTOP', 'NONE', timeout=8.0) is True
        self.pub_estop_ui.publish(Bool(data=False))
        self._spin(0.5)
        res = self._call_trigger('ui.resume_yes')
        assert res.accepted is True, res.reject_reason_key
        # WAIT から非常停止に入ったので、戻り先は WAIT（PAUSE ではない）。
        # 古い prev（NAV）が残っていれば、ここで誤って NAV へ走り出す。
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT', timeout=10.0) is True
        res = self._call_measure('meas.start')
        assert res.accepted is False
        assert res.reject_reason == 'state_not_accepting', res.reject_reason
        time.sleep(1.5)
        assert self._mode_state() == ('AUTO_MEASURE', 'WAIT'), self._mode_state()
        with self._lock:
            assert self.follow_goals == follows_before, \
                '捨てた目的地で走り出している'

    def test_k_venue_unloaded_still_drives(self):
        """k. VENUE/UNLOADED でも meas.goal で NAV に入り、ComputePath・FollowPath が
        呼ばれる（試験場内の SM-3.1.2-119 の門が自律測定に効かない）。"""
        self._ensure_auto_measure_wait()
        self.session_mode = 'UNLOADED'
        self._publish_session()
        time.sleep(1.0)  # venue へ UNLOADED が届く
        with self._lock:
            computes_before = len(self.compute_goals)
            follows_before = self.follow_goals
        res = self._call_measure('meas.goal', command_id=self._next_cid(),
                                 map_instance_id='m1', x=3.0, y=0.0, yaw=0.0)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'NAV', timeout=10.0) is True
        assert self._wait_follow_goals(follows_before + 1, timeout=10.0) is True
        with self._lock:
            assert len(self.compute_goals) > computes_before, \
                'ComputePathToPose が呼ばれていない'
