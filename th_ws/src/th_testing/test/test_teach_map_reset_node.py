"""
test_teach_map_reset_node.py
============================
教示の開始時に地図をまっさらに戻す配線の launch_testing（route_recorder の本番の経路）。

2026-10-09 実機: slam_toolbox が起動から約 3.7 時間ぶんの地図を作り続けたまま、その上に
教示を重ねて保存した（教示で地図の表示がリセットされない）。直す形:

  - 教示の経路選択（TEACH_* の ROUTE_SEL）に入ったら、route_recorder が
    /map_session/open に mode=reset を送る（slam_control が slam_toolbox を作り直す）
  - start_record はその完了（と map TF の復帰）を待ってから記録を始める。
    待たないと再起動中の map TF 無しで始まり、odom フレームの経路になる
    （保存で地図も残らず、再生で「使える地図がありません」になる）

観測は保存された経路 JSON の frame_id で行う（map なら地図付きで記録できた）。
slam_control は起動せず、/map_session/open と map→base_link TF は試験が代役を務める
（TF は代役のサービスが応答したあとにだけ出す＝再起動が終わるまで map TF が無い状況）。

rclpy 要＝ホスト除外（CLAUDE.md の除外一覧に載せる）。
"""
import json
import os
import tempfile
import threading
import time
import unittest

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

import tf2_ros
from geometry_msgs.msg import TransformStamped
from th_system_msgs.msg import StateEffect, SystemState
from th_system_msgs.srv import OpenMapSession

ROUTES_DIR = tempfile.mkdtemp(prefix='th_teach_map_reset_')
_RUN = [0]   # 試験ごとに別の map 座標（作り直した地図は前の地図と別の座標系）


@pytest.mark.launch_test
def generate_test_description():
    route_recorder = launch_ros.actions.Node(
        package='th_planning',
        executable='route_recorder.py',
        name='route_recorder',
        parameters=[{
            'routes_dir': ROUTES_DIR,
            'use_map_frame': True,
            'autosave_period_ms': 0,
            'map_session_id': 'sess_test',
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        route_recorder,
        launch_testing.actions.ReadyToTest(),
    ]), {}


_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


def _state(mode: str, st: str) -> SystemState:
    msg = SystemState()
    msg.mode = mode
    msg.state = st
    return msg


class TestTeachMapResetNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_teach_map_reset_client')
        _RUN[0] += 1
        # 作り直した地図は前の座標系と別物。新しい地図の TF は毎回この値から始める。
        # 記録の最初の点がこれ以上でなければ、死んだ slam の最後の TF（古い座標）を
        # 拾って始めている。
        self.tf_base = 100.0 * _RUN[0]
        self.requests = []           # (slot, mode, session_id)
        self.release = threading.Event()   # reset の応答を保留している間は未セット
        self._tf_run = threading.Event()
        self.node.create_service(
            OpenMapSession, '/map_session/open', self._on_open)
        self.tf_br = tf2_ros.TransformBroadcaster(self.node)
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', _STATE_QOS)
        self.pub_effect = self.node.create_publisher(StateEffect, '/system/effect', 10)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._tf_thread = threading.Thread(target=self._tf_loop, daemon=True)
        self._tf_thread.start()
        time.sleep(1.0)   # discovery 待ち

    def tearDown(self):
        self.release.set()
        self._tf_run.clear()
        self._executor.shutdown()
        self.node.destroy_node()

    # ── 代役 ──────────────────────────────────────────────
    def _on_open(self, request, response):
        self.requests.append((request.slot, request.mode, request.session_id))
        if request.mode == 'reset':
            # slam_toolbox の作り直しに時間がかかる状況（release されるまで返さない）。
            self.release.wait(20.0)
            self._tf_run.set()   # 応答のあとで新しい slam が map TF を出し始める
        response.success = True
        response.message = 'ok'
        return response

    def _tf_loop(self):
        x = self.tf_base
        while True:
            if self._tf_run.is_set():
                x += 0.03
                t = TransformStamped()
                t.header.stamp = self.node.get_clock().now().to_msg()
                t.header.frame_id = 'map'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = x
                t.transform.rotation.w = 1.0
                self.tf_br.sendTransform(t)
            time.sleep(0.05)

    # ── ヘルパー ──────────────────────────────────────────
    def _wait(self, cond, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if cond():
                return True
            time.sleep(0.05)
        return False

    def _effect(self, name: str, **args):
        eff = StateEffect()
        eff.name = name
        eff.dest = 'route_recorder'
        eff.args_json = json.dumps(args)
        self.pub_effect.publish(eff)

    def _saved_route(self, route_id: str):
        path = os.path.join(ROUTES_DIR, f'{route_id}.json')
        if not self._wait(lambda: os.path.exists(path), 10.0):
            return None
        with open(path, encoding='utf-8') as f:
            return json.load(f)

    # ── 本体 ──────────────────────────────────────────────
    def test_route_sel_resets_map_and_record_waits(self):
        """経路選択に入ると reset が送られ、start_record は完了を待って map 記録になる。"""
        # IDLE → TEACH_MANUAL/ROUTE_SEL（入口）
        self.pub_state.publish(_state('IDLE', 'NONE'))
        time.sleep(0.3)
        self.pub_state.publish(_state('TEACH_MANUAL', 'ROUTE_SEL'))
        assert self._wait(lambda: ('ROUTE', 'reset', 'teach') in self.requests, 5.0), \
            f'教示の経路選択に入っても mode=reset が送られない: {self.requests}'

        # 再起動が終わらないうちに経路が選ばれた（REC へ）。記録を始めてはいけない。
        self.pub_state.publish(_state('TEACH_MANUAL', 'REC'))
        self._effect('start_record', route_id='reset_wait')
        time.sleep(1.5)
        assert not os.path.exists(os.path.join(ROUTES_DIR, 'reset_wait.wip'))
        # 再起動が終わる → map TF が復帰 → 記録が始まる。
        self.release.set()
        time.sleep(3.0)
        self._effect('finalize_route', route_id='reset_wait')
        route = self._saved_route('reset_wait')
        assert route is not None, '経路が保存されていない'
        assert route.get('frame_id') == 'map', \
            f'再起動の完了を待たずに記録を始めた（frame={route.get("frame_id")!r}）。map TF 無しの odom 記録になる'
        x0 = route['points'][0][0]
        assert x0 >= self.tf_base, \
            f'記録の最初の点 x={x0} が作り直した地図の座標（{self.tf_base} 以上）でない。' \
            '死んだ slam の古い map TF を拾って始めている'

    def test_no_second_reset_while_recording_states(self):
        """REC／PAUSE への状態の再送では reset を送り直さない（記録中に地図を捨てない）。"""
        self.release.set()
        self.pub_state.publish(_state('TEACH_MANUAL', 'ROUTE_SEL'))
        assert self._wait(lambda: ('ROUTE', 'reset', 'teach') in self.requests, 5.0)
        n = sum(1 for r in self.requests if r[1] == 'reset')
        for st in ('REC', 'PAUSE', 'REC'):
            self.pub_state.publish(_state('TEACH_MANUAL', st))
            time.sleep(0.2)
        # ROUTE_SEL の再送でも二重に出さない
        self.pub_state.publish(_state('TEACH_MANUAL', 'ROUTE_SEL'))
        time.sleep(0.5)
        # 直前の REC を挟んで ROUTE_SEL に戻った = 新しい入口なので 1 回増えてよい。
        assert sum(1 for r in self.requests if r[1] == 'reset') == n + 1
        self.pub_state.publish(_state('TEACH_MANUAL', 'ROUTE_SEL'))
        time.sleep(0.5)
        assert sum(1 for r in self.requests if r[1] == 'reset') == n + 1, \
            'ROUTE_SEL の再送で reset を二重に送った'
