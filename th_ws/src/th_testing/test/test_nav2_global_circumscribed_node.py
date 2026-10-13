"""
test_nav2_global_circumscribed_node.py — 経路計画の地図は外接円で膨らませる
launch_testing 試験（2026-10-13 実機。ユーザー決定 B1。Spec-onsite §6.1.1 追補）。

実機で、計画（planner_server の NavFn。点として経路を引く）が、内接円 0.275 m は通るが
四角い機体（0.55 m 角。外接 0.389 m）の角は当たる隙間に経路を引き、追従側が
「前方に衝突」で打ち切る／側面が壁に接触して停止した。global_costmap を外接円
(robot_radius 0.389) で膨らませると、そうした隙間には経路を引かない。

本番ノード（nav2_planner の planner_server と、その中の global_costmap、nav2_map_server）を、
実機が読むのと同じ `config/nav2_params.yaml` で単体起動し、試験用に作った小さな占有格子地図
で `/compute_path_to_pose` を呼んで次を縛る。
  (a) 幅 1.0 m の通路は計画できる（実機で通った幅。リグレッション防止）
  (b) 幅 0.6 m の通路（四角い機体は物理的に入るが外接円の直径 0.778 m は入らない）は計画しない
  (c) 幅 0.7 m（機体幅 0.55 m 超・外接円の直径 0.778 m 未満）の隙間しか無ければ計画しない／幅 1.2 m の迂回路があればそちらを通る
  (d) 始点が壁際（機体の中心が壁から 0.3 m）でも計画が失敗しすぎない
  (e) global_costmap は円（robot_radius）、local_costmap は実寸の正方形のまま（RPP 側は変えない）
ROS（rclpy・nav2）が要るのでホストでは走らない。
"""
from __future__ import annotations

import math
import os
import tempfile
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from tf2_msgs.msg import TFMessage

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
_NAV2_STATIC = os.path.join(_WS_ROOT, 'src', 'th_bringup', 'config', 'nav2_params.yaml')

with open(_NAV2_STATIC, encoding='utf-8') as _f:
    _CFG = yaml.safe_load(_f)
_GCM = _CFG['global_costmap']['global_costmap']['ros__parameters']
_LCM = _CFG['local_costmap']['local_costmap']['ros__parameters']

RES = 0.05
# 地図は 4 つの部屋（各 6 m x 6 m）を横に並べる。部屋ごとに外周は壁で閉じる。
ROOM = 6.0
N_ROOMS = 5
W_M, H_M = ROOM * N_ROOMS, ROOM
W, H = int(W_M / RES), int(H_M / RES)
WALL_T = 0.15


def _build_map():
    g = [[254] * W for _ in range(H)]          # 254 = 自由（y は上が大きい向きで後で反転）

    def block(x0, x1, y0, y1):
        for iy in range(max(0, int(round(y0 / RES))), min(H, int(round(y1 / RES)))):
            for ix in range(max(0, int(round(x0 / RES))), min(W, int(round(x1 / RES)))):
                g[iy][ix] = 0

    def room(i):
        ox = i * ROOM
        block(ox, ox + WALL_T, 0, ROOM)
        block(ox + ROOM - WALL_T, ox + ROOM, 0, ROOM)
        block(ox, ox + ROOM, 0, WALL_T)
        block(ox, ox + ROOM, ROOM - WALL_T, ROOM)
        return ox

    def divider(ox, thick, gaps):
        """部屋の中央（x 方向の厚み thick）に y 方向の壁を立て、gaps=[(y0,y1)...] を開ける。"""
        x0 = ox + ROOM / 2 - thick / 2
        x1 = x0 + thick
        ys = [0.0] + [v for gap in sorted(gaps) for v in gap] + [ROOM]
        for k in range(0, len(ys), 2):
            block(x0, x1, ys[k], ys[k + 1])

    # 部屋 0 (a): 幅 1.0 m、長さ 1.5 m の通路
    ox = room(0)
    divider(ox, 1.5, [(2.5, 3.5)])
    # 部屋 1 (b): 幅 0.6 m の通路
    ox = room(1)
    divider(ox, 1.5, [(2.7, 3.3)])
    # 部屋 2 (c1): 障害物（薄い壁）の脇に幅 0.7 m の隙間だけ（内接円 0.55 は入るが外接円 0.778 は入らない）
    ox = room(2)
    divider(ox, 0.3, [(2.65, 3.35)])
    # 部屋 3 (c2): 幅 0.7 m の隙間（始点・終点を結ぶ直線上）と、遠い所に幅 1.2 m の迂回路
    ox = room(3)
    divider(ox, 0.3, [(2.65, 3.35), (4.2, 5.4)])
    # 部屋 4 (d): 開けた部屋。南壁の内側から 0.3 m（機体の中心）が始点
    room(4)
    return g


def _write_map(dir_):
    g = _build_map()
    pgm = os.path.join(dir_, 'm.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode())
        for iy in range(H - 1, -1, -1):        # pgm は上の行が先頭（y 最大）
            f.write(bytes(g[iy]))
    ymlp = os.path.join(dir_, 'm.yaml')
    with open(ymlp, 'w') as f:
        yaml.safe_dump({'image': pgm, 'resolution': RES, 'origin': [0.0, 0.0, 0.0],
                        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25,
                        'mode': 'trinary'}, f)
    return ymlp


_TMP = tempfile.mkdtemp(prefix='nav2_global_circ_')
_MAP_YAML = _write_map(_TMP)


@pytest.mark.launch_test
def generate_test_description():
    ms = launch_ros.actions.Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{'yaml_filename': _MAP_YAML, 'use_sim_time': False}])
    ps = launch_ros.actions.Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[_NAV2_STATIC])
    return launch.LaunchDescription([ms, ps, launch_testing.actions.ReadyToTest()]), {}


class TestGlobalCircumscribed(unittest.TestCase):
    _active = False
    base = (1.0, 1.0)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('global_circ_test')
        cls.exec = MultiThreadedExecutor(num_threads=4)
        cls.exec.add_node(cls.node)
        cls.thr = threading.Thread(target=cls.exec.spin, daemon=True)
        cls.thr.start()
        cls.tf_pub = cls.node.create_publisher(TFMessage, '/tf', 10)
        cls.node.create_timer(0.05, cls._tick)
        cls.ac = ActionClient(cls.node, ComputePathToPose, '/compute_path_to_pose')

    @classmethod
    def tearDownClass(cls):
        cls.exec.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _tick(cls):
        t = TransformStamped()
        t.header.stamp = cls.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = cls.base
        t.transform.rotation.w = 1.0
        cls.tf_pub.publish(TFMessage(transforms=[t]))

    def _call(self, srv, req_type, node_name, tid=None):
        cli = self.node.create_client(srv, f'/{node_name}/' + ('change_state' if tid is not None else 'get_state'))
        self.assertTrue(cli.wait_for_service(timeout_sec=30.0), f'{node_name} が見えない')
        req = req_type()
        if tid is not None:
            req.transition.id = tid
        fut = cli.call_async(req)
        end = time.monotonic() + 60.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        self.assertTrue(fut.done(), f'{node_name} の lifecycle 呼び出しが返らない')
        return fut.result()

    def _ensure_active(self):
        cls = type(self)
        if cls._active:
            return
        for name in ('map_server', 'planner_server'):
            for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                r = self._call(ChangeState, ChangeState.Request, name, tid)
                self.assertTrue(r.success, f'{name} の遷移 {tid} が失敗')
        time.sleep(3.0)     # global_costmap が地図を受け取り膨張させるのを待つ
        cls._active = True

    def _plan(self, start, goal):
        """(成功したか, 経路の (x, y) 列)。"""
        self._ensure_active()
        type(self).base = start
        self.assertTrue(self.ac.wait_for_server(timeout_sec=30.0))

        def ps(p):
            m = PoseStamped()
            m.header.frame_id = 'map'
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y = p
            m.pose.orientation.w = 1.0
            return m
        g = ComputePathToPose.Goal()
        g.start = ps(start)
        g.goal = ps(goal)
        g.use_start = True
        g.planner_id = 'GridBased'
        gf = self.ac.send_goal_async(g)
        end = time.monotonic() + 15.0
        while not gf.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(gf.done() and gf.result().accepted, 'ComputePathToPose が受理されない')
        rf = gf.result().get_result_async()
        end = time.monotonic() + 30.0
        while not rf.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(rf.done(), '計画が返らない')
        res = rf.result()
        pts = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
        return (res.status == 4 and len(pts) > 1), pts   # 4 = SUCCEEDED

    # ---- 設定の到達 ---------------------------------------------------------
    def test_00_costmap_models(self):
        # global は外接円（0.389 を下回らない）で footprint は持たない。local は実寸正方形のまま
        self.assertIn('robot_radius', _GCM)
        self.assertNotIn('footprint', _GCM)
        self.assertGreaterEqual(float(_GCM['robot_radius']), math.hypot(0.275, 0.275) - 1e-3)
        self.assertLessEqual(float(_GCM['robot_radius']), 0.40)
        self.assertEqual(yaml.safe_load(_LCM['footprint']),
                         [[0.275, 0.275], [-0.275, 0.275], [-0.275, -0.275], [0.275, -0.275]])
        self.assertNotIn('robot_radius', _LCM)

    # ---- 計画 ---------------------------------------------------------------
    def test_10_a_corridor_1_0m_plans(self):
        ok, pts = self._plan((1.0, 3.0), (5.0, 3.0))
        self.assertTrue(ok, '幅 1.0 m の通路が計画できない（実機で通れた幅）')
        self.assertGreater(len(pts), 20)

    def test_11_b_corridor_0_6m_does_not_plan(self):
        ok, pts = self._plan((ROOM + 1.0, 3.0), (ROOM + 5.0, 3.0))
        self.assertFalse(ok, '幅 0.6 m（外接円が入らない）の通路に経路を引いた'
                             f'（点数 {len(pts)}）。四角い機体の角が当たる')

    def test_12_c1_only_0_7m_gap_does_not_plan(self):
        ok, pts = self._plan((2 * ROOM + 1.0, 3.0), (2 * ROOM + 5.0, 3.0))
        self.assertFalse(ok, f'幅 0.7 m の隙間に経路を引いた（点数 {len(pts)}）')

    def test_13_c2_detours_via_1_2m_gap(self):
        ox = 3 * ROOM
        ok, pts = self._plan((ox + 1.0, 3.0), (ox + 5.0, 3.0))
        self.assertTrue(ok, '幅 1.2 m の迂回路があるのに計画できない')
        # 壁の x 位置（部屋の中央 ±0.15）を通る点の y は 1.2 m の隙間 (4.2〜5.4) の中
        cross = [y for x, y in pts if abs(x - (ox + ROOM / 2)) < 0.1]
        self.assertTrue(cross, '壁の位置を横切る点が無い')
        for y in cross:
            self.assertTrue(4.2 < y < 5.4, f'1.2 m の隙間でなく y={y:.2f} を通った')

    def test_14_d_start_0_3m_from_wall(self):
        ox = 4 * ROOM
        # 南壁の内側面は y=0.15。機体の中心を壁から 0.3 m（y=0.45）に置く
        ok, pts = self._plan((ox + 1.0, WALL_T + 0.30), (ox + 4.0, 3.0))
        print(f'[d] 壁際 0.3 m 始点の計画: ok={ok} 点数={len(pts)}')
        self.assertTrue(ok, '壁から 0.3 m の始点から計画できない（失敗しすぎ）')
