"""
test_jog_gate_node.py
======================
DetailedDesign-wp2.md WP-SAFE-04 §7 のノード統合テスト。

3 本のテスト:

`test_silent_when_state_stale`（**J-1 の直接検証**）:
  /system/state が古いあいだ /cmd_vel_manual_raw を送り続けても、
  /cmd_vel_manual に **1 通も出ない**こと（§7 の「publish 回数が 0」そのもの）。

`test_mux_can_select_priority_20`（J-1 の帰結）:
  jog_gate が黙っている（IDLE＝attributes.yaml で jog: denied）あいだ、
  /cmd_vel_manual_raw と /cmd_vel_behavior を**同時に**流し続けても
  /cmd_vel_behavior（priority 20）が /cmd_vel_muxed に出ること。

  **同時に流すことが要点**である。初版は raw を送り終えてから behavior を
  送っており、jog_gate が「塞ぐときにゼロを撃つ」ように壊しても
  （priority 30 の manual が 0.5 s でタイムアウトしてしまうため）
  **テストが緑のまま通った**（2026-09-01 に変異テストで実測）。

`test_forwards_unchanged_when_allowed`（J-3）:
  通す場面（MANUAL = is_drive）で、速度の値を変えずそのまま
  /cmd_vel_manual へ転送すること。初版はこれを検証しておらず、
  転送値をゼロに潰す変異が**素通りした**。

J-1 の意味するところ:
  /cmd_vel_manual は twist_mux の priority 30（最高）。もし jog_gate が IDLE で
  ゼロを撃ち続けると manual 入力が常に非タイムアウトになり、twist_mux は
  priority 20 / 10 を永久に選ばない。本テストは「jog_gate が黙っている ⇒
  behavior（priority 20）が選ばれる」ことを直接検証し、J-1 が満たされている
  ことを実機に頼らず示す。

起動するノード（launch_testing・unittest.TestCase。conftest のフィクスチャは
受け取れない）:
  - twist_mux（th_safety/config の static でなく、テストで明示的に topics を
    与える。behavior=20 / nav=10 / manual=30）
  - jog_gate（attributes.yaml は ament_index_cpp が指す th_state の share か
    attributes_yaml_path パラメータで与える）
"""

import time
import unittest

import pytest
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Twist
from th_system_msgs.msg import SystemState


def _real_attributes_path():
    # th_state の share/config/attributes.yaml を ament_index で解決する
    # （th_state はビルド済み依存。無ければフォールバックして既定の
    # ament_index_cpp 解決に任せる）。
    try:
        from ament_index_python.packages import get_package_share_directory
        import os
        return os.path.join(
            get_package_share_directory('th_state'), 'config', 'attributes.yaml')
    except Exception:
        return ''


@pytest.mark.launch_test
def generate_test_description():
    attrs_path = _real_attributes_path()

    twist_mux = launch_ros.actions.Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[{
            'topics': {
                'manual': {
                    'topic':    '/cmd_vel_manual',
                    'timeout':  0.5,
                    'priority': 30,
                },
                'behavior': {
                    'topic':    '/cmd_vel_behavior',
                    'timeout':  0.5,
                    'priority': 20,
                },
                'nav': {
                    'topic':    '/cmd_vel_nav',
                    'timeout':  0.5,
                    'priority': 10,
                },
            },
        }],
        remappings=[('cmd_vel_out', '/cmd_vel_muxed')],
        output='screen',
    )

    jog_gate_params = [{'state_stale_ms': 1500}]
    if attrs_path:
        jog_gate_params.append({'attributes_yaml_path': attrs_path})

    jog_gate = launch_ros.actions.Node(
        package='th_safety',
        executable='jog_gate',
        name='jog_gate',
        parameters=jog_gate_params,
        output='screen',
    )

    return launch.LaunchDescription([
        twist_mux,
        jog_gate,
        launch_testing.actions.ReadyToTest(),
    ]), {'jog_gate': jog_gate, 'twist_mux': twist_mux}


class TestJogGateNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_jog_gate_node')

        self._muxed: list[Twist] = []
        self._manual: list[Twist] = []
        self.node.create_subscription(
            Twist, '/cmd_vel_muxed', lambda m: self._muxed.append(m), 10)
        # jog_gate の出力そのもの。J-1 は「publish 回数が 0」なので、
        # muxed 経由ではなくここを直接数える。
        self.node.create_subscription(
            Twist, '/cmd_vel_manual', lambda m: self._manual.append(m), 10)

        # **QoS を state_manager と揃える**（reliable + transient_local, depth 1）。
        # jog_gate 側は transient_local で購読しているので、既定の volatile で
        # publish すると QoS 非互換で**1 通も届かない**。初版はこれを踏んでおり、
        # jog_gate が /system/state を一度も受信しないまま「常に沈黙」していた
        # ——そのせいで「塞ぐときにゼロを撃つ」変異も「通すときに値を潰す」変異も
        # 素通りした（2026-09-01 に変異テストで実測）。
        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_state    = self.node.create_publisher(SystemState, '/system/state', state_qos)
        self.pub_raw      = self.node.create_publisher(Twist, '/cmd_vel_manual_raw', 10)
        self.pub_behavior = self.node.create_publisher(Twist, '/cmd_vel_behavior', 10)

        self._state = SystemState()
        self._state.mode = 'IDLE'
        self._state.state = 'NONE'

        # 探索（discovery）が済むまで待つ。ここで待たないと最初の数通が
        # 誰にも届かず、以降の判定が「送っていないから出ない」と区別できない。
        time.sleep(1.0)
        self._pump(0.6, state=True)
        self._muxed.clear()
        self._manual.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _pump(self, sec: float, *, state: bool = True, raw: Twist = None,
              behavior: Twist = None, hz: float = 20.0):
        """sec 秒のあいだ、指定されたものを hz で publish しながら spin する。

        `state=False` にすると /system/state を送らない（stale を作るため）。
        raw と behavior を同時に渡せることが重要（J-1 の帰結の検証）。
        """
        period = 1.0 / hz
        deadline = time.time() + sec
        while time.time() < deadline:
            if state:
                self.pub_state.publish(self._state)
            if raw is not None:
                self.pub_raw.publish(raw)
            if behavior is not None:
                self.pub_behavior.publish(behavior)
            rclpy.spin_once(self.node, timeout_sec=period)

    @staticmethod
    def _twist(vx: float, wz: float) -> Twist:
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        return t

    # ════════════════════════════════════════════════════════
    # J-1 そのもの: 塞ぐ場面では /cmd_vel_manual に 1 通も出ない
    # ════════════════════════════════════════════════════════
    def test_silent_when_state_stale(self):
        # state_stale_ms = 1500。それを超えて /system/state を送らない。
        self._pump(2.2, state=False)
        self._manual.clear()

        # stale のまま手動指令を 1 秒ぶん送り込む
        self._pump(1.0, state=False, raw=self._twist(0.5, 0.3))

        self.assertEqual(
            len(self._manual), 0,
            f'/system/state が stale のあいだに /cmd_vel_manual へ '
            f'{len(self._manual)} 通 publish された。J-1 違反'
            f'（塞ぐときは沈黙する。ゼロも撃たない）')

    # ════════════════════════════════════════════════════════
    # J-1 の帰結: jog_gate が黙っている間、priority 20 が選ばれる
    # **raw と behavior を同時に流す**（これが検出力の要）
    # ════════════════════════════════════════════════════════
    def test_mux_can_select_priority_20(self):
        raw = self._twist(0.5, 0.5)        # IDLE なので通らないはず
        beh = self._twist(0.2, 0.1)        # priority 20

        self._pump(0.5, raw=raw, behavior=beh)
        self._manual.clear()
        self._muxed.clear()
        self._pump(1.2, raw=raw, behavior=beh)

        # ① jog_gate は 1 通も出していない（IDLE = denied）
        self.assertEqual(
            len(self._manual), 0,
            f'IDLE で /cmd_vel_manual に {len(self._manual)} 通出ている。'
            f'jog_gate が沈黙していない（J-1 違反）')

        # ② その結果として behavior（priority 20）が muxed に出る
        self.assertTrue(self._muxed, '/cmd_vel_muxed に出力が無い')
        out = self._muxed[-1]
        self.assertAlmostEqual(
            out.linear.x, 0.2, places=2,
            msg=f'behavior が /cmd_vel_muxed に出ていない: linear.x={out.linear.x:.3f}。'
                f'jog_gate がゼロを撃つと priority 30 が常に非タイムアウトになり、'
                f'twist_mux は priority 20/10 を永久に選ばない（J-1 の理由そのもの）')

    # ════════════════════════════════════════════════════════
    # J-3: 通す場面では速度を変えずそのまま転送する
    # ════════════════════════════════════════════════════════
    def test_forwards_unchanged_when_allowed(self):
        # MANUAL は attributes.yaml で jog: is_drive → 通す
        self._state.mode = 'MANUAL'
        self._state.state = 'RUN'
        self._pump(0.5)
        self._manual.clear()

        raw = self._twist(0.37, -0.21)
        self._pump(1.0, raw=raw)

        self.assertTrue(
            self._manual,
            'MANUAL（is_drive）なのに /cmd_vel_manual に 1 通も出ていない。'
            'is_drive を denied と同じに扱うと MANUAL で走れなくなる（FMEA ③）')
        out = self._manual[-1]
        self.assertAlmostEqual(
            out.linear.x, 0.37, places=6,
            msg=f'linear.x が変わっている: {out.linear.x!r}。'
                f'jog_gate はゲートであってリミッタではない（J-3）。'
                f'クランプは obstacle_limiter の仕事')
        self.assertAlmostEqual(
            out.angular.z, -0.21, places=6,
            msg=f'angular.z が変わっている: {out.angular.z!r}（J-3）')
