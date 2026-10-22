"""
test_map_freezer_node.py — 地図更新 OFF の間 /map を凍結する本番の経路（Spec-onsite §4.0.2）。

本物の map_freezer ノードを起動し、slam_toolbox の代役（/map_live）・state_manager の
代役（/system/state）・slam_control の代役（/map_session/freeze_anchor）を試験側から流して、
購読者が見る /map を実際に購読して確かめる。地図は 2x2 の格子で、data[0] を目印に使う。

  i.   地図を開いたあと、凍結中は /map_live が変わっても /map の出力が変わらない
  ii.  地図更新 ON にすると /map_live がそのまま通る
  iii. 保存せず OFF に戻すと、開いた地図の写しに戻る
  iv.  開き直す（opening → opened）と写しを取り直す。開く前の地図は掴まない
  v.   ON → 保存（saving → saved）→ OFF で、保存時点の地図で凍結する
  vi.  凍結中は写しを定期的に再配信する（後から購読した側にも届く）
  vii. PREP では通す（対象外）

rclpy が要るためホストでは走らない（Docker のみ）。
変異: map_freezer の `_cb_live` で凍結中も publish する → i が赤 / `on_anchor` の opened で
写しを取らない → iv が赤 / saved で写しを更新しない → v が赤。
"""
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from th_system_msgs.msg import SystemState

from th_config_manager.map_freezer_core import make_anchor


@pytest.mark.launch_test
def generate_test_description():
    freezer = launch_ros.actions.Node(
        package='th_config_manager', executable='map_freezer.py',
        name='map_freezer', parameters=[{'republish_period_s': 1.0}],
        output='screen')
    return launch.LaunchDescription([
        freezer, launch_testing.actions.ReadyToTest(),
    ]), {'map_freezer': freezer}


_LATCH = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    history=QoSHistoryPolicy.KEEP_LAST)
_ANCHOR = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)


def _grid(tag):
    m = OccupancyGrid()
    m.header.frame_id = 'map'
    m.info.resolution = 0.05
    m.info.width = 2
    m.info.height = 2
    m.data = [tag, 0, 0, 0]
    return m


class TestMapFreezer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_map_freezer')
        self.maps = []           # /map で受けた目印の列
        self.seq = 0
        self.node.create_subscription(
            OccupancyGrid, '/map', lambda m: self.maps.append(int(m.data[0])), _LATCH)
        self.p_live = self.node.create_publisher(OccupancyGrid, '/map_live', _LATCH)
        self.p_state = self.node.create_publisher(SystemState, '/system/state', _LATCH)
        self.p_anchor = self.node.create_publisher(
            String, '/map_session/freeze_anchor', _ANCHOR)
        time.sleep(3.0)
        self.pump(0.5)

    def tearDown(self):
        self.node.destroy_node()

    def pump(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def live(self, tag):
        self.p_live.publish(_grid(tag))
        self.pump(0.6)

    def state(self, mode, update):
        m = SystemState()
        m.mode = mode
        m.map_update = update
        self.p_state.publish(m)
        self.pump(0.6)

    def anchor(self, event):
        self.seq += 1
        self.p_anchor.publish(String(data=make_anchor('t', self.seq, event)))
        self.pump(0.6)

    def last(self):
        return self.maps[-1] if self.maps else None

    def test_freeze_flow(self):
        # 地図を開く（opening → 開いた直後の地図 → opened）
        self.state('REPLAY', False)
        self.live(5)                       # 開く前の PREP の地図。掴まない
        self.anchor('opening')
        self.live(10)                      # deserialize 直後の地図
        self.anchor('opened')
        self.assertEqual(self.last(), 10)

        # i. 凍結中は /map_live が変わっても /map は変わらない
        n = len(self.maps)
        self.live(20)
        self.live(21)
        self.assertEqual(self.last(), 10)
        self.assertNotIn(20, self.maps[n:])
        self.assertNotIn(21, self.maps[n:])

        # vi. 凍結中は写しを再配信する
        n = len(self.maps)
        self.pump(2.5)
        self.assertGreaterEqual(self.maps[n:].count(10), 1)
        self.assertEqual(set(self.maps[n:]), {10})

        # ii. 地図更新 ON で /map_live が通る
        self.state('REPLAY', True)
        self.assertEqual(self.last(), 21)
        self.live(22)
        self.assertEqual(self.last(), 22)

        # iii. 保存せず OFF → 開いた地図の写し
        self.state('REPLAY', False)
        self.assertEqual(self.last(), 10)
        self.live(23)
        self.assertEqual(self.last(), 10)

        # iv. 開き直すと写しを取り直す（開く前の地図 23 は掴まない）
        self.anchor('opening')
        self.live(30)
        self.anchor('opened')
        self.assertEqual(self.last(), 30)
        self.live(31)
        self.assertEqual(self.last(), 30)

        # v. ON → 保存 → OFF で保存時点の地図
        self.state('REPLAY', True)
        self.live(40)
        self.anchor('saving')
        self.live(41)
        self.anchor('saved')
        self.live(42)
        self.state('REPLAY', False)
        self.assertEqual(self.last(), 40)
        self.live(43)
        self.assertEqual(self.last(), 40)

        # vii. PREP では通す
        self.state('PREP', False)
        self.live(50)
        self.assertEqual(self.last(), 50)
        self.state('REPLAY', False)
        self.assertEqual(self.last(), 40)
