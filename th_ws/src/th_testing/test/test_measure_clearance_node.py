"""
test_measure_clearance_node.py
==============================
8-g P5b — 本物の `measure_clearance` を生成 yaml 先渡しで起動して縛る試験。
正本は docs/plan/spec/Spec-measure.md §4.4 と DetailedDesign-measure.md §6.2 P5b。

起動は「生成 yaml を先に渡す」本番の経路（P5a の
test_auto_measure_node.py と同じ）。試験側が TF（`base_link`←`laser_link`・
`map`←`base_link`）と `/scan_filtered`・`/local_costmap/costmap` を出す。

  a. TF が無い → clear=false（reason tf_unavailable）
  b. 円の外（1.0 m）だけの点 → clear=true（reason clear）。5 Hz で出ている
  c. 0.2 m に点 → clear=false（reason obstacle_scan）
  d. /scan_filtered を止める → clear=false（reason scan_stale）
  e. /local_costmap/costmap の円内に 100 のセル → clear=false（reason obstacle_costmap）

注意（launch_testing の挙動）: ノードはファイル単位で共有されるため、
TF が一度届くと tf2 のバッファに残って消えない。TF 無しの a を先に置く
（メソッドの alphabetical 順で a が最初）。
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

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

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
        _GEN_DIR = tempfile.mkdtemp(prefix='measure_clearance_test_gen_')
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
    measure_clearance = launch_ros.actions.Node(
        package='th_measure',
        executable='measure_clearance.py',
        name='measure_clearance',
        parameters=[_generated_yaml('measure_clearance.yaml')],
        output='screen',
    )
    return launch.LaunchDescription([
        measure_clearance,
        launch_testing.actions.ReadyToTest(),
    ]), {}


_LATCHED_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestMeasureClearanceNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_measure_clearance_node')
        self._lock = threading.Lock()
        self._statuses = []
        self.node.create_subscription(
            String, '/measure/corner_status', self._on_status, 20)

        self.tf_live = False
        self.scan_live = True
        self.scan_range_m = 1.0
        self.costmap_blocked = False

        self.pub_tf = self.node.create_publisher(
            TFMessage, '/tf_static', _LATCHED_QOS)
        self._tf_timer = self.node.create_timer(0.2, self._publish_tf)
        self.pub_scan = self.node.create_publisher(
            LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self._scan_timer = self.node.create_timer(0.1, self._publish_scan)
        self.pub_costmap = self.node.create_publisher(
            OccupancyGrid, '/local_costmap/costmap', _LATCHED_QOS)
        self._costmap_timer = self.node.create_timer(1.0, self._publish_costmap)

    def tearDown(self):
        self.tf_live = False
        self.scan_live = True
        for t in (getattr(self, '_tf_timer', None),
                  getattr(self, '_scan_timer', None),
                  getattr(self, '_costmap_timer', None)):
            if t is not None:
                try:
                    self.node.destroy_timer(t)
                except Exception:
                    pass
        self.node.destroy_node()

    # ── 試験側の出版 ──────────────────────────────────────
    def _on_status(self, msg):
        with self._lock:
            self._statuses.append((time.monotonic(), msg.data))

    def _now_msg(self):
        return self.node.get_clock().now().to_msg()

    def _publish_tf(self):
        if not self.tf_live:
            return
        stamp = self._now_msg()
        transforms = []
        for parent, child in (('base_link', 'laser_link'), ('map', 'base_link')):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.pub_tf.publish(TFMessage(transforms=transforms))

    def _publish_scan(self):
        if not self.scan_live:
            return
        msg = LaserScan()
        msg.header.stamp = self._now_msg()
        msg.header.frame_id = 'laser_link'
        msg.angle_min = 0.0
        msg.angle_increment = 0.1
        msg.range_min = 0.05
        msg.range_max = 10.0
        msg.ranges = [float(self.scan_range_m)]
        self.pub_scan.publish(msg)

    def _publish_costmap(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.05
        msg.info.width = 40
        msg.info.height = 40
        msg.info.origin.position.x = -1.0
        msg.info.origin.position.y = -1.0
        msg.info.origin.orientation.w = 1.0
        data = [0] * (40 * 40)
        if self.costmap_blocked:
            # （0.1, 0.1) を含むセルに 100 を入れる（円内）。
            data[22 * 40 + 22] = 100
        msg.data = data
        self.pub_costmap.publish(msg)

    # ── 補助 ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _latest(self):
        with self._lock:
            if not self._statuses:
                return None
            return json.loads(self._statuses[-1][1])

    def _wait_reason(self, reason, clear, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._spin(0.1)
            v = self._latest()
            if v is not None and v['reason'] == reason and v['clear'] is clear:
                return True
        return False

    # ════════════════════════════════════════════════════════
    def test_a_tf_unavailable(self):
        """a. TF が無い → clear=false（reason tf_unavailable）。"""
        self.tf_live = False
        self.scan_range_m = 1.0
        assert self._wait_reason('tf_unavailable', False), self._latest()

    def test_b_clear_outside_circle(self):
        """b. 円の外（1.0 m）だけの点 → clear=true（reason clear）。5 Hz で出ている。"""
        self.tf_live = True
        self.scan_range_m = 1.0
        self.costmap_blocked = False
        self._publish_tf()
        self._publish_costmap()
        assert self._wait_reason('clear', True), self._latest()
        # 5 Hz: 1 秒で 3 件以上届く。
        with self._lock:
            before = len(self._statuses)
        self._spin(1.0)
        with self._lock:
            count = len(self._statuses) - before
        assert count >= 3, f'/measure/corner_status が 1 秒で {count} 件しか届かない'

    def test_c_obstacle_scan(self):
        """c. 0.2 m に点 → clear=false（reason obstacle_scan）。"""
        self.tf_live = True
        self.scan_range_m = 0.2
        self.costmap_blocked = False
        self._publish_tf()
        self._publish_costmap()
        assert self._wait_reason('obstacle_scan', False), self._latest()

    def test_d_scan_stale(self):
        """d. /scan_filtered を止める → clear=false（reason scan_stale）。"""
        self.tf_live = True
        self.scan_live = False
        self.costmap_blocked = False
        self._publish_tf()
        self._publish_costmap()
        time.sleep(3.0)  # arm_clearance_scan_max_age_ms（500）の十分外
        assert self._wait_reason('scan_stale', False), self._latest()

    def test_e_obstacle_costmap(self):
        """e. 円内に 100 のセル → clear=false（reason obstacle_costmap）。"""
        self.tf_live = True
        self.scan_live = True
        self.scan_range_m = 1.0
        self.costmap_blocked = True
        self._publish_tf()
        self._publish_costmap()
        assert self._wait_reason('obstacle_costmap', False), self._latest()
