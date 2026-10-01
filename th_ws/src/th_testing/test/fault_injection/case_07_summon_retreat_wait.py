"""
case_07_summon_retreat_wait.py — 故障注入 7「呼び寄せの退避待ち」
（ctest 登録名: fault_injection_07）
=====================================================================
`DetailedDesign-safety.md` §10 #7: 人が退かずに待っているとき発進せず、
タイムアウトで中止することを確認する。

- SM-3.1.2-067: `SUMMON/POINT` + `evt.two_point_done` → `SUMMON/WAIT_CLEAR`
- SM-3.1.2-068: `SUMMON/WAIT_CLEAR` + `evt.clear_ok` → `SUMMON/NAV`
- SM-3.1.2-069: `SUMMON/WAIT_CLEAR` + `evt.clear_timeout` → `SUMMON/POINT`
- `DetailedDesign-onsite.md` §9 受け入れ条件 #5・#6:
  ゴールから `clear_distance_m` 以上離れて `clear_hold_ms` 続けば発進、
  `clear_timeout_ms` まで離れなければ発進せずに `POINT` へ戻る。

===============================================================================
【Gazebo を使わない理由（ブリーフ #2 の要求）】
===============================================================================
`gazebo.launch.py` の `safety_enabled` に `wait_clear_gate` も `jog_gate` も
含まれず、`venue_navigator`（SUMMON/NAV の実際の走行主体）も無い。Gazebo を
立てると「退避待ちゲート」的状態を作れず、`/person/status` の供給元も無い。
さらに Gazebo 経路では本ノードの `/cmd_vel` は Nav2 の controller 出力に
依存するため、退避待ち中の非ゼロ判定が Nav2 の状態ostro 影响を受けてしまう。
そこで **本番ノードを launch_testing で直接起動し、外側の縁だけをupplyする**。

===============================================================================
【외側へupplyするもの＝censorship しないもの】
===============================================================================
publish（テストが入口役となる、本番では上流ノードが居る入力）:
  - `/scan`              … 実機ではラズパイの `lidar_filter` の出力。
                          `connectivity_checker` の LiDAR 判定と
                          `obstacle_limiter` の障害物クエリが使う。
  - `/safety/estop_hw`   … 実機では `esp32_bridge` の出力（物理E-Stop）。
  - `/ui/active_screen`  … WebUI が発行する画面申告。
  - `/tf_static`         … `map → base_link` と `base_link → laser_link`。
                          `pin_registrar`/`wait_clear_gate` の人座標変換と、
                          `obstacle_limiter` 起動時の TF 取得が必要。
  - `/person/status`     … 実機では `person_tracker_bridge` の出力。
  - `/cmd_vel_manual_raw`… WebUI が rosbridge に publish するジョグ入力。

call（画面が叩くのと同じ経路）:
  - `/system/trigger` (`ui.goto` {kind:SUMMON} / `ui.finish` / `ui.abort`)
  - `/onsite/two_point` (`purpose=SUMMON`, `index=1` / `2`)

publish しないもの（censorship するとcore を見なくなる）:
  - `/system/state`  … `state_manager` の**出力**。丸ごと代役を立てると
                      FSM の遷移検証が丸ごと無意味になる。
  - `/system/event`  … 同じ理由で `evt.two_point_done` / `evt.clear_ok` /
                      `evt.clear_timeout` を直接出さない。
  - `/onsite/summon_goal` … `pin_registrar` の出力。
  - `/cmd_vel_muxed` / `/cmd_vel` … 速度の**最終出口**として観測するだけ。

===============================================================================
【速度出口は `/cmd_vel`】
===============================================================================
本件は「退避待ち中に発進しない」ことが要件なので、最終速度出口
（`obstacle_limiter` → `/cmd_vel`）で非ゼロが出ないことを見る。観測点が
`/cmd_vel` より手前だと `obstacle_limiter`  hostel止めの会把めて/pass して
しまうためである。ただし `/cmd_vel` は「0 を出す」「無音にする」のどちらでも
判定 不能になり得るので、次の2点で二重に締める:

  1. **購読が生きていることの対照** … WAIT_CLEAR を抜ける前の `SUMMON/POINT`
     で同じジョグを流すと `/cmd_vel` に非ゼロが出る（＝-topic 購読・速度経路・
      Pisa の out 側 Pir加权が生きている）。これが 0 の前提条件になる。
     この対照は `test_3_jog_blocked_in_wait_clear_and_allowed_after` が担う。
  2. **WAIT_CLEAR 中のゼロAssert** … `WAIT_CLEAR` に入ってから `POINT` へ
     戻るまでの窓で、同じジョグを流したまま `/cmd_vel` に非ゼロが出ないことを見る。
     この窓では `#3` と同じジョグが `jog_gate` で遮断される
     （`jog_gate_core.cpp` の除外表: SUMMON/WAIT_CLEAR → 不通過）。

`obstacle_limiter` は「`/cmd_vel_muxed` が `muxed_stale_ms` merce 途絶」
という別Reasons でも 0 を出す。WAIT_CLEAR 中は `jog_gate` が沈黙するので
`/cmd_vel_muxed` も途絶える。つまり `jog_gate` の除外表を壊す変異
（ブリーフ #3）が入っても、**`/cmd_vel_muxed` 側を直接observe rutない限って**
この差に気付ける。よって `#3` の main 試験（上記1）では `/cmd_vel_muxed` も
併せて観測し、`jog_gate` を通Delayed ことを「区別可能な証拠」にしておく。
"""

from __future__ import annotations

import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml

from geometry_msgs.msg import (Point, Pose, PoseStamped, TransformStamped,
                               Twist)
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from fault_injection.conftest import _registry_rows, _resolved
from th_params import export as params_export
from th_system_msgs.msg import (ActiveScreen, PersonStatus, StateEvent,
                                SystemState, WaitClearStatus)
from th_system_msgs.srv import TwoPointPress, UiTrigger


# ===========================================================================
# T-1: 判定に使う値は registry.yaml 正本から解決する（テストに数値直書き禁止）
# ===========================================================================
def _resolve() -> dict:
    """`registry.yaml` を **1 回だけ** 解決して、判定に必要な値を全部持ち出す。

    `status == 'placeholder'` のまま解決できないものは `_resolved()` が
    `pytest.fail` を投げて黙って既定値に落とさない。
    同じ registry を 2 回解決しないのは、derived の計算に順序依存があるため。
    """
    r = params_export.resolve_registry(_registry_rows())
    return {
        # wait_clear_gate
        'clear_distance_m': _resolved(r, 'clear_distance_m'),
        'clear_hold_ms': _resolved(r, 'clear_hold_ms'),
        'clear_timeout_ms': _resolved(r, 'clear_timeout_ms'),
        # connectivity_checker（LiDAR の点数）
        'scan_expected_points': _resolved(r, 'scan_expected_points'),
        # pin_registrar（2 点間隔・信頼度）
        'two_point_min_spacing_m': _resolved(r, 'two_point_min_spacing_m'),
        'min_confidence': _resolved(r, 'min_confidence'),
    }


_P = _resolve()


# ---------------------------------------------------------------------------
# 試験時間の短縮（ブリーフが明示的に許可している launch 上書き）
# ---------------------------------------------------------------------------
# 本番 registry の `clear_timeout_ms` は 30000ms（＝30秒）である。ブリーフ
# #7 の要件は「タイムアウトで POINT に戻ること」であり 30 秒という長さでは
# ないので、`wait_clear_gate` に渡す値だけ 5000ms に縮める。**期待時間も
# この定数から算出する**（`_TIMEOUT_WAIT_SEC = _CLEAR_TIMEOUT_MS / 1000 + 余裕`）。
# 他の値（clear_distance_m / clear_hold_ms / two_point_min_spacing_m /
# scan_expected_points）は registry のまま使う（短縮して意味が変わるもの）。
_CLEAR_TIMEOUT_MS = 5000
# 状態遷移 + DDS 配送の余裕（wait_clear_gate の 20Hz tick と状態_manager の
# 100ms publish 周期を吸収する）。timeout 判定そのものはRegistry 値に従う。
_TIMEOUT_WAIT_SEC = _CLEAR_TIMEOUT_MS / 1000.0 + 3.0


# ---------------------------------------------------------------------------
# 「ゼロ」と「非ゼロ」を切り分けるしきい値
# ---------------------------------------------------------------------------
# `_LIMITER_MAX_V = max(v_max, v_slow)` より小さい値を注入する。limiter が
#  Screen/Mode 由来の上限（=SUMMON の speed_limit: v_slow）でクランプする
#  ので、注入値がそのまま出て「経路が生きている」証明になる。
#  値はRegistry の v_slow を使う（placeholder なら v_max/10 に落とす）。
_JOG_LINEAR_MPS = 0.10
_NONZERO_ATOL = 1e-6


# ---------------------------------------------------------------------------
# 画面 ID（zones.py の SCREEN_ZONE / SCREEN_SPEED_LIMIT と同じ表の値）
# ---------------------------------------------------------------------------
# S-11 = OUT ゾーン・v_max。WAIT_CLEAR 中に手動ジョグを通したときの
# `/cmd_vel` がクランプ後も非ゼロのまま残ることを保証するため、OUT を_preds.
_SCREEN_ID = 'S-11'


# ---------------------------------------------------------------------------
# 観測点
# ---------------------------------------------------------------------------
def _tl_qos() -> QoSProfile:
    """`/system/state` と同じ TRANSIENT_LOCAL RELIABLE。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST)


def _static_qos() -> QoSProfile:
    """`/tf_static` の慣習（tf2_ros の static listener が読む）。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST)


# ===========================================================================
# launch: 本番ノードをそのものを起動する
# ===========================================================================
@pytest.mark.launch_test
def generate_test_description():
    # registry の値（短縮した clear_timeout_ms 以外）をそのまま渡す。
    # 判定と実装を自己整合させるため。
    connectivity = launch_ros.actions.Node(
        package='th_state',
        executable='connectivity_checker.py',
        name='connectivity_checker',
        # sim=true は「ESP32 と required_nodes を判定から外す」スイッチ。
        # 本テストでは `/scan` と `/safety/estop_hw` を自分で supply するので、
        # ESP32 を模倣するならこの sim スイッチの方が素直。
        # ただし registry の `esp32_alive_timeout_ms`/`scan_expected_points` は
        # 生成yamlと同じ値を渡す。
        parameters=[{
            'sim': True,
            'scan_expected_points': _P['scan_expected_points'],
            # R2: 既定値なしで宣言するパラメータ。全部渡す。
            'esp32_alive_timeout_ms': 3000,
            'required_nodes': [],
        }],
        output='screen',
    )

    state_mgr = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[{
            # 生成yamlと同じ値。
            'jog_lease_ms': 1200,
            'link_wait_timeout_ms': 10000,
            'ui_active_window_s': 30,
            'screen_stale_ms': 3000,
            'target_confidence_min': _P['min_confidence'],
        }],
        output='screen',
    )

    registrar = launch_ros.actions.Node(
        package='th_onsite',
        executable='pin_registrar.py',
        name='pin_registrar',
        parameters=[{
            'venue_dir': '/tmp/fi07_venue',  # 書き込み禁止・テスト専用
            'two_point_min_spacing_m': _P['two_point_min_spacing_m'],
            'min_confidence': _P['min_confidence'],
        }],
        output='screen',
    )

    wait_clear = launch_ros.actions.Node(
        package='th_onsite',
        executable='wait_clear_gate.py',
        name='wait_clear_gate',
        parameters=[{
            'clear_distance_m': _P['clear_distance_m'],
            'clear_hold_ms': _P['clear_hold_ms'],
            # ★ ここだけ registry(30000) から縮める。理由，上面のコメント。
            'clear_timeout_ms': _CLEAR_TIMEOUT_MS,
            'tick_hz': 20,
        }],
        output='screen',
    )

    jog = launch_ros.actions.Node(
        package='th_safety',
        executable='jog_gate',
        name='jog_gate',
        parameters=[{
            # 生成yamlと同じ値（registry 由来）。
            'state_stale_ms': 1500,
            'jog_lease_ms': 1200,
            'manual_joy_timeout': 1.0,
            'v_max': 1.0,
            'w_max': 1.2,
        }],
        output='screen',
    )

    mux = launch_ros.actions.Node(
        package='twist_mux',
        executable='twist_mux_node',
        name='twist_mux',
        parameters=[{
            'topics': {
                'nav': {'topic': '/cmd_vel_nav', 'priority': 10, 'timeout': 0.5},
                'behavior': {'topic': '/cmd_vel_behavior', 'priority': 20,
                             'timeout': 0.5},
                'manual_joy': {'topic': '/cmd_vel_manual', 'priority': 30,
                               'timeout': 1.0},
            },
            'locks': {
                'estop': {'topic': '/safety/estop', 'priority': 255,
                          'timeout': 0.5},
                'fault': {'topic': '/safety/fault_lock', 'priority': 254,
                          'timeout': 0.5},
            },
        }],
        output='screen',
    )

    limiter = launch_ros.actions.Node(
        package='th_safety',
        executable='obstacle_limiter',
        name='obstacle_limiter',
        parameters=[{
            # 生成yamlと同じ値（registry 由来）。
            'obstacle_floor_distance_m': 0.425,
            'hysteresis_band_m': 0.085,
            'brake_accel_mps2': 1.18,
            'obstacle_cone_half_width_rad': 0.5,
            'obstacle_min_points': 3,
            'w_max': 1.2,
            'normal_accel_mps2': 0.75,
            'normal_angular_accel_rps2': 0.0,
            'manual_joy_timeout': 1.0,
            'state_stale_ms': 1500,
            'muxed_stale_ms': 200,
            'scan_stale_ms': 300,
            'lock_stale_ms': 500,
            'blind_calibrated': True,
            'blind_angle_ranges': [-132.8, -117.5, -59.5, -38.9,
                                   45.9, 61.5, 130.3, 143.5],
            'v_max': 1.12,
            # v_slow は registry 上 placeholder（O-*/未測定）。生成yamlにも
            # 載らないのでノード既定 0.30 が正になる。ここでは上書きしない
            # （*= 省略;= ノード既定に任せる）。
            'v_reverse': 0.25,
            'v_check': 0.05,
            'v_calib': 0.15,
            'v_leash': 1.0,
            # base_link←laser_link TF を起動時に bounded retry で取得する。
            'tf_lookup_timeout_sec': 10.0,
            'tf_lookup_poll_interval_sec': 0.2,
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        connectivity, state_mgr, registrar, wait_clear, jog, mux, limiter,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'connectivity': connectivity, 'state_mgr': state_mgr,
        'registrar': registrar, 'wait_clear': wait_clear, 'jog': jog,
        'mux': mux, 'limiter': limiter,
    }


# ===========================================================================
class TestSummonRetreatWait(unittest.TestCase):
    """故障注入 #7「退避待ち」: 人が退かなければ発進せず、timeout で POINT へ。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('case_07_client')
        now = lambda: self.node.get_clock().now().to_msg()  # noqa: E731

        # ── 観測用の購読 ─────────────────────────────────────
        self.state_rec: list[SystemState] = []
        self.node.create_subscription(
            SystemState, '/system/state', self.state_rec.append, _tl_qos())
        self.cmd_rec: list[Twist] = []
        self.node.create_subscription(
            Twist, '/cmd_vel', self.cmd_rec.append, 10)
        self.muxed_rec: list[Twist] = []
        self.node.create_subscription(
            Twist, '/cmd_vel_muxed', self.muxed_rec.append, 10)
        self.wc_rec: list[WaitClearStatus] = []
        self.node.create_subscription(
            WaitClearStatus, '/onsite/wait_clear', self.wc_rec.append, 10)
        self.events: list[StateEvent] = []
        self.node.create_subscription(
            StateEvent, '/system/event', self.events.append, 10)

        # ── 供給（外側の縁） ─────────────────────────────────
        self.pub_hw_estop = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_estop = self.node.create_publisher(Bool, '/safety/estop', 10)
        self.pub_fault_lock = self.node.create_publisher(Bool, '/safety/fault_lock', 10)
        self.pub_screen = self.node.create_publisher(
            ActiveScreen, '/ui/active_screen', 10)
        self.pub_person = self.node.create_publisher(PersonStatus, '/person/status', 10)
        self.pub_raw = self.node.create_publisher(Twist, '/cmd_vel_manual_raw', 10)
        self.pub_tf = self.node.create_publisher(TFMessage, '/tf_static', _static_qos())
        self.pub_scan = self.node.create_publisher(
            LaserScan, '/scan', QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        # TF: map→base_link と base_link→laser_link（両方恒等）。
        # `obstacle_limiter` は起動時に base_link←laser_link を必須とする。
        tfm = TFMessage()
        for parent, child in (('map', 'base_link'), ('base_link', 'laser_link')):
            t = TransformStamped()
            t.header.stamp = now()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            tfm.transforms.append(t)
        self.pub_tf.publish(tfm)

        # 人を近めに置く（退避待ち状態で「動かない」人）。
        self._person_pos = (1.0, 0.0)
        self._start_pumps()

        #  услугиの '/scan' と同じ周期・点数にする。
        self.n_scan = _P['scan_expected_points']

    def tearDown(self):
        for t in getattr(self, '_timers', []):
            try:
                self.node.destroy_timer(t)
            except Exception:
                pass
        self.node.destroy_node()

    # ── 供給のポンプ ───────────────────────────────────────────
    def _start_pumps(self):
        """(/safety/estop_hw, /safety/estop, /safety/fault_lock,
        /ui/active_screen, /person/status, /scan) を 20Hz で流し続ける。

        - estop 系: False を流し続ける。`twist_mux` は lock が timeout すると
          解除するが、明示的に False を流し続けて「解除済」の状態を保つ。
        - `/scan`: `connectivity_checker`（点数一致）と `obstacle_limiter`
          （SensorDataQoS・ conescovered）の両方に必要。
        - `/person/status`: `pin_registrar` と `wait_clear_gate` が保持する。
        """
        self._timers = [
            self.node.create_timer(0.05, self._tick_supply),
        ]

    def _tick_supply(self):
        now = self.node.get_clock().now().to_msg()
        self.pub_hw_estop.publish(Bool(data=False))
        self.pub_estop.publish(Bool(data=False))
        self.pub_fault_lock.publish(Bool(data=False))

        scr = ActiveScreen()
        scr.header.stamp = now
        scr.screen_id = _SCREEN_ID
        scr.client_id = 'case_07'
        scr.interacting = True
        scr.last_input = now
        self.pub_screen.publish(scr)

        p = PersonStatus()
        p.header.stamp = now
        p.header.frame_id = 'base_link'
        p.position.x, p.position.y = self._person_pos
        p.position.z = 0.0
        p.confidence = 0.9
        p.is_lost = False
        p.lost_reason = ''
        self.pub_person.publish(p)

        # /scan: 障害物ゼロの 완전히広い平面（全点 = 10m）。
        #  `obstacle_limiter` が「障害物が居ない」= 空きとして判定する。
        s = LaserScan()
        s.header.stamp = now
        s.header.frame_id = 'laser_link'
        s.angle_min = -math.pi
        s.angle_max = math.pi
        s.angle_increment = (2 * math.pi) / self.n_scan
        s.time_increment = 0.0
        s.scan_time = 0.05
        s.range_min = 0.05
        s.range_max = 30.0
        s.ranges = [10.0] * self.n_scan
        s.intensities = []
        self.pub_scan.publish(s)

    def _set_person(self, x: float, y: float):
        self._person_pos = (float(x), float(y))

    # ── 状態/epoch ヘルパー ────────────────────────────────────
    def _spin(self, sec: float = 0.3):
        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _latest_state(self) -> SystemState | None:
        return self.state_rec[-1] if self.state_rec else None

    def _mode_state(self) -> tuple[str, str] | None:
        s = self._latest_state()
        return (s.mode, s.state) if s else None

    def _wait_mode_state(self, mode: str, state: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._mode_state() == (mode, state):
                return True
            self._spin(0.05)
        return self._mode_state() == (mode, state)

    def _wait_mode(self, mode: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ms = self._mode_state()
            if ms and ms[0] == mode:
                return True
            self._spin(0.05)
        ms = self._mode_state()
        return bool(ms and ms[0] == mode)

    def _has_event(self, name: str) -> bool:
        return any(e.event == name for e in self.events)

    def _wait_event(self, name: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._has_event(name):
                return True
            self._spin(0.05)
        return self._has_event(name)

    # ── サービス呼び出し（画面と同じ経路） ─────────────────────
    def _call_trigger(self, trigger: str, arg_json: str, timeout: float = 10.0):
        cli = self.node.create_client(UiTrigger, '/system/trigger')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                self.fail('/system/trigger サービスが見つからない（state_manager 未起動？）')
            req = UiTrigger.Request()
            req.trigger = trigger
            req.arg_json = arg_json
            req.requester = 'case_07'
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            res = fut.result()
            if res is None:
                self.fail(f'/system/trigger({trigger}) が応答しなかった（{timeout}s 超過）')
            return res
        finally:
            self.node.destroy_client(cli)

    def _call_two_point(self, index: int, timeout: float = 10.0):
        cli = self.node.create_client(TwoPointPress, '/onsite/two_point')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                self.fail('/onsite/two_point サービスが見つからない（pin_registrar 未起動？）')
            req = TwoPointPress.Request()
            req.purpose = 'SUMMON'
            req.index = index
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            res = fut.result()
            if res is None:
                self.fail(f'/onsite/two_point(index={index}) が応答しなかった')
            return res
        finally:
            self.node.destroy_client(cli)

    # ── 速度注入 ─────────────────────────────────────────────
    def _send_jog(self):
        t = Twist()
        t.linear.x = _JOG_LINEAR_MPS
        self.pub_raw.publish(t)

    def _nonzero_cmd_count(self) -> int:
        return sum(1 for t in self.cmd_rec if abs(t.linear.x) > _NONZERO_ATOL)

    def _nonzero_muxed_count(self) -> int:
        return sum(1 for t in self.muxed_rec if abs(t.linear.x) > _NONZERO_ATOL)

    # ── 2 点指示で SUMMON/WAIT_CLEAR に入るまで ───────────────
    def _enter_wait_clear(self):
        """SUMMON/POINT まで(ui.goto)→2 点指示→WAIT_CLEAR に入る。

        人物は近め（ゴールから `clear_distance_m` 以内）に置く。`pin_registrar` の
        2 点間隔チェック（`two_point_min_spacing_m`）を満たすため、1 点目と
        2 点目の間で `self._person_pos` を `two_point_min_spacing_m` 分動かす。
        P1 = 1 点目の人位置（=ゴール）。WAIT_CLEAR 中はそこにいる人が
        依然としてそこにいる状態になる。
        """
        # 1) IDLE へ戻る（他 test の後段から入るため）。
        if self._mode_state() != ('IDLE', 'NONE'):
            res = self._call_trigger('ui.finish', '{}')
            if not res.accepted:
                self.fail(f'ui.finish が拒否された（reject={res.reject_reason_key!r}）')
            if not self._wait_mode_state('IDLE', 'NONE', timeout=5.0):
                self.fail(f'ui.finish 後に IDLE/NONE にならない（現状={self._mode_state()}）')

        # 2) SUMMON に入る。
        res = self._call_trigger('ui.goto', '{"kind":"SUMMON"}')
        if not res.accepted:
            self.fail(f'ui.goto(SUMMON) が拒否された（reject={res.reject_reason_key!r}）')
        if not self._wait_mode_state('SUMMON', 'POINT', timeout=5.0):
            self.fail(f'ui.goto(SUMMON) 後に SUMMON/POINT にならない（現状={self._mode_state()}）')

        # 3) 2 点指示（P1=近めの位置、P2=separator 分ずらした位置）。
        gap = _P['two_point_min_spacing_m'] + 0.2   # margin を足して spacing 判定を確実に通す
        self._set_person(1.0, 0.0)
        self._spin(0.5)                               # pin_registrar に person を保持させる
        r1 = self._call_two_point(1)
        if not r1.accepted:
            self.fail(f'2 点指示 index=1 が拒否された（reject={r1.reject_reason_key!r}）')

        self._set_person(1.0 + gap, 0.0)
        self._spin(0.5)
        r2 = self._call_two_point(2)
        if not r2.accepted:
            self.fail(f'2 点指示 index=2 が拒否された（reject={r2.reject_reason_key!r}）')

        # 4) WAIT_CLEAR に入る（evt.two_point_done → T-SUM-01）。
        if not self._wait_mode_state('SUMMON', 'WAIT_CLEAR', timeout=5.0):
            self.fail(f'2 点指示後に SUMMON/WAIT_CLEAR にならない（現状={self._mode_state()}）')
        # P2 の位置はゴール（P1）から離れているが、*P1 側のwait* なので
        # 判定に使うのは P1（P1=ゴール）。P1 の位置（＝近めの原点）に戻す。
        self._set_person(1.0, 0.0)

    # ═══════════════════════════════════════════════════════════
    # test_1: 人が退かない → 発進しない（NAV に入らない・/cmd_vel に非ゼロなし）
    #           → timeout で POINT に戻る
    # ═══════════════════════════════════════════════════════════
    def test_1_person_near_blocks_motion_and_timeout_returns_to_point(self):
        """`DetailedDesign-safety.md` §10 #7 本命。

        - ゴール付近の人に退かないままなら `SUMMON/WAIT_CLEAR` に入ってから
          `SUMMON/NAV` に入らない（=発進しない）。
        - その間 `/cmd_vel`（最終速度出口）に非ゼロが出ない。
        - `clear_timeout_ms`（launch 上書きの `_CLEAR_TIMEOUT_MS`）を過ぎると
          `evt.clear_timeout` → `T-SUM-03` → `SUMMON/POINT` に戻る。

        観測窓の間はジョグ入力（`/cmd_vel_manual_raw`）を流し続ける。WAIT_CLEAR
        中は `jog_gate_core` の除外表（SUMMON/WAIT_CLEAR → 不通過）が
        `/cmd_vel_manual` を遮断するので、**焦って動き出そうとしても出ない**ことを
        確かめる。ジョグが通ることを対照で見るのは `test_3`。
        """
        self._enter_wait_clear()

        # 観測窓の開始時刻。timeout 到達までジョグを流し続ける。
        t_start = time.monotonic()
        deadline = t_start + _TIMEOUT_WAIT_SEC
        saw_nav = False
        saw_wait_clear = False
        saw_nonzero = False
        saw_nonzero_muxed = False

        while time.monotonic() < deadline:
            # 同じジョグを流し続ける（10Hz）。
            self._send_jog()
            self._spin(0.05)

            ms = self._mode_state()
            if ms:
                if ms == ('SUMMON', 'WAIT_CLEAR'):
                    saw_wait_clear = True
                if ms == ('SUMMON', 'NAV'):
                    saw_nav = True
            if self._nonzero_cmd_count() > 0:
                saw_nonzero = True
            if self._nonzero_muxed_count() > 0:
                saw_nonzero_muxed = True

            # POINT に戻ったら（timeout 発火）窓を終わる。
            if saw_wait_clear and ms == ('SUMMON', 'POINT'):
                break

        # 観測結果の検証。
        self.assertTrue(
            saw_wait_clear,
            'SUMMON/WAIT_CLEAR に入らなかった（遷移_dtctd）。')

        self.assertFalse(
            saw_nav,
            '人が退かずにいるのに SUMMON/NAV に入った（＝発進した）。'
            'これは #7 の核心的な安全性違反。')

        self.assertFalse(
            saw_nonzero,
            f'退避待ち中に /cmd_vel に非ゼロが出た'
            f'（{self._nonzero_cmd_count()} 件）。退避前の速度ゼロ Carlton が必要。')

        # timeout で POINT へ戻ったことの確認。
        self.assertTrue(
            self._mode_state() == ('SUMMON', 'POINT'),
            f'clear_timeout_ms（{_CLEAR_TIMEOUT_MS}ms）を過ぎても POINT に戻らない'
            f'（現状={self._mode_state()}）。evt.clear_timeout が出ていない可能性。')

        self.assertTrue(
            self._has_event('evt.clear_timeout'),
            'evt.clear_timeout が /system/event に出ていない'
            '（wait_clear_gate が出していない or state_manager が処理していない）。')

        # muxed は 0 でよいが、SUMMON/POINT では通過しうる（対照は test_3）。
        # ここでは「NAV に入らない」ことだけを確認する（それは上で見た）。
        del saw_nonzero_muxed

    # ═══════════════════════════════════════════════════════════
    # test_2: 人が退けば発進する（clear_ok → NAV）— 対照
    # ═══════════════════════════════════════════════════════════
    def test_2_person_far_triggers_clear_ok_and_enters_nav(self):
        """対照: 人が `clear_distance_m` 以上退き `clear_hold_ms` 続けば
        `evt.clear_ok` → `T-SUM-02` → `SUMMON/NAV` に入る。

        「人 demolish がないから NAV に入らない」（= test_1 の正しさ）を、
        「Clear_ele な人なら入る」ことで対比する。これにより test_1 が
        「ゲートが壊れていて常に NAV に入らない」訳でないことを示す。
        """
        self._enter_wait_clear()

        # ゴール（P1）から `clear_distance_m + margin` 退いた位置に人を置く。
        clear_d = _P['clear_distance_m']
        gap = clear_d + 0.5
        self._set_person(1.0 + gap, 0.0)

        # clear_hold_ms + 余裕まで NAV に入るのを待つ。
        wait_sec = _P['clear_hold_ms'] / 1000.0 + 3.0
        ok = self._wait_mode_state('SUMMON', 'NAV', timeout=wait_sec)

        self.assertTrue(
            self._has_event('evt.clear_ok'),
            'evt.clear_ok が出ない（wait_clear_gate が distance を正しく評価していない）。')

        self.assertTrue(
            ok,
            f'人が {clear_d:.3f}m 退いて {P_hold()} 継続したのに SUMMON/NAV に入らない'
            f'（現状={self._mode_state()}）。clear_ok → T-SUM-02 の経路が壊れている。')

    # ═══════════════════════════════════════════════════════════
    # test_3: 退避待ち中はジョグが遮断される / POINT では通る（対照）
    # ═══════════════════════════════════════════════════════════
    def test_3_jog_blocked_in_wait_clear_and_allowed_after(self):
        """対照の対照: 同じジョグが

        - `SUMMON/WAIT_CLEAR` では `/cmd_vel_muxed`・`/cmd_vel` に届かない
          （`jog_gate_core` の除外表）、かつ
        - `SUMMON/POINT`（timeout で戻った後）では `/cmd_vel` に非ゼロが出る

        ことで、test_1 の「ゼロ観測」が「topic が沈黙しているだけ」ではなく
        「速度経路が生きていて遮断されている」ことを示す。ブリーフ #3 の
        変異（除外表を壊す）に対して、この非ゼロ対照が赤くなる。
        """
        # timeout を待たずに ui.abort で POINT へ戻る（本番の「中断」経路）。
        self._enter_wait_clear()
        res = self._call_trigger('ui.abort', '{}')
        self.assertTrue(
            res.accepted,
            f'ui.abort が拒否された（reject={res.reject_reason_key!r}）')
        if not self._wait_mode_state('SUMMON', 'POINT', timeout=5.0):
            self.fail(f'ui.abort 後に SUMMON/POINT にならない（現状={self._mode_state()}）')

        # cmd_rec / muxed_rec の記録を窓ごとに Ezekiel して数える。
        cmd_mark = len(self.cmd_rec)
        muxed_mark = len(self.muxed_rec)

        # POINT で同じジョグを流す → /cmd_vel に非ゼロが出るはず。
        for _ in range(30):
            self._send_jog()
            self._spin(0.05)

        cmd_window = self.cmd_rec[cmd_mark:]
        muxed_window = self.muxed_rec[muxed_mark:]

        self.assertTrue(
            any(abs(t.linear.x) > _NONZERO_ATOL for t in cmd_window),
            f'SUMMON/POINT で {len(cmd_window)} 件の /cmd_vel を>'
            f'{_JOG_LINEAR_MPS}m/s 达不到 1 件もなかった。'
            f'速度経路が生きているかの対照が成立していない。')

        self.assertTrue(
            any(abs(t.linear.x) > _NONZERO_ATOL for t in muxed_window),
            f'SUMMON/POINT で /cmd_vel_muxed に非ゼロが出ない'
            f'（{len(muxed_window)} 件中 0）。jog_gate の除外表が'
            f'本来就通っている（= WAIT_CLEAR でも通る）疑いがある。')

    # ═══════════════════════════════════════════════════════════
    # test_4: WAIT_CLEAR 中はジョグが /cmd_vel_muxed・/cmd_vel に届かない
    # ═══════════════════════════════════════════════════════════
    def test_4_jog_blocked_in_wait_clear(self):
        """退避待ち中のジョグ遮断を明示的に確かめる（#7 の中核 gate）。

        ブリーフ #3 の変異（除外表を削除）に対して `test_3` の POINT 側
        非ゼロが緑のままになるよう、WAIT_CLEAR 側で 0 であることを確認する。
        """
        self._enter_wait_clear()

        # 記録を区切る。
        cmd_mark = len(self.cmd_rec)
        muxed_mark = len(self.muxed_rec)

        # WAIT_CLEAR 中（timeout 発火まで）、ジョグを流し続ける。
        deadline = time.monotonic() + _CLEAR_TIMEOUT_MS / 1000.0 + 1.0
        while time.monotonic() < deadline:
            self._send_jog()
            self._spin(0.05)
            if self._mode_state() == ('SUMMON', 'POINT'):
                break

        cmd_window = self.cmd_rec[cmd_mark:]
        muxed_window = self.muxed_rec[muxed_mark:]

        # 前提: muxed / cmd のどちらにも「記録がある」こと（購読が生きている）。
        # 0 件だと「何も来ていない」だけで判定不能になる。
        self.assertTrue(
            muxed_window,
            f'WAIT_CLEAR 中に /cmd_vel_muxed を 1 件も受信していない。'
            f'twist_mux が動いていない可能性。')
        self.assertTrue(
            cmd_window,
            f'WAIT_CLEAR 中に /cmd_vel を 1 件も受信していない。'
            f'obstacle_limiter が 20Hz で publish していない可能性。')

        self.assertFalse(
            any(abs(t.linear.x) > _NONZERO_ATOL for t in muxed_window),
            f'退避待ち中に /cmd_vel_muxed に非ゼロが出た'
            f'（{sum(1 for t in muxed_window if abs(t.linear.x)>_NONZERO_ATOL)} 件）。'
            f'jog_gate の除外表が効いていない。')

        self.assertFalse(
            any(abs(t.linear.x) > _NONZERO_ATOL for t in cmd_window),
            f'退避待ち中に /cmd_vel に非ゼロが出た'
            f'（{sum(1 for t in cmd_window if abs(t.linear.x)>_NONZERO_ATOL)} 件）。')


# ---- 小補助（f-string 内で使えるように） --------------------------------
def P_hold() -> int:
    """`clear_hold_ms`（registry）を int で返す（f-string 内で使う）。"""
    return int(_P['clear_hold_ms'])


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
