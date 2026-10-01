"""
case_07_summon_retreat_wait.py — 故障注入 7「呼び寄せの退避待ち」
（ctest 登録名: fault_injection_07）
=====================================================================
`DetailedDesign-safety.md` §10 #7: 人が退かずに待っているとき発進せず、
タイムアウトで中止することを確認する。

- SM-3.1.2-067: `SUMMON/POINT` + `evt.two_point_done` → `SUMMON/WAIT_CLEAR`
- SM-3.1.2-068: `SUMMON/WAIT_CLEAR` + `evt.clear_ok` → `SUMMON/NAV`
- SM-3.1.2-069: `SUMMON/WAIT_CLEAR` + `evt.clear_timeout` → `SUMMON/POINT`
- `DetailedDesign-onsite.md` §9 #5・#6: ゴールから `clear_distance_m` 以上離れて
  `clear_hold_ms` 続けば発進、`clear_timeout_ms` まで離れなければ発進せずに `POINT` へ。

===============================================================================
【Gazebo を使わない理由】
===============================================================================
`gazebo.launch.py` の `safety_enabled` に `wait_clear_gate` も `jog_gate` も
含まれず、`venue_navigator`（SUMMON/NAV の実際の走行主体）も無い。Gazebo を
立てると「退避待ち」状態を作れず、`/person/status` の供給元も無い。よって
**本番ノードを launch_testing で直接起動し、外側の縁だけを与える**。

===============================================================================
【外から入れてよいもの（これ以外は入れない）】
===============================================================================
- `/system/event` に `evt.link_ok` を 1 回 … **試験対象ではない起動の前提**。
  `INIT/CHECK → IDLE` は `evt.link_ok` だけで進む（`transitions.yaml` T-INIT-02）。
- `/tf_static` に `map → base_link` の恒等変換 … `pin_registrar` と
  `wait_clear_gate` がこれで人の座標を map に変換する。
- `/person/status` … `base_link` 相対・`is_lost=False`・`confidence` ≥ 0.5。
  押下ごとに位置を変える。
- `/system/trigger`（`ui.goto` / `ui.finish` / `ui.abort`）、`/onsite/two_point`
  （`TwoPointPress`）、`/cmd_vel_manual_raw`（`Twist`） … 画面が叩くのと同じ経路。

**入れないもの（入れると core を見なくなる）**:
- `/system/state` … `state_manager` の**出力**。代役を立てると FSM 検証が無意味になる。
- `/system/event` の `evt.two_point_done` / `evt.clear_ok` / `evt.clear_timeout`
  … ゲートと FSM がそれぞれ本来出すべきもの。直接出すと検証にならない。
- `/onsite/summon_goal` … `pin_registrar` の出力。

===============================================================================
【速度の観測点】
===============================================================================
- **退避待ちで発進しない** → `/system/state` が `NAV` に入らないこと
  （入らない限り走る挙動ノードは指令を出さない）。
- **ジョグが効かない** → `jog_gate` の出力 `/cmd_vel_manual` に非ゼロが出ないこと。

`/cmd_vel_muxed`（twist_mux の出力）も併せて見る。これは priority を_shift するだけで
前提条件を持たないので、「jog_gate が出していない」ことを独立に裏づける。

`/cmd_vel`（`obstacle_limiter` の出力）は**判定に使わない**。このスタックには
LiDAR が居ないため limiter は tier3（`/scan` 途絶）で無条件に停止し、退避待ち
ゲートと無関係な理由で 0 になる。0 icipated を「発進しなかった証拠」に使うのは
嘘になるので使わない（ノードは速度連鎖の文脈として起動はさせている）。
"""

from __future__ import annotations

import os
import shutil
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml

from geometry_msgs.msg import TransformStamped, Twist
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import PersonStatus, StateEvent, SystemState
from th_system_msgs.srv import TwoPointPress, UiTrigger


# ===========================================================================
# T-1: 判定に使う値は生成 YAML（registry.yaml 由来）から読む。直書きしない。
# ===========================================================================
_TH_TESTING_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
_WS_ROOT = os.path.abspath(os.path.join(_TH_TESTING_ROOT, '..'))


def _find_generated_dir() -> str:
    """`params_generation.GENERATED_DIR`（`/root/th_data/generated`）を探す。

    実体と観測点が食い違うので両方を見る:
    - コンテナ（`colcon test` の実行環境）: docker-compose.yml が
      `./data:/root/th_data` を bind mount しているので `/root/th_data/generated`
    - ホスト（素の `python3 -m pytest`）: 作業ツリー直下の `th_ws/data/generated`

    どちらも無ければ fail する。値をテスト内に直書きして逃げることはしない。
    """
    candidates = [
        os.environ.get('TH_GENERATED_DIR', ''),
        '/root/th_data/generated',
        os.path.join(_WS_ROOT, 'data', 'generated'),
    ]
    for path in candidates:
        if path and os.path.isdir(path):
            return path
    pytest.fail(
        '生成パラメータ置き場が見つからない（試した: '
        + ', '.join(c or '(env 未設定)' for c in candidates)
        + '）。judge/judge 変換が走っていない。'
        '`ros2 launch th_bringup bringup.launch.py params_only:=true` で生成される。')


_GENERATED = _find_generated_dir()


def _generated_param(yaml_name: str, node_name: str, key: str):
    """`data/generated/<yaml_name>` から `<node_name>.ros__parameters.<key>` を読む。

    生成 YAML は `params_generation` が registry.yaml から毎ビルド作る
    **正本**（CLAUDE.md「実際に効く設定は data/generated 側」）。判定値と
    launch に渡す値がこの 1 ファイルから出るので、試験とノードは自己整合する。
    """
    path = os.path.join(_GENERATED, yaml_name)
    if not os.path.exists(path):
        pytest.fail(
            f'生成パラメータ {path} が無い。judge/judge 変換が走っていない'
            f'（`ros2 launch th_bringup bringup.launch.py params_only:=true` で生成される）。'
            f'テストに値を直書きして逃げることはしない。')
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    params = (data.get(node_name) or {}).get('ros__parameters') or {}
    if key not in params:
        pytest.fail(f'{path} に {node_name}.ros__parameters.{key} が無い。'
                    f'registry.yaml の該当行を確認すること。')
    return params[key]


# ── wait_clear_gate（`clear_distance_m` / `clear_hold_ms` はここから） ──────────
_CLEAR_DISTANCE_M = float(_generated_param(
    'wait_clear_gate.yaml', 'wait_clear_gate', 'clear_distance_m'))
_CLEAR_HOLD_MS = int(_generated_param(
    'wait_clear_gate.yaml', 'wait_clear_gate', 'clear_hold_ms'))
_TICK_HZ = int(_generated_param(
    'wait_clear_gate.yaml', 'wait_clear_gate', 'tick_hz'))

# ── pin_registrar（2 点間隔・信頼度。P2 を置く位置の妥当性確認に使う） ──────────
_TWO_POINT_MIN_SPACING_M = float(_generated_param(
    'pin_registrar.yaml', 'pin_registrar', 'two_point_min_spacing_m'))
_MIN_CONFIDENCE = float(_generated_param(
    'pin_registrar.yaml', 'pin_registrar', 'min_confidence'))

# ── state_manager（起動の前提として本番と同じ値を渡す） ─────────────────────
_STATE_MGR_PARAMS = {
    k: _generated_param('state_manager.yaml', 'state_manager', k)
    for k in ('jog_lease_ms', 'link_wait_timeout_ms', 'ui_active_window_s',
              'screen_stale_ms', 'target_confidence_min')
}
_JOG_GATE_STATE_STALE_MS = int(_generated_param(
    'jog_gate.yaml', 'jog_gate', 'state_stale_ms'))


# ===========================================================================
# 試験時間の短縮（ブリーフが明示的に許可している launch 上書き）
# ===========================================================================
# registry の `clear_timeout_ms` は 30000ms（30 秒）。本項目の要件は
# 「タイムアウトで POINT に戻ること」であって 30 秒という長さではないため、
# `wait_clear_gate` に渡す値だけ縮める。**期待時間は必ずこの定数から導く**
# （`_TIMEOUT_SEC` / `_NOT_EARLY_FRACTION`）。
# 距離と保持時間（`_CLEAR_DISTANCE_M` / `_CLEAR_HOLD_MS`）は縮めない——
# 縮めると「退いた/退いていない」の判定そのものの意味が変わる。
_CLEAR_TIMEOUT_MS = 6000
_TIMEOUT_SEC = _CLEAR_TIMEOUT_MS / 1000.0
# 「发起得太快也是错的」: wait_clear_gate は `clear_timeout_ms` を満たすまで
# timeout を出さない。80% 未満で POINT に戻った場合は、timeout ではなく
# 別の事象（evt.target_lost 等）で落ちた疑いがあるので赤にする。
_NOT_EARLY_FRACTION = 0.8
_NOT_EARLY_SEC = _TIMEOUT_SEC * _NOT_EARLY_FRACTION
# timeout 判定の 20Hz tick + `/system/state` の 100ms publish 周期を吸収する余裕。
_TIMEOUT_MARGIN_SEC = 3.0


# ===========================================================================
# 人物の座標（base_link 相対。map ← base_link は恒等なのでそのまま map 座標）
# ===========================================================================
_P1 = (1.0, 0.0)    # ゴール（2 点指示の 1 点目）
_P2 = (1.0, 0.5)    # 2 点目（向きを決める点）
_FAR = (3.0, 0.0)   # 退避到位（陽性対照）

# 人物の确信度。`min_confidence`（0.5）以上であることが `pin_registrar` の受理条件。
_PERSON_CONFIDENCE = max(0.9, _MIN_CONFIDENCE)


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


# ── 配置の妥当性を import 時に検査する（値が動いたら静かに壊れないように） ────
# P1→P2 は `two_point_min_spacing_m` 以上（2 点指示が通る）。
# P1→P2 は `clear_distance_m` 未満（= 退かないケース）。
# P1→FAR は `clear_distance_m` 以上（= 退いたケース）。
assert _dist(_P1, _P2) >= _TWO_POINT_MIN_SPACING_M, (
    f'P1={_P1} と P2={_P2} の間隔 {_dist(_P1, _P2):.3f}m が '
    f'two_point_min_spacing_m={_TWO_POINT_MIN_SPACING_M} 未満。'
    f'2 点指示が two_point_too_close で拒否される。')
assert _dist(_P1, _P2) < _CLEAR_DISTANCE_M, (
    f'P1={_P1} と P2={_P2} の間隔 {_dist(_P1, _P2):.3f}m が '
    f'clear_distance_m={_CLEAR_DISTANCE_M} 以上。「退かない」ケースを'
    f'再現できていない（clear_ok が出てしまう）。')
assert _dist(_P1, _FAR) >= _CLEAR_DISTANCE_M, (
    f'P1={_P1} と FAR={_FAR} の間隔 {_dist(_P1, _FAR):.3f}m が '
    f'clear_distance_m={_CLEAR_DISTANCE_M} 未満。「退いた」ケースを'
    f'再現できていない（clear_ok が出ない）。')


# ── UI/2 点指示のサービス呼び出しに与える retry 上限 ─────────────────────────
# 同じコンテナ内で 14 本の故障注入を連続実行すると、前後の Gazebo 起動/終了と
# DDS discovery の収束が遅れ、`wait_for_service` は通るのに応答が届かない、
# あるいは `no_pending`（`begin_two_point` の effect が未着）で拒否される
# ことがある。単体実行では出ないが、全件を 1 本の `colcon test` で回す運用では
# 確実に出る。押せる状態になるまで押し直すのは人が画面を操作する時の挙動と
# 同じなので、その分を retry で吸収する。
_SERVICE_RETRY_BUDGET_SEC = 30.0
# retry してよい拒否理由。「押せる状態になっていない/周縁が未整」の系だけ。
# `two_point_too_close` は本試験の前提（P1/P2 の間隔）についての主張なので
# 入らない。
_RETRYABLE_REJECT_KEYS = frozenset({
    'no_pending', 'no_map_tf', 'target_lost', 'low_confidence',
})


# ===========================================================================
# 「ゼロ」と「非ゼロ」を切り分けるしきい値
# ===========================================================================
# `_JOG_LINEAR_MPS` は「0 より大きい・しかし小さい」値を意図的に選ぶ。
# `jog_gate` は値の大きさではなく `attributes.yaml` の jog 列と除外表だけで
# 通すか決めるので、0.1 で十分。同時にゼロ判定の atol より十分大きいこと。
_JOG_LINEAR_MPS = 0.1
_NONZERO_ATOL = 1e-6


def _attributes_yaml_path() -> str:
    """`th_state` share の attributes.yaml（`jog_gate` が読む J-4 のファイル）。

    `test_jog_gate_node.py::_real_attributes_path()` と同じ解決。取れなければ
    空文字を返し、`jog_gate` 側の既定（ament_index 解決）に任せる。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('th_state'),
                            'config', 'attributes.yaml')
    except Exception:
        return ''


# ===========================================================================
# launch: 本番ノードをそのものを起動する
# ===========================================================================
# `venue_dir` は SUMMON の経路では pin を永続化しない（WS-9AF: `_place_pin_effect`
# を経由しない）ので書き込みは起きないが、sure に一時ディレクトリを渡す。
# コンテナ内なので `/tmp` で構わない。
_VENUE_DIR = tempfile.mkdtemp(prefix='fi07_venue_')


@pytest.mark.launch_test
def generate_test_description():
    jog_params = [{'state_stale_ms': _JOG_GATE_STATE_STALE_MS}]
    attrs = _attributes_yaml_path()
    if attrs:
        jog_params.append({'attributes_yaml_path': attrs})

    state_mgr = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[_STATE_MGR_PARAMS],
        output='screen',
    )

    registrar = launch_ros.actions.Node(
        package='th_onsite',
        executable='pin_registrar.py',
        name='pin_registrar',
        parameters=[{
            'venue_dir': _VENUE_DIR,
            'two_point_min_spacing_m': _TWO_POINT_MIN_SPACING_M,
            'min_confidence': _MIN_CONFIDENCE,
        }],
        output='screen',
    )

    wait_clear = launch_ros.actions.Node(
        package='th_onsite',
        executable='wait_clear_gate.py',
        name='wait_clear_gate',
        parameters=[{
            'clear_distance_m': _CLEAR_DISTANCE_M,
            'clear_hold_ms': _CLEAR_HOLD_MS,
            # ★ registry(30000) から縮める。理由はこの定数の定義部。
            'clear_timeout_ms': _CLEAR_TIMEOUT_MS,
            'tick_hz': _TICK_HZ,
        }],
        output='screen',
    )

    jog_gate = launch_ros.actions.Node(
        package='th_safety',
        executable='jog_gate',
        name='jog_gate',
        parameters=jog_params,
        output='screen',
    )

    return launch.LaunchDescription([
        state_mgr, registrar, wait_clear, jog_gate,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_mgr, 'pin_registrar': registrar,
         'wait_clear_gate': wait_clear, 'jog_gate': jog_gate}


# ===========================================================================
def _tl_qos() -> QoSProfile:
    """`/system/state` と同じ TRANSIENT_LOCAL RELIABLE。

    plain int（既定 VOLATILE）で購読すると durability 不一致で 1 通も来ない
    （QoS incompatible。例外は出ず、WARN だけがノード側に出る）。
    """
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


class TestSummonRetreatWait(unittest.TestCase):
    """故障注入 #7「退避待ち」。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        shutil.rmtree(_VENUE_DIR, ignore_errors=True)

    def setUp(self):
        self.node = rclpy.create_node('case_07_client')

        # ── 観測 ──────────────────────────────────────────
        self.state_rec: list[SystemState] = []
        self.node.create_subscription(
            SystemState, '/system/state', self.state_rec.append, _tl_qos())
        self.events: list[StateEvent] = []
        self.node.create_subscription(
            StateEvent, '/system/event', self.events.append, 10)
        # jog_gate の出力。depth 1 の reliable（jog_gate の publisher と同じ）。
        self.cmd_manual_rec: list[Twist] = []
        self.node.create_subscription(
            Twist, '/cmd_vel_manual', self.cmd_manual_rec.append, 10)

        # ── 供給（外側の縁だけ） ───────────────────────────
        self.pub_event = self.node.create_publisher(StateEvent, '/system/event', 10)
        self.pub_person = self.node.create_publisher(PersonStatus, '/person/status', 10)
        self.pub_raw = self.node.create_publisher(Twist, '/cmd_vel_manual_raw', 10)
        self.pub_tf = self.node.create_publisher(TFMessage, '/tf_static', _tl_qos())

        tf = TFMessage()
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        tf.transforms.append(t)
        self.pub_tf.publish(tf)

        # 人物の位置。手順で切り替える。
        self._person_xy = _P1
        self._person_timer = self.node.create_timer(0.05, self._publish_person)

    def tearDown(self):
        try:
            self.node.destroy_timer(self._person_timer)
        except Exception:
            pass
        self.node.destroy_node()

    # ── 供給 ──────────────────────────────────────────────
    def _publish_person(self):
        p = PersonStatus()
        p.header.stamp = self.node.get_clock().now().to_msg()
        p.header.frame_id = 'base_link'
        p.position.x, p.position.y = self._person_xy
        p.position.z = 0.0
        p.confidence = _PERSON_CONFIDENCE
        p.is_lost = False
        p.lost_reason = ''
        self.pub_person.publish(p)

    def _set_person(self, xy: tuple[float, float], settle: float = 0.5):
        """人物を `xy` に移し、`pin_registrar` / `wait_clear_gate` が
        新しい `/person/status` を保持するまで少し回す。"""
        self._person_xy = xy
        self._spin(settle)

    def _publish_jog(self):
        """`/cmd_vel_manual_raw` に前進のジョグを流す。

        画面が送るのと同じ入力口（`ui.jog.hold` は SUMMON では
        `prep_states` なしで PAUSE になるため使わない）。
        """
        t = Twist()
        t.linear.x = _JOG_LINEAR_MPS
        self.pub_raw.publish(t)

    def _jog_window(self, sec: float) -> list[Twist]:
        """`sec` 秒だけジョグ入力を流し、その間の `/cmd_vel_manual` を受け取る。

        遮断側では 0 通、陽性対照側では非ゼロが何通か返ることを期待する。
        """
        mark = len(self.cmd_manual_rec)
        timer = self.node.create_timer(0.05, self._publish_jog)
        try:
            self._spin(sec)
        finally:
            self.node.destroy_timer(timer)
        return self.cmd_manual_rec[mark:]

    # ── 補助 ──────────────────────────────────────────────
    def _spin(self, sec: float = 0.3):
        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _mode_state(self) -> tuple[str, str] | None:
        if not self.state_rec:
            return None
        s = self.state_rec[-1]
        return (s.mode, s.state)

    def _wait_mode_state(self, mode: str, state: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while True:
            if self._mode_state() == (mode, state):
                return True
            if time.monotonic() >= deadline:
                return self._mode_state() == (mode, state)
            self._spin(0.05)

    def _has_event(self, name: str) -> bool:
        return any(e.event == name for e in self.events)

    def _count_event(self, name: str) -> int:
        return sum(1 for e in self.events if e.event == name)

    def _call_trigger(self, trigger: str, arg_json: str = '{}'):
        """`/system/trigger` を 1 回呼ぶ。応答が無ければ retry する。

        `wait_for_service` が通っても応答が届かないことがある。DDS の
        discovery が前の故障注入（Gazebo を使うもの）の後拖着
        1〜2 秒遅れるためで、`fault_injection_07` 単体の実行では出ない。
        同じコンテナ内で 14 本連続して回すと必ず顔を出すので、
        「押せる状態になるまで押し直す」に相当する retry を入れる。
        """
        deadline = time.monotonic() + _SERVICE_RETRY_BUDGET_SEC
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            cli = self.node.create_client(UiTrigger, '/system/trigger')
            try:
                if cli.wait_for_service(timeout_sec=2.0):
                    req = UiTrigger.Request()
                    req.trigger = trigger
                    req.arg_json = arg_json
                    req.requester = 'case_07'
                    fut = cli.call_async(req)
                    rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
                    res = fut.result()
                    if res is not None:
                        return res, attempt
            finally:
                self.node.destroy_client(cli)
            self._spin(0.5)
        self.fail(f'/system/trigger({trigger}) が '
                  f'{_SERVICE_RETRY_BUDGET_SEC:.0f}s 以内に応答しなかった'
                  f'（{attempt} 回試行）')

    def _call_two_point(self, index: int):
        """`/onsite/two_point` を 1 回押し、受理されるまで retry する。

        retry するのは **「押せる状態になっていない」系**だけ:
        - 応答自体が来ない（discovery 未収束）
        - `no_pending`（`begin_two_point` の effect がまだ届いていない）
        - `no_map_tf` / `target_lost` / `low_confidence`（周縁がまだ揃っていない）

        `two_point_too_close` は retry しない。これは本試験の前提
        （P1/P2 の間隔）についての主張であり、人物の位置を変えて
        押し直して通すのは意味を潰す。
        """
        deadline = time.monotonic() + _SERVICE_RETRY_BUDGET_SEC
        attempt = 0
        last_reason = '応答なし'
        while time.monotonic() < deadline:
            attempt += 1
            cli = self.node.create_client(TwoPointPress, '/onsite/two_point')
            try:
                if cli.wait_for_service(timeout_sec=2.0):
                    req = TwoPointPress.Request()
                    req.purpose = 'SUMMON'
                    req.index = index
                    fut = cli.call_async(req)
                    rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
                    res = fut.result()
                    if res is not None:
                        last_reason = res.reject_reason_key
                        if res.accepted or last_reason not in _RETRYABLE_REJECT_KEYS:
                            return res, attempt
            finally:
                self.node.destroy_client(cli)
            self._spin(0.5)
        self.fail(f'/onsite/two_point(index={index}) が '
                  f'{_SERVICE_RETRY_BUDGET_SEC:.0f}s 以に受理されなかった'
                  f'（{attempt} 回試行、最終 reject={last_reason!r}）')

    # ── 手順 1〜4（起動 → IDLE → SUMMON/POINT → 2 点指示 → WAIT_CLEAR） ──
    def _enter_wait_clear(self) -> float:
        """手順 1〜4 を実行し、`WAIT_CLEAR` に入った時刻を返す。

        1. `evt.link_ok` → `IDLE`
        2. `ui.goto {"kind":"SUMMON"}` → `SUMMON/POINT`
           （`C-15-summon` が `begin_two_point{kind:SUMMON}` を出す）
        3. 人物を P1 にして `/onsite/two_point` index=1
        4. 人物を P2 にして index=2 → `evt.two_point_done` → `SUMMON/WAIT_CLEAR`

        既に IDLE/NONE でなければ `ui.finish`（C-08）で IDLE に戻してから始める。
        前の test が SUMMON/NAV や SUMMON/POINT に居るため毎回必要。
        """
        # 0. 前回のはじまりをリセット。
        if self._mode_state() != ('IDLE', 'NONE'):
            res, _n = self._call_trigger('ui.finish')
            if not res.accepted:
                self.fail(f'ui.finish が拒否（reject={res.reject_reason_key!r}）')
            if not self._wait_mode_state('IDLE', 'NONE', timeout=5.0):
                self.fail(f'ui.finish 後に IDLE/NONE にならない（{self._mode_state()}）')

        # 1. 起動の前提: evt.link_ok。INIT/CHECK → IDLE はこれで進む。
        if self._mode_state() is None:
            self._spin(0.5)
        if self._mode_state() != ('IDLE', 'NONE'):
            ev = StateEvent()
            ev.header.stamp = self.node.get_clock().now().to_msg()
            ev.event = 'evt.link_ok'
            ev.source_node = 'case_07_test'
            ev.arg_json = '{}'
            self.pub_event.publish(ev)
            if not self._wait_mode_state('IDLE', 'NONE', timeout=5.0):
                self.fail(f'evt.link_ok を出して IDLE にならない（{self._mode_state()}）')

        # 2. SUMMON に入る。
        res, _n = self._call_trigger('ui.goto', '{"kind":"SUMMON"}')
        if not res.accepted:
            self.fail(f'ui.goto(SUMMON) が拒否（reject={res.reject_reason_key!r}）')
        if not self._wait_mode_state('SUMMON', 'POINT', timeout=5.0):
            self.fail(f'ui.goto(SUMMON) 後に SUMMON/POINT にならない'
                      f'（{self._mode_state()}）')

        # 3. P1 を 1 点目として押す。
        self._set_person(_P1)
        r1, n1 = self._call_two_point(1)
        if not r1.accepted:
            self.fail(f'2 点指示 index=1 が拒否（reject={r1.reject_reason_key!r}、'
                      f'{n1} 回試行）')

        # 4. P2 を 2 点目として押す → evt.two_point_done → WAIT_CLEAR。
        self._set_person(_P2)
        r2, n2 = self._call_two_point(2)
        if not r2.accepted:
            self.fail(f'2 点指示 index=2 が拒否（reject={r2.reject_reason_key!r}、'
                      f'{n2} 回試行）')

        t_wait_clear = time.monotonic()
        if not self._wait_mode_state('SUMMON', 'WAIT_CLEAR', timeout=5.0):
            self.fail(f'2 点指示後に SUMMON/WAIT_CLEAR にならない（{self._mode_state()}）')
        return t_wait_clear

    # ═══════════════════════════════════════════════════════════════════
    # 手順 5: 退かないケース — 発進せず、timeout で POINT へ戻る
    # ═══════════════════════════════════════════════════════════════════
    def test_1_person_does_not_retreat_blocks_start_and_times_out_to_point(self):
        """`DetailedDesign-safety.md` §10 #7 本命。

        ゴール（P1）から `clear_distance_m`（0.975 m）未満の距離にいる人を
        P2（0.5 m）に置いたままにしたとき、

        - `SUMMON/WAIT_CLEAR` の間 `/system/state` が `NAV` にならない（＝発進しない）
        - `clear_timeout_ms`（launch 上書きの `_CLEAR_TIMEOUT_MS`）＋余裕 以内に
          `evt.clear_timeout` → `SUMMON/POINT` に戻る
        - ただし上書き値の 80% より前には戻らない（=`clear_ok` などで早期に
          抜けていない）
        """
        t_wait_clear = self._enter_wait_clear()

        # 「退かない」: 人物を P2 のまま据え置く。
        self._set_person(_P2, settle=0.2)

        self.assertTrue(
            self._has_event('evt.two_point_done'),
            'evt.two_point_done が出ていない（pin_registrar が 2 点指示を完了していない）。')
        self.assertEqual(
            self._count_event('evt.clear_ok'), 0,
            '人物がまだ退いていないのに evt.clear_ok が出た'
            '（wait_clear_core の距離判定が壊れている）。')

        # WAIT_CLEAR の間を観測し、NAV に入らないことを確かめる。
        deadline = t_wait_clear + _TIMEOUT_SEC + _TIMEOUT_MARGIN_SEC
        t_point = None
        saw_nav = False
        while time.monotonic() < deadline:
            self._spin(0.05)
            ms = self._mode_state()
            if ms == ('SUMMON', 'NAV'):
                saw_nav = True
            if ms == ('SUMMON', 'POINT'):
                t_point = time.monotonic()
                break

        self.assertFalse(
            saw_nav,
            '人が退かずにいるのに SUMMON/NAV に入った（＝発進した）。'
            'これは #7 の核心的な安全性違反。')

        self.assertIsNotNone(
            t_point,
            f'clear_timeout_ms（{_CLEAR_TIMEOUT_MS}ms＝{_TIMEOUT_SEC}s）＋余裕'
            f'{_TIMEOUT_MARGIN_SEC}s 経っても SUMMON/POINT に戻らない'
            f'（現状={self._mode_state()}）。evt.clear_timeout が出ていない。')

        elapsed = t_point - t_wait_clear
        self.assertGreaterEqual(
            elapsed, _NOT_EARLY_SEC,
            f'WAIT_CLEAR から POINT への戻りが {elapsed:.2f}s で短すぎる'
            f'（下限 {_NOT_EARLY_SEC:.2f}s ＝ clear_timeout_ms の '
            f'{int(_NOT_EARLY_FRACTION * 100)}%）。clear_timeout 以外の経路で'
            f'抜けており、本項目の「タイムアウトで中止」を検証できていない。')

        self.assertTrue(
            self._has_event('evt.clear_timeout'),
            'evt.clear_timeout が /system/event に出ていない'
            '（wait_clear_gate が出していない、または state_manager が処理していない）。')

    # ═══════════════════════════════════════════════════════════════════
    # 手順 6: 退避待ち中のジョグは無効（陽性対照つき）
    # ═══════════════════════════════════════════════════════════════════
    def test_2_jog_is_ignored_while_waiting_and_works_at_point(self):
        """`DetailedDesign-onsite.md` §9 受け入れ条件 #6。

        人が退かず `SUMMON/WAIT_CLEAR` のあいだ、ジョグは効かない。
        `attributes.yaml` は SUMMON の jog を「許可」にしている（POINT と
        NAV では走れる）が、`jog_gate_core` は `SUMMON/WAIT_CLEAR` を除外表に
        入れており、そこでは `/cmd_vel_manual` を一切出さない。
        """
        self._enter_wait_clear()
        self._set_person(_P2, settle=0.2)

        # ── 遮断側: WAIT_CLEAR 中にジョグ入力 ──────────────
        blocked_window = self._jog_window(2.0)

        passed = [t for t in blocked_window
                  if abs(t.linear.x) > _NONZERO_ATOL or abs(t.linear.y) > _NONZERO_ATOL]
        self.assertEqual(
            len(passed), 0,
            f'SUMMON/WAIT_CLEAR 中に /cmd_vel_manual へ {len(passed)} 通の非ゼロが'
            f'出た（{len(blocked_window)} 通中）。ジョグが未被遮断で通過している。'
            f'jog_gate_core の除外表から SUMMON/WAIT_CLEAR が消えている疑い。')

        # ── 陽性対照: WAIT_CLEAR を抜けて POINT に戻し、同じ入力が通る ──
        res, _n = self._call_trigger('ui.abort')
        if not res.accepted:
            self.fail(f'ui.abort が拒否（reject={res.reject_reason_key!r}）')
        if not self._wait_mode_state('SUMMON', 'POINT', timeout=5.0):
            self.fail(f'ui.abort 後に SUMMON/POINT に戻らない（{self._mode_state()}）')

        pass_window = self._jog_window(2.0)
        moved = [t for t in pass_window if abs(t.linear.x) > _NONZERO_ATOL]
        self.assertTrue(
            moved,
            f'SUMMON/POINT（ジョグ許可状態）で同じ入力が /cmd_vel_manual に'
            f'出なかった（{len(pass_window)} 通受信）。'
            f'遮断の赤が「jog_gate が単に黙っている」ためではなく、'
            f'配線が生きていることを示せない。')

    # ═══════════════════════════════════════════════════════════════════
    # 手順 7: 退避側の陽性対照 — 退けば発進する
    # ═══════════════════════════════════════════════════════════════════
    def test_3_person_retreats_and_robot_starts(self):
        """`DetailedDesign-onsite.md` §9 #5 の陽性対照。

        手順 1〜4 で入れた `WAIT_CLEAR` から、人物をゴール（P1）から
        `clear_distance_m` 以上離れた位置（FAR）に動かすと、`clear_hold_ms`
        継続後に `evt.clear_ok` が出て `SUMMON/NAV` へ進む。

        これが付かないと、手順 5 の「発進しない」が
        「`WAIT_CLEAR` に入ること自体が壊れていて、常に発進しないだけ」
        という無意味な検証になってしまう。対比して初めて
        「退かないときだけ止まる」ことが言える。
        """
        self._enter_wait_clear()

        # 「退避した」ケース: FAR は P1 から clear_distance_m 以上。
        self._set_person(_FAR, settle=0.3)

        entered_nav = self._wait_mode_state('SUMMON', 'NAV', timeout=10.0)
        self.assertTrue(
            entered_nav,
            f'人物がゴールから {_CLEAR_DISTANCE_M} m 以上退いても '
            f'SUMMON/NAV に入らない（現状={self._mode_state()}）。'
            f'evt.clear_ok = {self._count_event("evt.clear_ok")} 件。'
            f'退避を検知できないと、人は永久に待たされ続ける。')

        self.assertGreaterEqual(
            self._count_event('evt.clear_ok'), 1,
            'SUMMON/NAV に入ったのに evt.clear_ok が出ていない'
            '（遷移の根拠になっていない）。')

        self.assertEqual(
            self._count_event('evt.clear_timeout'), 0,
            '人物が退いたのに evt.clear_timeout まで出てしまった'
            '（clear_ok と同時発火している、または保持判定が壊れている）。')


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
