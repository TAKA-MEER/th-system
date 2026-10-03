"""
case_09_person_detection_off.py — 故障注入 9「人検知 OFF とフォルトの区別」
（ctest 登録名: fault_injection_09）
============================================================================
`DetailedDesign-safety.md` §5.5（P-01＋P-02 済み。`f240942`）の本体（P-03）。
Gazebo は起動せず、`launch_testing` で本番ノードだけを立てる
（お手本: `case_07_summon_retreat_wait.py`）。

立てるもの:
- `safety_monitor`（本番バイナリ。`safety_monitor_sim.yaml` を土台にし、
  上書きで `person_report_only: false`・`enabled_targets: ['person']`。
  他の監視対象は有効化しない）
- `state_manager`（本番バイナリ。`evt.link_ok` を 1 回入れて `IDLE` まで進める。
  `tracker_enabled` は本番の `/system/set_flag` で切り替える）
- `person_tracker_stub.py`（本番スタブ。テストプロセスが `subprocess` で起動し
  PID を保持する。T1b／T2b で SIGTERM する）

判定の値は `safety_monitor_sim.yaml`（`person_timeout_ms`／
`person_startup_grace_ms`）から読み、テストに直書きしない。`startup_grace_sec`
も同じファイルの値を使い、起動直後の保留が明けるまで待ってから始める。

順序は T1→T2（§5.5.7）。片方だけでは沈黙で通るため両方必須:
- T1a: OFF・stub 稼働のまま `person_timeout_ms`＋余裕 → active にならない・
  `IDLE` のまま。stub が実際に `/person/targets` を出していたことも assert。
- T1b: OFF のまま stub を SIGTERM → `person_timeout_ms`＋2s 待っても
  active にならない・`IDLE` のまま（ゲートの証明。ゲートが無ければ
  T1a 時点の受信履歴があるため必ず active になる）。
- T2a: stub 停止のまま `tracker_enabled=true` → 猶予中は active にならない →
  猶予＋`person_timeout_ms`＋許容以内に active になる。
- T2b: OFF に戻して解除 → stub を起動 → ON → 猶予＋余裕待ち（active にならない）
  → stub を SIGTERM → `person_timeout_ms`＋許容以内に active になる。
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from th_system_msgs.msg import (FaultStatus, PersonTargets, StateEvent,
                                SystemState)
from th_system_msgs.srv import SetFlag


# ===========================================================================
# 判定に使う値は safety_monitor_sim.yaml から読む。直書きしない。
# ===========================================================================
def _sim_yaml_path() -> str:
    """`th_bringup` の `config/safety_monitor_sim.yaml` を探す。

    コンテナ（`colcon test`）では `ament_index` の share を優先し、
    無ければソースツリーを見る（ホストでの import 時の読み取り用）。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('th_bringup'),
                            'config', 'safety_monitor_sim.yaml')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    here = os.path.abspath(os.path.dirname(__file__))
    cand = os.path.abspath(os.path.join(
        here, '..', '..', '..', 'th_bringup', 'config', 'safety_monitor_sim.yaml'))
    if os.path.exists(cand):
        return cand
    pytest.fail(f'safety_monitor_sim.yaml が見つからない（試した: {cand}）')


_SIM_YAML = _sim_yaml_path()
with open(_SIM_YAML, encoding='utf-8') as _f:
    _SIM_PARAMS = (yaml.safe_load(_f) or {}).get(
        'safety_monitor', {}).get('ros__parameters') or {}


def _sim_param(key: str):
    if key not in _SIM_PARAMS:
        pytest.fail(f'{_SIM_YAML} に safety_monitor.ros__parameters.{key} が無い')
    return _SIM_PARAMS[key]


_PERSON_TIMEOUT_MS = int(_sim_param('person_timeout_ms'))
_PERSON_GRACE_MS = int(_sim_param('person_startup_grace_ms'))
_STARTUP_GRACE_SEC = int(_sim_param('startup_grace_sec'))
_CHECK_PERIOD_MS = int(_SIM_PARAMS.get('check_period_ms', 100))

_PERSON_TIMEOUT_S = _PERSON_TIMEOUT_MS / 1000.0
_PERSON_GRACE_S = _PERSON_GRACE_MS / 1000.0

# T1 の待ち（余裕は「本当に来ない」の証明のためのもの。合否のしきい値ではない）。
_T1_MARGIN_S = 2.0
# T2 の検知許容（判定周期＋ scheduling のばらつき。500ms では負荷時に落ちるため
# 余裕を載せる。上限側の判定であり、検知が遅いほど通りにくくはならないが、
# 速い検知を逃さないようポーリングで待つ）。
_T2_ALLOWANCE_S = 3.0

# state_manager の起動パラメータ（テストハーネスの値。合否のしきい値ではない。
# `test_state_manager_node.py` と同じ流儀で直接注入する）。
_STATE_MGR_PARAMS = {
    'jog_lease_ms': 300,
    'link_wait_timeout_ms': 60_000,
    'ui_active_window_s': 5,
    'screen_stale_ms': 60_000,
}

_IDLE_WAIT_S = 60.0


@pytest.mark.launch_test
def generate_test_description():
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[
            _SIM_YAML,
            {'person_report_only': False,
             'enabled_targets': ['person']},
        ],
        output='screen',
    )
    state_mgr = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[_STATE_MGR_PARAMS],
        output='screen',
    )
    return launch.LaunchDescription([
        safety, state_mgr,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety, 'state_manager': state_mgr}


def _tl_qos() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


class TestPersonDetectionOff(unittest.TestCase):
    """故障注入 09 本体。メソッドは名前順に走るため T1→T2 の順に番号を付ける。"""

    # ── 起動 ──────────────────────────────────────────────
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('case_09_client')

        cls.faults: list[FaultStatus] = []
        cls.node.create_subscription(
            FaultStatus, '/safety/fault', cls.faults.append, 10)
        cls.states: list[SystemState] = []
        cls.node.create_subscription(
            SystemState, '/system/state', cls.states.append, _tl_qos())
        cls.targets: list[PersonTargets] = []
        cls.node.create_subscription(
            PersonTargets, '/person/targets', cls.targets.append, 10)

        cls.pub_event = cls.node.create_publisher(
            StateEvent, '/system/event', 10)
        cls.cli_flag = cls.node.create_client(SetFlag, '/system/set_flag')
        if not cls.cli_flag.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                '/system/set_flag が見つからない（state_manager が起動していない?）')

        cls._stub_proc: subprocess.Popen | None = None

        # state_manager を IDLE まで進める（case_07 と同じく evt.link_ok を 1 回。
        # connectivity_checker は立てない。IDLE 到達自体はここの assert が縛る）。
        ok = cls._wait_until(
            cls._mode_is('IDLE'), _IDLE_WAIT_S,
            publish_link_ok=True)
        if not ok:
            last = cls.states[-1].mode if cls.states else '受信なし'
            assert False, (
                f'state_manager が {_IDLE_WAIT_S}秒以内に IDLE へ到達しなかった '
                f'(直近の mode: {last!r})')

        # safety_monitor の起動保留（startup_grace_sec）が明けるまで待つ。
        # この間は checkTimeout が一切動かない（conftest の解説参照）。
        cls._spin(_STARTUP_GRACE_SEC + 1.0)

        # T1a の前提: stub を起動し、OFF のままにしておく。
        cls._start_stub()
        cls._set_flag(False)
        cls._spin(1.0)

    @classmethod
    def tearDownClass(cls):
        try:
            cls._stop_stub()
        finally:
            try:
                cls.node.destroy_node()
            finally:
                rclpy.shutdown()

    # ── 補助 ──────────────────────────────────────────────
    @classmethod
    def _spin(cls, sec: float):
        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def _wait_until(cls, pred, timeout: float,
                    publish_link_ok: bool = False) -> bool:
        deadline = time.monotonic() + timeout
        ev = StateEvent()
        ev.event = 'evt.link_ok'
        while time.monotonic() < deadline:
            if publish_link_ok:
                cls.pub_event.publish(ev)
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if pred():
                return True
        return False

    @classmethod
    def _mode_is(cls, mode: str) -> bool:
        return (lambda: bool(cls.states) and cls.states[-1].mode == mode)

    @staticmethod
    def _is_active(faults, fault_type: str) -> bool:
        """`/safety/fault` は変化時だけ出るので、最後の edge を見る。"""
        last = None
        for f in faults:
            if f.fault_type == fault_type and f.active:
                last = True
            elif not f.active and last:
                last = False
        return bool(last)

    @classmethod
    def _set_flag(cls, value: bool):
        req = SetFlag.Request()
        req.flag = 'tracker_enabled'
        req.value = bool(value)
        future = cls.cli_flag.call_async(req)
        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=10.0)
        res = future.result()
        assert res is not None and res.success, (
            f'/system/set_flag(tracker_enabled={value}) が失敗した: {res}')

    @classmethod
    def _start_stub(cls):
        if cls._stub_proc is not None and cls._stub_proc.poll() is None:
            return
        cls.targets.clear()
        cls._stub_proc = subprocess.Popen(
            ['ros2', 'run', 'th_perception', 'person_tracker_stub.py',
             '--ros-args',
             '-p', 'pattern:=walk_forward',
             '-p', 'initial_x:=1.5'],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True)
        # stub が実際に /person/targets を出し始めるまで待つ。
        ok = cls._wait_until(lambda: len(cls.targets) > 0, 15.0)
        assert ok, ('person_tracker_stub が 15秒以内に /person/targets を '
                    '1 件も出さなかった')

    @classmethod
    def _stop_stub(cls):
        proc = cls._stub_proc
        cls._stub_proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
        try:
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=10.0)

    # ── T1 ────────────────────────────────────────────────
    def test_1a_off_with_stub_stays_silent(self):
        """T1a: OFF・stub 稼働のまま timeout＋余裕 → active にならない・IDLE のまま。"""
        type(self)._set_flag(False)
        mark = len(type(self).faults)
        type(self)._spin(_PERSON_TIMEOUT_S + _T1_MARGIN_S)
        new = type(self).faults[mark:]
        assert not type(self)._is_active(new, 'PERSON_TRACKER_LOST'), (
            'tracker OFF なのに PERSON_TRACKER_LOST が active になった')
        assert type(self).states and type(self).states[-1].mode == 'IDLE', (
            'tracker OFF なのに IDLE から遷移した '
            f'(mode={type(self).states[-1].mode if type(self).states else "受信なし"!r})')
        assert len(type(self).targets) > 0, (
            'stub が /person/targets を 1 件も出していない（T1a の前提が崩れた）')

    def test_1b_off_without_stub_stays_silent(self):
        """T1b: OFF のまま stub を止めて timeout＋2s → それでも active にならない。"""
        type(self)._stop_stub()
        type(self)._set_flag(False)
        # 配信停止の瞬間を記録し、そこから timeout＋2s 待つ。
        stop = time.monotonic()
        type(self)._spin(0.2)
        wait = _PERSON_TIMEOUT_S + 2.0 - (time.monotonic() - stop)
        if wait > 0:
            type(self)._spin(wait)
        assert not type(self)._is_active(type(self).faults, 'PERSON_TRACKER_LOST'), (
            'tracker OFF・配信停止なのに PERSON_TRACKER_LOST が active になった '
            '（ゲートが効いていない）')
        assert type(self).states and type(self).states[-1].mode == 'IDLE', (
            'tracker OFF なのに IDLE から遷移した')

    # ── T2 ────────────────────────────────────────────────
    def test_2a_grace_then_fires(self):
        """T2a（猶予）: stub 停止のまま ON → 猶予中は出ず、猶予＋timeout＋許容以内に出る。"""
        assert (type(self)._stub_proc is None
                or type(self)._stub_proc.poll() is not None), (
            'T2a の前提が崩れた：stub が生きている')
        type(self)._set_flag(True)
        # 猶予の間は active にならない（猶予いっぱいは待たず、9 割で見る。
        # 残り 1 割は scheduling のぶれで early-fire と誤判定しないため）。
        type(self)._spin(_PERSON_GRACE_S * 0.9)
        assert not type(self)._is_active(type(self).faults, 'PERSON_TRACKER_LOST'), (
            'ON 直後の猶予中なのに PERSON_TRACKER_LOST が active になった')
        # 猶予明け＋timeout＋許容以内に出る。
        ok = type(self)._wait_until(
            lambda: type(self)._is_active(type(self).faults, 'PERSON_TRACKER_LOST'),
            (_PERSON_GRACE_S * 0.1) + _PERSON_TIMEOUT_S + _T2_ALLOWANCE_S)
        assert ok, (
            'tracker ON・配信なしなのに PERSON_TRACKER_LOST が '
            f'猶予({_PERSON_GRACE_S:.1f}s)＋timeout({_PERSON_TIMEOUT_S:.1f}s)'
            f'＋許容({_T2_ALLOWANCE_S:.1f}s)以内に active にならない')

    def test_2b_kill_fires(self):
        """T2b（kill）: 戻して（OFF→解除→stub起動→ON→猶予＋余裕待ち。
        この間出ない）→ stub を SIGTERM → timeout＋許容以内に出る。"""
        # 状態を戻す：OFF にしてフォルト解除を待つ。
        type(self)._set_flag(False)
        ok = type(self)._wait_until(
            lambda: not type(self)._is_active(
                type(self).faults, 'PERSON_TRACKER_LOST'), 10.0)
        assert ok, 'OFF にしても PERSON_TRACKER_LOST が解除されない'
        # stub を起動し、ON にして猶予＋余裕待つ。この間は出ない。
        type(self)._start_stub()
        type(self)._set_flag(True)
        type(self)._spin(_PERSON_GRACE_S + 1.0)
        assert not type(self)._is_active(type(self).faults, 'PERSON_TRACKER_LOST'), (
            'tracker ON・配信中なのに PERSON_TRACKER_LOST が active になった')
        # kill → timeout＋許容以内に出る。
        mark = len(type(self).targets)
        type(self)._stop_stub()
        kill_time = time.monotonic()
        ok = type(self)._wait_until(
            lambda: type(self)._is_active(type(self).faults, 'PERSON_TRACKER_LOST'),
            _PERSON_TIMEOUT_S + _T2_ALLOWANCE_S)
        assert ok, (
            'stub を kill しても PERSON_TRACKER_LOST が '
            f'timeout({_PERSON_TIMEOUT_S:.1f}s)＋許容({_T2_ALLOWANCE_S:.1f}s)'
            f'以内に active にならない（kill からの経過で判定）')
        assert time.monotonic() - kill_time <= _PERSON_TIMEOUT_S + _T2_ALLOWANCE_S


if __name__ == '__main__':
    unittest.main()
