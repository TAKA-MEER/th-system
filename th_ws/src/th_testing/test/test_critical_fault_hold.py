"""WS-9O: 重大フォルトの発火に保持時間が効いていることを固定する。

実機フィードバック（2026-09-04）「一瞬でもエラーが出ると停止しそこから復帰できない」
の前半への対処。safety_monitor の監視ループ（check_period_ms=100）自身が一時的に
遅れると、複数の入力が同じ判定周期で同時にタイムアウト超過に見える。実機ログで
ESP32_DISCONNECTED と LIMITER_DEAD が同じ 1ms に発火し 26ms 後に両方解除された。

フォルトは edge で publish されるので、その 26ms でも active=true は state_manager に
届き、C-06a（ガード無し）で必ず ESTOP に落ちる。だから「すぐ消えたから無害」には
ならない。条件が critical_fault_hold_ms 継続したときだけ報告する必要がある。

safety_monitor は C++ ノードで、この挙動は実行時にしか現れない（launch テストは
Docker が要る）。ここでは配線をソース上で固定し、退行を pytest で捕まえる。
詳細は VISION.md §2 の 2026-09-04 の項。
"""
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..', '..'))
_CPP = os.path.join(_ROOT, 'th_ws', 'src', 'th_safety', 'src', 'safety_monitor.cpp')
_REGISTRY = os.path.join(
    _ROOT, 'th_ws', 'src', 'th_params', 'config', 'registry.yaml')

# 保持時間を課す重大フォルト。回復可能フォルト（LIDAR_LOST / ESP32_DISCONNECTED）は
# 一時停止から正常に再開できるので対象外＝素の checkTimeout のままにする。
_HELD_CRITICAL = ('LIMITER_DEAD', 'MUX_DEAD', 'STATE_INCONSISTENT')
_NOT_HELD = ('LIDAR_LOST', 'ESP32_DISCONNECTED')


def _src() -> str:
    with open(_CPP, encoding='utf-8') as f:
        return f.read()


def _update_fault_arg(src: str, fault_type: str) -> str:
    """updateFaultState("<fault_type>", <ここ>) の第 2 引数のテキストを返す。"""
    needle = f'updateFaultState("{fault_type}"'
    idx = src.index(needle)
    depth = 0
    start = src.index('(', idx)
    for i in range(start, len(src)):
        if src[i] == '(':
            depth += 1
        elif src[i] == ')':
            depth -= 1
            if depth == 0:
                inner = src[start + 1:i]
                return inner.split(',', 1)[1] if ',' in inner else ''
    raise AssertionError(f'updateFaultState("{fault_type}", ...) の括弧が閉じていない')


def test_critical_fault_hold_ms_is_declared():
    src = _src()
    assert 'declare_parameter("critical_fault_hold_ms"' in src, (
        'safety_monitor.cpp が critical_fault_hold_ms を宣言していない（WS-9O）')
    assert 'get_parameter("critical_fault_hold_ms")' in src, (
        'critical_fault_hold_ms を宣言しているだけで読んでいない（WS-9O）')


def test_critical_fault_hold_ms_is_in_registry():
    with open(_REGISTRY, encoding='utf-8') as f:
        reg = f.read()
    assert re.search(r'^- name: critical_fault_hold_ms$', reg, re.M), (
        'registry.yaml に critical_fault_hold_ms が無い（Spec-params が唯一の置き場）')


@pytest.mark.parametrize('fault_type', _HELD_CRITICAL)
def test_critical_faults_go_through_a_hold_timer(fault_type):
    """重大フォルトは HoldTimer 経由で報告されること。

    これが外れると監視ループ 1 周期の遅れで ESTOP に落ち、しかも
    フォルト起因の ESTOP は元のモードへ戻れない（C-09c のガード）。
    """
    arg = _update_fault_arg(_src(), fault_type)
    assert '_hold_.update(' in arg, (
        f'{fault_type} が HoldTimer を通らずに報告されている（WS-9O）: {arg.strip()!r}')


@pytest.mark.parametrize('fault_type', _NOT_HELD)
def test_recoverable_faults_are_not_delayed(fault_type):
    """回復可能フォルトには保持時間を課さない。

    一時停止に落ちても ui.resume_yes で正常に再開できるので、検知を遅らせる
    理由が無い。これらは /safety/fault_lock でモータをゼロにする側なので、
    遅らせると実害（停止が遅れる）が出る。
    """
    src = _src()
    assert f'checkTimeout("{fault_type}"' in src, (
        f'{fault_type} が checkTimeout を通っていない')
    assert f'updateFaultState("{fault_type}"' not in src, (
        f'{fault_type} が updateFaultState を直接呼んでいる。'
        f'保持時間つきに変わっていないか確認すること（WS-9O は重大フォルトのみ対象）')


def test_timeout_thresholds_are_unchanged():
    """しきい値そのものは緩めていないこと。

    WS-9O は「単発の誤検知を消す」対処で、「検知を鈍くする」対処ではない。
    しきい値を上げると本物の途絶の検知まで遅れるので、そちらへ流れていないか固定する。
    """
    src = _src()
    for name, expected in (('limiter_dead_ms', 250),
                            ('mux_dead_ms', 500),
                            ('state_stale_ms', 1500)):
        m = re.search(rf'declare_parameter\("{name}",\s*(\d+)\)', src)
        assert m, f'{name} の宣言が見つからない'
        assert int(m.group(1)) == expected, (
            f'{name} の既定値が {m.group(1)} に変わっている（期待 {expected}）。'
            'WS-9O はしきい値ではなく保持時間で誤検知を消す方針')


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
