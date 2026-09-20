"""test_dev_log_core.py — WP-DEV-01C（ログの選択記録）の試験。

ROS2 不要・ホストの素の pytest で走る（`dev_log_core` の純粋関数と、
`connectivity_checker.py` の配線に対する文字列検査だけ）。

対応する完了条件:
  1. 開発モード ON ＋ 対象を選んだときだけ、その対象が記録される
     → effective_selection／state_changed／fault_changed／should_record_cmdvel
  2. 開発モード OFF では何も記録されない
     → effective_selection が OFF で全偽（ノードは実効偽で出さない）
  3. 選択が ROS 側のパラメータで、`ros2 param set` から変えられる
     → ノードが `dev_log_state/fault/cmdvel` を宣言していること（配線検査）
  4. 純コアの単体試験がある（何を記録するかの判定・整形）
     → このファイル自体

変異チェック（実装をわざと壊して赤くなること）の記録はコミットメッセージにある。
"""
from __future__ import annotations

import json
import os
import re

from th_state import dev_log_core
from th_state.dev_log_core import (FaultSnap, StateSnap, TwistSum,
                                   effective_selection, fault_changed,
                                   format_line, format_timestamp,
                                   should_record_cmdvel, state_changed)


_SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CHECKER_PY = os.path.join(_SRC_ROOT, "th_state", "scripts", "connectivity_checker.py")


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


# ============================================================================
# 1. 選択の実効化（effective_selection）
# ============================================================================

def test_off_records_nothing():
    """完了条件2: OFF なら選択されていても実効は全偽（何も記録しない）。"""
    sel = {'state': True, 'fault': True, 'cmdvel': True}
    eff = effective_selection(False, sel)
    assert eff == {'state': False, 'fault': False, 'cmdvel': False}
    assert not any(eff.values())


def test_on_records_only_selected():
    """完了条件1: ON ＋ 選んだ対象だけが実効真になる。"""
    eff = effective_selection(True, {'state': True, 'fault': False, 'cmdvel': True})
    assert eff == {'state': True, 'fault': False, 'cmdvel': True}
    # 未指定の項目は選んでいない扱い（KeyError にしない）。
    eff = effective_selection(True, {})
    assert eff == {'state': False, 'fault': False, 'cmdvel': False}


def test_log_items_are_state_fault_cmdvel():
    """/scan・wheel 系は項目に無い（量の取捨選択の固定）。"""
    assert set(dev_log_core.LOG_ITEMS) == {'state', 'fault', 'cmdvel'}


# ============================================================================
# 2. 変化判定（state_changed / fault_changed）
# ============================================================================

def test_state_changed_on_mode_or_state():
    idle = StateSnap(mode='IDLE', state='NONE')
    assert state_changed(None, idle) is True  # 初見は出す
    assert state_changed(idle, StateSnap(mode='IDLE', state='NONE')) is False
    assert state_changed(idle, StateSnap(mode='MANUAL', state='NONE')) is True
    assert state_changed(idle, StateSnap(mode='IDLE', state='RUN')) is True


def test_fault_changed_on_active_type_severity():
    ok = FaultSnap(active=False, fault_type='NONE', severity='RECOVERABLE')
    assert fault_changed(None, ok) is True  # 初見は出す
    assert fault_changed(
        ok, FaultSnap(active=False, fault_type='NONE', severity='RECOVERABLE')) is False
    # 発生・解除・種別切替・深刻度切替はすべて拾う（一瞬のフォルトを逃がさない）。
    assert fault_changed(
        ok, FaultSnap(active=True, fault_type='LIDAR_LOST', severity='RECOVERABLE')) is True
    assert fault_changed(
        FaultSnap(active=True, fault_type='LIDAR_LOST', severity='RECOVERABLE'),
        FaultSnap(active=False, fault_type='NONE', severity='RECOVERABLE')) is True
    assert fault_changed(
        FaultSnap(active=True, fault_type='LIDAR_LOST', severity='RECOVERABLE'),
        FaultSnap(active=True, fault_type='ESP32_DISCONNECTED',
                  severity='RECOVERABLE')) is True
    assert fault_changed(
        FaultSnap(active=True, fault_type='LIDAR_LOST', severity='RECOVERABLE'),
        FaultSnap(active=True, fault_type='LIDAR_LOST', severity='CRITICAL')) is True


# ============================================================================
# 3. /cmd_vel 要約の間引き（should_record_cmdvel）
# ============================================================================

def test_cmdvel_first_sight_records():
    """初見は出す（選択した瞬間の現在値が 1 行出る）。"""
    assert should_record_cmdvel(10_000, None, None, TwistSum(0.0, 0.0)) is True


def test_cmdvel_interval_throttle():
    """値が変わっても最短間隔が空くまでは出さない（20 Hz → 1 Hz 上限）。"""
    prev = TwistSum(0.0, 0.0)
    curr = TwistSum(0.5, 0.0)
    assert should_record_cmdvel(10_000, 10_000, prev, curr) is False  # 直後
    assert should_record_cmdvel(
        10_000 + dev_log_core.CMDVEL_MIN_INTERVAL_MS - 1,
        10_000, prev, curr) is False  # 境界手前
    assert should_record_cmdvel(
        10_000 + dev_log_core.CMDVEL_MIN_INTERVAL_MS,
        10_000, prev, curr) is True  # 間隔が空いたら出す


def test_cmdvel_idle_heartbeat_only():
    """同じ値（停止中のゼロ連打）はハートビート間隔まで出さない。"""
    idle = TwistSum(0.0, 0.0)
    assert should_record_cmdvel(11_000, 10_000, idle, idle) is False
    assert should_record_cmdvel(
        10_000 + dev_log_core.CMDVEL_HEARTBEAT_MS - 1,
        10_000, idle, idle) is False
    assert should_record_cmdvel(
        10_000 + dev_log_core.CMDVEL_HEARTBEAT_MS,
        10_000, idle, idle) is True


def test_cmdvel_eps_tolerance():
    """微小な差は「同じ」とみなす（ノイズで毎秒出さない）。"""
    prev = TwistSum(0.0, 0.0)
    tiny = TwistSum(dev_log_core.CMDVEL_EPS_LINEAR / 2.0,
                    dev_log_core.CMDVEL_EPS_ANGULAR / 2.0)
    assert should_record_cmdvel(11_000, 10_000, prev, tiny) is False
    big = TwistSum(dev_log_core.CMDVEL_EPS_LINEAR * 2.0, 0.0)
    assert should_record_cmdvel(11_000, 10_000, prev, big) is True


def test_cmdvel_clock_goes_backwards_records_nothing():
    """時計が戻っていたら出さない（変な時刻の行を増やさない）。"""
    assert should_record_cmdvel(
        9_999, 10_000, TwistSum(0.0, 0.0), TwistSum(0.5, 0.0)) is False


# ============================================================================
# 4. 整形（format_line / format_timestamp）
# ============================================================================

def test_format_line_is_machine_readable_json_with_timestamp():
    """spec の要求: タイムスタンプ付き・あとから機械で読める形。"""
    line = format_line(0, 'state', {'mode': 'IDLE', 'state': 'NONE'})
    obj = json.loads(line)  # JSON として読めること
    assert obj['t'] == '1970-01-01T00:00:00.000Z'
    assert obj['kind'] == 'state'
    assert obj['mode'] == 'IDLE'
    assert obj['state'] == 'NONE'
    # 同じ入力は同じ行（sort_keys で安定化。差分・grep が壊れない）。
    assert format_line(0, 'state', {'mode': 'IDLE', 'state': 'NONE'}) == line


def test_format_timestamp_millis():
    assert format_timestamp(1_234) == '1970-01-01T00:00:01.234Z'
    assert re.fullmatch(r'\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z',
                        format_timestamp(1_718_000_000_123))


# ============================================================================
# 5. ノードの配線（connectivity_checker.py。本文書の試験だけでは「通るだけ」に
#    なるため、純粋関数と実装の接続点を縛る。test_dev_mode.py §4 と同じ狙い）
# ============================================================================

def test_checker_declares_dev_log_params():
    """完了条件3: 選択が ROS 側のパラメータ（`ros2 param set` で変えられる）。"""
    src = _read(CHECKER_PY)
    for param in ("dev_log_state", "dev_log_fault", "dev_log_cmdvel"):
        assert f"'{param}'" in src, f"パラメータ '{param}' の宣言が無い"


def test_checker_subscribes_to_logged_topics():
    """/system/state・/safety/fault・/cmd_vel を購読していること。"""
    src = _read(CHECKER_PY)
    assert "'/system/state'" in src, "/system/state の購読が無い"
    assert "'/safety/fault'" in src, "/safety/fault の購読が無い"
    assert "'/cmd_vel'" in src, "/cmd_vel の購読が無い"


def test_checker_timer_calls_selection_logger_with_pure_core():
    """1 Hz タイマが選択記録を呼び、判定・整形に純粋関数を使うこと。

    本番の呼び出し（`_on_timer` → `_maybe_log_selections`）を丸ごと殺しても
    赤くなる（過去の失敗「モック経路しか通っていない試験」の再発防止）。
    """
    src = _read(CHECKER_PY)
    assert "_maybe_log_selections" in src, "選択記録の呼び出しが無い"
    assert "_maybe_log_selections()" in src, "_on_timer から呼ばれていない"
    for fn in ("state_changed", "fault_changed", "should_record_cmdvel", "format_line"):
        assert fn in src, f"純粋関数 '{fn}' を使っていない（判定ロジックの重複実装か）"
    assert "dev_log: " in src, "ログ行の目印（'dev_log: '）が無い"


def test_checker_gate_logic_untouched():
    """やらないこと: 判定ロジックには触らない（パラメータ追加のみ）。"""
    src = _read(CHECKER_PY)
    assert "should_emit_link_ok" in src, "gate の純粋関数呼び出しが消えている"
    assert "report.all_ok() and self._estop_seen" not in src, "旧インライン gate 式の復活"
