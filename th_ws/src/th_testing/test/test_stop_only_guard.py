"""
test_stop_only_guard.py
=======================
1b-5: Spec.md SD-9 の純関数（stop_only_guard.py）の真理値表。ROS2 不要。

IDLE と PREP の機体が自分で走らない状態（MAPPING／REGISTER／EDIT／SAVED）で、
ジョグ中でないときだけ許可。RETURN・PAUSE・走行系・手動走行は拒否。
/system/state 未受信・古いときは拒否（安全側）。
"""

import ast
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_config_manager',
    'th_config_manager')))

from stop_only_guard import (  # noqa: E402
    ALLOWED_PREP_STATES,
    STATE_STALE_SEC,
    stop_only_allows,
)

_CONF_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_config_manager', 'scripts'))
_CONFIG_MANAGER = os.path.join(_CONF_SCRIPTS, 'config_manager.py')
_SLAM_CONTROL = os.path.join(_CONF_SCRIPTS, 'slam_control.py')
_PARAMS_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_params', 'scripts'))
_PARAMS_AUDIT = os.path.join(_PARAMS_SCRIPTS, 'params_audit.py')


def _allows(mode, state="NONE", jog=False, received=True, age=0.0):
    return stop_only_allows(mode, state, jog, received, age)


# ── 許可: IDLE ─────────────────────────────────────────────
def test_idle_allows():
    """IDLE は状態によらず許可（IDLE に状態は無く "NONE" が来る）。"""
    assert _allows("IDLE", "NONE") == (True, "")
    assert _allows("IDLE", "") == (True, "")


def test_stale_default_matches_registry():
    """鮮度の既定は registry の state_stale_ms=1500 と同じ 1.5 秒."""
    assert STATE_STALE_SEC == 1.5


# ── 許可: PREP の止まっている状態 ───────────────────────────
def test_prep_stopped_states_allow():
    """PREP の MAPPING／REGISTER／EDIT／SAVED は許可."""
    assert ALLOWED_PREP_STATES == {"MAPPING", "REGISTER", "EDIT", "SAVED"}
    for state in ("MAPPING", "REGISTER", "EDIT", "SAVED"):
        allowed, reason = _allows("PREP", state)
        assert allowed is True, f"PREP/{state} が拒否された: {reason}"


# ── 拒否: PREP の走る・止まっている途中 ─────────────────────
def test_prep_return_and_pause_deny():
    """PREP/RETURN（自律走行）・PREP/PAUSE は拒否."""
    for state in ("RETURN", "PAUSE"):
        allowed, reason = _allows("PREP", state)
        assert allowed is False, f"PREP/{state} が通った"
        assert reason == f"mode_state_not_stopped:PREP/{state}"


def test_prep_unknown_state_deny():
    """PREP の未知の状態（空を含む）は安全側で拒否."""
    for state in ("", "UNKNOWN"):
        assert _allows("PREP", state)[0] is False


# ── 拒否: 走行系・手動・停止系 ──────────────────────────────
def test_driving_modes_deny():
    """走行系の各モード・手動走行・ESTOP は拒否."""
    cases = [
        ("MANUAL", "NONE"),
        ("REPLAY", "RUN"),
        ("REPLAY", "LOCALIZE"),
        ("PANEL_NAV", "NAV"),
        ("SUMMON", "WAIT_CLEAR"),
        ("HOME_NAV", "NAV"),
        ("FOLLOW", "NONE"),
        ("TEACH_MANUAL", "REC"),
        ("ESTOP", "NONE"),
        ("CARRY", "NONE"),
        ("OPCHECK", "LIST"),
        ("CALIB", "LIST"),
        ("INIT", "CHECK"),
    ]
    for mode, state in cases:
        allowed, reason = _allows(mode, state)
        assert allowed is False, f"{mode}/{state} が通った"
        assert reason.startswith("mode_state_not_stopped:")


# ── 拒否: ジョグ中 ─────────────────────────────────────────
def test_jog_active_deny():
    """IDLE・PREP/MAPPING でもジョグ中は拒否."""
    assert _allows("IDLE", "NONE", jog=True)[0] is False
    assert _allows("IDLE", "NONE", jog=True)[1] == "jog_active"
    assert _allows("PREP", "MAPPING", jog=True)[0] is False


# ── 拒否: 未受信・古い ─────────────────────────────────────
def test_not_received_deny():
    """/system/state 未受信（起動直後を含む）は拒否."""
    assert _allows("IDLE", "NONE", received=False) == (False, "state_not_received")
    assert stop_only_allows(None, "NONE", False, False, 0.0) == (False, "state_not_received")


def test_stale_deny():
    """古い /system/state は拒否。ちょうど上限は新鮮側（含める）."""
    assert _allows("IDLE", "NONE", age=STATE_STALE_SEC + 0.01) == (False, "state_stale")
    assert _allows("IDLE", "NONE", age=STATE_STALE_SEC) == (True, "")
    assert _allows("PREP", "MAPPING", age=999.0) == (False, "state_stale")


# ── 配線の静的固定（launch_testing の代替ではない。補助） ────
def _src(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def test_nodes_use_stop_only_guard():
    """3 ノードが stop_only_guard を使い、旧 /robot/mode を見ない."""
    for path in (_CONFIG_MANAGER, _SLAM_CONTROL, _PARAMS_AUDIT):
        src = _src(path)
        tree = ast.parse(src)
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and (node.module or "").endswith("stop_only_guard"):
                imported.update(a.name for a in node.names)
        assert "stop_only_allows" in imported, f"{path} が stop_only_guard を import していない"
        assert "'/robot/mode'" not in src and '"/robot/mode"' not in src, \
            f"{path} が旧 /robot/mode をまだ購読している"
        assert "/system/state" in src, f"{path} に /system/state の購読が無い"
