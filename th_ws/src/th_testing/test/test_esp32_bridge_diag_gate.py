"""
test_esp32_bridge_diag_gate.py
================================
WS-9J-A: esp32_bridge の 1Hz 診断サマリ (esp32_bridge_diag:) を既定で黙らせ、
悪化した窓だけ warn で出す。

背景 (実機ログ 2026-09-03・enable_route_slam:=true の起動):
  esp32_bridge.py の `_cb_diag_summary` が無条件 1Hz で INFO サマリを出し続け、
  起動 4 分で 268 行中 178 行 (66%) がこのサマリで埋まった。本当に見たい
  警告が埋もれる。D-3 の計器 (2026-08 の受信ギャップ切り分け) は一時的な
  もので、切れるようにしないまま常設化していた。

方針:
  - 判定 (「この窓は警告に値するか」) は ROS 非依存の純関数
    `bridge_diagnostics.diag_window_is_abnormal` に置く。ノードは呼ぶだけ。
  - `diag_log_period_s` (既定 0.0) を declare し、> 0 のときだけ毎窓 INFO。
  - 0 でも feedback_gap 悪化 / drop 増分がある窓は warn で 1 回出す。

ROS2 なし・純粋 Python (純関数テスト + esp32_bridge.py の ast 静的検証)。
"""

import ast
import os

from dataclasses import replace

from bridge_diagnostics import (
    INITIAL_DIAG_STATS,
    diag_window_is_abnormal,
)

# esp32_bridge.py のソースを ast で静的検証するためのパス
_BRIDGE_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_esp32_bridge', 'scripts'))
ESP32_BRIDGE = os.path.join(_BRIDGE_SCRIPTS, 'esp32_bridge.py')

WARN_GAP_MS = 500.0

# 健全な窓: gap 120ms・drop 増分なし
_HEALTHY = replace(
    INITIAL_DIAG_STATS,
    feedback_gap_max_ms=120.0,
    coalescer_dropped_total=0,
    rx_queue_dropped_total=0,
    invalid_frames_total=0,
)
_ZERO_PREV = {
    'coalescer_dropped_total': 0,
    'rx_queue_dropped_total': 0,
    'invalid_frames_total': 0,
}


# ── 純関数 diag_window_is_abnormal ──────────────────────────────
class TestDiagWindowIsAbnormal:

    def test_healthy_window_is_not_abnormal(self):
        assert diag_window_is_abnormal(_HEALTHY, _ZERO_PREV, WARN_GAP_MS) is False

    def test_gap_exactly_at_threshold_is_not_abnormal(self):
        stats = replace(_HEALTHY, feedback_gap_max_ms=WARN_GAP_MS)
        # 閾値ちょうどは黙る (境界。> であって >= ではない)
        assert diag_window_is_abnormal(stats, _ZERO_PREV, WARN_GAP_MS) is False

    def test_gap_above_threshold_is_abnormal(self):
        stats = replace(_HEALTHY, feedback_gap_max_ms=WARN_GAP_MS + 0.1)
        assert diag_window_is_abnormal(stats, _ZERO_PREV, WARN_GAP_MS) is True

    def test_coalescer_drop_increment_is_abnormal(self):
        stats = replace(_HEALTHY, coalescer_dropped_total=3)
        prev = dict(_ZERO_PREV, coalescer_dropped_total=2)
        assert diag_window_is_abnormal(stats, prev, WARN_GAP_MS) is True

    def test_rx_queue_drop_increment_is_abnormal(self):
        stats = replace(_HEALTHY, rx_queue_dropped_total=10)
        prev = dict(_ZERO_PREV, rx_queue_dropped_total=9)
        assert diag_window_is_abnormal(stats, prev, WARN_GAP_MS) is True

    def test_invalid_frame_increment_is_abnormal(self):
        stats = replace(_HEALTHY, invalid_frames_total=1)
        prev = dict(_ZERO_PREV, invalid_frames_total=0)
        assert diag_window_is_abnormal(stats, prev, WARN_GAP_MS) is True

    def test_nonzero_but_unchanged_cumulative_is_not_abnormal(self):
        # 累積カウンタが 0 でなくても、前の窓から増えていなければ黙る
        stats = replace(
            _HEALTHY,
            coalescer_dropped_total=7,
            rx_queue_dropped_total=42,
            invalid_frames_total=5,
        )
        prev = {
            'coalescer_dropped_total': 7,
            'rx_queue_dropped_total': 42,
            'invalid_frames_total': 5,
        }
        assert diag_window_is_abnormal(stats, prev, WARN_GAP_MS) is False

    def test_missing_prev_key_defaults_to_zero(self):
        stats = replace(_HEALTHY, invalid_frames_total=1)
        assert diag_window_is_abnormal(stats, {}, WARN_GAP_MS) is True


# ── esp32_bridge.py の ast 静的検証 ────────────────────────────
def _bridge_tree():
    with open(ESP32_BRIDGE, encoding='utf-8') as f:
        src = f.read()
    return ast.parse(src, filename=ESP32_BRIDGE), src


def _parent_map(tree):
    parents = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            parents[child] = node
    return parents


def _create_timer_calls(tree):
    return [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and n.func.attr == 'create_timer'
    ]


class TestEsp32BridgeWiring:

    def test_declares_diag_log_period_s_parameter(self):
        tree, _src = _bridge_tree()
        declared = [
            n for n in ast.walk(tree)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == 'declare_parameter'
            and n.args
            and isinstance(n.args[0], ast.Constant)
            and n.args[0].value == 'diag_log_period_s'
        ]
        assert declared, "esp32_bridge.py が 'diag_log_period_s' を declare_parameter していない"
        # 既定は 0.0 (出さない)
        default = declared[0].args[1] if len(declared[0].args) > 1 else None
        assert isinstance(default, ast.Constant) and default.value == 0.0, (
            "diag_log_period_s の既定値が 0.0 (出さない) でない")

    def test_diag_summary_timer_is_not_created_unconditionally(self):
        tree, src = _bridge_tree()
        parents = _parent_map(tree)
        diag_summary_timers = []
        for call in _create_timer_calls(tree):
            seg = ast.get_source_segment(src, call) or ''
            if '_cb_diag_summary' in seg:
                diag_summary_timers.append(call)
        assert diag_summary_timers, (
            "create_timer(..., self._cb_diag_summary) が見当たらない。"
            "リファクタで名前が変わったらこのテストを追随させること")
        for call in diag_summary_timers:
            node = call
            guarded = False
            while node in parents:
                node = parents[node]
                if isinstance(node, ast.If):
                    guarded = True
                    break
            assert guarded, (
                "create_timer(..., self._cb_diag_summary) が if の外で無条件に"
                "作られている。diag_log_period_s > 0 のときだけ作ること")

    def test_diag_summary_timer_first_arg_is_the_parameter_not_literal_one(self):
        """無条件 `create_timer(1.0, self._cb_diag_summary)` が残っていないこと。"""
        tree, src = _bridge_tree()
        for call in _create_timer_calls(tree):
            seg = ast.get_source_segment(src, call) or ''
            if '_cb_diag_summary' not in seg:
                continue
            first = call.args[0] if call.args else None
            assert not (isinstance(first, ast.Constant) and first.value == 1.0), (
                "create_timer の第1引数が 1.0 リテラルのまま。"
                "diag_log_period_s の値を渡すこと")
