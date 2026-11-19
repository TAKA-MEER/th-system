"""test_single_controller_guard.py — th_bringup/launch/single_controller_guard.py の検証。

このファイルは ROS2 を起動しない（rclpy を import しない）。
純関数（find_other_controllers / check_single_controller / refusal_message）と、
bringup.launch.py への配線（最初の OpaqueFunction 群に登録されていること）を縛る。
実 rclpy でグラフを問う本体（run_startup_check）は
test_single_controller_guard_node.py（Docker・launch_testing）が縛る。
"""
from __future__ import annotations

import os
import sys

_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")

if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

import single_controller_guard as scg  # noqa: E402

BRINGUP_PY = os.path.join(_LAUNCH_DIR, "bringup.launch.py")


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


# ---------------------------------------------------------------------------
# find_other_controllers
# ---------------------------------------------------------------------------


def test_control_node_names_has_the_four_controller_nodes():
    assert set(scg.CONTROL_NODE_NAMES) == {
        "obstacle_limiter", "safety_monitor", "state_manager", "esp32_bridge"}


def test_find_other_controllers_empty_graph():
    assert scg.find_other_controllers([]) == []


def test_find_other_controllers_no_controller_nodes():
    assert scg.find_other_controllers(["twist_mux", "rosbag2", "state_manager2"]) == []


def test_find_other_controllers_detects_controller_node():
    assert scg.find_other_controllers(["twist_mux", "obstacle_limiter"]) == ["obstacle_limiter"]


def test_find_other_controllers_sorted_and_deduped():
    names = ["esp32_bridge", "safety_monitor", "esp32_bridge", "twist_mux"]
    assert scg.find_other_controllers(names) == ["esp32_bridge", "safety_monitor"]


# ---------------------------------------------------------------------------
# check_single_controller
# ---------------------------------------------------------------------------


def _clock(step=0.1):
    t = [0.0]

    def now():
        return t[0]

    def sleep(dt):
        t[0] += dt

    return now, sleep


def test_check_single_controller_passes_when_nobody_there():
    now, sleep = _clock()
    found, warning = scg.check_single_controller(
        watch_sec=0.3, poll_interval_sec=0.1,
        node_names_fn=lambda: ["twist_mux"], sleep_fn=sleep, now_fn=now)
    assert found == []
    assert warning is None


def test_check_single_controller_finds_other_pc_controller():
    now, sleep = _clock()
    calls = [0]

    def fn():
        calls[0] += 1
        return ["safety_monitor"] if calls[0] >= 2 else []

    found, warning = scg.check_single_controller(
        watch_sec=1.0, poll_interval_sec=0.1,
        node_names_fn=fn, sleep_fn=sleep, now_fn=now)
    assert found == ["safety_monitor"]
    assert warning is None


def test_check_single_controller_graph_failure_does_not_block():
    now, sleep = _clock()

    def fn():
        raise RuntimeError("graph down")

    found, warning = scg.check_single_controller(
        watch_sec=1.0, poll_interval_sec=0.1,
        node_names_fn=fn, sleep_fn=sleep, now_fn=now)
    assert found == []
    assert warning is not None and "警告" in warning


# ---------------------------------------------------------------------------
# refusal_message
# ---------------------------------------------------------------------------


def test_refusal_message_names_nodes_and_tells_to_stop():
    msg = scg.refusal_message(["obstacle_limiter", "safety_monitor"])
    assert "別の PC の制御系が動いています" in msg
    assert "obstacle_limiter" in msg and "safety_monitor" in msg
    assert "そちらを止めてから起動してください" in msg


# ---------------------------------------------------------------------------
# bringup.launch.py への配線
# ---------------------------------------------------------------------------


def test_bringup_registers_single_controller_opaque_function():
    src = _read(BRINGUP_PY)
    assert "from single_controller_guard import make_single_controller_opaque_function" in src
    assert "OpaqueFunction(function=make_single_controller_opaque_function())" in src


def test_bringup_registers_guard_before_any_node():
    """ガードはノードを 1 つも起動する前に（最初の OpaqueFunction 群に）登録する。"""
    src = _read(BRINGUP_PY)
    guard_pos = src.index("make_single_controller_opaque_function())")
    first_node_pos = src.index("nodes.append(Node(")
    prelaunch_pos = src.index("make_guard_opaque_function())")
    assert guard_pos < first_node_pos
    assert prelaunch_pos < guard_pos


# ---------------------------------------------------------------------------
# _opaque 本体（launch から呼ばれる関数の中身）
# ---------------------------------------------------------------------------


class _FakeLogInfo:
    def __init__(self, msg):
        self.msg = msg


def _install_fake_launch(monkeypatch):
    """host には launch が無いので、_opaque が中の遅延 import 先を差し替える。"""
    fake_launch = type(sys)("launch")
    fake_actions = type(sys)("launch.actions")
    fake_actions.LogInfo = _FakeLogInfo
    fake_launch.actions = fake_actions
    monkeypatch.setitem(sys.modules, "launch", fake_launch)
    monkeypatch.setitem(sys.modules, "launch.actions", fake_actions)


def test_opaque_propagates_startup_refusal(monkeypatch):
    """run_startup_check が RuntimeError を上げたら _opaque もそのまま上げ、
    launch が中止される（登録だけ縛って中身を空にしても通る変異を落とす）。"""
    _install_fake_launch(monkeypatch)

    def _raise(**kwargs):
        raise RuntimeError("別の PC の制御系が動いています（ノード名: obstacle_limiter）。"
                           "そちらを止めてから起動してください")

    monkeypatch.setattr(scg, "run_startup_check", _raise)
    opaque = scg.make_single_controller_opaque_function()
    import pytest
    with pytest.raises(RuntimeError, match="別の PC の制御系が動いています"):
        opaque(None)


def test_opaque_returns_warning_lines_as_loginfo(monkeypatch):
    """run_startup_check が警告の行を返したら LogInfo のリストになる（中身が空の
    変異でこの行も消えることを縛る）。"""
    _install_fake_launch(monkeypatch)
    monkeypatch.setattr(scg, "run_startup_check", lambda **kwargs: ["警告: グラフ未確認"])
    opaque = scg.make_single_controller_opaque_function()
    actions = opaque(None)
    assert len(actions) == 1
    assert isinstance(actions[0], _FakeLogInfo)
    assert actions[0].msg == "警告: グラフ未確認"


def test_opaque_returns_empty_when_nobody_there(monkeypatch):
    _install_fake_launch(monkeypatch)
    monkeypatch.setattr(scg, "run_startup_check", lambda **kwargs: [])
    opaque = scg.make_single_controller_opaque_function()
    assert opaque(None) == []
