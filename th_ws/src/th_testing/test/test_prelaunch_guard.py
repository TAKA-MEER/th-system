"""test_prelaunch_guard.py — th_bringup/launch/prelaunch_guard.py の検証。

このファイルは ROS2 を起動しない。`launch` / `launch_ros` は import しない
（prelaunch_guard.py が `make_guard_opaque_function()` の中でだけ遅延 import
する設計になっているため。params_generation.py と同じ理由）。

/proc の走査は実プロセスに触らず、tmp_path 配下に作った偽の /proc ツリーに対して
行う（signal 送信も fake kill_fn で差し替え、実プロセスへは一切シグナルを送らない）。
"""
from __future__ import annotations

import os
import sys

_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")

if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

import prelaunch_guard as pg  # noqa: E402

BRINGUP_PY = os.path.join(_LAUNCH_DIR, "bringup.launch.py")
GAZEBO_PY = os.path.join(_LAUNCH_DIR, "gazebo.launch.py")


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


# ---------------------------------------------------------------------------
# 偽の /proc ツリーを作るヘルパー
# ---------------------------------------------------------------------------


def _make_proc(proc_root, pid: int, *, ppid: int, cmdline: list[str]) -> None:
    d = os.path.join(proc_root, str(pid))
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "cmdline"), "wb") as f:
        f.write("\x00".join(cmdline).encode("utf-8") + b"\x00" if cmdline else b"")
    with open(os.path.join(d, "status"), "w", encoding="utf-8") as f:
        f.write(f"Name:\tfake\nPPid:\t{ppid}\n")


# ---------------------------------------------------------------------------
# find_other_launch_roots
# ---------------------------------------------------------------------------


def test_find_other_launch_roots_matches_bringup_launch(tmp_path):
    proc_root = str(tmp_path)
    _make_proc(proc_root, 100, ppid=1, cmdline=[
        "/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
        "th_bringup", "bringup.launch.py", "lidar_source:=network"])
    _make_proc(proc_root, 200, ppid=1, cmdline=["/usr/bin/some_unrelated_daemon"])

    roots = pg.find_other_launch_roots(own_pid=999, proc_root=proc_root)
    assert roots == [100]


def test_find_other_launch_roots_excludes_own_pid(tmp_path):
    proc_root = str(tmp_path)
    _make_proc(proc_root, 100, ppid=1, cmdline=[
        "/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
        "th_bringup", "gazebo.launch.py", "sim:=true"])

    roots = pg.find_other_launch_roots(own_pid=100, proc_root=proc_root)
    assert roots == []


def test_find_other_launch_roots_ignores_unrelated_ros2_launch(tmp_path):
    proc_root = str(tmp_path)
    # 別パッケージの launch は対象外（th_bringup を含まない）。
    _make_proc(proc_root, 100, ppid=1, cmdline=[
        "/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
        "some_other_pkg", "other.launch.py"])

    roots = pg.find_other_launch_roots(own_pid=999, proc_root=proc_root)
    assert roots == []


# ---------------------------------------------------------------------------
# find_descendants
# ---------------------------------------------------------------------------


def test_find_descendants_walks_full_tree(tmp_path):
    proc_root = str(tmp_path)
    _make_proc(proc_root, 100, ppid=1, cmdline=["root"])
    _make_proc(proc_root, 101, ppid=100, cmdline=["child-a"])
    _make_proc(proc_root, 102, ppid=100, cmdline=["child-b"])
    _make_proc(proc_root, 103, ppid=101, cmdline=["grandchild"])
    _make_proc(proc_root, 200, ppid=1, cmdline=["unrelated"])

    descendants = pg.find_descendants([100], proc_root=proc_root)
    assert descendants == [101, 102, 103]


# ---------------------------------------------------------------------------
# find_orphaned_stack_processes
# ---------------------------------------------------------------------------


def test_find_orphaned_stack_processes_requires_ppid_1_and_marker(tmp_path):
    proc_root = str(tmp_path)
    # orphan + マーカー一致 → 対象
    _make_proc(proc_root, 300, ppid=1, cmdline=[
        "python3", "/root/th_ws/install/th_safety/lib/th_safety/obstacle_limiter"])
    # orphan だがマーカー不一致 → 対象外
    _make_proc(proc_root, 301, ppid=1, cmdline=["/usr/bin/unrelated_orphan"])
    # マーカー一致だが親が生きている(orphanでない) → 対象外
    _make_proc(proc_root, 302, ppid=100, cmdline=[
        "python3", "/root/th_ws/install/th_safety/lib/th_safety/obstacle_limiter"])

    found = pg.find_orphaned_stack_processes(proc_root=proc_root)
    assert found == [300]


# ---------------------------------------------------------------------------
# terminate_pids
# ---------------------------------------------------------------------------


def test_terminate_pids_stops_at_sigterm_when_process_exits(tmp_path):
    sent = []
    alive_pids = {10, 11}

    def fake_kill(pid, sig):
        sent.append((pid, sig))
        if sig == 15:  # SIGTERM: このテストでは素直に終了する
            alive_pids.discard(pid)

    def fake_alive(pid):
        return pid in alive_pids

    result = pg.terminate_pids(
        [10, 11], term_wait_s=1.0, poll_interval_s=0.01,
        kill_fn=fake_kill, alive_fn=fake_alive,
        sleep_fn=lambda s: None, now_fn=_fake_clock())

    assert result == {10: "terminated", 11: "terminated"}
    assert (10, 15) in sent and (11, 15) in sent
    assert not any(sig == 9 for _, sig in sent)


def test_terminate_pids_escalates_to_sigkill_for_stragglers(tmp_path):
    sent = []
    alive_pids = {20}

    def fake_kill(pid, sig):
        sent.append((pid, sig))
        if sig == 9:
            alive_pids.discard(pid)
        # SIGTERM は無視する(このテストの狙い: ハングしたプロセス)

    def fake_alive(pid):
        return pid in alive_pids

    result = pg.terminate_pids(
        [20], term_wait_s=0.5, poll_interval_s=0.01,
        kill_fn=fake_kill, alive_fn=fake_alive,
        sleep_fn=lambda s: None, now_fn=_fake_clock())

    assert result == {20: "killed"}
    assert (20, 15) in sent
    assert (20, 9) in sent


def test_terminate_pids_marks_already_gone(tmp_path):
    def fake_kill(pid, sig):
        raise ProcessLookupError()

    result = pg.terminate_pids(
        [30], term_wait_s=0.5, poll_interval_s=0.01,
        kill_fn=fake_kill, alive_fn=lambda pid: False,
        sleep_fn=lambda s: None, now_fn=_fake_clock())

    assert result == {30: "already_gone"}


def _fake_clock():
    """time.monotonic() の代わり。呼ぶたびに時間が進む単調増加スタブ。"""
    state = {"t": 0.0}

    def _now():
        state["t"] += 0.05
        return state["t"]

    return _now


# ---------------------------------------------------------------------------
# sweep_stale_launch (結合)
# ---------------------------------------------------------------------------


def test_sweep_stale_launch_terminates_old_root_and_children(tmp_path):
    proc_root = str(tmp_path)
    _make_proc(proc_root, 100, ppid=1, cmdline=[
        "/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
        "th_bringup", "bringup.launch.py"])
    _make_proc(proc_root, 101, ppid=100, cmdline=[
        "python3", "/root/th_ws/install/th_esp32_bridge/lib/th_esp32_bridge/esp32_bridge.py"])
    # 自分自身の launch (own_pid) は対象外。
    _make_proc(proc_root, 999, ppid=1, cmdline=[
        "/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
        "th_bringup", "bringup.launch.py"])

    killed_pids = set()

    def fake_kill(pid, sig):
        killed_pids.add(pid)

    def fake_alive(pid):
        return False  # SIGTERM で即終了した体で進める

    lines = pg.sweep_stale_launch(
        own_pid=999, proc_root=proc_root, term_wait_s=0.2, poll_interval_s=0.01,
        kill_fn=fake_kill, alive_fn=fake_alive,
        sleep_fn=lambda s: None, now_fn=_fake_clock())

    assert killed_pids == {100, 101}
    assert 999 not in killed_pids
    assert lines and "前回起動の残存プロセス" in lines[0]


def test_sweep_stale_launch_noop_when_nothing_stale(tmp_path):
    proc_root = str(tmp_path)
    _make_proc(proc_root, 999, ppid=1, cmdline=[
        "/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
        "th_bringup", "bringup.launch.py"])

    calls = []
    lines = pg.sweep_stale_launch(
        own_pid=999, proc_root=proc_root,
        kill_fn=lambda pid, sig: calls.append((pid, sig)))

    assert lines == []
    assert calls == []


# ---------------------------------------------------------------------------
# launch ファイルへの配線（最初のアクションであること）
# ---------------------------------------------------------------------------


def test_bringup_registers_guard_before_params_generation():
    src = _read(BRINGUP_PY)
    assert "from prelaunch_guard import make_guard_opaque_function" in src
    guard_idx = src.index("make_guard_opaque_function()")
    gen_idx = src.index("params_generation_action = OpaqueFunction")
    assert guard_idx < gen_idx, (
        "bringup.launch.py: prelaunch_guard の登録が params_generation_action より後にある"
        "（G-1 と同様、後始末もノード起動より前に済ませる必要がある）")


def test_gazebo_registers_guard_before_params_generation():
    src = _read(GAZEBO_PY)
    assert "from prelaunch_guard import make_guard_opaque_function" in src
    return_start = src.index("return LaunchDescription(")
    return_src = src[return_start:]
    guard_idx = return_src.index("guard_action,")
    gen_idx = return_src.index("params_generation_action")
    assert guard_idx < gen_idx, (
        "gazebo.launch.py: guard_action が params_generation_action より後にある")
