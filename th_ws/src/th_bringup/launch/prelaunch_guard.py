#!/usr/bin/env python3
"""prelaunch_guard.py — 新しい bringup/gazebo launch の起動前に、
前回起動した launch の生き残りプロセスを自動で片付ける。

背景（2026-09-05 実機）: `ros2 launch` の各子ノードは自身のプロセスグループ/
セッションに入るため、launch 親プロセスに SIGTERM を送っても子には伝播しない
（CLAUDE.md「pkill -TERM -f "ros2 launch ..." は launch 親しか殺さず、子ノードは
生き残る」）。前回の launch を止め忘れたまま次の launch を起動すると、両者の
子ノード同士がポート（esp32_bridge:8766 等）やノード名を取り合い、片方が起動
直後に落ちたり、slam_toolbox が資源を奪い合ってセグフォルトの無限再起動
ループに入ったりする（実機で発生・手動で後始末した）。

このモジュールは新しい launch の開始時に自動でこれを検出・後始末する
（G-1 と同じ位置づけ: ノードを1つも起動する前に、最初の OpaqueFunction として
同期実行する。params_generation.py 参照）。

`launch` / `launch_ros` はモジュール読み込み時にはインポートしない
（python3 -m pytest から直接テストできるようにする）。
"""
from __future__ import annotations

import os
import signal
import time
from typing import Callable, Sequence

# orphan（ppid=1。前回の後始末で子だけが生き残ったケース）掃討の目印。
# th_* パッケージの install パスと、このスタック専用の実行ファイル名に絞る。
# robot_state_publisher / joint_state_publisher / ekf_node / twist_mux /
# sllidar_node のような汎用 vendor バイナリは、名前だけでは無関係なプロセスと
# 区別できないため対象に含めない（誤検知のリスクの方が大きい）。
ORPHAN_MARKERS: tuple[str, ...] = (
    "/th_ws/install/th_perception/",
    "/th_ws/install/th_esp32_bridge/",
    "/th_ws/install/th_safety/",
    "/th_ws/install/th_mode_manager/",
    "/th_ws/install/th_state/",
    "/th_ws/install/th_planning/",
    "/th_ws/install/th_config_manager/",
    "map_and_localization_slam_toolbox_node",
    "async_slam_toolbox_node",
    "rosbridge_websocket",
)

LAUNCH_FILES: tuple[str, ...] = ("bringup.launch.py", "gazebo.launch.py")


def _read_cmdline(pid: int, proc_root: str) -> list[str]:
    try:
        with open(os.path.join(proc_root, str(pid), "cmdline"), "rb") as f:
            raw = f.read()
    except OSError:
        return []
    return [p for p in raw.decode("utf-8", "replace").split("\x00") if p]


def _read_ppid(pid: int, proc_root: str) -> int | None:
    try:
        with open(os.path.join(proc_root, str(pid), "status"), encoding="utf-8") as f:
            for line in f:
                if line.startswith("PPid:"):
                    return int(line.split()[1])
    except OSError:
        return None
    return None


def _list_pids(proc_root: str) -> list[int]:
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return []
    return [int(e) for e in entries if e.isdigit()]


def find_other_launch_roots(*, own_pid: int, proc_root: str = "/proc",
                             launch_files: Sequence[str] = LAUNCH_FILES) -> list[int]:
    """自分以外に生きている `ros2 launch th_bringup <launch_files>` を探す。"""
    roots = []
    for pid in _list_pids(proc_root):
        if pid == own_pid:
            continue
        cmdline = _read_cmdline(pid, proc_root)
        if not cmdline:
            continue
        if ("launch" in cmdline and "th_bringup" in cmdline
                and any(lf in cmdline for lf in launch_files)):
            roots.append(pid)
    return roots


def find_descendants(root_pids: Sequence[int], *, proc_root: str = "/proc") -> list[int]:
    """指定した親PID群の、現時点で生きている子孫PIDを全て集める（BFS）。"""
    children_of: dict[int, list[int]] = {}
    for pid in _list_pids(proc_root):
        ppid = _read_ppid(pid, proc_root)
        if ppid is not None:
            children_of.setdefault(ppid, []).append(pid)

    seen: set[int] = set()
    queue = list(root_pids)
    while queue:
        pid = queue.pop()
        for child in children_of.get(pid, []):
            if child not in seen:
                seen.add(child)
                queue.append(child)
    return sorted(seen)


def find_orphaned_stack_processes(*, proc_root: str = "/proc",
                                   markers: Sequence[str] = ORPHAN_MARKERS) -> list[int]:
    """親を失った(ppid=1)、このスタック固有のプロセスを探す
    （前回の後始末で子だけが生き残ったケースの保険）。"""
    found = []
    for pid in _list_pids(proc_root):
        if _read_ppid(pid, proc_root) != 1:
            continue
        cmdline = _read_cmdline(pid, proc_root)
        if not cmdline:
            continue
        joined = " ".join(cmdline)
        if any(marker in joined for marker in markers):
            found.append(pid)
    return found


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def terminate_pids(pids: Sequence[int], *, term_wait_s: float = 5.0,
                    poll_interval_s: float = 0.2,
                    kill_fn: Callable[[int, int], None] = os.kill,
                    alive_fn: Callable[[int], bool] = _pid_alive,
                    sleep_fn: Callable[[float], None] = time.sleep,
                    now_fn: Callable[[], float] = time.monotonic) -> dict[int, str]:
    """pids に SIGTERM を送り、term_wait_s 以内に消えなければ SIGKILL で止める。

    DDS discovery を壊さないよう kill -9 は最後の手段に留める
    （CLAUDE.md「kill -TERM で落とすこと」）。

    戻り値: {pid: "terminated" | "killed" | "already_gone"}。
    """
    result: dict[int, str] = {}
    pending = []
    for pid in pids:
        try:
            kill_fn(pid, signal.SIGTERM)
            pending.append(pid)
        except ProcessLookupError:
            result[pid] = "already_gone"

    deadline = now_fn() + term_wait_s
    while pending and now_fn() < deadline:
        sleep_fn(poll_interval_s)
        pending = [p for p in pending if alive_fn(p)]

    for pid in pending:
        try:
            kill_fn(pid, signal.SIGKILL)
            result[pid] = "killed"
        except ProcessLookupError:
            result[pid] = "already_gone"

    for pid in pids:
        result.setdefault(pid, "terminated")
    return result


def sweep_stale_launch(*, own_pid: int | None = None, proc_root: str = "/proc",
                        launch_files: Sequence[str] = LAUNCH_FILES,
                        markers: Sequence[str] = ORPHAN_MARKERS,
                        term_wait_s: float = 5.0,
                        poll_interval_s: float = 0.2,
                        kill_fn: Callable[[int, int], None] = os.kill,
                        alive_fn: Callable[[int], bool] = _pid_alive,
                        sleep_fn: Callable[[float], None] = time.sleep,
                        now_fn: Callable[[], float] = time.monotonic) -> list[str]:
    """新しい launch 開始時に呼ぶ本体。前回起動の生き残りを止め、人間可読な
    ログ行のリストを返す（呼び出し側が LogInfo で表示する）。何も無ければ空。
    """
    own_pid = own_pid if own_pid is not None else os.getpid()

    other_roots = find_other_launch_roots(own_pid=own_pid, proc_root=proc_root,
                                           launch_files=launch_files)
    descendants = find_descendants(other_roots, proc_root=proc_root)
    orphans = find_orphaned_stack_processes(proc_root=proc_root, markers=markers)

    target_pids = (set(other_roots) | set(descendants) | set(orphans)) - {own_pid}
    if not target_pids:
        return []

    # 子(descendants/orphans)を先に、launch親(other_roots)を最後に止める。
    ordered = sorted(target_pids - set(other_roots)) + sorted(other_roots)
    statuses = terminate_pids(ordered, term_wait_s=term_wait_s,
                               poll_interval_s=poll_interval_s,
                               kill_fn=kill_fn, alive_fn=alive_fn,
                               sleep_fn=sleep_fn, now_fn=now_fn)

    killed = sorted(pid for pid, s in statuses.items() if s == "killed")
    lines = [
        f"[prelaunch_guard] 前回起動の残存プロセス {len(ordered)} 件を停止しました "
        f"(launch親={sorted(other_roots)}, orphan={sorted(orphans)})."
    ]
    if killed:
        lines.append(
            f"[prelaunch_guard] SIGTERM に応答しなかったため SIGKILL で強制停止: {killed}")
    return lines


def make_guard_opaque_function(**sweep_kwargs) -> Callable:
    """`OpaqueFunction(function=...)` に渡すコールバック。bringup.launch.py /
    gazebo.launch.py の一番最初のアクションとして登録すること
    （params_generation.make_opaque_function と同じ位置づけ）。
    """
    def _opaque(context, *args, **kwargs):
        from launch.actions import LogInfo

        lines = sweep_stale_launch(**sweep_kwargs)
        return [LogInfo(msg=line) for line in lines]

    return _opaque
