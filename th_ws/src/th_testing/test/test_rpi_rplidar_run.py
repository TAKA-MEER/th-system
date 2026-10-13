"""rpi_rplidar_run.sh の試験（ROS 不要の素の pytest）。

実機 2026-10-13 の失敗を再現する: LiDAR の USB を抜き差しすると by-id のリンクが別の
ttyUSB を指すが、rplidar_node は古いデバイスを握ったまま終了もしない。見張りスクリプトが
リンクの変化で偽コマンドをプロセスグループごと止め、非ゼロで終了することを縛る。

偽デバイス = 一時ディレクトリのシンボリックリンク、偽コマンド = 孫プロセス（sleep）を
持つ bash。実機の `ros2 run`（bash -> ros2 -> rplidar_node）と同じ「孫まである」形。
"""
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "rpi_rplidar_run.sh"

# 偽コマンド: 自分の PID と孫(sleep)の PID をファイルに書いて待つ
FAKE_CMD = 'echo $$ > "$PIDDIR/parent"; sleep 300 & echo $! > "$PIDDIR/child"; wait'


def alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    try:  # ゾンビは死んだ扱い
        state = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
        return state != "Z"
    except OSError:
        return False


def wait_for(pred, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return pred()


@pytest.fixture
def env(tmp_path):
    assert SCRIPT.is_file(), SCRIPT
    (tmp_path / "ttyUSB0").write_text("")
    (tmp_path / "ttyUSB2").write_text("")
    link = tmp_path / "by-id-link"
    link.symlink_to(tmp_path / "ttyUSB0")
    piddir = tmp_path / "pids"
    piddir.mkdir()
    e = dict(os.environ, RPLIDAR_DEVICE=str(link), RPLIDAR_CMD=FAKE_CMD,
             PIDDIR=str(piddir), RPLIDAR_WAIT_MAX="3", RPLIDAR_KILL_GRACE="2")
    procs = []

    def start(**over):
        p = subprocess.Popen(["bash", str(SCRIPT)], env=dict(e, **over),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        procs.append(p)
        return p

    yield tmp_path, link, piddir, start
    for p in procs:
        if p.poll() is None:
            p.kill()
    for name in ("parent", "child"):
        f = piddir / name
        if f.exists():
            try:
                os.kill(int(f.read_text()), signal.SIGKILL)
            except (ProcessLookupError, ValueError):
                pass


def pids(piddir):
    assert wait_for(lambda: (piddir / "child").exists() and (piddir / "child").read_text().strip(), 5)
    return int((piddir / "parent").read_text()), int((piddir / "child").read_text())


def relink(link, target):
    tmp = link.with_name("tmp-link")
    tmp.symlink_to(target)
    os.replace(tmp, link)


def test_a_retarget_stops_command_and_exits_nonzero(env):
    tmp, link, piddir, start = env
    p = start()
    parent, child = pids(piddir)
    t0 = time.time()
    relink(link, tmp / "ttyUSB2")  # ttyUSB0 -> ttyUSB2
    rc = p.wait(timeout=5)
    assert time.time() - t0 < 3.5
    assert rc != 0
    assert wait_for(lambda: not alive(parent) and not alive(child), 2), "孫プロセスが残った"


def test_b_recreate_same_target_stops(env):
    tmp, link, piddir, start = env
    p = start()
    parent, child = pids(piddir)
    t0 = time.time()
    link.unlink()
    link.symlink_to(tmp / "ttyUSB0")  # 同じ番号に戻る再挿入（POLL より短い間に）
    rc = p.wait(timeout=5)
    assert time.time() - t0 < 3.5
    assert rc != 0
    assert wait_for(lambda: not alive(parent) and not alive(child), 2)


def test_b2_removed_link_stops(env):
    tmp, link, piddir, start = env
    p = start()
    pids(piddir)
    link.unlink()
    assert p.wait(timeout=5) != 0


def test_c_nothing_happens_keeps_running(env):
    tmp, link, piddir, start = env
    p = start()
    parent, child = pids(piddir)
    time.sleep(5)
    assert p.poll() is None
    assert alive(parent) and alive(child)
    p.terminate()  # systemctl stop 相当: 孫まで止めて終わる
    p.wait(timeout=8)
    assert wait_for(lambda: not alive(parent) and not alive(child), 2)


def test_c2_command_exit_code_is_propagated(env):
    tmp, link, piddir, start = env
    p = start(RPLIDAR_CMD="sleep 1; exit 7")
    assert p.wait(timeout=6) == 7


def test_d_waits_for_device_then_starts(env):
    tmp, link, piddir, start = env
    link.unlink()
    p = start(RPLIDAR_WAIT_MAX="10")
    time.sleep(1.5)
    assert not (piddir / "parent").exists()
    assert p.poll() is None
    link.symlink_to(tmp / "ttyUSB0")
    pids(piddir)
    assert p.poll() is None


def test_d2_gives_up_after_wait_limit(env):
    tmp, link, piddir, start = env
    link.unlink()
    p = start(RPLIDAR_WAIT_MAX="2")
    assert p.wait(timeout=6) != 0
    assert not (piddir / "parent").exists()


def test_e_term_ignoring_command_gets_killed(env):
    tmp, link, piddir, start = env
    p = start(RPLIDAR_CMD='trap "" TERM; echo $$ > "$PIDDIR/parent"; sleep 300 & echo $! > "$PIDDIR/child"; while :; do sleep 1; done',
              RPLIDAR_KILL_GRACE="1")
    parent, child = pids(piddir)
    relink(link, tmp / "ttyUSB2")
    assert p.wait(timeout=8) != 0
    assert wait_for(lambda: not alive(parent), 2)
