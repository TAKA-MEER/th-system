"""test_start_setup_scripts.py — start.sh / setup.sh の試験。

本物の実機・本物の th_robot コンテナに触れずに、本番の経路を縛る。
docker・ping・ssh・npx・npm・nmcli を偽物のコマンド（PATH の先頭に置いた
スクリプト）に差し替え、呼ばれた引数を記録させて検証する。ROS 不要。

対象: th_ws/start.sh（新設）、th_ws/setup.sh（改修）。
ブリーフ .briefs/brief-startsh.md §3 の必須項目 1〜8 に対応する。
"""
import os
import signal
import shutil
import stat
import subprocess
import time

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
TH_WS = os.path.join(REPO, "th_ws")
START_SH = os.path.join(TH_WS, "start.sh")
SETUP_SH = os.path.join(TH_WS, "setup.sh")
WEB_UI = os.path.join(TH_WS, "web_ui")


# ── 偽コマンド群 ────────────────────────────────────────────

FAKE_DOCKER = """#!/usr/bin/env bash
# 偽 docker。呼ばれた引数を $FAKE_LOG に残すだけ。
echo "docker $*" >> "$FAKE_LOG"
if [ "${1:-}" = "inspect" ]; then
    if [ "${FAKE_CONTAINER:-present}" = "missing" ]; then exit 1; fi
    case "$*" in
        *"-f"*)
            if [ "${FAKE_CONTAINER:-present}" = "stopped" ]; then echo "false"; else echo "true"; fi
            ;;
    esac
    exit 0
fi
if [ "${1:-}" = "exec" ]; then
    shift
    # $1 = コンテナ名
    shift
    if [ "${1:-}" = "ps" ]; then
        n=0
        if [ -f "$FAKE_PS_COUNT" ]; then n=$(cat "$FAKE_PS_COUNT"); fi
        n=$((n + 1)); echo "$n" > "$FAKE_PS_COUNT"
        if [ -n "${FAKE_PS_ALWAYS:-}" ]; then
            printf '%s\\n' "$FAKE_PS_ALWAYS"
        elif [ "$n" -ge 2 ] && [ -n "${FAKE_PS_LATER:-}" ]; then
            printf '%s\\n' "$FAKE_PS_LATER"
        else
            echo "PID COMMAND"
        fi
        exit 0
    fi
    case "$*" in
        *"kill -INT"*) exit 0 ;;
    esac
    case "$*" in
        *"ros2 launch"*)
            if [ "${FAKE_LAUNCH_MODE:-exit}" = "block" ]; then sleep 300; exit 0; fi
            exit "${FAKE_LAUNCH_RC:-1}"
            ;;
    esac
    exit 0
fi
exit 0
"""

FAKE_SIMPLE = """#!/usr/bin/env bash
# 偽コマンド雛形。$FAKE_NAME で呼ばれた引数を記録し、$FAKE_RC で終了する。
echo "$FAKE_NAME $*" >> "$FAKE_LOG"
if [ "${FAKE_BLOCK:-0}" = "1" ]; then sleep 300; fi
exit "${FAKE_RC:-0}"
"""

FAKE_NPX = """#!/usr/bin/env bash
echo "npx $*" >> "$FAKE_LOG"
echo "$$" > "$FAKE_NPX_PID"
if [ "${FAKE_NPX_MODE:-once}" = "block" ]; then sleep 300; fi
exit 0
"""


def _write(path, content):
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


@pytest.fixture()
def fakebin(tmp_path):
    """偽コマンド群を置いたディレクトリ。env と一緒に使う。

    戻り値: (fakebin のパス, env 更新用の dict を作る関数)
    """
    bin_dir = tmp_path / "fakebin"
    bin_dir.mkdir()
    _write(str(bin_dir / "docker"), FAKE_DOCKER)
    for name in ("ping", "ssh", "npm", "nmcli"):
        _write(str(bin_dir / name), FAKE_SIMPLE.replace("$FAKE_NAME", name))
    _write(str(bin_dir / "npx"), FAKE_NPX)

    log = tmp_path / "calls.log"
    log.write_text("", encoding="utf-8")
    (tmp_path / "pscount").write_text("0", encoding="utf-8")

    def make_env(**overrides):
        env = dict(os.environ)
        env["PATH"] = str(bin_dir) + os.pathsep + env.get("PATH", "")
        env["FAKE_LOG"] = str(log)
        env["FAKE_PS_COUNT"] = str(tmp_path / "pscount")
        env["FAKE_NPX_PID"] = str(tmp_path / "npx.pid")
        env["STARTSH_RESTART_WAIT"] = "0"
        env.update({k: str(v) for k, v in overrides.items()})
        return env

    return bin_dir, make_env, log


@pytest.fixture()
def with_dist():
    """web_ui/dist/ を一時的に作る（start.sh の存在確認を通すため）。"""
    dist = os.path.join(WEB_UI, "dist")
    created = not os.path.isdir(dist)
    if created:
        os.makedirs(dist)
        with open(os.path.join(dist, ".startsh-test-keep"), "w", encoding="utf-8") as f:
            f.write("test fixture\n")
    yield dist
    if created:
        shutil.rmtree(dist, ignore_errors=True)


def _calls(log):
    with open(str(log), encoding="utf-8") as f:
        return [line.rstrip("\n") for line in f if line.strip()]


def _launch_execs(calls):
    return [c for c in calls if "ros2 launch" in c]


# ── 1. 既定の引数・追加・上書き ─────────────────────────────
#
# bringup は自分から正常終了しないため、FAKE_LAUNCH_RC=0 でも上限まで
# 立て直して非0で止まる（差し戻し A）。引数の検証は 1 回目の呼び出しで行う。

def test_default_launch_args(fakebin, with_dist):
    _, make_env, log = fakebin
    env = make_env(FAKE_LAUNCH_MODE="exit", FAKE_LAUNCH_RC="0",
                   STARTSH_RESTART_MAX="1")
    r = subprocess.run(["bash", START_SH], capture_output=True, text=True,
                       env=env, timeout=60)
    assert r.returncode != 0  # 上限（1 回）に達して止まる
    execs = _launch_execs(_calls(log))
    assert len(execs) == 1
    for want in ("lidar_source:=network", "use_stub:=false",
                 "enable_route_slam:=true"):
        assert want in execs[0], execs[0]


def test_extra_launch_arg_appended(fakebin, with_dist):
    _, make_env, log = fakebin
    env = make_env(FAKE_LAUNCH_MODE="exit", FAKE_LAUNCH_RC="0",
                   STARTSH_RESTART_MAX="1")
    r = subprocess.run(["bash", START_SH, "stage:=4"], capture_output=True,
                       text=True, env=env, timeout=60)
    assert r.returncode != 0  # 上限（1 回）に達して止まる
    execs = _launch_execs(_calls(log))
    assert len(execs) == 1
    assert "stage:=4" in execs[0]
    assert "lidar_source:=network" in execs[0]


def test_launch_arg_override_wins(fakebin, with_dist):
    _, make_env, log = fakebin
    env = make_env(FAKE_LAUNCH_MODE="exit", FAKE_LAUNCH_RC="0",
                   STARTSH_RESTART_MAX="1")
    r = subprocess.run(["bash", START_SH, "enable_route_slam:=false"],
                       capture_output=True, text=True, env=env, timeout=60)
    assert r.returncode != 0  # 上限（1 回）に達して止まる
    execs = _launch_execs(_calls(log))
    assert len(execs) == 1
    assert "enable_route_slam:=false" in execs[0]
    assert "enable_route_slam:=true" not in execs[0]


# ── 2. ping が届かないと何も触らずに止まる ──────────────────

def test_ping_failure_touches_nothing(fakebin, with_dist):
    _, make_env, log = fakebin
    env = make_env(FAKE_RC="1")  # 雛形コマンドは全部失敗。ping が最初に落ちる
    env["FAKE_LAUNCH_MODE"] = "exit"
    # ping だけ失敗させたいので、ssh 等は成功させる仕組みが要る。
    # 雛形は FAKE_RC を共有するため、ここでは ping の失敗＝先頭の関門で止まる
    # ことを見る。docker が呼ばれていないことが要点。
    r = subprocess.run(["bash", START_SH], capture_output=True, text=True,
                       env=env, timeout=60)
    assert r.returncode != 0
    calls = _calls(log)
    assert any(c.startswith("ping ") for c in calls)
    assert not any(c.startswith("docker ") for c in calls), calls
    assert "network.md" in (r.stdout + r.stderr)


# ── 3. 二重起動は「起動もしないし何も止めない」 ─────────────

def test_double_start_stops_nothing(fakebin, with_dist):
    _, make_env, log = fakebin
    env = make_env(
        FAKE_PS_ALWAYS="PID COMMAND\n1234 ros2 launch th_bringup bringup.launch.py lidar_source:=network")
    r = subprocess.run(["bash", START_SH], capture_output=True, text=True,
                       env=env, timeout=60)
    assert r.returncode != 0
    calls = _calls(log)
    assert not _launch_execs(calls), calls  # 起動しない
    assert not any("kill" in c for c in calls), calls  # 止めない
    assert "既に動いている" in (r.stdout + r.stderr)


# ── 4. 立て直しは上限どおり・上限で案内して止まる ────────────

def test_restart_until_limit(fakebin, with_dist):
    _, make_env, log = fakebin
    env = make_env(FAKE_LAUNCH_MODE="exit", FAKE_LAUNCH_RC="1",
                   STARTSH_RESTART_MAX="3")
    r = subprocess.run(["bash", START_SH], capture_output=True, text=True,
                       env=env, timeout=120)
    assert r.returncode != 0
    execs = _launch_execs(_calls(log))
    assert len(execs) == 3, execs  # 上限どおり（上限+1 ではない）
    out = r.stdout + r.stderr
    assert "再起動しています" in out
    assert "電源再投入" in out


def test_restart_even_on_zero_exit(fakebin, with_dist):
    # 差し戻し A: ros2 launch は SIGTERM での後始末で 0 を返しうる。
    # 操作者の停止でない限り、0 でも立て直す。
    _, make_env, log = fakebin
    env = make_env(FAKE_LAUNCH_MODE="exit", FAKE_LAUNCH_RC="0",
                   STARTSH_RESTART_MAX="3")
    r = subprocess.run(["bash", START_SH], capture_output=True, text=True,
                       env=env, timeout=120)
    assert r.returncode != 0
    execs = _launch_execs(_calls(log))
    assert len(execs) == 3, execs
    out = r.stdout + r.stderr
    assert "再起動しています" in out
    assert "電源再投入" in out


def test_sigint_during_restart_wait_starts_only_once(fakebin, with_dist):
    # 差し戻し B: 立て直しの待ち時間中に Ctrl-C しても、もう一度起動しない。
    _, make_env, log = fakebin
    env = make_env(FAKE_LAUNCH_MODE="exit", FAKE_LAUNCH_RC="1",
                   STARTSH_RESTART_MAX="3", STARTSH_RESTART_WAIT="3")
    proc = subprocess.Popen(["bash", START_SH], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=env,
                            start_new_session=True)
    try:
        deadline = time.time() + 20
        while time.time() < deadline:
            if any("ros2 launch" in c for c in _calls(log)):
                break
            time.sleep(0.1)
        else:
            raise AssertionError("launch が起動しなかった")
        time.sleep(0.5)  # 立て直しの待ち時間に入っているはず
        os.killpg(proc.pid, signal.SIGINT)
        out = proc.communicate(timeout=20)[0]
        assert proc.returncode == 0, out
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    # 待ち時間を過ぎても（3 秒＋余裕）2 回目の起動が無いこと
    time.sleep(4)
    execs = _launch_execs(_calls(log))
    assert len(execs) == 1, execs
    assert "再起動しています" not in out


def test_dist_missing_guides_to_build(fakebin):
    _, make_env, log = fakebin
    env = make_env()
    # with_dist を使わない＝本物の dist が無ければ案内が出る。
    # 環境に dist が実在する場合はスキップ（本番ビルド済みの環境）。
    if os.path.isdir(os.path.join(WEB_UI, "dist")):
        pytest.skip("web_ui/dist が実在するため")
    r = subprocess.run(["bash", START_SH], capture_output=True, text=True,
                       env=env, timeout=60)
    assert r.returncode != 0
    out = r.stdout + r.stderr
    assert "--build" in out or "setup.sh" in out
    assert not _launch_execs(_calls(log))


# ── 5. SIGINT は launch に INT を送り、WebUI も止める ────────

def test_sigint_stops_launch_and_webui(fakebin, with_dist, tmp_path):
    _, make_env, log = fakebin
    env = make_env(FAKE_LAUNCH_MODE="block", FAKE_NPX_MODE="block",
                   FAKE_PS_LATER="PID COMMAND\n4242 ros2 launch th_bringup bringup.launch.py lidar_source:=network")
    proc = subprocess.Popen(["bash", START_SH], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=env,
                            start_new_session=True)
    try:
        # launch の起動（偽 docker の block）が始まるまで待つ
        deadline = time.time() + 20
        while time.time() < deadline:
            if any("ros2 launch" in c for c in _calls(log)):
                break
            time.sleep(0.2)
        else:
            proc.kill()
            raise AssertionError("launch が起動しなかった:\n" + "\n".join(_calls(log)))
        # 端末の Ctrl-C と同じくグループ全体に INT を送る
        os.killpg(proc.pid, signal.SIGINT)
        out = proc.communicate(timeout=20)[0]
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    calls = _calls(log)
    assert any("kill -INT 4242" in c for c in calls), calls  # PID 指定で INT
    assert "再起動しています" not in out  # 操作者の停止なので立て直さない
    # WebUI（偽 npx の block）が止まっている
    npx_pid_file = tmp_path / "npx.pid"
    assert npx_pid_file.exists()
    npx_pid = int(npx_pid_file.read_text(encoding="utf-8").strip())
    with pytest.raises(OSError):
        os.kill(npx_pid, 0)


def test_no_dangerous_kill_patterns():
    with open(START_SH, encoding="utf-8") as f:
        body = f.read()
    assert "kill -9" not in body
    assert "pkill" not in body
    assert "pgrep" not in body


# ── 6. --dry-run は何も実行しない ────────────────────────────

def test_dry_run_calls_nothing(fakebin):
    _, make_env, log = fakebin
    env = make_env()
    for args in (["--dry-run"], ["--dry-run", "--build", "stage:=4"]):
        if os.path.exists(str(log)):
            os.remove(str(log))
            open(str(log), "w").close()
        r = subprocess.run(["bash", START_SH] + args, capture_output=True,
                           text=True, env=env, timeout=60)
        assert r.returncode == 0, r.stderr
        assert _calls(log) == [], args


# ── 7. setup.sh --dry-run は udev を触らず本番ビルドを含む ───

def test_setup_dry_run(fakebin):
    _, make_env, log = fakebin
    env = make_env()
    r = subprocess.run(["bash", SETUP_SH, "--dry-run"], capture_output=True,
                       text=True, env=env, timeout=60)
    assert r.returncode == 0, r.stderr
    assert _calls(log) == []
    assert "npm run build" in r.stdout


def test_setup_does_not_touch_udev():
    with open(SETUP_SH, encoding="utf-8") as f:
        body = f.read()
    # udev ルールの導入コマンドが無いこと（説明のコメントに名前が出る分はよい）
    assert "udevadm" not in body
    assert "/etc/udev" not in body
    assert "sudo cp" not in body
    assert "npm run build" in body
    assert "npm ci" in body
    assert "colcon build" in body
    assert "./start.sh" in body


# ── 8. git 上で実行権限付き ──────────────────────────────────

def test_scripts_executable_in_git():
    r = subprocess.run(["git", "ls-files", "-s", "th_ws/start.sh", "th_ws/setup.sh"],
                       capture_output=True, text=True, cwd=REPO, timeout=30)
    assert r.returncode == 0
    lines = {line.rsplit(None, 1)[-1]: line.split()[0] for line in r.stdout.splitlines()}
    assert lines.get("th_ws/start.sh") == "100755", r.stdout
    assert lines.get("th_ws/setup.sh") == "100755", r.stdout


def test_scripts_have_shebang_and_syntax():
    for path in (START_SH, SETUP_SH):
        with open(path, encoding="utf-8") as f:
            assert f.readline().strip() == "#!/usr/bin/env bash"
        r = subprocess.run(["bash", "-n", path], capture_output=True, text=True,
                           timeout=30)
        assert r.returncode == 0, r.stderr
