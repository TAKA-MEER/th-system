"""test_measure_link_core.py — th_measure/link_core の純コア試験。ROS 不要。

版の一致した heartbeat だけ数えること、途絶（timeout）と途絶理由
（never / timeout / version_mismatch）、`/measure/status` の速度上限の
組み立て（speed_limit 名→数値、stop→0。v_cap を混ぜるのは P5c）を縛る。
正本は docs/plan/detailed/DetailedDesign-measure.md §3.4・§6.1。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_measure'))

from th_measure.link_core import (INTERFACE_VERSION, LinkTracker, StatusFields,  # noqa: E402
                                  effective_v_limit, effective_w_limit,
                                  resolve_speed_limit)

TIMEOUT_MS = 1000


def test_only_matching_version_is_counted():
    t = LinkTracker(TIMEOUT_MS)
    assert t.on_heartbeat("2", 0) is False
    assert t.link_ok(0) is False
    assert t.on_heartbeat(INTERFACE_VERSION, 0) is True
    assert t.link_ok(0) is True


def test_timeout_after_last_counted():
    t = LinkTracker(TIMEOUT_MS)
    t.on_heartbeat(INTERFACE_VERSION, 1000)
    assert t.link_ok(1000 + TIMEOUT_MS) is True   # ちょうど境界は生きている
    assert t.link_ok(1000 + TIMEOUT_MS + 1) is False


def test_mismatched_heartbeats_do_not_extend_life():
    """版不一致の heartbeat は受信に数えない（§3.4）。途絶の時刻は版一致の
    最後から測る。"""
    t = LinkTracker(TIMEOUT_MS)
    t.on_heartbeat(INTERFACE_VERSION, 0)
    t.on_heartbeat("2", 900)
    t.on_heartbeat("2", 1900)
    assert t.link_ok(1900) is False
    assert t.reason(1900) == "timeout"


def test_reason_never_without_any_heartbeat():
    t = LinkTracker(TIMEOUT_MS)
    assert t.reason(0) == "never"
    assert t.link_ok(0) is False


def test_reason_version_mismatch_when_only_mismatched_seen():
    t = LinkTracker(TIMEOUT_MS)
    t.on_heartbeat("2", 0)
    t.on_heartbeat("99", 100)
    assert t.reason(200) == "version_mismatch"
    assert t.link_ok(200) is False


def test_reason_timeout_after_counted():
    t = LinkTracker(TIMEOUT_MS)
    t.on_heartbeat(INTERFACE_VERSION, 0)
    assert t.reason(500) == "ok"
    assert t.reason(TIMEOUT_MS + 1) == "timeout"


def test_reason_recovers_when_matching_returns():
    t = LinkTracker(TIMEOUT_MS)
    t.on_heartbeat(INTERFACE_VERSION, 0)
    assert t.link_ok(TIMEOUT_MS + 1) is False
    t.on_heartbeat(INTERFACE_VERSION, TIMEOUT_MS + 1)
    assert t.link_ok(TIMEOUT_MS + 1) is True
    assert t.reason(TIMEOUT_MS + 1) == "ok"


def test_resolve_speed_limit_stop_and_unknown_are_zero():
    table = {"v_max": 0.568, "v_slow": 0.30}
    assert resolve_speed_limit("stop", table) == 0.0
    assert resolve_speed_limit("v_typo", table) == 0.0
    assert resolve_speed_limit("v_max", table) == 0.568
    assert resolve_speed_limit("v_slow", table) == 0.30


def test_effective_v_limit_ignores_v_cap_until_p5c():
    """8-g P5b: v_cap は混ぜない（効かせるのは P5c）。指定があっても本システムの
    上限だけが v_limit_mps に出る。"""
    table = {"v_max": 0.568}
    assert effective_v_limit("v_max", table, 0.0) == 0.568
    assert effective_v_limit("v_max", table, 0.2) == 0.568      # P5b では無視
    assert effective_v_limit("v_max", table, 0.9) == 0.568
    assert effective_v_limit("stop", table, 0.2) == 0.0         # stop は 0


def test_effective_w_limit_p1_is_zero_without_cap():
    assert effective_w_limit(0.0) == 0.0
    assert effective_w_limit(0.5) == 0.5


def test_status_fields_assemble_measure_status():
    f = StatusFields(
        mode="INIT", state="CHECK", link_ok=True, pose_valid=True,
        map_instance_id="abc", x=1.0, y=2.0, yaw=0.5,
        pending_command_id="c1", pending_command="meas.goal",
        pause_reason="", speed_limit_name="v_max",
        speed_table={"v_max": 0.568}, v_cap_mps=0.0, w_cap_radps=0.0)
    d = f.as_dict()
    assert d["interface_version"] == "1"
    assert d["mode"] == "INIT" and d["state"] == "CHECK"
    assert d["link_ok"] is True and d["pose_valid"] is True
    assert d["map_instance_id"] == "abc"
    assert (d["x"], d["y"], d["yaw"]) == (1.0, 2.0, 0.5)
    assert d["pending_command_id"] == "c1" and d["pending_command"] == "meas.goal"
    assert d["v_limit_mps"] == 0.568 and d["w_limit_radps"] == 0.0
    assert d["blocked"] is False and d["reentry_required"] is False
