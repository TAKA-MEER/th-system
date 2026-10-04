"""test_connectivity_core.py — connectivity_core.evaluate() の純粋単体試験（WP-STATE-03 §7）。

ROS2 非依存（ノードを起動しない・最速）。DetailedDesign-state.md §12.2 の4行と、
そのフェイルセーフ既定（6.2）・L-1 を検証する。
"""
from th_state.connectivity_core import LinkReport, Params, evaluate, link_status


def _params(**overrides):
    base = dict(
        esp32_alive_timeout_ms=3000,
        scan_expected_points=360,
        required_nodes=("state_manager", "safety_monitor"),
    )
    base.update(overrides)
    return Params(**base)


_ALL_OK_KWARGS = dict(
    now_ms=10_000,
    last_fb_ms=9_500,
    last_cmd_alive_ms=9_600,
    last_scan_ms=9_800,
    scan_points=360,
    present_nodes=("state_manager", "safety_monitor", "connectivity_checker"),
)


def test_four_items():
    """DetailedDesign-state.md §12.2 の4行を1本ずつ、他は満たした状態で崩して確認する。"""
    p = _params()

    # 全部満たせば all_ok（4項目とも合格）。
    ok = evaluate(p=p, **_ALL_OK_KWARGS)
    assert ok == LinkReport(True, True, True, True, ())
    assert ok.all_ok()

    # 行1（ESP32/esp32_feedback）: wheel_feedback の受信間隔が timeout を超えて古い。
    kw = dict(_ALL_OK_KWARGS)
    kw["last_fb_ms"] = _ALL_OK_KWARGS["now_ms"] - p.esp32_alive_timeout_ms - 1
    r = evaluate(p=p, **kw)
    assert r.esp32_feedback is False
    assert not r.all_ok()

    # 行2（ESP32/esp32_loopback）: 速度指令の折り返し（wheel_cmd_speed）が古い。
    kw = dict(_ALL_OK_KWARGS)
    kw["last_cmd_alive_ms"] = _ALL_OK_KWARGS["now_ms"] - p.esp32_alive_timeout_ms - 1
    r = evaluate(p=p, **kw)
    assert r.esp32_loopback is False
    assert not r.all_ok()

    # 行3（RaspberryPi4/lidar・点数）: 周期は満たしていても点数が規定と違えば不合格。
    kw = dict(_ALL_OK_KWARGS)
    kw["scan_points"] = _ALL_OK_KWARGS["scan_points"] - 1
    r = evaluate(p=p, **kw)
    assert r.lidar is False
    assert not r.all_ok()

    # 行3'（RaspberryPi4/lidar・周期）: 点数は合っていても受信間隔が古い。
    kw = dict(_ALL_OK_KWARGS)
    kw["last_scan_ms"] = _ALL_OK_KWARGS["now_ms"] - p.esp32_alive_timeout_ms - 1
    r = evaluate(p=p, **kw)
    assert r.lidar is False
    assert not r.all_ok()

    # 行4（PC/nodes）: 必須ノードが1つ欠けている。
    kw = dict(_ALL_OK_KWARGS)
    kw["present_nodes"] = ("state_manager",)
    r = evaluate(p=p, **kw)
    assert r.nodes is False
    assert r.missing_nodes == ("safety_monitor",)
    assert not r.all_ok()


def test_unreceived_is_fail():
    """6.2 フェイルセーフ既定: 一度も受信していない（None）項目は不合格として扱う。"""
    p = _params()
    r = evaluate(
        now_ms=10_000,
        last_fb_ms=None,
        last_cmd_alive_ms=None,
        last_scan_ms=None,
        scan_points=0,
        present_nodes=(),
        p=p,
    )
    assert r.esp32_feedback is False
    assert r.esp32_loopback is False
    assert r.lidar is False
    assert r.nodes is False
    assert not r.all_ok()


def test_ap_not_a_criterion():
    """L-1: Wi-Fi AP を判定項目に入れない。LinkReport のフィールド集合を固定して確認する。"""
    fields = set(LinkReport.__dataclass_fields__.keys())
    assert fields == {"esp32_feedback", "esp32_loopback", "lidar", "nodes", "missing_nodes"}
    for name in fields:
        assert "ap" not in name.lower() and "wifi" not in name.lower(), (
            f"AP らしきフィールドが判定項目に紛れ込んでいる: {name}")


# --- 以下は §7 表の5本には含まれない追加のカバレッジ（§8 のシミュレーション除外） ---

def test_sim_excludes_esp32_and_nodes():
    """§8: sim=True では Gazebo に居ない ESP32 の2項目と required_nodes を除外する。"""
    p = _params(sim=True)
    r = evaluate(
        now_ms=10_000,
        last_fb_ms=None,   # ESP32 は一度も来ない想定でも
        last_cmd_alive_ms=None,
        last_scan_ms=9_800,
        scan_points=360,
        present_nodes=(),  # required_nodes も一切揃っていない想定でも
        p=p,
    )
    assert r.esp32_feedback is True
    assert r.esp32_loopback is True
    assert r.nodes is True
    assert r.missing_nodes == ()
    assert r.all_ok()   # lidar（Gazebo でも /scan は出る）さえ揃えば通る


# ── link_status()（S-00 の機器別の行。Spec-webui.md §3.1。2026-10-04）──────────

def _status(p=None, **overrides):
    kw = dict(_ALL_OK_KWARGS)
    kw.update(overrides)
    return link_status(p=p or _params(), estop_seen=True, hw_estop=False, **kw)


def test_link_status_all_ok_with_ages():
    st = _status()
    it = st["items"]
    assert all(it[k]["ok"] for k in ("esp32_feedback", "esp32_loopback", "lidar", "nodes"))
    assert it["esp32_feedback"]["age_ms"] == 500
    assert it["esp32_loopback"]["age_ms"] == 400
    assert it["lidar"]["age_ms"] == 200
    assert it["lidar"]["points"] == 360 and it["lidar"]["expected_points"] == 360
    assert it["nodes"]["missing"] == []
    assert st["timeout_ms"] == 3000


def test_link_status_each_row_is_independent():
    """4 行が別々に判定されること（以前の画面は 4 行とも全体の合否を出していた）。"""
    it = _status(last_fb_ms=None)["items"]
    assert it["esp32_feedback"]["ok"] is False and it["esp32_feedback"]["age_ms"] is None
    assert it["esp32_loopback"]["ok"] and it["lidar"]["ok"] and it["nodes"]["ok"]

    it = _status(scan_points=359)["items"]
    assert it["lidar"]["ok"] is False and it["lidar"]["points"] == 359
    assert it["esp32_feedback"]["ok"] and it["nodes"]["ok"]

    it = _status(present_nodes=("state_manager",))["items"]
    assert it["nodes"]["ok"] is False and it["nodes"]["missing"] == ["safety_monitor"]
    assert it["lidar"]["ok"]


def test_link_status_dev_ignore_shows_real_state():
    """開発モードで link を外していても ok は本当の受信で決める（外していることは ignored）。"""
    p = _params(dev_ignore_link=True)
    it = _status(p=p, last_fb_ms=None, last_cmd_alive_ms=None, last_scan_ms=None,
                 present_nodes=())["items"]
    for k in ("esp32_feedback", "esp32_loopback", "lidar", "nodes"):
        assert it[k]["ok"] is False, k
        assert it[k]["ignored"] is True, k


def test_link_status_sim_marks_excluded():
    it = _status(p=_params(sim=True), last_fb_ms=None)["items"]
    assert it["esp32_feedback"]["excluded"] and it["esp32_loopback"]["excluded"]
    assert it["nodes"]["excluded"] and not it["lidar"]["excluded"]
    assert it["esp32_feedback"]["ok"] is False   # 除外しても実際の受信は偽のまま


def test_link_status_loopback_reports_firmware_report_age():
    """折り返し（2026-10-04）: ESP32 が報告していなければ reported_age_ms は None
    （古いファーム）。報告はあるが受信中でないときは age_ms だけ古い／None。"""
    it = _status(last_cmd_alive_ms=None)["items"]["esp32_loopback"]
    assert it["ok"] is False and it["reported_age_ms"] is None

    it = link_status(p=_params(), estop_seen=True, hw_estop=False,
                     last_cmd_report_ms=9_900, **dict(_ALL_OK_KWARGS, last_cmd_alive_ms=None))
    it = it["items"]["esp32_loopback"]
    assert it["ok"] is False and it["reported_age_ms"] == 100 and it["age_ms"] is None
