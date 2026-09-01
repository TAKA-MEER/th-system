"""test_connectivity_core.py — connectivity_core.evaluate() の純粋単体試験（WP-STATE-03 §7）。

ROS2 非依存（ノードを起動しない・最速）。DetailedDesign-state.md §12.2 の4行と、
そのフェイルセーフ既定（6.2）・L-1 を検証する。
"""
from th_state.connectivity_core import LinkReport, Params, evaluate


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
    last_cmd_ms=9_600,
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
    kw["last_cmd_ms"] = _ALL_OK_KWARGS["now_ms"] - p.esp32_alive_timeout_ms - 1
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
        last_cmd_ms=None,
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
        last_cmd_ms=None,
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
