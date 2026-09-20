"""test_localization_health_core.py — localization_health_core.evaluate() の純粋単体試験（WP-SAFE-05）。

ROS2 非依存（ノードを起動しない・最速）。Spec-safety.md §3.5.0 の A と C、
起動猶予、B を出さないことを検証する。
"""
import pytest

from th_state.localization_health_core import (
    REASON_JUMP,
    REASON_NODE_DOWN,
    REASON_OK,
    REASON_STALE,
    HealthReport,
    Params,
    TransformSample,
    detect_jump,
    evaluate,
)


def _params(**overrides):
    base = dict(
        stale_ms=2000,
        warmup_ms=30000,
        expected_nodes=("slam_toolbox", "amcl"),
        jump_window_ms=500,
        jump_translation_m=0.11,
        jump_rotation_rad=0.0175,
    )
    base.update(overrides)
    return Params(**base)


def _fresh_kwargs(**overrides):
    """健全な状態：ノード居る・変換は新しい・猶予過ぎ。"""
    kw = dict(
        now_ms=100_000,
        boot_ms=0,
        last_transform_ms=99_900,
        present_nodes=("slam_toolbox", "state_manager"),
    )
    kw.update(overrides)
    return kw


def test_fresh_transform_is_ok():
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs())
    assert r == HealthReport(ok=True, reason=REASON_OK,
                             node_present=True, transform_age_sec=0.1)


def test_stale_transform_is_ng_with_stale():
    """A: map→odom が stale_ms 更新されなければ ok=false/stale。"""
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs(last_transform_ms=90_000))
    assert r.ok is False
    assert r.reason == REASON_STALE
    assert r.node_present is True
    assert r.transform_age_sec == 10.0


def test_missing_node_is_ng_with_node_down():
    """C: 推定ノードが 1 つも居なければ ok=false/node_down。"""
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs(present_nodes=("state_manager",)))
    assert r.ok is False
    assert r.reason == REASON_NODE_DOWN
    assert r.node_present is False


def test_either_expected_node_counts_as_present():
    """amcl だけでも在席扱い（map_yaml 指定起動の休眠経路）。"""
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs(present_nodes=("amcl",)))
    assert r.ok is True
    assert r.node_present is True


def test_never_seen_transform_is_stale_after_warmup():
    """一度も見ていなければ猶予後は stale（inf は stale_ms を超える）。"""
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs(last_transform_ms=None))
    assert r.ok is False
    assert r.reason == REASON_STALE


def test_warmup_reports_ok():
    """起動猶予の間はノード不在・変換なしでも ok（起動失敗は猶予後に捕まえる）。"""
    p = _params()
    r = evaluate(now_ms=5_000, boot_ms=0, last_transform_ms=None,
                 present_nodes=(), p=p)
    assert r.ok is True
    assert r.reason == REASON_OK


def test_warmup_expiry_enables_detection():
    """猶予が過ぎれば厳密に判定する（境界：warmup_ms ちょうどは猶予後）。"""
    p = _params(warmup_ms=30_000)
    ok_side = evaluate(now_ms=29_999, boot_ms=0, last_transform_ms=None,
                       present_nodes=(), p=p)
    assert ok_side.ok is True
    ng_side = evaluate(now_ms=30_000, boot_ms=0, last_transform_ms=None,
                       present_nodes=(), p=p)
    assert ng_side.ok is False
    assert ng_side.reason == REASON_NODE_DOWN


def test_boundary_stale_ms_is_ok():
    """stale_ms ちょうどはまだ ok（超えたら ng）。"""
    p = _params(stale_ms=2000)
    assert evaluate(p=p, **_fresh_kwargs(
        now_ms=100_000, last_transform_ms=98_000)).ok is True
    assert evaluate(p=p, **_fresh_kwargs(
        now_ms=100_000, last_transform_ms=97_999)).reason == REASON_STALE


def test_low_confidence_is_never_emitted():
    """B（low_confidence）は出さない。reason は 3 値のどれか。"""
    p = _params()
    cases = [
        _fresh_kwargs(),
        _fresh_kwargs(last_transform_ms=None),
        _fresh_kwargs(present_nodes=()),
        _fresh_kwargs(last_transform_ms=0),
    ]
    for kw in cases:
        r = evaluate(p=p, **kw)
        assert r.reason in (REASON_OK, REASON_STALE, REASON_NODE_DOWN)
        assert r.reason != "low_confidence"


# ============================================================================
# B′: detect_jump（WP-SAFE-05B）
# ============================================================================

def _sample(t_ms=100_000, x=0.0, y=0.0, yaw=0.0):
    return TransformSample(t_ms=t_ms, x=x, y=y, yaw=yaw)


def test_jump_fires_on_translation():
    """完了条件1（並進）: 比較周期のあいだに 0.11 m 超動いたら jump。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000), _sample(t_ms=100_500, x=0.2), p)
    assert r.is_jump is True
    assert r.trans_m == pytest.approx(0.2)
    assert r.rot_rad == pytest.approx(0.0)


def test_jump_fires_on_rotation():
    """完了条件1（回転）: 1°（0.0175 rad）超回ったら jump。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000), _sample(t_ms=100_500, yaw=0.05), p)
    assert r.is_jump is True
    assert r.trans_m == pytest.approx(0.0)
    assert r.rot_rad == pytest.approx(0.05)


def test_jump_fires_on_diagonal():
    """斜めの合成移動（hypot）で判定すること。x だけでは足りなくても組で超える。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000), _sample(t_ms=100_500, x=0.08, y=0.08), p)
    assert r.trans_m == pytest.approx(0.08 * 2 ** 0.5)
    assert r.is_jump is True   # 0.113 > 0.11


def test_jump_boundary_does_not_fire():
    """完了条件2: 閾値ぴったりでは出ない（`>` であり `>=` ではない）。
    浮動小数点の 1ulp を避けるため、2 進で正確な値（1.0）で境界を踏む。
    hypot(1.0, 0.0) は正確に 1.0 になる。"""
    p = _params(jump_translation_m=1.0, jump_rotation_rad=99.0)
    assert detect_jump(
        _sample(t_ms=100_000), _sample(t_ms=100_500, x=1.0), p).is_jump is False
    assert detect_jump(
        _sample(t_ms=100_000), _sample(t_ms=100_500, x=1.000001), p).is_jump is True


def test_jump_rotation_threshold_is_effective():
    """回転の閾値が効いている（deg/rad の取り違え等でずれていたら赤）。
    wrap の浮動小数点誤差（1ulp）を避けるため、閾値の前後で見る。"""
    p = _params()
    eps = 1e-9
    assert detect_jump(
        _sample(t_ms=100_000),
        _sample(t_ms=100_500, yaw=0.0175 * (1 - eps)), p).is_jump is False
    assert detect_jump(
        _sample(t_ms=100_000),
        _sample(t_ms=100_500, yaw=0.0175 * (1 + eps)), p).is_jump is True


def test_jump_yaw_wraps_around_pi():
    """±πまたぎ（3.14 → -3.14）は微小回転であり jump ではない。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000, yaw=3.14),
                    _sample(t_ms=100_500, yaw=-3.14), p)
    assert r.rot_rad == pytest.approx(0.0, abs=0.01)
    assert r.is_jump is False


def test_slow_continuous_correction_does_not_fire():
    """完了条件3: ゆっくりした連続補正では出ない（誤発火の本体）。
    毎 tick 0.02 m・0.003 rad ずつ 30 回（合計 0.6 m・0.09 rad）動かしても
    per-window では閾値未満なので ok のまま。"""
    p = _params()
    prev = _sample(t_ms=100_000)
    for i in range(1, 31):
        curr = _sample(t_ms=100_000 + i * 500, x=0.02 * i, yaw=0.003 * i)
        r = detect_jump(prev, curr, p)
        assert r.is_jump is False, f"tick {i} で誤発火: {r}"
        prev = curr


def test_first_sample_does_not_fire():
    """完了条件4: 初回（prev 無し）は出ない。"""
    p = _params()
    r = detect_jump(None, _sample(), p)
    assert r.is_jump is False


def test_gap_beyond_stale_does_not_fire():
    """不連続（時刻差が stale 超）は出さない。凍結→復帰の飛びは A の担当。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000, x=0.0),
                    _sample(t_ms=103_000, x=5.0), p)
    assert r.is_jump is False


def test_backward_clock_does_not_fire():
    """時刻が戻っていたら出さない（時計の異常時は安全側）。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_500, x=0.0),
                    _sample(t_ms=100_000, x=5.0), p)
    assert r.is_jump is False
