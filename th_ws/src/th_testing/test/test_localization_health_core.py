"""test_localization_health_core.py — localization_health_core.evaluate() の純粋単体試験（WP-SAFE-05）。

ROS2 非依存（ノードを起動しない・最速）。Spec-safety.md §3.5.0 の A と C、
起動猶予、B を出さないことを検証する。
"""
from th_state.localization_health_core import (
    REASON_NODE_DOWN,
    REASON_OK,
    REASON_STALE,
    HealthReport,
    Params,
    evaluate,
)


def _params(**overrides):
    base = dict(
        stale_ms=2000,
        warmup_ms=30000,
        expected_nodes=("slam_toolbox", "amcl"),
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
