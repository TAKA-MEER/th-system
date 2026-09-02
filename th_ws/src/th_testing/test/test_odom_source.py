"""
test_odom_source.py
===================
自己位置源選択 (odom_source.pick_odom_source) の単体テスト。

EKF 出力 (/odometry/filtered) の自己位置を優先し、古ければ生 /odom に
フォールバックする選択ロジックを検証する。ROS2 なし・純粋 Python で実行可能。

pytest で実行: python3 -m pytest test/test_odom_source.py -v
"""

import pytest

from th_planning.odom_source import pick_odom_source


def _fresh_filtered(now_ms=1000, stamp_ms=900):
    return ((1.0, 2.0, 0.5), stamp_ms)


def test_filtered_fresh_is_chosen():
    pose, source = pick_odom_source(
        _fresh_filtered(), ((9.0, 9.0, 0.0), 50), now_ms=1000, stale_ms=500)
    assert source == 'filtered'
    assert pose == (1.0, 2.0, 0.5)


def test_filtered_stale_raw_fresh_falls_back_to_raw():
    filtered = ((1.0, 2.0, 0.5), 100)      # 900ms 経過 → 古い
    raw = ((3.0, 4.0, 0.8), 950)           # 50ms 経過 → 新鮮
    pose, source = pick_odom_source(
        filtered, raw, now_ms=1000, stale_ms=500)
    assert source == 'raw'
    assert pose == (3.0, 4.0, 0.8)


def test_both_stale_returns_none_none():
    filtered = ((1.0, 2.0, 0.5), 100)      # 900ms 経過 → 古い
    raw = ((3.0, 4.0, 0.8), 200)           # 800ms 経過 → 古い
    pose, source = pick_odom_source(
        filtered, raw, now_ms=1000, stale_ms=500)
    assert pose is None
    assert source is None


def test_filtered_none_uses_raw():
    pose, source = pick_odom_source(
        None, ((3.0, 4.0, 0.8), 950), now_ms=1000, stale_ms=500)
    assert source == 'raw'
    assert pose == (3.0, 4.0, 0.8)


def test_exactly_stale_is_fresh_side():
    # 経過時間がちょうど stale_ms (=500) は新鮮側 (< =)
    filtered = _fresh_filtered(now_ms=1000, stamp_ms=500)
    raw = ((3.0, 4.0, 0.8), 950)
    pose, source = pick_odom_source(
        filtered, raw, now_ms=1000, stale_ms=500)
    assert source == 'filtered'
    assert pose == filtered[0]


def test_future_stamp_is_fresh():
    # stamp_ms が未来（負の経過時間）でも新鮮扱い
    filtered = _fresh_filtered(now_ms=1000, stamp_ms=1200)
    raw = ((3.0, 4.0, 0.8), 950)
    pose, source = pick_odom_source(
        filtered, raw, now_ms=1000, stale_ms=500)
    assert source == 'filtered'
    assert pose == filtered[0]


def test_just_over_stale_is_stale():
    # 境界の逆側: ちょうど stale_ms + 1 は古い
    filtered = _fresh_filtered(now_ms=1000, stamp_ms=498)   # 502ms 経過 → 古い
    raw = ((3.0, 4.0, 0.8), 950)                            # 新鮮
    pose, source = pick_odom_source(
        filtered, raw, now_ms=1000, stale_ms=500)
    assert source == 'raw'


def test_both_none_returns_none_none():
    pose, source = pick_odom_source(None, None, now_ms=1000, stale_ms=500)
    assert pose is None
    assert source is None
