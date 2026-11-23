"""test_adjust_core.py — th_measure/adjust_core の純コア試験。ROS 不要。

8-g P3（DetailedDesign-measure.md §6.4）。旋回は巻き戻さずに積み上げる
（3.5 rad が逆向きの近道を回らない）、並進は距離で向きを保つ。
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_measure'))

from th_measure.adjust_core import (accumulate_yaw, heading_hold, translate_done,  # noqa: E402
                                    turn_done, turn_wzr, wrap_pi)


def test_wrap_and_accumulate_never_shortcut():
    # +3.5 rad の指示: ヨーが +π を超えて -π 側に回っても積み上げは +3.5 へ向かう。
    acc = 0.0
    prev = 0.0
    for yaw in [1.0, 2.0, 3.0, 3.14, -3.14, -2.5, -2.0]:
        acc = accumulate_yaw(yaw, prev, acc)
        prev = yaw
    # 最終ヨーは -2.0（≡ 4.28 rad）。積み上げは約 4.28 を超えて 3.5 を過ぎる。
    assert acc > 3.5
    assert turn_done(acc, 3.5, 0.9)
    # 負の指示は負に積む。
    acc2 = 0.0
    prev = 0.0
    for yaw in [-1.0, -2.0, -3.0, 3.14, 2.5]:
        acc2 = accumulate_yaw(yaw, prev, acc2)
        prev = yaw
    assert acc2 < -3.5


def test_wrap_boundaries():
    assert abs(wrap_pi(math.pi) - math.pi) < 1e-9
    assert abs(wrap_pi(-math.pi) - math.pi) < 1e-9 or abs(wrap_pi(-math.pi) + math.pi) < 1e-9
    # π をちょうど超えた増分は小さい負に丸まる（近道側）。
    assert abs(wrap_pi(3.2) - (3.2 - 2 * math.pi)) < 1e-9


def test_turn_sign_and_speed_cap():
    assert turn_wzr(0.0, 1.5, 1.2, 0.6, 0.4) > 0   # 左回りが正
    assert turn_wzr(0.0, -1.5, 1.2, 0.6, 0.4) < 0
    assert abs(turn_wzr(0.0, 0.1, 1.2, 0.6, 0.4)) >= 0.4  # 不感帯の上を保つ
    assert abs(turn_wzr(0.0, 10.0, 1.2, 0.6, 0.4)) <= 0.6


def test_done_tolerances():
    assert turn_done(1.5, 1.52, 0.09)
    assert not turn_done(0.0, 1.5, 0.09)
    assert not turn_done(-1.5, 1.5, 0.09)  # 逆向きは届いていない
    assert translate_done(0.30, 0.30, 0.02)
    assert translate_done(0.29, 0.30, 0.02)
    assert not translate_done(0.10, 0.30, 0.02)


def test_heading_hold():
    assert heading_hold(0.0, 0.0, 1.2, 0.6) == 0.0
    assert heading_hold(0.1, 0.0, 1.2, 0.6) < 0
    assert heading_hold(-0.1, 0.0, 1.2, 0.6) > 0
    assert abs(heading_hold(3.0, 0.0, 1.2, 0.6)) <= 0.6
